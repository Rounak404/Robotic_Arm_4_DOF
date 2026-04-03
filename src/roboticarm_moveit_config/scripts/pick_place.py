import rclpy
import sys
import time
import os
import pickle
import math

from position_mover import PositionMover
from gripper_mover import GripperMover
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

SCENE_FILE = '/tmp/moveit_scene.pkl'

# ── Workspace constants (tune for your specific 4-DOF arm) ────────────────
MIN_SAFE_Z        = 0.08   # arm cannot reliably reach below this height
TRANSIT_Z         = 0.22   # cruise altitude between pick and place
MIN_REACH_XY      = 0.08   # minimum XY distance from base (inner dead zone)
MAX_REACH_XY      = 0.32   # maximum XY reach
SINGULARITY_ANGLE = math.pi / 2   # joint_1 = ~1.57 rad is the known bad zone
SINGULARITY_TOL   = 0.25          # ±14° around it is unsafe


class PickPlace:
    def __init__(self, arm_node, gripper_node):
        self.arm     = arm_node
        self.gripper = gripper_node

        self.apply_scene_client = self.arm.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene_client = self.arm.create_client(
            GetPlanningScene, '/get_planning_scene')

    # ── Spin helper ────────────────────────────────────────────────────────
    def _spin(self, duration=0.5):
        timeout = time.time() + duration
        while time.time() < timeout:
            rclpy.spin_once(self.arm, timeout_sec=0.05)

    # ── Workspace safety ───────────────────────────────────────────────────
    def _safe_z(self, z):
        return max(z, MIN_SAFE_Z)

    def _avoid_singularity(self, x, y):
        """
        For a 4-DOF arm, joint_1 = atan2(y, x).
        When that angle is near pi/2, IK returns -31.
        Nudge the XY angle just outside the danger zone and clamp radius.
        """
        angle = math.atan2(y, x)
        r_xy  = math.sqrt(x**2 + y**2)

        r_xy = max(r_xy, MIN_REACH_XY + 0.01)
        r_xy = min(r_xy, MAX_REACH_XY - 0.01)

        delta = angle - SINGULARITY_ANGLE
        if abs(delta) < SINGULARITY_TOL:
            sign  = 1 if delta >= 0 else -1
            angle = SINGULARITY_ANGLE + sign * (SINGULARITY_TOL + 0.05)
            self.arm.get_logger().warn(
                f'⚠️  Singularity nudge: joint_1 → {math.degrees(angle):.1f}°')

        return r_xy * math.cos(angle), r_xy * math.sin(angle)

    # ── Safe move wrapper ──────────────────────────────────────────────────
    def _move(self, x, y, z, label='', delay=0.5):
        z    = self._safe_z(z)
        x, y = self._avoid_singularity(x, y)
        log  = self.arm.get_logger()
        tag  = f' [{label}]' if label else ''
        log.info(f'➡️  Move{tag} → ({x:.3f}, {y:.3f}, {z:.3f})')
        self._spin(duration=0.3)
        self.arm.move_to_position(x, y, z)
        time.sleep(delay)
        self._spin(duration=0.2)

    # ── Scene helpers ──────────────────────────────────────────────────────
    def _apply_scene(self, scene: PlanningScene):
        self.apply_scene_client.wait_for_service(timeout_sec=5.0)
        req       = ApplyPlanningScene.Request()
        req.scene = scene
        future    = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm, future)
        return future.result()

    def box_exists(self, name: str) -> bool:
        self.get_scene_client.wait_for_service(timeout_sec=5.0)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm, future)
        return name in [o.id for o in future.result().scene.world.collision_objects]

    def is_object_attached(self, name: str) -> bool:
        """Check whether the object is still attached to the robot."""
        self.get_scene_client.wait_for_service(timeout_sec=5.0)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm, future)
        attached = future.result().scene.robot_state.attached_collision_objects
        return any(a.object.id == name for a in attached)

    def save_scene_state(self, name, x, y, z, sx, sy, sz):
        state = {'name': name, 'x': x, 'y': y, 'z': z,
                 'sx': sx, 'sy': sy, 'sz': sz}
        with open(SCENE_FILE, 'wb') as f:
            pickle.dump(state, f)
        self.arm.get_logger().info(f'💾 Scene saved to {SCENE_FILE}')

    def restore_scene_state(self):
        if not os.path.exists(SCENE_FILE):
            self.arm.get_logger().info('No previous scene — starting fresh')
            return
        with open(SCENE_FILE, 'rb') as f:
            state = pickle.load(f)
        self.arm.get_logger().info(f'♻️  Restoring "{state["name"]}"')
        self.add_box(state['name'],
                     state['x'], state['y'], state['z'],
                     state['sx'], state['sy'], state['sz'])

    def add_box(self, name, x, y, z, sx=0.04, sy=0.04, sz=0.04):
        box                 = CollisionObject()
        box.header.frame_id = 'base_link'
        box.id              = name
        shape               = SolidPrimitive()
        shape.type          = SolidPrimitive.BOX
        shape.dimensions    = [sx, sy, sz]
        pose                = Pose()
        pose.position.x     = x
        pose.position.y     = y
        pose.position.z     = z
        pose.orientation.w  = 1.0
        box.primitives.append(shape)
        box.primitive_poses.append(pose)
        box.operation       = CollisionObject.ADD
        scene               = PlanningScene()
        scene.is_diff       = True
        scene.world.collision_objects.append(box)
        result = self._apply_scene(scene)
        if result and result.success:
            self.arm.get_logger().info(
                f'📦 Added "{name}" at ({x:.3f},{y:.3f},{z:.3f}) '
                f'{sx*100:.1f}×{sy*100:.1f}×{sz*100:.1f} cm')
        else:
            self.arm.get_logger().error(f'❌ Failed to add "{name}"')
        time.sleep(0.5)

    def remove_object(self, name):
        obj                 = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id              = name
        obj.operation       = CollisionObject.REMOVE
        scene               = PlanningScene()
        scene.is_diff       = True
        scene.world.collision_objects.append(obj)
        self._apply_scene(scene)
        self.arm.get_logger().info(f'🗑️  Removed "{name}"')
        time.sleep(0.3)

    def attach_object(self, name):
        attached                        = AttachedCollisionObject()
        attached.link_name              = 'tcp_link'
        attached.object.id              = name
        attached.object.header.frame_id = 'base_link'
        attached.object.operation       = CollisionObject.ADD
        attached.touch_links            = ['tcp_link', 'link_4', 'link_5', 'link_6',name]
        scene                           = PlanningScene()
        scene.is_diff                   = True
        scene.robot_state.attached_collision_objects.append(attached)
        self._apply_scene(scene)
        self.arm.get_logger().info(f'🤏 Attached "{name}"')
        time.sleep(0.5)

    def get_box_pose(self, name):
        self.get_scene_client.wait_for_service(timeout_sec=5.0)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm, future)
        for obj in future.result().scene.world.collision_objects:
            if obj.id == name:
                return obj.primitive_poses[0]
        return None

    def detach_object(self, name):
        attached = AttachedCollisionObject()
        attached.link_name = 'tcp_link'  
        attached.object.id = name
        attached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(attached)

        self._apply_scene(scene)

        import time
        time.sleep(1)   

        self.arm.get_logger().info(f'📤 Detached "{name}" from gripper')

    def confirm_detach(self, name, timeout=3.0):
        """
        Spin until MoveIt confirms the object is no longer attached.
        This prevents the arm from retreating while MoveIt still thinks
        it's holding the object.
        """
        log      = self.arm.get_logger()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_object_attached(name):
                log.info(f'✅ "{name}" confirmed detached from scene')
                return True
            self._spin(duration=0.2)
        log.warn(f'⚠️  "{name}" detach not confirmed within {timeout}s — continuing')
        return False
    def go_home(self):
        self.arm.get_logger().info('🏠 Moving to Home position')
        self._move(-0.182, 0.020, 0.224, label='home')
    # ── Main sequence ──────────────────────────────────────────────────────
    def run(self, pick_x, pick_y, pick_z,
        place_x, place_y, place_z,
        box_size_x=0.04, box_size_y=0.04, box_size_z=0.04,
        object_name='box1'):

        log = self.arm.get_logger()
        place_z          = self._safe_z(place_z)
        place_x, place_y = self._avoid_singularity(place_x, place_y)
        grip_width       = box_size_x

        log.info('=' * 50)
        log.info('🚀  Pick & Place  [4-DOF mode]')
        log.info(f'    Pick : ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})')
        log.info(f'    Place: ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})')
        log.info('=' * 50)

        # 1. Add box to scene
        log.info('📌 Step 1 — Add box to scene')
        if self.box_exists(object_name):
            log.info('   Already exists, skipping')
        else:
            self.add_box(object_name, pick_x, pick_y, pick_z,
                        box_size_x, box_size_y, box_size_z)

        # 2. Open gripper
        log.info('📌 Step 2 — Open gripper')
        self.gripper.grip_box(grip_width)
        time.sleep(0.5)

        # 3. Hover above pick
        log.info('📌 Step 3 — Hover above pick')
        self._move(pick_x, pick_y, pick_z + 0.03, label='pick-hover')

        # 4. Descend to pick
        log.info('📌 Step 4 — Descend to pick')
        self._move(pick_x, pick_y, pick_z, label='pick-contact')

        # 5. Close gripper
        log.info('📌 Step 5 — Close gripper')
        self.gripper.close()
        time.sleep(0.5)

        # 6. Attach — box stays in world scene, touch_links handles collision
        log.info('📌 Step 6 — Attach to scene')
        self.attach_object(object_name)
        self._spin(duration=2.0)   # longer spin for seed to stabilize

        # 7. Lift to transit height
        log.info('📌 Step 7 — Lift')
        self._move(pick_x, pick_y, TRANSIT_Z, label='lift')

        # 8. Fly directly to place — NO midpoint transit
        log.info('📌 Step 8 — Approach place')
        self._move(place_x, place_y, TRANSIT_Z, label='place-high')

        # 9. Lower directly to place — NO intermediate step
        log.info('📌 Step 9 — Final descent')
        self._move(place_x, place_y, place_z, label='place-contact')

        
        # 10. Open gripper
        log.info('📌 Step 10 — Open gripper')
        self.gripper.open()
        time.sleep(0.8)

        # 11. Detach from scene
        log.info('📌 Step 11 — Detach from scene')
        self.detach_object(object_name)
        time.sleep(1.5)

        # 12. Retreat FIRST — before adding box ← KEY CHANGE
        # log.info('📌 Step 12 — Retreat up BEFORE adding box')
        # self._move(place_x, place_y, place_z + 0.08, label='retreat-up')

        

        # 14. Park above pick — arm fully away from place
        # log.info('📌 Step 14 — Park')
        # self._move(pick_x, pick_y, TRANSIT_Z, label='park')

        # 15. NOW add box — arm is safely away ← KEY CHANGE
        # log.info('📌 Step 15 — Register box at place')
        # self.remove_object(object_name)
        # time.sleep(0.2)
        # self.add_box(object_name,
        #              place_x, place_y, place_z - 0.02,
        #              box_size_x, box_size_y, box_size_z)
        # self._spin(duration=0.5)

        # 13. Move to transit height
        log.info('📌 Step 13 — Transit height')
        self._move(place_x, place_y, TRANSIT_Z, label='retreat-transit')

        # Move up first (safe)
        self._move(place_x, place_y, TRANSIT_Z, label='retreat')

        # Then go home
        self.go_home()
        # 16. Save scene
        self.save_scene_state(object_name,
                              place_x, place_y, place_z - 0.02,
                              box_size_x, box_size_y, box_size_z)

        log.info('=' * 50)
        log.info('🎉  Pick & Place complete!')
        log.info(f'📦  Box at ({place_x:.3f}, {place_y:.3f}, {place_z-0.02:.3f})')
        log.info('=' * 50)

def main(args=None):
    rclpy.init(args=args)
    arm_node     = PositionMover()
    gripper_node = GripperMover()

    raw_args = sys.argv[1:]
    if len(raw_args) not in [6, 9]:
        print('\nUsage:')
        print('  python3 pick_place.py'
              ' <pick_x> <pick_y> <pick_z>'
              ' <place_x> <place_y> <place_z>'
              ' [box_sx box_sy box_sz]')
        print('\nSafe example for 4-DOF arm:')
        print('  python3 pick_place.py'
              ' -0.014 -0.092 0.221'
              '  0.10  -0.05  0.10'
              '  0.04 0.04 0.04')
        rclpy.shutdown()
        return

    arm_node.get_logger().info('Waiting for IK service...')
    if not arm_node.ik_client.wait_for_service(timeout_sec=10.0):
        arm_node.get_logger().error('❌ IK service not available!')
        rclpy.shutdown()
        return

    arm_node.get_logger().info('Waiting for joint states...')
    deadline = time.time() + 5.0
    while arm_node.current_joint_state is None and time.time() < deadline:
        rclpy.spin_once(arm_node, timeout_sec=0.1)

    gripper_node.current_joint_state = arm_node.current_joint_state

    if arm_node.current_joint_state is None:
        arm_node.get_logger().error('❌ No joint states received!')
        rclpy.shutdown()
        return

    arm_node.get_logger().info('✅ Ready!')

    pick_x,  pick_y,  pick_z  = float(raw_args[0]), float(raw_args[1]), float(raw_args[2])
    place_x, place_y, place_z = float(raw_args[3]), float(raw_args[4]), float(raw_args[5])
    box_sx = float(raw_args[6]) if len(raw_args) == 9 else 0.04
    box_sy = float(raw_args[7]) if len(raw_args) == 9 else 0.04
    box_sz = float(raw_args[8]) if len(raw_args) == 9 else 0.04

    pp = PickPlace(arm_node, gripper_node)
    pp.run(pick_x, pick_y, pick_z,
           place_x, place_y, place_z,
           box_sx, box_sy, box_sz)

    arm_node.get_logger().info('✅ Done! Ctrl+C to exit.')
    try:
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()