import rclpy
import sys
import time
import os
import pickle

from position_mover import PositionMover
from gripper_mover import GripperMover
from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


SCENE_FILE = '/tmp/moveit_scene.pkl'


class PickPlace:
    def __init__(self, arm_node, gripper_node):
        self.arm     = arm_node
        self.gripper = gripper_node

        self.apply_scene_client = self.arm.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )
        self.get_scene_client = self.arm.create_client(
            GetPlanningScene,
            '/get_planning_scene'
        )

    # ── Scene helpers ──────────────────────────────────────────────────

    def _apply_scene(self, scene: PlanningScene):
        """Apply scene via service — persistent even after node shutdown."""
        self.apply_scene_client.wait_for_service(timeout_sec=5.0)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.apply_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm, future)
        return future.result()

    def box_exists(self, name: str) -> bool:
        """Check if object already exists in planning scene."""
        self.get_scene_client.wait_for_service(timeout_sec=5.0)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self.get_scene_client.call_async(req)
        rclpy.spin_until_future_complete(self.arm, future)
        result = future.result()
        existing = [obj.id for obj in result.scene.world.collision_objects]
        return name in existing

    def save_scene_state(self, name, x, y, z, sx, sy, sz):
        """Save box state to file so next run can restore it."""
        state = {
            'name': name,
            'x': x, 'y': y, 'z': z,
            'sx': sx, 'sy': sy, 'sz': sz
        }
        with open(SCENE_FILE, 'wb') as f:
            pickle.dump(state, f)
        self.arm.get_logger().info(f'💾 Scene saved to {SCENE_FILE}')

    def restore_scene_state(self):
        """Restore box from previous run if file exists."""
        if not os.path.exists(SCENE_FILE):
            self.arm.get_logger().info('No previous scene found — starting fresh')
            return
        with open(SCENE_FILE, 'rb') as f:
            state = pickle.load(f)
        self.arm.get_logger().info(
            f'♻️  Restoring "{state["name"]}" from previous run')
        self.add_box(
            state['name'],
            state['x'], state['y'], state['z'],
            state['sx'], state['sy'], state['sz']
        )

    def add_box(self, name, x, y, z, sx=0.04, sy=0.04, sz=0.04):
        """Add a box collision object — stays in scene permanently."""
        box = CollisionObject()
        box.header.frame_id = 'base_link'
        box.id = name

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [sx, sy, sz]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        box.primitives.append(shape)
        box.primitive_poses.append(pose)
        box.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(box)

        result = self._apply_scene(scene)
        if result and result.success:
            self.arm.get_logger().info(
                f'📦 Added "{name}" at ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'size ({sx*100:.1f} x {sy*100:.1f} x {sz*100:.1f} cm)')
        else:
            self.arm.get_logger().error(f'❌ Failed to add "{name}"')
        time.sleep(0.5)

    def remove_object(self, name):
        """Remove a collision object from scene."""
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = name
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        self._apply_scene(scene)
        self.arm.get_logger().info(f'🗑️  Removed "{name}" from scene')
        time.sleep(0.3)

    def attach_object(self, name):
        """Attach object to gripper — moves with arm."""
        attached = AttachedCollisionObject()
        attached.link_name = 'link_5'       # ← change to your gripper tip link
        attached.object.id = name
        attached.object.header.frame_id = 'base_link'
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = ['link_5', 'link_6']  # ← all gripper links

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)

        self._apply_scene(scene)
        self.arm.get_logger().info(f'🤏 Attached "{name}" to gripper')
        time.sleep(0.5)

    def detach_and_place(self, name, place_x, place_y, place_z,
                         sx, sy, sz):
        """Detach object and re-add it at place position."""
        # Detach from robot
        attached = AttachedCollisionObject()
        attached.link_name = 'link_5'       # ← same as attach_object
        attached.object.id = name
        attached.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        self._apply_scene(scene)
        self.arm.get_logger().info(f'📤 Detached "{name}"')
        time.sleep(0.3)

        # Re-add at place position permanently
        self.add_box(name, place_x, place_y, place_z, sx, sy, sz)

    # ── Pick & Place sequence ──────────────────────────────────────────

    def run(self, pick_x, pick_y, pick_z,
                  place_x, place_y, place_z,
                  box_size_x=0.04, box_size_y=0.04, box_size_z=0.04,
                  object_name='box1'):

        log = self.arm.get_logger()
        log.info('='*40)
        log.info('🚀 Starting Pick & Place')
        log.info(f'📦 Box: {box_size_x*100:.1f} x '
                 f'{box_size_y*100:.1f} x '
                 f'{box_size_z*100:.1f} cm')
        log.info(f'📍 Pick:  ({pick_x},  {pick_y},  {pick_z})')
        log.info(f'📍 Place: ({place_x}, {place_y}, {place_z})')
        log.info('='*40)

        grip_width = box_size_x

        # 1. Add box only if not already in scene
        log.info('📌 Step 1: Check/Add box to scene')
        if self.box_exists(object_name):
            log.info(f'📦 "{object_name}" already in scene — skipping add')
        else:
            self.add_box(object_name,
                         pick_x, pick_y, pick_z,
                         box_size_x, box_size_y, box_size_z)

        # 2. Open gripper wide enough for box
        log.info('📌 Step 2: Open gripper for box')
        self.gripper.grip_box(grip_width)

        # 3. Move above pick position
        log.info('📌 Step 3: Move above object')
        self.arm.move_to_position(pick_x, pick_y, pick_z + 0.08)
        time.sleep(0.5)

        # 4. Move down to pick
        log.info('📌 Step 4: Move down to object')
        self.arm.move_to_position(pick_x, pick_y, pick_z + 0.02)
        time.sleep(0.5)

        # 5. Close gripper to grasp
        log.info('📌 Step 5: Close gripper to grasp')
        self.gripper.close()
        time.sleep(0.3)

        # 6. Attach object to gripper
        log.info('📌 Step 6: Attach object to gripper')
        self.attach_object(object_name)

        # 7. Lift up
        log.info('📌 Step 7: Lift object')
        self.arm.move_to_position(pick_x, pick_y, pick_z + 0.15)
        time.sleep(0.5)

        # 8. Move above place position
        log.info('📌 Step 8: Move to place position')
        self.arm.move_to_position(place_x, place_y, place_z + 0.15)
        time.sleep(0.5)

        # 9. Lower to place
        log.info('📌 Step 9: Lower to place')
        self.arm.move_to_position(place_x, place_y, place_z + 0.02)
        time.sleep(0.5)

        # 10. Open gripper to release
        log.info('📌 Step 10: Open gripper to release')
        self.gripper.grip_box(grip_width)
        time.sleep(0.3)

        # 11. Detach and re-add box at place position
        log.info('📌 Step 11: Place object in scene')
        self.detach_and_place(object_name,
                              place_x, place_y, place_z,
                              box_size_x, box_size_y, box_size_z)

        # 12. Save scene state for next run
        self.save_scene_state(object_name,
                              place_x, place_y, place_z,
                              box_size_x, box_size_y, box_size_z)

        # 13. Retreat up
        log.info('📌 Step 12: Retreat')
        self.arm.move_to_position(place_x, place_y, place_z + 0.15)

        log.info('='*40)
        log.info('🎉 Pick & Place complete!')
        log.info('📦 Box remains at place location in scene')
        log.info('='*40)


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
        print('\nExamples:')
        print('  # default 4cm cube:')
        print('  python3 pick_place.py 0.15 0.0 0.05  -0.15 0.0 0.05')
        print('  # custom box 8cm x 4cm x 6cm:')
        print('  python3 pick_place.py 0.15 0.0 0.05  -0.15 0.0 0.05  0.08 0.04 0.06')
        rclpy.shutdown()
        return

    # Wait for IK service
    arm_node.get_logger().info('Waiting for IK service...')
    if not arm_node.ik_client.wait_for_service(timeout_sec=10.0):
        arm_node.get_logger().error('❌ IK service not available!')
        rclpy.shutdown()
        return

    # Wait for joint states
    arm_node.get_logger().info('Waiting for joint states...')
    timeout = time.time() + 5.0
    while arm_node.current_joint_state is None and time.time() < timeout:
        rclpy.spin_once(arm_node, timeout_sec=0.1)

    gripper_node.current_joint_state = arm_node.current_joint_state

    if arm_node.current_joint_state is None:
        arm_node.get_logger().error('❌ No joint states received!')
        rclpy.shutdown()
        return

    arm_node.get_logger().info('✅ Ready!')

    pick_x,  pick_y,  pick_z  = float(raw_args[0]), float(raw_args[1]), float(raw_args[2])
    place_x, place_y, place_z = float(raw_args[3]), float(raw_args[4]), float(raw_args[5])

    if len(raw_args) == 9:
        box_sx = float(raw_args[6])
        box_sy = float(raw_args[7])
        box_sz = float(raw_args[8])
    else:
        box_sx = box_sy = box_sz = 0.04

    pp = PickPlace(arm_node, gripper_node)

    # Restore previous scene state if exists
    pp.restore_scene_state()
    time.sleep(1.0)

    pp.run(pick_x, pick_y, pick_z,
           place_x, place_y, place_z,
           box_sx, box_sy, box_sz)

    arm_node.get_logger().info('✅ Done! Ctrl+C to exit.')
    try:
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()