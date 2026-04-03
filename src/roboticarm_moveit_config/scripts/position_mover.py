import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import numpy as np
import sys
import time


class PositionMover(Node):
    def __init__(self):
        super().__init__('position_mover')

        # MoveIt services
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Action client — sends trajectory and waits for result
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Joint state subscriber
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        self.current_joint_state = msg

    # ------------------------------------------------------------------
    def find_joints(self, tx, ty, tz):
        """Call MoveIt IK service to get joint angles for target XYZ."""

        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "tcp_link"
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = tx
        req.ik_request.pose_stamped.pose.position.y = ty
        req.ik_request.pose_stamped.pose.position.z = tz
        req.ik_request.pose_stamped.pose.orientation.x = 0.0
        req.ik_request.pose_stamped.pose.orientation.y = 0.0
        req.ik_request.pose_stamped.pose.orientation.z = 0.0
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        req.ik_request.robot_state.joint_state = self.current_joint_state

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        result = future.result()
        if result is None:
            self.get_logger().error('❌ IK service timed out!')
            return None

        if result.error_code.val != 1:
            self.get_logger().error(f'❌ IK failed with error code: {result.error_code.val}')
            return None

        # Extract only the 4 arm joints by name
        arm_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        solution = result.solution.joint_state
        joints = []
        for name in arm_joint_names:
            if name in solution.name:
                idx = list(solution.name).index(name)
                joints.append(solution.position[idx])
            else:
                self.get_logger().error(f"❌ Joint '{name}' not found in IK solution!")
                return None

        self.get_logger().info(
            f'IK solution: {dict(zip(arm_joint_names, [round(j, 3) for j in joints]))}')
        return joints

    # ------------------------------------------------------------------
    def move_to_position(self, x, y, z):
        """Find IK solution and send to controller via action."""

        joints = self.find_joints(x, y, z)
        if joints is None:
            self.get_logger().error('❌ IK failed!')
            return

        joints = np.clip(joints,
            [-3.14, -1.57, -1.57, -1.57],
            [3.14,  1.57,  1.57,  1.57])

        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.get_logger().info(
            f'Joints: {dict(zip(joint_names, [round(float(j), 3) for j in joints]))}')

        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(j) for j in joints]
        point.velocities = [0.0] * 4
        point.time_from_start = Duration(sec=4, nanosec=0)
        trajectory.points.append(point)

        # Build action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Action server not available!')
            return

        # Send goal
        self.get_logger().info('Sending goal...')
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by controller!')
            return

        self.get_logger().info('✅ Goal accepted! Robot moving...')

        # Wait for motion to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Motion complete!')
        else:
            self.get_logger().error(f'❌ Motion failed with error code: {result.error_code}')


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PositionMover()

    raw_args = sys.argv[1:]
    if len(raw_args) != 3:
        print('\nUsage: python3 position_mover.py <x> <y> <z>')
        print('\nKnown reachable positions:')
        print('  python3 position_mover.py -0.014 -0.092  0.221  # home')
        print('  python3 position_mover.py -0.008  0.000  0.266  # arms up')
        print('  python3 position_mover.py -0.016 -0.120  0.144  # arms down')
        rclpy.shutdown()
        return

    # Wait for IK service
    node.get_logger().info('Waiting for IK service...')
    if not node.ik_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('❌ IK service not available!')
        rclpy.shutdown()
        return

    # Wait for joint states
    node.get_logger().info('Waiting for joint states...')
    timeout = time.time() + 5.0
    while node.current_joint_state is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_joint_state is None:
        node.get_logger().error('❌ No joint states received!')
        rclpy.shutdown()
        return

    node.get_logger().info('✅ Ready!')

    x, y, z = float(raw_args[0]), float(raw_args[1]), float(raw_args[2])
    node.move_to_position(x, y, z)
    rclpy.shutdown()


if __name__ == '__main__':
    main()