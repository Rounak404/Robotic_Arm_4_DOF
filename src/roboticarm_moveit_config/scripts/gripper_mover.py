import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import sys
import time
import math




class GripperMover(Node):
    def __init__(self):
        super().__init__('gripper_mover')

        self.GRIPPER_OPEN  = 0.54   # JOINT_MAX
        self.GRIPPER_CLOSE = 0.0    # JOINT_MIN
        self.FINGER_LENGTH = 0.05   # ← your actual finger length
        self.MAX_WIDTH = 2 * self.FINGER_LENGTH * math.sin(self.GRIPPER_OPEN)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        self.current_joint_state = msg

    def width_to_angle(self, width_meters: float) -> float:
        clearance = 0.005
        target_width = width_meters + clearance

        if target_width > self.MAX_WIDTH:
            self.get_logger().warn(
                f'⚠️  Box {width_meters*100:.1f}cm exceeds max '
                f'{self.MAX_WIDTH*100:.1f}cm — using max')
            return self.GRIPPER_OPEN

        angle = math.asin(target_width / (2 * self.FINGER_LENGTH))
        return max(self.GRIPPER_CLOSE, min(self.GRIPPER_OPEN, angle))

    def move_gripper(self, joint_position: float):
        joint_position = max(self.GRIPPER_CLOSE, min(self.GRIPPER_OPEN, joint_position))

    def open(self):
        self.get_logger().info('Opening gripper fully...')
        self.move_gripper(self.GRIPPER_OPEN)    

    def close(self):
        self.get_logger().info('Closing gripper fully...')
        self.move_gripper(self.GRIPPER_CLOSE)   

    def move_gripper(self, joint_position: float):
        """Send joint_5 to given position in radians."""
        joint_position = max(self.GRIPPER_CLOSE, min(self.GRIPPER_OPEN, joint_position))

        self.get_logger().info(
            f'Moving gripper → joint_5 = {joint_position:.4f} rad '
            f'({math.degrees(joint_position):.1f}°)')

        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_5']
        point = JointTrajectoryPoint()
        point.positions = [joint_position]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=2, nanosec=0)
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info('Waiting for gripper action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Gripper action server not available!')
            return

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Gripper goal rejected!')
            return

        self.get_logger().info('✅ Gripper moving...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Gripper motion complete!')
        else:
            self.get_logger().error(f'❌ Gripper failed: {result.error_code}')

    def open(self):
        """Fully open gripper."""
        self.get_logger().info('Opening gripper fully...')
        self.move_gripper(self.GRIPPER_OPEN)

    def close(self):
        """Fully close gripper."""
        self.get_logger().info('Closing gripper fully...')
        self.move_gripper(self.GRIPPER_CLOSE)

    def grip_box(self, width_meters: float):
        """Open gripper to fit a box of given width in meters."""
        self.get_logger().info(f'Gripping box of width {width_meters*100:.1f}cm...')
        angle = self.width_to_angle(width_meters)
        self.move_gripper(angle)


def main(args=None):
    rclpy.init(args=args)
    node = GripperMover()

    raw_args = sys.argv[1:]
    if len(raw_args) == 0:
        print('\nUsage:')
        print('  python3 gripper_mover.py open              # fully open')
        print('  python3 gripper_mover.py close             # fully close')
        print('  python3 gripper_mover.py angle 0.3         # direct angle (radians)')
        print('  python3 gripper_mover.py box 0.04          # grip 4cm wide box')
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

    command = raw_args[0].lower()

    if command == 'open':
        node.open()
    elif command == 'close':
        node.close()
    elif command == 'angle' and len(raw_args) == 2:
        node.move_gripper(float(raw_args[1]))
    elif command == 'box' and len(raw_args) == 2:
        node.grip_box(float(raw_args[1]))
    else:
        print(f'❌ Unknown command: {command}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()