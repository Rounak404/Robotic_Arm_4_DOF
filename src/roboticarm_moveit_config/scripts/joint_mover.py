import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import time


class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')

        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

    def move_to_joint_values(self, joint_values):
        time.sleep(1.0)  # wait for connections

        # --- Arm: joint_1 to joint_4 ---
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

        arm_point = JointTrajectoryPoint()
        arm_point.positions = joint_values[:4]
        arm_point.time_from_start = Duration(sec=2, nanosec=0)
        arm_msg.points.append(arm_point)

        # --- Gripper: joint_5 ---
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = ['joint_5']

        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [joint_values[4]]
        gripper_point.time_from_start = Duration(sec=2, nanosec=0)
        gripper_msg.points.append(gripper_point)

        self.get_logger().info(
            f'Sending arm: {dict(zip(arm_msg.joint_names, joint_values[:4]))}')
        self.get_logger().info(
            f'Sending gripper: joint_5={joint_values[4]}')

        self.arm_publisher.publish(arm_msg)
        self.gripper_publisher.publish(gripper_msg)

        self.get_logger().info('✅ Commands sent!')


def main(args=None):
    rclpy.init(args=args)
    node = JointMover()

    raw_args = sys.argv[1:]

    if len(raw_args) != 5:
        print('\nUsage: python3 joint_mover.py <j1> <j2> <j3> <j4> <j5>')
        print('\nJoint limits:')
        print('  joint_1:  0.0   to  3.142')
        print('  joint_2: -1.57  to  1.57')
        print('  joint_3: -1.57  to  1.57')
        print('  joint_4: -1.57  to  1.57')
        print('  joint_5:  0.0   to  0.543  (gripper)')
        rclpy.shutdown()
        return

    joint_values = [float(v) for v in raw_args]
    node.move_to_joint_values(joint_values)
    rclpy.shutdown()


if __name__ == '__main__':
    main()