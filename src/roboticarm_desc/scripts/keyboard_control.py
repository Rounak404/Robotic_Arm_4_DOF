import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

    def move_arm(self,positions):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1','joint_2','joint_3','joint_4',]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2

        msg.points.append(point)
        self.arm_pub.publish(msg)

    def move_gripper(self,positions):
        msg = JointTrajectory()
        msg.joint_names = ['joint_5']

        point = JointTrajectoryPoint()
        point.positions = [positions] 
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.gripper_pub.publish(msg)

def main():
    rclpy.init()
    node = KeyboardControl()

    print("🚀 Keyboard Control Started")

    while True:
        print("\nOptions:")
        print("1 → Move Arm (4 joints)")
        print("2 → Move Gripper")
        print("q → Quit")

        choice = input("Enter choice: ").strip()

        # 🦾 ARM CONTROL
        if choice == '1':
            cmd = input("Enter 4 joint values (space-separated): ").strip()

            try:
                values = cmd.split()

                if len(values) != 4:
                    print("❌ Please enter exactly 4 values")
                    continue

                values = [float(v) for v in values]
                node.move_arm(values)
                print("✅ Arm command sent:", values)

            except Exception as e:
                print("❌ Invalid input:", e)

        # ✋ GRIPPER CONTROL
        elif choice == '2':
            cmd = input("Enter gripper value: ").strip()

            try:
                value = float(cmd)
                node.move_gripper(value)
                print("✅ Gripper command sent:", value)

            except Exception as e:
                print("❌ Invalid input:", e)

        # ❌ EXIT
        elif choice.lower() == 'q':
            print("👋 Exiting...")
            break

        else:
            print("❌ Invalid choice")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
