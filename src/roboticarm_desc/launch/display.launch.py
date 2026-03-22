from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = "roboticarm_desc"
    pkg_path = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_path, "urdf", "robotic_arm.urdf")
    controllers_file = os.path.join(pkg_path, "config", "controllers.yaml")

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_file],
            remappings=[
                ("robot_description", "/robot_description"),
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller"],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        )
    ])