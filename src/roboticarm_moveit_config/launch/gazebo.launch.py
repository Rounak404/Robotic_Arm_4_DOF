import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_moveit = get_package_share_directory('roboticarm_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('roboticarm')
        .robot_description(
            file_path='config/roboticarm_desc.urdf.xacro')
        .robot_description_semantic(
            file_path='config/roboticarm_desc.srdf')
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml')
        .robot_description_kinematics(
            file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    # 1. Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch', 'gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-r  empty.sdf',
        }.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # 3. ROS-Gazebo Clock Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # 4. Spawn robot in Gazebo
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'roboticarm',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',
                ],
                output='screen'
            )
        ]
    )

    # 5. Joint State Broadcaster
    joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    # 6. Arm Controller
    arm_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller'],
                output='screen',
            )
        ]
    )

    # 7. Gripper Controller
    gripper_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller'],
                output='screen',
            )
        ]
    )

    # 8. MoveIt move_group
    move_group = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[moveit_config.to_dict()],
            )
        ]
    )

    # 9. RViz
    # rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')
    # rviz = TimerAction(
    #     period=12.0,
    #     actions=[
    #         Node(
    #             package='rviz2',
    #             executable='rviz2',
    #             output='screen',
    #             arguments=['-d', rviz_config],
    #             parameters=[
    #                 moveit_config.robot_description,
    #                 moveit_config.robot_description_semantic,
    #                 moveit_config.robot_description_kinematics,
    #             ],
    #         )
    #     ]
    # )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        move_group,
        
    ])