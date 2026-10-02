"""
Launch file for the Mensabot real robot.

This launch file starts all software components required for operating the
physical robot, including:

- robot_state_publisher
- ros2_control
- Hardware Interface
- EKF localization
- IMU driver and Madgwick filter
- Dual LiDAR drivers
- Laser scan merger
- RF2O laser odometry
- Safety Control Node
- LiDAR Field Selection Node

The launch file also supports an optional LiDAR reset before the remaining
system components are started.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode 
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """
    Create the launch description for the real Mensabot platform.

    The launch sequence initializes all required ROS 2 nodes in a controlled
    order. Event handlers are used to synchronize dependent components and
    prevent race conditions during system startup.

    Returns:
        LaunchDescription: Complete launch configuration for the real robot.
    """    

    pkg_mensabot_description = get_package_share_directory('mensabot_description')
    pkg_mensabot_bringup = get_package_share_directory('mensabot_bringup')
    pkg_mensabot_navigation = get_package_share_directory('mensabot_navigation')    
    pkg_mensabot_hardware = get_package_share_directory('mensabot_hardware')
    pkg_mensabot_utils = get_package_share_directory('mensabot_utils')


    model_arg = DeclareLaunchArgument(
        'model',
        default_value='mensabot.urdf.xacro',
    )

    lidar_reset_arg = DeclareLaunchArgument(
        'lidar_reset',
        default_value='false',
        description='Perform lidar reset before startup'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_mensabot_description,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    controller_manager_yaml_path = os.path.join(
        pkg_mensabot_bringup,
        'config',
        'controller.yaml'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path, ' use_sim:=false']), #use_sim:= false to not include gazebo specific plugins in the URDF when running on real robot
            },
        ],
    )

    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mensabot_base_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': Command(['xacro', ' ', urdf_file_path, ' use_sim:=false']),
            },
            controller_manager_yaml_path],
        output='screen',
    )   

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=[('/odometry/filtered', '/odom')],   # Remap the output of the EKF to /odom instead of /odometry/filtered like default
        parameters=[
            os.path.join(pkg_mensabot_bringup, 'config', 'ekf.yaml'),
        ]
    )

    cmd_vel_transform_node = Node(
        package='mensabot_utils',
        executable='cmd_vel_transform',
        name='cmd_vel_transform_node',
        output='screen'
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=0.5,
                    actions=[joint_state_broadcaster_node]
                )
            ]
        )
    )

    delayed_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[diff_drive_controller_node]
        )
    )

    normal_startup = GroupAction(
        actions=[
            controller_manager_node,
            robot_state_publisher_node,
            delayed_joint_state_broadcaster,
            delayed_diff_drive_controller,
            ekf_node,
            cmd_vel_transform_node,
        ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(lidar_reset_arg)

    # ---------------------------------------
    # Start without reset
    # ---------------------------------------

    launchDescriptionObject.add_action(
        GroupAction(
            condition=UnlessCondition(
                LaunchConfiguration('lidar_reset')
            ),
            actions=[
                normal_startup
            ]
        )
    )


    return launchDescriptionObject