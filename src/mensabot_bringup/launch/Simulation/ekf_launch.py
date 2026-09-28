import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Create the launch description for the simulated robot platform.

    The launch description starts the Gazebo world, spawns the robot model,
    initializes ros2_control, localization, sensor processing, safety functions
    and all supporting utility nodes required for operating the robot in
    simulation.

    Returns:
        LaunchDescription: Complete launch description for the simulated robot.
    """

    pkg_mensabot_bringup = get_package_share_directory('mensabot_bringup')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    diff_drive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mensabot_base_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        #remappings=[('/odometry/filtered', '/odom')],   # Remap the output of the EKF to /odom instead of /odometry/filtered like default
        parameters=[
            os.path.join(pkg_mensabot_bringup, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )    

    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',

        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom_rgbd',

            # Noch kein TF von RTAB-Map
            'publish_tf': False,

            # Wir geben RGB + Depth + CameraInfo separat hinein
            'subscribe_rgbd': False,

            # RGB und Depth müssen nicht exakt denselben Timestamp haben
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,

            'wait_for_transform': 0.2,
            'use_sim_time': True,
        }],

        remappings=[
            ('rgb/image', '/camera/rgbd/image'),
            ('depth/image', '/camera/rgbd/depth_image'),
            ('rgb/camera_info', '/camera/rgbd/camera_info'),

            ('odom', '/odometry/rgbd'),
        ],
    )

    launchDescriptionObject = LaunchDescription()
    
    
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(joint_state_broadcaster_node)
    launchDescriptionObject.add_action(diff_drive_controller_node)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(rgbd_odometry_node)
    return launchDescriptionObject