import os

import launch
import launch.actions
import launch.conditions

import launch_ros
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    urdf_path = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'urdf',
        'golf_cart.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = launch_ros.actions.Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.5', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '3.141592653589793',
            '--frame-id', 'base_link', '--child-frame-id', 'os_lidar'
        ]
        )

    imu_tf = launch_ros.actions.Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
        )

    rviz_config_path = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'rviz',
        'localization.rviz')

    # Off by default on the vehicle: rviz2 renders the 5.8M-point map plus the
    # live cloud on the same box as the node, and /rosout from the test runs is
    # full of rviz2 "queue is full" drops. Pass use_rviz:=true from a desk
    # machine when you actually want to look at it.
    use_rviz = launch.substitutions.LaunchConfiguration('use_rviz', default='false')

    rviz_node = launch_ros.actions.Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=launch.conditions.IfCondition(use_rviz),
        output='screen')

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar_localization_ros2'),
            'param',  
            'localization.yaml'))

    lidar_localization = launch_ros.actions.LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir],
        remappings=[
            ('/cloud', '/ouster/points'),
            ('odom', '/odometry'),
        ],
        output='screen')

    # The /odometry that use_odom depends on. Started here rather than by
    # golfcart_low_level so it cannot go missing separately from its only
    # consumer; see the docstring in scripts/odom_bridge.py.
    odom_bridge = launch_ros.actions.Node(
        name='odom_bridge',
        package='lidar_localization_ros2',
        executable='odom_bridge.py',
        output='screen')

    # nav2's lifecycle_manager brings the node up (configure -> activate) with
    # retries/timeouts instead of the old EmitEvent/OnStateTransition dance,
    # which could silently stall waiting on the node's change_state service.
    lifecycle_manager = launch_ros.actions.Node(
        name='lifecycle_manager_lidar_localization',
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['lidar_localization'],
            'bond_timeout': 0.0,
        }])

    ld.add_action(launch.actions.DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start rviz2 alongside the localization node.'))
    ld.add_action(odom_bridge)
    ld.add_action(lidar_localization)
    ld.add_action(lifecycle_manager)
    ld.add_action(robot_state_publisher)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)
    ld.add_action(rviz_node)

    return ld
