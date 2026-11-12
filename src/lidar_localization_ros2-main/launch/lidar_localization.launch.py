import os

import launch
import launch.actions
import launch.conditions

import launch_ros
import launch_ros.actions
import yaml

from ament_index_python.packages import get_package_share_directory

# Buildings of the ModRide web map placed on the loaded map: map_frames.yaml
# says where each map lies on the earth, make_buildings.py writes
# buildings_<map>.json, buildings_node.py draws it on /campus_buildings.
# vehicle_bridge.py goes the other way: the cart's pose to lat/lon for the web map.
# auto_initial_pose.py sets the initial pose itself at start-up (no 2D Pose
# Estimate needed): last saved pose, else GPS, else the whole map, checked
# against /match_inlier_ratio. auto_initial_pose:=false turns it off.
BUILDINGS_DIR = '/home/myuser/2Localize_get_path/src/campus_buildings'

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

    # Attaches the Ouster's own tree (os_sensor -> os_lidar, published by the
    # driver) under base_link. This used to publish base_link -> os_lidar
    # directly, which gave os_lidar two parents: whichever static transform
    # arrived last won, so starting the driver AFTER this launch left the scan
    # frame cut off from base_link and every cloud failed to transform. The
    # numbers are the same transform: os_sensor -> os_lidar is z +0.038195 with
    # yaw pi, so base_link -> os_lidar still comes out at x 0.5, yaw pi.
    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.5', '--y', '0', '--z', '-0.038195',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'os_sensor'
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

    # map_path:=<file> picks the map for this run; without it the node uses
    # map_path from the parameter file. The RViz panel (cart_rviz_panel) passes
    # the map chosen there. A map is its own frame: the route and the initial
    # pose belong to one map.
    def lidar_localization(context):
        parameters = [localization_param_dir]
        map_path = launch.substitutions.LaunchConfiguration('map_path').perform(context).strip()
        if map_path:
            parameters.append({'map_path': os.path.expanduser(map_path)})
        return [launch_ros.actions.LifecycleNode(
            name='lidar_localization',
            namespace='',
            package='lidar_localization_ros2',
            executable='lidar_localization_node',
            parameters=parameters,
            remappings=[
                ('/cloud', '/ouster/points'),
                ('odom', '/odometry'),
            ],
            output='screen')]

    # The /odometry that use_odom depends on. Started here rather than by
    # golfcart_low_level so it cannot go missing separately from its only
    # consumer; see the docstring in scripts/odom_bridge.py.
    odom_bridge = launch_ros.actions.Node(
        name='odom_bridge',
        package='lidar_localization_ros2',
        executable='odom_bridge.py',
        output='screen')

    # map_path is the one the node loads: the argument, else the parameter file's.
    def campus_buildings(context):
        script = os.path.join(BUILDINGS_DIR, 'buildings_node.py')
        if not os.path.isfile(script):
            return []
        map_path = launch.substitutions.LaunchConfiguration('map_path').perform(context).strip()
        if not map_path:
            with open(localization_param_dir.perform(context), encoding='utf-8') as f:
                params = yaml.safe_load(f)
            map_path = next(iter(params.values()))['ros__parameters'].get('map_path', '')
        nodes = [launch_ros.actions.Node(
            name='campus_buildings',
            executable='/usr/bin/python3',
            arguments=[script],
            parameters=[{'map_path': os.path.expanduser(map_path)}],
            output='screen')]
        bridge = os.path.join(BUILDINGS_DIR, 'vehicle_bridge.py')
        if os.path.isfile(bridge):
            nodes.append(launch_ros.actions.Node(
                name='modride_bridge',
                executable='/usr/bin/python3',
                arguments=[bridge],
                parameters=[{'map_path': os.path.expanduser(map_path)}],
                output='screen'))
        autopose = os.path.join(BUILDINGS_DIR, 'auto_initial_pose.py')
        use_autopose = launch.substitutions.LaunchConfiguration('auto_initial_pose').perform(context)
        if os.path.isfile(autopose) and use_autopose.strip().lower() not in ('false', '0', 'no', 'off'):
            nodes.append(launch_ros.actions.Node(
                name='auto_initial_pose',
                executable='/usr/bin/python3',
                # -s: scipy from apt breaks on the numpy 2 in ~/.local
                arguments=['-s', autopose],
                parameters=[{'map_path': os.path.expanduser(map_path)}],
                output='screen'))
        return nodes

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
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'auto_initial_pose', default_value='true',
        description='Set the initial pose without RViz 2D Pose Estimate (saved pose / GPS / whole map).'))
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'map_path', default_value='',
        description='Map (.pcd/.ply) to localize on; empty = map_path in the parameter file.'))
    ld.add_action(odom_bridge)
    ld.add_action(launch.actions.OpaqueFunction(function=lidar_localization))
    ld.add_action(launch.actions.OpaqueFunction(function=campus_buildings))
    ld.add_action(lifecycle_manager)
    ld.add_action(robot_state_publisher)
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)
    ld.add_action(rviz_node)

    return ld
