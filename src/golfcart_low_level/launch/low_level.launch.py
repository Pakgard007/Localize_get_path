from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    default_config = PathJoinSubstitution([
        FindPackageShare('golfcart_low_level'),
        'config',
        'golfcart_low_level.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML file with golfcart_low_level node parameters.',
        ),
        Node(
            package='golfcart_low_level',
            executable='ros_can_bridge',
            name='ros_can_bridge',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='golfcart_low_level',
            executable='steering_node',
            name='steering_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
