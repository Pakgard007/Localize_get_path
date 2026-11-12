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
            description='YAML file with joy_cmd_vel node parameters.',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='golfcart_low_level',
            executable='joy_cmd_vel',
            name='joy_cmd_vel',
            output='screen',
            parameters=[config_file],
        ),
    ])
