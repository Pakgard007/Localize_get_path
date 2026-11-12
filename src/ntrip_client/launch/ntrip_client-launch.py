import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('ntrip_client'), 'config')
    # ntrip.yaml is in git with an empty account; ntrip.local.yaml (gitignored)
    # holds the real username/password and is loaded after it, so it wins.
    params = [os.path.join(config_dir, 'ntrip.yaml')]
    local = os.path.join(config_dir, 'ntrip.local.yaml')
    if os.path.exists(local):
        params.append(local)

    return LaunchDescription([
        Node(
            package='ntrip_client',
            executable='ntrip_client',
            name='ntrip_client',
            output='screen',
            parameters=params,
        ),
    ])
