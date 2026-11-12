# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with os_cloud and os_"""

from pathlib import Path
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():

    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    default_params_file = Path(ouster_ros_pkg_dir) / 'config' / 'driver_params.yaml'

    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_params_file),
        description='name or path to the parameters file to use.'
    )

    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns',
        default_value='ouster'
    )

    os_driver_name = LaunchConfiguration('os_driver_name')
    os_driver_name_arg = DeclareLaunchArgument(
        'os_driver_name',
        default_value='os_driver'
    )

    # A 1024x128 cloud is 6.3 MB, and FastDDS's default shared-memory segment
    # is about 0.5 MB: the publisher runs out of room mid-message and the whole
    # cloud is lost. Measured 2026-10-02: subscribers received 71% of clouds
    # with the default (6.8 Hz of 10, gaps up to 1.6 s on the cart) and 97-98%
    # with this profile. Only the publishing side needs it.
    fastdds_profile = Path(ouster_ros_pkg_dir) / 'config' / 'fastdds_large_data.xml'

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name=os_driver_name,
        namespace=ouster_ns,
        parameters=[params_file],
        output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': str(fastdds_profile)},
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver,
            goal_state='inactive',
            entities=[
                LogInfo(msg="os_driver activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(os_driver),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True
        )
    )

    sensor_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver,
            goal_state='finalized',
            entities=[
                LogInfo(msg="Failed to communicate with the sensor in a timely manner."),
                EmitEvent(event=launch.events.Shutdown(
                    reason="Couldn't communicate with sensor"))
            ],
        )
    )

    return launch.LaunchDescription([
        params_file_arg,
        ouster_ns_arg,
        os_driver_name_arg,
        os_driver,
        sensor_configure_event,
        sensor_activate_event,
        sensor_finalized_event
    ])

