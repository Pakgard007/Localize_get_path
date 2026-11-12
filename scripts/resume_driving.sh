#!/bin/bash
# Resumes normal driving after pause_driving.sh paused it.

source /opt/ros/humble/setup.bash
source /home/myuser/2Localize_get_path/install/setup.bash

ros2 topic pub /pause_cmd_vel std_msgs/msg/Bool "{data: false}" --once
notify-send "Driving resumed" 2>/dev/null || true
