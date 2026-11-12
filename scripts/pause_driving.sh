#!/bin/bash
# Pauses pagard_adaptive_fixed.py's cmd_vel immediately (publishes zero
# velocity and holds it) without killing the node. Run resume_driving.sh
# (or the matching keyboard shortcut) to let it drive again.

source /opt/ros/humble/setup.bash
source /home/myuser/2Localize_get_path/install/setup.bash

ros2 topic pub /pause_cmd_vel std_msgs/msg/Bool "{data: true}" --once
notify-send "Driving paused" "cmd_vel held at zero" 2>/dev/null || true
