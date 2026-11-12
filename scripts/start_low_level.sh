#!/bin/bash
# Starts golfcart_low_level's low_level.launch.py.
# Run stop_low_level.sh (or the "Stop Low Level" desktop icon) to shut it down cleanly.

PIDFILE="/tmp/golfcart_low_level_launch.pid"

if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "low_level.launch.py is already running (PID $(cat "$PIDFILE")). Not starting again."
    read -n 1 -s -r -p "Press any key to close this window..."
    exit 0
fi

echo $$ > "$PIDFILE"

source /opt/ros/humble/setup.bash
source /home/myuser/2Localize_get_path/install/setup.bash

exec ros2 launch golfcart_low_level low_level.launch.py
