#!/bin/bash
# Stops whatever start_low_level.sh started, the same way Ctrl+C in that
# terminal would (SIGINT, so ros2 launch shuts its child nodes down cleanly).

PIDFILE="/tmp/golfcart_low_level_launch.pid"

if [ ! -f "$PIDFILE" ]; then
    echo "No PID file found -- low_level.launch.py doesn't look like it's running."
    read -n 1 -s -r -p "Press any key to close this window..."
    exit 1
fi

PID=$(cat "$PIDFILE")

if kill -0 "$PID" 2>/dev/null; then
    echo "Stopping low_level.launch.py (PID $PID)..."
    kill -INT "$PID"

    for i in $(seq 1 10); do
        kill -0 "$PID" 2>/dev/null || break
        sleep 0.5
    done

    if kill -0 "$PID" 2>/dev/null; then
        echo "Still running after 5s, sending SIGTERM..."
        kill -TERM "$PID"
    else
        echo "Stopped."
    fi
else
    echo "Process $PID not running (stale PID file)."
fi

rm -f "$PIDFILE"
read -n 1 -s -r -p "Press any key to close this window..."
