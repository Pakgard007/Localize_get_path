# Brake Control Examples

Use these examples after `can0` is up and `ros_can_bridge` is running.

## Recommended ROS Topic

Use `/brake_cmd` for manual brake tests:

```bash
ros2 topic pub --once /brake_cmd std_msgs/msg/Bool "{data: true}"
```

This manual override makes `ros_can_bridge` repeatedly send:

```text
CAN 0x121 speed_enable = 0
CAN 0x120 target_speed = 0.00 m/s
CAN 0x130 brake_cmd = 1
```

Release the manual override:

```bash
ros2 topic pub --once /brake_cmd std_msgs/msg/Bool "{data: false}"
```

After release, the bridge returns to normal `/cmd_vel` control. Keep a fresh
zero `/cmd_vel` running while releasing:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

With the default `brake_on_zero_cmd: false`, zero `/cmd_vel` releases the brake.
If `brake_on_zero_cmd: true`, zero `/cmd_vel` keeps the brake engaged.

## Watch Feedback

```bash
ros2 topic echo /brake_status
ros2 topic echo /brake_current
ros2 topic echo /brake_angle
ros2 topic echo /e_stop_status
ros2 topic echo /lowlevel_fault
```

Raw CAN check:

```bash
candump can0,130:7FF,131:7FF,134:7FF
```

Expected command when braking:

```text
can0  130   [1]  01
```

Expected status topics:

```text
/brake_status: 1      # relay active
/brake_angle: near configured stop/engage angle
```

## Safety Notes

`/brake_cmd` cannot override safety. If e-stop, board timeout, speed fault, or
`/cmd_vel` timeout is active, `ros_can_bridge` still commands:

```text
speed_enable = 0
target_speed = 0
brake_cmd = 1
```

Do not send raw `cansend can0 130#00` while `ros_can_bridge` is running, because
the bridge is the command owner and will immediately overwrite the raw CAN frame.
