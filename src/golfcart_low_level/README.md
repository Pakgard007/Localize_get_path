# golfcart_low_level

ROS 2 package for the golf cart low-level control layer.

The low-level launch runs two hardware-facing nodes on the Jetson:

- `ros_can_bridge`: supervisor and ROS 2 <-> STM32 CAN bridge.
- `steering_node`: KEYA steering motor driver over extended 29-bit CAN.

Command sources such as `joy_cmd_vel`, `joy_node`, `fake_cmd_vel`, or a
navigation stack must run separately and publish `/cmd_vel`.

Recommended folder layout:

```text
GolfCart/
  golfcart-low-level-control/
    ros2_ws/
      src/
        golfcart_low_level/
```

## CAN Bus

Use InnoMaker USB2CAN as SocketCAN `can0` at 250 kbit/s:

```bash
sudo ip link set can0 up type can bitrate 250000 restart-ms 100
```

The included helper script does the same setup:

```bash
bash ~/GolfCart/golfcart-low-level-control/ros2_ws/src/golfcart_low_level/scripts/setup_can0.sh can0 250000
```

All CAN nodes on the same physical bus must use the same bitrate. The STM32
brake/speed firmware, KEYA steering motor, and Jetson CAN interface should all
be set to 250 kbit/s.

The STM32 firmware in `firmware/stm32_motorbrake_car_shield` handles both brake
and speed/Curtis I/O:

- `0x120` speed command
- `0x121` speed enable
- `0x122` speed status
- `0x123` speed diagnostics
- `0x130` brake command
- `0x131` brake status
- `0x132` servo start/stop command
- `0x133` servo status
- `0x134` brake angle

## Build On Jetson

Install dependencies:

```bash
sudo apt update
sudo apt install -y python3-aenum python3-can can-utils python3-colcon-common-extensions
```

Build:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/GolfCart/golfcart-low-level-control/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Bring up CAN first:

```bash
bash src/golfcart_low_level/scripts/setup_can0.sh can0 250000
```

Launch the low-level stack. This starts only the hardware-facing low-level
nodes and waits for `/cmd_vel` from another process:

```bash
ros2 launch golfcart_low_level low_level.launch.py
```

Run exactly one `/cmd_vel` source in another terminal. For fake test commands:

```bash
source install/setup.bash
ros2 run golfcart_low_level fake_cmd_vel
```

For joystick control, start the USB joystick driver and the deadman teleop
together:

```bash
source install/setup.bash
ros2 launch golfcart_low_level joy_cmd_vel.launch.py
```

You can also run the two low-level nodes manually instead of using the launch
file:

```bash
ros2 run golfcart_low_level ros_can_bridge
ros2 run golfcart_low_level steering_node
```

## Main Topics

Inputs:

- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/joy` (`sensor_msgs/msg/Joy`, used by `joy_cmd_vel`)
- `/brake_cmd` (`std_msgs/msg/Bool`, manual brake override; `true = brake`, `false = return to /cmd_vel control`)

`joy_cmd_vel` only drives while the safety trigger is held. Default mapping:

- Safety: right trigger, either `buttons[7] == 1` or `axes[5]` pressed toward `-1.0`
- Emergency brake: A button (`buttons[0]`) publishes `/brake_cmd=true` and forces zero `/cmd_vel` while held
- Speed: left analog Y axis (`axes[1]`) scaled to `-3.0..3.0 m/s`
- Steering: right analog X axis (`axes[3]`) scaled to `-0.7..0.7 rad`

The default low-level config sets `ros_can_bridge.cmd_vel_steering_mode` to
`steering_angle`, so `cmd_vel.angular.z` is treated as a direct steering angle
in radians. Use `yaw_rate` if another navigation stack publishes normal ROS
`cmd_vel` yaw rate commands.

Internal bridge-to-steering command:

- `/steering_cmd` (`std_msgs/msg/Float64`, radians)

Status outputs:

- `/e_stop_status`
- `/lowlevel_fault`
- `/speed_status`
- `/speed_target`
- `/speed_status_flags`
- `/speed_fault_code`
- `/speed_sequence`
- `/speed_diagnostics`
- `/brake_status`
- `/brake_current`
- `/brake_sequence`
- `/brake_angle`
- `/servo_status`
- `/steering_angle`
- `/steering_fault`

Brake servo angle command:

- `/servo_command` (`std_msgs/msg/Float32`, updates brake engaged/stop angle; bridge sends CAN `0x132` as start+stop float32 pair)
- `/brake_angle_cmd` (`std_msgs/msg/Float32`, alias for engaged/stop angle)
- `/brake_servo_start_cmd` (`std_msgs/msg/Float32`, updates release/start angle)
- `/brake_servo_stop_cmd` (`std_msgs/msg/Float32`, updates engaged/stop angle)

## Quick Test

With `low_level.launch.py` already running, publish a stopped command from a
separate terminal:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Then a slow straight command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Watch status:

```bash
ros2 topic echo /lowlevel_fault
ros2 topic echo /e_stop_status
ros2 topic echo /speed_status
ros2 topic echo /speed_diagnostics
ros2 topic echo /brake_status
ros2 topic echo /steering_angle
```

## Brake Test

Apply the manual brake override:

```bash
ros2 topic pub --once /brake_cmd std_msgs/msg/Bool "{data: true}"
```

Release the manual override:

```bash
ros2 topic pub --once /brake_cmd std_msgs/msg/Bool "{data: false}"
```

Keep `/cmd_vel` publishing while releasing. With the default
`brake_on_zero_cmd: false`, zero `/cmd_vel` releases the brake after
`/brake_cmd` is false. Safety states such as e-stop, board timeout, or speed
fault still force the brake on.
