#!/usr/bin/env python3
"""Joystick teleop for the golf cart.

Publishes /cmd_vel only while the safety trigger is held. The left stick Y axis
controls speed and the right stick X axis controls steering angle in radians.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyCmdVel(Node):
    def __init__(self):
        super().__init__('joy_cmd_vel')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('brake_cmd_topic', '/brake_cmd')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('left_stick_y_axis', 1)
        self.declare_parameter('right_stick_x_axis', 3)
        self.declare_parameter('safety_button_index', 7)
        self.declare_parameter('emergency_button_index', 0)
        self.declare_parameter('safety_axis_index', 5)
        self.declare_parameter('safety_axis_pressed_value', -1.0)
        self.declare_parameter('safety_axis_threshold', 0.5)
        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('max_steering_rad', 0.7)
        self.declare_parameter('speed_deadzone', 0.05)
        self.declare_parameter('steering_deadzone', 0.05)
        self.declare_parameter('speed_axis_gain', 1.0)
        self.declare_parameter('steering_axis_gain', 1.0)
        self.declare_parameter('invert_speed_axis', False)
        self.declare_parameter('invert_steering_axis', False)
        self.declare_parameter('publish_zero_when_released', True)
        self.declare_parameter('joy_timeout_s', 0.5)
        self.declare_parameter('debug_joy', False)

        joy_topic = str(self.get_parameter('joy_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        brake_cmd_topic = str(self.get_parameter('brake_cmd_topic').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.left_y_axis = int(self.get_parameter('left_stick_y_axis').value)
        self.right_x_axis = int(self.get_parameter('right_stick_x_axis').value)
        self.safety_button_index = int(self.get_parameter('safety_button_index').value)
        self.emergency_button_index = int(self.get_parameter('emergency_button_index').value)
        self.safety_axis_index = int(self.get_parameter('safety_axis_index').value)
        self.safety_axis_pressed_value = float(
            self.get_parameter('safety_axis_pressed_value').value
        )
        self.safety_axis_threshold = abs(
            float(self.get_parameter('safety_axis_threshold').value)
        )
        self.max_speed = abs(float(self.get_parameter('max_speed_mps').value))
        self.max_steering = abs(float(self.get_parameter('max_steering_rad').value))
        self.speed_deadzone = abs(float(self.get_parameter('speed_deadzone').value))
        self.steering_deadzone = abs(float(self.get_parameter('steering_deadzone').value))
        self.speed_axis_gain = float(self.get_parameter('speed_axis_gain').value)
        self.steering_axis_gain = float(self.get_parameter('steering_axis_gain').value)
        self.invert_speed_axis = bool(self.get_parameter('invert_speed_axis').value)
        self.invert_steering_axis = bool(self.get_parameter('invert_steering_axis').value)
        self.publish_zero_when_released = bool(
            self.get_parameter('publish_zero_when_released').value
        )
        self.joy_timeout_s = float(self.get_parameter('joy_timeout_s').value)
        self.debug_joy = bool(self.get_parameter('debug_joy').value)

        self.last_joy = None
        self.last_joy_stamp = None
        self.safety_held = False
        self.emergency_active = False

        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pub_brake = self.create_publisher(Bool, brake_cmd_topic, 10)
        self.create_subscription(Joy, joy_topic, self._cb_joy, 10)
        self.create_timer(1.0 / rate, self._publish_cmd)

        self.get_logger().info(
            f'joy_cmd_vel up: {joy_topic} -> {cmd_vel_topic}, '
            f'brake_cmd={brake_cmd_topic}, '
            f'max_speed={self.max_speed:.2f} m/s, '
            f'max_steering={self.max_steering:.2f} rad')

    def _cb_joy(self, msg: Joy):
        self.last_joy = msg
        self.last_joy_stamp = self.get_clock().now()
        if self.debug_joy:
            axes = ', '.join(f'{i}:{v:.3f}' for i, v in enumerate(msg.axes))
            buttons = ', '.join(f'{i}:{v}' for i, v in enumerate(msg.buttons))
            self.get_logger().info(
                f'joy axes=[{axes}] buttons=[{buttons}]',
                throttle_duration_sec=0.5)

    def _publish_cmd(self):
        msg = Twist()
        joy_fresh = self._joy_is_fresh()
        emergency = joy_fresh and self._button_is_pressed(
            self.last_joy, self.emergency_button_index
        )
        active = joy_fresh and not emergency and self._safety_is_held(self.last_joy)

        if active:
            speed_axis = self._axis(self.last_joy, self.left_y_axis)
            steering_axis = self._axis(self.last_joy, self.right_x_axis)

            if self.invert_speed_axis:
                speed_axis *= -1.0
            if self.invert_steering_axis:
                steering_axis *= -1.0

            speed_axis = self._deadzone(speed_axis, self.speed_deadzone)
            steering_axis = self._deadzone(steering_axis, self.steering_deadzone)
            speed_axis = self._clamp_unit(speed_axis * self.speed_axis_gain)
            steering_axis = self._clamp_unit(
                steering_axis * self.steering_axis_gain
            )

            msg.linear.x = self.max_speed * speed_axis
            msg.angular.z = self.max_steering * steering_axis

        if active or self.publish_zero_when_released:
            self.pub_cmd.publish(msg)

        if emergency:
            self.pub_brake.publish(Bool(data=True))

        if emergency != self.emergency_active:
            self.pub_brake.publish(Bool(data=emergency))
            state = 'active' if emergency else 'released'
            self.get_logger().warn(f'joy emergency {state}')
            self.emergency_active = emergency

        if active != self.safety_held:
            state = 'held' if active else 'released'
            self.get_logger().info(f'joy safety trigger {state}')
            self.safety_held = active

    def _joy_is_fresh(self) -> bool:
        if self.last_joy is None or self.last_joy_stamp is None:
            return False
        age = (self.get_clock().now() - self.last_joy_stamp).nanoseconds * 1e-9
        return age <= self.joy_timeout_s

    def _safety_is_held(self, msg: Joy) -> bool:
        if msg is None:
            return False

        button_pressed = (
            self._button_is_pressed(msg, self.safety_button_index)
        )

        axis_pressed = False
        if 0 <= self.safety_axis_index < len(msg.axes):
            value = float(msg.axes[self.safety_axis_index])
            axis_pressed = abs(value - self.safety_axis_pressed_value) <= self.safety_axis_threshold

        return button_pressed or axis_pressed

    @staticmethod
    def _button_is_pressed(msg: Joy, index: int) -> bool:
        return (
            msg is not None
            and 0 <= index < len(msg.buttons)
            and bool(msg.buttons[index])
        )

    @staticmethod
    def _axis(msg: Joy, index: int) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        return JoyCmdVel._clamp_unit(float(msg.axes[index]))

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return max(-1.0, min(1.0, value))

    @staticmethod
    def _deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value


def main(args=None):
    rclpy.init(args=args)
    node = JoyCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
