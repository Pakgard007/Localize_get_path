#!/usr/bin/env python3
"""Publish alternating /cmd_vel commands for low-level integration tests."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class FakeCmdVel(Node):
    def __init__(self):
        super().__init__('fake_cmd_vel')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('switch_period_s', 10.0)

        topic = str(self.get_parameter('topic').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.switch_period_s = float(self.get_parameter('switch_period_s').value)

        self.commands = [
            (1.0, 0.7),
            (3.0, -0.7),
            (0.0, 0.0),
        ]
        self.active_index = 0
        self.switch_time = self.get_clock().now()

        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.get_logger().info(
            f'publishing {topic} at {rate_hz:.1f} Hz; '
            f'switch_period={self.switch_period_s:.1f}s')
        self._log_active_command()

    def _timer_cb(self):
        now = self.get_clock().now()
        elapsed = (now - self.switch_time).nanoseconds * 1e-9
        if elapsed >= self.switch_period_s:
            self.active_index = (self.active_index + 1) % len(self.commands)
            self.switch_time = now
            self._log_active_command()

        linear_x, angular_z = self.commands[self.active_index]
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def _log_active_command(self):
        linear_x, angular_z = self.commands[self.active_index]
        self.get_logger().info(
            f'active cmd_vel: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = FakeCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
