#!/usr/bin/env python3
"""Keyboard teleoperation node that drives ``angular.z`` on a ROS 2 ``Twist`` topic.

Run:
    python3 keyboard_cmd_vel_z.py

Edit the ``declare_parameter`` defaults below to change the topic or limits.
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Keyboard cmd_vel controller (angular.z)
  Type a number then Enter : send that angular velocity in rad/s
                              example: 0.40 Enter, -0.25 Enter, 0 Enter (stop)
  L / R           : hold the current linear.x, type a number then Enter to change it
  Backspace       : edit the typed number
  Space or X      : stop immediately
  Q or Ctrl-C     : stop and quit
"""


class KeyboardCmdVelZ(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_vel_z')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('max_linear_speed', 10.0)
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.max_angular = max(0.0, float(self.get_parameter('max_angular_speed').value))
        self.max_linear = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self.linear = min(
            self.max_linear,
            max(-self.max_linear, float(self.get_parameter('linear_speed').value)),
        )
        self.angular = 0.0
        self.target = 'angular'
        self.number_buffer = ''
        self.lock = threading.Lock()
        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publisher = self.create_publisher(Twist, topic, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(f'Publishing keyboard angular commands to {topic}')

    def publish_command(self):
        with self.lock:
            linear, angular = self.linear, self.angular
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def stop(self):
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0
        self.publish_command()

    def apply_value(self, value):
        if self.target == 'linear':
            value = min(self.max_linear, max(-self.max_linear, value))
            with self.lock:
                self.linear = value
            self.get_logger().info(f'cmd_vel.linear.x = {value:.2f} m/s')
        else:
            value = min(self.max_angular, max(-self.max_angular, value))
            with self.lock:
                self.angular = value
            self.get_logger().info(f'cmd_vel.angular.z = {value:.2f} rad/s')

    def handle_key(self, key):
        if key in ('\r', '\n'):
            try:
                value = float(self.number_buffer)
            except ValueError:
                self.get_logger().warning('Enter a valid number, for example 0.5')
            else:
                self.apply_value(value)
            self.number_buffer = ''
        elif key in ('\x7f', '\b'):
            self.number_buffer = self.number_buffer[:-1]
        elif key in '0123456789.-' and len(self.number_buffer) < 12:
            self.number_buffer += key
        elif key in ('l', 'r'):
            self.number_buffer = ''
            self.target = 'linear' if key == 'l' else 'angular'
            self.get_logger().info(f'Next number sets {self.target}')
        elif key in (' ', 'x'):
            self.number_buffer = ''
            self.stop()


def read_key():
    """Read one character; collect the full three-byte arrow-key sequence."""
    key = sys.stdin.read(1)
    if key == '\x1b' and select.select([sys.stdin], [], [], 0.03)[0]:
        key += sys.stdin.read(2)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCmdVelZ()
    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP, flush=True)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key.lower() == 'q' or key == '\x03':
                break
            node.handle_key(key.lower() if key.isalpha() else key)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
