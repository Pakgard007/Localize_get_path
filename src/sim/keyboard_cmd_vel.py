#!/usr/bin/env python3
"""Keyboard teleoperation node for a ROS 2 ``geometry_msgs/Twist`` topic.

Run:
    python3 keyboard_cmd_vel.py

Use ``--ros-args -p cmd_vel_topic:=/some_topic`` to select another topic.
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
Keyboard cmd_vel controller
  Type a number then Enter : send that linear velocity in m/s
                              example: 1.25 Enter, -0.30 Enter, 0 Enter (stop)
  Backspace       : edit the typed number
  Space or X      : stop immediately
  Q or Ctrl-C    : stop and quit
"""


class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_vel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_speed', 10.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.max_linear = max(0.0, float(self.get_parameter('max_linear_speed').value))
        self.linear = 0.0
        self.angular = 0.0
        self.number_buffer = ''
        self.lock = threading.Lock()
        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publisher = self.create_publisher(Twist, topic, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(f'Publishing keyboard commands to {topic}')

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

    def handle_key(self, key):
        if key in ('\r', '\n'):
            try:
                value = float(self.number_buffer)
            except ValueError:
                self.get_logger().warning('Enter a valid number, for example 0.5')
            else:
                value = min(self.max_linear, max(-self.max_linear, value))
                with self.lock:
                    self.linear = value
                    self.angular = 0.0
                self.get_logger().info(f'cmd_vel.linear.x = {value:.2f} m/s')
            self.number_buffer = ''
        elif key in ('\x7f', '\b'):
            self.number_buffer = self.number_buffer[:-1]
        elif key in '0123456789.-' and len(self.number_buffer) < 12:
            self.number_buffer += key
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
    node = KeyboardCmdVel()
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
            node.handle_key(key)
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
