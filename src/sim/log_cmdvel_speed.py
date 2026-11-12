#!/usr/bin/env python3
"""เก็บ log เทียบคำสั่ง /cmd_vel กับความเร็วจริง /speed_status ลงไฟล์ CSV

Run:
    python3 log_cmdvel_speed.py

เปลี่ยน topic/อัตราการเก็บได้ด้วย ``--ros-args -p cmd_vel_topic:=/cmd_vel_preview``
หรือ ``-p log_rate_hz:=20.0``

ต่างจาก logcsv.py ตรงที่ไม่ต้องรอ pose/NDT/controller debug ใด ๆ ขอแค่มี
cmd_vel กับ speed_status ก็เก็บได้ เอาไว้ดู response ของรถล้วน ๆ
"""

import csv
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class CmdVelSpeedLogger(Node):
    def __init__(self):
        super().__init__('log_cmdvel_speed')

        # ตอน pagard อยู่โหมด dry_run คำสั่งจะออกที่ /cmd_vel_preview ไม่ใช่ /cmd_vel
        # จึง subscribe ทั้งสองเส้นเหมือน logcsv.py ใครส่งมาก็เก็บอันนั้น
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('dry_run_cmd_vel_topic', '/cmd_vel_preview')
        self.declare_parameter('speed_topic', '/speed_status')
        self.declare_parameter('log_rate_hz', 10.0)
        self.declare_parameter('csv_path', '')

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.dry_run_cmd_vel_topic = str(self.get_parameter('dry_run_cmd_vel_topic').value)
        self.speed_topic = str(self.get_parameter('speed_topic').value)
        rate = max(1.0, float(self.get_parameter('log_rate_hz').value))

        self.cmd_speed = None
        self.cmd_steer = None
        self.speed_status = None
        self.cmd_vel_source = None
        self.last_cmd_sec = None
        self.last_speed_sec = None
        self.start_time = None

        csv_path = str(self.get_parameter('csv_path').value)
        if not csv_path:
            # แยกโฟลเดอร์ตามวันที่เริ่มรัน เหมือน log ตัวอื่นใน ~/controller_logs
            csv_path = datetime.now().strftime(
                '~/controller_logs/%Y-%m-%d/cmdvel_speed_log_%Y%m%d_%H%M%S.csv'
            )
        self.csv_filename = os.path.expanduser(csv_path)
        log_dir = os.path.dirname(self.csv_filename)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Time (s)',
            'ROS Time (s)',
            'cmd_vel Source',
            'cmd_vel linear.x (m/s)',
            'cmd_vel angular.z (rad/s)',
            'speed_status (m/s)',
            'Speed Error (cmd - actual)',
            'cmd_vel Age (s)',
            'speed_status Age (s)',
        ])

        self.create_subscription(
            Twist, self.cmd_vel_topic,
            lambda msg: self.cmd_vel_callback(msg, self.cmd_vel_topic), 10
        )
        if self.dry_run_cmd_vel_topic != self.cmd_vel_topic:
            self.create_subscription(
                Twist, self.dry_run_cmd_vel_topic,
                lambda msg: self.cmd_vel_callback(msg, self.dry_run_cmd_vel_topic), 10
            )
        self.create_subscription(Float64, self.speed_topic, self.speed_status_callback, 10)

        self.create_timer(1.0 / rate, self.write_row)

        self.get_logger().info(
            f'Logging {self.cmd_vel_topic} (+{self.dry_run_cmd_vel_topic}) '
            f'and {self.speed_topic} -> {self.csv_filename} at {rate:.1f} Hz'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_vel_callback(self, msg, source_topic):
        self.cmd_speed = float(msg.linear.x)
        self.cmd_steer = float(msg.angular.z)
        self.last_cmd_sec = self.now_sec()
        if self.cmd_vel_source != source_topic:
            self.cmd_vel_source = source_topic
            self.get_logger().info(f'cmd_vel source: {source_topic}')

    def speed_status_callback(self, msg):
        self.speed_status = float(msg.data)
        self.last_speed_sec = self.now_sec()

    def write_row(self):
        # ยังไม่มีข้อมูลสักฝั่งก็ไม่ต้องเขียน กันไฟล์เต็มด้วยแถวว่าง ๆ ตอนรอ node อื่นขึ้น
        if self.cmd_speed is None and self.speed_status is None:
            return

        now = self.now_sec()
        if self.start_time is None:
            self.start_time = now
        elapsed = now - self.start_time

        error = None
        if self.cmd_speed is not None and self.speed_status is not None:
            error = self.cmd_speed - self.speed_status

        cmd_age = None if self.last_cmd_sec is None else now - self.last_cmd_sec
        speed_age = None if self.last_speed_sec is None else now - self.last_speed_sec

        self.csv_writer.writerow([
            f'{elapsed:.3f}',
            f'{now:.3f}',
            self.cmd_vel_source if self.cmd_vel_source is not None else '',
            '' if self.cmd_speed is None else f'{self.cmd_speed:.4f}',
            '' if self.cmd_steer is None else f'{self.cmd_steer:.4f}',
            '' if self.speed_status is None else f'{self.speed_status:.4f}',
            '' if error is None else f'{error:.4f}',
            '' if cmd_age is None else f'{cmd_age:.3f}',
            '' if speed_age is None else f'{speed_age:.3f}',
        ])
        # flush ทุกแถว เผื่อโดน Ctrl-C หรือไฟดับกลางทาง ข้อมูลที่เก็บมาแล้วไม่หาย
        self.csv_file.flush()

    def destroy_node(self):
        if not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'CSV saved: {self.csv_filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSpeedLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
