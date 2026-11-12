#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3Stamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float64, Int32, Float32, Bool

import csv
import math
import os
from datetime import datetime


class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger_node')

        # ---- parameters ----
        # cmd_vel: ตอน pagard อยู่โหมด dry_run คำสั่งจะออกที่ /cmd_vel_preview ไม่ใช่ /cmd_vel
        # จึง subscribe ทั้งสองเส้น ใครมาก่อนใช้อันนั้น ไม่ต้องมาแก้ config ทุกครั้งที่สลับโหมด
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('dry_run_cmd_vel_topic', '/cmd_vel_preview')
        # true = กลับไปใช้พฤติกรรมเดิม คือต้องครบทุก topic ถึงจะเขียนสักแถว
        self.declare_parameter('require_all_topics', False)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.dry_run_cmd_vel_topic = str(
            self.get_parameter('dry_run_cmd_vel_topic').value
        )
        self.require_all_topics = bool(
            self.get_parameter('require_all_topics').value
        )
        self.cmd_vel_source = None
        self.logged_optional_gaps = False

        self.controller_frequency = 10.0

        self.vehicle_x = None
        self.vehicle_y = None
        self.vehicle_z = None
        self.target_x = None
        self.target_y = None
        self.target_z = None
        # z จาก /pcl_pose แยกไว้ต่างหาก เพราะเป็นผลลัพธ์ NDT ตรง ๆ
        # ส่วน z ใน /current_pose อาจถูก controller เขียนทับ/ปรับมาแล้ว
        self.pcl_z = None
        # ความเร็วหลักใช้ /speed_status (encoder) เป็นแหล่งจริง
        # ส่วนที่ดิฟจาก /pcl_pose เก็บไว้เป็น fallback ตอนยังไม่มีเฟรม CAN เท่านั้น
        # เพราะ pcl_pose ส่งซ้ำเฟรมเดิมบ่อย ทำให้ค่าเด้ง 0 สลับกับค่าพุ่ง
        self.velocity_pose = None
        self.velocity_source = None
        self.yaw = None
        # ตำแหน่ง/yaw รับได้จากสองทาง: /current_pose (pagard แปลงมาให้) กับ
        # /pcl_pose (ผลลัพธ์ NDT ตรง ๆ) — ถ้าไม่มี pagard รัน /current_pose จะเงียบ
        # จึงต้องอ่าน /pcl_pose ได้เอง ไม่งั้นเก็บข้อมูลไม่ได้เลยทั้งที่ localization ทำงานอยู่
        self.last_current_pose_sec = None
        self.last_euler_sec = None
        self.pose_source = None
        self.yaw_source = None
        self.source_timeout_sec = 1.0
        self.adaptive_ld = None

        # Real control-loop variables, subscribed from pagard_adaptive_fixed.py's
        # own debug/command topics instead of being re-derived locally.
        self.controller_velocity = None
        self.debug_cte = None
        self.debug_heading_error = None
        self.debug_path_heading = None
        self.debug_lookahead_distance = None
        self.debug_adaptive_ld = None
        self.curvature = None
        self.cmd_speed = None
        self.cmd_steer = None
        self.speed_status = None
        # steering chain: /cmd_vel.angular.z -> ros_can_bridge clamp -> /steering_cmd
        #                 -> KEYA -> /steering_angle (feedback)
        self.steering_cmd_bridge = None
        self.steering_angle_actual = None
        self.steering_fault = None
        self.remaining_distance = None
        self.closest_waypoint_index = None
        self.target_waypoint_index = None

        self.prev_x = None
        self.prev_y = None
        self.prev_time = None

        # แยกโฟลเดอร์ตามวันที่เริ่มรัน: ~/controller_logs/2026-10-07/logcsv_<วันเวลา>.csv
        # ชื่อขึ้นต้น logcsv_ เพราะ pagard เขียน controller_log_<วันเวลา>.csv (คนละคอลัมน์)
        # ลงโฟลเดอร์เดียวกัน ถ้าชื่อซ้ำแล้วเริ่มวินาทีเดียวกันจะเขียนทับไฟล์กัน
        self.csv_filename = os.path.expanduser(
            datetime.now().strftime(
                "~/controller_logs/%Y-%m-%d/logcsv_%Y%m%d_%H%M%S.csv"
            )
        )
        os.makedirs(os.path.dirname(self.csv_filename), exist_ok=True)

        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "Controller Frequency:",
            "Vehicle Position (x)",
            "Vehicle Position (y)",
            "Vehicle Position (z)",
            "Vehicle Position (z, pcl_pose)",
            "Target Point (x)",
            "Target Point (y)",
            "Target Point (z)",
            "Velocity (speed_status)",
            "Velocity (pose est.)",
            "Velocity (controller)",
            "Yaw",
            "CTE",
            "Heading Error",
            "Path Heading",
            "Curvature (κ)",
            "Lookahead Distance",
            "Adaptive LD (raw fuzzy)",
            "Adaptive LD (applied)",
            "Cmd Speed",
            "Steering Angle (cmd)",
            "Steering Cmd (bridge rad)",
            "Steering Angle (actual, keya feedback)",
            "Steering Cmd (bridge deg)",
            "Steering Angle (actual deg)",
            "Steering Error (bridge cmd - actual, deg)",
            "Steering Fault",
            "Speed Status (encoder m/s)",
            "Remaining Distance",
            "Closest Waypoint Index",
            "Target Waypoint Index",
        ])

        self.create_subscription(PoseStamped, '/current_pose', self.current_pose_callback, 10)
        self.create_subscription(PoseStamped, '/target_waypoint', self.target_waypoint_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', self.pcl_pose_callback, 10)
        self.create_subscription(Vector3Stamped, '/filter/euler', self.euler_callback, 10)

        self.create_subscription(Float64, '/adaptive_ld', self.adaptive_ld_callback, 10)
        self.create_subscription(Float64, '/velocity', self.controller_velocity_callback, 10)
        self.create_subscription(Float64, '/curvature', self.curvature_callback, 10)
        self.create_subscription(Float64, '/controller_debug/cte', self.debug_cte_callback, 10)
        self.create_subscription(Float64, '/controller_debug/heading_error', self.debug_heading_error_callback, 10)
        self.create_subscription(Float64, '/controller_debug/path_heading', self.debug_path_heading_callback, 10)
        self.create_subscription(Float64, '/controller_debug/lookahead_distance', self.debug_lookahead_distance_callback, 10)
        self.create_subscription(Float64, '/controller_debug/adaptive_ld', self.debug_adaptive_ld_callback, 10)
        self.create_subscription(
            Twist, self.cmd_vel_topic,
            lambda msg: self.cmd_vel_callback(msg, self.cmd_vel_topic), 10
        )
        if self.dry_run_cmd_vel_topic != self.cmd_vel_topic:
            self.create_subscription(
                Twist, self.dry_run_cmd_vel_topic,
                lambda msg: self.cmd_vel_callback(msg, self.dry_run_cmd_vel_topic), 10
            )
        self.create_subscription(Float64, '/speed_status', self.speed_status_callback, 10)
        self.create_subscription(Float64, '/steering_cmd', self.steering_cmd_callback, 10)
        self.create_subscription(Float64, '/steering_angle', self.steering_angle_actual_callback, 10)
        self.create_subscription(Bool, '/steering_fault', self.steering_fault_callback, 10)
        self.create_subscription(Float32, '/remaining_distance', self.remaining_distance_callback, 10)
        self.create_subscription(Int32, '/closest_waypoint_index', self.closest_index_callback, 10)
        self.create_subscription(Int32, '/target_waypoint_index', self.target_index_callback, 10)

        timer_period = 1.0 / self.controller_frequency
        self.timer = self.create_timer(timer_period, self.write_csv)

        self.get_logger().info(f"CSV Logger started: {self.csv_filename}")

    def adaptive_ld_callback(self, msg):
        self.adaptive_ld = float(msg.data)

    def controller_velocity_callback(self, msg):
        self.controller_velocity = float(msg.data)

    def curvature_callback(self, msg):
        self.curvature = float(msg.data)

    def debug_cte_callback(self, msg):
        self.debug_cte = float(msg.data)

    def debug_heading_error_callback(self, msg):
        self.debug_heading_error = float(msg.data)

    def debug_path_heading_callback(self, msg):
        self.debug_path_heading = float(msg.data)

    def debug_lookahead_distance_callback(self, msg):
        self.debug_lookahead_distance = float(msg.data)

    def debug_adaptive_ld_callback(self, msg):
        self.debug_adaptive_ld = float(msg.data)

    def cmd_vel_callback(self, msg, source_topic):
        self.cmd_speed = float(msg.linear.x)
        self.cmd_steer = float(msg.angular.z)
        if self.cmd_vel_source != source_topic:
            self.cmd_vel_source = source_topic
            self.get_logger().info(f"cmd_vel source: {source_topic}")

    def speed_status_callback(self, msg):
        self.speed_status = float(msg.data)

    def steering_cmd_callback(self, msg):
        self.steering_cmd_bridge = float(msg.data)

    def steering_angle_actual_callback(self, msg):
        self.steering_angle_actual = float(msg.data)

    def steering_fault_callback(self, msg):
        self.steering_fault = bool(msg.data)

    def remaining_distance_callback(self, msg):
        self.remaining_distance = float(msg.data)

    def closest_index_callback(self, msg):
        self.closest_waypoint_index = int(msg.data)

    def target_index_callback(self, msg):
        self.target_waypoint_index = int(msg.data)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_fresh(self, stamp_sec):
        return stamp_sec is not None and (self.now_sec() - stamp_sec) <= self.source_timeout_sec

    def note_source(self, kind, name):
        """log เฉพาะตอนแหล่งข้อมูลเปลี่ยน จะได้รู้ว่ากำลังเก็บจากอะไรอยู่"""
        attr = 'pose_source' if kind == 'pose' else 'yaw_source'
        if getattr(self, attr) != name:
            setattr(self, attr, name)
            self.get_logger().info(f'{kind} source: {name}')

    def apply_pose(self, x, y, z, yaw_from_pose, source):
        self.vehicle_x = x
        self.vehicle_y = y
        self.vehicle_z = z
        self.note_source('pose', source)

        # /filter/euler ถ้ายังสดให้ชนะ ไม่งั้นใช้ yaw จาก pose และต้องอัปเดตทุกเฟรม
        # (ของเดิมเซ็ตแค่ตอน yaw เป็น None ทำให้ค้างค่าแรกตลอดถ้าไม่มี /filter/euler)
        if not self.is_fresh(self.last_euler_sec):
            self.yaw = yaw_from_pose
            self.note_source('yaw', source)

    def current_pose_callback(self, msg):
        self.last_current_pose_sec = self.now_sec()
        q = msg.pose.orientation
        self.apply_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            self.quaternion_to_yaw(q.x, q.y, q.z, q.w),
            '/current_pose'
        )

    def target_waypoint_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z

    def pcl_pose_callback(self, msg):
        stamp = msg.header.stamp
        current_time = rclpy.time.Time.from_msg(stamp)
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        self.pcl_z = msg.pose.pose.position.z

        if self.prev_x is not None and self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                dx = current_x - self.prev_x
                dy = current_y - self.prev_y
                self.velocity_pose = math.sqrt(dx**2 + dy**2) / dt

        # /current_pose มาจาก pagard ถ้าไม่ได้รันอยู่ จะไม่มีใครส่งเลย
        # กรณีนั้นใช้ NDT ตรง ๆ เป็นแหล่งตำแหน่ง/yaw เพื่อให้เก็บข้อมูลได้โดยไม่ต้องขยับรถ
        if not self.is_fresh(self.last_current_pose_sec):
            q = msg.pose.pose.orientation
            self.apply_pose(
                current_x,
                current_y,
                self.pcl_z,
                self.quaternion_to_yaw(q.x, q.y, q.z, q.w),
                '/pcl_pose'
            )

        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_time = current_time

    def euler_callback(self, msg):
        self.yaw = msg.vector.z
        self.last_euler_sec = self.now_sec()
        self.note_source('yaw', '/filter/euler')

    def write_csv(self):
        # เดิมต้องครบทุก topic ถึงจะเขียนสักแถว ทำให้ของที่ต้องพึ่งฮาร์ดแวร์
        # (/speed_status, /steering_angle ที่ต้องมีเฟรม CAN) หรือ /cmd_vel ตอน dry run
        # ขาดไปแค่อันเดียวก็บล็อกการเก็บทั้งหมด
        # ตอนนี้บังคับเฉพาะข้อมูลแกนหลักที่ทำให้แถวมีความหมาย ที่เหลือขาดได้ เว้นเป็นช่องว่างไว้
        gate = self.has_all_data if self.require_all_topics else self.has_core_data
        if not gate():
            self.print_missing_data()
            return

        self.log_optional_gaps_once()

        self.csv_writer.writerow([
            self.controller_frequency,
            self.vehicle_x,
            self.vehicle_y,
            self.vehicle_z,
            self.pcl_z,
            self.target_x,
            self.target_y,
            self.target_z,
            self.current_velocity(),
            self.velocity_pose,
            self.controller_velocity,
            self.yaw,
            self.debug_cte,
            self.debug_heading_error,
            self.debug_path_heading,
            self.curvature,
            self.debug_lookahead_distance,
            self.adaptive_ld,
            self.debug_adaptive_ld,
            self.cmd_speed,
            self.cmd_steer,
            self.steering_cmd_bridge,
            self.steering_angle_actual,
            self.to_degrees(self.steering_cmd_bridge),
            self.to_degrees(self.steering_angle_actual),
            self.steering_error_deg(),
            self.steering_fault,
            self.speed_status,
            self.remaining_distance,
            self.closest_waypoint_index,
            self.target_waypoint_index,
        ])

        self.csv_file.flush()

    def current_velocity(self):
        """ความเร็วที่ใช้จริงใน CSV — encoder ก่อน แล้วค่อย fallback ไป pose"""
        if self.speed_status is not None:
            self.note_velocity_source('/speed_status')
            return self.speed_status

        if self.velocity_pose is not None:
            self.note_velocity_source('/pcl_pose (fallback)')
        return self.velocity_pose

    def note_velocity_source(self, source):
        if self.velocity_source != source:
            self.velocity_source = source
            self.get_logger().info(f"velocity source: {source}")

    def to_degrees(self, radians):
        """คอลัมน์องศาไว้อ่านด้วยตาเปล่า ค่าที่ยังไม่มาก็เว้นว่างเหมือนคอลัมน์ rad"""
        if radians is None:
            return None
        return math.degrees(radians)

    def steering_error_deg(self):
        """มุมที่สั่งจริงจาก bridge เทียบกับมุมที่ KEYA ทำได้ — ใช้ดู lag/ค้างของพวงมาลัย"""
        if self.steering_cmd_bridge is None or self.steering_angle_actual is None:
            return None
        return math.degrees(self.steering_cmd_bridge - self.steering_angle_actual)

    def has_core_data(self):
        """ข้อมูลขั้นต่ำที่ทำให้แถวหนึ่งมีความหมาย — ตำแหน่ง ความเร็ว และทิศหัวรถ

        ความเร็วนับว่ามีแล้วถ้าได้จาก encoder หรือจาก pose ทางใดทางหนึ่ง
        จึงยังเก็บได้ทั้งตอนวิ่งจริงและตอนไม่มี CAN
        """
        return (
            self.vehicle_x is not None and
            self.vehicle_y is not None and
            self.current_velocity() is not None and
            self.yaw is not None
        )

    def optional_missing_topics(self):
        pending = []
        if self.target_x is None or self.target_y is None:
            pending.append("/target_waypoint")
        if self.adaptive_ld is None:
            pending.append("/adaptive_ld")
        if self.controller_velocity is None:
            pending.append("/velocity")
        if self.curvature is None:
            pending.append("/curvature")
        if (
            self.debug_cte is None or
            self.debug_heading_error is None or
            self.debug_path_heading is None or
            self.debug_lookahead_distance is None or
            self.debug_adaptive_ld is None
        ):
            pending.append("/controller_debug/*")
        if self.cmd_speed is None or self.cmd_steer is None:
            pending.append(f"{self.cmd_vel_topic} หรือ {self.dry_run_cmd_vel_topic}")
        if self.speed_status is None:
            pending.append("/speed_status")
        if self.steering_cmd_bridge is None:
            pending.append("/steering_cmd")
        if self.steering_angle_actual is None:
            pending.append("/steering_angle")
        if self.steering_fault is None:
            pending.append("/steering_fault")
        if self.remaining_distance is None:
            pending.append("/remaining_distance")
        if self.closest_waypoint_index is None:
            pending.append("/closest_waypoint_index")
        if self.target_waypoint_index is None:
            pending.append("/target_waypoint_index")
        return pending

    def log_optional_gaps_once(self):
        """บอกครั้งเดียวตอนเริ่มเขียนจริง ว่าคอลัมน์ไหนจะว่างเพราะยังไม่มีคนส่ง"""
        if self.logged_optional_gaps:
            return
        self.logged_optional_gaps = True

        pending = self.optional_missing_topics()
        if pending:
            self.get_logger().warn(
                "เริ่มเก็บ CSV แล้ว แต่คอลัมน์เหล่านี้จะว่างจนกว่าจะมีข้อมูล: "
                + ", ".join(pending)
            )
        else:
            self.get_logger().info("เริ่มเก็บ CSV แล้ว ข้อมูลครบทุก topic")

    def has_all_data(self):
        return (
            self.vehicle_x is not None and
            self.vehicle_y is not None and
            self.target_x is not None and
            self.target_y is not None and
            self.velocity_pose is not None and
            self.yaw is not None and
            self.adaptive_ld is not None and
            self.controller_velocity is not None and
            self.debug_cte is not None and
            self.debug_heading_error is not None and
            self.debug_path_heading is not None and
            self.debug_lookahead_distance is not None and
            self.debug_adaptive_ld is not None and
            self.curvature is not None and
            self.cmd_speed is not None and
            self.cmd_steer is not None and
            self.steering_cmd_bridge is not None and
            self.steering_angle_actual is not None and
            self.steering_fault is not None and
            self.speed_status is not None and
            self.remaining_distance is not None and
            self.closest_waypoint_index is not None and
            self.target_waypoint_index is not None
        )

    def print_missing_data(self):
        missing = []

        if self.vehicle_x is None or self.vehicle_y is None:
            missing.append("/current_pose หรือ /pcl_pose")

        if self.current_velocity() is None:
            missing.append("/speed_status หรือ /pcl_pose")

        if self.yaw is None:
            missing.append("/filter/euler หรือ yaw จาก /current_pose หรือ /pcl_pose")

        # ที่เหลือบล็อกการเก็บเฉพาะตอน require_all_topics เท่านั้น
        if self.require_all_topics:
            missing.extend(self.optional_missing_topics())

        if missing:
            self.get_logger().warn(
                "Waiting for data: " + ", ".join(missing),
                throttle_duration_sec=2.0
            )

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f"CSV saved: {self.csv_filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CSVLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()