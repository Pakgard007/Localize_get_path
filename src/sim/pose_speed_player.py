#!/usr/bin/env python3
"""ส่งความเร็วตาม "ตำแหน่ง" ของรถ โดยอ่านโปรไฟล์จาก CSV

รถอยู่ตรงไหน -> หา waypoint ที่ใกล้ที่สุดใน CSV -> ส่ง v_target ของจุดนั้นออก /cmd_vel
ต่างจาก csv_cmdvel_player.py ที่ไล่ตาม "เวลา" (t_s) อันนี้ไล่ตามระยะ/ตำแหน่งจริง
รถช้ากว่าโปรไฟล์ก็ไม่หลุด เพราะ index เดินตามรถ ไม่ใช่ตามนาฬิกา

Run:
    python3 pose_speed_player.py

สำคัญ: อย่ารัน pagard_adaptive_fixed.py พร้อมกันบน /cmd_vel เส้นเดียวกัน
สอง publisher จะแย่งกัน (ros_can_bridge ใช้ข้อความล่าสุดที่เข้ามา) รถจะกระตุกหรือไม่ออกตัวเลย

พวงมาลัยคำนวณเองด้วย pure pursuit จาก waypoint ชุดเดียวกับที่ใช้หาความเร็ว
ค่าที่ส่งออก angular.z คือ "มุมเลี้ยวล้อ (rad)" ไม่ใช่ yaw rate เพราะ config ของรถคันนี้
ตั้ง cmd_vel_steering_mode: steering_angle ไว้ (golfcart_low_level.yaml) ros_can_bridge
จึงส่งค่านี้ต่อให้ชุดพวงมาลัยตรง ๆ แล้ว clamp ที่ max_steering_deg = 50 องศา

ถ้าอยากให้ pagard คุมพวงมาลัยแทน:
    รัน pagard ด้วย dry_run:=true (Twist ทั้งก้อนไปออก /cmd_vel_preview แทน /cmd_vel)
    แล้วรันไฟล์นี้ด้วย -p steer_source_topic:=/cmd_vel_preview
ไฟล์นี้จะหยิบ angular.z ของ pagard มาต่อให้ ส่วน linear.x ใช้ของตัวเอง
"""

import csv
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class PoseSpeedPlayer(Node):

    def __init__(self):
        super().__init__('pose_speed_player')

        # ---------- parameters ----------
        self.declare_parameter('csv_path', os.path.join(SCRIPT_DIR, 'loop1_qp_comfort.csv'))
        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed_column', 'v_target')
        # '' = คำนวณพวงมาลัยเอง. ตั้งเป็น /cmd_vel_preview เพื่อยืม angular.z จาก pagard dry_run แทน
        self.declare_parameter('steer_source_topic', '')
        self.declare_parameter('steer_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        # ---------- pure pursuit ----------
        self.declare_parameter('wheelbase', 1.57)  # ค่าเดียวกับที่ pagard ใช้คำนวณ pure pursuit
        # lookahead ยิ่งไกลยิ่งนิ่งแต่ตัดโค้ง ยิ่งใกล้ยิ่งเกาะเส้นแต่ส่าย
        # โตตามความเร็ว: Ld = v * gain แล้ว clamp
        self.declare_parameter('lookahead_gain', 1.6)
        self.declare_parameter('min_lookahead_m', 3.0)
        self.declare_parameter('max_lookahead_m', 8.0)
        self.declare_parameter('max_steer_rad', 0.8)  # ros_can_bridge clamp ที่ 50 องศา (0.873) อยู่แล้ว
        self.declare_parameter('steer_slew_rad_per_s', 0.8)  # กันคำสั่งกระชากเมื่อ pose กระโดด
        self.declare_parameter('invert_steer_sign', False)  # กลับซ้าย/ขวาถ้าชุดขับตีความตรงข้าม
        # เป้า lookahead อยู่หลังรถเกินมุมนี้ = หันผิดทาง/localization พลิก ให้หยุดดีกว่าเลี้ยวมั่ว
        self.declare_parameter('max_heading_error_rad', 1.57)

        # เส้นทาง loop2 วนกลับมาทับตัวเอง การค้นหา waypoint ใกล้สุดทั้งไฟล์จึงเสี่ยง
        # กระโดดข้ามไปอีกฝั่งของ loop ปกติจึงค้นเฉพาะหน้าต่างรอบ index เดิม
        self.declare_parameter('search_back_points', 20)
        self.declare_parameter('search_forward_points', 120)
        self.declare_parameter('loop_path', True)  # ถึงจุดสุดท้ายแล้ววนกลับจุดแรก

        # ---------- safety ----------
        self.declare_parameter('max_speed', 4.0)  # เพดานคำสั่งความเร็ว (m/s)
        self.declare_parameter('speed_scale', 1.0)
        # ramp: โปรไฟล์อิงตำแหน่ง ถ้ารถไปจอดกลางเส้นทางที่ CSV บันทึกไว้ 3.5 m/s
        # คำสั่งจะกระโดดจาก 0 ไป 3.5 ทันที ซึ่งรถทำตามไม่ทันและไปทริป command check
        # ของ pagard ได้ จึงจำกัดอัตราเร่ง/หน่วงของ "คำสั่ง" ไว้เท่าที่รถทำได้จริง
        self.declare_parameter('max_accel_mps2', 0.9)
        self.declare_parameter('max_decel_mps2', 1.2)
        # e-stop / fault ของ low level: ระหว่างที่ค้างอยู่ ห้ามให้ ramp ไต่ขึ้นเงียบ ๆ
        # ไม่งั้นตอนปลด e-stop รถจะโดนคำสั่งเต็มพิกัดทันทีโดยไม่มีการไล่ความเร็ว
        self.declare_parameter('estop_topic', '/e_stop_status')
        self.declare_parameter('fault_topic', '/lowlevel_fault')
        # 0.80 เท่ากับ pose_stale_timeout_sec ที่ pagard จูนไว้: NDT จริงมี jitter ถึง ~0.7s
        # เป็นปกติ ตั้ง 0.5 จะขึ้น pose_stale รัว ๆ ทั้งที่ localization ยังทำงานอยู่
        self.declare_parameter('pose_timeout_sec', 0.8)  # pose ไม่สดเกินนี้ = สั่งหยุด
        self.declare_parameter('max_offtrack_m', 5.0)  # ห่างเส้นทางเกินนี้ = สั่งหยุด

        self.csv_path = str(self.get_parameter('csv_path').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.speed_column = str(self.get_parameter('speed_column').value)
        self.steer_source_topic = str(self.get_parameter('steer_source_topic').value).strip()
        self.steer_timeout_sec = max(0.0, float(self.get_parameter('steer_timeout_sec').value))
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.control_period = 1.0 / rate

        self.search_back = max(0, int(self.get_parameter('search_back_points').value))
        self.search_forward = max(1, int(self.get_parameter('search_forward_points').value))
        self.loop_path = bool(self.get_parameter('loop_path').value)

        self.max_speed = max(0.0, float(self.get_parameter('max_speed').value))
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_accel = max(0.0, float(self.get_parameter('max_accel_mps2').value))
        self.max_decel = max(0.0, float(self.get_parameter('max_decel_mps2').value))
        self.estop_topic = str(self.get_parameter('estop_topic').value).strip()
        self.fault_topic = str(self.get_parameter('fault_topic').value).strip()
        self.pose_timeout_sec = max(0.0, float(self.get_parameter('pose_timeout_sec').value))
        self.max_offtrack_m = max(0.0, float(self.get_parameter('max_offtrack_m').value))

        self.wheelbase = max(0.1, float(self.get_parameter('wheelbase').value))
        self.lookahead_gain = max(0.0, float(self.get_parameter('lookahead_gain').value))
        self.min_lookahead = max(0.1, float(self.get_parameter('min_lookahead_m').value))
        self.max_lookahead = max(self.min_lookahead, float(self.get_parameter('max_lookahead_m').value))
        self.max_steer_rad = max(0.0, float(self.get_parameter('max_steer_rad').value))
        self.steer_slew = max(0.0, float(self.get_parameter('steer_slew_rad_per_s').value))
        self.invert_steer_sign = bool(self.get_parameter('invert_steer_sign').value)
        self.max_heading_error = max(0.0, float(self.get_parameter('max_heading_error_rad').value))

        # ---------- load CSV ----------
        self.xs, self.ys, self.speeds = self.load_profile(self.csv_path)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.last_pose_ns = None
        self.last_index = None
        self.prev_cmd_out = 0.0
        self.last_stop_reason = None
        self.steer_value = 0.0
        self.last_steer_ns = None
        self.steer_cmd_out = 0.0  # มุมล่าสุดที่ส่งออกไปแล้ว ใช้เป็นฐานของ slew limit
        self.estop_active = False
        self.fault_active = False

        # ---------- ROS I/O ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.index_pub = self.create_publisher(Int32, '/pose_speed_player/index', 10)
        self.target_pub = self.create_publisher(Float64, '/pose_speed_player/target_speed', 10)
        self.offtrack_pub = self.create_publisher(Float64, '/pose_speed_player/offtrack', 10)
        self.steer_pub = self.create_publisher(Float64, '/pose_speed_player/steer', 10)
        self.heading_error_pub = self.create_publisher(
            Float64, '/pose_speed_player/heading_error', 10
        )

        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )

        if self.steer_source_topic:
            self.create_subscription(
                Twist, self.steer_source_topic, self.steer_callback, 10
            )

        if self.estop_topic:
            self.create_subscription(Bool, self.estop_topic, self.estop_callback, 10)
        if self.fault_topic:
            self.create_subscription(Bool, self.fault_topic, self.fault_callback, 10)

        self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            f'Profile: {self.csv_path} ({len(self.xs)} points, '
            f'max {max(self.speeds):.2f} m/s)'
        )
        self.get_logger().info(
            f'{self.pose_topic} -> {self.cmd_vel_topic} @ {rate:.1f} Hz | '
            f'max_speed={self.max_speed:.2f} scale={self.speed_scale:.2f} '
            f'accel={self.max_accel:.2f} decel={self.max_decel:.2f} '
            f'loop={self.loop_path}'
        )
        if self.steer_source_topic:
            self.get_logger().info(
                f'Steering (angular.z) forwarded from {self.steer_source_topic}'
            )
        else:
            self.get_logger().info(
                f'Steering: pure pursuit | wheelbase={self.wheelbase:.2f} m '
                f'Ld={self.min_lookahead:.1f}-{self.max_lookahead:.1f} m '
                f'(gain {self.lookahead_gain:.2f}) '
                f'max_steer={math.degrees(self.max_steer_rad):.1f} deg '
                f'slew={math.degrees(self.steer_slew):.1f} deg/s '
                f'invert={self.invert_steer_sign}'
            )
            self.get_logger().info(
                'angular.z = มุมเลี้ยวล้อ (rad) ตาม cmd_vel_steering_mode: steering_angle'
            )

    # =========================
    # CSV
    # =========================
    def load_profile(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

        xs, ys, speeds = [], [], []

        with open(path, newline='') as f:
            reader = csv.DictReader(f)

            for column in ('x', 'y', self.speed_column):
                if column not in reader.fieldnames:
                    raise ValueError(
                        f'Column "{column}" not found in {path}. '
                        f'Available: {reader.fieldnames}'
                    )

            for row in reader:
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    v = float(row[self.speed_column])
                except (TypeError, ValueError):
                    continue  # แถวเสีย/ว่าง ข้ามไป ไม่ต้องล้มทั้งไฟล์

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(v)):
                    continue

                xs.append(x)
                ys.append(y)
                speeds.append(max(v, 0.0))

        if len(xs) < 2:
            raise ValueError(f'Need at least 2 valid rows in {path}')

        return xs, ys, speeds

    # =========================
    # Pose
    # =========================
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        if not (math.isfinite(x) and math.isfinite(y)):
            return

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if not math.isfinite(yaw):
            return

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.last_pose_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def fault_callback(self, msg: Bool):
        self.fault_active = bool(msg.data)

    def steer_callback(self, msg: Twist):
        value = float(msg.angular.z)
        if not math.isfinite(value):
            return
        self.steer_value = value
        self.last_steer_ns = self.get_clock().now().nanoseconds

    def steer_is_fresh(self, now_ns):
        if not self.steer_source_topic:
            return True  # ไม่ได้ใช้แหล่งพวงมาลัย ก็ไม่มีอะไรให้หมดอายุ
        if self.last_steer_ns is None:
            return False
        return (now_ns - self.last_steer_ns) * 1e-9 <= self.steer_timeout_sec

    def find_closest_index(self, x, y):
        """หา waypoint ใกล้ที่สุด: รอบแรกค้นทั้งไฟล์ รอบต่อไปค้นเฉพาะหน้าต่างรอบ index เดิม"""
        count = len(self.xs)

        if self.last_index is None:
            candidates = range(count)
        else:
            start = self.last_index - self.search_back
            stop = self.last_index + self.search_forward + 1
            if self.loop_path:
                candidates = (i % count for i in range(start, stop))
            else:
                candidates = range(max(0, start), min(count, stop))

        best_index = None
        best_dist_sq = None

        for i in candidates:
            dx = self.xs[i] - x
            dy = self.ys[i] - y
            dist_sq = dx * dx + dy * dy
            if best_dist_sq is None or dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_index = i

        return best_index, math.sqrt(best_dist_sq)

    # =========================
    # Steering (pure pursuit)
    # =========================
    def lookahead_distance(self, speed):
        """Ld โตตามความเร็ว: ช้า = มองใกล้ เกาะเส้น / เร็ว = มองไกล ไม่ส่าย"""
        return min(
            self.max_lookahead,
            max(self.min_lookahead, speed * self.lookahead_gain)
        )

    def find_lookahead_point(self, start_index, lookahead):
        """เดินไปข้างหน้าตาม waypoint จนระยะสะสมถึง Ld แล้วคืนจุดนั้น"""
        count = len(self.xs)
        travelled = 0.0
        i = start_index

        for _ in range(count - 1):
            j = i + 1
            if j >= count:
                if not self.loop_path:
                    break  # สุดเส้นทางแล้ว ใช้จุดสุดท้ายเป็นเป้า
                j = 0

            travelled += math.hypot(self.xs[j] - self.xs[i], self.ys[j] - self.ys[i])
            i = j

            if travelled >= lookahead:
                break

        return self.xs[i], self.ys[i]

    def compute_steer(self, index, speed):
        """คืน (มุมเลี้ยว rad, heading error rad) — heading error ใช้เป็น guard ด้านนอก"""
        lookahead = self.lookahead_distance(speed)
        tx, ty = self.find_lookahead_point(index, lookahead)

        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.hypot(dx, dy)

        if distance < 1e-3:
            # เป้าทับตัวรถพอดี (เส้นทางสั้นมาก/จบแล้ว) คำนวณมุมไม่ได้ ให้คงมุมเดิม
            return self.steer_cmd_out, 0.0

        alpha = self.normalize_angle(math.atan2(dy, dx) - self.current_yaw)

        # bicycle model: delta = atan2(2 L sin(alpha), Ld) โดยใช้ระยะจริงถึงเป้า
        # ไม่ใช่ Ld ที่ขอไว้ เพราะจุดที่เจออาจไกล/ใกล้กว่าที่ขอเล็กน้อย
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        steer = max(-self.max_steer_rad, min(self.max_steer_rad, steer))

        return steer, alpha

    def slew_limit_steer(self, steer):
        if self.steer_slew <= 0.0:
            self.steer_cmd_out = steer
            return steer

        max_delta = self.steer_slew * self.control_period
        delta = max(-max_delta, min(steer - self.steer_cmd_out, max_delta))
        self.steer_cmd_out += delta
        return self.steer_cmd_out

    # =========================
    # Control
    # =========================
    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds

        # e-stop/fault มาก่อนทุกอย่าง: hold_ramp=True บังคับให้คำสั่งกลับไป 0 ทันที
        # แทนที่จะค่อย ๆ ลดตาม decel เพราะ low level ตัดกำลังไปแล้ว การไล่ ramp ลง
        # ไม่มีความหมาย มีแต่จะทำให้ ramp ค้างสูงตอนปลด e-stop
        if self.estop_active:
            self.publish_speed(0.0, reason='e_stop_active', hold_ramp=True)
            return

        if self.fault_active:
            self.publish_speed(0.0, reason='lowlevel_fault', hold_ramp=True)
            return

        if self.current_x is None:
            self.publish_speed(0.0, reason='waiting_for_pose')
            return

        pose_age = (now_ns - self.last_pose_ns) * 1e-9
        if pose_age > self.pose_timeout_sec:
            self.publish_speed(0.0, reason=f'pose_stale ({pose_age:.2f}s)')
            return

        if not self.steer_is_fresh(now_ns):
            # แหล่งพวงมาลัยเงียบ = ไม่รู้ว่าควรเลี้ยวเท่าไร วิ่งต่อไม่ได้
            # steer ค้างมุมเดิมไว้เหมือนที่ ros_can_bridge ทำตอนหยุด ไม่กระชากกลับศูนย์
            self.publish_speed(0.0, reason=f'steer_source_stale ({self.steer_source_topic})')
            return

        index, offtrack = self.find_closest_index(self.current_x, self.current_y)
        self.offtrack_pub.publish(Float64(data=float(offtrack)))

        if offtrack > self.max_offtrack_m:
            # หลุดออกนอกเส้นทางที่บันทึกไว้ ความเร็วของ waypoint นั้นไม่มีความหมายแล้ว
            # ล้าง last_index ด้วย รอบหน้าจะได้ค้นทั้งไฟล์ใหม่: ถ้า pose กระโดดไปไกล
            # (NDT หลุด แล้วกลับมาเกาะใหม่คนละที่) การค้นแบบหน้าต่างจะหาไม่เจอตลอดไป
            # เพราะหน้าต่างยังคาอยู่ที่ index เดิม กลับมาวิ่งต่อไม่ได้เลย
            self.last_index = None
            self.publish_speed(0.0, reason=f'offtrack {offtrack:.2f} m')
            return

        # ถึงปลายทางแล้วและไม่ได้ตั้งให้วน = จบ สั่งหยุด
        if (
            not self.loop_path
            and self.last_index is not None
            and index >= len(self.xs) - 1
        ):
            self.last_index = index
            self.publish_speed(0.0, reason='end_of_path')
            return

        self.last_index = index
        self.index_pub.publish(Int32(data=int(index)))

        target = self.speeds[index] * self.speed_scale
        target = max(0.0, min(target, self.max_speed))
        self.target_pub.publish(Float64(data=float(target)))

        if self.steer_source_topic:
            # โหมดยืมพวงมาลัยจากภายนอก: ความสดเช็คไปแล้วด้านบน
            self.publish_speed(target, steer=self.steer_value)
            return

        # Ld อิงความเร็วที่กำลังสั่งอยู่จริง (prev_cmd_out) ไม่ใช่เป้าปลายทาง
        # ตอนออกตัวยังช้าอยู่จึงมองใกล้และเกาะเส้นได้ ไม่ใช่มองไกลแล้วตัดโค้งตั้งแต่แรก
        steer, heading_error = self.compute_steer(index, self.prev_cmd_out)
        self.heading_error_pub.publish(Float64(data=float(heading_error)))

        if abs(heading_error) > self.max_heading_error:
            # เป้าอยู่ด้านหลังรถ: หันผิดทาง หรือ localization พลิก
            # เดินหน้าต่อจะพารถออกนอกเส้นทาง หยุดให้คนมาตัดสินใจดีกว่า
            self.publish_speed(
                0.0,
                reason=f'heading_error {math.degrees(heading_error):+.0f} deg'
            )
            return

        self.publish_speed(target, steer=steer)

    def rate_limit(self, target):
        delta = target - self.prev_cmd_out
        if delta >= 0.0:
            max_delta = self.max_accel * self.control_period
        else:
            max_delta = self.max_decel * self.control_period
        delta = max(-max_delta, min(delta, max_delta))
        self.prev_cmd_out += delta
        return self.prev_cmd_out

    def publish_speed(self, target, reason=None, hold_ramp=False, steer=None):
        """steer=None คือ 'คงมุมเดิม' ใช้ตอนสั่งหยุด

        ที่ไม่คืนพวงมาลัยเป็น 0 ตอนหยุด เพราะ cmd_vel_steering_mode ของรถคันนี้เป็น
        steering_angle ค่า angular.z จึงถูกส่งไปที่ชุดพวงมาลัยตรง ๆ สั่ง 0 ตอนรถจอด
        = สั่งให้หมุนพวงมาลัยกลับมาตรงทันที ซึ่งไม่ใช่สิ่งที่ต้องการเวลา e-stop
        """
        if reason is not None and reason != self.last_stop_reason:
            self.get_logger().warn(f'Commanding 0: {reason}')
        if reason is None and self.last_stop_reason is not None:
            self.get_logger().info('Resuming speed profile')
        self.last_stop_reason = reason

        if hold_ramp:
            # ตรึง ramp ไว้ที่ 0 ระหว่างที่รถถูกตัดกำลัง พอกลับมาพร้อมวิ่ง
            # คำสั่งจะไต่ขึ้นใหม่ตั้งแต่ศูนย์ ไม่กระชากเต็มพิกัด
            self.prev_cmd_out = 0.0
            speed = 0.0
        else:
            speed = self.rate_limit(target)

        if steer is None:
            steer_out = self.steer_cmd_out
        else:
            steer_out = self.slew_limit_steer(float(steer))

        if self.invert_steer_sign:
            steer_out = -steer_out

        self.steer_pub.publish(Float64(data=float(steer_out)))

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = round(float(steer_out), 3)
        self.cmd_pub.publish(cmd)

    def stop_now(self):
        """สั่ง 0 แบบไม่ผ่าน ramp ใช้ตอนปิดโปรแกรม — พวงมาลัยคงมุมเดิม ไม่ดีดกลับตรง"""
        self.prev_cmd_out = 0.0
        # โดน SIGTERM (launch file สั่งปิด/kill) context จะถูกปิดไปก่อนถึงตรงนี้
        # publish ตอนนั้นจะ raise แล้วกลบ traceback จริง จึงเช็คก่อน
        if not self.context.ok():
            return
        cmd = Twist()
        cmd.linear.x = 0.0
        steer_out = -self.steer_cmd_out if self.invert_steer_sign else self.steer_cmd_out
        cmd.angular.z = round(float(steer_out), 3)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseSpeedPlayer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGTERM: rclpy ปิด context ระหว่างที่ spin ยังรออยู่ แล้วโยน RCLError
        # ('context is not valid') ออกมาแทน ExternalShutdownException
        # ถ้า context ยังดีอยู่แปลว่าเป็น error จริง ต้องไม่กลืน
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.stop_now()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
