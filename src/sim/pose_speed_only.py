#!/usr/bin/env python3
"""ส่ง "เฉพาะความเร็ว" ตามตำแหน่งของรถ โดยอ่านโปรไฟล์จาก CSV

รถอยู่ตรงไหน -> หา waypoint ที่ใกล้ที่สุดใน CSV -> ส่ง v_target ของจุดนั้นออก /cmd_vel
เป็นรุ่นตัดพวงมาลัยออกทั้งหมดของ pose_speed_player.py: ไม่คำนวณ pure pursuit
ไม่อ่าน yaw ไม่ยืม angular.z จากใคร

angular.z จะเป็น 0 เสมอ และรถคันนี้ตั้ง cmd_vel_steering_mode: steering_angle ไว้
(golfcart_low_level.yaml) ค่า angular.z จึงถูกส่งเข้าชุดพวงมาลัยตรง ๆ
แปลว่าไฟล์นี้ = "สั่งความเร็วพร้อมบังคับพวงมาลัยให้ตรง" ไม่ใช่ "ปล่อยพวงมาลัยลอย"
เหมาะกับทดสอบคันเร่ง/เบรกบนทางตรง หรือเทียบคำสั่งกับ /speed_status
ถ้าต้องให้รถเลี้ยวตามเส้นทางด้วย ใช้ pose_speed_player.py แทน

Run:
    python3 pose_speed_only.py

สำคัญ: อย่ารัน pagard_adaptive_fixed.py หรือ player ตัวอื่นพร้อมกันบน /cmd_vel เส้นเดียวกัน
สอง publisher จะแย่งกัน (ros_can_bridge ใช้ข้อความล่าสุดที่เข้ามา) รถจะกระตุกหรือไม่ออกตัวเลย
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


class PoseSpeedOnly(Node):

    def __init__(self):
        super().__init__('pose_speed_only')

        # ---------- parameters ----------
        self.declare_parameter('csv_path', os.path.join(SCRIPT_DIR, 'loop1_qp_comfort.csv'))
        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed_column', 'v_target')
        self.declare_parameter('publish_rate_hz', 20.0)

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

        # ---------- load CSV ----------
        self.xs, self.ys, self.speeds = self.load_profile(self.csv_path)

        self.current_x = None
        self.current_y = None
        self.last_pose_ns = None
        self.last_index = None
        self.prev_cmd_out = 0.0
        self.last_stop_reason = None
        self.estop_active = False
        self.fault_active = False

        # ---------- ROS I/O ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.index_pub = self.create_publisher(Int32, '/pose_speed_only/index', 10)
        self.target_pub = self.create_publisher(Float64, '/pose_speed_only/target_speed', 10)
        self.offtrack_pub = self.create_publisher(Float64, '/pose_speed_only/offtrack', 10)

        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
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
        self.get_logger().info('Speed only: angular.z = 0 (พวงมาลัยถูกสั่งให้ตรงตลอด)')

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

        self.current_x = x
        self.current_y = y
        self.last_pose_ns = self.get_clock().now().nanoseconds

    def estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def fault_callback(self, msg: Bool):
        self.fault_active = bool(msg.data)

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

        self.publish_speed(target)

    def rate_limit(self, target):
        delta = target - self.prev_cmd_out
        if delta >= 0.0:
            max_delta = self.max_accel * self.control_period
        else:
            max_delta = self.max_decel * self.control_period
        delta = max(-max_delta, min(delta, max_delta))
        self.prev_cmd_out += delta
        return self.prev_cmd_out

    def publish_speed(self, target, reason=None, hold_ramp=False):
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

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def stop_now(self):
        """สั่ง 0 แบบไม่ผ่าน ramp ใช้ตอนปิดโปรแกรม"""
        self.prev_cmd_out = 0.0
        # โดน SIGTERM (launch file สั่งปิด/kill) context จะถูกปิดไปก่อนถึงตรงนี้
        # publish ตอนนั้นจะ raise แล้วกลบ traceback จริง จึงเช็คก่อน
        if not self.context.ok():
            return
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseSpeedOnly()
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
