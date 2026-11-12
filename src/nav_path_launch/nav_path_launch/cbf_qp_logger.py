#!/usr/bin/env python3
"""
cbf_qp_logger.py
================
เก็บผลการวิ่งของสาย CBF-QP ลง CSV ไฟล์เดียว — หนึ่งแถวต่อรอบ (ค่าเริ่มต้น 20 Hz)

รวมสามส่วนไว้ในแถวเดียวกัน:
  - สถานะรถ/เส้นทางจาก pagard      ตำแหน่ง, cte, heading_error, ระยะมองที่ใช้จริง
  - คำตอบและข้อจำกัดจาก cbf_qp_node  v*, L_d*, h₁, ψ̇₁+α₁ψ₁, h₂, feasible ฯลฯ
  - คำสั่งเทียบกับค่าจริงจาก CAN      cmd_vel, /speed_status, /steering_angle, fault
  + ค่าพารามิเตอร์ของ /cbf_qp_solver ที่ใช้ในรอบนั้น (อ่านซ้ำทุก 1 วินาที)

ไฟล์ออกที่ ~/controller_logs/<ปี-เดือน-วัน>/cbf_qp_run_<วันเวลา>.csv (โฟลเดอร์แยกตามวันที่เริ่มรัน)
เริ่มเขียนเมื่อมีตำแหน่งรถแล้ว; topic ไหนยังไม่มา/เงียบเกิน stale_sec ช่องนั้นเว้นว่าง
(ไม่เอาค่าค้างมาเขียนซ้ำ) กด Ctrl-C เพื่อปิดไฟล์

Run:
    python3 cbf_qp_logger.py        หรือมากับ  ros2 launch nav_path_launch cbf_qp.launch.py

หมายเหตุเครื่องหมาย: cte / heading_error เก็บตามที่ pagard ส่ง (cte + = รถอยู่ขวาของ path)
เหมือน controller_log เดิม — e_lat ของสไลด์ = −cte
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, Int32

# คอลัมน์ -> topic (Float64 ทั้งหมด)
FLOAT_TOPICS = {
    'speed_actual': '/speed_status',
    'v_des': '/desired_speed',
    'v_star': '/adaptive_speed',
    'cte': '/controller_debug/cte',
    'heading_error': '/controller_debug/heading_error',
    'kappa_path': '/cbf_qp/kappa_path',
    'kappa_hat': '/cbf_qp/kappa_hat',
    'ld_star': '/adaptive_ld',
    'ld_ref': '/cbf_qp/ld_ref',
    'ld_ref_raw': '/cbf_qp/ld_ref_raw',
    'lookahead_applied': '/controller_debug/lookahead_distance',
    'delta_qp': '/cbf_qp/delta',
    'steering_actual': '/steering_angle',
    'a_y_qp': '/cbf_qp/a_y',
    'h1': '/cbf_qp/h1',
    'psi1': '/cbf_qp/psi1',
    'hocbf': '/cbf_qp/hocbf',
    'h2': '/cbf_qp/h2',
    'v_limit': '/cbf_qp/v_limit',
    'v_curve': '/cbf_qp/v_curve',
    'h2_curve': '/cbf_qp/h2_curve',
    'speed_cap': '/cbf_qp/speed_cap',
    'j': '/cbf_qp/j',
}
INT_TOPICS = {
    'closest_index': '/closest_waypoint_index',
    'target_index': '/target_waypoint_index',
}
# สถานะแบบ Bool — บางตัวส่งเฉพาะตอนเปลี่ยน จึงถือค่าล่าสุดไว้ ไม่ตัดทิ้งตามเวลา
BOOL_TOPICS = {
    'feasible': '/cbf_qp/feasible',
    'steering_fault': '/steering_fault',
    'e_stop': '/e_stop_status',
    'lowlevel_fault': '/lowlevel_fault',
}
SOLVER_PARAMS = (
    'w_v', 'w_ld', 'e_max', 'ay_max', 'alpha2', 'curve_decel', 'tau', 'k_adapt', 'ld_min', 'ld_max',
    # สวิตช์ของการทดลอง (0/1) และ L_d ของโหมด Pure Pursuit ล้วน
    'use_h1', 'use_h2', 'pure_pursuit_only', 'fixed_ld',
)

COLUMNS = (
    'time_sec', 'x', 'y', 'yaw', 'closest_index', 'target_index', 'remaining_distance',
    'speed_actual', 'v_des', 'v_star', 'cmd_speed',
    'cte', 'heading_error', 'kappa_path', 'kappa_hat',
    'ld_star', 'ld_ref', 'ld_ref_raw', 'lookahead_applied',
    'delta_qp', 'cmd_steer', 'steering_actual',
    'a_y_qp', 'a_y_actual',
    'h1', 'psi1', 'hocbf', 'h2', 'v_limit', 'v_curve', 'h2_curve', 'speed_cap', 'j', 'feasible',
    'steering_fault', 'e_stop', 'lowlevel_fault', 'cmd_vel_source',
) + SOLVER_PARAMS


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CbfQpLogger(Node):

    def __init__(self):
        super().__init__('cbf_qp_logger')

        self.declare_parameter('log_rate_hz', 20.0)
        self.declare_parameter('stale_sec', 1.0)             # เงียบเกินนี้ = เว้นช่องว่าง
        self.declare_parameter('wheelbase', 1.57)            # ใช้คิด a_y_actual
        self.declare_parameter('solver_node', '/cbf_qp_solver')
        # ตอน pagard อยู่โหมด dry_run คำสั่งออกที่ /cmd_vel_preview — ฟังทั้งสองเส้น
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('dry_run_cmd_vel_topic', '/cmd_vel_preview')
        self.declare_parameter(
            'csv_path',
            os.path.expanduser(
                datetime.now().strftime(
                    '~/controller_logs/%Y-%m-%d/cbf_qp_run_%Y%m%d_%H%M%S.csv'
                )
            )
        )

        log_rate_hz = min(max(float(self.get_parameter('log_rate_hz').value), 1.0), 100.0)
        self.stale_sec = max(float(self.get_parameter('stale_sec').value), 0.05)
        self.wheelbase = max(float(self.get_parameter('wheelbase').value), 1e-3)

        self.values = {}          # คอลัมน์ -> (ค่า, เวลาที่รับ)
        self.flags = {}           # คอลัมน์ Bool -> ค่าล่าสุด
        self.solver_params = {}
        self.cmd_vel_source = None
        self.last_current_pose_sec = None
        self.start_sec = None
        self.rows = 0
        self.gaps_reported = False
        self.last_wait_warn_sec = None

        for column, topic in FLOAT_TOPICS.items():
            self.create_subscription(
                Float64, topic, lambda msg, c=column: self.store(c, float(msg.data)), 10)
        for column, topic in INT_TOPICS.items():
            self.create_subscription(
                Int32, topic, lambda msg, c=column: self.store(c, int(msg.data)), 10)
        for column, topic in BOOL_TOPICS.items():
            self.create_subscription(
                Bool, topic, lambda msg, c=column: self.flags.__setitem__(c, int(msg.data)), 10)
        self.create_subscription(
            Float32, '/remaining_distance',
            lambda msg: self.store('remaining_distance', float(msg.data)), 10)
        self.create_subscription(PoseStamped, '/current_pose', self.current_pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', self.pcl_pose_cb, 10)
        for topic in dict.fromkeys((
            str(self.get_parameter('cmd_vel_topic').value),
            str(self.get_parameter('dry_run_cmd_vel_topic').value),
        )):
            self.create_subscription(
                Twist, topic, lambda msg, t=topic: self.cmd_vel_cb(msg, t), 10)

        solver_node = str(self.get_parameter('solver_node').value).rstrip('/')
        self.param_client = self.create_client(GetParameters, f'{solver_node}/get_parameters')
        self.param_request_pending = False

        self.csv_path = os.path.expanduser(str(self.get_parameter('csv_path').value))
        log_dir = os.path.dirname(self.csv_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(COLUMNS)

        self.create_timer(1.0 / log_rate_hz, self.write_row)
        self.create_timer(1.0, self.request_solver_params)
        self.get_logger().info(f'CSV logger started @ {log_rate_hz:.0f} Hz: {self.csv_path}')

    # ==============================================================
    #  Callbacks
    # ==============================================================
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def store(self, column, value):
        if isinstance(value, float) and not math.isfinite(value):
            return
        self.values[column] = (value, self.now_sec())

    def fresh(self, column, now):
        entry = self.values.get(column)
        if entry is None or (now - entry[1]) > self.stale_sec:
            return None
        return entry[0]

    def store_pose(self, position, orientation):
        self.store('x', float(position.x))
        self.store('y', float(position.y))
        self.store('yaw', quaternion_to_yaw(orientation))

    def current_pose_cb(self, msg):
        self.last_current_pose_sec = self.now_sec()
        self.store_pose(msg.pose.position, msg.pose.orientation)

    def pcl_pose_cb(self, msg):
        # /current_pose มาจาก pagard — ถ้า pagard ไม่ได้รัน ใช้ผล NDT ตรง ๆ แทน
        last = self.last_current_pose_sec
        if last is None or (self.now_sec() - last) > self.stale_sec:
            self.store_pose(msg.pose.pose.position, msg.pose.pose.orientation)

    def cmd_vel_cb(self, msg, topic):
        self.store('cmd_speed', float(msg.linear.x))
        self.store('cmd_steer', float(msg.angular.z))
        if self.cmd_vel_source != topic:
            self.cmd_vel_source = topic
            self.get_logger().info(f'cmd_vel source: {topic}')

    def request_solver_params(self):
        if self.param_request_pending or not self.param_client.service_is_ready():
            return
        self.param_request_pending = True
        future = self.param_client.call_async(GetParameters.Request(names=list(SOLVER_PARAMS)))
        future.add_done_callback(self.on_solver_params)

    def on_solver_params(self, future):
        self.param_request_pending = False
        try:
            values = future.result().values
        except Exception:  # solver ปิดไประหว่างรอคำตอบ
            return
        for name, value in zip(SOLVER_PARAMS, values):
            # type 1 = bool, 2 = integer, 3 = double (ตั้งผ่าน ros2 param set ด้วยจำนวนเต็มได้)
            if value.type == 1:
                self.solver_params[name] = int(value.bool_value)
            elif value.type == 3:
                self.solver_params[name] = value.double_value
            elif value.type == 2:
                self.solver_params[name] = float(value.integer_value)

    # ==============================================================
    #  เขียน CSV
    # ==============================================================
    def write_row(self):
        now = self.now_sec()
        row = {column: self.fresh(column, now) for column in self.values}
        if row.get('x') is None or row.get('y') is None:
            if self.last_wait_warn_sec is None or now - self.last_wait_warn_sec > 5.0:
                self.last_wait_warn_sec = now
                self.get_logger().warn('รอตำแหน่งรถจาก /current_pose หรือ /pcl_pose ...')
            return
        if self.start_sec is None:
            self.start_sec = now

        row['time_sec'] = now - self.start_sec
        row.update(self.flags)
        row.update(self.solver_params)
        row['cmd_vel_source'] = self.cmd_vel_source
        # แรงเหวี่ยงที่เกิดจริง = v²·|tanδ|/L จากความเร็วและมุมล้อที่วัดได้
        speed, steer = row.get('speed_actual'), row.get('steering_actual')
        if speed is not None and steer is not None:
            row['a_y_actual'] = speed * speed * abs(math.tan(steer)) / self.wheelbase

        self.csv_writer.writerow([self.format(row.get(column)) for column in COLUMNS])
        self.rows += 1
        if self.rows % 20 == 0:
            self.csv_file.flush()
        if not self.gaps_reported and row['time_sec'] >= 2.0:
            self.report_gaps(row)

    @staticmethod
    def format(value):
        if value is None:
            return ''
        if isinstance(value, float):
            return f'{value:.5f}'
        return value

    def report_gaps(self, row):
        """บอกครั้งเดียวหลังเริ่มเก็บ 2 วินาที ว่าคอลัมน์ไหนยังว่างอยู่"""
        self.gaps_reported = True
        # speed_cap ว่าง = ไม่มีเพดาน (∞, h₂ ปิดหรือ QP คุมความเร็วเอง) ไม่ใช่ข้อมูลขาด
        empty = [c for c in COLUMNS if row.get(c) is None and c not in ('a_y_actual', 'speed_cap')]
        if empty:
            self.get_logger().warn(
                'เก็บ CSV อยู่ แต่คอลัมน์เหล่านี้ยังว่าง (ไม่มีข้อมูลเข้า): ' + ', '.join(empty)
            )
        else:
            self.get_logger().info('เก็บ CSV อยู่ ข้อมูลครบทุกคอลัมน์')

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.get_logger().info(f'CSV saved: {self.csv_path} ({self.rows} แถว)')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CbfQpLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C ใต้ ros2 launch มาสองครั้ง (จาก terminal และจาก launch) ครั้งที่สองมักตกตรงนี้
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
