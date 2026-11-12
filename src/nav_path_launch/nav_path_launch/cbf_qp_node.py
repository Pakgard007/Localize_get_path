#!/usr/bin/env python3
"""
cbf_qp_node.py
==============
ROS2 node — CBF-QP Solver ตามสไลด์ "CBF-QP Tuning Lab" (สมการอยู่ใน cbf_qp_core.py)

ใช้ "แทน" src/sim/adaptive_ld_node_fixed.py — topic เข้า/ออกชุดเดียวกัน ห้ามรันคู่กัน
เพราะทั้งสองตัวส่ง /adaptive_ld กับ /adaptive_speed

สิ่งที่ต่างจาก adaptive_ld_node_fixed.py (ทำให้ตรงสไลด์):
  - h₁ เป็น HOCBF สองชั้น ψ̇₁ + α₁ψ₁ ≥ 0 และ α₁ = v/e_max คำนวณเอง (ไม่มี alpha1 ให้ตั้ง)
  - L_d,ref = L_d,min + τ·v แล้วปรับตามโค้ง (k_adapt) และความสบาย √(0.4/a_y,max)
  - κ̂ มองโค้งล่วงหน้าระยะ max(L_d, v/α₂) ของแต่ละ candidate (คำนวณจาก /nav_path เอง)
  - QP เลือก v กับ L_d พร้อมกัน บนกริด Δv = 0.05 m/s, ΔL_d = 0.1 m
  - ปรับพารามิเตอร์ตอนรันได้:  ros2 param set /cbf_qp_solver tau 1.2

สวิตช์สำหรับการทดลอง (ติ๊กได้ในหน้าต่าง rqt_reconfigure ของ tune:=true):
  use_h1, use_h2      ปิด = ข้อจำกัดนั้นไม่ตัด candidate (ค่ายังออก /cbf_qp/h1, h2 ให้ดู)
  pure_pursuit_only   เปิด = ไม่มี QP: L_d* = fixed_ld คงที่, v* = v_des
                      และ pagard เลี้ยวด้วย δ = atan(2L·sinα / L_d) ล้วน
  fixed_ld            L_d (m) ของโหมดนั้น เปลี่ยนตอนรถวิ่งได้

INPUTS — จาก pagard_adaptive_fixed.py และ nav_path.py:
  /controller_debug/cte            Float64  ระยะเยื้องด้านข้าง (+ = รถอยู่ "ขวา" ของ path)
  /controller_debug/heading_error  Float64  path_heading − yaw
  /velocity                        Float64  ความเร็วจริงของรถ (m/s)
  /closest_waypoint_index          Int32    segment ของ path ที่รถอยู่
  /nav_path                        Path     เส้นทางอ้างอิง (ใช้หา κ_path และจุดเป้าหมาย)
  /desired_speed        (optional) Float64  v_des(s) จากคอลัมน์ v_target ของ CSV

  สไลด์นิยามเครื่องหมายกลับด้านกับ pagard (e_lat > 0 = ซ้าย, θ_e = yaw − path_heading)
  node นี้กลับเครื่องหมายสองค่านี้ตอนรับเข้า แล้วใช้ตามสไลด์ทั้งหมด

OUTPUTS:
  /adaptive_ld     Float64  L_d*
  /adaptive_speed  Float64  v*
  /cbf_qp/feasible Bool     False = QP infeasible (ใช้จุดที่ละเมิดข้อจำกัดน้อยที่สุดอยู่)
  /cbf_qp/pure_pursuit_only Bool  True = โหมด Pure Pursuit ล้วน (pagard ฟังตัวนี้)
  /cbf_qp/{h1,h2,j,kappa_hat,a_y,delta,ld_ref}          (ชื่อเดิม)
  /cbf_qp/{psi1,hocbf,alpha1,v_limit,ld_ref_raw,kappa_path}
  /cbf_qp/h1_corridor MarkerArray  ขอบกรอบ h₁ (|e_lat| = e_max) สองฝั่งของ /nav_path ให้ RViz
                      ฟ้า = h₁ ใช้อยู่, เทา = ปิด h₁ หรือโหมด Pure Pursuit

Run:
    python3 cbf_qp_node.py          หรือ   ros2 launch nav_path_launch cbf_qp.launch.py
"""

import csv
import math
import os
import sys
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA, Float64, Int32
from visualization_msgs.msg import Marker, MarkerArray

# cbf_qp_core.py ที่อยู่ข้างไฟล์นี้ก่อนเสมอ: launch รันไฟล์นี้จาก src ตรง ๆ แต่ถ้า import
# ตามปกติจะได้สำเนาใน install (ซึ่งไม่ตามการแก้ใน src) แทน
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from nav_path_launch import cbf_qp_core as core  # noqa: E402

# พารามิเตอร์ที่ผูกกับ CbfQpParams — ปรับตอนรันได้ทุกตัว (เทียบได้กับสไลเดอร์ของ Tuning Lab)
SOLVER_PARAMS = (
    'wheelbase', 'max_steer_rad', 'v_min', 'v_max', 'v_step', 'ld_min', 'ld_max', 'ld_step',
    'w_v', 'w_ld', 'e_max', 'ay_max', 'alpha2', 'curve_decel', 'tau', 'k_adapt',
    'optimize_speed', 'recovery_v_max',
    'use_h1', 'use_h2', 'pure_pursuit_only', 'fixed_ld',
)
# ช่วง (ต่ำสุด, สูงสุด) ของสไลเดอร์ใน rqt_reconfigure — ค่าที่ตั้งนอกช่วงนี้จะถูกปฏิเสธ
SLIDER_RANGES = {
    'w_v': (0.0, 10.0),
    'w_ld': (0.0, 2.0),
    'e_max': (0.1, 3.0),
    'ay_max': (0.05, 5.0),
    'alpha2': (0.05, 3.0),
    'curve_decel': (0.1, 3.0),
    'tau': (0.0, 5.0),
    'k_adapt': (0.0, 20.0),
    'ld_min': (0.5, 8.0),
    'ld_max': (2.0, 20.0),
    'recovery_v_max': (0.0, 5.0),
    'fixed_ld': (1.0, 12.0),
}
# ค่าที่บันทึกลง CSV ทุกแถว เพื่อรู้ว่ารอบนั้นใช้พารามิเตอร์ชุดไหน
LOGGED_PARAMS = (
    'w_v', 'w_ld', 'e_max', 'ay_max', 'alpha2', 'curve_decel', 'tau', 'k_adapt', 'ld_min', 'ld_max',
    'use_h1', 'use_h2', 'pure_pursuit_only', 'fixed_ld',
)
LOGGED_STATE = ('e_lat', 'theta_e', 'v', 'kappa_path', 'v_des')
LOGGED_SOLUTION = (
    'v_star', 'ld_star', 'j', 'feasible', 'feasible_count', 'ld_ref_raw', 'ld_ref',
    'eta', 'delta', 'kappa_cmd', 'kappa_hat', 'a_y', 'h2', 'v_limit',
    'v_curve', 'h2_curve', 'speed_cap',
    'alpha1', 'h1', 'h1_dot', 'psi1', 'hocbf',
)
DIAGNOSTIC_TOPICS = (
    'h1', 'h2', 'j', 'kappa_hat', 'a_y', 'delta', 'ld_ref',
    'psi1', 'hocbf', 'alpha1', 'v_limit', 'ld_ref_raw',
    'v_curve', 'h2_curve', 'speed_cap',
)


def mode_text(p):
    """โหมดของตัวแก้ในหนึ่งบรรทัด สำหรับ log ตอนเริ่มและตอนสลับ"""
    if p.pure_pursuit_only:
        return f'Pure Pursuit ล้วน: L_d = {p.fixed_ld:.2f} m คงที่, v = v_des, ไม่ใช้ h₁ h₂'
    used = [name for name, on in (('h₁', p.use_h1), ('h₂', p.use_h2)) if on]
    unused = [name for name, on in (('h₁', p.use_h1), ('h₂', p.use_h2)) if not on]
    speed = ''
    if not p.optimize_speed:
        speed = (' | ความเร็วจากไฟล์ ชะลอตาม h₂ (+ โค้งข้างหน้า)' if p.use_h2
                 else ' | ความเร็วจากไฟล์ล้วน')
    return (
        'CBF-QP: ' + (' + '.join(used) if used else 'ไม่มีข้อจำกัด (เลือก L_d ใกล้ L_d,ref)')
        + (' | ปิด ' + ', '.join(unused) if unused else '') + speed
    )


def mode_tag(p):
    """ป้ายสั้นหน้าบรรทัด log ของแต่ละรอบ ว่างถ้าเป็นโหมดปกติ (h₁ + h₂)"""
    if p.pure_pursuit_only:
        return f'[PP L_d={p.fixed_ld:.1f}] '
    off = [name for name, on in (('h₁', p.use_h1), ('h₂', p.use_h2)) if not on]
    return f'[ปิด {",".join(off)}] ' if off else ''


class CbfQpSolverNode(Node):

    def __init__(self):
        super().__init__('cbf_qp_solver')

        # =========================================================
        # พารามิเตอร์ของตัวแก้ — ค่าเริ่มต้น = "preset ค่าเริ่มต้น" ในสไลด์
        # =========================================================
        # dynamic_typing: ให้ `ros2 param set ... tau 1` (จำนวนเต็ม) ใช้ได้ ไม่ต้องพิมพ์ 1.0
        defaults = core.CbfQpParams()
        for name in SOLVER_PARAMS:
            descriptor = ParameterDescriptor(dynamic_typing=True)
            if name in SLIDER_RANGES:
                low, high = SLIDER_RANGES[name]
                descriptor.floating_point_range = [
                    FloatingPointRange(from_value=low, to_value=high, step=0.0)
                ]
            self.declare_parameter(name, getattr(defaults, name), descriptor)
        # v_des สำรอง ใช้เมื่อ /desired_speed ไม่มา/ไม่สด
        self.declare_parameter('v_des', 2.0)

        # runtime
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('log_period_sec', 0.5)
        self.declare_parameter('curvature_half_window', 3)     # 3 -> หน้าต่าง 7 จุด
        self.declare_parameter('loop_gap_max_m', 2.0)          # ช่องว่างปิดลูปเกินนี้ = path เปิด
        # log แบบ Tuning Lab: หนึ่งแถวต่อรอบ พร้อมค่าพารามิเตอร์ที่ใช้ในรอบนั้น
        self.declare_parameter('enable_csv_log', True)
        self.declare_parameter(
            'csv_log_path',
            os.path.expanduser(
                datetime.now().strftime(
                    '~/controller_logs/%Y-%m-%d/cbf_qp_log_%Y%m%d_%H%M%S.csv'
                )
            )
        )

        # ชื่อ topic
        self.declare_parameter('cte_topic', '/controller_debug/cte')
        self.declare_parameter('heading_error_topic', '/controller_debug/heading_error')
        self.declare_parameter('velocity_topic', '/velocity')
        self.declare_parameter('closest_index_topic', '/closest_waypoint_index')
        self.declare_parameter('path_topic', '/nav_path')
        self.declare_parameter('desired_speed_topic', '/desired_speed')
        self.declare_parameter('adaptive_ld_topic', '/adaptive_ld')
        self.declare_parameter('adaptive_speed_topic', '/adaptive_speed')
        self.declare_parameter('corridor_topic', '/cbf_qp/h1_corridor')
        self.declare_parameter('corridor_period_sec', 1.0)     # 0 = ไม่ส่งเส้นกรอบ h₁

        self.params = core.CbfQpParams(**{
            name: type(getattr(defaults, name))(self.get_parameter(name).value)
            for name in SOLVER_PARAMS
        }).validated()
        self.rebuild_grids()

        self.v_des_param = max(float(self.get_parameter('v_des').value), 0.0)
        control_rate_hz = min(max(float(self.get_parameter('control_rate_hz').value), 5.0), 100.0)
        self.input_timeout_sec = max(float(self.get_parameter('input_timeout_sec').value), 0.05)
        self.log_period_sec = max(float(self.get_parameter('log_period_sec').value), 0.0)
        self.curvature_half_window = max(int(self.get_parameter('curvature_half_window').value), 1)
        self.loop_gap_max = max(float(self.get_parameter('loop_gap_max_m').value), 0.0)

        # =========================================================
        # สถานะที่รับเข้ามา (เก็บตามเครื่องหมายของสไลด์แล้ว)
        # =========================================================
        self.e_lat = 0.0
        self.theta_e = 0.0
        self.velocity = 0.0
        self.closest_index = 0
        self.v_des_topic = None
        self.path = None
        self.path_signature = None
        self.path_frame = 'map'
        self.path_z = None
        self.corridor_key = None
        self.corridor_points = None

        self.last_cte_sec = None
        self.last_heading_sec = None
        self.last_velocity_sec = None
        self.last_index_sec = None
        self.last_desired_speed_sec = None
        self.last_log_sec = None
        self.last_wait_warn_sec = None
        self.inputs_ready = False
        self.last_feasible = True
        self.last_infeasible_log_sec = -math.inf
        self.start_sec = self.now_sec()

        self.create_subscription(
            Float64, str(self.get_parameter('cte_topic').value), self.cte_cb, 10)
        self.create_subscription(
            Float64, str(self.get_parameter('heading_error_topic').value),
            self.heading_error_cb, 10)
        self.create_subscription(
            Float64, str(self.get_parameter('velocity_topic').value), self.velocity_cb, 10)
        self.create_subscription(
            Int32, str(self.get_parameter('closest_index_topic').value), self.index_cb, 10)
        self.create_subscription(
            Path, str(self.get_parameter('path_topic').value), self.path_cb, 10)
        self.create_subscription(
            Float64, str(self.get_parameter('desired_speed_topic').value),
            self.desired_speed_cb, 10)

        self.adaptive_ld_topic = str(self.get_parameter('adaptive_ld_topic').value)
        self.pub_ld = self.create_publisher(Float64, self.adaptive_ld_topic, 10)
        self.pub_speed = self.create_publisher(
            Float64, str(self.get_parameter('adaptive_speed_topic').value), 10
        )
        self.pub_feasible = self.create_publisher(Bool, '/cbf_qp/feasible', 10)
        self.pub_pure_pursuit = self.create_publisher(Bool, '/cbf_qp/pure_pursuit_only', 10)
        self.pub_kappa_path = self.create_publisher(Float64, '/cbf_qp/kappa_path', 10)
        self.diagnostic_pubs = {
            name: self.create_publisher(Float64, f'/cbf_qp/{name}', 10)
            for name in DIAGNOSTIC_TOPICS
        }
        self.pub_corridor = self.create_publisher(
            MarkerArray, str(self.get_parameter('corridor_topic').value), 1)
        self.corridor_period_sec = max(float(self.get_parameter('corridor_period_sec').value), 0.0)

        self.csv_file = None
        self.csv_writer = None
        self.csv_rows = 0
        self.init_csv_log()

        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.create_timer(1.0 / control_rate_hz, self.timer_cb)
        if self.corridor_period_sec > 0.0:
            self.create_timer(self.corridor_period_sec, self.publish_corridor)
        self.duplicate_check_timer = self.create_timer(3.0, self.check_duplicate_solver)

        p = self.params
        self.get_logger().info(
            f'CBF-QP solver started @ {control_rate_hz:.1f} Hz '
            f'(grid {len(self.v_grid)}v x {len(self.ld_grid)}Ld = '
            f'{len(self.v_grid) * len(self.ld_grid)} คู่/รอบ)'
        )
        self.get_logger().info(
            f'Bounds: v=[{p.v_min:.2f}, {p.v_max:.2f}] m/s, L_d=[{p.ld_min:.2f}, {p.ld_max:.2f}] m, '
            f'δ_max={p.max_steer_rad:.2f} rad, L={p.wheelbase:.2f} m'
        )
        self.get_logger().info(
            f'h₁: e_max={p.e_max:.2f} m, α₁ = v/e_max (auto, HOCBF) | '
            f'h₂: a_y,max={p.ay_max:.2f} m/s², α₂={p.alpha2:.2f} | '
            f'h₂ᶜ (โค้งข้างหน้า): ชะลอ b={p.curve_decel:.2f} m/s²'
        )
        self.get_logger().info(
            f'L_d,ref = {p.ld_min:.2f} + {p.tau:.2f}·v, k_adapt={p.k_adapt:.2f} | '
            f'J = {p.w_v:.2f}(v − v_des)² + {p.w_ld:.2f}(L_d − L_d,ref^eff)²'
        )
        if not p.optimize_speed:
            self.get_logger().warn(
                'optimize_speed=False -> v = v_des ของไฟล์ เว้นแต่ h₂ เปิด: '
                'แล้ว QP ลดได้ (ส่ง /cbf_qp/speed_cap ให้ pagard)'
            )
        self.get_logger().info('โหมด: ' + mode_text(p))

    # ==============================================================
    #  พารามิเตอร์ / กริด
    # ==============================================================
    def rebuild_grids(self):
        p = self.params
        self.v_grid = core.make_grid(p.v_min, p.v_max, p.v_step)
        self.ld_grid = core.lookahead_grid(p)
        # ระยะมองโค้งไกลสุดที่ candidate ใดจะขอได้
        self.preview_reach = max(p.ld_max, float(self.ld_grid.max()), p.v_max / p.alpha2) + 1.0

    def on_set_parameters(self, parameters):
        changes = {p.name: p.value for p in parameters if p.name in SOLVER_PARAMS}
        try:
            for name, value in changes.items():
                kind = type(getattr(self.params, name))
                if kind is bool and not isinstance(value, bool):
                    raise ValueError(f'{name} ต้องเป็น true/false')
                if kind is float and isinstance(value, bool):
                    raise ValueError(f'{name} ต้องเป็นตัวเลข')
                changes[name] = kind(value)
            new_params = self.params.updated(**changes)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        for p in parameters:
            if p.name == 'v_des':
                self.v_des_param = max(float(p.value), 0.0)
        if changes:
            old_mode = mode_text(self.params)
            self.params = new_params
            self.rebuild_grids()
            self.get_logger().info(
                'ปรับพารามิเตอร์: '
                + ', '.join(f'{name}={value}' for name, value in changes.items())
            )
            if mode_text(new_params) != old_mode:
                self.get_logger().warn('เปลี่ยนโหมด -> ' + mode_text(new_params))
        return SetParametersResult(successful=True)

    # ==============================================================
    #  Callbacks
    # ==============================================================
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cte_cb(self, msg):
        if math.isfinite(msg.data):
            self.e_lat = -float(msg.data)          # pagard: + = ขวา -> สไลด์: + = ซ้าย
            self.last_cte_sec = self.now_sec()

    def heading_error_cb(self, msg):
        if math.isfinite(msg.data):
            self.theta_e = -float(msg.data)        # pagard: path − yaw -> สไลด์: yaw − path
            self.last_heading_sec = self.now_sec()

    def velocity_cb(self, msg):
        if math.isfinite(msg.data):
            self.velocity = abs(float(msg.data))
            self.last_velocity_sec = self.now_sec()

    def index_cb(self, msg):
        self.closest_index = int(msg.data)
        self.last_index_sec = self.now_sec()

    def desired_speed_cb(self, msg):
        if math.isfinite(msg.data):
            self.v_des_topic = abs(float(msg.data))
            self.last_desired_speed_sec = self.now_sec()

    def path_cb(self, msg):
        # nav_path ส่ง path เดิมซ้ำทุกวินาที — สร้าง PathModel ใหม่เฉพาะตอนเส้นทางเปลี่ยนจริง
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        signature = hash(tuple(points))
        if signature == self.path_signature:
            return
        self.path_signature = signature
        self.path_frame = msg.header.frame_id or 'map'
        self.path_z = np.array([p.pose.position.z for p in msg.poses], dtype=float)
        try:
            self.path = core.PathModel(points, self.curvature_half_window, self.loop_gap_max)
        except ValueError as exc:
            self.path = None
            self.get_logger().warn(f'ใช้ path ไม่ได้: {exc} -> หยุดส่ง u*')
            return
        kappa = self.path.kappa
        self.get_logger().info(
            f'Path: {self.path.count} จุด ยาว {self.path.total:.1f} m '
            f'({"ปิดลูป" if self.path.loop else "ไม่ปิดลูป"}), '
            f'κ_path ∈ [{kappa.min():+.3f}, {kappa.max():+.3f}] 1/m'
        )

    def publish_corridor(self):
        """เส้นขอบกรอบ h₁ สองฝั่งของเส้นทาง (ที่ e_lat = ±e_max) ให้ RViz วาด

        ส่งซ้ำทุก corridor_period_sec พร้อม lifetime สามเท่าของรอบ: e_max หรือสวิตช์ h₁
        ที่เปลี่ยนจะขึ้นภายในหนึ่งรอบ และเส้นหายเองเมื่อ node นี้หยุด
        """
        if self.path is None:
            return
        p = self.params
        key = (self.path_signature, p.e_max)
        if key != self.corridor_key:
            try:
                edges = self.path.corridor(p.e_max)
            except ValueError as exc:
                self.get_logger().warn(f'วาดกรอบ h₁ ไม่ได้: {exc}')
                return
            count = self.path.count
            z = self.path_z[:count] if self.path_z is not None and len(self.path_z) >= count \
                else np.zeros(count)
            if self.path.loop:
                z = np.append(z, z[0])
            self.corridor_points = [
                [Point(x=float(x), y=float(y), z=float(h)) for (x, y), h in zip(edge, z)]
                for edge in edges
            ]
            self.corridor_key = key

        active = p.use_h1 and not p.pure_pursuit_only
        color = ColorRGBA(r=0.38, g=0.65, b=0.98, a=0.95) if active \
            else ColorRGBA(r=0.55, g=0.58, b=0.63, a=0.55)
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for marker_id, points in enumerate(self.corridor_points):   # 0 = ซ้าย, 1 = ขวา
            marker = Marker()
            marker.header.frame_id = self.path_frame
            marker.header.stamp = stamp
            marker.ns = 'h1_corridor'
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05 if active else 0.035
            marker.color = color
            marker.points = points
            marker.lifetime.sec = int(math.ceil(3.0 * self.corridor_period_sec))
            markers.markers.append(marker)
        self.pub_corridor.publish(markers)

    def is_fresh(self, stamp_sec, now_sec):
        return stamp_sec is not None and (now_sec - stamp_sec) <= self.input_timeout_sec

    def check_duplicate_solver(self):
        # ตัวแก้ QP สองตัวบน /adaptive_ld = pagard ได้ L_d* สลับไปมาระหว่างสองคำตอบ
        self.duplicate_check_timer.cancel()
        if self.count_publishers(self.adaptive_ld_topic) > 1:
            self.get_logger().error(
                f'มี publisher อื่นบน {self.adaptive_ld_topic} อยู่ด้วย '
                '(adaptive_ld_node_fixed.py ยังรันอยู่?) -> ปิดตัวใดตัวหนึ่งก่อนใช้งาน'
            )

    # ==============================================================
    #  Control loop
    # ==============================================================
    def timer_cb(self):
        now = self.now_sec()

        # สถานะไม่ครบ/ไม่สด -> "หยุด publish" ให้ pagard timeout แล้วถอยไปใช้ heuristic เอง
        missing = [
            name for name, ok in (
                ('cte', self.is_fresh(self.last_cte_sec, now)),
                ('heading_error', self.is_fresh(self.last_heading_sec, now)),
                ('velocity', self.is_fresh(self.last_velocity_sec, now)),
                ('closest_waypoint_index', self.is_fresh(self.last_index_sec, now)),
                ('nav_path', self.path is not None),
            ) if not ok
        ]
        if missing:
            if self.inputs_ready or self.last_wait_warn_sec is None \
                    or now - self.last_wait_warn_sec > 5.0:
                self.inputs_ready = False
                self.last_wait_warn_sec = now
                self.get_logger().warn(
                    f'ยังไม่ได้รับ/ไม่สด: {", ".join(missing)} -> หยุดส่ง u* ชั่วคราว '
                    'ให้ controller ถอยไปใช้ค่า heuristic'
                )
            return
        if not self.inputs_ready:
            self.inputs_ready = True
            self.get_logger().info('รับสถานะครบแล้ว -> เริ่มส่ง u* = [v*, L_d*]')

        params = self.params
        path = self.path
        state = {
            'e_lat': float(self.e_lat),
            'theta_e': float(self.theta_e),
            'v': float(self.velocity),
            'kappa_path': path.kappa_at(self.closest_index),
            # โปรไฟล์จาก path มาก่อนเสมอ แต่ต้องยังสด ไม่งั้นใช้ค่า parameter
            'v_des': (
                self.v_des_topic
                if self.v_des_topic is not None
                and self.is_fresh(self.last_desired_speed_sec, now)
                else self.v_des_param
            ),
        }

        try:
            eta, chord = path.goal_geometry(
                self.closest_index, state['e_lat'], state['theta_e'], self.ld_grid
            )
            top = max(params.v_max, state['v_des'])
            v_curve = path.curve_speed_limit(
                self.closest_index, params.ay_max, params.curve_decel,
                core.curve_horizon(params, top),
            )
            solution = core.solve(
                params, self.v_grid, self.ld_grid,
                state['e_lat'], state['theta_e'], state['v'], state['kappa_path'],
                state['v_des'], eta, chord,
                path.preview(self.closest_index, self.preview_reach),
                v_curve=v_curve,
            )
        except Exception as exc:  # pragma: no cover - safety net
            self.get_logger().warn(f'CBF-QP error: {exc}')
            return

        self.pub_ld.publish(Float64(data=solution['ld_star']))
        self.pub_speed.publish(Float64(data=solution['v_star']))
        self.pub_feasible.publish(Bool(data=solution['feasible']))
        self.pub_pure_pursuit.publish(Bool(data=params.pure_pursuit_only))
        self.pub_kappa_path.publish(Float64(data=state['kappa_path']))
        for name, pub in self.diagnostic_pubs.items():
            value = solution[name]
            if name == 'v_limit':          # ∞ บนทางตรง (κ̂ = 0) -> รายงานเพดานของกริดแทน
                value = min(value, params.v_max)
            elif name in ('v_curve', 'h2_curve'):   # ∞ = ไม่มีโค้งในระยะที่มอง
                value = min(value, 99.0)
            # speed_cap ส่ง ∞ ตามจริง: pagard ถือว่า "ไม่จำกัด"
            pub.publish(Float64(data=value))

        self.write_csv_row(now, params, state, solution)
        self.log_solution(now, params, state, solution)

    def log_solution(self, now, params, state, solution):
        feasible = solution['feasible']
        # จังหวะที่เพิ่งกลายเป็น infeasible ให้ขึ้น log ทันที ไม่รอรอบ log ปกติ
        # (จำกัดไว้ 1 ครั้ง/วินาที กันข้อความท่วมตอนสถานะสลับไปมาทุกรอบ)
        became_infeasible = self.last_feasible and not feasible
        self.last_feasible = feasible
        if became_infeasible and now - self.last_infeasible_log_sec >= 1.0:
            self.last_infeasible_log_sec = now
        else:
            if self.log_period_sec <= 0.0:
                return
            if self.last_log_sec is not None and (now - self.last_log_sec) < self.log_period_sec:
                return
        self.last_log_sec = now

        line = mode_tag(params) + (
            f"e_lat={state['e_lat']:+.3f} θ_e={math.degrees(state['theta_e']):+5.1f}° "
            f"v={state['v']:.2f} κ={state['kappa_path']:+.3f} | "
            f"u*: v*={solution['v_star']:.2f} (des {state['v_des']:.2f}) "
            f"L_d*={solution['ld_star']:.2f} (ref {solution['ld_ref']:.2f}) "
            f"δ={math.degrees(solution['delta']):+5.1f}° κ̂={solution['kappa_hat']:.3f} | "
            f"h₁={solution['h1']:+.3f} ψ̇₁+α₁ψ₁={solution['hocbf']:+.3f} "
            f"h₂={solution['h2']:+.3f} v_curve={min(solution['v_curve'], 99.0):.2f} "
            f"J={solution['j']:.4f}"
        )
        if not feasible:
            self.get_logger().warn(line + '  [QP ไม่มีคำตอบ -> ใช้จุดที่ละเมิดข้อจำกัดน้อยที่สุด]')
        elif solution['h1'] < 0.0:
            # เงื่อนไข HOCBF ยังผ่านได้ถ้า δ ของ candidate ดึงกลับแรงพอ แต่รถหลุดกรอบไปแล้วจริง
            # (h₁ ปิดอยู่ = เป็นแค่ข้อมูล ไม่ใช่คำเตือน)
            log = self.get_logger().warn if params.use_h1 and not params.pure_pursuit_only \
                else self.get_logger().info
            log(line + '  [รถอยู่นอกกรอบ e_max (h₁ < 0)]')
        else:
            self.get_logger().info(line)

    # ==============================================================
    #  CSV log
    # ==============================================================
    def init_csv_log(self):
        if not bool(self.get_parameter('enable_csv_log').value):
            return
        csv_path = os.path.expanduser(str(self.get_parameter('csv_log_path').value))
        try:
            log_dir = os.path.dirname(csv_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            self.csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
        except OSError as exc:
            self.get_logger().warn(f'เปิด CSV log ไม่ได้ ({exc}) -> ทำงานต่อโดยไม่บันทึก')
            return
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ('time_sec', 'closest_index') + LOGGED_STATE + LOGGED_SOLUTION + LOGGED_PARAMS
        )
        self.get_logger().info(f'CBF-QP CSV log: {csv_path}')

    def write_csv_row(self, now, params, state, solution):
        if self.csv_writer is None:
            return
        row = [f'{now - self.start_sec:.3f}', self.closest_index]
        row += [f'{state[name]:.5f}' for name in LOGGED_STATE]
        for name in LOGGED_SOLUTION:
            value = solution[name]
            row.append(int(value) if isinstance(value, (bool, int)) else f'{value:.5f}')
        for name in LOGGED_PARAMS:
            value = getattr(params, name)
            row.append(int(value) if isinstance(value, bool) else value)   # สวิตช์ -> 0/1
        self.csv_writer.writerow(row)
        self.csv_rows += 1
        if self.csv_rows % 20 == 0:        # ~1 s ที่ 20 Hz — กันข้อมูลหายถ้า process ถูก kill
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CbfQpSolverNode()
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
