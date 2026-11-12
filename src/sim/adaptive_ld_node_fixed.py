#!/usr/bin/env python3
"""
adaptive_ld_node_fixed.py
=========================
ROS2 node — CBF-QP Adaptive Speed + Lookahead Solver

แทนที่ fuzzy inference ตัวเดิม ด้วย CBF-QP ตามสถาปัตยกรรมใหม่:
node นี้ "เลือก v และ L_d พร้อมกัน" ทุกเฟรม แทนที่จะปรับทีละตัวแบบ heuristic

  u  = [ v, L_d ]ᵀ                                   (ตัวแปรตัดสินใจ)
  J  = w_v (v − v_des)² + w_Ld (L_d − L_d,ref)²      (ฟังก์ชันวัตถุประสงค์)
  u* = argmin J   s.t.  CBF ทั้งสองข้อ + input bounds

INPUTS (subscribed topics) — มาจาก pagard_adaptive_fixed.py ทั้งหมด:
  /controller_debug/cte            Float64  e_lat   ระยะเยื้องด้านข้าง (+ = รถอยู่ "ขวา" ของ path)
  /controller_debug/heading_error  Float64  θ_e     มุมผิดพลาดทิศหัวรถ (path_heading − yaw)
  /velocity                        Float64  v       ความเร็วจริงของรถ (m/s)
  /curvature                       Float64  κ       ความโค้ง path ตรงตำแหน่งรถ (|κ|)
  /curvature_preview               Float64  κ_preview  ความโค้งสูงสุดในระยะ v/α₂ ข้างหน้า
  /desired_speed        (optional) Float64  v_des   จาก pagard = คอลัมน์ v_target ของ CSV
                                                    ที่ waypoint ปัจจุบัน; ถ้าไม่มา/ไม่สด ใช้ค่า parameter

OUTPUTS (published topics):
  /adaptive_ld     Float64  L_d*  ระยะมองล่วงหน้าที่ QP เลือก (m)   ← ชื่อเดิม ไม่ทำ logger พัง
  /adaptive_speed  Float64  v*    ความเร็วเป้าหมายที่ QP เลือก (m/s)
  /cbf_qp/h1, /cbf_qp/h2, /cbf_qp/j, /cbf_qp/kappa_hat,
  /cbf_qp/a_y, /cbf_qp/delta, /cbf_qp/ld_ref, /cbf_qp/feasible   (diagnostics)

--------------------------------------------------------------------------
สมการที่ใช้ (เรียงตามลำดับการคำนวณในหนึ่งเฟรม)
--------------------------------------------------------------------------
D. ตัวแปรที่คำนวณต่อ — ทำใหม่ทุกคู่ (v, L_d) ใน grid search
     จุดเป้าหมายบนแบบจำลอง path เฉพาะที่ (locally-straight) ใน body frame:
       p = ( s·cosθ_e ,  e_lat + s·sinθ_e )   โดยเลือก s ให้ |p| = L_d
       η = atan2(p_y, p_x)
       δ        = atan( 2L·sinη / |p| )    clamped ±δ_max
       κ_cmd    = | tanδ | / L
       κ̂        = max( κ_cmd, κ_preview )
       a_y      = v²κ̂
E. ตัวแปรความปลอดภัย (บวก = ปลอดภัย)
       h₁ = e_max² − e_lat²          ,  ḣ₁ = −2·e_lat·v·sinθ_e
       เงื่อนไข CBF:  ḣ₁ + α₁h₁ ≥ 0   (คุม "อัตรา" ที่เข้าใกล้ขอบกรอบ ไม่ใช่แค่ค่าปัจจุบัน)
       h₂ = a_y,max − v²κ̂ ≥ 0        (เช็คตรง ๆ ตามที่ระบุไว้ในแล็บ)
F. ผลลัพธ์
       u* = [v*, L_d*] = argmin J บนคู่ที่ผ่านข้อจำกัดทุกข้อ

--------------------------------------------------------------------------
หมายเหตุสำคัญ 2 ข้อ (ส่วนที่ขยายจากสเปกในเอกสาร — เปิด/ปิดได้)
--------------------------------------------------------------------------
1) ḣ₁ + α₁h₁ ≥ 0 ในรูป relative-degree 1 มี v อยู่ตัวเดียว ไม่มี L_d
   ⇒ เงื่อนไขนี้ "ชะลอความเร็ว" ได้อย่างเดียว บังคับให้ L_d สั้นลงเพื่อดึงรถ
   กลับเข้า path ไม่ได้ (h₂ กลับดัน L_d ให้ยาวขึ้นด้วยซ้ำ เพราะ L_d ยาว = δ น้อย
   = κ_cmd น้อย). จึงเพิ่ม predictive corridor check: ทำนาย e_lat ไปข้างหน้า
   corridor_horizon_sec วินาที ด้วย δ ของคู่ที่กำลังพิจารณา แล้วบังคับ
   h₁(e_pred) ≥ 0 ด้วย — นี่คือตัวที่ทำให้ QP เลือก L_d สั้นลงตอนหลุด path จริง ๆ
   ปิดได้ด้วย use_predictive_corridor:=false เพื่อกลับไปตรงตามสเปกเป๊ะ ๆ
2) L_d,ref เป็น "อินพุตหมวด C" ที่ต้องมีคนผลิตให้ QP. ที่นี่สร้างจาก
       L_d,ref = clamp(base + k_v·v, ...) / (1 + k_κ·κ_preview)
   ค่า default ปรับให้ตรงกับเป้าหมายคนขับจริงของสนาม KMUTT ที่ใช้จูน fuzzy เดิม
   (ทางตรง κ≈0.005 → ≈6.9 m, โค้ง κ≈0.18-0.22 → ≈2.3-2.6 m)
   QP มีหน้าที่ "กรองความปลอดภัย + หาสมดุลกับ v" ทับบนค่าอ้างอิงนี้อีกที
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


def clamp(value, low, high):
    return max(low, min(high, value))


class CbfQpAdaptiveNode(Node):

    def __init__(self):
        super().__init__('adaptive_ld_node')

        # =========================================================
        # หมวด A — ค่าคงที่ของรถ / ขอบเขตตัวแปรนำเข้า (input bounds)
        # =========================================================
        self.declare_parameter('wheelbase', 1.57)          # L (m)
        self.declare_parameter('max_steer_rad', 0.8)       # δ_max (rad)
        self.declare_parameter('v_min', 0.0)               # v_min (m/s)
        self.declare_parameter('v_max', 4.0)               # v_max (m/s)
        # ต้องครอบ v_target สูงสุดของ CSV (3.794) ไม่ใช่แค่เรื่อง diagnostic:
        # v_des ถูก clamp ด้วย v_max ตั้งแต่ desired_speed_cb แล้ว L_d,ref = base + k_v·v
        # กับ h₂ = a_y,max − v²κ̂ ก็ใช้ v ตัวนั้น ถ้า clamp ต่ำกว่าความเร็วจริง
        # QP จะเลือก L_d สั้นเกินจริงและประเมิน a_y ต่ำกว่าความเป็นจริง
        self.declare_parameter('ld_min', 2.0)              # L_d,min (m)
        self.declare_parameter('ld_max', 8.0)              # L_d,max (m)

        # =========================================================
        # หมวด C — พารามิเตอร์ปรับจูน (เป้าหมาย + น้ำหนัก + ขอบปลอดภัย)
        # =========================================================
        self.declare_parameter('v_des', 2.0)               # v_des (m/s)
        self.declare_parameter('w_v', 1.0)                 # w_v
        self.declare_parameter('w_ld', 0.15)               # w_Ld
        self.declare_parameter('e_max', 0.6)               # ครึ่งความกว้างกรอบเส้นทาง (m)
        self.declare_parameter('alpha1', 2.0)              # α₁ class-K gain ของ h₁ (1/s)
        self.declare_parameter('ay_max', 2.0)              # a_y,max (m/s²)
        # α₂ = ระยะมองโค้งล่วงหน้า (preview distance = v/α₂); ตัวคำนวณจริงอยู่ใน
        # pagard เพราะที่นั่นมี path ทั้งเส้น — ประกาศไว้ที่นี่เพื่อ log/ตรวจสอบว่าตั้งตรงกัน
        self.declare_parameter('alpha2', 0.4)

        # ตัวสร้างค่าอ้างอิง L_d,ref (ดูหมายเหตุข้อ 2 ด้านบน)
        self.declare_parameter('ld_ref_base', 2.0)
        self.declare_parameter('ld_ref_speed_gain', 2.6)
        self.declare_parameter('ld_ref_curve_gain', 9.6)

        # =========================================================
        # ตัวแก้ QP (grid search) และ runtime
        # =========================================================
        self.declare_parameter('v_grid_points', 45)
        # False = QP เลิกหา v เอง ใช้ v_des (โปรไฟล์จาก CSV) ตรง ๆ แล้วหาแต่ L_d
        # J เหลือแค่ w_Ld(L_d − L_d,ref)² และ CBF ทั้งสองข้อกลายเป็นข้อจำกัดของ L_d
        # อย่างเดียว — ความเร็วจะไม่ถูกอะไรในโหนดนี้ปรับอีก
        self.declare_parameter('optimize_speed', False)
        self.declare_parameter('ld_grid_points', 61)
        self.declare_parameter('use_predictive_corridor', True)
        self.declare_parameter('corridor_horizon_sec', 1.0)
        self.declare_parameter('corridor_horizon_steps', 5)
        self.declare_parameter('infeasible_violation_weight', 200.0)
        # เพดานความเร็วตอนไม่มีคู่ไหนผ่านข้อจำกัด (เช่น หลุดกรอบไปแล้ว h₁ < 0)
        # จำเป็นเพราะ ḣ₁ = −2·e_lat·v·sinθ_e: ถ้ารถหลุดออกไปแล้วแต่กำลังวิ่งกลับเข้าเส้น
        # การ "เร่ง" จะทำให้ ḣ₁ + α₁h₁ ดูดีขึ้นในเชิงตัวเลข -> QP จะเลือก v_max ซึ่งอันตราย
        # ในโลกจริง ตอนหลุดกรอบต้องช้าลงเสมอ ไม่ใช่เร่งกลับเข้าเส้น
        self.declare_parameter('recovery_v_max', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('log_period_sec', 0.5)

        # =========================================================
        # ชื่อ topic
        # =========================================================
        self.declare_parameter('cte_topic', '/controller_debug/cte')
        self.declare_parameter('heading_error_topic', '/controller_debug/heading_error')
        self.declare_parameter('velocity_topic', '/velocity')
        self.declare_parameter('curvature_topic', '/curvature')
        self.declare_parameter('curvature_preview_topic', '/curvature_preview')
        self.declare_parameter('desired_speed_topic', '/desired_speed')
        self.declare_parameter('adaptive_ld_topic', '/adaptive_ld')
        self.declare_parameter('adaptive_speed_topic', '/adaptive_speed')

        self.wheelbase = max(float(self.get_parameter('wheelbase').value), 1e-3)
        self.max_steer_rad = clamp(float(self.get_parameter('max_steer_rad').value), 0.05, 1.5)
        self.v_min = max(float(self.get_parameter('v_min').value), 0.0)
        self.v_max = max(float(self.get_parameter('v_max').value), self.v_min + 1e-3)
        self.ld_min = max(float(self.get_parameter('ld_min').value), 0.1)
        self.ld_max = max(float(self.get_parameter('ld_max').value), self.ld_min + 1e-3)

        self.v_des_param = clamp(float(self.get_parameter('v_des').value), self.v_min, self.v_max)
        self.w_v = max(float(self.get_parameter('w_v').value), 0.0)
        self.w_ld = max(float(self.get_parameter('w_ld').value), 0.0)
        self.e_max = max(float(self.get_parameter('e_max').value), 1e-3)
        self.alpha1 = max(float(self.get_parameter('alpha1').value), 0.0)
        self.ay_max = max(float(self.get_parameter('ay_max').value), 1e-3)
        self.alpha2 = max(float(self.get_parameter('alpha2').value), 1e-3)

        self.ld_ref_base = float(self.get_parameter('ld_ref_base').value)
        self.ld_ref_speed_gain = float(self.get_parameter('ld_ref_speed_gain').value)
        self.ld_ref_curve_gain = max(float(self.get_parameter('ld_ref_curve_gain').value), 0.0)

        self.optimize_speed = bool(self.get_parameter('optimize_speed').value)
        n_v = max(int(self.get_parameter('v_grid_points').value), 2)
        n_ld = max(int(self.get_parameter('ld_grid_points').value), 2)
        self.use_predictive_corridor = bool(
            self.get_parameter('use_predictive_corridor').value
        )
        self.corridor_horizon_sec = max(
            float(self.get_parameter('corridor_horizon_sec').value), 0.0
        )
        self.corridor_horizon_steps = max(
            int(self.get_parameter('corridor_horizon_steps').value), 1
        )
        self.violation_weight = max(
            float(self.get_parameter('infeasible_violation_weight').value), 1.0
        )
        self.recovery_v_max = clamp(
            float(self.get_parameter('recovery_v_max').value), self.v_min, self.v_max
        )
        control_rate_hz = clamp(float(self.get_parameter('control_rate_hz').value), 5.0, 100.0)
        self.control_period = 1.0 / control_rate_hz
        self.input_timeout_sec = max(float(self.get_parameter('input_timeout_sec').value), 0.05)
        self.log_period_sec = max(float(self.get_parameter('log_period_sec').value), 0.0)

        # grid ของตัวแปรตัดสินใจ — สร้างครั้งเดียว ใช้ซ้ำทุกเฟรม
        self.v_grid = np.linspace(self.v_min, self.v_max, n_v)
        self.ld_grid = np.linspace(self.ld_min, self.ld_max, n_ld)
        self.v_col = self.v_grid.reshape(-1, 1)     # (n_v, 1)
        self.ld_row = self.ld_grid.reshape(1, -1)   # (1, n_ld)

        # =========================================================
        # หมวด B — สถานะที่รับมาจาก pagard
        # =========================================================
        self.e_lat = 0.0
        self.theta_e = 0.0
        self.velocity = 0.0
        self.curvature = 0.0
        self.curvature_preview = 0.0
        self.v_des_topic = None
        self.last_desired_speed_sec = None

        self.last_cte_sec = None
        self.last_heading_sec = None
        self.last_velocity_sec = None
        self.last_curvature_sec = None
        self.last_preview_sec = None
        self.last_log_sec = None
        self.last_stale_warn_sec = None
        self.inputs_ready = False

        cte_topic = str(self.get_parameter('cte_topic').value)
        heading_topic = str(self.get_parameter('heading_error_topic').value)
        velocity_topic = str(self.get_parameter('velocity_topic').value)
        curvature_topic = str(self.get_parameter('curvature_topic').value)
        preview_topic = str(self.get_parameter('curvature_preview_topic').value)
        desired_speed_topic = str(self.get_parameter('desired_speed_topic').value)

        self.create_subscription(Float64, cte_topic, self.cte_cb, 10)
        self.create_subscription(Float64, heading_topic, self.heading_error_cb, 10)
        self.create_subscription(Float64, velocity_topic, self.velocity_cb, 10)
        self.create_subscription(Float64, curvature_topic, self.curvature_cb, 10)
        self.create_subscription(Float64, preview_topic, self.curvature_preview_cb, 10)
        self.create_subscription(Float64, desired_speed_topic, self.desired_speed_cb, 10)

        self.pub_ld = self.create_publisher(
            Float64, str(self.get_parameter('adaptive_ld_topic').value), 10
        )
        self.pub_speed = self.create_publisher(
            Float64, str(self.get_parameter('adaptive_speed_topic').value), 10
        )
        self.pub_h1 = self.create_publisher(Float64, '/cbf_qp/h1', 10)
        self.pub_h2 = self.create_publisher(Float64, '/cbf_qp/h2', 10)
        self.pub_j = self.create_publisher(Float64, '/cbf_qp/j', 10)
        self.pub_kappa_hat = self.create_publisher(Float64, '/cbf_qp/kappa_hat', 10)
        self.pub_ay = self.create_publisher(Float64, '/cbf_qp/a_y', 10)
        self.pub_delta = self.create_publisher(Float64, '/cbf_qp/delta', 10)
        self.pub_ld_ref = self.create_publisher(Float64, '/cbf_qp/ld_ref', 10)
        self.pub_feasible = self.create_publisher(Bool, '/cbf_qp/feasible', 10)

        self.create_timer(self.control_period, self.timer_cb)

        self.get_logger().info(
            f'CBF-QP adaptive node started @ {control_rate_hz:.1f} Hz '
            f'(grid {n_v}v x {n_ld}Ld = {n_v * n_ld} คู่/เฟรม)'
        )
        self.get_logger().info(
            f'Bounds: v=[{self.v_min:.2f}, {self.v_max:.2f}] m/s, '
            f'L_d=[{self.ld_min:.2f}, {self.ld_max:.2f}] m, '
            f'δ_max={math.degrees(self.max_steer_rad):.1f} deg, L={self.wheelbase:.2f} m'
        )
        self.get_logger().info(
            f'CBF: e_max={self.e_max:.2f} m, α₁={self.alpha1:.2f} 1/s, '
            f'a_y,max={self.ay_max:.2f} m/s², α₂={self.alpha2:.2f} '
            f'(preview distance = v/α₂ คำนวณฝั่ง pagard)'
        )
        self.get_logger().info(
            f'Recovery mode: ตอน infeasible บีบ v <= {self.recovery_v_max:.2f} m/s'
        )
        self.get_logger().info(
            f'Objective: J = {self.w_v:.2f}(v − v_des)² + {self.w_ld:.2f}(L_d − L_d,ref)², '
            f'v_des={self.v_des_param:.2f} m/s'
        )
        self.get_logger().info(
            'Predictive corridor: '
            + (
                f'enabled (T={self.corridor_horizon_sec:.2f}s, '
                f'{self.corridor_horizon_steps} steps)'
                if self.use_predictive_corridor
                else 'disabled (relative-degree-1 CBF ตามสเปกเป๊ะ)'
            )
        )

    # ==============================================================
    #  Callbacks
    # ==============================================================
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cte_cb(self, msg):
        self.e_lat = float(msg.data)
        self.last_cte_sec = self.now_sec()

    def heading_error_cb(self, msg):
        self.theta_e = float(msg.data)
        self.last_heading_sec = self.now_sec()

    def velocity_cb(self, msg):
        self.velocity = abs(float(msg.data))
        self.last_velocity_sec = self.now_sec()

    def curvature_cb(self, msg):
        self.curvature = abs(float(msg.data))
        self.last_curvature_sec = self.now_sec()

    def curvature_preview_cb(self, msg):
        self.curvature_preview = abs(float(msg.data))
        self.last_preview_sec = self.now_sec()

    def desired_speed_cb(self, msg):
        self.v_des_topic = clamp(abs(float(msg.data)), self.v_min, self.v_max)
        self.last_desired_speed_sec = self.now_sec()

    def is_fresh(self, stamp_sec, now_sec):
        return stamp_sec is not None and (now_sec - stamp_sec) <= self.input_timeout_sec

    # ==============================================================
    #  หมวด C — ตัวสร้างค่าอ้างอิง L_d,ref
    # ==============================================================
    def compute_ld_ref(self, velocity, kappa_preview):
        """L_d,ref — คิดจาก "ความเร็วที่กำลังพิจารณา" ไม่ใช่ความเร็วที่วัดได้

        เพราะ v กับ L_d ถูกเลือกพร้อมกัน ค่าอ้างอิงจึงต้องสอดคล้องกันเองด้วย
        (ถ้าอิงความเร็วที่วัดได้ ตอนออกตัวจาก v=0 ค่า L_d,ref จะตกไปที่ L_d,min ทั้งที่
        QP กำลังจะสั่งให้เร่งขึ้นถึง v_des) — velocity รับได้ทั้งสเกลาร์และ numpy array
        """
        base = self.ld_ref_base + self.ld_ref_speed_gain * np.maximum(velocity, 0.0)
        shaped = base / (1.0 + self.ld_ref_curve_gain * max(kappa_preview, 0.0))
        return np.clip(shaped, self.ld_min, self.ld_max)

    # ==============================================================
    #  ตัวแก้ CBF-QP (grid search บนคู่ (v, L_d) ทั้งหมด)
    # ==============================================================
    def solve_qp(self, e_lat, theta_e, kappa_preview, v_des):
        L = self.wheelbase
        # optimize_speed=False -> กริดความเร็วเหลือจุดเดียวคือ v_des จาก CSV
        # โครงสร้างเมทริกซ์ที่เหลือทำงานเหมือนเดิมทุกอย่าง แค่มิติ v เหลือ 1
        if self.optimize_speed:
            v_grid = self.v_grid
        else:
            v_grid = np.array([clamp(v_des, self.v_min, self.v_max)])
        v_col = v_grid.reshape(-1, 1)
        ct = math.cos(theta_e)
        st = math.sin(theta_e)
        ld = self.ld_grid

        # ---- หมวด D: เรขาคณิต pure pursuit ของทุก L_d (ไม่ขึ้นกับ v) ----
        # จุดเป้าหมายบน path จำลองแบบเส้นตรงเฉพาะที่ ใน body frame (x หน้า, y ซ้าย):
        #   path ผ่านจุด (0, e_lat) ในทิศ (cosθ_e, sinθ_e)
        #   เลือก arclength s ให้ระยะจากรถถึงจุดนั้นเท่ากับ L_d พอดี
        disc = ld * ld - (e_lat * ct) ** 2
        s = np.where(
            disc > 0.0,
            -e_lat * st + np.sqrt(np.maximum(disc, 0.0)),
            0.0,
        )
        s = np.maximum(s, 0.0)
        px = s * ct
        py = e_lat + s * st
        chord = np.maximum(np.hypot(px, py), 1e-3)

        eta = np.arctan2(py, px)                                  # η
        delta = np.arctan2(2.0 * L * np.sin(eta), chord)          # δ
        delta = np.clip(delta, -self.max_steer_rad, self.max_steer_rad)
        kappa_cmd = np.abs(np.tan(delta)) / L                     # κ_cmd
        kappa_hat = np.maximum(kappa_cmd, max(kappa_preview, 0.0))  # κ̂

        # ---- หมวด E: ตัวแปรความปลอดภัย ----
        v = v_col                                                 # (n_v, 1)
        a_y = (v * v) * kappa_hat[None, :]                        # a_y = v²κ̂
        h2 = self.ay_max - a_y                                    # h₂ ≥ 0

        h1 = self.e_max ** 2 - e_lat ** 2                         # h₁ (สเกลาร์)
        h1_dot = -2.0 * e_lat * v * st                            # ḣ₁ = −2·e_lat·v·sinθ_e
        cbf1 = h1_dot + self.alpha1 * h1                          # ḣ₁ + α₁h₁ ≥ 0

        margins = [np.broadcast_to(cbf1, a_y.shape), h2]

        if self.use_predictive_corridor and self.corridor_horizon_sec > 0.0:
            h1_pred = self.predict_corridor_margin(e_lat, theta_e, delta, v_col)
            margins.append(h1_pred)
        else:
            h1_pred = np.full(a_y.shape, h1)

        # ---- หมวด C -> ฟังก์ชันวัตถุประสงค์ J ----
        ld_ref = self.compute_ld_ref(v, kappa_preview)            # (n_v, 1)
        cost = self.w_ld * (self.ld_row - ld_ref) ** 2
        if self.optimize_speed:
            cost = cost + self.w_v * (v - v_des) ** 2

        feasible_mask = np.ones(a_y.shape, dtype=bool)
        for margin in margins:
            feasible_mask &= (margin >= 0.0)

        if feasible_mask.any():
            feasible = True
            selection = np.where(feasible_mask, cost, np.inf)
        else:
            # ไม่มีคู่ไหนผ่านครบ (เช่น หลุดกรอบไปแล้ว h₁ < 0) -> ผ่อนเป็น soft constraint
            # เลือกคู่ที่ละเมิดน้อยที่สุดก่อน แล้วค่อยดู J
            feasible = False
            violation = np.zeros(a_y.shape)
            for margin in margins:
                violation = violation + np.maximum(-margin, 0.0)
            selection = self.violation_weight * violation + cost
            # บีบให้อยู่ในโหมดกู้คืน: ตอนละเมิดข้อจำกัดอยู่ ห้ามเลือกความเร็วเกิน recovery_v_max
            # ทำได้เฉพาะตอน v เป็นตัวแปรตัดสินใจ ถ้า v ถูกตรึงจาก CSV แล้วเงื่อนไขนี้จะ
            # ตัดทุกช่องเป็น inf เมื่อ v_des > recovery_v_max ทำให้ argmin เลือกมั่ว
            if self.optimize_speed:
                selection = np.where(v <= self.recovery_v_max + 1e-9, selection, np.inf)

        flat_idx = int(np.argmin(selection))
        i_v, i_ld = np.unravel_index(flat_idx, selection.shape)

        return {
            'v_star': float(v_grid[i_v]),
            'ld_star': float(self.ld_grid[i_ld]),
            'ld_ref': float(np.asarray(ld_ref).reshape(-1)[i_v]),
            'delta': float(delta[i_ld]),
            'eta': float(eta[i_ld]),
            'kappa_cmd': float(kappa_cmd[i_ld]),
            'kappa_hat': float(kappa_hat[i_ld]),
            'a_y': float(a_y[i_v, i_ld]),
            'h1': float(h1),
            'h1_pred': float(np.asarray(h1_pred)[i_v, i_ld]),
            'h2': float(h2[i_v, i_ld]),
            'cbf1': float(np.asarray(cbf1).reshape(-1)[i_v]),
            'j': float(cost[i_v, i_ld]),
            'feasible': feasible,
        }

    def predict_corridor_margin(self, e_lat, theta_e, delta, v_col=None):
        """ทำนาย h₁ ที่เวลา T ข้างหน้า ด้วย δ ของแต่ละคู่ (v, L_d)

        แบบจำลอง: unicycle บน path ที่สมมติว่าตรงในช่วง T (โค้งจริงถูกดูแลโดย h₂/κ_preview)
          ψ = มุมหัวรถเทียบทิศ path = −θ_e ,  ψ̇ = v·tanδ / L
          ė_lat = v·sinθ_e = −v·sinψ
        """
        # ต้องใช้กริดความเร็วชุดเดียวกับ solve_qp ไม่งั้น shape ไม่ตรงตอน v ถูกตรึง
        v = self.v_col if v_col is None else v_col        # (n_v, 1)
        yaw_rate = v * np.tan(delta)[None, :] / self.wheelbase   # (n_v, n_ld)
        psi = np.full(yaw_rate.shape, -theta_e)
        e_pred = np.full(yaw_rate.shape, float(e_lat))

        dt = self.corridor_horizon_sec / self.corridor_horizon_steps
        for _ in range(self.corridor_horizon_steps):
            psi_mid = psi + 0.5 * yaw_rate * dt
            e_pred = e_pred - v * np.sin(psi_mid) * dt
            psi = psi + yaw_rate * dt

        return self.e_max ** 2 - e_pred ** 2

    # ==============================================================
    #  Control loop
    # ==============================================================
    def timer_cb(self):
        now = self.now_sec()

        # e_lat / θ_e / v คือสถานะที่ขาดไม่ได้ — ถ้าค้างเกิน timeout ให้ "หยุด publish"
        # แทนที่จะเดาค่า เพื่อให้ pagard timeout แล้วถอยไปใช้ heuristic ของตัวเองได้
        if not (
            self.is_fresh(self.last_cte_sec, now)
            and self.is_fresh(self.last_heading_sec, now)
            and self.is_fresh(self.last_velocity_sec, now)
        ):
            if self.inputs_ready or self.last_stale_warn_sec is None:
                self.inputs_ready = False
                self.last_stale_warn_sec = now
                self.get_logger().warn(
                    'สถานะจาก pagard (cte / heading_error / velocity) ไม่สด -> '
                    'หยุดส่ง u* ชั่วคราว ให้ controller ถอยไปใช้ค่า heuristic'
                )
            elif now - self.last_stale_warn_sec > 5.0:
                self.last_stale_warn_sec = now
                self.get_logger().warn('ยังไม่ได้รับสถานะที่สดจาก pagard')
            return

        if not self.inputs_ready:
            self.inputs_ready = True
            self.get_logger().info('รับสถานะครบแล้ว -> เริ่มส่ง u* = [v*, L_d*]')

        e_lat = float(self.e_lat)
        theta_e = float(self.theta_e)
        velocity = float(self.velocity)
        # κ_preview คือค่าที่ h₂ ใช้จริง ต้องเป็นค่าที่วัดจากระยะ v/α₂ ตรง ๆ
        # ถ้าไม่สดค่อยถอยไปใช้ /curvature (ซึ่งฝั่ง pagard เป็น effective curvature
        # ที่คูณ gain เผื่อไว้แล้ว จึงอนุรักษ์นิยมกว่า — เหมาะกับการเป็น fallback แต่ไม่ควรใช้เป็นค่าหลัก)
        kappa_preview = (
            self.curvature_preview
            if self.is_fresh(self.last_preview_sec, now)
            else self.curvature
        )

        # โปรไฟล์จาก path มาก่อนเสมอ แต่ต้องยัง "สด" — ถ้า pagard หยุดส่ง (path หมด,
        # โปรไฟล์ไม่ตรงกับ path) ห้ามค้างที่ค่าสุดท้าย ให้ถอยไปใช้ค่า parameter
        if self.v_des_topic is not None and self.is_fresh(self.last_desired_speed_sec, now):
            v_des = self.v_des_topic
        else:
            v_des = self.v_des_param

        try:
            solution = self.solve_qp(e_lat, theta_e, kappa_preview, v_des)
        except Exception as exc:  # pragma: no cover - safety net
            self.get_logger().warn(f'CBF-QP error: {exc}')
            return

        self.pub_ld.publish(Float64(data=solution['ld_star']))
        self.pub_speed.publish(Float64(data=solution['v_star']))
        self.pub_h1.publish(Float64(data=solution['h1']))
        self.pub_h2.publish(Float64(data=solution['h2']))
        self.pub_j.publish(Float64(data=solution['j']))
        self.pub_kappa_hat.publish(Float64(data=solution['kappa_hat']))
        self.pub_ay.publish(Float64(data=solution['a_y']))
        self.pub_delta.publish(Float64(data=solution['delta']))
        self.pub_ld_ref.publish(Float64(data=solution['ld_ref']))
        self.pub_feasible.publish(Bool(data=bool(solution['feasible'])))

        self.log_solution(now, e_lat, theta_e, velocity, solution)

    def log_solution(self, now, e_lat, theta_e, velocity, solution):
        if self.log_period_sec <= 0.0:
            return
        if self.last_log_sec is not None and (now - self.last_log_sec) < self.log_period_sec:
            return
        self.last_log_sec = now

        line = (
            f"e_lat={e_lat:+.3f} θ_e={math.degrees(theta_e):+5.1f}° v={velocity:.2f} "
            f"κ̂={solution['kappa_hat']:.3f} | u*: v*={solution['v_star']:.2f} "
            f"L_d*={solution['ld_star']:.2f} (ref {solution['ld_ref']:.2f}) "
            f"δ={math.degrees(solution['delta']):+5.1f}° a_y={solution['a_y']:.2f} "
            f"h₁={solution['h1']:+.3f} h₂={solution['h2']:+.3f} J={solution['j']:.3f}"
        )
        if solution['feasible']:
            self.get_logger().info(line)
        else:
            self.get_logger().warn(line + '  [INFEASIBLE -> soft-constraint fallback]')


def main(args=None):
    rclpy.init(args=args)
    node = CbfQpAdaptiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
