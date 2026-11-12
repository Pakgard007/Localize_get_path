#!/usr/bin/env python3
"""
cbf_qp_core.py
==============
สมการ CBF-QP ตามสไลด์ "CBF-QP Tuning Lab" — ไฟล์นี้ไม่มี ROS เลย (numpy ล้วน)
เพื่อให้ทดสอบตัวเลขเทียบกับสไลด์ได้ตรง ๆ (ดู test/test_cbf_qp_core.py)

ทิศทางบวกตามสไลด์ 8 (ใช้ทั้งไฟล์):
  e_lat  > 0  รถอยู่ "ซ้าย" ของเส้นทาง
  θ_e    > 0  หัวรถหมุนทวนเข็มเทียบเส้นสัมผัสเส้นทาง (θ_รถ − θ_เส้นทาง)
  δ      > 0  เลี้ยวซ้าย
  κ_path > 0  เส้นทางโค้งซ้าย

ลำดับการคำนวณในหนึ่งรอบ (เลขสมการ = เลขในสไลด์):
  (3.3) δ      = atan(2L·sinη / L_d)                 Pure Pursuit ของแต่ละ candidate L_d
  (4.1) L_d,ref = L_d,min + τ·v                      time-headway (v = ความเร็วที่วัดได้)
  (4.2)–(4.4) L_d,ref^eff = clamp(L_d,ref/(1+k_adapt·κ)·√(0.4/a_y,max), L_d,min, L_d,max)
  (2.1) h₁ = e_max² − e_lat²      (2.2) ḣ₁ = −2·e_lat·ė_lat ,  ė_lat = v·sinθ_e
  (2.4) ψ₁ = ḣ₁ + α₁h₁ ,  α₁ = v/e_max (คำนวณอัตโนมัติ)
  (2.7) ë_lat = v²(tanδ/L − κ_path)
  (2.8) −2ė_lat² − 2e_lat·ë_lat + 2α₁ḣ₁ + α₁²h₁ ≥ 0      ← HOCBF ของ h₁
  (3.2) κ̂ = max(|tanδ|/L, κ_path ในระยะ max(L_d, v/α₂))
  (3.1) h₂ = a_y,max − v²κ̂ ≥ 0
  J = w_v(v − v_des)² + w_Ld(L_d − L_d,ref^eff)²
  u* = [v*, L_d*] = argmin J บน candidate ที่ผ่าน (2.8) และ (3.1)

เงื่อนไขความโค้งข้างหน้า (เพิ่มจากสไลด์ 2026-11-11 คู่กับ h₂ ใช้สวิตช์ use_h2 ตัวเดียวกัน):
  v_curve² = min_s ( a_y,max/|κ_path(s)| + 2·b·s )   s = ระยะไปข้างหน้า, b = curve_decel
  h₂ᶜ = v_curve² − v² ≥ 0
  ความเร็วตอนนี้ต้องชะลอด้วย b ลงไปถึงความเร็วที่โค้งข้างหน้ารับได้ทันก่อนถึงโค้ง
  (h₂ ของสไลด์มองแค่ระยะ max(L_d, v/α₂) และ κ̂ มักมาจาก |tanδ|/L ตอนรถเยื้องเส้นทาง)

สวิตช์สำหรับการทดลอง (ไม่อยู่ในสไลด์):
  use_h1 / use_h2 = False   ไม่ใช้ข้อจำกัดนั้นตัดสิน candidate (ยังคำนวณค่าไว้ดูใน log)
  pure_pursuit_only = True  ไม่มี QP: L_d* = fixed_ld คงที่, v* = v_des, ไม่ใช้ h₁ h₂
                            (pagard เห็น /cbf_qp/pure_pursuit_only แล้วเลี้ยวด้วย (3.3) ล้วน)
  optimize_speed = False    ความเร็วจากไฟล์เส้นทาง: ปิด h₂ = v ตรึงที่ v_des, เปิด h₂ = v เลือกได้
                            ใน [v_min, v_des] และ speed_cap = v* บอก pagard ให้ชะลอตามนั้น

สถานะ (e_lat, θ_e, v, κ_path) ใน (2.8) เป็น "ค่าที่วัดได้" ของรอบนั้น ส่วน candidate
มีผลผ่าน δ(L_d) อย่างเดียว (สไลด์ 13) — ต่างจาก h₂ ที่ใช้ v ของ candidate ตรง ๆ
"""

import math
from dataclasses import dataclass, replace

import numpy as np

# a_y,max ที่ระบบจูนไว้แต่แรก = ตัวตั้งของ √(0.4/a_y,max) ใน (4.3)
AY_REF = 0.4
# กันผลจากทศนิยมลอยตัวที่ขอบเซตพอดี (เช่น h₂ = 0)
FEAS_TOL = 1e-9


@dataclass(frozen=True)
class CbfQpParams:
    """ค่าเริ่มต้น = "preset ค่าเริ่มต้น" ในสไลด์"""

    # ค่าคงที่ของรถ / ขอบเขตตัวแปรตัดสินใจ
    wheelbase: float = 1.57       # L (m)
    max_steer_rad: float = 0.8    # δ ∈ [−0.8, 0.8] rad
    v_min: float = 0.0
    v_max: float = 3.8            # v ∈ [0, 3.8] m/s
    v_step: float = 0.05          # Δv ของกริด
    ld_min: float = 2.0           # L_d,min (m)
    ld_max: float = 8.0           # L_d,max (m)
    ld_step: float = 0.1          # ΔL_d ของกริด
    # น้ำหนักของ J
    w_v: float = 1.0
    w_ld: float = 0.30
    # ข้อจำกัด
    e_max: float = 0.80           # ครึ่งความกว้างกรอบเส้นทาง (m)
    ay_max: float = 0.40          # เพดานแรงเหวี่ยงด้านข้าง (m/s²)
    alpha2: float = 0.4           # ระยะมองโค้งล่วงหน้า = max(L_d, v/α₂)
    curve_decel: float = 0.5      # b ของ h₂ᶜ: อัตราชะลอก่อนเข้าโค้ง (m/s²)
    # กฎ L_d,ref
    tau: float = 0.70             # time headway (s)
    k_adapt: float = 3.0          # เกนไวต่อโค้ง
    # False = ตรึง v = v_des แล้วหาแต่ L_d (ใช้คู่กับ pagard โหมดความเร็วจาก CSV ล้วน)
    optimize_speed: bool = True
    # เพดานความเร็วตอน QP infeasible — ส่วนที่เพิ่มจากสไลด์ (สไลด์ให้ Safety Supervisor
    # พิจารณาเอง) ตั้ง >= v_max เพื่อปิด
    recovery_v_max: float = 1.0
    # สวิตช์ของการทดลอง: เปิด/ปิดข้อจำกัดทีละตัว หรือเหลือแค่ Pure Pursuit ที่ L_d คงที่
    use_h1: bool = True
    use_h2: bool = True
    pure_pursuit_only: bool = False
    fixed_ld: float = 4.0         # L_d (m) ของโหมด pure_pursuit_only

    def validated(self):
        """คืนตัวเองถ้าค่าใช้ได้ ไม่งั้น raise ValueError พร้อมเหตุผล"""
        checks = [
            (self.wheelbase > 0.0, 'wheelbase ต้อง > 0'),
            (0.0 < self.max_steer_rad < math.pi / 2, 'max_steer_rad ต้องอยู่ใน (0, π/2)'),
            (self.v_min >= 0.0, 'v_min ต้อง >= 0'),
            (self.v_max > self.v_min, 'v_max ต้อง > v_min'),
            (self.v_step > 0.0, 'v_step ต้อง > 0'),
            (self.ld_min > 0.0, 'ld_min ต้อง > 0'),
            (self.ld_max > self.ld_min, 'ld_max ต้อง > ld_min'),
            (self.ld_step > 0.0, 'ld_step ต้อง > 0'),
            (self.w_v >= 0.0, 'w_v ต้อง >= 0'),
            (self.w_ld >= 0.0, 'w_ld ต้อง >= 0'),
            (self.e_max > 0.0, 'e_max ต้อง > 0'),
            (self.ay_max > 0.0, 'ay_max ต้อง > 0'),
            (self.alpha2 > 0.0, 'alpha2 ต้อง > 0'),
            (self.curve_decel > 0.0, 'curve_decel ต้อง > 0'),
            (self.tau >= 0.0, 'tau ต้อง >= 0'),
            (self.k_adapt >= 0.0, 'k_adapt ต้อง >= 0'),
            (self.recovery_v_max >= 0.0, 'recovery_v_max ต้อง >= 0'),
            (self.fixed_ld > 0.0, 'fixed_ld ต้อง > 0'),
        ]
        for name, value in self.__dict__.items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f'{name} ต้องเป็นจำนวนจำกัด')
        for ok, reason in checks:
            if not ok:
                raise ValueError(reason)
        return self

    def updated(self, **changes):
        return replace(self, **changes).validated()


def make_grid(low, high, step):
    """กริดห่างเท่ากันจาก low ถึง high (รวมปลายทั้งสอง) ระยะห่าง ≈ step"""
    count = max(int(round((high - low) / step)) + 1, 2)
    return np.linspace(low, high, count)


def lookahead_grid(params):
    """candidate ของ L_d: กริด [L_d,min, L_d,max] หรือค่าเดียว fixed_ld ในโหมด Pure Pursuit"""
    if params.pure_pursuit_only:
        return np.array([params.fixed_ld])
    return make_grid(params.ld_min, params.ld_max, params.ld_step)


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# ======================================================================
#  เส้นทางอ้างอิง: ความโค้ง (มีเครื่องหมาย) + ระยะมองล่วงหน้า + จุดเป้าหมาย
# ======================================================================
class PathModel:
    """เส้นทางจาก /nav_path ในรูปที่ QP ใช้

    index ที่รับเข้ามาคือ /closest_waypoint_index ของ pagard = หมายเลข segment
    (จุด i -> i+1) ที่รถฉายลงไป pagard เติมจุดแรกต่อท้ายเสมอถ้า path ยังไม่ปิดลูป
    index ของสองฝั่งจึงตรงกันเมื่อ mod ด้วยจำนวนจุดไม่ซ้ำ
    """

    def __init__(self, points_xy, curvature_half_window=3, loop_gap_max=2.0):
        pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
        if len(pts) < 3:
            raise ValueError('path ต้องมีอย่างน้อย 3 จุด')
        if not np.all(np.isfinite(pts)):
            raise ValueError('path มีพิกัดที่ไม่ใช่จำนวนจำกัด')
        # จุดสุดท้ายทับจุดแรก (path ปิดลูปมาแล้ว) -> ตัดตัวซ้ำออก
        if len(pts) > 3 and np.hypot(*(pts[-1] - pts[0])) <= 1e-6:
            pts = pts[:-1]
        self.points = pts
        self.count = len(pts)

        seg = np.hypot(*(np.roll(pts, -1, axis=0) - pts).T)   # seg[i] = |P[i+1] − P[i]| (วนลูป)
        spacing = float(np.median(seg[:-1])) if self.count > 1 else 0.0
        # ช่องว่างปิดลูปใหญ่ผิดปกติ = path เปิด (ไม่มองโค้ง/เป้าหมายข้ามรอยต่อ)
        self.loop = bool(seg[-1] <= max(loop_gap_max, 5.0 * spacing))
        self.seg = seg
        self.s = np.concatenate(([0.0], np.cumsum(seg[:-1])))  # ระยะสะสมของแต่ละจุด
        self.total = float(self.s[-1] + (seg[-1] if self.loop else 0.0))
        self.kappa = self._signed_curvature(max(int(curvature_half_window), 1))

    def _signed_curvature(self, half):
        """κ จากหน้าต่าง 2·half+1 จุด (ค่าเริ่มต้น 7 จุด) — บวก = โค้งซ้าย

        ทิศของครึ่งหน้าต่างหลัง vs ครึ่งหน้า หารด้วยระยะเฉลี่ยของสองครึ่ง
        ช่วงกว้างช่วยเฉลี่ย noise ของ waypoint เหมือนที่ pagard ทำ
        """
        n = self.count
        idx = np.arange(n)
        if self.loop:
            ia, ib = (idx - half) % n, (idx + half) % n
        else:
            ia, ib = np.maximum(idx - half, 0), np.minimum(idx + half, n - 1)
        back = self.points[idx] - self.points[ia]
        fwd = self.points[ib] - self.points[idx]
        len_back = np.hypot(back[:, 0], back[:, 1])
        len_fwd = np.hypot(fwd[:, 0], fwd[:, 1])
        turn = wrap_angle(
            np.arctan2(fwd[:, 1], fwd[:, 0]) - np.arctan2(back[:, 1], back[:, 0])
        )
        ds = 0.5 * (len_back + len_fwd)
        valid = (len_back > 1e-6) & (len_fwd > 1e-6)
        return np.where(valid, turn / np.maximum(ds, 1e-6), 0.0)

    def corridor(self, e_max):
        """ขอบกรอบ h₁ = 0 สองฝั่ง -> (ซ้าย, ขวา) อาร์เรย์ (n, 2)

        ทุกจุดของเส้นทางเลื่อนไป ±e_max ตามแนวตั้งฉาก (ทิศจากจุดก่อนหน้าไปจุดถัดไป)
        path ปิดลูป = ต่อจุดแรกท้ายเส้นให้ขอบปิดตามกัน
        """
        pts = self.points
        if self.loop:
            tangent = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
        else:
            tangent = np.gradient(pts, axis=0)
        length = np.hypot(tangent[:, 0], tangent[:, 1])
        valid = length > 1e-9
        if not valid.any():
            raise ValueError('path ไม่มีทิศทาง (ทุกจุดซ้อนกัน)')
        # จุดซ้อน (ไม่มีทิศ) ใช้ทิศของจุดใกล้สุดที่มีทิศ
        nearest = np.flatnonzero(valid)
        pick = nearest[np.clip(np.searchsorted(nearest, np.arange(self.count)), 0, len(nearest) - 1)]
        tangent = tangent[pick] / length[pick, None]
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))   # ชี้ซ้ายของเส้นทาง
        left = pts + e_max * normal
        right = pts - e_max * normal
        if self.loop:
            left = np.vstack((left, left[:1]))
            right = np.vstack((right, right[:1]))
        return left, right

    def clamp_index(self, index):
        index = int(index)
        if self.loop:
            return index % self.count
        return min(max(index, 0), self.count - 2)

    def kappa_at(self, index):
        return float(self.kappa[self.clamp_index(index)])

    def _ahead(self, index, max_dist):
        """จุดตั้งแต่ index ไปข้างหน้าไม่เกิน max_dist (m) -> (ids, ระยะจาก index)"""
        index = self.clamp_index(index)
        if self.loop:
            ids = (index + np.arange(self.count)) % self.count
            offsets = (self.s[ids] - self.s[index]) % self.total
        else:
            ids = np.arange(index, self.count)
            offsets = self.s[ids] - self.s[index]
        keep = max(int(np.searchsorted(offsets, max_dist, side='right')), 1)
        return ids[:keep], offsets[:keep]

    def preview(self, index, max_dist):
        """คืนฟังก์ชัน f(ระยะ) = |κ_path| สูงสุดตั้งแต่ตำแหน่งรถถึงระยะนั้นข้างหน้า"""
        ids, offsets = self._ahead(index, max_dist)
        running_max = np.maximum.accumulate(np.abs(self.kappa[ids]))

        def max_abs_curvature(distance):
            pos = np.searchsorted(offsets, np.asarray(distance, dtype=float), side='right') - 1
            return running_max[np.clip(pos, 0, len(running_max) - 1)]

        return max_abs_curvature

    def curve_speed_limit(self, index, ay_max, decel, max_dist):
        """v_curve ของ h₂ᶜ: √min_s(a_y,max/|κ(s)| + 2·decel·s) ในระยะ max_dist ข้างหน้า"""
        ids, offsets = self._ahead(index, max_dist)
        kappa = np.maximum(np.abs(self.kappa[ids]), 1e-6)
        return float(math.sqrt(np.min(ay_max / kappa + 2.0 * decel * offsets)))

    def goal_geometry(self, index, e_lat, theta_e, ld_grid):
        """จุดเป้าหมาย G ของ Pure Pursuit สำหรับทุก L_d -> (η, ความยาวคอร์ด PG)

        G = จุดแรกบนเส้นทางข้างหน้าที่ห่างจากเพลาหลังเป็นเส้นตรง L_d (สไลด์ 23–24)
        ตำแหน่งรถสร้างกลับจากสถานะ: กึ่งกลาง segment ที่ใกล้สุด + e_lat ตามแนวตั้งฉาก
        (pagard ส่งมาแค่หมายเลข segment จึงคลาดได้ไม่เกินครึ่งระยะ waypoint)
        """
        ld_grid = np.asarray(ld_grid, dtype=float)
        index = self.clamp_index(index)
        nxt = (index + 1) % self.count
        tangent = self.points[nxt] - self.points[index]
        norm = float(np.hypot(tangent[0], tangent[1]))
        tangent = tangent / norm if norm > 1e-9 else np.array([1.0, 0.0])
        normal = np.array([-tangent[1], tangent[0]])               # ชี้ซ้ายของเส้นทาง
        position = self.points[index] + 0.5 * norm * tangent + e_lat * normal
        heading = math.atan2(tangent[1], tangent[0]) + theta_e
        cos_h, sin_h = math.cos(heading), math.sin(heading)

        reach = float(ld_grid.max()) + abs(e_lat) + 2.0 * max(norm, 0.5)
        ids, _ = self._ahead(nxt, reach)
        rel = self.points[ids] - position
        body_x = rel[:, 0] * cos_h + rel[:, 1] * sin_h             # หน้ารถ
        body_y = -rel[:, 0] * sin_h + rel[:, 1] * cos_h            # ซ้ายรถ
        dist = np.hypot(body_x, body_y)

        beyond = dist[None, :] >= ld_grid[:, None]                 # (n_ld, n_pts)
        hit = beyond.any(axis=1)
        k = np.where(hit, beyond.argmax(axis=1), len(dist) - 1)    # ไม่เจอ = ใช้จุดสุดท้าย
        k0 = np.maximum(k - 1, 0)
        gap = dist[k] - dist[k0]
        ratio = np.where(
            hit & (k > 0) & (gap > 1e-9),
            (ld_grid - dist[k0]) / np.where(gap > 1e-9, gap, 1.0),
            1.0,
        )
        goal_x = body_x[k0] + ratio * (body_x[k] - body_x[k0])
        goal_y = body_y[k0] + ratio * (body_y[k] - body_y[k0])
        return np.arctan2(goal_y, goal_x), np.maximum(np.hypot(goal_x, goal_y), 1e-3)


def straight_path_goal(e_lat, theta_e, ld_grid):
    """จุดเป้าหมายบนเส้นทางที่ถือว่า "ตรง" เฉพาะที่ — ใช้ตอนไม่มี PathModel และในเทสต์

    ในเฟรมเส้นทาง รถอยู่ที่ (0, e_lat) หันทำมุม θ_e; จุดบนเส้นทางที่ระยะ s คือ (s, 0)
    เลือก s ให้ระยะเส้นตรงจากรถเท่ากับ L_d แล้วหมุนเข้า body frame (x หน้า, y ซ้าย)
    """
    ld_grid = np.asarray(ld_grid, dtype=float)
    cos_t, sin_t = math.cos(theta_e), math.sin(theta_e)
    along = np.sqrt(np.maximum(ld_grid ** 2 - e_lat ** 2, 0.0))
    goal_x = along * cos_t - e_lat * sin_t
    goal_y = -along * sin_t - e_lat * cos_t
    return np.arctan2(goal_y, goal_x), np.maximum(np.hypot(goal_x, goal_y), 1e-3)


# ======================================================================
#  สมการย่อย (แยกไว้ให้เทสต์เทียบตัวเลขในสไลด์ได้ทีละตัว)
# ======================================================================
def pure_pursuit_delta(eta, chord, wheelbase, max_steer_rad):
    """(3.3) δ = atan(2L·sinη / L_d) แล้วตัดที่ขอบเขตพวงมาลัยจริง"""
    delta = np.arctan2(2.0 * wheelbase * np.sin(eta), chord)
    return np.clip(delta, -max_steer_rad, max_steer_rad)


def lookahead_reference(params, v, kappa):
    """(4.1)–(4.4) -> (L_d,ref ก่อนปรับ, L_d,ref^eff)"""
    ld_ref = params.ld_min + params.tau * max(v, 0.0)
    shaped = ld_ref / (1.0 + params.k_adapt * max(kappa, 0.0))
    shaped *= math.sqrt(AY_REF / params.ay_max)
    return ld_ref, min(max(shaped, params.ld_min), params.ld_max)


def hocbf_h1(params, e_lat, e_lat_dot, v, kappa_path, tan_delta):
    """(2.1)–(2.8) -> dict ของ h₁, ḣ₁, α₁, ψ₁ และค่า ψ̇₁ + α₁ψ₁ ของแต่ละ tanδ"""
    alpha1 = v / params.e_max
    h1 = params.e_max ** 2 - e_lat ** 2
    h1_dot = -2.0 * e_lat * e_lat_dot
    e_lat_ddot = v * v * (np.asarray(tan_delta, dtype=float) / params.wheelbase - kappa_path)
    h1_ddot = -2.0 * e_lat_dot ** 2 - 2.0 * e_lat * e_lat_ddot
    return {
        'alpha1': alpha1,
        'h1': h1,
        'h1_dot': h1_dot,
        'psi1': h1_dot + alpha1 * h1,
        'condition': h1_ddot + 2.0 * alpha1 * h1_dot + alpha1 ** 2 * h1,
    }


# ======================================================================
#  ตัวแก้ CBF-QP (Grid Search บน v × L_d)
# ======================================================================
def curve_horizon(params, v_top):
    """ระยะที่ h₂ᶜ ต้องมองไปข้างหน้า: ชะลอจาก v_top ลงถึงหยุดด้วย curve_decel"""
    return v_top * v_top / (2.0 * params.curve_decel) + params.ld_max


def solve(params, v_grid, ld_grid, e_lat, theta_e, v, kappa_path, v_des, eta, chord, preview,
          v_curve=math.inf):
    """หนึ่งรอบควบคุม -> dict ของ u* = [v*, L_d*] และตัวแปรวินิจฉัย

    eta, chord : มุม η และความยาวคอร์ดไปจุดเป้าหมาย ของแต่ละค่าใน ld_grid
    preview    : f(ระยะ) -> |κ_path| สูงสุดในระยะนั้นข้างหน้า (รับ array ได้)
    v_curve    : PathModel.curve_speed_limit() ของตำแหน่งนี้ (∞ = ไม่มีโค้งข้างหน้า)
    """
    L = params.wheelbase
    v = max(float(v), 0.0)
    ld = np.asarray(ld_grid, dtype=float)

    # ---- Pure Pursuit ของทุก L_d (ไม่ขึ้นกับ v) ----
    delta = pure_pursuit_delta(eta, chord, L, params.max_steer_rad)
    tan_delta = np.tan(delta)
    kappa_cmd = np.abs(tan_delta) / L

    # ---- L_d,ref^eff: เป้าหมายใน J เท่านั้น ไม่ใช่ hard constraint ----
    ld_ref_raw = params.ld_min + params.tau * v
    kappa_ref = float(preview(max(ld_ref_raw, v / params.alpha2)))
    ld_ref = lookahead_reference(params, v, kappa_ref)[1]

    # ---- h₁: HOCBF สองชั้น — สถานะที่วัดได้ + δ ของ candidate ----
    barrier = hocbf_h1(params, e_lat, v * math.sin(theta_e), v, kappa_path, tan_delta)
    hocbf = barrier['condition']                                   # (n_ld,)

    use_h1 = params.use_h1 and not params.pure_pursuit_only
    use_h2 = params.use_h2 and not params.pure_pursuit_only
    # ---- h₂: เช็คตรง ๆ ด้วย v ของ candidate ----
    if params.optimize_speed:
        v_cand = np.asarray(v_grid, dtype=float)
    elif use_h2:
        # ความเร็วจากไฟล์ + h₂: เลือกได้ตั้งแต่ v_min ถึง v_des ของไฟล์ (ไม่ตัดที่ v_max)
        top = max(float(v_des), params.v_min)
        v_cand = np.unique(np.append(np.arange(params.v_min, top, params.v_step), top))
    else:
        v_cand = np.array([min(max(float(v_des), params.v_min), params.v_max)])
    v_col = v_cand.reshape(-1, 1)                                  # (n_v, 1)
    look = np.maximum(ld[None, :], v_col / params.alpha2)          # ระยะมองโค้งล่วงหน้า
    kappa_hat = np.maximum(kappa_cmd[None, :], preview(look))      # (n_v, n_ld)
    a_y = v_col ** 2 * kappa_hat
    h2 = params.ay_max - a_y
    # ---- h₂ᶜ: ชะลอทันก่อนถึงโค้งข้างหน้า (ไม่ขึ้นกับ L_d) ----
    h2_curve = (v_curve ** 2 if math.isfinite(v_curve) else math.inf) - v_col ** 2 + np.zeros_like(h2)

    if params.pure_pursuit_only:
        # กริดของโหมดนี้มีแค่ fixed_ld (lookahead_grid) — ถ้าได้กริดเต็มมาก็เลือกตัวที่ใกล้สุด
        cost = (ld[None, :] - params.fixed_ld) ** 2 + np.zeros_like(h2)
    else:
        cost = params.w_ld * (ld[None, :] - ld_ref) ** 2 + np.zeros_like(h2)
    if len(v_cand) > 1:
        # w_v = 0 ก็ยังต้องเลือกความเร็วสูงสุดที่ผ่าน ไม่ใช่ตัวไหนก็ได้
        cost = cost + max(params.w_v, 1e-3) * (v_col - v_des) ** 2

    # ข้อจำกัดที่ปิดไว้ยังคำนวณค่าเหมือนเดิม (ดูใน log ได้ว่าถ้าเปิดจะผ่านไหม) แต่ไม่ตัด candidate
    feasible_mask = np.ones(h2.shape, dtype=bool)
    violation = np.zeros(h2.shape)
    if use_h1:
        feasible_mask &= hocbf[None, :] >= -FEAS_TOL
        violation = violation + np.maximum(-hocbf, 0.0)[None, :]
    if use_h2:
        feasible_mask &= (h2 >= -FEAS_TOL) & (h2_curve >= -FEAS_TOL)
        violation = violation + np.maximum(-h2, 0.0) + np.maximum(-h2_curve, 0.0)
    feasible = bool(feasible_mask.any())
    if feasible:
        selection = np.where(feasible_mask, cost, np.inf)
    else:
        # QP infeasible -> จุดที่ละเมิดข้อจำกัดน้อยที่สุด (สไลด์ 25) แล้วค่อยดู J
        if params.optimize_speed:
            slow = np.broadcast_to(v_col <= params.recovery_v_max + 1e-9, violation.shape)
            if slow.any():
                violation = np.where(slow, violation, np.inf)
        selection = np.where(violation <= violation.min() + 1e-9, cost, np.inf)

    i_v, i_ld = np.unravel_index(int(np.argmin(selection)), selection.shape)
    kappa_star = float(kappa_hat[i_v, i_ld])
    return {
        'v_star': float(v_cand[i_v]),
        'ld_star': float(ld[i_ld]),
        'j': float(cost[i_v, i_ld]),
        'feasible': feasible,
        'feasible_count': int(feasible_mask.sum()),
        'ld_ref_raw': float(ld_ref_raw),
        'ld_ref': float(ld_ref),
        'eta': float(np.asarray(eta)[i_ld]),
        'delta': float(delta[i_ld]),
        'kappa_cmd': float(kappa_cmd[i_ld]),
        'kappa_hat': kappa_star,
        'a_y': float(a_y[i_v, i_ld]),
        'h2': float(h2[i_v, i_ld]),
        'v_limit': math.sqrt(params.ay_max / kappa_star) if kappa_star > 1e-9 else math.inf,
        'v_curve': float(v_curve),
        'h2_curve': float(h2_curve[i_v, i_ld]),
        # เพดานที่ pagard ใช้กับความเร็วจากไฟล์: มีค่าเฉพาะตอนนั้นเปิด h₂ อยู่
        'speed_cap': float(v_cand[i_v]) if (not params.optimize_speed and use_h2) else math.inf,
        'alpha1': float(barrier['alpha1']),
        'h1': float(barrier['h1']),
        'h1_dot': float(barrier['h1_dot']),
        'psi1': float(barrier['psi1']),
        'hocbf': float(hocbf[i_ld]),
    }
