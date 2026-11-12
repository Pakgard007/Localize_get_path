"""เทียบ cbf_qp_core กับตัวเลขในสไลด์ "CBF-QP Tuning Lab" (preset ค่าเริ่มต้น)

รัน:  python3 -m pytest src/nav_path_launch/test -q
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from nav_path_launch.cbf_qp_core import (  # noqa: E402
    CbfQpParams,
    PathModel,
    hocbf_h1,
    lookahead_grid,
    lookahead_reference,
    make_grid,
    pure_pursuit_delta,
    solve,
    straight_path_goal,
)

P = CbfQpParams()
V_GRID = make_grid(P.v_min, P.v_max, P.v_step)
LD_GRID = make_grid(P.ld_min, P.ld_max, P.ld_step)


def constant_preview(kappa):
    return lambda distance: np.full(np.shape(distance), kappa)


def solve_on_path(params=P, v=2.8, kappa=0.05, v_des=3.0, e_lat=0.0, theta_e=0.0):
    """สมมติฐานของสไลด์ 26: κ̂ ≈ κ_path ทั้งกริด"""
    eta, chord = straight_path_goal(e_lat, theta_e, LD_GRID)
    return solve(
        params, V_GRID, LD_GRID, e_lat, theta_e, v, kappa, v_des,
        eta, chord, constant_preview(abs(kappa)),
    )


# ---------- กริด (สไลด์ 25) ----------
def test_grid_matches_slide_25():
    assert len(V_GRID) == 77 and len(LD_GRID) == 61
    assert V_GRID[1] - V_GRID[0] == pytest.approx(0.05)
    assert LD_GRID[1] - LD_GRID[0] == pytest.approx(0.1)
    assert (V_GRID[0], V_GRID[-1]) == (0.0, 3.8)
    assert (LD_GRID[0], LD_GRID[-1]) == (2.0, 8.0)


# ---------- h₁ / HOCBF (สไลด์ 15–16) ----------
def test_h1_case_a_deep_inside_corridor():
    b = hocbf_h1(P, e_lat=0.200, e_lat_dot=-0.18, v=2.80, kappa_path=0.05, tan_delta=0.0)
    assert b['alpha1'] == pytest.approx(3.50)
    assert b['h1'] == pytest.approx(0.6000)
    assert b['h1_dot'] == pytest.approx(0.0720)
    assert b['psi1'] == pytest.approx(2.1720)
    gamma = float(b['condition'])                   # ที่ tanδ = 0 เงื่อนไขเหลือ Γ พอดี
    assert gamma == pytest.approx(7.946, abs=1e-3)
    bound = gamma * P.wheelbase / (2 * 2.80 ** 2 * 0.200)
    assert bound == pytest.approx(3.978, abs=2e-3)
    assert math.atan(bound) > P.max_steer_rad       # constraint inactive


def test_h1_case_b_near_edge_drifting_out():
    b = hocbf_h1(P, e_lat=0.75, e_lat_dot=0.10, v=2.80, kappa_path=0.05, tan_delta=0.0)
    assert b['h1'] == pytest.approx(0.0775)
    assert b['h1_dot'] == pytest.approx(-0.150)
    gamma = float(b['condition'])
    assert gamma == pytest.approx(0.4674, abs=1e-3)
    bound = gamma * P.wheelbase / (2 * 2.80 ** 2 * 0.75)
    assert bound == pytest.approx(0.0624, abs=1e-4)

    def condition(tan_delta):
        return float(hocbf_h1(P, 0.75, 0.10, 2.80, 0.05, tan_delta)['condition'])

    assert condition(P.wheelbase * 0.05) < 0        # เลี้ยวตามโค้ง (tanδ = Lκ) ถูกห้าม
    assert condition(bound - 1e-3) > 0 > condition(bound + 1e-3)


def test_h1_right_side_gives_lower_bound():
    # e_lat < 0: อสมการกลับทิศ -> ห้ามเลี้ยวขวาเกินขีด (สไลด์ 14)
    def condition(tan_delta):
        return float(hocbf_h1(P, -0.75, -0.10, 2.80, -0.05, tan_delta)['condition'])

    assert condition(-0.0785) < 0 < condition(0.0)


def test_hocbf_is_trivially_satisfied_at_standstill():
    b = hocbf_h1(P, 0.75, 0.0, 0.0, 0.05, np.array([-0.5, 0.0, 0.5]))
    assert np.allclose(b['condition'], 0.0)


# ---------- h₂ / Pure Pursuit (สไลด์ 19, 24) ----------
def test_pure_pursuit_and_h2_example():
    delta = float(pure_pursuit_delta(0.08, 3.60, P.wheelbase, P.max_steer_rad))
    assert delta == pytest.approx(0.0696, abs=1e-4)
    kappa_cmd = abs(math.tan(delta)) / P.wheelbase
    assert kappa_cmd == pytest.approx(0.0444, abs=1e-4)
    kappa_hat = max(kappa_cmd, 0.05)
    assert P.ay_max - 2.80 ** 2 * kappa_hat == pytest.approx(0.008)
    assert math.sqrt(P.ay_max / kappa_hat) == pytest.approx(2.828, abs=1e-3)

    delta_star = float(pure_pursuit_delta(0.08, 3.50, P.wheelbase, P.max_steer_rad))
    assert delta_star == pytest.approx(0.0716, abs=1e-4)
    assert math.degrees(delta_star) == pytest.approx(4.10, abs=0.01)


def test_pure_pursuit_is_clamped_to_steering_limit():
    delta = pure_pursuit_delta(np.array([1.5, -1.5]), 2.0, P.wheelbase, P.max_steer_rad)
    assert np.allclose(delta, [0.8, -0.8])


# ---------- L_d,ref (สไลด์ 21–22) ----------
def test_lookahead_reference_chain():
    raw, eff = lookahead_reference(P, v=2.80, kappa=0.05)
    assert raw == pytest.approx(3.96)
    assert eff == pytest.approx(3.96 / 1.15)
    assert eff == pytest.approx(3.44, abs=5e-3)


def test_lookahead_reference_comfort_scaling_and_clamp():
    _, base = lookahead_reference(P, 2.80, 0.05)
    _, stricter = lookahead_reference(P.updated(ay_max=0.2), 2.80, 0.05)
    assert stricter == pytest.approx(base * math.sqrt(2.0))        # ตั้งต่ำกว่า 0.4 -> ยืด
    assert lookahead_reference(P, 0.0, 5.0)[1] == P.ld_min
    assert lookahead_reference(P.updated(tau=5.0), 3.8, 0.0)[1] == P.ld_max


# ---------- Grid Search (สไลด์ 9, 25, 26) ----------
def test_grid_search_matches_slide_25_real_grid():
    sol = solve_on_path()
    assert sol['feasible']
    assert sol['v_star'] == pytest.approx(2.80)
    assert sol['ld_star'] == pytest.approx(3.40)
    assert sol['j'] == pytest.approx(0.0405, abs=1e-4)
    assert sol['h2'] == pytest.approx(0.008, abs=1e-6)
    assert sol['v_limit'] == pytest.approx(2.828, abs=1e-3)
    assert sol['ld_ref_raw'] == pytest.approx(3.96)
    assert sol['ld_ref'] == pytest.approx(3.44, abs=5e-3)


def test_hand_grid_candidates_of_slide_26():
    def cost(v, ld):
        return P.w_v * (v - 3.00) ** 2 + P.w_ld * (ld - 3.44) ** 2

    def h2(v):
        return P.ay_max - 0.05 * v ** 2

    table = {'A': (2.40, 3.00), 'B': (2.60, 3.50), 'C': (2.80, 3.50),
             'D': (2.80, 4.00), 'E': (3.00, 3.50)}
    assert cost(*table['A']) == pytest.approx(0.4181, abs=1e-4)
    assert cost(*table['B']) == pytest.approx(0.1611, abs=1e-4)
    assert cost(*table['C']) == pytest.approx(0.04108, abs=1e-5)
    assert cost(*table['D']) == pytest.approx(0.1341, abs=1e-4)
    assert h2(3.00) == pytest.approx(-0.050) and h2(2.80) == pytest.approx(0.008)
    passing = {k: cost(*c) for k, c in table.items() if h2(c[0]) >= 0}
    assert min(passing, key=passing.get) == 'C' and 'E' not in passing


def test_w_ld_zero_lets_lookahead_float():
    sol = solve_on_path(P.updated(w_ld=0.0))
    assert sol['v_star'] == pytest.approx(2.80)


def test_optimize_speed_false_with_h2_caps_the_file_speed():
    # ความเร็วจากไฟล์ + h₂: ใต้เพดานแรงเหวี่ยงได้ v_des ตรง ๆ, เกินก็ลดลงมาให้ผ่าน h₂
    params = P.updated(optimize_speed=False)
    sol = solve_on_path(params, v_des=2.0)
    assert sol['v_star'] == pytest.approx(2.0) and sol['feasible'] and sol['speed_cap'] == pytest.approx(2.0)
    sol = solve_on_path(params, v_des=3.0)                    # √(0.4/0.05) = 2.83 m/s
    assert sol['feasible'] and sol['h2'] >= 0
    assert 2.7 <= sol['v_star'] <= math.sqrt(P.ay_max / 0.05) + 1e-9
    assert sol['speed_cap'] == pytest.approx(sol['v_star'])


def test_optimize_speed_false_without_h2_pins_v_and_sets_no_cap():
    sol = solve_on_path(P.updated(optimize_speed=False, use_h2=False), v_des=2.0)
    assert sol['v_star'] == pytest.approx(2.0) and math.isinf(sol['speed_cap'])
    sol = solve_on_path(P.updated(optimize_speed=False, pure_pursuit_only=True), v_des=2.0)
    assert math.isinf(sol['speed_cap'])


def test_file_speed_above_v_max_is_not_cut_by_h2_on_a_straight():
    sol = solve_on_path(P.updated(optimize_speed=False), kappa=0.0, v_des=5.2)
    assert sol['v_star'] == pytest.approx(5.2) and sol['speed_cap'] == pytest.approx(5.2)


def test_zero_speed_is_always_a_feasible_candidate():
    sol = solve_on_path(kappa=5.0)                  # โค้งแคบมาก
    assert sol['feasible'] and sol['v_star'] <= math.sqrt(P.ay_max / 5.0) + 1e-9


def test_infeasible_falls_back_to_least_violation_and_slows_down():
    # หลุดกรอบไปแล้วและกำลังเบี่ยงออกเร็ว: ไม่มี L_d ไหนพอ
    eta, chord = straight_path_goal(1.6, 0.6, LD_GRID)
    sol = solve(P, V_GRID, LD_GRID, 1.6, 0.6, 2.5, 0.0, 2.5, eta, chord, constant_preview(0.0))
    assert not sol['feasible'] and sol['feasible_count'] == 0
    assert sol['hocbf'] < 0 and sol['h1'] < 0
    assert sol['v_star'] <= P.recovery_v_max + 1e-9
    assert sol['delta'] < 0                         # อยู่ซ้าย -> ต้องเลี้ยวขวากลับเข้าเส้น


def test_near_edge_hocbf_removes_candidates_that_steer_outward():
    # ใกล้ขอบซ้ายบนโค้งซ้าย: L_d ยาวเล็งเข้าโค้ง (δ มาก) ต้องถูกตัด เหลือแต่ L_d ที่ดึงกลับ
    radius = 20.0
    angles = np.arange(0.0, 2 * np.pi, 0.3 / radius)
    path = PathModel(np.c_[radius * np.cos(angles), radius * np.sin(angles)])
    e_lat, theta_e = 0.75, 0.10                     # กำลังเบี่ยงออก ~5.7°
    eta, chord = path.goal_geometry(0, e_lat, theta_e, LD_GRID)
    delta = pure_pursuit_delta(eta, chord, P.wheelbase, P.max_steer_rad)
    cond = hocbf_h1(
        P, e_lat, 2.8 * math.sin(theta_e), 2.8, path.kappa_at(0), np.tan(delta)
    )['condition']
    assert (cond >= 0).any() and (cond < 0).any()
    assert delta[cond >= 0].max() < delta[cond < 0].min()
    assert LD_GRID[cond >= 0].max() < LD_GRID[cond < 0].min()   # ผ่านเฉพาะ L_d สั้น
    sol = solve(P, V_GRID, LD_GRID, e_lat, theta_e, 2.8, path.kappa_at(0), 2.5,
                eta, chord, path.preview(0, 10.0))
    assert sol['feasible'] and sol['hocbf'] >= 0
    assert sol['ld_star'] == pytest.approx(LD_GRID[cond >= 0].max())


# ---------- PathModel ----------
def circle(radius, clockwise=False, spacing=0.3):
    angles = np.arange(0.0, 2 * np.pi, spacing / radius)
    if clockwise:
        angles = -angles
    return np.c_[radius * np.cos(angles), radius * np.sin(angles)]


def test_curvature_sign_and_magnitude():
    left = PathModel(circle(20.0))
    right = PathModel(circle(20.0, clockwise=True))
    assert left.loop and right.loop
    assert np.allclose(left.kappa, 0.05, atol=1e-3)
    assert np.allclose(right.kappa, -0.05, atol=1e-3)
    assert left.total == pytest.approx(2 * np.pi * 20.0, rel=1e-2)


def test_closed_path_with_repeated_first_point_keeps_indices():
    pts = circle(20.0)
    closed = PathModel(np.vstack([pts, pts[:1]]))
    assert closed.count == len(pts)
    assert closed.clamp_index(len(pts)) == 0


def test_open_path_does_not_wrap():
    line = PathModel(np.c_[np.arange(0.0, 30.0, 0.3), np.zeros(100)])
    assert not line.loop and np.allclose(line.kappa, 0.0)
    assert line.clamp_index(500) == line.count - 2
    eta, chord = line.goal_geometry(95, 0.0, 0.0, LD_GRID)   # ใกล้ปลายทาง: ใช้จุดสุดท้าย
    assert np.allclose(eta, 0.0, atol=1e-9) and chord.max() < 2.0


def test_preview_looks_only_as_far_as_requested():
    # ทางตรง 20 m แล้วเข้าโค้งรัศมี 10 m
    straight = np.c_[np.arange(0.0, 20.0, 0.3), np.zeros(67)]
    arc_angles = np.arange(0.0, np.pi, 0.03)
    arc = np.c_[straight[-1, 0] + 10.0 * np.sin(arc_angles), 10.0 - 10.0 * np.cos(arc_angles)]
    path = PathModel(np.vstack([straight, arc[1:]]), loop_gap_max=0.0)
    assert not path.loop
    preview = path.preview(0, 40.0)
    assert preview(5.0) == pytest.approx(0.0, abs=1e-6)
    assert preview(30.0) == pytest.approx(0.1, abs=5e-3)
    assert np.all(np.diff(preview(np.linspace(0.0, 40.0, 50))) >= 0)


def test_goal_geometry_matches_straight_line_model():
    line = PathModel(np.c_[np.arange(0.0, 60.0, 0.3), np.zeros(200)])
    for e_lat, theta_e in [(0.0, 0.0), (0.4, 0.0), (-0.4, 0.1), (0.3, -0.2)]:
        eta, chord = line.goal_geometry(20, e_lat, theta_e, LD_GRID)
        eta_ref, chord_ref = straight_path_goal(e_lat, theta_e, LD_GRID)
        # คอร์ดจากการ interpolate ระหว่าง waypoint คลาดได้ระดับ 0.1 mm
        assert np.allclose(chord, LD_GRID, atol=1e-3) and np.allclose(chord_ref, LD_GRID)
        assert np.allclose(eta, eta_ref, atol=0.02)


def test_corridor_edges_sit_e_max_either_side():
    # วงกลมทวนเข็ม: ซ้ายของเส้นทาง = ด้านในวง
    left, right = PathModel(circle(20.0)).corridor(0.6)
    assert np.allclose(np.hypot(*left.T), 19.4, atol=1e-3)
    assert np.allclose(np.hypot(*right.T), 20.6, atol=1e-3)
    assert np.allclose(left[0], left[-1]) and np.allclose(right[0], right[-1])   # ปิดลูป

    # ทางตรงตามแกน x มีจุดซ้อนกลางทาง: ใช้ทิศของจุดข้าง ๆ และไม่ต่อปลายเข้าหาต้น
    pts = np.c_[np.arange(0.0, 30.0, 0.3), np.zeros(100)]
    line = PathModel(np.vstack([pts[:50], pts[49:]]), loop_gap_max=0.0)
    left, right = line.corridor(0.8)
    assert len(left) == line.count
    assert np.allclose(left[:, 1], 0.8) and np.allclose(right[:, 1], -0.8)
    assert np.allclose(left[:, 0], line.points[:, 0])


def test_goal_point_sign_convention():
    eta, _ = straight_path_goal(0.5, 0.0, LD_GRID)       # รถอยู่ซ้าย -> เป้าอยู่ขวา
    assert np.all(eta < 0)
    eta, _ = straight_path_goal(0.0, -0.2, LD_GRID)      # หัวรถเบนขวา -> เป้าอยู่ซ้าย
    assert np.all(eta > 0)


# ---------- สวิตช์ของการทดลอง: เปิด/ปิด h₁ h₂ และ Pure Pursuit ล้วน ----------
def test_h1_off_no_longer_rejects_leaving_the_corridor():
    # กรณีเดียวกับ test_infeasible_...: เปิด h₁ ไม่มีคู่ไหนผ่าน ปิดแล้วเหลือแค่ h₂
    # (รถเลี้ยวแรงกลับเข้าเส้น κ̂ = |tanδ|/L สูง h₂ จึงยังจำกัดความเร็วอยู่)
    eta, chord = straight_path_goal(1.6, 0.6, LD_GRID)
    args = (V_GRID, LD_GRID, 1.6, 0.6, 2.5, 0.0, 2.5, eta, chord, constant_preview(0.0))
    sol = solve(P.updated(use_h1=False), *args)
    assert sol['feasible'] and sol['feasible_count'] > 0
    assert sol['hocbf'] < 0 and sol['h2'] >= 0      # h₁ ยังคำนวณไว้ให้ดูใน log
    assert solve(P.updated(use_h1=False, use_h2=False), *args)['v_star'] == pytest.approx(2.5)


def test_h2_off_lets_v_des_above_the_lateral_limit():
    params = P.updated(optimize_speed=False, use_h2=False)
    sol = solve_on_path(params, v_des=3.0)
    assert sol['feasible'] and sol['v_star'] == pytest.approx(3.0) and sol['h2'] < 0


def test_both_off_is_the_unconstrained_minimum_of_j():
    sol = solve_on_path(P.updated(use_h1=False, use_h2=False), v_des=3.0)
    assert sol['v_star'] == pytest.approx(3.0)
    assert sol['ld_star'] == pytest.approx(LD_GRID[np.argmin(abs(LD_GRID - sol['ld_ref']))])


def test_pure_pursuit_only_uses_fixed_lookahead_and_v_des():
    params = P.updated(pure_pursuit_only=True, fixed_ld=5.3)
    grid = lookahead_grid(params)
    assert list(grid) == [5.3]
    # สถานะที่ QP ไม่มีคำตอบ: โหมดนี้ไม่มีข้อจำกัด จึงไม่ชะลอและไม่เปลี่ยน L_d
    eta, chord = straight_path_goal(1.6, 0.6, grid)
    sol = solve(params, V_GRID, grid, 1.6, 0.6, 2.5, 0.0, 2.5, eta, chord, constant_preview(0.0))
    assert sol['feasible'] and sol['ld_star'] == 5.3 and sol['v_star'] == pytest.approx(2.5)
    expected = pure_pursuit_delta(eta, chord, P.wheelbase, P.max_steer_rad)[0]
    assert sol['delta'] == pytest.approx(expected)


def test_pure_pursuit_only_picks_fixed_ld_even_from_a_full_grid():
    sol = solve_on_path(P.updated(pure_pursuit_only=True, fixed_ld=6.0, w_ld=0.0), kappa=0.5)
    assert sol['ld_star'] == pytest.approx(6.0) and sol['feasible']


def test_lookahead_grid_is_the_usual_grid_otherwise():
    assert np.allclose(lookahead_grid(P), LD_GRID)


# ---------- พารามิเตอร์ ----------
@pytest.mark.parametrize('bad', [
    {'e_max': 0.0}, {'ay_max': -1.0}, {'alpha2': 0.0}, {'w_v': -0.1},
    {'ld_min': 9.0}, {'v_max': 0.0}, {'tau': float('nan')}, {'fixed_ld': 0.0},
])
def test_invalid_parameters_are_rejected(bad):
    with pytest.raises(ValueError):
        P.updated(**bad)


# ---------- h₂ᶜ: ชะลอทันก่อนถึงโค้งข้างหน้า ----------
def straight_then_arc(radius=10.0, straight=30.0, step=0.3):
    xs = np.arange(0.0, straight, step)
    pts = [(x, 0.0) for x in xs]
    angles = np.arange(step / radius, math.pi / 2, step / radius)
    pts += [(straight + radius * math.sin(a), radius - radius * math.cos(a)) for a in angles]
    return PathModel(pts, curvature_half_window=3, loop_gap_max=0.5)


def test_curve_speed_limit_anticipates_the_arc():
    path = straight_then_arc(radius=10.0, straight=30.0)
    b, ay = 0.5, 0.4
    at_arc = path.curve_speed_limit(int(30.0 / 0.3) + 5, ay, b, 60.0)
    assert at_arc == pytest.approx(math.sqrt(ay * 10.0), rel=0.05)      # √(a_y·R) = 2 m/s
    before = path.curve_speed_limit(int(15.0 / 0.3), ay, b, 60.0)       # 15 m ก่อนเข้าโค้ง
    assert before == pytest.approx(math.sqrt(ay * 10.0 + 2 * b * 15.0), rel=0.06)
    assert path.curve_speed_limit(0, ay, b, 5.0) > 6.0                  # มองไม่ถึงโค้ง = ไม่จำกัด


def test_h2_curve_slows_down_before_a_curve_in_both_speed_modes():
    for optimize in (True, False):
        params = P.updated(optimize_speed=optimize)
        eta, chord = straight_path_goal(0.0, 0.0, LD_GRID)
        sol = solve(params, V_GRID, LD_GRID, 0.0, 0.0, 3.0, 0.0, 3.0, eta, chord,
                    constant_preview(0.0), v_curve=2.2)
        assert sol['feasible'] and sol['v_star'] <= 2.2 + 1e-9 and sol['h2_curve'] >= -1e-9
        assert sol['v_star'] >= 2.1


def test_h2_off_ignores_the_curve_ahead_too():
    eta, chord = straight_path_goal(0.0, 0.0, LD_GRID)
    sol = solve(P.updated(optimize_speed=False, use_h2=False), V_GRID, LD_GRID, 0.0, 0.0, 3.0,
                0.0, 3.0, eta, chord, constant_preview(0.0), v_curve=1.0)
    assert sol['v_star'] == pytest.approx(3.0) and sol['h2_curve'] < 0
