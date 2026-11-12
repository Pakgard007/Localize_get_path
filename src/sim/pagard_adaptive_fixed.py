#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped, Twist, Point, Quaternion, TransformStamped, TwistWithCovarianceStamped
from std_msgs.msg import Int32, Bool, Float32, Float64, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class WaypointFollowerCmdVelIMU(Node):
    def __init__(self):
        super().__init__('waypoint_follower_cmdvel_imu')

        # =========================
        # Parameters
        # =========================
        # Topic ที่ node นี้รับ/ส่งข้อมูล
        self.declare_parameter('path_topic', '/nav_path')  # path/waypoint ที่ต้องให้รถวิ่งตาม
        self.declare_parameter('pose_topic', '/pcl_pose')  # pose ตำแหน่งปัจจุบันของรถ
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')  # topic ที่ส่งคำสั่งความเร็ว/เลี้ยว
        # DRY RUN — ยังไม่ปล่อยคำสั่งไปที่รถจริง
        # ตอน dry_run=True จะ publish Twist ไปที่ dry_run_cmd_vel_topic แทน /cmd_vel
        # ซึ่ง steering_node / ros_can_bridge ไม่ได้ subscribe อยู่ -> รถไม่ขยับ
        # แต่ pipeline ทั้งเส้น (CBF-QP -> pure pursuit -> Twist) ยังเดินครบให้ดู/เก็บ log ได้
        # พร้อมออกถนนจริงเมื่อไหร่ ตั้ง dry_run:=false อย่างเดียว
        #
        # 2026-09-09: เปลี่ยน default เป็น False = คำสั่งออก /cmd_vel จริง
        # เพราะ node นี้ถูกรันด้วย `python3 pagard_adaptive_fixed.py` เปล่า ๆ
        # (ดู ~/Desktop/Untitled Document 4) ไม่มี --ros-args มาทับ default เลย
        # ค่า True จึงทำให้ /cmd_vel ไม่มี publisher ตลอดกาล
        #
        # หมายเหตุ: publisher ถูกสร้างครั้งเดียวตอน start (บรรทัด ~1006) จากค่าที่
        # ตัดสินไว้ด้านล่าง และ node นี้ไม่มี add_on_set_parameters_callback
        # -> `ros2 param set dry_run` ตอนรันอยู่ไม่มีผล ต้อง restart เท่านั้น
        #
        # กลับไปโหมดปลอดภัย: แก้เป็น True หรือรันด้วย -p dry_run:=true
        self.declare_parameter('dry_run', False)
        self.declare_parameter('dry_run_cmd_vel_topic', '/cmd_vel_preview')
        self.declare_parameter('target_goal_topic', '/target_goal')
        self.declare_parameter('target_goal_timeout_sec', 2.0)
        self.declare_parameter('path_coordinate_limit', 500.0)  # ถ้า |x| หรือ |y| เกินค่านี้ให้หยุดรถทันที

        # Path, frame, และเงื่อนไขการถึงเป้าหมาย
        self.declare_parameter('lookahead_distance', 2.0)  # อ้างอิงสำหรับ CTE slowdown threshold (m)
        self.declare_parameter('curvature_topic', '/curvature')
        self.declare_parameter('velocity_topic', '/velocity')
        self.declare_parameter('lookahead_topic', '/lookahead_distance')
        self.declare_parameter('arrival_tolerance', 5.0)  # ห่างจากจุดสุดท้ายน้อยกว่านี้ = ถึงเป้าหมาย (m)
        self.declare_parameter('world_frame', 'map')  # frame ใหญ่ของโลก/map
        self.declare_parameter('output_frame', 'map')  # frame ที่ใช้ publish path, pose, marker
        self.declare_parameter('base_frame', 'base_link')  # frame ตัวรถ ใช้ publish TF

        # ค่าพื้นฐานของรถและความเร็ว
        self.declare_parameter('wheelbase', 1.57)  # ระยะฐานล้อ ใช้คำนวณ pure pursuit (m)
        self.declare_parameter('binary_speed_mode', False)  # ปล่อยให้ controller ชะลอความเร็วตอนเข้าโค้งได้
        self.declare_parameter('forward_speed', 2.0)  # ความเร็วสูงสุด/ความเร็วปกติเมื่อตาม path ได้ดี
        self.declare_parameter('slow_speed', 1.6)  # ความเร็วชะลอที่ยังมีแรงพอให้รถไม่อืดเกินไป
        self.declare_parameter('turn_speed', 1.6)  # ความเร็วเข้าโค้งขั้นต่ำที่ยังรักษาแรงฉุดได้
        self.declare_parameter('stop_speed', 0.0)  # compatibility: โหมดรถไม่มีเบรกจะปล่อยคันเร่งเป็นศูนย์แทน
        self.declare_parameter('slowdown_distance', 5.0)  # ระยะก่อนถึง goal ที่เริ่มลดความเร็ว (m)
        self.declare_parameter('speed_command_scale', 1.0)  # แปลงความเร็วจริง (m/s) ไปเป็นคำสั่งบน cmd_vel
        self.declare_parameter('speed_command_offset', 0.0)  # offset เพิ่มให้คำสั่งความเร็วตอนเริ่มวิ่ง
        self.declare_parameter('speed_command_max', 4.1)  # จำกัดคำสั่งความเร็วสูงสุดที่ส่งออก
        # 4.1 = v_target สูงสุดของ loop2_with_speed.csv (3.794) + speed_feedback_max_delta (0.25)
        # เผื่อให้ feedback ดันคำสั่งขึ้นได้เต็มช่วงโดยไม่โดนตัดยอดโปรไฟล์
        self.declare_parameter('speed_feedback_up_kp', 0.12)  # ถ้าวัดได้ช้ากว่า target จะเพิ่มคำสั่งด้วย gain นี้
        self.declare_parameter('speed_feedback_down_kp', 0.30)  # ถ้าวัดได้เร็วกว่า target จะลดคำสั่งด้วย gain นี้
        self.declare_parameter('speed_feedback_max_delta', 0.25)  # จำกัด correction ของ feedback ต่อรอบ
        # จำกัดอัตราเร่ง/หน่วงของคำสั่งความเร็วที่ส่งออกจริง กันไม่ให้กระโดดเร็วเกินไป
        # (เช่น ตอนขึ้นเนินที่มอเตอร์ตามเป้าหมายไม่ทัน อาจกระตุ้น watchdog ของบอร์ดความเร็ว)
        # โปรไฟล์ใน CSV เร่งสูงสุด +0.867 m/s² (ช่วง launch) ถ้าคง 0.6 ไว้ rate limiter
        # จะรั้งไม่ให้ตามโปรไฟล์ทัน -> ตั้ง 0.9 ให้มีที่เหลือพอดี ส่วนเบรกโปรไฟล์ต้องการ
        # แค่ -0.767 m/s² ของเดิม 1.2 จึงพอแล้ว ไม่ต้องแตะ
        self.declare_parameter('max_speed_accel_mps2', 0.9)
        self.declare_parameter('max_speed_decel_mps2', 1.2)

        # ขอบเขตการเลี้ยวและ lookahead แบบปรับตามสถานการณ์
        self.declare_parameter('max_steer_rad', 0.8)  # จำกัดคำสั่งเลี้ยวสูงสุด (rad)
        self.declare_parameter('red_zone_max_steer_deg', 10.0)  # legacy: เก็บไว้ไม่ให้ config เดิมพัง แต่ไม่ได้ใช้จำกัดมุมเลี้ยวแล้ว
        self.declare_parameter('yellow_zone_max_steer_deg', 25.0)  # legacy: เก็บไว้เพื่อ compatibility
        self.declare_parameter('green_zone_max_steer_deg', 44.0)  # legacy: เก็บไว้เพื่อ compatibility
        self.declare_parameter('green_zone_speed_max', 0.25)  # legacy: เก็บไว้เพื่อ compatibility
        self.declare_parameter('yellow_zone_speed_max', 0.35)  # legacy: เก็บไว้เพื่อ compatibility
        self.declare_parameter('min_lookahead_distance', 3.0)  # lookahead ต่ำสุด (m)
        self.declare_parameter('max_lookahead_distance', 8.0)  # lookahead สูงสุด (m)
        self.declare_parameter('lookahead_turn_gain', 2.4)  # heading error มากขึ้น จะลด lookahead ตาม gain นี้
        self.declare_parameter('lookahead_heading_blend', 0.35)  # น้ำหนักเริ่มต้นในการผสม heading ของ path
        self.declare_parameter('lookahead_heading_blend_gain', 0.9)  # heading error มากขึ้น จะเพิ่มการ blend
        self.declare_parameter('lookahead_heading_blend_max', 0.75)  # จำกัดค่า heading blend สูงสุด
        self.declare_parameter('curve_lookahead_gain', 3.2)  # path โค้งมากขึ้น จะลด lookahead ตาม gain นี้
        self.declare_parameter('curve_speed_gain', 4.2)  # path โค้งมากขึ้น จะลดความเร็วตาม gain นี้
        self.declare_parameter('curve_heading_blend_gain', 1.2)  # path โค้งมากขึ้น จะเพิ่มการใช้ heading ของ path
        self.declare_parameter('curve_preview_points', 12)  # มองความโค้งล่วงหน้ากี่จุด
        self.declare_parameter('curve_preview_gain', 1.8)  # น้ำหนักความโค้งล่วงหน้าเพื่อให้เริ่มเลี้ยวไวขึ้น
        self.declare_parameter('turn_anticipation_gain', 0.85)  # เห็นโค้งล่วงหน้าแล้วให้ตัด lookahead ลงไวขึ้นแม้ยังอยู่บนทางตรง
        self.declare_parameter('sweep_preview_points', 28)  # มองมุมสะสมของโค้งยาวเพิ่ม เพื่อจับโค้งอ่อน ๆ ระยะยาว
        self.declare_parameter('sweep_turn_angle_gain', 0.75)  # น้ำหนักมุมสะสมของโค้งยาวต่อการตัดสินใจเลี้ยว

        # โหมดทางตรง ใช้ลดอาการส่ายซ้ายขวาเมื่อ path ค่อนข้างตรง
        self.declare_parameter('straight_curvature_threshold', 1.5)  # curvature ต่ำกว่านี้ถือว่าเป็นทางตรง
        self.declare_parameter('straight_mode_hysteresis', 0.02)  # กันการสลับโหมดทางตรง/โค้งถี่เกินเมื่อ curvature แกว่งใกล้ threshold
        self.declare_parameter('straight_preview_turn_deg', 5.0)  # ถ้าโค้งสะสมเกินมุมนี้ แม้ curvature ต่ำก็ไม่ควรถูกมองเป็นทางตรง
        self.declare_parameter('straight_preview_turn_hysteresis_deg', 1.5)  # กันการสลับโหมดเมื่อโค้งสะสมใกล้ threshold
        self.declare_parameter('straight_lookahead_bonus', 5.)  # เพิ่ม lookahead ตอนทางตรงเพื่อให้รถนิ่งขึ้น (m)
        self.declare_parameter('straight_heading_blend_scale', 0.35)  # ลด heading blend ตอนทางตรง แต่ยังให้พอช่วยดึงกลับ
        self.declare_parameter('straight_cte_gain_scale', 0.75)  # ลดแรงแก้ CTE ตอนทางตรงให้น้อยลงจากโค้ง แต่ไม่อ่อนเกินไป
        self.declare_parameter('straight_path_lock_cte_m', 0.08)  # ถ้า CTE ยังเกินนี้ ห้ามนิ่งขนาน path ต้องดึงกลับเข้าเส้นก่อน
        self.declare_parameter('straight_bonus_fade_cte_m', 0.30)  # CTE มากขึ้นจะค่อย ๆ ตัด straight lookahead bonus
        self.declare_parameter('sweep_heading_blend_gain', 0.9)  # โค้งยาวอ่อน ๆ ให้เชื่อ heading ของ path มากขึ้น
        self.declare_parameter('curve_lookahead_bonus', 1.6)  # เพิ่ม lookahead ในโค้งเพื่อมองไลน์ไกลขึ้น (m)

        # =========================
        # CBF-QP interface (สถาปัตยกรรมใหม่)
        # =========================
        # adaptive_ld_node แก้ CBF-QP ทุกเฟรมแล้วส่ง u* = [v*, L_d*] กลับมา
        # node นี้มีหน้าที่ (1) ส่งสถานะ x = [e_lat, θ_e, v] + κ, κ_preview ให้ QP
        # (2) เอา u* ไปใช้กับ pure pursuit จริง (3) ตรวจคำสั่งกับค่าจริง (Command Check)
        # ถ้า QP เงียบเกิน timeout จะถอยกลับไปใช้ heuristic lookahead/speed ชุดเดิมอัตโนมัติ
        self.declare_parameter('use_cbf_qp', True)
        self.declare_parameter('cbf_qp_ld_topic', '/adaptive_ld')  # L_d* จาก QP
        self.declare_parameter('cbf_qp_speed_topic', '/adaptive_speed')  # v* จาก QP
        self.declare_parameter('cbf_qp_timeout_sec', 0.5)  # ไม่ได้ u* ใหม่เกินนี้ = ถอยไปใช้ heuristic
        # เพดานความเร็วจาก h₂ ของ solver ตอนความเร็วมาจากไฟล์ (∞ = ไม่จำกัด, h₂ ปิด)
        self.declare_parameter('cbf_qp_speed_cap_topic', '/cbf_qp/speed_cap')
        # solver อยู่โหมด pure_pursuit_only (ติ๊กใน rqt_reconfigure ของ /cbf_qp_solver) ->
        # เลี้ยวด้วย δ = atan(2L·sinα / L_d) ล้วน ไม่ผ่าน heading blend / CTE correction / filter
        self.declare_parameter('cbf_qp_pure_pursuit_topic', '/cbf_qp/pure_pursuit_only')
        # 'cap' = ใช้ v* เป็นเพดานทับ heuristic (safety filter, ค่าเริ่มต้น)
        # 'replace' = เชื่อ v* ตรง ๆ ตามสมการข้อ 7 (ยังผ่าน goal slowdown/localization guard อยู่)
        self.declare_parameter('cbf_speed_mode', 'cap')
        self.declare_parameter('curvature_preview_topic', '/curvature_preview')  # κ_preview ที่ส่งให้ QP
        # κ_preview = max κ ในระยะ v/α₂ ข้างหน้า (ต้องตั้งให้ตรงกับ alpha2 ฝั่ง adaptive_ld_node)
        self.declare_parameter('cbf_preview_alpha2', 0.4)
        self.declare_parameter('cbf_preview_min_distance', 2.0)  # กันระยะพรีวิวสั้นเกินตอนรถช้า (m)
        self.declare_parameter('cbf_preview_max_distance', 15.0)  # กันระยะพรีวิวยาวเกินตอนรถเร็ว (m)

        # Command Check (สมการข้อ 9): Δv = v_cmd − /speed_status, Δδ = δ_cmd − /steering_angle
        # เป็นชั้นตรวจสอบที่สองต่อจาก CBF — จับกรณีสั่งไปแล้วรถไม่ทำตาม (มอเตอร์/CAN เสีย)
        # เกณฑ์ตั้งไว้กว้างและต้องผิดต่อเนื่องนาน เพราะช่วงเร่ง/พวงมาลัยไล่ตามคำสั่งมี error ปกติอยู่แล้ว
        self.declare_parameter('command_check_enable', True)
        self.declare_parameter('steering_angle_topic', '/steering_angle')  # มุมเลี้ยวจริง (rad)
        self.declare_parameter('command_check_max_speed_error', 1.2)  # |Δv| เกินนี้ถือว่าผิดปกติ (m/s)
        self.declare_parameter('command_check_max_steer_error_deg', 20.0)  # |Δδ| เกินนี้ถือว่าผิดปกติ
        self.declare_parameter('command_check_confirm_cycles', 40)  # ต้องผิดต่อเนื่องกี่รอบค่อยหยุด (40 @20Hz = 2 s)
        self.declare_parameter('command_check_feedback_timeout_sec', 0.5)  # feedback ไม่สด = ข้ามการตรวจ
        self.declare_parameter('command_check_min_speed_cmd', 0.3)  # สั่งช้ากว่านี้ไม่ต้องตรวจ Δv
        self.declare_parameter('command_check_steer_feedback_sign', 1.0)  # -1.0 ถ้าบอร์ดตีความซ้าย/ขวากลับด้าน

        # จำนวน waypoint ที่ใช้คำนวณทิศทางของ path
        self.declare_parameter('path_heading_back_points', 1)  # ดู waypoint ย้อนหลังเท่านี้จุด
        self.declare_parameter('path_heading_forward_points', 3)  # ดู waypoint ข้างหน้าเท่านี้จุด
        self.declare_parameter('use_path_orientation', True)  # ใช้ orientation ที่มากับ nav_path ช่วยกำหนด heading
        # เปิดไว้เป็น default เพราะ nav_path/pagard ถูกรันตรง ๆ ด้วย python3 ไม่มี --ros-args
        # ส่งเข้ามา ถ้าปิดไว้ pose_stamp ใน CSV จะถูกอ่านทิ้งเงียบ ๆ
        # หมายเหตุ: ค่านี้เป็นเพดานความเร็วเท่านั้น (speed_cmd = min(speed_cmd, ...)) ไม่เคยเร่ง
        # เกิน forward_speed และมีพื้นที่ path_speed_min_limit รองอยู่
        self.declare_parameter('use_path_speed_profile', True)  # ใช้ timestamp ใน nav_path สร้าง speed profile
        # v_des ของ CBF-QP: อ่านคอลัมน์ v_target ที่ nav_path ส่งมาเป็นอาร์เรย์คู่กับ path
        # (คนละทางกับ path_segment_speed_mps ด้านบน ซึ่งถอดจาก Δstamp และใช้เป็น "เพดาน")
        self.declare_parameter('use_path_target_speed', True)
        self.declare_parameter('path_speed_profile_topic', '/nav_path_speed')
        self.declare_parameter('desired_speed_topic', '/desired_speed')
        # 0 = ใช้ v_target ที่ waypoint นั้นตรง ๆ ไม่มีการมองล่วงหน้าแล้วกดค่าลง
        # ตั้ง > 0 ถ้าอยากได้ preview-min กลับมา (0.3 m/จุด -> 10 จุด ≈ 3 m)
        self.declare_parameter('path_target_speed_preview_points', 0)
        # แถวแรกของโปรไฟล์คือ launch ที่ v_target=0.000 ถ้าปล่อยตามตรง รถจะได้ v_des=0
        # ตอนจอดอยู่ที่ waypoint แรก แล้วไม่มีวันออกตัว (ไม่ขยับ = closest_index ไม่เลื่อน)
        # floor นี้จะยกขึ้นเฉพาะตอนที่ "ข้างหน้าเร็วขึ้น" เท่านั้น path ที่ปิดเป็นวงจะมองข้ามรอยต่อ
        # ไปต้นวงด้วย จอดตรงท้ายไฟล์ (v_target=0) จึงออกตัวได้ ตั้งเป็น 0.0 เพื่อปิดกลไกนี้
        # 0.5 ไม่พอ: PID ใน STM32 (Kp 1, Ki 0.1) ต้องรอ integral ~13 s กว่าล้อจะเริ่มหมุน (log 11-02)
        self.declare_parameter('path_target_speed_launch_floor', 1.0)
        # ออกตัวจากกลาง path: v_target ตรงนั้นอาจสูงกว่าความเร็วรถมาก จึงให้คำสั่งนำความเร็ววัดจริง
        # ได้ไม่เกินค่านี้ตอนเร่ง (ขาลดความเร็วยังตามไฟล์ทันที) ต้องน้อยกว่า
        # command_check_max_speed_error ไม่งั้น safety stop ทริปซ้ำทุก ~4 s แล้ววนอยู่ที่ ~1 m/s
        # ตั้งเป็น 0.0 เพื่อปิดกลไกนี้
        self.declare_parameter('path_speed_max_lead_mps', 1.0)
        # รถไม่ขยับ (ต่ำกว่า stall_speed): เพิ่ม lead ทีละนิดจนเกิน tolerance ของ command check
        # ถ้าฮาร์ดแวร์เสียจริง check ยังทริปได้ ที่ค่าปัจจุบันใช้เวลาราว 6 s
        self.declare_parameter('path_speed_stall_lead_gain', 0.05)  # m/s ต่อวินาทีที่รถไม่ขยับ
        self.declare_parameter('path_speed_stall_speed_mps', 0.15)
        # True = ความเร็วมาจากโปรไฟล์ CSV อย่างเดียว ข้าม compute_speed ทั้งก้อน
        # (turn_speed / slow_speed / curve_speed_gain / straight_mode / goal slowdown)
        # และไม่เอา v* จาก QP มา cap ทับ
        self.declare_parameter('speed_from_path_profile_only', True)
        # True = ค่าจาก CSV ถึง cmd_vel โดยไม่มีตัวคุมความเร็วตัวไหนแปลงระหว่างทาง
        # ปิด: preview-min, feedback correction, rate limit, speed_command_max
        # ยังเหลือแต่ตัวที่ "สั่งหยุด" ล้วน ๆ (safety stop / localization / command check)
        self.declare_parameter('speed_profile_passthrough', True)
        self.declare_parameter('path_speed_scale', 1.0)  # scale speed profile จาก nav_path ก่อนเอามาเป็น speed limit
        self.declare_parameter('path_speed_min_limit', 0.12)  # speed limit ต่ำสุดจาก nav_path
        self.declare_parameter('path_speed_preview_points', 12)  # มอง speed profile ข้างหน้าเพิ่มอีกกี่ช่วง
        self.declare_parameter('path_speed_limit_quantile', 0.35)  # ใช้ quantile แทน min เพื่อลดผลจากจุดช้าหรือ noise เพียงไม่กี่จุด
        self.declare_parameter('turn_angle_preview_points', 12)  # มอง heading ข้างหน้าเพิ่มอีกกี่จุดเพื่อประเมินโค้งที่จะมา
        self.declare_parameter('turn_angle_lookahead_gain', 2.4)  # โค้งล่วงหน้าแรงขึ้น จะลด lookahead เพิ่ม
        self.declare_parameter('turn_angle_speed_gain', 3.2)  # โค้งล่วงหน้าแรงขึ้น จะลด speed เพิ่ม
        self.declare_parameter('turn_heading_blend_gain', 1.2)  # โค้งล่วงหน้าแรงขึ้น จะเชื่อ heading ของ path มากขึ้น
        self.declare_parameter('turn_heading_blend_max', 0.98)  # จำกัด heading blend สูงสุดตอนเจอโค้งล่วงหน้า

        # CTE = cross-track error คือระยะที่รถเบี่ยงซ้าย/ขวาจาก path
        self.declare_parameter('cte_heading_gain', 0.20)  # gain ดึงรถกลับเข้า path
        self.declare_parameter('cte_derivative_gain', 0.14)  # gain ลดการส่ายจากอัตราเปลี่ยน CTE
        self.declare_parameter('cte_max_correction_rad', 0.28)  # จำกัดมุมแก้ CTE สูงสุด (rad)
        self.declare_parameter('cte_lookahead_gain', 1.2)  # ถ้าหลุด path มากขึ้น จะลด lookahead เพิ่ม
        self.declare_parameter('cte_heading_blend_gain', 0.9)  # ถ้าหลุด path มากขึ้น จะเชื่อ heading ไปยัง target มากขึ้น
        self.declare_parameter('cte_speed_ref_mps', 1.2)  # ใช้ normalize ว่าเร็ว/ช้าสำหรับปรับแรงดึง CTE
        self.declare_parameter('straight_cte_low_speed_scale', 1.10)  # ทางตรงวิ่งช้าให้ดึงกลับได้มากขึ้น
        self.declare_parameter('straight_cte_high_speed_scale', 0.58)  # ทางตรงวิ่งเร็วให้ลดแรงดึงลงเพื่อลดอาการส่าย
        self.declare_parameter('curve_cte_low_speed_scale', 1.00)  # ในโค้งวิ่งช้าให้คงแรงดึง CTE ได้เต็ม
        self.declare_parameter('curve_cte_high_speed_scale', 0.82)  # ในโค้งวิ่งเร็วลดแรงดึงลง แต่ไม่ลดมากเท่าทางตรง
        self.declare_parameter('curve_cte_curvature_ref', 0.08)  # curvature ระดับนี้ขึ้นไปจะเพิ่มแรงกู้ CTE ชัดเจน
        self.declare_parameter('curve_cte_curvature_boost', 0.35)  # โค้งมากขึ้นจะคืนแรงดึง CTE เพิ่ม
        self.declare_parameter('curve_cte_gain_scale', 0.80)  # ตอนเข้าโค้งลดแรงดึง CTE ลงเพื่อลดอาการสะบัด แต่ยังต้องพอดึงกลับทัน
        self.declare_parameter('curve_cte_derivative_scale', 0.18)  # ตอนเข้าโค้งลดผล derivative ของ CTE ลงเพื่อลดการแกว่ง
        self.declare_parameter('cte_rate_filter_alpha', 0.18)  # filter derivative ของ CTE ก่อนใช้จริง
        self.declare_parameter('curve_sign_flip_cte_m', 0.18)  # ถ้า CTE ยังเล็กอยู่ ไม่ควรสลับทิศพวงมาลัยแรง ๆ ในโค้ง
        self.declare_parameter('curve_sign_flip_heading_deg', 7.0)  # heading error ยังเล็กอยู่ให้หน่วงการกลับขั้วพวงมาลัย
        self.declare_parameter('curve_sign_flip_damping', 0.12)  # หน่วงการกลับขั้วพวงมาลัยในโค้งเพื่อลดอาการส่าย

        # ทำให้คำสั่งเลี้ยวนุ่มขึ้นและไม่เปลี่ยนเร็วเกินไป
        self.declare_parameter('steering_filter_alpha', 0.25)  # filter steering: ต่ำ = นุ่มขึ้น แต่ตอบสนองช้าลง
        self.declare_parameter('max_steer_rate_rad', 0.9)  # จำกัดอัตราเปลี่ยน steering (rad/s)
        self.declare_parameter('straight_high_speed_steer_limit_scale', 0.72)  # ตอนวิ่งเร็วบนทางตรง บีบเพดานเลี้ยวลง
        self.declare_parameter('straight_high_speed_filter_scale', 0.82)  # ตอนวิ่งเร็วบนทางตรง ลดความไวของ steering filter
        self.declare_parameter('straight_high_speed_rate_scale', 0.70)  # ตอนวิ่งเร็วบนทางตรง ลดอัตราเปลี่ยน steering
        self.declare_parameter('curve_high_speed_steer_limit_scale', 0.88)  # ตอนวิ่งเร็วในโค้ง ยังลดเพดานเลี้ยวลงเล็กน้อย
        self.declare_parameter('curve_high_speed_filter_scale', 0.92)  # ตอนวิ่งเร็วในโค้ง ลดความไวลงเล็กน้อย
        self.declare_parameter('curve_high_speed_rate_scale', 0.82)  # ตอนวิ่งเร็วในโค้ง จำกัดอัตราเปลี่ยน steering ลงเล็กน้อย
        # จุดอ้างอิงความเร็วสำหรับคำนวณ steering damping โดยเฉพาะ แยกจาก turn_speed/forward_speed
        # ที่ใช้วางแผนความเร็ว path เพื่อไม่ให้การปรับความเร็ว climb เนิน/โค้งไปกระทบการหน่วงพวงมาลัย
        self.declare_parameter('steer_damping_speed_low_ref', 0.3)
        self.declare_parameter('steer_damping_speed_high_ref', 1.0)
        self.declare_parameter('steering_deadband_deg', 2.0)  # ลด deadband compensation ลง เพื่อไม่ให้ใกล้ path แล้วยังโดนดันมุมเลี้ยวสูง
        self.declare_parameter('steering_activation_deg', 2.4)  # ต้องมีคำสั่งเลี้ยวมากกว่านี้ค่อยเริ่มชดเชย deadband
        self.declare_parameter('invert_steer_sign', False)  # กลับเครื่องหมายพวงมาลัยถ้าชุดขับตีความซ้าย/ขวาตรงข้าม
        self.declare_parameter('turn_slowdown_heading_deg', 18.0)  # heading error เกินค่านี้จะลดความเร็ว (deg)
        self.declare_parameter('steering_hold_cte_m', 0.22)  # อยู่ใกล้ path มากขึ้นให้ถือว่าไม่ต้องเลี้ยว
        self.declare_parameter('steering_hold_heading_deg', 6.0)  # heading ใกล้ตรง path มากขึ้นให้ปล่อยพวงมาลัยเป็นศูนย์
        self.declare_parameter('steering_zero_band_deg', 2.3)  # steering เล็ก ๆ แถวกลางทางให้ตัดเป็นศูนย์ได้มากขึ้น
        self.declare_parameter('straight_hold_cte_m', 0.12)  # ทางตรงถ้าเกือบเกาะ path แล้วให้หยุดสั่งเลี้ยวไวขึ้น
        self.declare_parameter('straight_hold_heading_deg', 4.0)  # ทางตรง heading error เล็กน้อยยังถือว่าไม่ต้องเลี้ยว
        self.declare_parameter('straight_zero_band_deg', 2.2)  # บีบคำสั่งเล็ก ๆ บนทางตรงให้เป็นศูนย์มากขึ้น
        self.declare_parameter('straight_steering_activation_deg', 2.2)  # ทางตรงต้องมีคำสั่งมากพอค่อยเริ่มชดเชย deadband
        self.declare_parameter('straight_cte_slowdown_m', 0.22)  # ถ้า CTE ทางตรงเกินนี้ให้เริ่มชะลอ
        self.declare_parameter('straight_cte_turn_speed_m', 0.38)  # ถ้า CTE ทางตรงเกินนี้ให้ใช้ turn_speed
        self.declare_parameter('straight_mode_max_steer_deg', 12.0)  # ถ้า path ยังตรงอยู่ ให้บีบเพดานเลี้ยวให้น้อยลงเพื่อลดอาการส่าย
        self.declare_parameter('curve_mode_max_steer_deg', 50.0)  # ถ้าเข้าโค้ง อนุญาตให้เลี้ยวได้มากขึ้นตามเพดานนี้
        self.declare_parameter('curve_mode_full_curvature', 0.08)  # curvature ระดับนี้ขึ้นไป จะเปิดเพดานเลี้ยวโหมดโค้งได้เต็ม
        self.declare_parameter('curve_cte_slowdown_m', 0.08)  # ถ้า CTE ตอนอยู่ในโค้งเริ่มสูง ให้รีบชะลอก่อนจะหลุดโค้ง
        self.declare_parameter('curve_cte_turn_speed_m', 0.14)  # ถ้า CTE ตอนอยู่ในโค้งสูงกว่านี้ ให้บังคับใช้ turn_speed
        self.declare_parameter('turn_preview_full_steer_deg', 18.0)  # ถ้าโค้งล่วงหน้าสะสมถึงมุมนี้ ให้ปลดเพดานเลี้ยวขึ้นไปถึงโซนเขียว
        self.declare_parameter('cte_recovery_full_steer_m', 0.30)  # ถ้าหลุด path ระดับนี้ ให้ปลดเพดานเลี้ยวขึ้นไปถึงโซนเขียวเพื่อดึงกลับเร็วขึ้น
        self.declare_parameter('hard_turn_preview_deg', 12.0)  # ถ้าเห็นมุมโค้งล่วงหน้าแรงกว่านี้ ให้เข้าโหมดหักล่วงหน้า
        self.declare_parameter('hard_turn_curvature', 0.07)  # ถ้า curvature เกินนี้ ให้ถือว่าเป็นโค้งหนัก
        self.declare_parameter('hard_turn_speed', 1.6)  # ความเร็วตอนเข้าโค้งหนัก ลดลงอีกเพื่อกันอาการพุ่งออกนอกเส้น
        self.declare_parameter('curve_motion_min_speed', 0.22)  # floor ของความเร็วในโค้ง เพื่อคงแรงฉุดตอนพวงมาลัยกำลังหัก
        self.declare_parameter('hard_turn_lookahead_distance', 1.35)  # floor ของ lookahead ตอนเข้าโค้งหนัก ไม่ให้สั้นเกินไป
        self.declare_parameter('hard_turn_lookahead_bonus_scale', 0.6)  # แม้โค้งหนักก็คืน lookahead bonus บางส่วน
        self.declare_parameter('curve_steering_filter_alpha', 0.55)  # ตอนเข้าโค้งให้ตอบสนองไวขึ้น แต่ไม่ไวจนส่าย
        self.declare_parameter('curve_max_steer_rate_rad', 2.1)  # ตอนเข้าโค้งให้เปลี่ยนมุมเลี้ยวได้ไวขึ้นแบบคุมอาการสะบัด

        # Pose yaw และ speed reference
        self.declare_parameter('yaw_offset', -0.045)  # ชดเชยมุม yaw จาก pose quaternion ให้ตรงกับตัวรถ (rad)
        self.declare_parameter('yaw_filter_alpha', 0.2)  # filter yaw: ต่ำ = นุ่มขึ้น แต่หน่วงมากขึ้น
        self.declare_parameter('motion_heading_min_dist', 0.08)  # ขยับเกินระยะนี้ค่อยใช้ทิศจากการเคลื่อนที่
        self.declare_parameter('motion_heading_weight', 0.85)  # น้ำหนัก heading จากการเคลื่อนที่ เทียบกับ quaternion
        self.declare_parameter('pose_speed_filter_alpha', 0.15)  # filter ความเร็วที่คำนวณจาก pose
        self.declare_parameter('pose_speed_timeout_sec', 0.5)  # ไม่ได้ pose ใหม่เกินเวลานี้ จะ fallback ไปใช้ commanded speed (s)
        self.declare_parameter('speed_status_topic', '/speed_status')  # ความเร็วจริงจาก CAN (golfcart_low_level)
        self.declare_parameter('speed_status_timeout_sec', 0.5)  # ไม่ได้ /speed_status ใหม่เกินเวลานี้ จะ fallback ไปใช้ pose speed
        # แหล่งความเร็วรถจริง เรียงลำดับความน่าเชื่อถือ: speed_status (CAN) -> pose (/pcl_pose)
        # -> odometry -> คำสั่งล่าสุด. 'auto' = ไล่ตามลำดับนี้, หรือบังคับเลือกแหล่งเดียวได้
        self.declare_parameter('velocity_source', 'auto')  # auto | speed_status | pose | odom | gps
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('odom_timeout_sec', 0.5)
        # ป้องกันกรณี speed sensor เสีย: /speed_status ยัง publish อยู่แต่ค้างที่ 0.00 ตลอด
        # ถ้าค้าง 0 นานเกินนี้ทั้งที่ pose บอกว่ารถกำลังเคลื่อนที่ ให้ถือว่าแหล่งนี้ตาย
        # แล้วข้ามไปใช้ pose speed แทน (0 = ปิดการตรวจ เชื่อ /speed_status เสมอ)
        self.declare_parameter('speed_status_stuck_zero_sec', 2.0)
        self.declare_parameter('moving_speed_epsilon', 0.15)  # เร็วกว่านี้ถือว่ารถเคลื่อนที่จริง (m/s)
        # GPS heading fallback: ตอน localization เข้าโหมด degraded (pose เริ่มขาด/ห่าง)
        # ใช้ course-over-ground จาก GPS Doppler (/ublox_gps_node/fix_velocity) แทน current_yaw
        # ที่ค้างอยู่ที่ค่า pose ล่าสุด. gps_heading_yaw_offset_deg ต้อง calibrate ให้ตรงกับ
        # gnss_to_map_yaw_offset_deg ฝั่ง lidar_localization_ros2 (การหมุนจาก ENU ไปยัง map frame
        # เดียวกัน) ไม่งั้น heading ที่ได้จะหมุนผิดทิศ; ปิดไว้เป็นค่าเริ่มต้นจนกว่าจะ calibrate
        self.declare_parameter('use_gps_heading_fallback', False)
        self.declare_parameter('gps_velocity_topic', '/ublox_gps_node/fix_velocity')
        self.declare_parameter('gps_heading_yaw_offset_deg', 0.0)
        self.declare_parameter('gps_heading_min_speed_mps', 0.5)  # ต่ำกว่านี้ course จาก Doppler ไม่นิ่งพอจะเชื่อ
        self.declare_parameter('gps_heading_max_age_sec', 1.0)
        self.declare_parameter('enable_pose_prediction', False)  # ปิดค่าเริ่มต้นเพื่อให้ CTE/pose ใช้ตำแหน่งจริง ไม่คาดล่วงหน้า
        self.declare_parameter('localization_latency_sec', 0.15)  # ชดเชยความหน่วงของ localization แบบอนุรักษ์นิยม
        self.declare_parameter('max_localization_latency_sec', 0.35)  # จำกัดเวลาชดเชย latency สูงสุดเพื่อไม่ให้พยากรณ์ตำแหน่งไกลเกินจริง
        self.declare_parameter('turn_latency_scale', 0.35)  # ตอนกำลังเลี้ยวให้ลดแรงชดเชย latency ลงอีก
        self.declare_parameter('pose_stale_timeout_sec', 0.80)  # pose ค้างเกินนี้ให้หยุดก่อนใช้ตำแหน่งเก่าต่อ (ผ่อนจาก 0.35: NDT จริงมี jitter ถึง ~0.7s เป็นปกติ)
        self.declare_parameter('localization_latency_warn_sec', 0.18)  # pose เริ่มเก่าจนควรชะลอ
        self.declare_parameter('localization_latency_stop_sec', 0.35)  # pose เก่ามากจนควรหยุด
        self.declare_parameter('pose_interval_warn_sec', 0.30)  # pose มาเริ่มห่างเกินนี้ให้ชะลอ (ผ่อนจาก 0.14: interval จริงของ NDT อยู่ที่ ~0.16-0.25s ปกติ)
        self.declare_parameter('pose_interval_stop_sec', 0.50)  # pose มาห่างมากเกินนี้ให้หยุด (ผ่อนจาก 0.30 ตามเหตุผลเดียวกัน)
        self.declare_parameter('use_pose_header_latency_guard', False)  # บางระบบ pose stamp คนละ clock กัน จึงปิดการใช้ latency จาก header เป็นค่าเริ่มต้น
        self.declare_parameter('localization_health_filter_alpha', 0.20)  # filter latency/interval กัน false alarm
        self.declare_parameter('localization_degraded_confirm_cycles', 3)  # ต้องหน่วงต่อเนื่องกี่รอบค่อยลดความเร็ว
        self.declare_parameter('localization_stop_confirm_cycles', 6)  # ต้องหน่วงต่อเนื่องกี่รอบค่อยหยุด
        self.declare_parameter('localization_recover_confirm_cycles', 4)  # ต้องกลับมาปกติกี่รอบค่อยยกเลิก degraded/stop
        self.declare_parameter('localization_degraded_speed_scale', 0.60)  # ตอน localization หน่วงให้บีบ speed target ลง
        self.declare_parameter('localization_degraded_max_speed', 0.25)  # เพดาน speed target ตอน localization หน่วง
        self.declare_parameter('speed_lookahead_gain', 0.8)  # ความเร็วมากขึ้น จะเพิ่ม lookahead ตาม gain นี้
        self.declare_parameter('min_control_speed_ref', 0.30)  # speed reference ต่ำสุดที่ใช้คำนวณ steering correction
        self.declare_parameter('steering_release_deg', 1.4)  # ปล่อย deadband compensation เร็วขึ้นเมื่อรถกลับมาเกือบตรง

        # RViz visualization
        self.declare_parameter('broadcast_vehicle_tf', False)  # False = ปล่อยให้ lidar_localization_ros2 เป็นเจ้าของ TF map->base_link แต่ผู้เดียว กันแย่งกัน publish จนสั่น
        self.declare_parameter('trace_min_dist', 0.1)  # รถต้องขยับเกินระยะนี้ถึงเพิ่มจุดใน vehicle_trace (m)
        self.declare_parameter('enable_controller_csv_log', True)
        # แยกโฟลเดอร์ตามวันที่เริ่มรัน: ~/controller_logs/2026-10-07/controller_log_<วันเวลา>.csv
        self.declare_parameter(
            'controller_csv_log_path',
            os.path.expanduser(
                datetime.now().strftime(
                    '~/controller_logs/%Y-%m-%d/controller_log_%Y%m%d_%H%M%S.csv'
                )
            )
        )
        self.declare_parameter('controller_csv_log_period', 0.1)
        self.declare_parameter('controller_csv_log_flush_every_row', True)

        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.dry_run_cmd_vel_topic = str(
            self.get_parameter('dry_run_cmd_vel_topic').value
        )
        # ปลายทางจริงของ Twist ทุกเส้นทาง (รวม publish_stop) ตัดสินที่นี่ที่เดียว
        self.effective_cmd_vel_topic = (
            self.dry_run_cmd_vel_topic if self.dry_run else self.cmd_vel_topic
        )
        self.curvature_topic = self.get_parameter('curvature_topic').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.lookahead_topic = self.get_parameter('lookahead_topic').value
        self.target_goal_topic = self.get_parameter('target_goal_topic').value
        self.target_goal_timeout_sec = max(
            float(self.get_parameter('target_goal_timeout_sec').value),
            0.1
        )
        self.path_coordinate_limit = max(
            float(self.get_parameter('path_coordinate_limit').value),
            0.0
        )

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.world_frame = self.get_parameter('world_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.binary_speed_mode = bool(
            self.get_parameter('binary_speed_mode').value
        )
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.stop_speed = float(self.get_parameter('stop_speed').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.speed_command_scale = max(
            float(self.get_parameter('speed_command_scale').value),
            0.0
        )
        self.speed_command_offset = max(
            float(self.get_parameter('speed_command_offset').value),
            0.0
        )
        self.speed_command_max = max(
            float(self.get_parameter('speed_command_max').value),
            0.0
        )
        self.speed_feedback_up_kp = max(
            float(self.get_parameter('speed_feedback_up_kp').value),
            0.0
        )
        self.speed_feedback_down_kp = max(
            float(self.get_parameter('speed_feedback_down_kp').value),
            0.0
        )
        self.speed_feedback_max_delta = max(
            float(self.get_parameter('speed_feedback_max_delta').value),
            0.0
        )
        self.max_speed_accel_mps2 = max(
            float(self.get_parameter('max_speed_accel_mps2').value),
            0.01
        )
        self.max_speed_decel_mps2 = max(
            float(self.get_parameter('max_speed_decel_mps2').value),
            0.01
        )
        self.prev_speed_cmd_out = 0.0
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.red_zone_max_steer_rad = math.radians(
            float(self.get_parameter('red_zone_max_steer_deg').value)
        )
        self.yellow_zone_max_steer_rad = math.radians(
            float(self.get_parameter('yellow_zone_max_steer_deg').value)
        )
        self.green_zone_max_steer_rad = math.radians(
            float(self.get_parameter('green_zone_max_steer_deg').value)
        )
        self.green_zone_speed_max = float(
            self.get_parameter('green_zone_speed_max').value
        )
        self.yellow_zone_speed_max = float(
            self.get_parameter('yellow_zone_speed_max').value
        )
        self.min_lookahead_distance = float(self.get_parameter('min_lookahead_distance').value)
        self.max_lookahead_distance = float(self.get_parameter('max_lookahead_distance').value)
        self.lookahead_turn_gain = float(self.get_parameter('lookahead_turn_gain').value)
        self.lookahead_heading_blend = float(self.get_parameter('lookahead_heading_blend').value)
        self.lookahead_heading_blend_gain = float(
            self.get_parameter('lookahead_heading_blend_gain').value
        )
        self.lookahead_heading_blend_max = float(
            self.get_parameter('lookahead_heading_blend_max').value
        )
        self.curve_lookahead_gain = float(self.get_parameter('curve_lookahead_gain').value)
        self.curve_speed_gain = float(self.get_parameter('curve_speed_gain').value)
        self.curve_heading_blend_gain = float(
            self.get_parameter('curve_heading_blend_gain').value
        )
        self.curve_preview_points = int(
            self.get_parameter('curve_preview_points').value
        )
        self.curve_preview_gain = float(self.get_parameter('curve_preview_gain').value)
        self.turn_anticipation_gain = self.clamp(
            float(self.get_parameter('turn_anticipation_gain').value),
            0.0,
            1.5
        )
        self.sweep_preview_points = max(
            self.curve_preview_points,
            int(self.get_parameter('sweep_preview_points').value)
        )
        self.sweep_turn_angle_gain = self.clamp(
            float(self.get_parameter('sweep_turn_angle_gain').value),
            0.0,
            2.0
        )
        self.straight_curvature_threshold = float(
            self.get_parameter('straight_curvature_threshold').value
        )
        self.straight_mode_hysteresis = max(
            float(self.get_parameter('straight_mode_hysteresis').value),
            0.0
        )
        self.straight_preview_turn_rad = math.radians(
            float(self.get_parameter('straight_preview_turn_deg').value)
        )
        self.straight_preview_turn_hysteresis_rad = math.radians(
            float(self.get_parameter('straight_preview_turn_hysteresis_deg').value)
        )
        self.straight_lookahead_bonus = float(
            self.get_parameter('straight_lookahead_bonus').value
        )
        self.straight_heading_blend_scale = float(
            self.get_parameter('straight_heading_blend_scale').value
        )
        self.straight_cte_gain_scale = float(
            self.get_parameter('straight_cte_gain_scale').value
        )
        self.straight_path_lock_cte_m = float(
            self.get_parameter('straight_path_lock_cte_m').value
        )
        self.straight_bonus_fade_cte_m = max(
            float(self.get_parameter('straight_bonus_fade_cte_m').value),
            1e-3
        )
        self.sweep_heading_blend_gain = self.clamp(
            float(self.get_parameter('sweep_heading_blend_gain').value),
            0.0,
            2.0
        )
        self.curve_lookahead_bonus = max(
            float(self.get_parameter('curve_lookahead_bonus').value),
            0.0
        )
        self.use_cbf_qp = bool(self.get_parameter('use_cbf_qp').value)
        self.cbf_qp_ld_topic = str(self.get_parameter('cbf_qp_ld_topic').value)
        self.cbf_qp_speed_topic = str(self.get_parameter('cbf_qp_speed_topic').value)
        self.cbf_qp_speed_cap_topic = str(self.get_parameter('cbf_qp_speed_cap_topic').value)
        self.cbf_qp_pure_pursuit_topic = str(
            self.get_parameter('cbf_qp_pure_pursuit_topic').value
        )
        self.cbf_qp_timeout_sec = max(
            float(self.get_parameter('cbf_qp_timeout_sec').value),
            0.05
        )
        self.cbf_speed_mode = str(self.get_parameter('cbf_speed_mode').value).lower()
        if self.cbf_speed_mode not in ('cap', 'replace'):
            self.get_logger().warn(
                f"cbf_speed_mode='{self.cbf_speed_mode}' ไม่รู้จัก -> ใช้ 'cap' แทน")
            self.cbf_speed_mode = 'cap'
        self.curvature_preview_topic = str(
            self.get_parameter('curvature_preview_topic').value
        )
        self.cbf_preview_alpha2 = max(
            float(self.get_parameter('cbf_preview_alpha2').value),
            1e-3
        )
        self.cbf_preview_min_distance = max(
            float(self.get_parameter('cbf_preview_min_distance').value),
            0.1
        )
        self.cbf_preview_max_distance = max(
            float(self.get_parameter('cbf_preview_max_distance').value),
            self.cbf_preview_min_distance + 1e-3
        )
        self.command_check_enable = bool(
            self.get_parameter('command_check_enable').value
        )
        if self.command_check_enable and self.dry_run:
            # dry run = คำสั่งไม่ได้ไปถึงรถ รถจึงไม่มีทางทำตามได้
            # ปล่อยให้ตรวจต่อ = Δv ค้างเกินเกณฑ์ตลอด แล้วสั่ง safety stop ผิด ๆ ภายใน 2 วิ
            self.command_check_enable = False
        self.steering_angle_topic = str(self.get_parameter('steering_angle_topic').value)
        self.command_check_max_speed_error = max(
            float(self.get_parameter('command_check_max_speed_error').value),
            0.05
        )
        self.command_check_max_steer_error_rad = math.radians(
            max(float(self.get_parameter('command_check_max_steer_error_deg').value), 1.0)
        )
        self.command_check_confirm_cycles = max(
            1,
            int(self.get_parameter('command_check_confirm_cycles').value)
        )
        self.command_check_feedback_timeout_sec = max(
            float(self.get_parameter('command_check_feedback_timeout_sec').value),
            0.05
        )
        self.command_check_min_speed_cmd = max(
            float(self.get_parameter('command_check_min_speed_cmd').value),
            0.0
        )
        self.command_check_steer_feedback_sign = (
            -1.0
            if float(self.get_parameter('command_check_steer_feedback_sign').value) < 0.0
            else 1.0
        )
        self.path_heading_back_points = int(
            self.get_parameter('path_heading_back_points').value
        )
        self.path_heading_forward_points = int(
            self.get_parameter('path_heading_forward_points').value
        )
        self.use_path_orientation = bool(
            self.get_parameter('use_path_orientation').value
        )
        self.use_path_target_speed = bool(
            self.get_parameter('use_path_target_speed').value
        )
        self.path_speed_profile_topic = str(
            self.get_parameter('path_speed_profile_topic').value
        )
        self.desired_speed_topic = str(self.get_parameter('desired_speed_topic').value)
        self.path_target_speed_preview_points = max(
            0,
            int(self.get_parameter('path_target_speed_preview_points').value)
        )
        self.path_target_speed_launch_floor = max(
            0.0,
            float(self.get_parameter('path_target_speed_launch_floor').value)
        )
        self.launch_detect_points = 10
        self.path_speed_max_lead_mps = max(
            0.0,
            float(self.get_parameter('path_speed_max_lead_mps').value)
        )
        self.path_speed_stall_lead_gain = max(
            0.0,
            float(self.get_parameter('path_speed_stall_lead_gain').value)
        )
        self.path_speed_stall_speed_mps = max(
            0.0,
            float(self.get_parameter('path_speed_stall_speed_mps').value)
        )
        self.path_speed_stall_sec = 0.0
        self.speed_from_path_profile_only = bool(
            self.get_parameter('speed_from_path_profile_only').value
        )
        self.speed_profile_passthrough = bool(
            self.get_parameter('speed_profile_passthrough').value
        )
        self.use_path_speed_profile = bool(
            self.get_parameter('use_path_speed_profile').value
        )
        self.path_speed_scale = float(self.get_parameter('path_speed_scale').value)
        self.path_speed_min_limit = float(
            self.get_parameter('path_speed_min_limit').value
        )
        self.path_speed_preview_points = max(
            1,
            int(self.get_parameter('path_speed_preview_points').value)
        )
        self.path_speed_limit_quantile = self.clamp(
            float(self.get_parameter('path_speed_limit_quantile').value),
            0.0,
            1.0
        )
        self.turn_angle_preview_points = max(
            1,
            int(self.get_parameter('turn_angle_preview_points').value)
        )
        self.turn_angle_lookahead_gain = float(
            self.get_parameter('turn_angle_lookahead_gain').value
        )
        self.turn_angle_speed_gain = float(
            self.get_parameter('turn_angle_speed_gain').value
        )
        self.turn_heading_blend_gain = float(
            self.get_parameter('turn_heading_blend_gain').value
        )
        self.turn_heading_blend_max = self.clamp(
            float(self.get_parameter('turn_heading_blend_max').value),
            self.lookahead_heading_blend,
            1.0,
        )
        self.cte_heading_gain = float(self.get_parameter('cte_heading_gain').value)
        self.cte_derivative_gain = float(self.get_parameter('cte_derivative_gain').value)
        self.cte_max_correction_rad = float(self.get_parameter('cte_max_correction_rad').value)
        self.cte_lookahead_gain = float(self.get_parameter('cte_lookahead_gain').value)
        self.cte_heading_blend_gain = float(
            self.get_parameter('cte_heading_blend_gain').value
        )
        self.cte_speed_ref_mps = max(
            float(self.get_parameter('cte_speed_ref_mps').value),
            0.1
        )
        self.straight_cte_low_speed_scale = max(
            float(self.get_parameter('straight_cte_low_speed_scale').value),
            0.0
        )
        self.straight_cte_high_speed_scale = max(
            float(self.get_parameter('straight_cte_high_speed_scale').value),
            0.0
        )
        self.curve_cte_low_speed_scale = max(
            float(self.get_parameter('curve_cte_low_speed_scale').value),
            0.0
        )
        self.curve_cte_high_speed_scale = max(
            float(self.get_parameter('curve_cte_high_speed_scale').value),
            0.0
        )
        self.curve_cte_curvature_ref = max(
            float(self.get_parameter('curve_cte_curvature_ref').value),
            1e-3
        )
        self.curve_cte_curvature_boost = max(
            float(self.get_parameter('curve_cte_curvature_boost').value),
            0.0
        )
        self.curve_cte_gain_scale = self.clamp(
            float(self.get_parameter('curve_cte_gain_scale').value),
            0.0,
            1.0
        )
        self.curve_cte_derivative_scale = self.clamp(
            float(self.get_parameter('curve_cte_derivative_scale').value),
            0.0,
            1.0
        )
        self.cte_rate_filter_alpha = self.clamp(
            float(self.get_parameter('cte_rate_filter_alpha').value),
            0.0,
            1.0
        )
        self.curve_sign_flip_cte_m = max(
            float(self.get_parameter('curve_sign_flip_cte_m').value),
            0.0
        )
        self.curve_sign_flip_heading_rad = math.radians(
            float(self.get_parameter('curve_sign_flip_heading_deg').value)
        )
        self.curve_sign_flip_damping = self.clamp(
            float(self.get_parameter('curve_sign_flip_damping').value),
            0.0,
            1.0
        )
        self.steering_filter_alpha = float(self.get_parameter('steering_filter_alpha').value)
        self.max_steer_rate_rad = float(self.get_parameter('max_steer_rate_rad').value)
        self.straight_high_speed_steer_limit_scale = self.clamp(
            float(self.get_parameter('straight_high_speed_steer_limit_scale').value),
            0.1,
            1.0
        )
        self.straight_high_speed_filter_scale = self.clamp(
            float(self.get_parameter('straight_high_speed_filter_scale').value),
            0.1,
            1.0
        )
        self.straight_high_speed_rate_scale = self.clamp(
            float(self.get_parameter('straight_high_speed_rate_scale').value),
            0.1,
            1.0
        )
        self.curve_high_speed_steer_limit_scale = self.clamp(
            float(self.get_parameter('curve_high_speed_steer_limit_scale').value),
            0.1,
            1.0
        )
        self.curve_high_speed_filter_scale = self.clamp(
            float(self.get_parameter('curve_high_speed_filter_scale').value),
            0.1,
            1.0
        )
        self.curve_high_speed_rate_scale = self.clamp(
            float(self.get_parameter('curve_high_speed_rate_scale').value),
            0.1,
            1.0
        )
        self.steer_damping_speed_low_ref = max(
            float(self.get_parameter('steer_damping_speed_low_ref').value),
            0.0
        )
        self.steer_damping_speed_high_ref = max(
            float(self.get_parameter('steer_damping_speed_high_ref').value),
            self.steer_damping_speed_low_ref + 1e-3
        )
        self.steering_deadband_rad = math.radians(
            float(self.get_parameter('steering_deadband_deg').value)
        )
        self.steering_activation_rad = math.radians(
            float(self.get_parameter('steering_activation_deg').value)
        )
        self.invert_steer_sign = bool(self.get_parameter('invert_steer_sign').value)
        self.turn_slowdown_heading_rad = math.radians(
            float(self.get_parameter('turn_slowdown_heading_deg').value)
        )
        self.steering_hold_cte_m = float(self.get_parameter('steering_hold_cte_m').value)
        self.steering_hold_heading_rad = math.radians(
            float(self.get_parameter('steering_hold_heading_deg').value)
        )
        self.steering_zero_band_rad = math.radians(
            float(self.get_parameter('steering_zero_band_deg').value)
        )
        self.straight_hold_cte_m = float(self.get_parameter('straight_hold_cte_m').value)
        self.straight_hold_heading_rad = math.radians(
            float(self.get_parameter('straight_hold_heading_deg').value)
        )
        self.straight_zero_band_rad = math.radians(
            float(self.get_parameter('straight_zero_band_deg').value)
        )
        self.straight_steering_activation_rad = math.radians(
            float(self.get_parameter('straight_steering_activation_deg').value)
        )
        self.straight_cte_slowdown_m = float(
            self.get_parameter('straight_cte_slowdown_m').value
        )
        self.straight_cte_turn_speed_m = float(
            self.get_parameter('straight_cte_turn_speed_m').value
        )
        self.straight_mode_max_steer_rad = math.radians(
            float(self.get_parameter('straight_mode_max_steer_deg').value)
        )
        self.curve_mode_max_steer_rad = math.radians(
            float(self.get_parameter('curve_mode_max_steer_deg').value)
        )
        self.curve_mode_full_curvature = max(
            float(self.get_parameter('curve_mode_full_curvature').value),
            self.straight_curvature_threshold + 1e-6
        )
        self.curve_cte_slowdown_m = float(
            self.get_parameter('curve_cte_slowdown_m').value
        )
        self.curve_cte_turn_speed_m = float(
            self.get_parameter('curve_cte_turn_speed_m').value
        )
        self.turn_preview_full_steer_rad = math.radians(
            float(self.get_parameter('turn_preview_full_steer_deg').value)
        )
        self.cte_recovery_full_steer_m = max(
            float(self.get_parameter('cte_recovery_full_steer_m').value),
            1e-3
        )
        self.hard_turn_preview_rad = math.radians(
            float(self.get_parameter('hard_turn_preview_deg').value)
        )
        self.hard_turn_curvature = max(
            float(self.get_parameter('hard_turn_curvature').value),
            self.straight_curvature_threshold + 1e-6
        )
        self.hard_turn_speed = max(
            self.stop_speed,
            float(self.get_parameter('hard_turn_speed').value)
        )
        self.curve_motion_min_speed = max(
            self.stop_speed,
            float(self.get_parameter('curve_motion_min_speed').value)
        )
        self.hard_turn_lookahead_distance = self.clamp(
            float(self.get_parameter('hard_turn_lookahead_distance').value),
            0.1,
            self.max_lookahead_distance
        )
        self.hard_turn_lookahead_bonus_scale = self.clamp(
            float(self.get_parameter('hard_turn_lookahead_bonus_scale').value),
            0.0,
            1.0
        )
        self.curve_steering_filter_alpha = self.clamp(
            float(self.get_parameter('curve_steering_filter_alpha').value),
            self.steering_filter_alpha,
            1.0
        )
        self.curve_max_steer_rate_rad = max(
            float(self.get_parameter('curve_max_steer_rate_rad').value),
            self.max_steer_rate_rad
        )

        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.yaw_filter_alpha = float(self.get_parameter('yaw_filter_alpha').value)
        self.motion_heading_min_dist = float(
            self.get_parameter('motion_heading_min_dist').value
        )
        self.motion_heading_weight = self.clamp(
            float(self.get_parameter('motion_heading_weight').value),
            0.0,
            1.0,
        )
        self.pose_speed_filter_alpha = float(
            self.get_parameter('pose_speed_filter_alpha').value
        )
        self.pose_speed_timeout_sec = float(
            self.get_parameter('pose_speed_timeout_sec').value
        )
        self.speed_status_topic = str(self.get_parameter('speed_status_topic').value)
        self.speed_status_timeout_sec = float(
            self.get_parameter('speed_status_timeout_sec').value
        )
        self.velocity_source = str(self.get_parameter('velocity_source').value).lower()
        if self.velocity_source not in ('auto', 'speed_status', 'pose', 'odom', 'gps'):
            self.get_logger().warn(
                f"velocity_source='{self.velocity_source}' ไม่รู้จัก -> ใช้ 'auto' แทน")
            self.velocity_source = 'auto'
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.speed_status_stuck_zero_sec = max(
            float(self.get_parameter('speed_status_stuck_zero_sec').value),
            0.0
        )
        self.moving_speed_epsilon = max(
            float(self.get_parameter('moving_speed_epsilon').value),
            0.0
        )
        self.use_gps_heading_fallback = bool(
            self.get_parameter('use_gps_heading_fallback').value
        )
        self.gps_velocity_topic = str(self.get_parameter('gps_velocity_topic').value)
        self.gps_heading_yaw_offset_rad = math.radians(
            float(self.get_parameter('gps_heading_yaw_offset_deg').value)
        )
        self.gps_heading_min_speed_mps = max(
            float(self.get_parameter('gps_heading_min_speed_mps').value),
            0.0
        )
        self.gps_heading_max_age_sec = max(
            float(self.get_parameter('gps_heading_max_age_sec').value),
            0.05
        )
        self.enable_pose_prediction = bool(
            self.get_parameter('enable_pose_prediction').value
        )
        self.localization_latency_sec = max(
            0.0,
            float(self.get_parameter('localization_latency_sec').value)
        )
        self.max_localization_latency_sec = max(
            0.0,
            float(self.get_parameter('max_localization_latency_sec').value)
        )
        self.turn_latency_scale = self.clamp(
            float(self.get_parameter('turn_latency_scale').value),
            0.0,
            1.0,
        )
        self.pose_stale_timeout_sec = max(
            float(self.get_parameter('pose_stale_timeout_sec').value),
            0.05
        )
        self.localization_latency_warn_sec = max(
            float(self.get_parameter('localization_latency_warn_sec').value),
            0.0
        )
        self.localization_latency_stop_sec = max(
            float(self.get_parameter('localization_latency_stop_sec').value),
            self.localization_latency_warn_sec + 1e-3
        )
        self.pose_interval_warn_sec = max(
            float(self.get_parameter('pose_interval_warn_sec').value),
            0.05
        )
        self.pose_interval_stop_sec = max(
            float(self.get_parameter('pose_interval_stop_sec').value),
            self.pose_interval_warn_sec + 1e-3
        )
        self.use_pose_header_latency_guard = bool(
            self.get_parameter('use_pose_header_latency_guard').value
        )
        self.localization_health_filter_alpha = self.clamp(
            float(self.get_parameter('localization_health_filter_alpha').value),
            0.0,
            1.0
        )
        self.localization_degraded_confirm_cycles = max(
            1,
            int(self.get_parameter('localization_degraded_confirm_cycles').value)
        )
        self.localization_stop_confirm_cycles = max(
            self.localization_degraded_confirm_cycles,
            int(self.get_parameter('localization_stop_confirm_cycles').value)
        )
        self.localization_recover_confirm_cycles = max(
            1,
            int(self.get_parameter('localization_recover_confirm_cycles').value)
        )
        self.localization_degraded_speed_scale = self.clamp(
            float(self.get_parameter('localization_degraded_speed_scale').value),
            0.05,
            1.0
        )
        self.localization_degraded_max_speed = max(
            float(self.get_parameter('localization_degraded_max_speed').value),
            0.0
        )
        self.speed_lookahead_gain = float(self.get_parameter('speed_lookahead_gain').value)
        self.min_control_speed_ref = float(self.get_parameter('min_control_speed_ref').value)
        self.steering_release_rad = math.radians(
            float(self.get_parameter('steering_release_deg').value)
        )
        self.broadcast_vehicle_tf = bool(self.get_parameter('broadcast_vehicle_tf').value)
        self.trace_min_dist = float(self.get_parameter('trace_min_dist').value)
        self.enable_controller_csv_log = bool(
            self.get_parameter('enable_controller_csv_log').value
        )
        self.controller_csv_log_path = os.path.expanduser(
            self.get_parameter('controller_csv_log_path').value
        )
        self.controller_csv_log_period = max(
            float(self.get_parameter('controller_csv_log_period').value),
            self.control_period if hasattr(self, 'control_period') else 0.01
        )
        self.controller_csv_log_flush_every_row = bool(
            self.get_parameter('controller_csv_log_flush_every_row').value
        )

        # =========================
        # Internal state
        # =========================
        self.path_points_local = []
        self.path_states_local = []
        self.path_target_speed_mps = []      # v_target ต่อ waypoint จาก nav_path
        self.path_speed_profile_raw = []     # อาร์เรย์ดิบล่าสุด รอจับคู่กับ path เฟรมถัดไป
        self.path_target_speed_warned = False
        self.path_segment_speed_mps = []
        self.path_ready = False

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.localization_x = None
        self.localization_y = None
        self.localization_z = None
        self.localization_yaw = None
        self.pose_received = False
        self.last_pose_wait_log_ns = 0
        self.last_pose_speed_log_ns = 0
        self.prev_steer_cmd = 0.0
        self.prev_steer_filtered = 0.0
        self.cte_integral = 0.0
        self.prev_cte_error = 0.0
        self.filtered_cte_rate = 0.0
        self.pid_initialized = False
        self.current_speed_x = 0.0
        self.pose_speed_received = False
        self.last_speed_update_ns = 0
        self.speed_status_value = 0.0
        self.speed_status_received = False
        self.last_speed_status_ns = 0
        self.speed_status_zero_since_ns = 0
        self.speed_status_healthy = True
        self.odom_speed_value = 0.0
        self.odom_speed_received = False
        self.last_odom_speed_ns = 0
        self.active_velocity_source = 'none'
        self.gps_course_map_yaw = None
        self.gps_course_speed_mps = 0.0
        self.gps_course_stamp_sec = None
        self.last_pose_stamp_sec = None
        self.last_pose_receive_sec = None
        self.filtered_pose_latency_sec = 0.0
        self.filtered_pose_interval_sec = 0.0
        self.localization_health_state = 'unknown'
        self.last_localization_health_log_ns = 0
        self.localization_warn_count = 0
        self.localization_stop_count = 0
        self.localization_recover_count = 0
        self.deadband_comp_active = False
        self.current_straight_mode = False
        self.current_speed_zone = 'red'
        self.last_cmd_speed = 0.0

        # CBF-QP state — u* ล่าสุดที่ได้จาก adaptive_ld_node
        self.qp_ld_star = None
        self.qp_v_star = None
        self.qp_speed_cap = math.inf
        self.last_qp_ld_ns = 0
        self.last_qp_speed_ns = 0
        self.last_qp_speed_cap_ns = 0
        self.qp_active = False
        self.qp_pure_pursuit = False
        self.last_qp_pure_pursuit_ns = 0
        self.pure_pursuit_active = False
        self.telemetry_curvature_preview = 0.0
        self.telemetry_desired_speed = None

        # Command Check state
        self.steering_angle_value = 0.0
        self.steering_angle_received = False
        self.last_steering_angle_ns = 0
        self.command_check_speed_count = 0
        self.command_check_steer_count = 0
        self.command_check_speed_ref = None
        self.last_command_check_log_ns = 0

        # safety state
        self.safety_stop_active = False
        self.last_stop_reason = None
        self.target_goal_x = None
        self.target_goal_y = None
        self.last_target_goal_msg_sec = None
        self.controller_log_file = None
        self.controller_log_writer = None
        self.controller_log_start_sec = self.get_now_sec()
        self.last_controller_log_sec = None
        self.controller_log_sample = 0

        # RViz trace state
        self.vehicle_trace = Path()
        self.vehicle_trace.header.frame_id = self.output_frame
        self.last_trace_x = None
        self.last_trace_y = None

        self.init_controller_csv_log()

        # TF broadcaster for RViz2
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # =========================
        # Subscribers
        # =========================
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )
        self.create_subscription(
            Float64MultiArray,
            self.path_speed_profile_topic,
            self.path_speed_profile_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )
        self.speed_status_sub = self.create_subscription(
            Float64, self.speed_status_topic, self.speed_status_callback, 10
        )
        self.gps_velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, self.gps_velocity_topic,
            self.gps_velocity_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        # u* = [v*, L_d*] จาก CBF-QP solver
        self.qp_ld_sub = self.create_subscription(
            Float64, self.cbf_qp_ld_topic, self.cbf_qp_ld_callback, 10
        )
        self.qp_speed_sub = self.create_subscription(
            Float64, self.cbf_qp_speed_topic, self.cbf_qp_speed_callback, 10
        )
        self.qp_speed_cap_sub = self.create_subscription(
            Float64, self.cbf_qp_speed_cap_topic, self.cbf_qp_speed_cap_callback, 10
        )
        self.qp_pure_pursuit_sub = self.create_subscription(
            Bool, self.cbf_qp_pure_pursuit_topic, self.cbf_qp_pure_pursuit_callback, 10
        )
        # มุมเลี้ยวจริงสำหรับ Command Check (Δδ)
        self.steering_angle_sub = self.create_subscription(
            Float64, self.steering_angle_topic, self.steering_angle_callback, 10
        )
        # =========================
        # Publishers
        # =========================
        self.target_pub = self.create_publisher(PoseStamped, '/target_waypoint', 10)
        self.lookahead_point_pub = self.create_publisher(PointStamped, '/lookahead_point', 10)
        self.closest_index_pub = self.create_publisher(Int32, '/closest_waypoint_index', 10)
        self.target_index_pub = self.create_publisher(Int32, '/target_waypoint_index', 10)
        self.goal_done_pub = self.create_publisher(Bool, '/path_following_done', 10)
        self.remaining_dist_pub = self.create_publisher(Float32, '/remaining_distance', 10)
        self.current_local_pub = self.create_publisher(PointStamped, '/current_local_position', 10)
        self.path_local_pub = self.create_publisher(Path, '/nav_path_local', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.effective_cmd_vel_topic, 10)
        self.controller_yaw_pub = self.create_publisher(Float64, '/controller_debug/yaw', 10)
        self.controller_cte_pub = self.create_publisher(Float64, '/controller_debug/cte', 10)
        self.path_heading_pub = self.create_publisher(Float64, '/controller_debug/path_heading', 10)
        self.heading_error_pub = self.create_publisher(Float64, '/controller_debug/heading_error', 10)
        self.lookahead_distance_pub = self.create_publisher(Float64, '/controller_debug/lookahead_distance', 10)
        self.adaptive_ld_debug_pub = self.create_publisher(Float64, '/controller_debug/adaptive_ld', 10)
        self.curvature_pub = self.create_publisher(Float64, self.curvature_topic, 10)
        self.velocity_pub = self.create_publisher(Float64, self.velocity_topic, 10)
        self.desired_speed_pub = self.create_publisher(
            Float64, self.desired_speed_topic, 10
        )
        self.lookahead_pub = self.create_publisher(Float64, self.lookahead_topic, 10)
        # κ_preview สำหรับ CBF-QP (h₂ ใช้ κ̂ = max(κ_cmd, κ_preview))
        self.curvature_preview_pub = self.create_publisher(
            Float64, self.curvature_preview_topic, 10
        )
        # Command Check telemetry (สมการข้อ 9)
        self.speed_error_pub = self.create_publisher(Float64, '/controller_debug/speed_error', 10)
        self.steer_error_pub = self.create_publisher(Float64, '/controller_debug/steer_error', 10)

        # Latest values streamed on curvature/velocity/lookahead every control tick.
        # Reset at the start of each tick so a loop that bails out early reports
        # "no path following" instead of a stale number.
        self.telemetry_curvature = 0.0
        self.telemetry_lookahead = self.min_lookahead_distance

        # RViz publishers
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.vehicle_trace_pub = self.create_publisher(Path, '/vehicle_trace', 10)
        self.cte_marker_pub = self.create_publisher(Marker, '/cte_line', 10)
        self.vehicle_marker_pub = self.create_publisher(MarkerArray, '/vehicle_model', 10)

        self.control_period = 0.05
        self.timer = self.create_timer(self.control_period, self.timer_callback)

        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Pose topic: {self.pose_topic}')
        if self.dry_run:
            self.get_logger().warn(
                f'*** DRY RUN *** คำสั่งออกที่ {self.effective_cmd_vel_topic} '
                f'ไม่ใช่ {self.cmd_vel_topic} -> รถไม่ขยับ '
                f'(ตั้ง dry_run:=false เมื่อพร้อมสั่งรถจริง)'
            )
        else:
            self.get_logger().info(f'CMD topic: {self.effective_cmd_vel_topic} (LIVE)')
        self.get_logger().info(
            f'Target goal topic: {self.target_goal_topic} (ignored; path-only control mode)'
        )
        self.get_logger().info(
            f'Path coordinate safety limit: |x|, |y| <= {self.path_coordinate_limit:.1f}'
        )
        self.get_logger().info(f'Output frame: {self.output_frame}')
        self.get_logger().info(f'World frame: {self.world_frame}')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info(
            f'Dynamic lookahead: min={self.min_lookahead_distance:.2f} m, '
            f'max={self.max_lookahead_distance:.2f} m, '
            f'speed_gain={self.speed_lookahead_gain:.2f}, '
            f'curve_gain={self.curve_lookahead_gain:.2f}, '
            f'turn_gain={self.lookahead_turn_gain:.2f}, '
            f'straight_bonus={self.straight_lookahead_bonus:.2f} m, '
            f'curve_bonus={self.curve_lookahead_bonus:.2f} m, '
            f'hard_turn_floor={self.hard_turn_lookahead_distance:.2f} m'
        )
        if self.use_cbf_qp:
            self.get_logger().info(
                f'CBF-QP enabled: u* = [v*, L_d*] from {self.cbf_qp_speed_topic} / '
                f'{self.cbf_qp_ld_topic} (speed_mode={self.cbf_speed_mode}, '
                f'timeout={self.cbf_qp_timeout_sec:.2f}s -> fallback to heuristic)'
            )
            self.get_logger().info(
                f'CBF-QP state feed: e_lat -> /controller_debug/cte, '
                f'θ_e -> /controller_debug/heading_error, v -> {self.velocity_topic}, '
                f'κ -> {self.curvature_topic}, '
                f'κ_preview -> {self.curvature_preview_topic} '
                f'(preview distance = v/α₂, α₂={self.cbf_preview_alpha2:.2f}, '
                f'clamped {self.cbf_preview_min_distance:.1f}-'
                f'{self.cbf_preview_max_distance:.1f} m)'
            )
        else:
            self.get_logger().info(
                'CBF-QP disabled -> ใช้ heuristic lookahead/speed ชุดเดิมอย่างเดียว'
            )
        if self.command_check_enable:
            self.get_logger().info(
                f'Command check: |Δv| > {self.command_check_max_speed_error:.2f} m/s vs '
                f'{self.speed_status_topic} หรือ |Δδ| > '
                f'{math.degrees(self.command_check_max_steer_error_rad):.1f} deg vs '
                f'{self.steering_angle_topic} ติดต่อกัน '
                f'{self.command_check_confirm_cycles} รอบ -> safety stop'
            )
        elif self.dry_run:
            self.get_logger().info(
                'Command check ปิดอัตโนมัติเพราะอยู่ใน dry run '
                '(รถไม่ได้รับคำสั่ง จะเทียบ Δv/Δδ ไม่ได้)'
            )
        else:
            self.get_logger().info('Command check disabled')
        self.get_logger().info(
            f'Telemetry published every control tick to {self.curvature_topic}, '
            f'{self.velocity_topic}, {self.lookahead_topic} and '
            f'{self.curvature_preview_topic}'
        )
        self.get_logger().info(
            f'Steer tuning: filter_alpha={self.steering_filter_alpha:.2f}, '
            f'rate_limit={self.max_steer_rate_rad:.2f} rad/s, '
            f'cte_gain={self.cte_heading_gain:.2f}, '
            f'cte_d={self.cte_derivative_gain:.2f}'
        )
        self.get_logger().info(
            f'Speed-aware steer scaling: straight limit/filter/rate='
            f'{self.straight_high_speed_steer_limit_scale:.2f}/'
            f'{self.straight_high_speed_filter_scale:.2f}/'
            f'{self.straight_high_speed_rate_scale:.2f}, '
            f'curve='
            f'{self.curve_high_speed_steer_limit_scale:.2f}/'
            f'{self.curve_high_speed_filter_scale:.2f}/'
            f'{self.curve_high_speed_rate_scale:.2f}'
        )
        self.get_logger().info(
            f'Steer sign inversion: {"enabled" if self.invert_steer_sign else "disabled"}'
        )
        self.get_logger().info(
            f'Steer deadband compensation: {math.degrees(self.steering_deadband_rad):.1f} deg '
            f'(activation {math.degrees(self.steering_activation_rad):.1f} deg)'
        )
        self.get_logger().info(
            f'Heading blend={self.lookahead_heading_blend:.2f}-{self.lookahead_heading_blend_max:.2f}, '
            f'cte_limit={self.cte_max_correction_rad:.2f} rad'
        )
        self.get_logger().info(
            f'Curve tuning: lookahead_gain={self.curve_lookahead_gain:.2f}, '
            f'speed_gain={self.curve_speed_gain:.2f}, '
            f'heading_gain={self.curve_heading_blend_gain:.2f}, '
            f'preview_points={self.curve_preview_points}, '
            f'preview_gain={self.curve_preview_gain:.2f}'
        )
        self.get_logger().info(
            f'Path profile: orientation={self.use_path_orientation}, '
            f'speed_profile={self.use_path_speed_profile}, '
            f'speed_scale={self.path_speed_scale:.2f}, '
            f'quantile={self.path_speed_limit_quantile:.2f}'
        )
        self.get_logger().info(
            f'Turn preview: angle_points={self.turn_angle_preview_points}, '
            f'lookahead_gain={self.turn_angle_lookahead_gain:.2f}, '
            f'speed_gain={self.turn_angle_speed_gain:.2f}, '
            f'heading_gain={self.turn_heading_blend_gain:.2f}'
        )
        self.get_logger().info(
            f'Straight tuning: curvature<{self.straight_curvature_threshold:.3f}, '
            f'lookahead_bonus={self.straight_lookahead_bonus:.2f}, '
            f'cte_scale={self.straight_cte_gain_scale:.2f}'
        )
        self.get_logger().info(
            f'Curve lookahead bonus: +{self.curve_lookahead_bonus:.2f} m scaled by turn anticipation'
        )
        self.get_logger().info(
            f'Straight path lock: cte<{self.straight_path_lock_cte_m:.2f} m, '
            f'bonus_fade={self.straight_bonus_fade_cte_m:.2f} m'
        )
        self.get_logger().info(
            f'Straight steering activation: {math.degrees(self.straight_steering_activation_rad):.1f} deg'
        )
        self.get_logger().info(
            f'Pose speed: filter_alpha={self.pose_speed_filter_alpha:.2f}, '
            f'timeout={self.pose_speed_timeout_sec:.2f}s, '
            f'min_ref={self.min_control_speed_ref:.2f}'
        )
        self.get_logger().info(
            f'Velocity source: {self.velocity_source} '
            f'(auto = {self.speed_status_topic} -> {self.pose_topic} -> {self.odom_topic}), '
            f'stuck-zero guard={self.speed_status_stuck_zero_sec:.1f}s '
            f'@ >{self.moving_speed_epsilon:.2f} m/s'
        )
        self.get_logger().info(
            f'Speed tuning: binary_mode={self.binary_speed_mode}, '
            f'cruise={self.forward_speed:.2f}, slow={self.slow_speed:.2f}, '
            f'turn={self.turn_speed:.2f}, slowdown_distance={self.slowdown_distance:.2f} m'
        )
        self.get_logger().info(
            f'Speed command mapping: cmd = target_mps * {self.speed_command_scale:.3f} '
            f'+ {self.speed_command_offset:.3f}, max={self.speed_command_max:.3f}'
        )
        self.get_logger().info(
            f'Speed feedback: up_kp={self.speed_feedback_up_kp:.2f}, '
            f'down_kp={self.speed_feedback_down_kp:.2f}, '
            f'max_delta={self.speed_feedback_max_delta:.2f}'
        )
        self.get_logger().info(
            'Speed-based steer zones disabled; using road-shape steer limit only'
        )
        self.get_logger().info(
            f'Steer hold zone: cte<{self.steering_hold_cte_m:.2f} m, '
            f'heading<{math.degrees(self.steering_hold_heading_rad):.1f} deg'
        )
        self.get_logger().info(
            f'Steer release threshold: {math.degrees(self.steering_release_rad):.1f} deg'
        )
        self.get_logger().info(
            f'Straight hold zone: cte<{self.straight_hold_cte_m:.2f} m, '
            f'heading<{math.degrees(self.straight_hold_heading_rad):.1f} deg'
        )
        self.get_logger().info(
            f'Straight slowdown: slow>{self.straight_cte_slowdown_m:.2f} m, '
            f'turn>{self.straight_cte_turn_speed_m:.2f} m'
        )
        self.get_logger().info(
            f'Road steer limits: straight={math.degrees(self.straight_mode_max_steer_rad):.1f} deg, '
            f'curve={math.degrees(self.curve_mode_max_steer_rad):.1f} deg, '
            f'full_curve_curvature={self.curve_mode_full_curvature:.3f}'
        )
        self.get_logger().info(
            f'Curve slowdown: slow>{self.curve_cte_slowdown_m:.2f} m, '
            f'turn>{self.curve_cte_turn_speed_m:.2f} m'
        )
        self.get_logger().info(
            f'Turn steer assist: full_preview={math.degrees(self.turn_preview_full_steer_rad):.1f} deg, '
            f'cte_recovery={self.cte_recovery_full_steer_m:.2f} m'
        )
        self.get_logger().info(
            'Speed-based steer zones disabled; steering limit blends by road shape and recovery'
        )
        self.get_logger().info(
            f'Hard turn mode: preview>{math.degrees(self.hard_turn_preview_rad):.1f} deg, '
            f'curvature>{self.hard_turn_curvature:.3f}, '
            f'speed<={self.hard_turn_speed:.2f}, '
            f'curve_floor>={self.curve_motion_min_speed:.2f}, '
            f'lookahead_floor>={self.hard_turn_lookahead_distance:.2f}, '
            f'bonus_scale={self.hard_turn_lookahead_bonus_scale:.2f}'
        )
        self.get_logger().info(
            f'Hard turn steering response: alpha={self.curve_steering_filter_alpha:.2f}, '
            f'rate_limit={self.curve_max_steer_rate_rad:.2f} rad/s'
        )


        self.publish_static_world_to_output_tf()
        self.get_logger().info(f'Yaw offset: {self.yaw_offset:.3f} rad')
        self.get_logger().info(f'Yaw filter alpha: {self.yaw_filter_alpha:.3f}')
        self.get_logger().info(
            f'Motion heading: min_dist={self.motion_heading_min_dist:.2f} m, '
            f'weight={self.motion_heading_weight:.2f}'
        )
        self.get_logger().info(
            f'Pose prediction: {"enabled" if self.enable_pose_prediction else "disabled"}, '
            f'latency compensation base={self.localization_latency_sec:.2f}s, '
            f'max={self.max_localization_latency_sec:.2f}s, '
            f'turn_scale={self.turn_latency_scale:.2f}'
        )
        self.get_logger().info(
            'Localization health: '
            f'stale_timeout={self.pose_stale_timeout_sec:.2f}s, '
            f'warn_latency>={self.localization_latency_warn_sec:.2f}s, '
            f'stop_latency>={self.localization_latency_stop_sec:.2f}s, '
            f'warn_interval>={self.pose_interval_warn_sec:.2f}s, '
            f'stop_interval>={self.pose_interval_stop_sec:.2f}s, '
            f'use_header_latency={self.use_pose_header_latency_guard}, '
            f'confirm(degraded/stop/recover)='
            f'{self.localization_degraded_confirm_cycles}/'
            f'{self.localization_stop_confirm_cycles}/'
            f'{self.localization_recover_confirm_cycles}'
        )

    def init_controller_csv_log(self):
        if not self.enable_controller_csv_log:
            return

        log_dir = os.path.dirname(self.controller_csv_log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        self.controller_log_file = open(
            self.controller_csv_log_path,
            'w',
            newline='',
            encoding='utf-8'
        )
        self.controller_log_writer = csv.writer(self.controller_log_file)
        self.controller_log_writer.writerow([
            'sample',
            'time_sec',
            'controller_frequency_hz',
            'vehicle_position_x',
            'vehicle_position_y',
            'target_point_x',
            'target_point_y',
            'target_distance',
            'velocity',
            'yaw',
            'cte',
            'path_heading',
            'heading_error',
            'curvature_kappa',
            'steering_angle',
            'adaptive_ld',
            'lookahead_distance',
            'cmd_speed',
            'cmd_steer',
            'closest_waypoint_index',
            'target_waypoint_index',
            'remaining_distance',
            # CBF-QP columns (ต่อท้ายไว้เพื่อไม่ให้ schema เดิมของ logger/plot พัง)
            'curvature_preview',
            'qp_v_star',
            'qp_ld_star',
            'qp_active',
        ])
        self.controller_log_file.flush()
        self.get_logger().info(
            f'Controller CSV log enabled: {self.controller_csv_log_path}'
        )

    def maybe_log_controller_row(
        self,
        *,
        target_x,
        target_y,
        target_distance,
        velocity,
        yaw,
        cte,
        path_heading,
        heading_error,
        curvature_kappa,
        steering_angle,
        adaptive_ld,
        lookahead_distance,
        cmd_speed,
        cmd_steer,
        closest_waypoint_index,
        target_waypoint_index,
        remaining_distance,
        curvature_preview=0.0,
        qp_v_star=None,
        qp_ld_star=None
    ):
        if self.controller_log_writer is None:
            return

        now_sec = self.get_now_sec()
        if self.last_controller_log_sec is not None:
            dt = now_sec - self.last_controller_log_sec
            if dt < self.controller_csv_log_period:
                return
            frequency_hz = 1.0 / max(dt, 1e-6)
        else:
            frequency_hz = 1.0 / max(self.controller_csv_log_period, 1e-6)

        self.last_controller_log_sec = now_sec
        self.controller_log_sample += 1
        elapsed_sec = now_sec - self.controller_log_start_sec
        self.controller_log_writer.writerow([
            self.controller_log_sample,
            round(elapsed_sec, 6),
            round(frequency_hz, 3),
            self.current_x,
            self.current_y,
            target_x,
            target_y,
            target_distance,
            velocity,
            yaw,
            cte,
            path_heading,
            heading_error,
            curvature_kappa,
            steering_angle,
            adaptive_ld,
            lookahead_distance,
            cmd_speed,
            cmd_steer,
            closest_waypoint_index,
            target_waypoint_index,
            remaining_distance,
            curvature_preview,
            qp_v_star,
            qp_ld_star,
            int(qp_ld_star is not None and qp_v_star is not None),
        ])
        if self.controller_csv_log_flush_every_row:
            self.controller_log_file.flush()

    def close_controller_csv_log(self):
        if self.controller_log_file is None:
            return
        self.controller_log_file.flush()
        self.controller_log_file.close()
        self.controller_log_file = None
        self.controller_log_writer = None

    # =========================
    # Safety helpers
    # =========================
    def is_invalid_local_point(self, x, y, z):
        return not (
            math.isfinite(x) and
            math.isfinite(y) and
            math.isfinite(z)
        )

    def publish_empty_local_path(self):
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = self.output_frame
        self.path_local_pub.publish(empty_path)

    def reset_trace(self):
        self.vehicle_trace = Path()
        self.vehicle_trace.header.frame_id = self.output_frame
        self.last_trace_x = None
        self.last_trace_y = None
        self.vehicle_trace_pub.publish(self.vehicle_trace)

    def clear_cte_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.output_frame
        marker.ns = 'cte_line'
        marker.id = 0
        marker.action = Marker.DELETE
        self.cte_marker_pub.publish(marker)

    def trigger_safety_stop(self, reason):
        if not self.safety_stop_active:
            self.get_logger().error(f'SAFETY STOP: {reason}')

        self.safety_stop_active = True
        self.path_ready = False
        self.path_points_local = []
        self.path_states_local = []
        self.path_segment_speed_mps = []

        self.publish_empty_local_path()
        self.clear_cte_marker()
        self.publish_stop(reason)

        done = Bool()
        done.data = False
        self.goal_done_pub.publish(done)

        remain = Float32()
        remain.data = 0.0
        self.remaining_dist_pub.publish(remain)

    # =========================
    # Path callback
    # =========================
    def path_speed_profile_callback(self, msg: Float64MultiArray):
        """v_target ต่อ waypoint จาก nav_path (NaN = จุดนั้นไม่มีค่าใน CSV)

        เก็บอาร์เรย์ดิบไว้ก่อน แล้วค่อยจับคู่กับ path ตอน bind_path_target_speed()
        เพราะสองหัวข้อนี้มาคนละข้อความ ลำดับมาถึงไม่การันตี
        """
        self.path_speed_profile_raw = [
            None if math.isnan(value) else abs(float(value))
            for value in msg.data
        ]
        self.bind_path_target_speed()

    def bind_path_target_speed(self):
        """ผูกโปรไฟล์ v_target เข้ากับ path ปัจจุบัน ถ้าความยาวไม่ตรงถือว่าใช้ไม่ได้

        path_states_local อาจยาวกว่าโปรไฟล์อยู่ 1 จุดเมื่อ should_close_path_loop()
        เติมจุดแรกต่อท้ายเพื่อปิดลูป — กรณีนั้นเติมค่าแรกต่อท้ายให้ตรงกัน
        """
        profile = self.path_speed_profile_raw
        path_length = len(self.path_states_local)

        if not self.use_path_target_speed or not profile or path_length == 0:
            self.path_target_speed_mps = []
            return

        if len(profile) == path_length - 1:
            profile = profile + [profile[0]]

        if len(profile) != path_length:
            # ปกติเกิดชั่วคราวตอน path เพิ่งเปลี่ยนแต่โปรไฟล์ยังไม่มาถึง
            self.path_target_speed_mps = []
            if not self.path_target_speed_warned:
                self.path_target_speed_warned = True
                self.get_logger().warn(
                    f'speed profile ยาว {len(self.path_speed_profile_raw)} จุด '
                    f'แต่ path ยาว {path_length} จุด -> ยังใช้ v_target ไม่ได้ '
                    f'รอให้ {self.path_speed_profile_topic} กับ {self.path_topic} ตรงกันก่อน')
            return

        if self.path_target_speed_warned:
            self.get_logger().info(
                f'speed profile ตรงกับ path แล้ว ({path_length} จุด) -> ใช้ v_target ต่อ')
        self.path_target_speed_warned = False
        self.path_target_speed_mps = profile

    def get_path_target_speed(self, center_index):
        """v_des ที่ waypoint ปัจจุบัน

        preview_points = 0 -> ใช้ v_target ตรงจุดนั้นตรง ๆ (ค่าจาก CSV ล้วน)
        preview_points > 0 -> เอาค่าต่ำสุดในช่วงข้างหน้า เพื่อชะลอก่อนถึงโซนช้า
        คืน None ถ้าไม่มีโปรไฟล์ ให้ผู้เรียกถอยไปใช้พฤติกรรมเดิม
        """
        if not self.path_target_speed_mps:
            return None

        last_index = len(self.path_target_speed_mps) - 1
        start_index = max(0, min(center_index, last_index))

        end_index = min(last_index, start_index + self.path_target_speed_preview_points)
        candidates = [
            self.path_target_speed_mps[index]
            for index in range(start_index, end_index + 1)
            if self.path_target_speed_mps[index] is not None
        ]
        if not candidates:
            return None

        target_speed = min(candidates)

        # ยก floor เฉพาะกรณี "ออกตัว": v_target ตรงจุดที่รถอยู่ต่ำกว่า floor อยู่แล้ว
        # แต่ข้างหน้าเร่งขึ้นเกิน floor แปลว่ากำลังอยู่ต้นทางลาดเร่ง
        # ตัวตรวจนี้มองไปข้างหน้าเป็นระยะของตัวเอง ไม่ผูกกับ preview_points เพราะ
        # preview_points = 0 (โหมด passthrough) จะทำให้มองเห็นแค่จุดเดียวแล้วตรวจไม่เจอ
        if (
            self.path_target_speed_launch_floor > 0.0
            and target_speed < self.path_target_speed_launch_floor
        ):
            if self.is_path_loop_closed():
                # จุดสุดท้ายของวงคือจุดแรกซ้ำ จึงวนด้วย last_index
                ahead_indices = [
                    (start_index + offset) % last_index
                    for offset in range(self.launch_detect_points + 1)
                ]
            else:
                launch_end = min(last_index, start_index + self.launch_detect_points)
                ahead_indices = range(start_index, launch_end + 1)
            ahead = [
                self.path_target_speed_mps[index]
                for index in ahead_indices
                if self.path_target_speed_mps[index] is not None
            ]
            if ahead and max(ahead) > self.path_target_speed_launch_floor:
                return self.path_target_speed_launch_floor
        return target_speed

    def limit_path_speed_rise(self, target_speed_mps):
        """จำกัดขาเร่งของ v_target ให้นำความเร็ววัดจริงได้ไม่เกิน path_speed_max_lead_mps

        ไม่เก็บค่าคำสั่งเก่าไว้ จึงหายเองเมื่อรถเร็วถึงโปรไฟล์ และค่าวัดเพี้ยนรอบเดียว
        (เช่น 327.67) มีผลแค่รอบนั้น
        """
        if self.path_speed_max_lead_mps <= 0.0:
            return target_speed_mps

        measured_speed_mps = self.get_fresh_pose_speed_mps()
        if measured_speed_mps is None:
            return target_speed_mps

        if measured_speed_mps < self.path_speed_stall_speed_mps:
            self.path_speed_stall_sec += self.control_period
        else:
            self.path_speed_stall_sec = 0.0

        return min(
            target_speed_mps,
            measured_speed_mps
            + self.path_speed_max_lead_mps
            + self.path_speed_stall_lead_gain * self.path_speed_stall_sec,
        )

    def path_callback(self, msg: Path):
        # path ว่าง = ไม่มี target ที่ active แล้ว ให้ล้าง path เดิมและหยุดรถ
        if not msg.poses:
            if self.path_ready or self.path_points_local:
                self.get_logger().info(
                    f'Received empty {self.path_topic}. Clearing active path and stopping.'
                )
            self.path_ready = False
            self.path_points_local = []
            self.path_states_local = []
            self.path_segment_speed_mps = []
            self.path_target_speed_mps = []
            self.publish_empty_local_path()
            self.clear_cte_marker()
            self.publish_stop('empty_path_received')
            done = Bool()
            done.data = False
            self.goal_done_pub.publish(done)

            remain = Float32()
            remain.data = 0.0
            self.remaining_dist_pub.publish(remain)
            return

        # nav_path ส่งค่า x/y/z มาในเฟรม local/map โดยตรง
        for i, p in enumerate(msg.poses):
            x = float(p.pose.position.x)
            y = float(p.pose.position.y)
            z = float(p.pose.position.z)

            if self.is_invalid_local_point(x, y, z):
                self.get_logger().warn(
                    f'Ignore invalid path point at index {i}: x={x}, y={y}, z={z}'
                )
                return

            if (
                abs(x) > self.path_coordinate_limit
                or abs(y) > self.path_coordinate_limit
            ):
                self.trigger_safety_stop(
                    f'path_coordinate_limit_exceeded index={i} x={x:.3f} y={y:.3f} '
                    f'limit={self.path_coordinate_limit:.1f}'
                )
                return

        # ถ้า path มีน้อยเกินไป ให้ข้ามไป ไม่ reset ระบบ
        if len(msg.poses) < 2:
            self.get_logger().warn(
                f'Ignore short {self.path_topic} update: {len(msg.poses)} point(s)'
            )
            return

        # ถ้า path ใหม่ valid ให้ปลด safety stop
        if self.safety_stop_active:
            self.get_logger().warn('Valid path received. Clearing safety stop.')
            # counter ของ command check ต้องเริ่มนับใหม่ด้วย ไม่งั้นมันยังค้างที่
            # confirm_cycles อยู่ รอบควบคุมถัดไปจะทริป safety stop ซ้ำตั้งแต่รอบแรก
            # รถได้คันเร่งครั้งละรอบเดียวแล้วโดนตัด วนแบบนี้ไม่มีทางออกตัวได้เลย
            self.command_check_speed_count = 0
            self.command_check_steer_count = 0
            self.command_check_speed_ref = None
        self.safety_stop_active = False

        self.path_points_local = []
        self.path_states_local = []
        self.path_segment_speed_mps = []

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.output_frame

        for p in msg.poses:
            local_x = float(p.pose.position.x)
            local_y = float(p.pose.position.y)
            local_z = float(p.pose.position.z)

            self.path_points_local.append((local_x, local_y))
            self.path_states_local.append(
                {
                    'x': local_x,
                    'y': local_y,
                    'z': local_z,
                    'yaw': self.extract_path_pose_yaw(p),
                    'stamp_sec': self.stamp_to_sec(p.header.stamp),
                }
            )

            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(local_x)
            ps.pose.position.y = float(local_y)
            ps.pose.position.z = float(local_z)
            ps.pose.orientation = p.pose.orientation
            out.poses.append(ps)

        if self.should_close_path_loop():
            first_x, first_y = self.path_points_local[0]
            first_state = self.path_states_local[0]
            self.path_points_local.append((first_x, first_y))
            self.path_states_local.append(
                {
                    'x': first_state['x'],
                    'y': first_state['y'],
                    'z': first_state['z'],
                    'yaw': first_state['yaw'],
                    'stamp_sec': first_state['stamp_sec'],
                }
            )

            loop_pose = PoseStamped()
            loop_pose.header = out.header
            loop_pose.pose.position.x = float(first_state['x'])
            loop_pose.pose.position.y = float(first_state['y'])
            loop_pose.pose.position.z = float(first_state['z'])
            loop_pose.pose.orientation = out.poses[0].pose.orientation
            out.poses.append(loop_pose)

        self.path_segment_speed_mps = self.compute_path_segment_speeds()
        self.bind_path_target_speed()
        self.path_local_pub.publish(out)
        self.path_ready = len(self.path_points_local) >= 2

        self.get_logger().info(
            f'Path updated from {self.path_topic}: points={len(self.path_points_local)}'
        )

    # =========================
    # Pose callback
    # =========================
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.update_current_pose(msg.header, msg.pose.pose)

    def target_goal_callback(self, msg: PoseStamped):
        self.target_goal_x = float(msg.pose.position.x)
        self.target_goal_y = float(msg.pose.position.y)
        self.last_target_goal_msg_sec = self.get_clock().now().nanoseconds * 1e-9

    def has_active_target_goal(self):
        if (
            self.target_goal_x is None
            or self.target_goal_y is None
            or self.last_target_goal_msg_sec is None
        ):
            return False

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return now_sec - self.last_target_goal_msg_sec <= self.target_goal_timeout_sec

    def update_current_pose(self, header, pose):
        first_pose = not self.pose_received
        now_sec = self.get_now_sec()
        pose_stamp_sec = self.stamp_to_sec(header.stamp)
        pose_latency_sec = max(0.0, now_sec - pose_stamp_sec)
        quaternion_yaw = self.normalize_angle(
            self.quaternion_to_yaw(pose.orientation) + self.yaw_offset
        )
        previous_x = self.localization_x
        previous_y = self.localization_y
        previous_z = self.localization_z
        previous_stamp_sec = self.last_pose_stamp_sec
        previous_receive_sec = self.last_pose_receive_sec

        self.pose_received = True
        self.localization_x = float(pose.position.x)
        self.localization_y = float(pose.position.y)
        self.localization_z = float(pose.position.z)
        self.last_pose_stamp_sec = pose_stamp_sec
        self.last_pose_receive_sec = now_sec

        if first_pose:
            self.filtered_pose_latency_sec = pose_latency_sec
            self.filtered_pose_interval_sec = 0.0
        else:
            self.filtered_pose_latency_sec = (
                self.filtered_pose_latency_sec
                + self.localization_health_filter_alpha
                * (pose_latency_sec - self.filtered_pose_latency_sec)
            )
            if previous_receive_sec is not None and now_sec > previous_receive_sec:
                pose_interval_sec = now_sec - previous_receive_sec
                self.filtered_pose_interval_sec = (
                    self.filtered_pose_interval_sec
                    + self.localization_health_filter_alpha
                    * (pose_interval_sec - self.filtered_pose_interval_sec)
                )

        yaw_source = quaternion_yaw

        if (
            previous_x is not None and previous_y is not None and previous_z is not None
            and previous_stamp_sec is not None and pose_stamp_sec > previous_stamp_sec + 1e-6
        ):
            dt = pose_stamp_sec - previous_stamp_sec
            dx = self.localization_x - previous_x
            dy = self.localization_y - previous_y
            dz = self.localization_z - previous_z
            planar_dist = math.hypot(dx, dy)

            if planar_dist >= self.motion_heading_min_dist:
                motion_yaw = math.atan2(dy, dx)
                yaw_source = self.normalize_angle(
                    quaternion_yaw
                    + self.motion_heading_weight
                    * self.normalize_angle(motion_yaw - quaternion_yaw)
                )

            raw_speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            if not self.pose_speed_received:
                self.current_speed_x = raw_speed
                self.pose_speed_received = True
            else:
                self.current_speed_x = (
                    self.current_speed_x
                    + self.pose_speed_filter_alpha * (raw_speed - self.current_speed_x)
                )
            self.last_speed_update_ns = self.get_clock().now().nanoseconds

        if self.localization_yaw is None:
            self.localization_yaw = yaw_source
        else:
            yaw_error = self.normalize_angle(yaw_source - self.localization_yaw)
            self.localization_yaw = self.normalize_angle(
                self.localization_yaw + self.yaw_filter_alpha * yaw_error
            )

        self.update_predicted_state()

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = self.output_frame
        pt.point.x = float(self.current_x)
        pt.point.y = float(self.current_y)
        pt.point.z = float(self.current_z)
        self.current_local_pub.publish(pt)

        if first_pose:
            self.get_logger().info(
                f'Pose received on {self.pose_topic}: '
                f'x={self.localization_x:.3f}, y={self.localization_y:.3f}, z={self.localization_z:.3f}'
            )

    def update_predicted_state(self):
        if (
            self.localization_x is None
            or self.localization_y is None
            or self.localization_z is None
        ):
            return

        base_yaw = (
            self.localization_yaw
            if self.localization_yaw is not None
            else (self.current_yaw if self.current_yaw is not None else 0.0)
        )
        pose_speed = self.get_fresh_pose_speed_mps()
        if pose_speed is not None:
            predicted_speed = max(abs(pose_speed), 0.0)
        else:
            predicted_speed = max(abs(self.last_cmd_speed), 0.0)

        latency = min(self.localization_latency_sec, self.max_localization_latency_sec)
        if abs(self.prev_steer_cmd) > self.steering_activation_rad:
            latency *= self.turn_latency_scale

        predicted_x = self.localization_x
        predicted_y = self.localization_y
        predicted_yaw = base_yaw

        if self.enable_pose_prediction and latency > 1e-6 and predicted_speed > 1e-4:
            yaw_rate = 0.0
            if self.wheelbase > 1e-6:
                yaw_rate = predicted_speed * math.tan(self.prev_steer_cmd) / self.wheelbase

            if abs(yaw_rate) > 1e-5:
                predicted_yaw = self.normalize_angle(base_yaw + yaw_rate * latency)
                radius = predicted_speed / yaw_rate
                predicted_x = self.localization_x + radius * (
                    math.sin(predicted_yaw) - math.sin(base_yaw)
                )
                predicted_y = self.localization_y - radius * (
                    math.cos(predicted_yaw) - math.cos(base_yaw)
                )
            else:
                predicted_x = self.localization_x + predicted_speed * latency * math.cos(base_yaw)
                predicted_y = self.localization_y + predicted_speed * latency * math.sin(base_yaw)

        self.current_x = predicted_x
        self.current_y = predicted_y
        self.current_z = self.localization_z
        self.current_yaw = predicted_yaw

    # =========================
    # Main loop
    # =========================
    def timer_callback(self):
        # curvature/velocity/lookahead ต้องออกทุกรอบ แม้ control loop จะ return ก่อน
        # (path ยังไม่พร้อม, pose หาย, safety stop) เพื่อให้ logger/plot ไม่ขาดช่วง
        self.telemetry_curvature = 0.0
        self.telemetry_lookahead = self.min_lookahead_distance
        self.telemetry_curvature_preview = 0.0
        # None = ยังไม่มีโปรไฟล์ให้ใช้ -> ไม่ publish เลย ปล่อยให้ QP ใช้ v_des ของตัวเอง
        self.telemetry_desired_speed = None
        try:
            self.run_control_step()
        finally:
            self.publish_lookahead_telemetry()

    def run_control_step(self):
        # ให้ RViz มี pose/TF ต่อเนื่อง แม้ยังไม่มี path
        if self.current_x is not None and self.current_y is not None and self.current_yaw is not None:
            self.publish_current_pose()
            if self.broadcast_vehicle_tf:
                self.publish_vehicle_tf()
            self.update_vehicle_trace_if_ready()
            self.publish_vehicle_marker()

        if self.safety_stop_active:
            self.clear_cte_marker()
            self.publish_stop('safety_stop_active')
            return

        if not self.path_ready:
            self.clear_cte_marker()
            self.publish_stop('path_not_ready')
            return

        if self.current_x is None or self.current_y is None:
            self.log_wait_once_in_a_while('pose')
            return
        if self.current_yaw is None:
            self.log_wait_once_in_a_while('pose')
            return

        localization_state, localization_reason = self.evaluate_localization_health()
        if localization_state == 'stop':
            self.clear_cte_marker()
            self.publish_stop(localization_reason)
            return

        if localization_state == 'degraded':
            # current_yaw is frozen at the last pose_callback value while pose
            # updates are sparse/late; refresh it from GPS course so steering
            # isn't aimed at a stale heading during the dropout.
            gps_yaw = self.get_fresh_gps_course_yaw()
            if gps_yaw is not None:
                self.current_yaw = gps_yaw

        closest_state = self.find_closest_path_state(self.current_x, self.current_y)
        if closest_state is None:
            self.clear_cte_marker()
            self.publish_stop('closest_state_not_found')
            return

        closest_index = closest_state['index']
        signed_cte = self.compute_signed_cte(closest_state)
        abs_cte = abs(signed_cte)
        path_heading = self.compute_path_heading(closest_state)
        path_curvature = self.compute_path_curvature(closest_state)
        preview_curvature = self.compute_preview_curvature(closest_index)
        preview_turn_angle = self.compute_preview_turn_angle(closest_index)
        sweep_turn_angle = self.compute_preview_turn_angle(
            closest_index,
            self.sweep_preview_points
        )
        effective_turn_angle = max(
            preview_turn_angle,
            self.sweep_turn_angle_gain * sweep_turn_angle
        )
        effective_curvature = max(
            path_curvature,
            self.curve_preview_gain * preview_curvature
        )
        self.telemetry_curvature = effective_curvature
        straight_mode = self.classify_straight_mode(
            effective_curvature,
            effective_turn_angle
        )
        self.current_straight_mode = straight_mode
        heading_error_path = self.normalize_angle(path_heading - self.current_yaw)
        control_speed = self.get_control_speed_reference(self.forward_speed)

        # κ_preview ที่ CBF-QP ต้องใช้ประกอบ κ̂ = max(κ_cmd, κ_preview)
        # ต้องคำนวณ/ส่งออกทุกเฟรมไม่ว่าจะเปิด use_cbf_qp หรือไม่ เพื่อให้ solver
        # ที่รันคู่กันมีอินพุตครบก่อนที่ผลลัพธ์จะถูกนำมาใช้จริง
        self.telemetry_curvature_preview = self.compute_cbf_preview_curvature(
            closest_index,
            self.get_lookahead_velocity_input()
        )

        # สมการข้อ 7: u* = argmin J s.t. ข้อ 2-4 — L_d* มาแทน heuristic lookahead ทั้งก้อน
        qp_v_star, qp_ld_star = self.get_cbf_qp_solution()
        self.note_cbf_qp_state(qp_ld_star is not None)
        if qp_ld_star is not None:
            # ไม่ clamp ซ้ำด้วย min/max_lookahead_distance ของที่นี่ เพราะขอบเขต
            # L_d,min <= L_d <= L_d,max เป็นข้อจำกัดข้อ 4 ที่ QP บังคับไว้แล้วในตอนแก้
            # (สองชุดนี้ตั้งค่าคนละค่ากันได้ ถ้ามาบีบซ้ำจะได้ L_d ที่ไม่ใช่คำตอบของ QP)
            # min/max_lookahead_distance จึงมีผลเฉพาะกับ heuristic fallback ด้านล่าง
            effective_lookahead = max(qp_ld_star, 0.1)
        else:
            effective_lookahead = self.compute_dynamic_lookahead(
                abs(heading_error_path),
                effective_curvature,
                control_speed,
                effective_turn_angle,
                abs_cte
            )
        lookahead_target = self.find_lookahead_target(closest_state, effective_lookahead)
        target_index = lookahead_target['index']
        path_speed_limit = self.get_path_speed_limit(target_index)
        # v_des ของ QP อิง closest_index (ตำแหน่งรถจริง) ไม่ใช่ target_index
        # เพราะ target_index เลื่อนตาม L_d ที่ QP เพิ่งเลือก -> ป้อนกลับเข้าตัวเอง
        self.telemetry_desired_speed = self.get_path_target_speed(closest_index)

        # RViz visualization
        self.publish_cte_line(closest_state)

        goal_x, goal_y = self.path_points_local[-1]
        dist_to_goal = math.hypot(goal_x - self.current_x, goal_y - self.current_y)

        done = Bool()
        # Keep reporting "not done" so the controller continues commanding motion while a path exists.
        done.data = False
        self.goal_done_pub.publish(done)

        remain = Float32()
        remain.data = float(self.compute_remaining_distance(closest_state))
        self.remaining_dist_pub.publish(remain)

        tx = lookahead_target['x']
        ty = lookahead_target['y']
        self.publish_target(tx, ty)
        self.publish_lookahead_point(tx, ty)
        self.publish_indices(closest_index, target_index)
        self.publish_controller_debug(
            signed_cte=signed_cte,
            path_heading=path_heading,
            heading_error=heading_error_path,
            lookahead_distance=effective_lookahead
        )

        dx = tx - self.current_x
        dy = ty - self.current_y

        target_heading = math.atan2(dy, dx)
        preview_heading = self.compute_preview_path_heading(target_index)
        lookahead_heading_error = self.normalize_angle(preview_heading - target_heading)
        heading_blend = self.clamp(
            self.lookahead_heading_blend
            + self.lookahead_heading_blend_gain * abs(heading_error_path),
            self.lookahead_heading_blend,
            self.lookahead_heading_blend_max
        )
        heading_blend = self.clamp(
            heading_blend + self.curve_heading_blend_gain * effective_curvature,
            self.lookahead_heading_blend,
            self.lookahead_heading_blend_max
        )
        heading_blend = self.clamp(
            heading_blend + self.turn_heading_blend_gain * effective_turn_angle,
            self.lookahead_heading_blend,
            self.turn_heading_blend_max
        )
        heading_blend = self.clamp(
            heading_blend + self.sweep_heading_blend_gain * sweep_turn_angle,
            self.lookahead_heading_blend,
            self.turn_heading_blend_max
        )
        heading_blend = self.clamp(
            heading_blend + self.cte_heading_blend_gain * min(abs_cte, 1.0),
            self.lookahead_heading_blend,
            self.turn_heading_blend_max
        )
        if straight_mode and abs_cte < self.straight_path_lock_cte_m:
            heading_blend *= self.straight_heading_blend_scale
            heading_reference = self.normalize_angle(
                path_heading + heading_blend * lookahead_heading_error
            )
        else:
            heading_reference = self.normalize_angle(
                target_heading + heading_blend * lookahead_heading_error
            )
        alpha = self.normalize_angle(heading_reference - self.current_yaw)

        Ld = max(math.hypot(dx, dy), 0.001)
        steer_pp = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
        # โปรไฟล์จาก CSV เป็นตัวกำหนดความเร็วตัวเดียว: ข้าม heuristic ทั้งก้อน
        # และไม่ให้ QP มา cap ทับ ตัวชะลอตามโค้ง/CTE/เป้าหมายจึงไม่ทำงานในโหมดนี้
        # ถ้าโปรไฟล์ยังไม่มา (path เพิ่งเปลี่ยน/ความยาวไม่ตรง) ค่อยถอยไปใช้ทางเดิม
        profile_speed_mps = (
            self.telemetry_desired_speed
            if self.speed_from_path_profile_only
            else None
        )
        if profile_speed_mps is not None:
            # h₂ ของ solver (เมื่อเปิด) ชะลอความเร็วจากไฟล์ตามแรงเหวี่ยงและโค้งข้างหน้า
            profile_speed_mps = min(max(float(profile_speed_mps), 0.0), self.get_cbf_qp_speed_cap())
            speed_target_mps = self.limit_path_speed_rise(profile_speed_mps)
        else:
            speed_target_mps = self.compute_speed(
                dist_to_goal,
                heading_error_path,
                signed_cte,
                effective_curvature,
                path_speed_limit,
                effective_turn_angle,
                straight_mode
            )
        # v* ยังถูก log ไว้ตามเดิม (QP ยังทำงานอยู่ เพียงแต่ไม่ได้คุมความเร็วแล้ว)
        # แต่ไม่เอามา cap ทับโปรไฟล์ ไม่งั้นความเร็วก็ไม่ได้มาจาก CSV ล้วน ๆ
        if qp_v_star is not None and profile_speed_mps is None:
            # 'cap'     : v* เป็นเพดานเพิ่มเติม — CBF ลดความเร็วได้ แต่ห้ามดันให้เร็วกว่า
            #             ที่ตัวชะลอเดิม (goal slowdown / CTE / path speed profile) ยอม
            # 'replace' : เชื่อ v* ตรง ๆ ตามสมการข้อ 7 (ยังโดน localization guard ด้านล่างอีกชั้น)
            if self.cbf_speed_mode == 'replace':
                speed_target_mps = qp_v_star
            else:
                speed_target_mps = min(speed_target_mps, qp_v_star)
        if localization_state == 'degraded':
            speed_target_mps = min(
                speed_target_mps * self.localization_degraded_speed_scale,
                self.localization_degraded_max_speed
            )
        control_speed = self.get_control_speed_reference(speed_target_mps)
        pure_pursuit = qp_ld_star is not None and self.cbf_qp_pure_pursuit_requested()
        self.note_pure_pursuit_state(pure_pursuit, effective_lookahead)
        if pure_pursuit:
            steer = self.pure_pursuit_steer_command(target_heading, Ld)
        else:
            steer_cte = self.compute_cte_correction(
                signed_cte,
                control_speed,
                effective_curvature,
                straight_mode
            )
            steer_raw = steer_pp + steer_cte
            if self.should_hold_zero_steer(
                signed_cte, heading_error_path, steer_raw, straight_mode
            ):
                steer_raw = 0.0
            steer_raw = self.apply_curve_steer_damping(
                steer_raw,
                signed_cte,
                heading_error_path,
                straight_mode
            )

            steer_limit_rad = self.get_zone_steer_limit(
                speed_target_mps,
                straight_mode=straight_mode,
                path_curvature=effective_curvature,
                upcoming_turn_angle=effective_turn_angle,
                abs_cte=abs_cte,
            )
            steer = self.smooth_steer_command(steer_raw, speed_target_mps, steer_limit_rad)

        # Fade steering to zero when nearly stationary — prevents oscillation at rest
        # caused by pose noise cycling the deadband-compensation on/off.
        _measured_v = self.get_fresh_pose_speed_mps()
        if _measured_v is not None and _measured_v < 0.35:
            steer *= _measured_v / 0.35

        speed_cmd = self.map_speed_target_to_cmd(speed_target_mps)
        speed_cmd = self.rate_limit_speed_cmd(speed_cmd)

        cmd = Twist()
        cmd.linear.x = float(speed_cmd)
        steer_cmd = -steer if self.invert_steer_sign else steer
        cmd.angular.z = round(float(steer_cmd), 3)
        self.last_cmd_speed = float(speed_target_mps)
        self.cmd_vel_pub.publish(cmd)
        measured_velocity = _measured_v if _measured_v is not None else control_speed

        # สมการข้อ 9: ตรวจคำสั่งที่เพิ่งส่งกับค่าที่วัดได้จริง
        # เทียบด้วย speed_target_mps (m/s) ไม่ใช่ speed_cmd ที่ผ่าน scale/offset ของบอร์ดแล้ว
        # เพื่อให้เทียบหน่วยเดียวกับ /speed_status ได้ตรง ๆ
        if self.run_command_check(float(speed_target_mps), float(steer_cmd)):
            return

        self.maybe_log_controller_row(
            target_x=tx,
            target_y=ty,
            target_distance=Ld,
            velocity=measured_velocity,
            yaw=self.current_yaw,
            cte=signed_cte,
            path_heading=path_heading,
            heading_error=heading_error_path,
            curvature_kappa=effective_curvature,
            steering_angle=steer_cmd,
            adaptive_ld=effective_lookahead,
            lookahead_distance=effective_lookahead,
            cmd_speed=speed_cmd,
            cmd_steer=steer_cmd,
            closest_waypoint_index=closest_index,
            target_waypoint_index=target_index,
            remaining_distance=remain.data,
            curvature_preview=self.telemetry_curvature_preview,
            qp_v_star=qp_v_star,
            qp_ld_star=qp_ld_star,
        )

    # =========================
    # RViz helpers
    # =========================
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_static_world_to_output_tf(self):
        if self.world_frame == self.output_frame:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.output_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)

    def publish_current_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(self.current_x)
        msg.pose.position.y = float(self.current_y)
        msg.pose.position.z = float(self.current_z if self.current_z is not None else 0.0)
        msg.pose.orientation = self.yaw_to_quaternion(self.current_yaw)
        self.current_pose_pub.publish(msg)

    def publish_vehicle_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.output_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = float(self.current_x)
        tf_msg.transform.translation.y = float(self.current_y)
        tf_msg.transform.translation.z = float(self.current_z if self.current_z is not None else 0.0)
        tf_msg.transform.rotation = self.yaw_to_quaternion(self.current_yaw)
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_vehicle_marker(self):
        """Publish golf-cart body (box + direction arrow) for RViz visualization."""
        now = self.get_clock().now().to_msg()
        frame = self.output_frame
        x = float(self.current_x)
        y = float(self.current_y)
        z = float(self.current_z if self.current_z is not None else 0.0)
        yaw = float(self.current_yaw)

        # Golf cart dimensions (m)
        body_length = 2.40
        body_width  = 1.20
        body_height = 0.90
        # Body center is wheelbase/2 ahead of rear-axle origin
        half_wb = self.wheelbase * 0.5
        cx = x + half_wb * math.cos(yaw)
        cy = y + half_wb * math.sin(yaw)
        cz = z + body_height * 0.5

        q = self.yaw_to_quaternion(yaw)
        arr = MarkerArray()

        # Body box
        body = Marker()
        body.header.stamp = now
        body.header.frame_id = frame
        body.ns = 'golf_cart'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = cx
        body.pose.position.y = cy
        body.pose.position.z = cz
        body.pose.orientation = q
        body.scale.x = body_length
        body.scale.y = body_width
        body.scale.z = body_height
        body.color.r = 0.20
        body.color.g = 0.70
        body.color.b = 0.20
        body.color.a = 0.75
        arr.markers.append(body)

        # Heading arrow (yellow) on top of body
        arrow = Marker()
        arrow.header.stamp = now
        arrow.header.frame_id = frame
        arrow.ns = 'golf_cart'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = cx
        arrow.pose.position.y = cy
        arrow.pose.position.z = cz + body_height * 0.5 + 0.10
        arrow.pose.orientation = q
        arrow.scale.x = body_length * 0.55
        arrow.scale.y = 0.18
        arrow.scale.z = 0.18
        arrow.color.r = 1.0
        arrow.color.g = 0.85
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        arr.markers.append(arrow)

        self.vehicle_marker_pub.publish(arr)

    def update_vehicle_trace_if_ready(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.last_trace_x is not None and self.last_trace_y is not None:
            d = math.hypot(self.current_x - self.last_trace_x, self.current_y - self.last_trace_y)
            if d < self.trace_min_dist:
                return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.output_frame
        ps.pose.position.x = float(self.current_x)
        ps.pose.position.y = float(self.current_y)
        ps.pose.position.z = float(self.current_z if self.current_z is not None else 0.0)
        ps.pose.orientation = self.yaw_to_quaternion(self.current_yaw)

        self.vehicle_trace.header.stamp = ps.header.stamp
        self.vehicle_trace.header.frame_id = self.output_frame
        self.vehicle_trace.poses.append(ps)
        self.vehicle_trace_pub.publish(self.vehicle_trace)

        self.last_trace_x = self.current_x
        self.last_trace_y = self.current_y

    def publish_cte_line(self, closest_state):
        if closest_state is None:
            return

        px = closest_state['proj_x']
        py = closest_state['proj_y']

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.output_frame
        marker.ns = 'cte_line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        p1 = Point()
        p1.x = float(self.current_x)
        p1.y = float(self.current_y)
        p1.z = 0.1

        p2 = Point()
        p2.x = float(px)
        p2.y = float(py)
        p2.z = 0.1

        marker.points = [p1, p2]
        self.cte_marker_pub.publish(marker)

    # =========================
    # Control helpers
    # =========================
    def publish_stop(self, reason=None):
        if reason != self.last_stop_reason and reason is not None:
            self.get_logger().warn(f'Publishing stop: {reason}')
        self.last_stop_reason = reason
        self.prev_steer_cmd = 0.0
        self.prev_steer_filtered = 0.0
        self.deadband_comp_active = False
        self.current_speed_zone = 'red'
        self.last_cmd_speed = 0.0
        self.prev_speed_cmd_out = 0.0
        self.path_speed_stall_sec = 0.0
        self.reset_cte_controller()
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def publish_lookahead_point(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.lookahead_point_pub.publish(msg)

    def publish_controller_debug(
        self,
        signed_cte,
        path_heading,
        heading_error,
        lookahead_distance
    ):
        if self.current_yaw is not None:
            self.controller_yaw_pub.publish(Float64(data=float(self.current_yaw)))
        self.controller_cte_pub.publish(Float64(data=float(signed_cte)))
        self.path_heading_pub.publish(Float64(data=float(path_heading)))
        self.heading_error_pub.publish(Float64(data=float(heading_error)))
        self.lookahead_distance_pub.publish(Float64(data=float(lookahead_distance)))
        self.adaptive_ld_debug_pub.publish(Float64(data=float(lookahead_distance)))
        self.telemetry_lookahead = float(lookahead_distance)

    def publish_indices(self, closest_idx, target_idx):
        c = Int32()
        c.data = int(closest_idx)
        self.closest_index_pub.publish(c)

        t = Int32()
        t.data = int(target_idx)
        self.target_index_pub.publish(t)

    def find_closest_path_state(self, x, y):
        if len(self.path_points_local) < 2:
            return None

        best_state = None
        best_dist = float('inf')

        for i in range(len(self.path_points_local) - 1):
            x1, y1 = self.path_points_local[i]
            x2, y2 = self.path_points_local[i + 1]
            seg_x = x2 - x1
            seg_y = y2 - y1
            seg_len_sq = seg_x * seg_x + seg_y * seg_y

            if seg_len_sq < 1e-9:
                proj_ratio = 0.0
                proj_x = x1
                proj_y = y1
            else:
                proj_ratio = (
                    ((x - x1) * seg_x + (y - y1) * seg_y) / seg_len_sq
                )
                proj_ratio = self.clamp(proj_ratio, 0.0, 1.0)
                proj_x = x1 + proj_ratio * seg_x
                proj_y = y1 + proj_ratio * seg_y

            dist = math.hypot(proj_x - x, proj_y - y)
            if dist < best_dist:
                best_dist = dist
                best_state = {
                    'index': i,
                    'proj_ratio': proj_ratio,
                    'proj_x': proj_x,
                    'proj_y': proj_y,
                    'distance': dist,
                }

        return best_state

    def find_lookahead_target(self, closest_state, lookahead_distance):
        start_index = closest_state['index']
        remaining = max(0.0, lookahead_distance)
        segment_count = max(len(self.path_points_local) - 1, 0)

        if segment_count <= 0:
            last_x, last_y = self.path_points_local[-1]
            return {
                'x': last_x,
                'y': last_y,
                'index': len(self.path_points_local) - 1,
            }

        if self.is_path_loop_closed():
            loop_length = self.compute_path_total_length()
            if loop_length > 1e-6:
                remaining = math.fmod(remaining, loop_length)

        for offset in range(segment_count):
            i = (start_index + offset) % segment_count
            x1, y1 = self.path_points_local[i]
            x2, y2 = self.path_points_local[i + 1]
            seg_len = math.hypot(x2 - x1, y2 - y1)
            if seg_len < 1e-9:
                continue

            start_ratio = closest_state['proj_ratio'] if i == start_index else 0.0
            usable_len = max(0.0, (1.0 - start_ratio) * seg_len)

            if remaining <= usable_len:
                ratio = start_ratio + (remaining / seg_len)
                return {
                    'x': x1 + ratio * (x2 - x1),
                    'y': y1 + ratio * (y2 - y1),
                    'index': i + 1,
                }

            remaining -= usable_len

        if self.is_path_loop_closed():
            first_x, first_y = self.path_points_local[0]
            return {
                'x': first_x,
                'y': first_y,
                'index': 0,
            }

        last_x, last_y = self.path_points_local[-1]
        return {
            'x': last_x,
            'y': last_y,
            'index': len(self.path_points_local) - 1,
        }

    def should_close_path_loop(self):
        if len(self.path_points_local) < 2:
            return False
        return not self.is_path_loop_closed()

    def is_path_loop_closed(self):
        if len(self.path_points_local) < 3:
            return False
        first_x, first_y = self.path_points_local[0]
        last_x, last_y = self.path_points_local[-1]
        return math.hypot(last_x - first_x, last_y - first_y) <= 1e-6

    def compute_path_total_length(self):
        if len(self.path_points_local) < 2:
            return 0.0

        total = 0.0
        for i in range(len(self.path_points_local) - 1):
            x1, y1 = self.path_points_local[i]
            x2, y2 = self.path_points_local[i + 1]
            total += math.hypot(x2 - x1, y2 - y1)
        return total

    def compute_remaining_distance(self, closest_state):
        total = math.hypot(
            closest_state['proj_x'] - self.current_x,
            closest_state['proj_y'] - self.current_y
        )

        start_index = closest_state['index']
        x1, y1 = self.path_points_local[start_index]
        x2, y2 = self.path_points_local[start_index + 1]
        first_seg_len = math.hypot(x2 - x1, y2 - y1)
        total += max(0.0, (1.0 - closest_state['proj_ratio']) * first_seg_len)

        for i in range(start_index + 1, len(self.path_points_local) - 1):
            x1, y1 = self.path_points_local[i]
            x2, y2 = self.path_points_local[i + 1]
            total += math.hypot(x2 - x1, y2 - y1)

        return total

    def extract_path_pose_yaw(self, pose_stamped):
        q = pose_stamped.pose.orientation
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq < 1e-12:
            return None
        return self.quaternion_to_yaw(q)

    def compute_path_segment_speeds(self):
        if len(self.path_states_local) < 2:
            return []

        segment_speeds = []
        last_valid_speed = None
        for index in range(len(self.path_states_local) - 1):
            p0 = self.path_states_local[index]
            p1 = self.path_states_local[index + 1]
            stamp0 = p0['stamp_sec']
            stamp1 = p1['stamp_sec']

            if stamp0 is None or stamp1 is None or stamp1 <= stamp0 + 1e-6:
                segment_speeds.append(last_valid_speed)
                continue

            dx = p1['x'] - p0['x']
            dy = p1['y'] - p0['y']
            dz = p1['z'] - p0['z']
            dt = stamp1 - stamp0
            speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            segment_speeds.append(speed)
            last_valid_speed = speed

        return segment_speeds

    def get_path_yaw_near_index(self, index):
        if not self.use_path_orientation or not self.path_states_local:
            return None

        index = max(0, min(index, len(self.path_states_local) - 1))
        max_offset = max(self.path_heading_forward_points, self.path_heading_back_points, 1)
        for offset in range(0, max_offset + 1):
            left_index = index - offset
            right_index = index + offset
            if left_index >= 0:
                yaw = self.path_states_local[left_index]['yaw']
                if yaw is not None:
                    return yaw
            if right_index < len(self.path_states_local):
                yaw = self.path_states_local[right_index]['yaw']
                if yaw is not None:
                    return yaw
        return None

    def get_path_speed_limit(self, center_index):
        if not self.use_path_speed_profile or not self.path_segment_speed_mps:
            return None

        start_index = max(0, center_index - 1)
        end_index = min(
            len(self.path_segment_speed_mps) - 1,
            center_index + self.path_speed_preview_points
        )
        speed_candidates = [
            self.path_segment_speed_mps[index]
            for index in range(start_index, end_index + 1)
            if self.path_segment_speed_mps[index] is not None
        ]
        if not speed_candidates:
            return None

        speed_candidates.sort()
        quantile_index = min(
            len(speed_candidates) - 1,
            max(
                0,
                int(round((len(speed_candidates) - 1) * self.path_speed_limit_quantile))
            )
        )
        speed_limit = speed_candidates[quantile_index] * max(self.path_speed_scale, 0.0)
        return max(speed_limit, self.path_speed_min_limit)

    def compute_preview_turn_angle(self, start_index, preview_points=None):
        if len(self.path_points_local) < 2:
            return 0.0

        if preview_points is None:
            preview_points = self.turn_angle_preview_points
        start_heading = self.compute_preview_path_heading(start_index)
        end_index = min(
            len(self.path_points_local) - 1,
            start_index + max(preview_points, 1)
        )
        end_heading = self.compute_preview_path_heading(end_index)
        return abs(self.normalize_angle(end_heading - start_heading))

    def compute_path_heading(self, closest_state):
        if len(self.path_points_local) < 2:
            return self.current_yaw if self.current_yaw is not None else 0.0

        return self.compute_preview_path_heading(closest_state['index'])

    def compute_preview_path_heading(self, center_index):
        if len(self.path_points_local) < 2:
            return self.current_yaw if self.current_yaw is not None else 0.0

        path_yaw = self.get_path_yaw_near_index(center_index)
        if path_yaw is not None:
            return path_yaw

        start_index = max(0, center_index - self.path_heading_back_points)
        end_index = min(
            len(self.path_points_local) - 1,
            center_index + self.path_heading_forward_points
        )
        if end_index <= start_index:
            end_index = min(len(self.path_points_local) - 1, start_index + 1)

        x1, y1 = self.path_points_local[start_index]
        x2, y2 = self.path_points_local[end_index]
        return math.atan2(y2 - y1, x2 - x1)

    def compute_path_curvature(self, closest_state):
        n = len(self.path_points_local)
        if n < 3:
            return 0.0

        i = closest_state['index']
        # Use a wide span (±3 points ≈ ±1.5 m at 0.5 m spacing) so GPS noise
        # in individual waypoints is averaged out instead of amplified.
        half = 3
        ia = max(0, i - half)
        ib = min(n - 1, i + half)

        if ib <= ia:
            return 0.0

        if self.use_path_orientation:
            yaw_a = self.get_path_yaw_near_index(ia)
            yaw_b = self.get_path_yaw_near_index(ib)
            if yaw_a is not None and yaw_b is not None:
                xa, ya = self.path_points_local[ia]
                xb, yb = self.path_points_local[ib]
                ds = math.hypot(xb - xa, yb - ya)
                if ds >= 1e-3:
                    return abs(self.normalize_angle(yaw_b - yaw_a)) / ds

        # Fallback: compute heading of first half vs second half of the span
        im = (ia + ib) // 2
        xa, ya = self.path_points_local[ia]
        xm, ym = self.path_points_local[im]
        xb, yb = self.path_points_local[ib]

        h1 = math.atan2(ym - ya, xm - xa)
        h2 = math.atan2(yb - ym, xb - xm)
        ds = 0.5 * (math.hypot(xm - xa, ym - ya) + math.hypot(xb - xm, yb - ym))
        if ds < 1e-3:
            return 0.0

        return abs(self.normalize_angle(h2 - h1)) / ds

    def compute_preview_curvature(self, start_index):
        if len(self.path_points_local) < 3:
            return 0.0

        end_index = min(
            len(self.path_points_local) - 2,
            start_index + max(self.curve_preview_points, 1)
        )
        curvatures = [
            self.compute_path_curvature({'index': idx})
            for idx in range(start_index, end_index + 1)
        ]
        if not curvatures:
            return 0.0
        # 75th percentile: captures real corners ahead while ignoring noise spikes
        curvatures.sort()
        p75_idx = int(0.75 * len(curvatures))
        return curvatures[min(p75_idx, len(curvatures) - 1)]

    def compute_signed_cte(self, closest_state):
        if len(self.path_points_local) < 2:
            return 0.0

        start_index = closest_state['index']
        end_index = start_index + 1

        x1, y1 = self.path_points_local[start_index]
        x2, y2 = self.path_points_local[end_index]

        seg_x = x2 - x1
        seg_y = y2 - y1
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1e-6:
            return 0.0

        # positive = vehicle is to the right of path direction
        cross = seg_x * (self.current_y - y1) - seg_y * (self.current_x - x1)
        return -cross / seg_len

    def compute_dynamic_lookahead(
        self,
        abs_heading_error,
        path_curvature,
        control_speed,
        upcoming_turn_angle=0.0,
        abs_cte=0.0
    ):
        # Straights -> long LD (stability); curves -> short LD (tight tracking),
        # driven directly by control_speed and curvature/turn-angle ahead.
        speed_component = self.speed_lookahead_gain * max(control_speed, 0.0)
        base_lookahead = self.min_lookahead_distance + speed_component

        curvature_factor = 1.0 / (1.0 + self.curve_lookahead_gain * max(path_curvature, 0.0))
        heading_factor = 1.0 / (1.0 + self.lookahead_turn_gain * abs(abs_heading_error))

        turn_ratio = self.compute_turn_anticipation_ratio(path_curvature, upcoming_turn_angle)
        anticipation_factor = self.clamp(
            1.0 - self.turn_anticipation_gain * turn_ratio,
            0.0,
            1.0
        )

        lookahead = base_lookahead * curvature_factor * heading_factor * anticipation_factor

        hard_turn = self.is_hard_turn(path_curvature, upcoming_turn_angle, abs_cte)
        if self.current_straight_mode and not hard_turn:
            fade_ratio = self.clamp(
                1.0 - abs_cte / self.straight_bonus_fade_cte_m,
                0.0,
                1.0
            )
            lookahead += self.straight_lookahead_bonus * fade_ratio
        else:
            bonus_scale = self.hard_turn_lookahead_bonus_scale if hard_turn else 1.0
            lookahead += self.curve_lookahead_bonus * turn_ratio * bonus_scale

        lower_bound = self.hard_turn_lookahead_distance if hard_turn else self.min_lookahead_distance
        return self.clamp(lookahead, lower_bound, self.max_lookahead_distance)

    def publish_lookahead_telemetry(self):
        curvature_msg = Float64()
        curvature_msg.data = float(max(self.telemetry_curvature, 0.0))
        self.curvature_pub.publish(curvature_msg)

        velocity_msg = Float64()
        velocity_msg.data = float(self.get_lookahead_velocity_input())
        self.velocity_pub.publish(velocity_msg)

        lookahead_msg = Float64()
        lookahead_msg.data = float(self.telemetry_lookahead)
        self.lookahead_pub.publish(lookahead_msg)

        preview_msg = Float64()
        preview_msg.data = float(max(self.telemetry_curvature_preview, 0.0))
        self.curvature_preview_pub.publish(preview_msg)

        if self.telemetry_desired_speed is not None:
            desired_speed_msg = Float64()
            desired_speed_msg.data = float(max(self.telemetry_desired_speed, 0.0))
            self.desired_speed_pub.publish(desired_speed_msg)

    # =========================
    # CBF-QP interface
    # =========================
    def cbf_qp_ld_callback(self, msg: Float64):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.qp_ld_star = value
        self.last_qp_ld_ns = self.get_clock().now().nanoseconds

    def cbf_qp_speed_callback(self, msg: Float64):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.qp_v_star = max(value, 0.0)
        self.last_qp_speed_ns = self.get_clock().now().nanoseconds

    def cbf_qp_speed_cap_callback(self, msg: Float64):
        value = float(msg.data)
        self.qp_speed_cap = max(value, 0.0) if math.isfinite(value) else math.inf
        self.last_qp_speed_cap_ns = self.get_clock().now().nanoseconds

    def get_cbf_qp_speed_cap(self):
        """เพดานความเร็วจาก h₂ ถ้ายังสด ไม่งั้น ∞ (solver หยุด/ช้า = ใช้ความเร็วจากไฟล์ตามเดิม)"""
        if not self.use_cbf_qp:
            return math.inf
        age_ns = self.get_clock().now().nanoseconds - self.last_qp_speed_cap_ns
        if age_ns > self.cbf_qp_timeout_sec * 1e9:
            return math.inf
        return self.qp_speed_cap

    def cbf_qp_pure_pursuit_callback(self, msg: Bool):
        self.qp_pure_pursuit = bool(msg.data)
        self.last_qp_pure_pursuit_ns = self.get_clock().now().nanoseconds

    def cbf_qp_pure_pursuit_requested(self):
        """solver อยู่โหมด pure_pursuit_only และยังส่งสถานะนี้มาสด ๆ"""
        if not self.use_cbf_qp or not self.qp_pure_pursuit:
            return False
        age_ns = self.get_clock().now().nanoseconds - self.last_qp_pure_pursuit_ns
        return age_ns <= self.cbf_qp_timeout_sec * 1e9

    def note_pure_pursuit_state(self, active, lookahead):
        if active == self.pure_pursuit_active:
            return
        self.pure_pursuit_active = active
        if active:
            self.get_logger().warn(
                f'Pure Pursuit ล้วน: δ = atan(2L·sinα / L_d), L_d = {lookahead:.2f} m '
                '(ไม่มี heading blend, CTE correction, filter, deadband comp)'
            )
        else:
            self.get_logger().info('ออกจาก Pure Pursuit ล้วน -> กลับไปใช้ steering เดิมของ pagard')

    def pure_pursuit_steer_command(self, target_heading, chord):
        """δ = atan(2L·sinα / L_d) ของจุดเป้าหมายบน path ที่ระยะ L_d คงที่จาก solver

        α = มุมจากหัวรถไปจุดเป้าหมาย, L_d ในสูตร = ระยะเส้นตรงถึงจุดนั้น
        เหลือไว้แค่เพดานมุมเลี้ยวของรถ (max_steer_rad) กับอัตราเปลี่ยนสูงสุด
        (max_steer_rate_rad) กันมอเตอร์พวงมาลัยกระชาก
        """
        alpha = self.normalize_angle(target_heading - self.current_yaw)
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), chord)
        steer = self.clamp(steer, -self.max_steer_rad, self.max_steer_rad)
        max_delta = self.max_steer_rate_rad * self.control_period
        steer = self.clamp(steer, self.prev_steer_cmd - max_delta, self.prev_steer_cmd + max_delta)
        # ให้ filter ของโหมดเดิมเริ่มจากค่าที่ส่งจริง สลับกลับแล้วพวงมาลัยไม่กระตุก
        self.prev_steer_filtered = steer
        self.prev_steer_cmd = steer
        self.deadband_comp_active = False
        return steer

    def get_cbf_qp_solution(self):
        """คืน (v*, L_d*) ถ้า u* ยังสด — ไม่งั้นคืน (None, None) ให้ถอยไปใช้ heuristic

        ต้องสดพร้อมกันทั้งคู่ เพราะ CBF-QP เลือก v กับ L_d เป็นคำตอบชุดเดียวกัน
        การหยิบมาใช้แค่ตัวเดียวจะทำลายความสอดคล้องของข้อจำกัดที่ h₂ ผูก v กับ L_d ไว้
        """
        if not self.use_cbf_qp:
            return None, None
        if self.qp_ld_star is None or self.qp_v_star is None:
            return None, None

        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = self.cbf_qp_timeout_sec * 1e9
        if (now_ns - self.last_qp_ld_ns) > timeout_ns:
            return None, None
        if (now_ns - self.last_qp_speed_ns) > timeout_ns:
            return None, None

        return self.qp_v_star, self.qp_ld_star

    def note_cbf_qp_state(self, active):
        if active == self.qp_active:
            return
        self.qp_active = active
        if active:
            self.get_logger().info('CBF-QP: รับ u* = [v*, L_d*] แล้ว -> ใช้คำตอบจาก QP')
        else:
            self.get_logger().warn(
                'CBF-QP: ไม่ได้ u* ใหม่ตามกำหนด -> ถอยไปใช้ heuristic lookahead/speed'
            )

    def compute_cbf_preview_curvature(self, start_index, velocity):
        """κ_preview = ความโค้งสูงสุดของ path ในระยะ v/α₂ ข้างหน้า (สมการหมวด D)

        ใช้ระยะทางจริงบน path ไม่ใช่จำนวนจุดคงที่ เพราะระยะที่ต้องมองล่วงหน้า
        ขึ้นกับความเร็ว: ยิ่งเร็ว ยิ่งต้องเห็นโค้งเร็วขึ้นเพื่อให้ h₂ เริ่มกดความเร็วทัน
        """
        n = len(self.path_points_local)
        if n < 3:
            return 0.0

        preview_distance = self.clamp(
            max(velocity, 0.0) / self.cbf_preview_alpha2,
            self.cbf_preview_min_distance,
            self.cbf_preview_max_distance
        )

        max_curvature = 0.0
        travelled = 0.0
        idx = max(0, min(start_index, n - 2))
        while idx <= n - 2 and travelled <= preview_distance:
            max_curvature = max(max_curvature, self.compute_path_curvature({'index': idx}))
            x1, y1 = self.path_points_local[idx]
            x2, y2 = self.path_points_local[idx + 1]
            travelled += math.hypot(x2 - x1, y2 - y1)
            idx += 1

        return max_curvature

    # =========================
    # Command check (สมการข้อ 9)
    # =========================
    def steering_angle_callback(self, msg: Float64):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.steering_angle_value = value * self.command_check_steer_feedback_sign
        self.steering_angle_received = True
        self.last_steering_angle_ns = self.get_clock().now().nanoseconds

    def run_command_check(self, speed_cmd_mps, steer_cmd_rad):
        """Δv = v_cmd − /speed_status, Δδ = δ_cmd − /steering_angle

        ชั้นตรวจสอบที่สองต่อจาก CBF: CBF การันตีว่า "คำสั่ง" ปลอดภัย ส่วนตรงนี้ตรวจว่า
        รถ "ทำตามคำสั่ง" จริงหรือไม่ ถ้าไม่ตามต่อเนื่องนานพอ = ฮาร์ดแวร์มีปัญหา -> หยุดฉุกเฉิน
        """
        if not self.command_check_enable:
            return False

        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = self.command_check_feedback_timeout_sec * 1e9

        speed_fresh = (
            self.speed_status_received
            and (now_ns - self.last_speed_status_ns) <= timeout_ns
        )
        if speed_fresh and speed_cmd_mps >= self.command_check_min_speed_cmd:
            measured = abs(self.speed_status_value)
            speed_error = speed_cmd_mps - measured
            self.speed_error_pub.publish(Float64(data=float(speed_error)))

            # เทียบกับ "ความเร็วที่รถควรทำได้ ณ ตอนนี้" ไม่ใช่เป้าปลายทาง:
            # ออกตัวจากหยุดนิ่งไปหา 3 m/s ต้องใช้เวลาหลายวินาทีตามอัตราเร่งจริงของรถ
            # ถ้าเทียบกับเป้าตรง ๆ ทุกครั้งที่เริ่มวิ่งจะเกิน max_speed_error ทันที
            # แล้วทริป safety stop ทั้งที่รถทำงานปกติ
            # ref ไล่ขึ้นจากความเร็วที่วัดได้ด้วยอัตราเร่งสูงสุดของรถ ถ้ารถไม่ขยับจริง
            # ref จะแซงไปเรื่อย ๆ จนเกิน tolerance เอง การจับฮาร์ดแวร์เสียจึงยังอยู่ครบ
            if self.command_check_speed_ref is None:
                self.command_check_speed_ref = measured
            self.command_check_speed_ref = min(
                speed_cmd_mps,
                max(
                    measured,
                    self.command_check_speed_ref
                    + self.max_speed_accel_mps2 * self.control_period,
                ),
            )
            if (self.command_check_speed_ref - measured) > self.command_check_max_speed_error:
                self.command_check_speed_count += 1
            else:
                self.command_check_speed_count = 0
        else:
            self.command_check_speed_count = 0
            self.command_check_speed_ref = None

        steer_fresh = (
            self.steering_angle_received
            and (now_ns - self.last_steering_angle_ns) <= timeout_ns
        )
        if steer_fresh:
            steer_error = steer_cmd_rad - self.steering_angle_value
            self.steer_error_pub.publish(Float64(data=float(steer_error)))
            if abs(steer_error) > self.command_check_max_steer_error_rad:
                self.command_check_steer_count += 1
            else:
                self.command_check_steer_count = 0
        else:
            self.command_check_steer_count = 0

        if self.command_check_speed_count >= self.command_check_confirm_cycles:
            reachable = (
                self.command_check_speed_ref
                if self.command_check_speed_ref is not None
                else speed_cmd_mps
            )
            self.trigger_safety_stop(
                f'command_check_speed_mismatch: cmd={speed_cmd_mps:.2f} m/s '
                f'(ควรทำได้แล้ว {reachable:.2f} m/s) vs '
                f'{self.speed_status_topic}={self.speed_status_value:.2f} m/s '
                f'นานเกิน {self.command_check_confirm_cycles} รอบ'
            )
            return True

        if self.command_check_steer_count >= self.command_check_confirm_cycles:
            self.trigger_safety_stop(
                f'command_check_steer_mismatch: cmd={math.degrees(steer_cmd_rad):.1f} deg vs '
                f'{self.steering_angle_topic}={math.degrees(self.steering_angle_value):.1f} deg '
                f'นานเกิน {self.command_check_confirm_cycles} รอบ'
            )
            return True

        return False

    def get_lookahead_velocity_input(self):
        """ค่าที่ publish ออก /velocity — ความเร็วรถจริง ถ้าไม่มีแหล่งไหนสดค่อยใช้ค่าที่สั่ง"""
        measured_speed, _source = self.get_measured_speed_mps()
        if measured_speed is not None:
            return measured_speed
        return max(abs(self.last_cmd_speed), 0.0)

    def compute_turn_anticipation_ratio(self, path_curvature, upcoming_turn_angle):
        turn_ratio = self.clamp(
            max(upcoming_turn_angle, 0.0) / max(self.hard_turn_preview_rad, 1e-6),
            0.0,
            1.0
        )
        curvature_ratio = self.clamp(
            max(path_curvature, 0.0) / max(self.hard_turn_curvature, 1e-6),
            0.0,
            1.0
        )
        return max(turn_ratio, curvature_ratio)

    def classify_straight_mode(self, effective_curvature, effective_turn_angle=0.0):
        enter_threshold = max(
            self.straight_curvature_threshold - self.straight_mode_hysteresis,
            0.0
        )
        exit_threshold = self.straight_curvature_threshold + self.straight_mode_hysteresis
        enter_turn_threshold = max(
            self.straight_preview_turn_rad - self.straight_preview_turn_hysteresis_rad,
            0.0
        )
        exit_turn_threshold = (
            self.straight_preview_turn_rad + self.straight_preview_turn_hysteresis_rad
        )
        if self.current_straight_mode:
            return (
                effective_curvature < exit_threshold
                and effective_turn_angle < exit_turn_threshold
            )
        return (
            effective_curvature < enter_threshold
            and effective_turn_angle < enter_turn_threshold
        )

    def compute_speed(
        self,
        dist_to_goal,
        heading_error_path,
        signed_cte,
        path_curvature,
        path_speed_limit=None,
        upcoming_turn_angle=0.0,
        straight_mode=False
    ):
        if self.binary_speed_mode:
            _ = (
                dist_to_goal,
                heading_error_path,
                signed_cte,
                path_curvature,
                path_speed_limit,
                upcoming_turn_angle,
                straight_mode,
            )
            return max(self.forward_speed, 0.0)

        _ = dist_to_goal
        speed_cmd = self.forward_speed
        in_goal_slowdown_zone = False
        turning_active = (
            (not straight_mode)
            or abs(heading_error_path) > 0.5 * self.turn_slowdown_heading_rad
            or max(upcoming_turn_angle, 0.0) > 0.5 * self.hard_turn_preview_rad
        )
        curve_motion_floor = self.stop_speed

        if turning_active:
            curve_motion_floor = max(self.turn_speed, self.curve_motion_min_speed)

        if abs(heading_error_path) > self.turn_slowdown_heading_rad:
            speed_cmd = min(speed_cmd, self.turn_speed)

        if abs(signed_cte) > self.lookahead_distance * 0.35:
            speed_cmd = min(speed_cmd, self.turn_speed)

        if straight_mode:
            if abs(signed_cte) > self.straight_cte_turn_speed_m:
                speed_cmd = min(speed_cmd, self.turn_speed)
            elif abs(signed_cte) > self.straight_cte_slowdown_m:
                speed_cmd = min(speed_cmd, self.slow_speed)
        else:
            if abs(signed_cte) > self.curve_cte_turn_speed_m:
                speed_cmd = min(speed_cmd, self.turn_speed)
            elif abs(signed_cte) > self.curve_cte_slowdown_m:
                speed_cmd = min(speed_cmd, self.slow_speed)

        curve_speed_limit = self.forward_speed / (1.0 + self.curve_speed_gain * path_curvature)
        speed_cmd = min(speed_cmd, max(self.turn_speed, curve_speed_limit))

        turn_speed_limit = self.forward_speed / (
            1.0 + self.turn_angle_speed_gain * max(upcoming_turn_angle, 0.0)
        )
        speed_cmd = min(speed_cmd, max(self.turn_speed, turn_speed_limit))

        curve_ratio = self.clamp(
            max(path_curvature, 0.0) / max(self.hard_turn_curvature, 1e-6),
            0.0,
            1.0
        )
        preview_ratio = self.clamp(
            max(upcoming_turn_angle, 0.0) / max(self.hard_turn_preview_rad, 1e-6),
            0.0,
            1.0
        )
        caution_ratio = max(curve_ratio, preview_ratio)
        if caution_ratio > 0.45:
            speed_cmd = min(speed_cmd, self.slow_speed)
        if caution_ratio > 0.75:
            speed_cmd = min(speed_cmd, self.turn_speed)

        if self.is_hard_turn(path_curvature, upcoming_turn_angle):
            speed_cmd = min(speed_cmd, self.hard_turn_speed)

        if path_speed_limit is not None:
            speed_cmd = min(
                speed_cmd,
                max(curve_motion_floor, self.path_speed_min_limit, path_speed_limit)
            )

        if turning_active and not in_goal_slowdown_zone:
            speed_cmd = max(speed_cmd, curve_motion_floor)

        return speed_cmd

    def is_hard_turn(self, path_curvature, upcoming_turn_angle, abs_cte=0.0):
        _ = abs_cte
        return (
            max(path_curvature, 0.0) >= self.hard_turn_curvature
            or max(upcoming_turn_angle, 0.0) >= self.hard_turn_preview_rad
        )

    def rate_limit_speed_cmd(self, speed_cmd):
        if self.speed_profile_passthrough:
            # โปรไฟล์ใน CSV เร่ง/เบรกอยู่ในช่วง +0.867 / −0.767 m/s² ซึ่งลาดอยู่แล้ว
            # ตัวจำกัดอัตราจึงถูกปิดไปด้วยตามที่สั่ง — คำสั่งเปลี่ยนตามโปรไฟล์ทันที
            self.prev_speed_cmd_out = float(speed_cmd)
            return float(speed_cmd)

        speed_cmd = max(float(speed_cmd), 0.0)
        delta = speed_cmd - self.prev_speed_cmd_out
        if delta >= 0.0:
            max_delta = self.max_speed_accel_mps2 * self.control_period
        else:
            max_delta = self.max_speed_decel_mps2 * self.control_period
        delta = self.clamp(delta, -max_delta, max_delta)
        self.prev_speed_cmd_out += delta
        return self.prev_speed_cmd_out

    def map_speed_target_to_cmd(self, target_speed_mps):
        target_speed_mps = max(float(target_speed_mps), 0.0)
        if target_speed_mps <= 1e-6:
            return 0.0

        if self.speed_profile_passthrough:
            # ค่าจากโปรไฟล์ผ่านตรง ๆ: ไม่มี feedback correction และไม่มี
            # speed_command_max มาตัดยอด เหลือแต่ scale/offset ซึ่งเป็นการแปลงหน่วย
            # ไปเป็นคำสั่งบน cmd_vel ไม่ใช่ตัวคุมความเร็ว
            return max(
                target_speed_mps * self.speed_command_scale
                + self.speed_command_offset,
                0.0
            )

        if self.binary_speed_mode:
            return self.clamp(
                self.forward_speed * self.speed_command_scale
                + self.speed_command_offset,
                0.0,
                self.speed_command_max
            )

        speed_cmd = (
            target_speed_mps * self.speed_command_scale
            + self.speed_command_offset
        )
        measured_speed_mps = self.get_fresh_pose_speed_mps()
        if measured_speed_mps is not None:
            speed_error_mps = target_speed_mps - measured_speed_mps
            feedback_kp = (
                self.speed_feedback_up_kp
                if speed_error_mps >= 0.0
                else self.speed_feedback_down_kp
            )
            speed_cmd += self.clamp(
                feedback_kp * speed_error_mps,
                -self.speed_feedback_max_delta,
                self.speed_feedback_max_delta,
            )
        return self.clamp(speed_cmd, 0.0, self.speed_command_max)

    def speed_status_callback(self, msg: Float64):
        self.speed_status_value = float(msg.data)
        self.speed_status_received = True
        self.last_speed_status_ns = self.get_clock().now().nanoseconds
        if abs(self.speed_status_value) > 1e-6:
            self.speed_status_zero_since_ns = 0
            if not self.speed_status_healthy:
                self.speed_status_healthy = True
                self.get_logger().info(
                    f'{self.speed_status_topic} กลับมามีค่าแล้ว -> ใช้เป็นแหล่งความเร็วหลักอีกครั้ง')

    def odom_callback(self, msg: Odometry):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self.odom_speed_value = math.hypot(vx, vy)
        self.odom_speed_received = True
        self.last_odom_speed_ns = self.get_clock().now().nanoseconds

    def gps_velocity_callback(self, msg: TwistWithCovarianceStamped):
        # ENU: linear.x = east, linear.y = north (u-blox convention)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        speed = math.hypot(vx, vy)
        self.gps_course_speed_mps = speed
        self.gps_course_stamp_sec = self.get_now_sec()
        if speed >= self.gps_heading_min_speed_mps:
            enu_course = math.atan2(vy, vx)
            self.gps_course_map_yaw = self.normalize_angle(
                enu_course + self.gps_heading_yaw_offset_rad
            )

    def get_fresh_gps_course_yaw(self):
        """Map-frame heading from GPS Doppler course, or None if not usable.

        Requires gps_heading_yaw_offset_deg to be calibrated to this map (same
        rotation as lidar_localization_ros2's gnss_to_map_yaw_offset_deg) --
        a wrong offset silently steers wrong during exactly the localization
        dropout this is meant to help with, so it stays gated behind
        use_gps_heading_fallback until that calibration is done.
        """
        if not self.use_gps_heading_fallback:
            return None
        if self.gps_course_map_yaw is None or self.gps_course_stamp_sec is None:
            return None
        if self.gps_course_speed_mps < self.gps_heading_min_speed_mps:
            return None
        if self.get_now_sec() - self.gps_course_stamp_sec > self.gps_heading_max_age_sec:
            return None
        return self.gps_course_map_yaw

    # =========================
    # Measured vehicle speed
    # =========================
    @staticmethod
    def _source_is_fresh(received, last_ns, timeout_sec, now_ns):
        return (
            received
            and last_ns > 0
            and now_ns - last_ns <= int(timeout_sec * 1e9)
        )

    def get_pose_speed_mps(self, now_ns):
        """ความเร็วจากผลต่างตำแหน่ง /pcl_pose ระหว่างเฟรม (กรองแล้ว)"""
        if not self._source_is_fresh(
            self.pose_speed_received, self.last_speed_update_ns,
            self.pose_speed_timeout_sec, now_ns
        ):
            return None
        return max(abs(self.current_speed_x), 0.0)

    def get_odom_speed_mps(self, now_ns):
        """ความเร็วจาก /odometry (odom_bridge)"""
        if not self._source_is_fresh(
            self.odom_speed_received, self.last_odom_speed_ns,
            self.odom_timeout_sec, now_ns
        ):
            return None
        return max(abs(self.odom_speed_value), 0.0)

    def get_gps_speed_mps(self, now_ns):
        """ความเร็วจาก GPS Doppler — ใช้เฉพาะเมื่อเลือก velocity_source='gps'"""
        if self.gps_course_stamp_sec is None:
            return None
        if self.get_now_sec() - self.gps_course_stamp_sec > self.gps_heading_max_age_sec:
            return None
        return max(abs(self.gps_course_speed_mps), 0.0)

    def get_speed_status_mps(self, now_ns):
        """ความเร็วจริงจาก CAN (/speed_status) ถ้ายังสดและเซนเซอร์ยังไม่ตาย"""
        if not self._source_is_fresh(
            self.speed_status_received, self.last_speed_status_ns,
            self.speed_status_timeout_sec, now_ns
        ):
            return None
        self.update_speed_status_health(now_ns)
        if not self.speed_status_healthy:
            return None
        return max(abs(self.speed_status_value), 0.0)

    def update_speed_status_health(self, now_ns):
        """จับกรณี speed sensor ตาย: /speed_status ยัง publish อยู่ตรงเวลาแต่ค้างที่ 0.00
        ทั้งที่ pose ยืนยันว่ารถกำลังเคลื่อนที่ ถ้าค้างนานเกิน speed_status_stuck_zero_sec
        ให้เลิกเชื่อแหล่งนี้ แล้วปล่อยให้ chain ตกไปใช้ pose speed แทน"""
        if self.speed_status_stuck_zero_sec <= 0.0 or abs(self.speed_status_value) > 1e-6:
            return

        pose_speed = self.get_pose_speed_mps(now_ns)
        if pose_speed is None or pose_speed <= self.moving_speed_epsilon:
            # รถอาจจอดอยู่จริง ๆ — 0.00 ถูกต้องแล้ว ตัดสินไม่ได้ จึงรีเซ็ตตัวจับเวลา
            self.speed_status_zero_since_ns = 0
            return

        if self.speed_status_zero_since_ns == 0:
            self.speed_status_zero_since_ns = now_ns
        elif (
            self.speed_status_healthy
            and now_ns - self.speed_status_zero_since_ns
            > int(self.speed_status_stuck_zero_sec * 1e9)
        ):
            self.speed_status_healthy = False
            self.get_logger().warn(
                f'{self.speed_status_topic} ค้างที่ 0.00 นานเกิน '
                f'{self.speed_status_stuck_zero_sec:.1f}s ทั้งที่ pose วัดได้ '
                f'{pose_speed:.2f} m/s -> speed sensor น่าจะเสีย '
                f'เปลี่ยนไปใช้ความเร็วจาก pose แทน')

    def get_measured_speed_mps(self):
        """ความเร็วรถจริง (m/s, ไม่มีเครื่องหมาย) คืน (ค่า, ชื่อแหล่งที่มา)

        auto = ไล่จากแหล่งที่แม่นสุดลงไป: /speed_status (CAN) -> /pcl_pose -> /odometry
        ถ้าไม่มีแหล่งไหนใช้ได้เลยจะคืน (None, 'cmd') ให้ผู้เรียก fallback ไปใช้ค่าที่สั่ง
        """
        now_ns = self.get_clock().now().nanoseconds
        getters = {
            'speed_status': self.get_speed_status_mps,
            'pose': self.get_pose_speed_mps,
            'odom': self.get_odom_speed_mps,
            'gps': self.get_gps_speed_mps,
        }
        order = (
            ('speed_status', 'pose', 'odom')
            if self.velocity_source == 'auto'
            else (self.velocity_source,)
        )

        for name in order:
            value = getters[name](now_ns)
            if value is not None:
                self.note_velocity_source(name)
                return value, name

        self.note_velocity_source('cmd')
        return None, 'cmd'

    def note_velocity_source(self, source):
        if source != self.active_velocity_source:
            self.active_velocity_source = source
            self.get_logger().info(f'แหล่งความเร็วรถที่ใช้อยู่: {source}')

    def get_fresh_pose_speed_mps(self):
        """ความเร็วรถที่วัดได้จริง หรือ None ถ้าไม่มีแหล่งไหนสดเลย

        ชื่อเดิมคงไว้เพราะ control loop เรียกใช้หลายจุด — ตอนนี้เป็นตัวห่อ
        get_measured_speed_mps() ที่เลือกแหล่งให้อัตโนมัติ
        """
        speed, _source = self.get_measured_speed_mps()
        return speed

    def evaluate_localization_health(self):
        if not self.pose_received or self.last_pose_receive_sec is None:
            self.localization_health_state = 'stop'
            return 'stop', 'pose_not_received'

        now_sec = self.get_now_sec()
        pose_stale_sec = max(0.0, now_sec - self.last_pose_receive_sec)
        effective_interval_sec = max(self.filtered_pose_interval_sec, pose_stale_sec)
        effective_latency_sec = pose_stale_sec
        if self.use_pose_header_latency_guard:
            effective_latency_sec = max(self.filtered_pose_latency_sec, pose_stale_sec)

        if pose_stale_sec >= self.pose_stale_timeout_sec:
            state = 'stop'
            reason = (
                'pose_stale_timeout '
                f'pose_stale={pose_stale_sec:.3f}s '
                f'interval={effective_interval_sec:.3f}s'
            )
            self.localization_warn_count = 0
            self.localization_stop_count = self.localization_stop_confirm_cycles
            self.localization_recover_count = 0
            self.log_localization_health_change(state, reason)
            self.localization_health_state = state
            return state, reason

        desired_state = 'ok'
        reason = 'localization_ok'
        stop_issue = effective_interval_sec >= self.pose_interval_stop_sec
        warn_issue = effective_interval_sec >= self.pose_interval_warn_sec
        if self.use_pose_header_latency_guard:
            stop_issue = stop_issue or effective_latency_sec >= self.localization_latency_stop_sec
            warn_issue = warn_issue or effective_latency_sec >= self.localization_latency_warn_sec

        if stop_issue:
            desired_state = 'stop'
            reason = (
                'localization_stop '
                f'pose_stale={pose_stale_sec:.3f}s '
                f'latency={effective_latency_sec:.3f}s '
                f'interval={effective_interval_sec:.3f}s'
            )
        elif warn_issue:
            desired_state = 'degraded'
            reason = (
                'localization_degraded '
                f'pose_stale={pose_stale_sec:.3f}s '
                f'latency={effective_latency_sec:.3f}s '
                f'interval={effective_interval_sec:.3f}s'
            )

        if desired_state == 'stop':
            self.localization_stop_count += 1
            self.localization_warn_count = max(
                self.localization_warn_count,
                self.localization_degraded_confirm_cycles
            )
            self.localization_recover_count = 0
        elif desired_state == 'degraded':
            self.localization_warn_count += 1
            self.localization_stop_count = 0
            self.localization_recover_count = 0
        else:
            self.localization_warn_count = 0
            self.localization_stop_count = 0
            self.localization_recover_count += 1

        current_state = self.localization_health_state
        if current_state not in ('ok', 'degraded', 'stop'):
            current_state = 'ok'

        state = current_state
        if self.localization_stop_count >= self.localization_stop_confirm_cycles:
            state = 'stop'
        elif self.localization_warn_count >= self.localization_degraded_confirm_cycles:
            state = 'degraded'
        elif self.localization_recover_count >= self.localization_recover_confirm_cycles:
            state = 'ok'
            reason = 'localization_ok'

        self.log_localization_health_change(state, reason)
        self.localization_health_state = state
        return state, reason

    def reset_cte_controller(self):
        self.cte_integral = 0.0
        self.prev_cte_error = 0.0
        self.filtered_cte_rate = 0.0
        self.pid_initialized = False

    def compute_cte_correction(
        self,
        signed_cte,
        control_speed,
        path_curvature=0.0,
        straight_mode=False
    ):
        if not self.pid_initialized:
            self.prev_cte_error = signed_cte
            self.filtered_cte_rate = 0.0
            self.pid_initialized = True

        dt = max(self.control_period, 1e-3)
        cte_rate = (signed_cte - self.prev_cte_error) / dt
        self.prev_cte_error = signed_cte
        self.filtered_cte_rate = self.filtered_cte_rate + self.cte_rate_filter_alpha * (
            cte_rate - self.filtered_cte_rate
        )

        if straight_mode:
            cte_gain_scale = self.straight_cte_gain_scale
            derivative_scale = 0.0
        else:
            cte_gain_scale = self.curve_cte_gain_scale
            derivative_scale = self.curve_cte_derivative_scale

        speed_ratio = self.clamp(
            abs(control_speed) / self.cte_speed_ref_mps,
            0.0,
            1.0
        )
        if straight_mode:
            speed_scale = (
                self.straight_cte_low_speed_scale
                + (self.straight_cte_high_speed_scale - self.straight_cte_low_speed_scale)
                * speed_ratio
            )
        else:
            speed_scale = (
                self.curve_cte_low_speed_scale
                + (self.curve_cte_high_speed_scale - self.curve_cte_low_speed_scale)
                * speed_ratio
            )
            curvature_ratio = self.clamp(
                max(path_curvature, 0.0) / self.curve_cte_curvature_ref,
                0.0,
                1.0
            )
            speed_scale *= 1.0 + self.curve_cte_curvature_boost * curvature_ratio

        cte_gain_scale *= speed_scale
        derivative_scale *= max(0.45, min(speed_scale, 1.35))
        recovery_ratio = self.clamp(
            abs(signed_cte) / max(self.cte_recovery_full_steer_m, 1e-6),
            0.0,
            1.0
        )
        if straight_mode:
            center_ratio = self.clamp(
                abs(signed_cte) / max(self.straight_path_lock_cte_m, 1e-6),
                0.0,
                1.0
            )
            cte_gain_scale *= center_ratio * center_ratio
        cte_gain_scale *= (1.0 + 0.9 * recovery_ratio)
        derivative_scale *= (1.0 + 0.3 * recovery_ratio)
        cte_term = (
            self.cte_heading_gain * cte_gain_scale * signed_cte
            + self.cte_derivative_gain * derivative_scale * self.filtered_cte_rate
        )
        correction = math.atan2(cte_term, max(abs(control_speed), self.turn_speed, 0.3))
        correction_limit = min(
            self.max_steer_rad,
            self.cte_max_correction_rad * (1.0 + 0.45 * recovery_ratio)
        )
        return self.clamp(
            correction,
            -correction_limit,
            correction_limit
        )

    def apply_curve_steer_damping(
        self,
        steer_raw,
        signed_cte,
        heading_error_path,
        straight_mode=False
    ):
        if straight_mode:
            return steer_raw
        if abs(self.prev_steer_filtered) < 1e-4 or abs(steer_raw) < 1e-4:
            return steer_raw
        if steer_raw * self.prev_steer_filtered >= 0.0:
            return steer_raw
        if abs(signed_cte) > self.curve_sign_flip_cte_m:
            return steer_raw
        if abs(heading_error_path) > self.curve_sign_flip_heading_rad:
            return steer_raw
        return self.prev_steer_filtered + self.curve_sign_flip_damping * (
            steer_raw - self.prev_steer_filtered
        )

    def get_control_speed_reference(self, fallback_speed):
        fallback_speed = max(fallback_speed, 0.0)
        pose_speed = self.get_fresh_pose_speed_mps()
        if pose_speed is not None:
            return max(pose_speed, self.min_control_speed_ref)

        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self.pose_speed_timeout_sec * 1e9)
        if timeout_ns > 0 and now_ns - self.last_pose_speed_log_ns >= 5_000_000_000:
            self.get_logger().warn(
                f'Waiting for fresh pose-derived speed on {self.pose_topic}, fallback to commanded speed'
            )
            self.last_pose_speed_log_ns = now_ns
        return max(fallback_speed, self.min_control_speed_ref)

    def should_hold_zero_steer(self, signed_cte, heading_error_path, steer_raw, straight_mode=False):
        if straight_mode:
            return (
                abs(signed_cte) < self.straight_hold_cte_m
                and abs(heading_error_path) < self.straight_hold_heading_rad
                and abs(steer_raw) < self.straight_zero_band_rad
            )

        return (
            abs(signed_cte) < self.steering_hold_cte_m
            and abs(heading_error_path) < self.steering_hold_heading_rad
            and abs(steer_raw) < self.steering_zero_band_rad
        )

    def get_zone_steer_limit(
        self,
        speed_cmd,
        straight_mode=False,
        path_curvature=0.0,
        upcoming_turn_angle=0.0,
        abs_cte=0.0
    ):
        straight_limit = min(
            self.straight_mode_max_steer_rad,
            self.max_steer_rad
        )
        curve_limit = min(
            self.curve_mode_max_steer_rad,
            self.max_steer_rad
        )

        curvature_ratio = self.clamp(
            (
                max(path_curvature, 0.0) - self.straight_curvature_threshold
            ) / max(
                self.curve_mode_full_curvature - self.straight_curvature_threshold,
                1e-6
            ),
            0.0,
            1.0
        )
        turn_ratio = self.clamp(
            max(upcoming_turn_angle, 0.0) / max(self.turn_preview_full_steer_rad, 1e-6),
            0.0,
            1.0
        )
        _ = abs_cte
        assist_ratio = max(curvature_ratio, turn_ratio)

        if straight_mode and assist_ratio <= 1e-3:
            self.current_speed_zone = 'straight'
            return self.clamp(straight_limit, 0.0, self.max_steer_rad)

        if assist_ratio < 0.35:
            self.current_speed_zone = 'transition'
        else:
            self.current_speed_zone = 'curve'

        blended_limit = straight_limit + (curve_limit - straight_limit) * assist_ratio
        limit_scale, _, _ = self.get_speed_sensitive_steer_scales(speed_cmd)
        scaled_limit = blended_limit * limit_scale
        min_limit = min(straight_limit, blended_limit)
        return self.clamp(scaled_limit, min_limit, self.max_steer_rad)


    def get_speed_sensitive_steer_scales(self, speed_cmd):
        speed_cmd = max(float(speed_cmd), 0.0)
        high_speed_ratio = self.clamp(
            (
                speed_cmd - self.steer_damping_speed_low_ref
            ) / max(self.steer_damping_speed_high_ref - self.steer_damping_speed_low_ref, 1e-6),
            0.0,
            1.0
        )

        if self.current_speed_zone == 'curve':
            limit_scale = (
                1.0 - (1.0 - self.curve_high_speed_steer_limit_scale) * high_speed_ratio
            )
            filter_scale = (
                1.0 - (1.0 - self.curve_high_speed_filter_scale) * high_speed_ratio
            )
            rate_scale = (
                1.0 - (1.0 - self.curve_high_speed_rate_scale) * high_speed_ratio
            )
            return limit_scale, filter_scale, rate_scale

        if self.current_speed_zone == 'transition':
            limit_scale = (
                1.0
                - 0.5 * (
                    (1.0 - self.straight_high_speed_steer_limit_scale)
                    + (1.0 - self.curve_high_speed_steer_limit_scale)
                ) * high_speed_ratio
            )
            filter_scale = (
                1.0
                - 0.5 * (
                    (1.0 - self.straight_high_speed_filter_scale)
                    + (1.0 - self.curve_high_speed_filter_scale)
                ) * high_speed_ratio
            )
            rate_scale = (
                1.0
                - 0.5 * (
                    (1.0 - self.straight_high_speed_rate_scale)
                    + (1.0 - self.curve_high_speed_rate_scale)
                ) * high_speed_ratio
            )
            return limit_scale, filter_scale, rate_scale

        limit_scale = (
            1.0 - (1.0 - self.straight_high_speed_steer_limit_scale) * high_speed_ratio
        )
        filter_scale = (
            1.0 - (1.0 - self.straight_high_speed_filter_scale) * high_speed_ratio
        )
        rate_scale = (
            1.0 - (1.0 - self.straight_high_speed_rate_scale) * high_speed_ratio
        )
        return limit_scale, filter_scale, rate_scale


    def smooth_steer_command(self, steer_raw, speed_cmd, steer_limit_rad=None):
        if steer_limit_rad is None:
            steer_limit_rad = self.max_steer_rad
        steer_limit_rad = self.clamp(steer_limit_rad, 0.0, self.max_steer_rad)

        steer_raw = self.clamp(steer_raw, -steer_limit_rad, steer_limit_rad)

        steer_response_alpha = self.steering_filter_alpha
        steer_rate_limit = self.max_steer_rate_rad
        if self.current_speed_zone == 'curve':
            steer_response_alpha = self.curve_steering_filter_alpha
            steer_rate_limit = self.curve_max_steer_rate_rad

        _, filter_scale, rate_scale = self.get_speed_sensitive_steer_scales(speed_cmd)
        steer_response_alpha = self.clamp(
            steer_response_alpha * filter_scale,
            0.01,
            1.0
        )
        steer_rate_limit = max(steer_rate_limit * rate_scale, 0.05)

        filtered = self.prev_steer_filtered + steer_response_alpha * (
            steer_raw - self.prev_steer_filtered
        )
        max_delta = steer_rate_limit * self.control_period
        delta = self.clamp(
            filtered - self.prev_steer_filtered,
            -max_delta,
            max_delta
        )

        self.prev_steer_filtered = self.prev_steer_filtered + delta
        self.prev_steer_filtered = self.clamp(
            self.prev_steer_filtered,
            -steer_limit_rad,
            steer_limit_rad
        )

        steer_cmd = self.apply_steering_deadband_compensation(
            self.prev_steer_filtered,
            steer_limit_rad
        )
        steer_cmd = self.clamp(steer_cmd, -steer_limit_rad, steer_limit_rad)
        self.prev_steer_cmd = steer_cmd
        return steer_cmd

    def apply_steering_deadband_compensation(self, steer_cmd, steer_limit_rad=None):
        if steer_limit_rad is None:
            steer_limit_rad = self.max_steer_rad

        steer_abs = abs(steer_cmd)
        activation_rad = (
            self.straight_steering_activation_rad
            if self.current_straight_mode
            else self.steering_activation_rad
        )

        if self.deadband_comp_active:
            if steer_abs < self.steering_release_rad:
                self.deadband_comp_active = False
                return 0.0
        elif steer_abs < activation_rad:
            return 0.0

        self.deadband_comp_active = True

        if self.steering_deadband_rad >= steer_limit_rad:
            return math.copysign(steer_limit_rad, steer_cmd)

        scaled = self.steering_deadband_rad + (
            (steer_abs - activation_rad)
            * (steer_limit_rad - self.steering_deadband_rad)
            / max(steer_limit_rad - activation_rad, 1e-6)
        )
        scaled = self.clamp(scaled, self.steering_deadband_rad, steer_limit_rad)
        return math.copysign(scaled, steer_cmd)

    # =========================
    # Math helpers
    # =========================
    def quaternion_to_yaw(self, q):
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq < 1e-12:
            return self.current_yaw if self.current_yaw is not None else 0.0
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stamp_to_sec(self, stamp):
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_now_sec()

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def log_wait_once_in_a_while(self, source):
        now_ns = self.get_clock().now().nanoseconds
        wait_interval_ns = 5_000_000_000

        if source == 'pose':
            if now_ns - self.last_pose_wait_log_ns >= wait_interval_ns:
                self.get_logger().warn(f'Waiting for pose on {self.pose_topic}')
                self.last_pose_wait_log_ns = now_ns
            return

    def log_localization_health_change(self, state, reason):
        now_ns = self.get_clock().now().nanoseconds
        periodic_ns = 3_000_000_000
        should_log = state != self.localization_health_state
        if not should_log and state != 'ok':
            should_log = now_ns - self.last_localization_health_log_ns >= periodic_ns
        if not should_log:
            return

        if state == 'ok':
            self.get_logger().info('Localization health recovered.')
        elif state == 'degraded':
            self.get_logger().warn(f'Localization degraded: {reason}')
        else:
            self.get_logger().error(f'Localization unhealthy: {reason}')
        self.last_localization_health_log_ns = now_ns

    def get_now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        try:
            self.close_controller_csv_log()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerCmdVelIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
