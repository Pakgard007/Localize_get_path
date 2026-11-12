#!/usr/bin/env python3

import csv
import math
import os
import statistics

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class CsvPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')

        # loop1_qp_comfort.csv (เดิมชื่อ loop_qp_comfort.csv, ก่อนนั้น loop2_with_speed.csv)
        # = เส้นทางของแผนที่ Loop1 + โปรไฟล์ความเร็วจาก QP
        # 0.3 m spacing 1821 จุด + คอลัมน์ pose_stamp ที่สร้างจาก t_s ด้วย
        # scripts/add_pose_stamp.py
        # x/y/z ถูกแทนด้วย geometry ของเส้นที่เก็บใหม่ (path_with_speed.csv, get_path.py)
        # แล้ว โดย project จุดเดิมลงบนเส้นใหม่ ส่วน heading_deg/kappa/grade_pct คิดใหม่
        # ตามนั้น ไฟล์ก่อนแทนอยู่ที่ loop_qp_comfort.csv.bak
        # pose_stamp ยังใช้อยู่สำหรับ speed *limit* เดิม (pagard คิดจาก distance/Δstamp)
        # ส่วนคอลัมน์ v_target ตอนนี้ถูกอ่านตรง ๆ แล้วส่งออกทาง speed_profile_topic
        # เป็นอาร์เรย์ที่เรียงตรงกับ poses ใน path_topic 1:1 เพื่อให้ pagard ใช้เป็น
        # v_des ของ CBF-QP ได้ตาม waypoint (แม่นกว่าการถอดกลับจาก Δstamp)
        # เส้นทางเดิมยังอยู่ที่ path_data.csv (เส้นเดียวกัน 544 m แต่ห่าง 2.7 m/จุด)
        self.declare_parameter('csv_path', os.path.join(SCRIPT_DIR, 'loop1_qp_comfort.csv'))
        self.declare_parameter('path_topic', '/nav_path')
        self.declare_parameter('speed_profile_topic', '/nav_path_speed')
        self.declare_parameter('speed_profile_column', 'v_target')
        self.declare_parameter('display_path_topic', '/nav_path_display')
        self.declare_parameter('driven_path_topic', '/driven_path')
        self.declare_parameter('target_pose_topic', '/target_goal')
        self.declare_parameter('target_marker_topic', '/target_goal_marker')
        self.declare_parameter('enable_pose_tracking', False)
        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('target_name_topic', '/target_name')
        self.declare_parameter('target_name_map', 'kmutt canteen:550')
        self.declare_parameter('target_timeout_sec', 0.0)
        self.declare_parameter('use_target_name', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wheelbase', 1.8)
        self.declare_parameter('path_smoothing_enabled', True)
        self.declare_parameter('path_smoothing_median_window', 5)
        self.declare_parameter('path_smoothing_data_weight', 0.35)
        self.declare_parameter('path_smoothing_smooth_weight', 0.2)
        self.declare_parameter('path_smoothing_iterations', 120)
        self.declare_parameter('path_gap_fill_enabled', True)
        self.declare_parameter('path_gap_fill_max_spacing_m', 0.5)
        # ค่าหลุดจุดเดียวของ /speed_status ที่ติดมาในคอลัมน์ความเร็วของไฟล์ที่เก็บจากการขับจริง
        # (เช่น 2.24 -> 4.89 -> 2.61 ภายใน 0.8 m) จุดที่สูงหรือต่ำกว่าจุดข้างเคียงทั้งสองฝั่งเกินค่านี้
        # จะถูกแทนด้วยค่าเฉลี่ยของสองจุดนั้น เพราะ pagard ส่งความเร็วจากไฟล์ออกไปตรง ๆ (0 = ไม่แก้)
        self.declare_parameter('speed_profile_spike_mps', 1.0)
        # ไฟล์ที่เก็บด้วย get_path.py มักวิ่งเลยจุดเริ่มต้นไปอีกช่วงหนึ่งก่อนหยุดเก็บ หางช่วงนั้น
        # ซ้อนทับต้นทาง: pagard หาจุดใกล้สุดจากทั้งเส้นจึงสลับไปมาระหว่างสองรอบ และตอนปิดลูป
        # ปลายหางจะถูกลากเส้นตรงย้อนกลับไปจุดแรก -> ตัดหางออกให้เหลือรอบเดียว (0 = ไม่ตัด)
        self.declare_parameter('lap_overlap_trim_radius_m', 2.0)
        self.declare_parameter('lap_overlap_min_lap_m', 30.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('driven_path_min_spacing_m', 0.05)

        self.csv_path = os.path.expanduser(self.get_parameter('csv_path').value)
        self.path_topic = self.get_parameter('path_topic').value
        self.speed_profile_topic = str(self.get_parameter('speed_profile_topic').value)
        self.speed_profile_column = str(self.get_parameter('speed_profile_column').value)
        self.display_path_topic = self.get_parameter('display_path_topic').value
        self.driven_path_topic = self.get_parameter('driven_path_topic').value
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.target_marker_topic = self.get_parameter('target_marker_topic').value
        self.enable_pose_tracking = bool(
            self.get_parameter('enable_pose_tracking').value
        )
        self.pose_topic = self.get_parameter('pose_topic').value
        self.target_name_topic = self.get_parameter('target_name_topic').value
        self.target_name_map_raw = self.get_parameter('target_name_map').value
        self.target_timeout_sec = max(
            float(self.get_parameter('target_timeout_sec').value),
            0.0
        )
        self.use_target_name = bool(self.get_parameter('use_target_name').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.path_smoothing_enabled = bool(
            self.get_parameter('path_smoothing_enabled').value
        )
        self.path_smoothing_median_window = max(
            int(self.get_parameter('path_smoothing_median_window').value),
            1
        )
        self.path_smoothing_data_weight = max(
            float(self.get_parameter('path_smoothing_data_weight').value),
            0.0
        )
        self.path_smoothing_smooth_weight = max(
            float(self.get_parameter('path_smoothing_smooth_weight').value),
            0.0
        )
        self.path_smoothing_iterations = max(
            int(self.get_parameter('path_smoothing_iterations').value),
            0
        )
        self.path_gap_fill_enabled = bool(
            self.get_parameter('path_gap_fill_enabled').value
        )
        self.path_gap_fill_max_spacing_m = max(
            float(self.get_parameter('path_gap_fill_max_spacing_m').value),
            1e-3
        )
        self.speed_profile_spike_mps = max(
            float(self.get_parameter('speed_profile_spike_mps').value),
            0.0
        )
        self.lap_overlap_trim_radius_m = max(
            float(self.get_parameter('lap_overlap_trim_radius_m').value),
            0.0
        )
        self.lap_overlap_min_lap_m = max(
            float(self.get_parameter('lap_overlap_min_lap_m').value),
            0.0
        )
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.driven_path_min_spacing_m = max(
            float(self.get_parameter('driven_path_min_spacing_m').value),
            0.0
        )

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.speed_profile_pub = self.create_publisher(
            Float64MultiArray, self.speed_profile_topic, 10
        )
        self.display_path_pub = self.create_publisher(Path, self.display_path_topic, 10)
        self.driven_path_pub = self.create_publisher(Path, self.driven_path_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.display_path_msg = Path()
        self.display_path_msg.header.frame_id = self.frame_id
        self.driven_path_msg = Path()
        self.driven_path_msg.header.frame_id = self.frame_id
        self.last_loaded_signature = None
        self.full_path_poses = []
        self.full_path_x = []
        self.full_path_y = []
        self.full_path_z = []
        self.full_path_indices = []
        self.full_path_pose_stamp_sec = []
        self.full_path_v_target = []
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.path_indices = []
        self.path_pose_stamp_sec = []
        self.path_v_target = []
        self.driven_path_x = []
        self.driven_path_y = []
        self.driven_path_z = []
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.pose_received = False
        self.current_target_name = '-'
        self.target_waypoint_x = None
        self.target_waypoint_y = None
        self.target_waypoint_index = None
        self.last_target_msg_sec = None
        self.target_name_map = self.parse_target_name_map(self.target_name_map_raw)

        self.pose_cov_subscription = None
        self.pose_stamped_subscription = None
        if self.enable_pose_tracking:
            self.pose_cov_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_cov_callback,
                10
            )
            self.pose_stamped_subscription = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_stamped_callback,
                10
            )
        self.target_name_subscription = self.create_subscription(
            String,
            self.target_name_topic,
            self.target_name_callback,
            10
        )

        self.load_csv()
        self.timer = self.create_timer(self.publish_period, self.publish_path)

        self.get_logger().info(
            f'Publishing {self.path_topic} from {self.csv_path} in frame {self.frame_id}'
        )
        self.get_logger().info(f'Display path topic: {self.display_path_topic}')
        self.get_logger().info(f'Driven path topic: {self.driven_path_topic}')
        self.get_logger().info(f'Target pose topic: {self.target_pose_topic}')
        self.get_logger().info(f'Target marker topic: {self.target_marker_topic}')
        if self.enable_pose_tracking:
            self.get_logger().info(f'Pose topic: {self.pose_topic}')
        else:
            self.get_logger().info(
                f'Pose tracking disabled; not subscribing to {self.pose_topic}'
            )
        self.get_logger().info(f'Target topic: {self.target_name_topic}')
        self.get_logger().info(f'Target map: {self.target_name_map_raw}')
        self.get_logger().info(
            f'Use target_name filtering: {self.use_target_name}'
        )
        if self.target_timeout_sec > 0.0:
            self.get_logger().info(f'Target timeout: {self.target_timeout_sec:.1f}s')
        else:
            self.get_logger().info(
                'Target timeout: disabled (keep latest target until a new one arrives or it is cleared)'
            )
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.3f} m')
        self.get_logger().info(
            'Path smoothing: '
            f'enabled={self.path_smoothing_enabled}, '
            f'median_window={self.path_smoothing_median_window}, '
            f'data_weight={self.path_smoothing_data_weight:.2f}, '
            f'smooth_weight={self.path_smoothing_smooth_weight:.2f}, '
            f'iterations={self.path_smoothing_iterations}'
        )
        self.get_logger().info(
            'Path gap fill: '
            f'enabled={self.path_gap_fill_enabled}, '
            f'max_spacing={self.path_gap_fill_max_spacing_m:.2f} m'
        )
        self.get_logger().info(
            f'Driven path min spacing: {self.driven_path_min_spacing_m:.2f} m'
        )

    def get_file_signature(self):
        try:
            stat = os.stat(self.csv_path)
        except FileNotFoundError:
            return None
        return (stat.st_mtime_ns, stat.st_size)

    def load_csv(self, force=False):
        if not os.path.exists(self.csv_path):
            self.get_logger().error(f'CSV not found: {self.csv_path}')
            return

        file_signature = self.get_file_signature()
        if not force and file_signature == self.last_loaded_signature:
            return

        poses = []
        path_x = []
        path_y = []
        path_z = []
        path_indices = []
        path_pose_stamp_sec = []
        path_v_target = []
        with open(self.csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row_number, row in enumerate(reader):
                if not row.get('x') or not row.get('y'):
                    continue

                pose = PoseStamped()
                pose.header.frame_id = row.get('frame_id', self.frame_id) or self.frame_id

                try:
                    pose.pose.position.x = float(row['x'])
                    pose.pose.position.y = float(row['y'])
                    pose.pose.position.z = float(row.get('z', 0.0) or 0.0)
                    pose.pose.orientation.x = float(row.get('qx', 0.0) or 0.0)
                    pose.pose.orientation.y = float(row.get('qy', 0.0) or 0.0)
                    pose.pose.orientation.z = float(row.get('qz', 0.0) or 0.0)
                    pose.pose.orientation.w = float(row.get('qw', 1.0) or 1.0)
                except (TypeError, ValueError):
                    continue

                poses.append(pose)
                path_x.append(pose.pose.position.x)
                path_y.append(pose.pose.position.y)
                path_z.append(pose.pose.position.z)
                path_indices.append(self.parse_row_index(row.get('index'), row_number))
                pose_stamp_sec = self.parse_float(row.get('pose_stamp'))
                path_pose_stamp_sec.append(pose_stamp_sec)
                # ไม่มีคอลัมน์ v_target = None -> ฝั่ง pagard จะถอยไปใช้ค่า parameter เอง
                path_v_target.append(self.parse_float(row.get(self.speed_profile_column)))
                if pose_stamp_sec is not None and pose_stamp_sec > 0.0:
                    pose.header.stamp.sec = int(pose_stamp_sec)
                    pose.header.stamp.nanosec = int(
                        round((pose_stamp_sec - int(pose_stamp_sec)) * 1e9)
                    )

        path_v_target, spike_count = self.reject_speed_spikes(path_v_target)
        if spike_count > 0:
            self.get_logger().warn(
                f'Replaced {spike_count} single-point spike(s) in {self.speed_profile_column} '
                f'(more than {self.speed_profile_spike_mps:.1f} m/s off both neighbours)'
            )

        # ตัดจากตำแหน่งดิบ ก่อน smoothing/gap fill จะขยับหรือเติมจุด
        keep_count = self.lap_overlap_keep_count(path_x, path_y)
        if keep_count < len(poses):
            self.get_logger().warn(
                f'Dropped the last {len(poses) - keep_count} of {len(poses)} waypoints: '
                'they run past the start and overlap the beginning of the path '
                f'(one lap = {keep_count} waypoints)'
            )
            poses = poses[:keep_count]
            path_x = path_x[:keep_count]
            path_y = path_y[:keep_count]
            path_z = path_z[:keep_count]
            path_indices = path_indices[:keep_count]
            path_pose_stamp_sec = path_pose_stamp_sec[:keep_count]
            path_v_target = path_v_target[:keep_count]

        inserted_pose_count = 0

        if self.path_smoothing_enabled and len(poses) >= 3:
            path_x, path_y = self.smooth_path_positions(path_x, path_y)

        if self.path_gap_fill_enabled and len(poses) >= 2:
            (
                poses,
                path_x,
                path_y,
                path_z,
                path_indices,
                path_pose_stamp_sec,
                path_v_target,
                inserted_pose_count,
            ) = self.fill_path_gaps(
                poses,
                path_x,
                path_y,
                path_z,
                path_indices,
                path_pose_stamp_sec,
                path_v_target,
            )

        if poses:
            self.update_pose_positions_and_headings(
                poses,
                path_x,
                path_y,
                path_z,
                path_pose_stamp_sec,
            )

        self.last_loaded_signature = file_signature
        self.full_path_poses = poses
        self.full_path_x = path_x
        self.full_path_y = path_y
        self.full_path_z = path_z
        self.full_path_indices = path_indices
        self.full_path_pose_stamp_sec = path_pose_stamp_sec
        self.full_path_v_target = path_v_target
        self.rebuild_active_path()
        self.rebuild_display_path()
        self.refresh_target_waypoint()

        if inserted_pose_count > 0:
            self.get_logger().info(
                f'Loaded {len(poses)} poses from CSV ({inserted_pose_count} interpolated)'
            )
        else:
            self.get_logger().info(f'Loaded {len(poses)} poses from CSV')

    def reject_speed_spikes(self, speeds):
        """แทนค่าหลุดจุดเดียวในโปรไฟล์ความเร็วด้วยค่าเฉลี่ยของจุดข้างเคียง คืน (โปรไฟล์, จำนวนที่แก้)

        ค่าหลุด = ต่างจากจุดก่อนและจุดหลังไปทางเดียวกันเกิน speed_profile_spike_mps ทั้งคู่
        การเร่งหรือเบรกจริงไม่เข้าเงื่อนไขนี้ เพราะจุดก่อนกับจุดหลังอยู่คนละฝั่งของค่านั้น
        """
        limit = self.speed_profile_spike_mps
        if limit <= 0.0:
            return speeds, 0

        cleaned = list(speeds)
        spike_count = 0
        for i in range(1, len(speeds) - 1):
            before, value, after = speeds[i - 1], speeds[i], speeds[i + 1]
            if before is None or value is None or after is None:
                continue
            above_before = value - before
            above_after = value - after
            if min(above_before, above_after) > limit or max(above_before, above_after) < -limit:
                cleaned[i] = 0.5 * (before + after)
                spike_count += 1
        return cleaned, spike_count

    def lap_overlap_keep_count(self, path_x, path_y):
        """จำนวนจุดแรกที่ควรเก็บไว้ เมื่อเส้นทางวนกลับมาผ่านจุดเริ่มต้นแล้ววิ่งต่อ

        หาครั้งแรก (หลังวิ่งไปแล้วอย่างน้อย lap_overlap_min_lap_m) ที่เส้นทางกลับเข้ามาในรัศมี
        lap_overlap_trim_radius_m ของจุดแรกโดยหันไปทางเดียวกับตอนออกตัว แล้วตัดที่จุดสุดท้าย
        ก่อนถึงจุดแรก ปลายเส้นจึงห่างจุดแรกไม่เกินระยะระหว่างจุด และ pagard ปิดลูปได้ต่อเนื่อง
        เส้นทางที่ไม่ได้วนกลับมา หรือหยุดก่อนถึงจุดแรก ได้จำนวนจุดเท่าเดิม
        """
        count = len(path_x)
        radius = self.lap_overlap_trim_radius_m
        if radius <= 0.0 or count < 4:
            return count

        start_x = path_x[0]
        start_y = path_y[0]
        # ทิศออกตัว: จากจุดแรกไปยังจุดแรกที่ห่างเกิน 1 m
        dir_x = dir_y = None
        for i in range(1, count):
            dx = path_x[i] - start_x
            dy = path_y[i] - start_y
            dist = math.hypot(dx, dy)
            if dist >= 1.0:
                dir_x = dx / dist
                dir_y = dy / dist
                break
        if dir_x is None:
            return count

        travelled = 0.0
        entered_at = None   # จุดแรกของช่วงที่กลับเข้ามาในรัศมี
        keep_index = None   # จุดสุดท้ายในช่วงนั้นที่ยังไม่ถึงจุดแรก
        for i in range(1, count):
            step_x = path_x[i] - path_x[i - 1]
            step_y = path_y[i] - path_y[i - 1]
            step = math.hypot(step_x, step_y)
            travelled += step
            if travelled < self.lap_overlap_min_lap_m:
                continue

            off_x = path_x[i] - start_x
            off_y = path_y[i] - start_y
            if math.hypot(off_x, off_y) > radius:
                if entered_at is not None:
                    break
                continue
            # ผ่านจุดแรกแบบสวนทางหรือตัดขวาง ไม่ใช่การวนครบรอบ
            if step < 1e-6 or (step_x * dir_x + step_y * dir_y) / step < 0.5:
                continue

            if entered_at is None:
                entered_at = i
            if off_x * dir_x + off_y * dir_y <= 0.0:
                keep_index = i

        if entered_at is None:
            return count
        if keep_index is None:
            # ช่วงจุดห่างคร่อมจุดแรกพอดี: ตัดที่จุดสุดท้ายก่อนเข้ารัศมี
            keep_index = entered_at - 1
        return min(keep_index + 1, count)

    def publish_path(self):
        self.load_csv()
        self.check_target_timeout()
        self.rebuild_active_path()
        self.rebuild_display_path()

        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        self.path_msg.header.frame_id = self.frame_id
        self.display_path_msg.header.stamp = now
        self.display_path_msg.header.frame_id = self.frame_id
        self.driven_path_msg.header.stamp = now
        self.driven_path_msg.header.frame_id = self.frame_id

        for pose in self.path_msg.poses:
            pose.header.frame_id = self.frame_id

        for pose in self.display_path_msg.poses:
            pose.header.frame_id = self.frame_id

        for pose in self.driven_path_msg.poses:
            pose.header.frame_id = self.frame_id

        # ส่ง speed profile ก่อน path เสมอ: ฝั่ง pagard ผูกโปรไฟล์เข้ากับ path ตอนที่
        # path_callback ทำงาน ถ้าส่งสลับกันจะมีหนึ่งเฟรมที่ path ใหม่จับคู่กับโปรไฟล์เก่า
        self.publish_speed_profile()
        self.path_pub.publish(self.path_msg)
        self.display_path_pub.publish(self.display_path_msg)
        self.driven_path_pub.publish(self.driven_path_msg)
        self.publish_target_visualization(now)

    def publish_speed_profile(self):
        """v_target ของทุก pose ใน path_topic เรียงตรงกัน 1:1

        จุดที่ CSV ไม่มี v_target จะส่งเป็น NaN แทนการข้าม เพื่อให้ความยาวอาร์เรย์
        เท่ากับจำนวน pose เสมอ — ฝั่งรับใช้ความยาวนี้ยืนยันว่าโปรไฟล์ตรงกับ path
        """
        msg = Float64MultiArray()
        msg.data = [
            float('nan') if value is None else float(value)
            for value in self.path_v_target[:len(self.path_msg.poses)]
        ]
        self.speed_profile_pub.publish(msg)

    def pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        self.update_current_pose(msg.header.stamp, msg.pose.pose)

    def pose_stamped_callback(self, msg: PoseStamped):
        self.update_current_pose(msg.header.stamp, msg.pose)

    def target_name_callback(self, msg: String):
        if not self.use_target_name:
            if self.current_target_name != '-':
                self.clear_current_target()
            return

        self.last_target_msg_sec = self.get_clock().now().nanoseconds * 1e-9
        target_name = msg.data.strip() or '-'
        if target_name == self.current_target_name:
            self.rebuild_active_path()
            self.rebuild_display_path()
            return

        self.current_target_name = target_name
        self.refresh_target_waypoint(log_when_missing=True)

    def clear_current_target(self):
        self.current_target_name = '-'
        self.target_waypoint_x = None
        self.target_waypoint_y = None
        self.target_waypoint_index = None
        self.last_target_msg_sec = None

    def check_target_timeout(self):
        if not self.use_target_name:
            return

        if self.target_timeout_sec <= 0.0:
            return

        if self.last_target_msg_sec is None or self.current_target_name in ('', '-'):
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_target_msg_sec <= self.target_timeout_sec:
            return

        timed_out_target = self.current_target_name
        self.clear_current_target()
        self.get_logger().warn(
            f'Target "{timed_out_target}" timed out after {self.target_timeout_sec:.1f}s'
        )

    def publish_target_visualization(self, stamp):
        if not self.has_active_target():
            self.delete_target_marker(stamp)
            return

        target_pose = PoseStamped()
        target_pose.header.stamp = stamp
        target_pose.header.frame_id = self.frame_id
        target_pose.pose.position.x = float(self.target_waypoint_x)
        target_pose.pose.position.y = float(self.target_waypoint_y)
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0
        self.target_pose_pub.publish(target_pose)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'target_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = target_pose.pose
        marker.scale.x = 1.4
        marker.scale.y = 1.4
        marker.scale.z = 1.4
        marker.color.a = 0.9
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        self.target_marker_pub.publish(marker)

    def delete_target_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'target_goal'
        marker.id = 0
        marker.action = Marker.DELETE
        self.target_marker_pub.publish(marker)

    def refresh_target_waypoint(self, log_when_missing=False):
        self.target_waypoint_x = None
        self.target_waypoint_y = None
        self.target_waypoint_index = None

        if not self.use_target_name:
            return

        normalized_target_name = self.normalize_target_name(self.current_target_name)
        if not normalized_target_name or normalized_target_name == '-':
            return

        mapped_index = self.target_name_map.get(normalized_target_name)
        if mapped_index is None:
            if log_when_missing:
                self.get_logger().warn(
                    f'Target "{self.current_target_name}" is not in target_name_map'
                )
            return

        for path_pos, path_index in enumerate(self.full_path_indices):
            if path_index != mapped_index:
                continue

            self.target_waypoint_index = mapped_index
            self.target_waypoint_x = self.full_path_x[path_pos]
            self.target_waypoint_y = self.full_path_y[path_pos]
            self.get_logger().info(
                f'Target "{self.current_target_name}" -> waypoint index {mapped_index} '
                f'at x={self.target_waypoint_x:.3f}, y={self.target_waypoint_y:.3f}'
            )
            return

        if log_when_missing:
            self.get_logger().warn(
                f'Target "{self.current_target_name}" maps to index {mapped_index}, '
                'but that index was not found in the loaded path'
            )

    def rebuild_active_path(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.path_msg.poses = [
            self.copy_pose_stamped(pose)
            for pose in self.full_path_poses
        ]
        self.path_x = self.full_path_x[:]
        self.path_y = self.full_path_y[:]
        self.path_z = self.full_path_z[:]
        self.path_indices = self.full_path_indices[:]
        self.path_pose_stamp_sec = self.full_path_pose_stamp_sec[:]
        self.path_v_target = self.full_path_v_target[:]

    def rebuild_display_path(self):
        self.display_path_msg = Path()
        self.display_path_msg.header.frame_id = self.frame_id

        if len(self.full_path_poses) < 2:
            return

        self.display_path_msg.poses = [
            self.copy_pose_stamped(pose)
            for pose in self.full_path_poses
        ]

    def update_current_pose(self, stamp, pose):
        first_pose = not self.pose_received
        self.current_x = float(pose.position.x)
        self.current_y = float(pose.position.y)
        self.current_z = float(pose.position.z)
        current_yaw = self.quaternion_to_yaw(pose.orientation)
        self.pose_received = True

        self.append_driven_path_pose(stamp, current_yaw)
        self.rebuild_active_path()

        if first_pose:
            self.get_logger().info(
                f'Pose received on {self.pose_topic}: '
                f'x={self.current_x:.3f}, y={self.current_y:.3f}, z={self.current_z:.3f}'
            )

    def parse_float(self, value):
        if value in (None, ''):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def parse_row_index(self, value, fallback_index):
        if value in (None, ''):
            return int(fallback_index)
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return int(fallback_index)

    def normalize_target_name(self, target_name):
        return ' '.join((target_name or '').strip().lower().split())

    def parse_target_name_map(self, raw_value):
        target_map = {}
        for item in str(raw_value).split(';'):
            item = item.strip()
            if not item or ':' not in item:
                continue

            name, index_value = item.rsplit(':', 1)
            normalized_name = self.normalize_target_name(name)
            if not normalized_name:
                continue

            try:
                target_map[normalized_name] = int(index_value.strip())
            except ValueError:
                continue

        return target_map

    def has_active_target(self):
        return (
            self.current_target_name not in ('', '-')
            and self.target_waypoint_index is not None
        )

    def clear_active_path(self):
        self.path_msg.poses = []
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.path_indices = []
        self.path_pose_stamp_sec = []
        self.path_v_target = []

    def copy_pose_stamped(self, source_pose):
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = source_pose.header.frame_id
        copied_pose.header.stamp = source_pose.header.stamp
        copied_pose.pose.position.x = source_pose.pose.position.x
        copied_pose.pose.position.y = source_pose.pose.position.y
        copied_pose.pose.position.z = source_pose.pose.position.z
        copied_pose.pose.orientation.x = source_pose.pose.orientation.x
        copied_pose.pose.orientation.y = source_pose.pose.orientation.y
        copied_pose.pose.orientation.z = source_pose.pose.orientation.z
        copied_pose.pose.orientation.w = source_pose.pose.orientation.w
        return copied_pose

    def make_pose_stamped(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        self.set_pose_yaw(pose, yaw)
        return pose

    def append_driven_path_pose(self, stamp, yaw):
        if self.current_x is None or self.current_y is None or self.current_z is None:
            return

        new_pose = self.make_pose_stamped(
            self.current_x,
            self.current_y,
            self.current_z,
            yaw,
        )
        new_pose.header.stamp = stamp

        if self.driven_path_x:
            dx = self.current_x - self.driven_path_x[-1]
            dy = self.current_y - self.driven_path_y[-1]
            dz = self.current_z - self.driven_path_z[-1]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < self.driven_path_min_spacing_m:
                self.driven_path_msg.poses[-1] = new_pose
                self.driven_path_x[-1] = self.current_x
                self.driven_path_y[-1] = self.current_y
                self.driven_path_z[-1] = self.current_z
                return

        self.driven_path_msg.poses.append(new_pose)
        self.driven_path_x.append(self.current_x)
        self.driven_path_y.append(self.current_y)
        self.driven_path_z.append(self.current_z)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, q):
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq < 1e-12:
            return 0.0
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def fill_path_gaps(
        self,
        poses,
        path_x,
        path_y,
        path_z,
        path_indices,
        path_pose_stamp_sec,
        path_v_target,
    ):
        if len(poses) < 2 or self.path_gap_fill_max_spacing_m <= 1e-6:
            return (
                poses[:],
                path_x[:],
                path_y[:],
                path_z[:],
                path_indices[:],
                path_pose_stamp_sec[:],
                path_v_target[:],
                0,
            )

        filled_poses = [self.copy_pose_stamped(poses[0])]
        filled_x = [path_x[0]]
        filled_y = [path_y[0]]
        filled_z = [path_z[0]]
        filled_indices = [path_indices[0]]
        filled_pose_stamp_sec = [path_pose_stamp_sec[0]]
        filled_v_target = [path_v_target[0]]
        inserted_pose_count = 0

        for index in range(len(poses) - 1):
            start_x = path_x[index]
            start_y = path_y[index]
            start_z = path_z[index]
            end_x = path_x[index + 1]
            end_y = path_y[index + 1]
            end_z = path_z[index + 1]
            segment_length = math.hypot(end_x - start_x, end_y - start_y)
            insert_count = max(
                int(math.ceil(segment_length / self.path_gap_fill_max_spacing_m)) - 1,
                0,
            )

            for step in range(1, insert_count + 1):
                ratio = step / float(insert_count + 1)
                filled_poses.append(
                    self.make_pose_stamped(
                        self.interpolate_value(start_x, end_x, ratio),
                        self.interpolate_value(start_y, end_y, ratio),
                        self.interpolate_value(start_z, end_z, ratio),
                        0.0,
                    )
                )
                filled_x.append(self.interpolate_value(start_x, end_x, ratio))
                filled_y.append(self.interpolate_value(start_y, end_y, ratio))
                filled_z.append(self.interpolate_value(start_z, end_z, ratio))
                filled_indices.append(path_indices[index])
                filled_pose_stamp_sec.append(
                    self.interpolate_optional_value(
                        path_pose_stamp_sec[index],
                        path_pose_stamp_sec[index + 1],
                        ratio,
                    )
                )
                # v_target ระหว่างจุดที่แทรกเข้ามา ใช้ interpolate เหมือน pose_stamp
                filled_v_target.append(
                    self.interpolate_optional_value(
                        path_v_target[index],
                        path_v_target[index + 1],
                        ratio,
                    )
                )
                inserted_pose_count += 1

            filled_poses.append(self.copy_pose_stamped(poses[index + 1]))
            filled_x.append(end_x)
            filled_y.append(end_y)
            filled_z.append(end_z)
            filled_indices.append(path_indices[index + 1])
            filled_pose_stamp_sec.append(path_pose_stamp_sec[index + 1])
            filled_v_target.append(path_v_target[index + 1])

        return (
            filled_poses,
            filled_x,
            filled_y,
            filled_z,
            filled_indices,
            filled_pose_stamp_sec,
            filled_v_target,
            inserted_pose_count,
        )

    def smooth_path_positions(self, path_x, path_y):
        filtered_x = self.apply_median_filter(
            path_x,
            self.path_smoothing_median_window
        )
        filtered_y = self.apply_median_filter(
            path_y,
            self.path_smoothing_median_window
        )

        if self.path_smoothing_iterations <= 0 or (
            self.path_smoothing_data_weight <= 0.0
            and self.path_smoothing_smooth_weight <= 0.0
        ):
            return filtered_x, filtered_y

        smoothed_x = filtered_x[:]
        smoothed_y = filtered_y[:]
        reference_x = filtered_x[:]
        reference_y = filtered_y[:]

        for _ in range(self.path_smoothing_iterations):
            for index in range(1, len(smoothed_x) - 1):
                smoothed_x[index] += (
                    self.path_smoothing_data_weight
                    * (reference_x[index] - smoothed_x[index])
                    + self.path_smoothing_smooth_weight
                    * (
                        smoothed_x[index - 1]
                        + smoothed_x[index + 1]
                        - 2.0 * smoothed_x[index]
                    )
                )
                smoothed_y[index] += (
                    self.path_smoothing_data_weight
                    * (reference_y[index] - smoothed_y[index])
                    + self.path_smoothing_smooth_weight
                    * (
                        smoothed_y[index - 1]
                        + smoothed_y[index + 1]
                        - 2.0 * smoothed_y[index]
                    )
                )

        smoothed_x[0] = path_x[0]
        smoothed_y[0] = path_y[0]
        smoothed_x[-1] = path_x[-1]
        smoothed_y[-1] = path_y[-1]
        return smoothed_x, smoothed_y

    def apply_median_filter(self, values, window_size):
        if len(values) < 3 or window_size <= 1:
            return values[:]

        if window_size % 2 == 0:
            window_size += 1
        half_window = window_size // 2
        filtered = values[:]

        for index in range(1, len(values) - 1):
            start_index = max(0, index - half_window)
            end_index = min(len(values), index + half_window + 1)
            filtered[index] = statistics.median(values[start_index:end_index])

        return filtered

    def update_pose_positions_and_headings(
        self,
        poses,
        path_x,
        path_y,
        path_z,
        path_pose_stamp_sec,
    ):
        headings = self.compute_path_headings(path_x, path_y)
        for index, pose in enumerate(poses):
            pose.pose.position.x = path_x[index]
            pose.pose.position.y = path_y[index]
            pose.pose.position.z = path_z[index]
            pose.header.frame_id = self.frame_id
            pose_stamp_sec = self.parse_float(
                None if index >= len(path_pose_stamp_sec) else path_pose_stamp_sec[index]
            )
            if pose_stamp_sec is not None and pose_stamp_sec > 0.0:
                pose.header.stamp.sec = int(pose_stamp_sec)
                pose.header.stamp.nanosec = int(
                    round((pose_stamp_sec - int(pose_stamp_sec)) * 1e9)
                )
            self.set_pose_yaw(pose, headings[index])

    def set_pose_yaw(self, pose_stamped, yaw):
        half_yaw = 0.5 * yaw
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(half_yaw)
        pose_stamped.pose.orientation.w = math.cos(half_yaw)

    def interpolate_value(self, start_value, end_value, ratio):
        return start_value + (end_value - start_value) * ratio

    def interpolate_optional_value(self, start_value, end_value, ratio):
        if start_value is None and end_value is None:
            return None
        if start_value is None:
            return end_value
        if end_value is None:
            return start_value
        return self.interpolate_value(start_value, end_value, ratio)

    def compute_path_headings(self, path_x, path_y):
        point_count = len(path_x)
        if point_count == 0:
            return []
        if point_count == 1:
            return [0.0]

        headings = []
        for index in range(point_count):
            if index == 0:
                dx = path_x[1] - path_x[0]
                dy = path_y[1] - path_y[0]
            elif index == point_count - 1:
                dx = path_x[-1] - path_x[-2]
                dy = path_y[-1] - path_y[-2]
            else:
                dx = path_x[index + 1] - path_x[index - 1]
                dy = path_y[index + 1] - path_y[index - 1]

            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                heading = headings[-1] if headings else 0.0
            else:
                heading = math.atan2(dy, dx)
            headings.append(heading)

        return headings



def main(args=None):
    rclpy.init(args=args)
    node = CsvPathPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
