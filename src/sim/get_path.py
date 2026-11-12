#!/usr/bin/env python3

from collections import deque
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import csv
import os


class PathToCSV(Node):

    def __init__(self):
        super().__init__('path_to_csv')

        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('speed_topic', '/speed_status')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('file_path', os.path.expanduser('~/path_with_speed.csv'))
        # 0.30 m/จุด = ระยะเดียวกับ loop2_with_speed.csv ที่ nav_path.py ใช้อยู่
        self.declare_parameter('min_spacing_m', 0.3)
        self.declare_parameter('append_to_file', False)
        self.declare_parameter('max_speed_age_sec', 0.2)
        # /speed_status มีค่าหลุด 327.67 m/s (int16 เต็มสเกล = บอร์ดยังไม่มีค่าอ่าน) ปนมาเป็นครั้งคราว
        # v_target ในไฟล์นี้คือความเร็วเป้าหมายตอนรถวิ่งเอง จึงทิ้งค่าที่เกินนี้ไม่เก็บลงไฟล์
        # (เกณฑ์เดียวกับ odom_bridge.py)
        self.declare_parameter('max_plausible_speed_mps', 8.0)
        self.declare_parameter('max_imu_age_sec', 0.2)
        self.declare_parameter('drop_unsynced_samples', False)
        # heading คิดจาก chord ย้อนหลังเท่านี้ ไม่ใช่จุดติดกัน กัน noise ของ localization
        self.declare_parameter('heading_window_m', 0.9)
        self.declare_parameter('curve_kappa', 0.025)
        self.declare_parameter('climb_grade_pct', 2.5)
        self.declare_parameter('descent_grade_pct', -2.5)
        self.declare_parameter('launch_speed_mps', 2.0)
        self.declare_parameter('stop_speed_mps', 0.1)
        self.declare_parameter('brake_decel_mps2', 0.3)

        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.speed_topic = self.get_parameter('speed_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.min_spacing_m = float(self.get_parameter('min_spacing_m').value)
        self.append_to_file = bool(self.get_parameter('append_to_file').value)
        self.max_speed_age_sec = float(self.get_parameter('max_speed_age_sec').value)
        self.max_plausible_speed_mps = float(
            self.get_parameter('max_plausible_speed_mps').value
        )
        self.max_imu_age_sec = float(self.get_parameter('max_imu_age_sec').value)
        self.drop_unsynced_samples = bool(
            self.get_parameter('drop_unsynced_samples').value
        )
        self.heading_window_m = float(self.get_parameter('heading_window_m').value)
        self.curve_kappa = float(self.get_parameter('curve_kappa').value)
        self.climb_grade_pct = float(self.get_parameter('climb_grade_pct').value)
        self.descent_grade_pct = float(self.get_parameter('descent_grade_pct').value)
        self.launch_speed_mps = float(self.get_parameter('launch_speed_mps').value)
        self.stop_speed_mps = float(self.get_parameter('stop_speed_mps').value)
        self.brake_decel_mps2 = float(self.get_parameter('brake_decel_mps2').value)

        self.saved_waypoint_count = 0
        self.last_written_position = None
        self.speed_samples = deque(maxlen=200)
        self.imu_samples = deque(maxlen=200)
        self.last_unsynced_warn_ns = 0

        # สถานะสำหรับคอลัมน์ s_m / heading_deg / kappa / grade_pct / t_s
        self.recent_points = deque(maxlen=64)
        self.s_total_m = 0.0
        self.first_pose_stamp = None
        self.prev_heading_deg = None
        self.prev_v_target = None
        self.prev_t_s = None

        file_dir = os.path.dirname(self.file_path)
        if file_dir:
            os.makedirs(file_dir, exist_ok=True)

        if (not self.append_to_file) or (not os.path.exists(self.file_path)):
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        'index',
                        'pose_stamp',
                        's_m',
                        'x',
                        'y',
                        'z',
                        'heading_deg',
                        'kappa',
                        'grade_pct',
                        'behavior',
                        'v_target',
                        't_s',
                    ]
                )

        self.pose_cov_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_cov_callback,
            10
        )
        self.speed_subscription = self.create_subscription(
            Float64,
            self.speed_topic,
            self.speed_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f'Pose logger started on {self.pose_topic}, writing to {self.file_path}, '
            f'min_spacing={self.min_spacing_m:.2f} m, '
            f'speed_topic={self.speed_topic}, imu_topic={self.imu_topic}, '
            f'max_speed_age={self.max_speed_age_sec:.3f}s, '
            f'max_imu_age={self.max_imu_age_sec:.3f}s'
        )

    def speed_callback(self, msg: Float64):
        speed_mps = float(msg.data)
        # NaN ก็ไม่ผ่านเงื่อนไขนี้เหมือนกัน จุดนั้นจะใช้ค่าปกติตัวที่ใกล้ที่สุดแทน
        if not abs(speed_mps) <= self.max_plausible_speed_mps:
            self.get_logger().warn(
                f'Ignoring implausible {self.speed_topic} {speed_mps:.2f} m/s',
                throttle_duration_sec=10.0,
            )
            return
        speed_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.speed_samples.append((speed_stamp, speed_mps))

    def imu_callback(self, msg: Imu):
        imu_stamp = self.stamp_to_sec(msg.header.stamp)
        yaw = self.quaternion_to_yaw(msg.orientation)
        imu_yaw = self.normalize_angle(yaw + self.yaw_offset)
        self.imu_samples.append((imu_stamp, imu_yaw))

    def pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        self.save_pose(msg.header, msg.pose.pose)

    def save_pose(self, header, pose):
        pose_stamp = self.stamp_to_sec(header.stamp)
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)

        step_m = 0.0
        if self.last_written_position is not None:
            dx = x - self.last_written_position[0]
            dy = y - self.last_written_position[1]
            step_m = math.hypot(dx, dy)
            if step_m < self.min_spacing_m:
                return

        speed_mps, speed_age_sec = self.find_closest_sample(
            self.speed_samples, pose_stamp, self.max_speed_age_sec
        )
        imu_yaw_rad, imu_age_sec = self.find_closest_sample(
            self.imu_samples, pose_stamp, self.max_imu_age_sec
        )

        if self.drop_unsynced_samples and (speed_mps is None or imu_yaw_rad is None):
            self.log_unsynced_pose(speed_age_sec, imu_age_sec)
            return

        if self.first_pose_stamp is None:
            self.first_pose_stamp = pose_stamp
        t_s = pose_stamp - self.first_pose_stamp
        self.s_total_m += step_m

        # ไม่มี /speed_status ก็ถอยไปใช้ความเร็วจากระยะที่ pose ขยับต่อเวลา
        v_target = speed_mps
        if v_target is None:
            v_target = self.speed_from_motion(step_m, t_s)

        # จุดอ้างอิงย้อนหลัง 1 จุด ใช้ร่วมกันทั้ง heading / kappa / grade / accel
        anchor = self.window_anchor(x, y)
        heading_deg, heading_from_chord = self.compute_heading_deg(
            x, y, anchor, pose, imu_yaw_rad
        )
        kappa = self.compute_kappa(heading_deg, heading_from_chord, anchor)
        grade_pct = self.compute_grade_pct(x, y, z, anchor)
        accel_mps2 = self.compute_accel(v_target, t_s, anchor)
        behavior = self.classify_behavior(v_target, kappa, grade_pct, accel_mps2)

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    self.saved_waypoint_count,
                    f'{pose_stamp:.6f}',
                    f'{self.s_total_m:.2f}',
                    f'{x:.3f}',
                    f'{y:.3f}',
                    f'{z:.3f}',
                    f'{heading_deg:.1f}',
                    f'{kappa:.5f}',
                    f'{grade_pct:.2f}',
                    behavior,
                    f'{v_target:.3f}',
                    f'{t_s:.2f}',
                ]
            )

        self.recent_points.append(
            (self.s_total_m, x, y, z, heading_deg, v_target, t_s, heading_from_chord)
        )
        self.prev_heading_deg = heading_deg
        self.prev_v_target = v_target
        self.prev_t_s = t_s
        self.saved_waypoint_count += 1
        self.last_written_position = (x, y, z)
        self.get_logger().info(
            f'Saved waypoint {self.saved_waypoint_count} from {self.pose_topic} '
            f'at s={self.s_total_m:.2f} m, x={x:.3f}, y={y:.3f}, '
            f'v_target={v_target:.2f}, behavior={behavior}'
        )

    def speed_from_motion(self, step_m, t_s):
        if self.prev_t_s is None:
            return 0.0
        dt = t_s - self.prev_t_s
        if dt <= 0.0:
            return self.prev_v_target if self.prev_v_target is not None else 0.0
        return step_m / dt

    def window_anchor(self, x, y):
        """จุดย้อนหลังที่ห่างอย่างน้อย heading_window_m ไว้ทำ chord ของ heading/grade"""
        anchor = None
        for point in reversed(self.recent_points):
            anchor = point
            if math.hypot(x - point[1], y - point[2]) >= self.heading_window_m:
                break
        return anchor

    def compute_heading_deg(self, x, y, anchor, pose, imu_yaw_rad):
        if anchor is not None:
            dx = x - anchor[1]
            dy = y - anchor[2]
            if math.hypot(dx, dy) > 1e-6:
                return math.degrees(math.atan2(dy, dx)), True

        # จุดแรกยังไม่มี chord ให้คิด ใช้ yaw ของ pose (ไม่มีก็เอา imu)
        yaw = self.quaternion_to_yaw(pose.orientation)
        if abs(yaw) < 1e-9 and imu_yaw_rad is not None:
            yaw = imu_yaw_rad
        if self.prev_heading_deg is not None and yaw == 0.0:
            return self.prev_heading_deg, False
        return math.degrees(self.normalize_angle(yaw + self.yaw_offset)), False

    def compute_kappa(self, heading_deg, heading_from_chord, anchor):
        """kappa = dheading/ds เทียบกับจุดอ้างอิง (บวก = เลี้ยวซ้าย) เหมือน loop2_with_speed.csv

        heading ที่ยังไม่ได้มาจาก chord (จุดแรก ๆ) ใช้คิดไม่ได้ ไม่งั้น kappa จะเด้ง
        """
        if anchor is None or not heading_from_chord or not anchor[7]:
            return 0.0
        ds = self.s_total_m - anchor[0]
        if ds <= 1e-6:
            return 0.0
        dtheta = self.normalize_angle(math.radians(heading_deg - anchor[4]))
        return dtheta / ds

    def compute_grade_pct(self, x, y, z, anchor):
        if anchor is None:
            return 0.0
        run_m = math.hypot(x - anchor[1], y - anchor[2])
        if run_m <= 1e-6:
            return 0.0
        return (z - anchor[3]) / run_m * 100.0

    def compute_accel(self, v_target, t_s, anchor):
        if anchor is None:
            return None
        dt = t_s - anchor[6]
        if dt <= 1e-6:
            return None
        return (v_target - anchor[5]) / dt

    def classify_behavior(self, v_target, kappa, grade_pct, accel):
        if v_target <= self.stop_speed_mps:
            return 'launch'
        if accel is not None and accel <= -self.brake_decel_mps2:
            return 'brake'
        if v_target < self.launch_speed_mps and (accel is None or accel > 0.0):
            return 'launch'

        parts = []
        if abs(kappa) >= self.curve_kappa:
            parts.append('curve')
        if grade_pct >= self.climb_grade_pct:
            parts.append('climb')
        elif grade_pct <= self.descent_grade_pct:
            parts.append('descent')

        return '+'.join(parts) if parts else 'cruise'

    def stamp_to_sec(self, stamp):
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_clock().now().nanoseconds * 1e-9

    def find_closest_sample(self, samples, target_stamp_sec, max_age_sec):
        best_value = None
        best_age_sec = None

        for sample_stamp_sec, sample_value in reversed(samples):
            age_sec = abs(sample_stamp_sec - target_stamp_sec)
            if age_sec <= max_age_sec and (
                best_age_sec is None or age_sec < best_age_sec
            ):
                best_value = sample_value
                best_age_sec = age_sec

        return best_value, best_age_sec

    def log_unsynced_pose(self, speed_age_sec, imu_age_sec):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_unsynced_warn_ns < 1_000_000_000:
            return

        self.last_unsynced_warn_ns = now_ns
        self.get_logger().warn(
            'Skipped pose because synced speed/imu data was not available within '
            f'age limits: speed_age={speed_age_sec}, imu_age={imu_age_sec}'
        )

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    node = PathToCSV()

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
