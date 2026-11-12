#!/usr/bin/env python3
"""
odom_bridge — combines /speed_status (CAN wheel speed) + IMU yaw rate into
nav_msgs/Odometry so lidar_localization's use_odom path (odomReceived in
lidar_localization_component.cpp) has a motion prior between LiDAR scans.
Neither /speed_status (std_msgs/Float64) nor the IMU alone carry both fields
that odomReceived needs (twist.linear.x + twist.angular.z).

Lives in this package, and is started by lidar_localization.launch.py, because
localization is the only consumer and it fails badly without it. With use_odom
on and nothing publishing /odometry, every scan's prior is "the cart has not
moved": the max_correction_distance gate then rejects good NDT matches as soon
as the cart drives, holds the pose still for a second and jumps 2-3 m. Measured
2026-10-01: rejections at fitness 0.79, lock lost, pose left 17 m behind the
cart. The copy that used to live in golfcart_low_level went away when that
package was restored to its July source.

Two corrections the raw data needs:
  speed_scale   /speed_status over-reports by ~11 % (measured 2026-09-07 with
                scripts/calibrate_odom.py; re-measure after any change to
                ros_can_bridge's speed conversion).
  gyro_bias_z   /ouster/imu angular_velocity.z reads ~0.0118 rad/s (0.68 deg/s)
                at rest (re-measured 2026-10-01). Only the starting value: it
                is re-estimated whenever the cart is stationary.
"""

import fcntl
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

LOCK_PATH = '/tmp/odom_bridge.lock'
# Per-IMU-sample weight of the stationary bias estimate: ~2 s at 100 Hz.
BIAS_ALPHA = 0.005
# A sample this far from the estimate is the cart being rocked, not bias.
BIAS_MAX_STEP_RPS = 0.02


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        self.declare_parameter('speed_topic', '/speed_status')
        self.declare_parameter('imu_topic', '/ouster/imu')
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        # Faster than either input on purpose. Scan deskew refuses to run unless
        # the newest odom stamp reaches the end of the sweep, and a cloud lands
        # only ~10 ms after its sweep ends: at 50 Hz the newest sample is up to
        # 20 ms old, so most scans went un-deskewed.
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('max_speed_age_s', 0.5)
        self.declare_parameter('max_imu_age_s', 0.5)
        self.declare_parameter('speed_scale', 0.90)
        # ros_can_bridge occasionally reports 327.67 m/s (int16 full scale, the
        # board's "no reading" value). Passed through, one such sample moves
        # the dead-reckoned pose ~6 m: 7 of them in a 5-minute drive on
        # 2026-10-02 injected 41 m and threw the pose 12-18 m each time. The
        # cart tops out near 3.2 m/s, so anything past this is not a speed.
        self.declare_parameter('max_plausible_speed_mps', 8.0)
        self.declare_parameter('gyro_bias_z', 0.0118)
        self.declare_parameter('stationary_speed_mps', 0.02)
        self.declare_parameter('stationary_settle_s', 1.0)

        speed_topic = self.get_parameter('speed_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_speed_age_s = float(self.get_parameter('max_speed_age_s').value)
        self.max_imu_age_s = float(self.get_parameter('max_imu_age_s').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_plausible_speed_mps = float(self.get_parameter('max_plausible_speed_mps').value)
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)
        self.stationary_speed_mps = float(self.get_parameter('stationary_speed_mps').value)
        self.stationary_settle_s = float(self.get_parameter('stationary_settle_s').value)

        self.speed_mps = 0.0
        self.speed_stamp = None
        self.yaw_rate_rps = 0.0
        self.imu_stamp = None
        self.stationary_since = None
        self.rejected_speeds = 0

        self.create_subscription(Float64, speed_topic, self._cb_speed, 10)
        self.create_subscription(Imu, imu_topic, self._cb_imu, qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)

        self.create_timer(1.0 / publish_rate_hz, self._publish_odom)
        self.create_timer(5.0, self._check_inputs)

        self.get_logger().info(
            f'odom_bridge: {speed_topic} (speed x{self.speed_scale:.2f}) + '
            f'{imu_topic} (yaw rate) -> {self.odom_topic} @ {publish_rate_hz:.0f} Hz'
        )

    def _age_s(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _is_stationary(self):
        return (
            self.stationary_since is not None
            and self._age_s(self.stationary_since) >= self.stationary_settle_s
        )

    def _cb_speed(self, msg: Float64):
        raw = float(msg.data)
        # Keep the last good reading; a run of bad ones ages out via
        # max_speed_age_s like any other dropout. NaN fails the comparison too.
        if not abs(raw) <= self.max_plausible_speed_mps:
            self.rejected_speeds += 1
            self.get_logger().warn(
                f'Ignoring implausible /speed_status {raw:.2f} m/s '
                f'({self.rejected_speeds} so far).',
                throttle_duration_sec=10.0,
            )
            return
        self.speed_mps = raw * self.speed_scale
        self.speed_stamp = self.get_clock().now()
        if abs(self.speed_mps) > self.stationary_speed_mps:
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = self.speed_stamp

    def _cb_imu(self, msg: Imu):
        raw = float(msg.angular_velocity.z)
        self.imu_stamp = self.get_clock().now()
        if self._is_stationary() and abs(raw - self.gyro_bias_z) < BIAS_MAX_STEP_RPS:
            self.gyro_bias_z += BIAS_ALPHA * (raw - self.gyro_bias_z)
        self.yaw_rate_rps = raw - self.gyro_bias_z

    def _publish_odom(self):
        speed = self.speed_mps
        if self._age_s(self.speed_stamp) > self.max_speed_age_s:
            speed = 0.0

        yaw_rate = self.yaw_rate_rps
        # A parked cart cannot yaw, so whatever the gyro still reads is noise.
        if self._age_s(self.imu_stamp) > self.max_imu_age_s or self._is_stationary():
            yaw_rate = 0.0

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        self.pub_odom.publish(odom)

    def _check_inputs(self):
        stale = []
        if self._age_s(self.speed_stamp) > self.max_speed_age_s:
            stale.append('speed (publishing 0 m/s)')
        if self._age_s(self.imu_stamp) > self.max_imu_age_s:
            stale.append('IMU (publishing 0 rad/s)')
        if stale:
            self.get_logger().warn('No fresh ' + ' or '.join(stale) + ' input.')

        others = self.count_publishers(self.odom_topic) - 1
        if others > 0:
            self.get_logger().warn(
                f'{others} other publisher(s) on {self.odom_topic}; '
                'only one odometry source should be running.',
                throttle_duration_sec=60.0,
            )


def main(args=None):
    # One publisher per machine: a second copy would only interleave duplicate
    # samples into the same topic. It stands by instead of exiting, so stopping
    # the first copy never leaves /odometry without a source.
    lock = open(LOCK_PATH, 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print('odom_bridge is already running; standing by to take over.', file=sys.stderr)
        try:
            fcntl.flock(lock, fcntl.LOCK_EX)
        except KeyboardInterrupt:
            return

    rclpy.init(args=args)
    node = OdomBridge()
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
