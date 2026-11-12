#!/usr/bin/env python3
"""
Re-measure the two odom_bridge calibrations from a recorded rosbag.

    source /opt/ros/humble/setup.bash
    python3 scripts/calibrate_odom.py ~/ROsBAg/my_bag_20260811_223351 [more bags...]

Reports, per bag and pooled:

  speed_scale   true_speed / reported_speed, from GNSS Doppler
                (/ublox_gps_node/fix_velocity), cross-checked against NDT
                displacement over straight windows. This is the number to put
                in golfcart_low_level.yaml -> odom_bridge -> speed_scale.
                Both references are independent of each other: Doppler does not
                know about the map, NDT does not know about the satellites.

  gyro_bias_z   mean /ouster/imu angular_velocity.z while /speed_status is 0.
                odom_bridge estimates this online, so it is only a sanity check
                (and the value to expect the estimator to converge to).

The bag needs /speed_status and /ouster/imu for the bias, plus
/ublox_gps_node/fix_velocity (and optionally /pcl_pose) for the scale.

NOTE: /odometry is already scaled by odom_bridge, so on a bag recorded after a
speed_scale was set this reports the RESIDUAL factor. Multiply it by the
speed_scale that was in force during that recording to get the new value; a
correctly calibrated cart reports ~1.00 here.
"""

import bisect
import math
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WANTED = [
    '/pcl_pose',
    '/odometry',
    '/speed_status',
    '/ouster/imu',
    '/ublox_gps_node/fix_velocity',
]


def read(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    out = {t: [] for t in topics}
    classes = {t: get_message(types[t]) for t in present}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out[topic].append(deserialize_message(data, classes[topic]))
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def percentile(values, p):
    values = sorted(values)
    return values[int(p * (len(values) - 1))]


def analyse(path):
    data = read(path, WANTED)
    print('=' * 72)
    print(path.rstrip('/').split('/')[-1])

    speed_source = data['/odometry'] or None
    if speed_source:
        times = [stamp(m) for m in data['/odometry']]
        speeds = [m.twist.twist.linear.x for m in data['/odometry']]
        source_name = '/odometry.twist.linear.x'
    elif data['/speed_status']:
        # /speed_status carries no header; fall back to nothing useful.
        print('  only /speed_status present (no timestamps) - cannot pair with GNSS')
        return None
    else:
        print('  no odometry in this bag')
        return None

    pooled = []
    doppler = data['/ublox_gps_node/fix_velocity']
    if doppler:
        pairs = []
        for m in doppler:
            t = stamp(m)
            gnss_speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
            j = min(bisect.bisect_left(times, t), len(speeds) - 1)
            if speeds[j] > 0.3:
                pairs.append((speeds[j], gnss_speed))
        if pairs:
            sxx = sum(o * o for o, _ in pairs)
            sxy = sum(o * g for o, g in pairs)
            scale = sxy / sxx
            ratios = [g / o for o, g in pairs]
            print(f'  speed source: {source_name}  ({len(pairs)} moving samples paired with GNSS)')
            print(f'  speed_scale (GNSS Doppler, through-origin fit) = {scale:.4f}')
            print(f'    per-sample ratio  med={percentile(ratios, .5):.4f} '
                  f'p10={percentile(ratios, .1):.4f} p90={percentile(ratios, .9):.4f}')
            pooled = pairs

    # Cross-check against NDT over windows with no appreciable turning, where
    # the straight-line distance between poses equals the distance driven.
    poses = data['/pcl_pose']
    if poses:
        cumulative = [0.0]
        for i in range(1, len(times)):
            dt = times[i] - times[i - 1]
            cumulative.append(cumulative[-1] + (abs(speeds[i]) * dt if 0 < dt < 1 else 0))

        def odom_distance(t0, t1):
            i = min(bisect.bisect_left(times, t0), len(times) - 1)
            j = min(bisect.bisect_left(times, t1), len(times) - 1)
            return cumulative[j] - cumulative[i]

        pose_times = [stamp(m) for m in poses]
        ratios = []
        k = 0
        window = 3.0
        while k < len(poses) - 1:
            j = bisect.bisect_left(pose_times, pose_times[k] + window)
            if j >= len(poses):
                break
            y0 = yaw_of(poses[k].pose.pose.orientation)
            y1 = yaw_of(poses[j].pose.pose.orientation)
            turned = abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))
            a = poses[k].pose.pose.position
            b = poses[j].pose.pose.position
            travelled = odom_distance(pose_times[k], pose_times[j])
            if travelled > 1.0 and turned < math.radians(2.0):
                ratios.append(math.hypot(b.x - a.x, b.y - a.y) / travelled)
            k = j
        if ratios:
            print(f'  cross-check (NDT over {len(ratios)} straight {window:.0f}s windows) '
                  f'= {percentile(ratios, .5):.4f}')

    imu = data['/ouster/imu']
    status = data['/speed_status']
    if imu and status:
        # /speed_status has no header stamp, so pair on the odometry speed,
        # which is derived from it at 50 Hz.
        parked = []
        for m in imu:
            j = min(bisect.bisect_left(times, stamp(m)), len(speeds) - 1)
            if speeds[j] == 0.0:
                parked.append(m.angular_velocity.z)
        if len(parked) > 100:
            print(f'  gyro_bias_z (parked, n={len(parked)}) = '
                  f'{statistics.mean(parked):+.5f} rad/s '
                  f'({math.degrees(statistics.mean(parked)):+.3f} deg/s), '
                  f'std={statistics.pstdev(parked):.5f}')
        else:
            print(f'  gyro_bias_z: only {len(parked)} parked samples - '
                  f'record a bag with the cart standing still for ~30 s')
    return pooled


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    pooled = []
    for path in sys.argv[1:]:
        result = analyse(path)
        if result:
            pooled += result
    if len(sys.argv) > 2 and pooled:
        sxx = sum(o * o for o, _ in pooled)
        sxy = sum(o * g for o, g in pooled)
        print('=' * 72)
        print(f'POOLED over {len(pooled)} moving samples: speed_scale = {sxy / sxx:.4f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
