#!/usr/bin/env python3
"""ส่งตำแหน่งรถไปหน้าเว็บ ModRide ให้รถกอล์ฟ 3D บนแผนที่ (iPad) อยู่ตรงกับรถใน RViz

ข้อความเดียว ตามที่ระบุใน ~/ModRide/VEHICLE-POSITION.md ส่งออกสามทาง เนื้อหาเหมือนกันทุกตัว:
  <web_dir>/vehicle.json   หน้าเว็บอ่านไฟล์นี้ (http.server หรือ scripts/serve.py ของ ModRide)
  /modride/vehicle         std_msgs/String ข้อความ JSON เดียวกัน ไว้ ros2 topic echo เทียบ
  /campus_buildings        ป้ายเหนือรถใน RViz (namespace "vehicle") บอกค่าที่เว็บได้รับ

/pcl_pose (พิกัด map ของแผนที่ที่โหลด) -> lat/lon ด้วย map_frames.yaml ชุดเดียวกับที่ใช้วางตึก
ถ้า pose เงียบเกิน pose_timeout_sec ใช้ /ublox_gps_node/fix แทน (ไม่มีทิศของรถ)
lidar_localization.launch.py เปิด node นี้พร้อมส่ง map_path ของรอบนั้นมาให้
"""
import json
import math
import os
import tempfile
from datetime import datetime, timezone

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
VEHICLE_ID = 'modride-01'          # the one vehicle the web page knows
VEHICLE_NAME = 'ModRide 01'         # what both RViz and the web page call it
LABEL_HEIGHT = 1.4                  # metres above base_link (the roof is ~0.1 above it)
SPEED_WINDOW_SEC = 1.0             # speed = distance over about this much pose history
CAMPUS = ((13.63, 13.67), (100.48, 100.51))   # anything else is a bad transform or fix


class WebBridge(Node):
    def __init__(self):
        super().__init__('modride_bridge')
        self.declare_parameter('map_path', '')
        self.declare_parameter('web_dir', os.path.expanduser('~/ModRide/dist'))
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('fix_topic', '/ublox_gps_node/fix')
        self.declare_parameter('pose_timeout_sec', 1.0)

        self.out = os.path.join(str(self.get_parameter('web_dir').value), 'vehicle.json')
        self.pose_timeout = float(self.get_parameter('pose_timeout_sec').value)
        self.frame = self.load_frame(str(self.get_parameter('map_path').value))
        self.map_name = os.path.splitext(os.path.basename(str(self.get_parameter('map_path').value)))[0]
        self.pose = None            # (receive time, x, y, z, yaw)
        self.history = []           # (receive time, x, y) for the speed
        self.fix = None             # (receive time, lat, lon, status)
        self.warned = set()

        self.create_subscription(
            PoseWithCovarianceStamped, str(self.get_parameter('pose_topic').value),
            self.on_pose, 10)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('fix_topic').value), self.on_fix,
            qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.publish)
        self.json_pub = self.create_publisher(String, '/modride/vehicle', 10)
        # The buildings display in RViz already listens here (transient local, as it requires).
        self.label_pub = self.create_publisher(
            MarkerArray, '/campus_buildings', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if not os.path.isdir(os.path.dirname(self.out)):
            self.get_logger().warn(f'ไม่มีโฟลเดอร์ {os.path.dirname(self.out)}: หน้าเว็บจะไม่เห็นรถ')

    def load_frame(self, map_path):
        name = os.path.splitext(os.path.basename(map_path))[0]
        with open(os.path.join(HERE, 'map_frames.yaml'), encoding='utf-8') as f:
            frames = yaml.safe_load(f)
        frame = frames['maps'].get(name)
        if frame is None:
            self.get_logger().warn(
                f'แผนที่ {name or "(ไม่ระบุ)"} ไม่มีใน map_frames.yaml: ใช้ได้แค่ตำแหน่งจาก GPS')
            return None
        origin = frames['enu_origin']
        a, e2 = 6378137.0, 6.69437999014e-3
        lat0 = math.radians(origin['lat'])
        s = math.sin(lat0)
        self.get_logger().info(f'ส่งตำแหน่งรถบนแผนที่ {name} -> {self.out}')
        return {
            'lat0': origin['lat'], 'lon0': origin['lon'],
            'east_per_rad': a / math.sqrt(1 - e2 * s * s) * math.cos(lat0),
            'north_per_rad': a * (1 - e2) / (1 - e2 * s * s) ** 1.5,
            'yaw': math.radians(frame['yaw_deg']), 'tx': frame['tx'], 'ty': frame['ty'],
        }

    def to_lat_lon(self, x, y):
        """map -> WGS84, the inverse of make_buildings.py (map = R(yaw) * ENU + t)."""
        f = self.frame
        dx, dy = x - f['tx'], y - f['ty']
        c, s = math.cos(f['yaw']), math.sin(f['yaw'])
        east, north = c * dx + s * dy, -s * dx + c * dy
        return (f['lat0'] + math.degrees(north / f['north_per_rad']),
                f['lon0'] + math.degrees(east / f['east_per_rad']))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = self.now()
        self.pose = (t, p.x, p.y, p.z, yaw)
        self.history = [h for h in self.history if t - h[0] <= SPEED_WINDOW_SEC] + [(t, p.x, p.y)]

    def on_fix(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and math.isfinite(msg.latitude):
            self.fix = (self.now(), msg.latitude, msg.longitude, msg.status.status)

    def publish(self):
        t = self.now()
        state = None
        where = None                # (x, y, z) in map for the RViz label
        if self.frame and self.pose and t - self.pose[0] <= self.pose_timeout:
            _, x, y, z, yaw = self.pose
            lat, lon = self.to_lat_lon(x, y)
            # map yaw -> ENU yaw -> compass bearing (0 = north, clockwise)
            heading = (90.0 - math.degrees(yaw - self.frame['yaw'])) % 360.0
            first = self.history[0]
            span = t - first[0]
            speed = math.hypot(x - first[1], y - first[2]) / span if span > 0.2 else 0.0
            state = {'latitude': lat, 'longitude': lon, 'fix': 'lidar',
                     'heading_deg': round(heading, 1), 'heading_valid': True,
                     'speed_mps': round(speed, 2),
                     'map': {'name': self.map_name, 'x': round(x, 2), 'y': round(y, 2),
                             'yaw_deg': round(math.degrees(yaw), 1)}}
            where = (x, y, z)
        elif self.fix and t - self.fix[0] <= self.pose_timeout:
            _, lat, lon, _status = self.fix
            state = {'latitude': lat, 'longitude': lon, 'fix': 'gnss',
                     'heading_deg': None, 'heading_valid': False, 'speed_mps': None, 'map': None}
        if state is None:
            return       # the page dims the cart once the file stops changing
        if not (CAMPUS[0][0] < state['latitude'] < CAMPUS[0][1]
                and CAMPUS[1][0] < state['longitude'] < CAMPUS[1][1]):
            self.warn_once('outside', f'ตำแหน่ง {state["latitude"]:.6f}, {state["longitude"]:.6f} '
                                      'อยู่นอก มจธ. ไม่ส่งให้หน้าเว็บ (map_frames.yaml ผิดแผนที่?)')
            return
        stamp = datetime.fromtimestamp(t, timezone.utc).isoformat(timespec='milliseconds')
        message = {'vehicle_id': VEHICLE_ID, 'timestamp': stamp.replace('+00:00', 'Z'),
                   'latitude': round(state['latitude'], 7), 'longitude': round(state['longitude'], 7),
                   'fix_valid': True, 'fix': state['fix'], 'h_acc_m': None,
                   'heading_deg': state['heading_deg'], 'heading_valid': state['heading_valid'],
                   'speed_mps': state['speed_mps'], 'map': state['map']}
        text = json.dumps(message, ensure_ascii=False)
        self.write(text)
        self.json_pub.publish(String(data=text))
        if where:
            self.publish_label(message, where)

    def publish_label(self, message, where):
        """The same numbers the web page shows on its vehicle card, above the cart in RViz."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'vehicle', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y = where[0], where[1]
        m.pose.position.z = where[2] + LABEL_HEIGHT
        m.pose.orientation.w = 1.0
        m.scale.z = 0.6
        m.color = ColorRGBA(r=1.0, g=0.62, b=0.3, a=1.0)
        m.text = '%s -> web\n%s\n(%.2f, %.2f) %s' % (
            VEHICLE_NAME, vehicle_text(message), message['map']['x'], message['map']['y'],
            message['map']['name'])
        self.label_pub.publish(MarkerArray(markers=[m]))

    def write(self, text):
        directory = os.path.dirname(self.out)
        try:
            fd, temp = tempfile.mkstemp(dir=directory, prefix='.vehicle-', suffix='.json')
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                f.write(text)
            os.chmod(temp, 0o644)
            os.replace(temp, self.out)
        except OSError as error:
            self.warn_once('write', f'เขียน {self.out} ไม่ได้: {error}')

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)


def vehicle_text(message):
    """One line, the same as the web page's vehicle card (cart.js vehicleText)."""
    text = '%.6f, %.6f' % (message['latitude'], message['longitude'])
    if message['heading_valid']:
        text += ' | %d deg' % round(message['heading_deg'])
    return text


def main():
    rclpy.init()
    node = WebBridge()
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
