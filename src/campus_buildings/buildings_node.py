#!/usr/bin/env python3
"""วาดตึกของเว็บ ModRide ใน RViz บนแผนที่ที่ localization โหลดอยู่ (MarkerArray /campus_buildings)

ข้อมูลมาจาก buildings_<ชื่อแผนที่>.json ที่ make_buildings.py สร้างไว้ข้างไฟล์นี้
lidar_localization.launch.py เปิด node นี้พร้อมส่ง map_path ของรอบนั้นมาให้
"""
import json
import os

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))

# โทนเดียวกับตึกในเว็บ (campus-map.js) แต่โปร่งให้เห็นจุดของแผนที่ข้างหลัง
ROOF = ColorRGBA(r=0.98, g=0.98, b=0.96, a=0.45)
WALL = ColorRGBA(r=0.77, g=0.78, b=0.78, a=0.25)
EDGE = ColorRGBA(r=0.92, g=0.93, b=0.90, a=0.9)
LABEL = ColorRGBA(r=0.95, g=0.55, b=0.26, a=1.0)


def signed_area(ring):
    return 0.5 * sum(
        x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]))


def triangulate(ring):
    """ear clipping ของ polygon ธรรมดา (ไม่มีรู) ที่เรียงทวนเข็มแล้ว คืนดัชนีสามเหลี่ยม"""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0

    left = list(range(len(ring)))
    triangles = []
    while len(left) > 3:
        for k in range(len(left)):
            i, j, n = left[k - 1], left[k], left[(k + 1) % len(left)]
            a, b, c = ring[i], ring[j], ring[n]
            if cross(a, b, c) <= 1e-9:
                continue
            if any(inside(ring[m], a, b, c) for m in left if m not in (i, j, n)):
                continue
            triangles.append((i, j, n))
            del left[k]
            break
        else:
            break   # รูปเพี้ยน (ขอบตัดกัน): หลังคาส่วนที่เหลือไม่วาด
    if len(left) == 3:
        triangles.append(tuple(left))
    return triangles


class CampusBuildings(Node):
    def __init__(self):
        super().__init__('campus_buildings')
        self.declare_parameter('map_path', '')
        self.declare_parameter('topic', '/campus_buildings')
        self.declare_parameter('publish_period_sec', 5.0)

        map_path = str(self.get_parameter('map_path').value)
        name = os.path.splitext(os.path.basename(map_path))[0]
        data_path = os.path.join(HERE, f'buildings_{name}.json')
        self.markers = MarkerArray()
        if not os.path.isfile(data_path):
            self.get_logger().warn(
                f'ไม่มี {data_path}: แผนที่ {name or "(ไม่ระบุ)"} ยังไม่ได้วางตึก '
                '(ต้องหาตำแหน่งแผนที่ใน map_frames.yaml แล้วรัน make_buildings.py)')
        else:
            with open(data_path, encoding='utf-8') as f:
                data = json.load(f)
            self.markers = self.build(data)
            self.get_logger().info(
                f'{len(data["buildings"])} ตึกบนแผนที่ {name} -> '
                f'{self.get_parameter("topic").value}')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, str(self.get_parameter('topic').value), qos)
        self.publish()
        # ส่งซ้ำเป็นระยะด้วย display ที่ตั้ง Durability เป็น Volatile จะได้เห็น
        self.create_timer(float(self.get_parameter('publish_period_sec').value), self.publish)

    def publish(self):
        if self.markers.markers:
            self.pub.publish(self.markers)

    def build(self, data):
        def marker(ns, index, kind, color):
            m = Marker()
            m.header.frame_id = data['frame_id']
            m.ns, m.id, m.type, m.action = ns, index, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color = color
            return m

        out = MarkerArray()
        for index, b in enumerate(data['buildings']):
            ring = [tuple(p) for p in b['ring']]
            if signed_area(ring) < 0:
                ring.reverse()
            low, high = b['base'], b['base'] + b['height']

            walls = marker('walls', index, Marker.TRIANGLE_LIST, WALL)
            edges = marker('edges', index, Marker.LINE_LIST, EDGE)
            edges.scale.x = 0.08
            for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
                a, b_, c, d = (Point(x=x1, y=y1, z=low), Point(x=x2, y=y2, z=low),
                               Point(x=x2, y=y2, z=high), Point(x=x1, y=y1, z=high))
                walls.points += [a, b_, c, a, c, d]
                edges.points += [d, c, a, b_, a, d]

            roof = marker('roofs', index, Marker.TRIANGLE_LIST, ROOF)
            for tri in triangulate(ring):
                roof.points += [Point(x=ring[i][0], y=ring[i][1], z=high) for i in tri]
            out.markers += [walls, roof, edges]

            if b.get('code'):
                label = marker('labels', index, Marker.TEXT_VIEW_FACING, LABEL)
                cx = sum(p[0] for p in ring) / len(ring)
                cy = sum(p[1] for p in ring) / len(ring)
                label.pose.position = Point(x=cx, y=cy, z=high + 2.0)
                label.scale.z = 3.0
                label.text = b['code']
                out.markers.append(label)
        return out


def main():
    rclpy.init()
    node = CampusBuildings()
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
