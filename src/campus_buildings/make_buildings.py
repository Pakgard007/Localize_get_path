#!/usr/bin/env python3
"""ตึกของเว็บ ModRide -> พิกัด map ของแต่ละแผนที่ PCD ให้ buildings_node.py วาดใน RViz

    python3 -s make_buildings.py              # ทุกแผนที่ใน map_frames.yaml
    python3 -s make_buildings.py FullGlobalMap

รูปตึกกับความสูงเอามาจาก campus-buildings.geojson ของเว็บตรง ๆ (ความสูงเดียวกับที่เว็บใช้)
ตำแหน่งจาก map_frames.yaml ส่วนระดับพื้นของแต่ละหลังหาจากจุดพื้นใน PCD รอบตึก
ผลออกเป็น buildings_<แผนที่>.json ข้างไฟล์นี้ รันใหม่เมื่อ geojson หรือ map_frames.yaml เปลี่ยน
"""
import json
import math
import os
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_SRC = os.path.dirname(HERE)
GEOJSON = os.path.expanduser('~/ModRide/dist/campus-buildings.geojson')
GROUND_CELL_M = 2.0
GROUND_PERCENTILE = 3       # จุดต่ำสุดในช่องยังมีสัญญาณรบกวนใต้พื้นบ้าง
GROUND_RADIUS_M = 4.0       # ระดับพื้นของตึก = ช่องพื้นที่อยู่ในตึกหรือห่างขอบไม่เกินนี้
GROUND_PLANE_TOLERANCE_M = 0.4  # ช่องที่สูงจากระนาบพื้นเกินนี้คือยอดไม้/ผนัง ไม่ใช่พื้น
SEEN_WALL_RADIUS_M = 1.0    # จุดที่ห่างขอบตึกไม่เกินนี้นับเป็นผนังของตึกนั้น


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip()
            key, _, value = line.partition(' ')
            header[key] = value.split()
            if key == 'DATA':
                break
        if header['DATA'][0] != 'binary' or header['TYPE'] != ['F'] * len(header['FIELDS']):
            raise SystemExit(f'{path}: อ่านได้เฉพาะ PCD binary ที่ทุก field เป็น float32')
        fields = header['FIELDS']
        count = int(header['POINTS'][0])
        raw = np.frombuffer(f.read(count * 4 * len(fields)), dtype=np.float32)
    points = raw.reshape(count, len(fields))[:, [fields.index(axis) for axis in 'xyz']]
    return points[np.isfinite(points).all(axis=1)].astype(np.float64)


def enu_converter(origin):
    """lon/lat (deg) -> เมตร ตะวันออก/เหนือ บนระนาบสัมผัสที่ origin (WGS84)"""
    a, e2 = 6378137.0, 6.69437999014e-3
    lat0, lon0 = math.radians(origin['lat']), math.radians(origin['lon'])
    s = math.sin(lat0)
    prime = a / math.sqrt(1 - e2 * s * s)
    meridian = a * (1 - e2) / (1 - e2 * s * s) ** 1.5

    def convert(lonlat):
        lonlat = np.radians(np.asarray(lonlat, dtype=np.float64))
        return np.c_[(lonlat[:, 0] - lon0) * prime * math.cos(lat0),
                     (lonlat[:, 1] - lat0) * meridian]
    return convert


def ground_grid(points):
    """พื้นต่อช่อง GROUND_CELL_M: คืน (จุดกลางช่อง, ระดับพื้น)"""
    cells = np.floor(points[:, :2] / GROUND_CELL_M).astype(np.int64)
    keys = cells[:, 0] * 1_000_003 + cells[:, 1]
    order = np.argsort(keys, kind='stable')
    keys, z = keys[order], points[order, 2]
    unique, start = np.unique(keys, return_index=True)
    stop = np.r_[start[1:], len(keys)]
    ground = np.array([np.percentile(z[a:b], GROUND_PERCENTILE) for a, b in zip(start, stop)])
    first = cells[order][start]
    return (first + 0.5) * GROUND_CELL_M, ground


def fit_ground_plane(centers, ground):
    """ระนาบพื้นของทั้งแผนที่ (ตัดช่องที่เป็นยอดไม้/ผนังออกทีละรอบ) คืน (สัมประสิทธิ์, ช่องที่เป็นพื้น)

    ช่องริมแผนที่มักมีแต่ยอดไม้หรือผนัง จุดต่ำสุดในช่องจึงไม่ใช่พื้น ส่วนพื้นจริงของ
    ทั้งสองแผนที่ราบ (เอียง ~0.05%) ระนาบเดียวจึงพอใช้กับตึกนอกพื้นที่สแกน
    """
    keep = np.ones(len(ground), dtype=bool)
    for tolerance in (1.0, 1.0, 1.0) + (GROUND_PLANE_TOLERANCE_M,) * 5:
        design = np.c_[np.ones(keep.sum()), centers[keep]]
        coef, *_ = np.linalg.lstsq(design, ground[keep], rcond=None)
        keep = np.abs(ground - (coef[0] + centers @ coef[1:])) < tolerance
    return coef, keep


def ring_samples(ring, step=0.5):
    out = []
    for p, q in zip(ring[:-1], ring[1:]):
        n = max(1, int(np.linalg.norm(q - p) / step))
        out.append(p + (q - p) * (np.arange(n)[:, None] / n))
    return np.vstack(out)


def inside(ring, xy):
    """จุดใน polygon (ray casting) ring ปิดหัวท้ายแล้ว"""
    x, y = xy[:, 0], xy[:, 1]
    result = np.zeros(len(xy), dtype=bool)
    for (x1, y1), (x2, y2) in zip(ring[:-1], ring[1:]):
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            at = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        result ^= crosses & (x < at)
    return result


def make(name, frame, origin, features, pcd_path):
    print(f'{name}: อ่าน {pcd_path}', flush=True)
    points = read_pcd_xyz(pcd_path)
    centers, ground = ground_grid(points)
    plane, is_ground = fit_ground_plane(centers, ground)
    centers, ground = centers[is_ground], ground[is_ground]
    ground_tree = cKDTree(centers)
    to_enu = enu_converter(origin)
    yaw = math.radians(frame['yaw_deg'])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    offset = np.array([frame['tx'], frame['ty']])

    rings = []
    for feature in features:
        ring = (rotation @ to_enu(feature['geometry']['coordinates'][0]).T).T + offset
        rings.append(ring)

    # จุดสูงกว่าพื้นใกล้ขอบตึก: ใช้บอกว่าแผนที่เห็นผนังสูงถึงไหน (เทียบความสูงของโมเดล)
    edge_points = [ring_samples(ring) for ring in rings]
    edge_tree = cKDTree(np.vstack(edge_points))
    owner = np.concatenate([np.full(len(e), i) for i, e in enumerate(edge_points)])
    near, nearest = edge_tree.query(points[:, :2], distance_upper_bound=SEEN_WALL_RADIUS_M)
    wall = np.isfinite(near)
    wall_owner, wall_z = owner[nearest[wall]], points[wall, 2]

    buildings = []
    for i, (feature, ring) in enumerate(zip(features, rings)):
        props = feature['properties']
        lo, hi = ring.min(axis=0) - GROUND_RADIUS_M, ring.max(axis=0) + GROUND_RADIUS_M
        candidates = ground_tree.query_ball_point((lo + hi) / 2, np.linalg.norm(hi - lo) / 2)
        candidates = np.array(candidates, dtype=int)
        base, ground_source = None, 'none'
        if len(candidates):
            xy = centers[candidates]
            near_edge, _ = cKDTree(edge_points[i]).query(xy)
            keep = (near_edge <= GROUND_RADIUS_M) | inside(ring, xy)
            if keep.sum() >= 3:
                base, ground_source = float(np.median(ground[candidates[keep]])), 'around'
        if base is None:
            # ตึกนอกพื้นที่ที่สแกนไว้
            center = ring.mean(axis=0)
            base, ground_source = float(plane[0] + center @ plane[1:]), 'plane'
        seen = wall_z[wall_owner == i]
        seen_top = float(np.percentile(seen, 99) - base) if len(seen) >= 50 else None
        buildings.append({
            'id': feature['id'],
            'code': props.get('directoryId'),
            'name': props.get('name'),
            'ring': np.round(ring[:-1], 3).tolist(),
            'base': round(base, 3),
            'height': float(props['height']),
            'height_source': props.get('heightSource'),
            'ground_source': ground_source,
            'seen_wall_top': None if seen_top is None else round(seen_top, 2),
        })

    out = os.path.join(HERE, f'buildings_{name}.json')
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'map': name, 'frame_id': 'map', 'source': GEOJSON, 'buildings': buildings},
                  f, ensure_ascii=False, indent=1)
    taller = [b for b in buildings
              if b['seen_wall_top'] is not None and b['seen_wall_top'] > b['height']]
    print(f'{name}: {len(buildings)} ตึก -> {out}')
    for b in taller:
        print(f"  {b['code'] or b['id']}: แผนที่เห็นผนังสูง {b['seen_wall_top']:.1f} m "
              f"เกินโมเดล {b['height']:.0f} m ({b['height_source']})")


def main():
    with open(os.path.join(HERE, 'map_frames.yaml'), encoding='utf-8') as f:
        frames = yaml.safe_load(f)
    with open(GEOJSON, encoding='utf-8') as f:
        features = [
            feature for feature in json.load(f)['features']
            if feature['geometry']['type'] == 'Polygon'
        ]
    names = sys.argv[1:] or list(frames['maps'])
    for name in names:
        if name not in frames['maps']:
            raise SystemExit(f'{name}: ไม่มีใน map_frames.yaml')
        pcd = os.path.join(WORKSPACE_SRC, f'{name}.pcd')
        if not os.path.isfile(pcd):
            pcd = os.path.join(WORKSPACE_SRC, 'old_maps', f'{name}.pcd')   # maps no longer in use
        make(name, frames['maps'][name], frames['enu_origin'], features, pcd)


if __name__ == '__main__':
    main()
