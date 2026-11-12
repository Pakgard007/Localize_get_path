#!/usr/bin/env python3
"""ปรับรูปตึกของเว็บ ModRide (มุม ด้าน โค้ง ส่วนเว้า) ให้ตรงกับผนังในแผนที่ PCD

    python3 -s fit_outlines.py FullGlobalMap            # ดูผลอย่างเดียว ไม่เขียนไฟล์
    python3 -s fit_outlines.py FullGlobalMap --write    # เขียนทับ campus-buildings.geojson

ทำทีละตึก ในพิกัดของแผนที่ (map_frames.yaml):
  1. ผนัง = คอลัมน์ 0.25 m ที่มีจุดกระจายสูงจากพื้น 0.8-12 m อย่างน้อย 2 m (เรือนยอดไม้ส่วนใหญ่ไม่ผ่าน)
  2. เลื่อน/หมุนทั้งหลังให้ขอบทับผนังก่อน
  3. ด้านยาวที่เห็นผนัง: fit เส้นตรง (หมุนได้ไม่เกิน 8 deg) ถ้าผนังด้านเดียวกันเหลื่อมเป็นขั้นก็แตกด้าน
     มุมใหม่ = จุดตัดของด้านที่ติดกัน ส่วนเว้า/นูนจึงตามผนังจริง
  4. ด้านสั้นที่ต่อกันเป็นโค้ง: ขยับแต่ละจุดตามแนวตั้งฉากเข้าหาผนัง
  5. ด้านที่แผนที่ไม่เห็นคงเส้นเดิม ผลที่ผิดรูป (ตัดกันเอง พื้นที่เปลี่ยนมาก) ใช้แค่ข้อ 2
ตึกที่แผนที่ไม่เห็นไม่แตะ ของเดิมจาก OSM เก็บไว้ที่ ~/ModRide/data/campus-buildings.osm.geojson
"""
import argparse
import json
import math
import os
import shutil
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_buildings as mb  # noqa: E402

GEOJSON = os.path.expanduser('~/ModRide/dist/campus-buildings.geojson')
ORIGINAL = os.path.expanduser('~/ModRide/data/campus-buildings.osm.geojson')
COLUMN_M = 0.25
BAND_M = 2.0            # wall points this far either side of an edge belong to it
LONG_EDGE_M = 2.5       # shorter edges are part of a curve or a small jog
MAX_TURN_DEG = 8.0      # an edge may turn this much to follow its wall
MIN_STEP_M = 0.8        # a wall that steps out at least this much splits its edge
MAX_MOVE_M = 5.0
SEEN_FRACTION = 0.3     # buildings whose outline the map sees less than this are left alone
MIN_GAIN = 0.05         # a new shape has to put this much more of its outline on walls


def wall_columns(points, centers, ground):
    plane, is_ground = mb.fit_ground_plane(centers, ground)
    _, nearest = cKDTree(centers[is_ground]).query(points[:, :2])
    h = points[:, 2] - ground[is_ground][nearest]
    keep = (h > 0.8) & (h < 12)
    col = np.floor(points[keep, :2] / COLUMN_M).astype(np.int64)
    key = col[:, 0] * 4_000_037 + col[:, 1]
    order = np.argsort(key, kind='stable')
    key, hz, col = key[order], h[keep][order], col[order]
    starts = np.r_[0, np.nonzero(np.diff(key))[0] + 1]
    ends = np.r_[starts[1:], len(key)]
    spread = np.maximum.reduceat(hz, starts) - np.minimum.reduceat(hz, starts)
    sel = (spread >= 2.0) & (ends - starts >= 5)
    return (col[starts[sel]] + 0.5) * COLUMN_M


def area(ring):
    return 0.5 * float(np.sum(ring[:, 0] * np.roll(ring[:, 1], -1) - np.roll(ring[:, 0], -1) * ring[:, 1]))


def simple(ring):
    """No two non-neighbouring edges cross."""
    n = len(ring)
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            c, d = ring[j], ring[(j + 1) % n]
            if (cross(a, b, c) * cross(a, b, d) < 0) and (cross(c, d, a) * cross(c, d, b) < 0):
                return False
    return True


def coverage(ring, tree, tol=0.5):
    closed = np.vstack([ring, ring[:1]])
    d, _ = tree.query(mb.ring_samples(closed, 0.25))
    return float((d < tol).mean())


def rigid_align(ring, walls, tree):
    """Move and turn the whole outline onto its walls (small steps only)."""
    closed = np.vstack([ring, ring[:1]])
    E = mb.ring_samples(closed, 0.25)
    R, t = np.eye(2), np.zeros(2)
    for gate in (2.0, 1.5, 1.0, 0.7, 0.5):
        Q = (R @ E.T).T + t
        d, i = tree.query(Q)
        m = d < gate
        if m.sum() < 30:
            break
        a, b = Q[m], walls[i[m]]
        ma, mb_ = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb_))
        dR = (U @ Vt).T
        if np.linalg.det(dR) < 0:
            Vt[1] *= -1
            dR = (U @ Vt).T
        R, t = dR @ R, dR @ t + mb_ - dR @ ma
    if abs(math.degrees(math.atan2(R[1, 0], R[0, 0]))) > 5 or np.linalg.norm(t) > MAX_MOVE_M:
        return ring
    moved = (R @ ring.T).T + t
    return moved if coverage(moved, tree) > coverage(ring, tree) else ring


def edge_points(a, b, walls, tree):
    L = float(np.linalg.norm(b - a))
    d = (b - a) / L
    n = np.array([d[1], -d[0]])                      # outward for a counter-clockwise ring
    idx = tree.query_ball_point((a + b) / 2, L / 2 + BAND_M + 0.5)
    if not idx:
        return L, d, n, np.empty(0), np.empty(0)
    w = walls[idx] - a
    u, v = w @ d, w @ n
    keep = (np.abs(v) <= BAND_M) & (u >= -0.5) & (u <= L + 0.5)
    return L, d, n, u[keep], v[keep]


def fit_line(u, v):
    """Robust v = c + s*u, |atan s| <= MAX_TURN_DEG."""
    c, s = float(np.median(v)), 0.0
    for _ in range(8):
        r = v - (c + s * u)
        w = np.where(np.abs(r) < 0.3, 1.0, 0.3 / np.maximum(np.abs(r), 1e-6))
        w[np.abs(r) > 1.2] = 0.0
        if w.sum() < 5:
            break
        A = np.c_[np.ones_like(u), u] * np.sqrt(w)[:, None]
        c, s = np.linalg.lstsq(A, v * np.sqrt(w), rcond=None)[0]
        lim = math.tan(math.radians(MAX_TURN_DEG))
        s = float(np.clip(s, -lim, lim))
    return float(c), float(s)


def steps(u, v, L):
    """Break points where the wall along one edge steps in or out: [(u0, u1, offset), ...]."""
    bins = np.arange(0, L, 1.0)
    med = []
    for b0 in bins:
        k = (u >= b0) & (u < b0 + 1.0)
        med.append(np.median(v[k]) if k.sum() >= 2 else np.nan)
    med = np.array(med)
    ok = ~np.isnan(med)
    if ok.sum() < 8:
        return None
    x, y = bins[ok] + 0.5, med[ok]
    base = np.abs(y - np.median(y)).sum()
    best, best_cost = None, base
    n = len(y)
    for i in range(3, n - 2):                        # one step: two runs of at least 3 m
        c1, c2 = np.median(y[:i]), np.median(y[i:])
        cost = np.abs(y[:i] - c1).sum() + np.abs(y[i:] - c2).sum() + 1.0
        if abs(c1 - c2) >= MIN_STEP_M and cost < best_cost:
            best, best_cost = [(0.0, (x[i - 1] + x[i]) / 2, c1), ((x[i - 1] + x[i]) / 2, L, c2)], cost
    for i in range(3, n - 5):                        # a notch: three runs
        for j in range(i + 3, n - 2):
            c1, c2, c3 = np.median(y[:i]), np.median(y[i:j]), np.median(y[j:])
            cost = (np.abs(y[:i] - c1).sum() + np.abs(y[i:j] - c2).sum()
                    + np.abs(y[j:] - c3).sum() + 2.0)
            if min(abs(c1 - c2), abs(c3 - c2)) >= MIN_STEP_M and cost < best_cost:
                ui, uj = (x[i - 1] + x[i]) / 2, (x[j - 1] + x[j]) / 2
                best, best_cost = [(0.0, ui, c1), (ui, uj, c2), (uj, L, c3)], cost
    return best if best is not None and best_cost < 0.5 * base else None


def intersect(p1, d1, p2, d2):
    A = np.array([d1, -d2]).T
    if abs(np.linalg.det(A)) < 1e-9:
        return None
    s = np.linalg.solve(A, p2 - p1)[0]
    return p1 + s * d1


def refit(ring, walls, tree):
    """Outline with each wall-backed edge on its wall; vertices from neighbouring lines."""
    n = len(ring)
    lines = []          # per original edge: list of (point, direction) lines, in order
    fitted = []
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        L, d, nrm, u, v = edge_points(a, b, walls, tree)
        occupied = len(np.unique(np.floor(u / 0.5))) if len(u) else 0
        if L < LONG_EDGE_M:
            lines.append(None)
            fitted.append(False)
            continue
        if len(u) < 8 or occupied < 0.35 * L / 0.5:
            lines.append([(a, d)])
            fitted.append(False)
            continue
        runs = steps(u, v, L) if L >= 8 else None
        if runs:
            seq = []
            for k, (u0, u1, off) in enumerate(runs):
                seq.append((a + nrm * off, d))
                if k + 1 < len(runs):                # the step itself, across the edge
                    seq.append((a + d * u1, nrm))
            lines.append(seq)
        else:
            c, s = fit_line(u, v)
            direction = d + nrm * s
            direction /= np.linalg.norm(direction)
            lines.append([(a + nrm * c, direction)])
        fitted.append(True)

    out = []
    for i in range(n):                               # vertex i joins edge i-1 and edge i
        prev, nxt = lines[i - 1], lines[i]
        v0 = ring[i]
        if prev is None and nxt is None:             # inside a curve
            out.append(curve_point(ring, i, walls, tree))
            continue
        if prev is None or nxt is None:
            p, d = (nxt[0] if prev is None else prev[-1])
            out.append(p + d * float((v0 - p) @ d))
            continue
        (p1, d1), (p2, d2) = prev[-1], nxt[0]
        angle = math.degrees(math.acos(min(1.0, abs(float(d1 @ d2)))))
        x = intersect(p1, d1, p2, d2) if angle > 20 else None
        if x is None:
            x = ((p1 + d1 * float((v0 - p1) @ d1)) + (p2 + d2 * float((v0 - p2) @ d2))) / 2
        out.append(x)
    # Insert the corners of stepped edges after their start vertex.
    result = []
    for i in range(n):
        result.append(out[i])
        seq = lines[i]
        if seq and len(seq) > 1:
            for (pa, da), (pb, db) in zip(seq[:-1], seq[1:]):
                x = intersect(pa, da, pb, db)
                if x is not None:
                    result.append(x)
    return np.array(result), sum(fitted)


def curve_point(ring, i, walls, tree):
    """A vertex between two short edges, moved along its normal onto the nearby wall."""
    a, v0, b = ring[i - 1], ring[i], ring[(i + 1) % len(ring)]
    t = b - a
    t = t / (np.linalg.norm(t) or 1.0)
    nrm = np.array([t[1], -t[0]])
    idx = tree.query_ball_point(v0, 1.5)
    if len(idx) < 3:
        return v0
    off = (walls[idx] - v0) @ nrm
    off = off[np.abs(off) < 1.5]
    return v0 + nrm * float(np.median(off)) if len(off) >= 3 else v0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('map', nargs='?', default='FullGlobalMap')
    parser.add_argument('--write', action='store_true')
    parser.add_argument('--report', default='')
    parser.add_argument('--output', default='', help='เขียนผลไปไฟล์นี้แทน (ไม่แตะไฟล์ของเว็บ)')
    args = parser.parse_args()

    with open(os.path.join(HERE, 'map_frames.yaml'), encoding='utf-8') as f:
        frames = yaml.safe_load(f)
    frame, origin = frames['maps'][args.map], frames['enu_origin']
    yaw = math.radians(frame['yaw_deg'])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t = np.array([frame['tx'], frame['ty']])
    to_enu = mb.enu_converter(origin)
    a, e2 = 6378137.0, 6.69437999014e-3
    lat0 = math.radians(origin['lat'])
    s0 = math.sin(lat0)
    east_per_rad = a / math.sqrt(1 - e2 * s0 * s0) * math.cos(lat0)
    north_per_rad = a * (1 - e2) / (1 - e2 * s0 * s0) ** 1.5

    def to_lonlat(m):
        enu = (R.T @ (m - t).T).T
        return np.c_[origin['lon'] + np.degrees(enu[:, 0] / east_per_rad),
                     origin['lat'] + np.degrees(enu[:, 1] / north_per_rad)]

    pcd = os.path.join(mb.WORKSPACE_SRC, f'{args.map}.pcd')
    print(f'อ่าน {pcd}', flush=True)
    points = mb.read_pcd_xyz(pcd)
    centers, ground = mb.ground_grid(points)
    walls = wall_columns(points, centers, ground)
    tree = cKDTree(walls)
    seen_tree = cKDTree(points[::4, :2])
    print(f'ผนัง {len(walls)} คอลัมน์', flush=True)

    # Shapes always start from OSM's; everything else (heights set by hand) from the current file.
    source = ORIGINAL if os.path.isfile(ORIGINAL) else GEOJSON
    with open(source, encoding='utf-8') as f:
        data = json.load(f)
    with open(GEOJSON, encoding='utf-8') as f:
        current = {feature['id']: feature['properties'] for feature in json.load(f)['features']}
    for feature in data['features']:
        if feature['id'] in current:
            feature['properties'] = dict(current[feature['id']])
            feature['properties'].pop('shapeSource', None)
    report = []
    for feature in data['features']:
        if feature['geometry']['type'] != 'Polygon':
            continue
        lonlat = np.array(feature['geometry']['coordinates'][0])
        ring = (R @ to_enu(lonlat).T).T + t
        if np.allclose(ring[0], ring[-1]):
            ring = ring[:-1]
        if area(ring) < 0:
            ring = ring[::-1]
        props = feature['properties']
        name = props.get('directoryId') or str(feature['id'])
        closed = np.vstack([ring, ring[:1]])
        seen = float((seen_tree.query(mb.ring_samples(closed, 0.5))[0] < 2.0).mean())
        before = coverage(ring, tree)
        if seen < SEEN_FRACTION:
            report.append((name, seen, before, before, 'ไม่เห็น', 0, 0.0))
            continue
        moved = rigid_align(ring, walls, tree)
        new, n_fit = refit(moved, walls, tree)
        reason = 'ปรับรูป'
        outline = cKDTree(mb.ring_samples(closed, 0.1))
        move = float(outline.query(new)[0].max())
        if (not simple(new) or not 0.7 < area(new) / area(ring) < 1.4 or move > MAX_MOVE_M
                or coverage(new, tree) < coverage(moved, tree)):
            new, reason = moved, 'เลื่อนทั้งหลัง'
        after = coverage(new, tree)
        if after < before + MIN_GAIN:
            report.append((name, seen, before, before, 'คงเดิม', 0, 0.0))
            continue
        shift = float(np.median(outline.query(new)[0])) if reason == 'ปรับรูป' else 0.0
        report.append((name, seen, before, after, reason, n_fit, shift))
        out = to_lonlat(np.vstack([new, new[:1]]))
        feature['geometry']['coordinates'] = [[[round(x, 7), round(y, 7)] for x, y in out]]
        props['shapeSource'] = f'lidar-fit {args.map}'

    print('%-8s %5s %6s %6s  %s' % ('ตึก', 'เห็น', 'ก่อน', 'หลัง', 'ผล'))
    for name, seen, b, a_, reason, n_fit, shift in sorted(report, key=lambda r: -(r[3] - r[2])):
        extra = f' ({n_fit} ด้านตามผนัง, จุดมุมขยับค่ากลาง {shift:.1f} m)' if reason == 'ปรับรูป' else ''
        print('%-8s %5.2f %5.0f%% %5.0f%%  %s%s' % (name, seen, 100 * b, 100 * a_, reason, extra))
    if args.report:
        with open(args.report, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False)
    if args.write:
        if not os.path.isfile(ORIGINAL):
            os.makedirs(os.path.dirname(ORIGINAL), exist_ok=True)
            shutil.copy2(GEOJSON, ORIGINAL)
        temp = GEOJSON + '.tmp'
        with open(temp, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False)
        os.replace(temp, GEOJSON)
        print(f'เขียน {GEOJSON} แล้ว (ของเดิมจาก OSM: {ORIGINAL})')


if __name__ == '__main__':
    main()
