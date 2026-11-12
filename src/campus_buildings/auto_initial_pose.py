#!/usr/bin/env python3
"""ตั้งตำแหน่งเริ่มของ localization เองตอนเปิด ไม่ต้องกด 2D Pose Estimate ใน RViz

ตอนเปิด รอสแกนแรกจาก lidar แล้วหาตำแหน่งที่ผนังในสแกนทับผนังในแผนที่มากที่สุด ไล่ตามลำดับ:
  1. ตำแหน่งที่บันทึกไว้ตอนใช้งานครั้งก่อน (~/.ros/cart_last_pose.json) ค้นรอบ ๆ ±1.5 m ±12 deg
  2. GPS (/ublox_gps_node/fix -> พิกัดแผนที่ด้วย map_frames.yaml) ค้นรอบจุดนั้นทุกทิศ
  3. ทั้งแผนที่ (FFT correlation ทุก 2 deg)
แล้วส่ง /initialpose ให้ localization และดู /match_inlier_ratio ว่าจับได้จริง ไม่ได้ก็ค้นกว้างขึ้น
ระหว่างใช้งาน บันทึกตำแหน่งลงไฟล์ทุก 2 s เมื่อ inlier ratio ผ่านเกณฑ์ ไว้ใช้ตอนเปิดครั้งหน้า
ถ้ากด 2D Pose Estimate เอง node นี้หยุดค้นทันที
lidar_localization.launch.py เปิด node นี้พร้อมส่ง map_path ของรอบนั้นมาให้
"""
import json
import math
import os
import sys
import tempfile
import threading
import time

import numpy as np
from scipy import fft as sfft
from scipy import ndimage
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_buildings as mb  # noqa: E402

CELL = 0.5                  # m, occupancy grid of the map and the scan
BAND = (1.0, 6.0)           # m above the ground: walls, poles, parked things; not the road
LAYER = 0.5                 # m, height layers of the grid inside BAND
N_LAYERS = int(round((BAND[1] - BAND[0]) / LAYER))
BASE_HEIGHT = 1.84          # base_link above the ground (cart_body_joint in golf_cart.urdf)
SCAN_RANGE = (1.5, 40.0)    # m from the lidar
MAX_FREE = 6000             # free-space samples kept per scan (a fixed random pick)
LOW_LAYERS = 3              # layers 1.0-2.5 m: the free space the whole-map search uses
FFT_WORKERS = 2             # threads for the whole-map search (the localization runs alongside)
CACHE_DIR = os.path.expanduser('~/.cache/cart_auto_pose')
STATE_FILE = os.path.expanduser('~/.ros/cart_last_pose.json')


# ======================================================================
#  map and scan on a 0.5 m grid in 0.5 m height layers
# ======================================================================
LIDAR_XY = (0.5, 0.0)       # the lidar in base_link (lidar_tf in lidar_localization.launch.py)
LIDAR_HEIGHT = BASE_HEIGHT  # os_lidar is level with base_link


def layer_of(h):
    """height above the ground -> layer index"""
    return np.clip(np.floor((h - BAND[0]) / LAYER), 0, N_LAYERS - 1)


class Scan:
    """One lidar scan: what stands 1-6 m above the ground, and the free space in front of it.

    points  (n, 2)  cells in 2D, for the whole-map FFT search and the ICP
    points3 (n, 3)  x, y, layer: a wall has to be at the same height in the map
    free    (n, 3)  x, y, layer: where the rays to those walls passed. This is what makes a pose
                    wrong when the scan is dropped into a thicket or a building. Each sample is
                    at the height its ray had there, so a ray under a roof edge or a tree crown
                    does not count against the pose.
    """

    def __init__(self, points_base):
        h = points_base[:, 2] + BASE_HEIGHT
        band = (h > BAND[0]) & (h < BAND[1])
        xy = points_base[band, :2]
        self.points = self._cells(xy)
        cells = np.unique(np.c_[np.floor(xy / CELL), layer_of(h[band])].astype(np.int64), axis=0)
        self.points3 = np.c_[cells[:, :2] * CELL + CELL / 2, cells[:, 2]]
        # a sample every 1 m along each ray, from 1 m out to 1 m short of what it hit
        origin = np.asarray(LIDAR_XY)
        d = self.points3[:, :2] - origin
        top = BAND[0] + (cells[:, 2] + 0.5) * LAYER
        r = np.hypot(d[:, 0], d[:, 1])
        d, top, r = d[r > 2.0], top[r > 2.0], r[r > 2.0]
        n = np.ceil(r - 2.0).astype(int)
        ray = np.repeat(np.arange(len(r)), n)
        f = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 1.0) / r[ray]
        h_ray = LIDAR_HEIGHT + (top[ray] - LIDAR_HEIGHT) * f
        free = np.unique(np.c_[np.floor((origin + d[ray] * f[:, None]) / CELL), layer_of(h_ray)].astype(np.int64),
                         axis=0)
        if len(free) > MAX_FREE:
            free = free[np.random.default_rng(0).choice(len(free), MAX_FREE, replace=False)]
        self.free = np.c_[free[:, :2] * CELL + CELL / 2, free[:, 2]]

    @staticmethod
    def _cells(xy):
        if not len(xy):
            return np.empty((0, 2))
        return np.unique(np.floor(xy / CELL).astype(np.int64), axis=0) * CELL + CELL / 2

    def __len__(self):
        return len(self.points)


class MapRaster:
    """Where the map has something standing 1-6 m above its ground: 0.5 m cells, 0.5 m layers."""

    def __init__(self, occupied, layers, origin, walls):
        self.occupied = ndimage.binary_dilation(occupied)   # 2D, for the FFT search
        self.layers = layers                                # bool (nx, ny, N_LAYERS): the free-space test
        # one cell and one layer of slack for the walls
        self.layers_hit = ndimage.binary_dilation(layers, structure=np.ones((3, 3, 3), dtype=bool))
        self.spectra = None                                 # of the map, made on the first whole-map search
        self.origin = np.asarray(origin, dtype=float)
        self.walls = walls                           # (n, 2) points for the final ICP
        self.tree = cKDTree(walls)

    @classmethod
    def from_pcd(cls, pcd_path):
        st = os.stat(pcd_path)
        key = '%s_%d_%d_v4' % (os.path.splitext(os.path.basename(pcd_path))[0], st.st_size, int(st.st_mtime))
        cache = os.path.join(CACHE_DIR, key + '.npz')
        if os.path.isfile(cache):
            data = np.load(cache)
            return cls(data['occupied'], data['layers'], data['origin'], data['walls'])
        points = mb.read_pcd_xyz(pcd_path)
        centers, ground = mb.ground_grid(points)
        _, is_ground = mb.fit_ground_plane(centers, ground)
        _, nearest = cKDTree(centers[is_ground]).query(points[:, :2])
        h = points[:, 2] - ground[is_ground][nearest]
        in_band = (h > BAND[0]) & (h < BAND[1])
        band = points[in_band, :2]
        origin = band.min(axis=0) - 5.0
        ij = np.floor((band - origin) / CELL).astype(int)
        occupied = np.zeros(ij.max(axis=0) + 6, dtype=bool)
        occupied[ij[:, 0], ij[:, 1]] = True
        layers = np.zeros(occupied.shape + (N_LAYERS,), dtype=bool)
        layers[ij[:, 0], ij[:, 1], layer_of(h[in_band]).astype(int)] = True
        walls = np.unique(np.floor(band / 0.25).astype(np.int64), axis=0) * 0.25 + 0.125
        os.makedirs(CACHE_DIR, exist_ok=True)
        fd, temp = tempfile.mkstemp(dir=CACHE_DIR, suffix='.npz')
        os.close(fd)
        np.savez_compressed(temp, occupied=occupied, layers=layers, origin=origin, walls=walls)
        os.replace(temp, cache)
        for old in os.listdir(CACHE_DIR):           # earlier versions of this map's cache
            if old.startswith(key.rsplit('_', 1)[0] + '_v') and old != key + '.npz':
                os.remove(os.path.join(CACHE_DIR, old))
        return cls(occupied, layers, origin, walls)

    def _lookup(self, grid, pts, translations, yaw):
        """grid value under pts (x, y, layer) rotated by yaw and moved by each translation -> (n_t, n_pts)"""
        c, s = math.cos(yaw), math.sin(yaw)
        rot = pts[:, :2] @ np.array([[c, s], [-s, c]])
        i = np.floor((rot[None, :, 0] + translations[:, None, 0] - self.origin[0]) / CELL).astype(int)
        j = np.floor((rot[None, :, 1] + translations[:, None, 1] - self.origin[1]) / CELL).astype(int)
        k = np.broadcast_to(pts[None, :, 2].astype(int), i.shape)
        nx, ny, _ = grid.shape
        ok = (i >= 0) & (j >= 0) & (i < nx) & (j < ny)
        out = np.zeros(i.shape, dtype=bool)
        out[ok] = grid[i[ok], j[ok], k[ok]]
        return out

    def _scores(self, scan, translations, yaw):
        hit = self._lookup(self.layers_hit, scan.points3, translations, yaw).mean(axis=1)
        if len(scan.free):
            hit = hit - self._lookup(self.layers, scan.free, translations, yaw).mean(axis=1)
        return hit

    def score(self, scan, poses):
        """(walls on map walls at their height) − (ray samples on map walls at theirs), for each (x, y, yaw)."""
        return np.array([self._scores(scan, np.array([[x, y]]), yaw)[0] for x, y, yaw in np.atleast_2d(poses)])

    def search_window(self, scan, centre, radius, step, yaws):
        """Best pose of each yaw on a square grid around centre -> [(x, y, yaw, score)]"""
        offs = np.arange(-radius, radius + 1e-9, step)
        gx, gy = np.meshgrid(offs + centre[0], offs + centre[1])
        txy = np.c_[gx.ravel(), gy.ravel()]
        results = []
        for yaw in yaws:
            best = None
            for a in range(0, len(txy), 128):
                sc = self._scores(scan, txy[a:a + 128], yaw)
                k = int(np.argmax(sc))
                if best is None or sc[k] > best[3]:
                    best = (txy[a + k, 0], txy[a + k, 1], yaw, float(sc[k]))
            results.append(best)
        return results

    def search_global(self, scan, yaw_step_deg=2.0, keep=30):
        """Whole map: FFT correlations of the scan with the map for each heading -> candidate places.

        Two per heading, the best `keep` places of each: walls on map walls, and the same minus
        the scan's low free space on low map walls. Walls alone favour places with many walls;
        with the free space subtracted, a place whose rays pass close to walls can drop out.
        Between the two, the true place was among the candidates for all 31 real scans and
        8 synthetic ones tried (2026-11-11); each alone missed some.
        """
        shape = tuple(self.occupied.shape)
        if self.spectra is None:
            low = self.layers[:, :, :LOW_LAYERS].any(axis=2)
            self.spectra = (sfft.rfft2(self.occupied.astype(np.float32), workers=FFT_WORKERS),
                            sfft.rfft2(low.astype(np.float32), workers=FFT_WORKERS))
        walls, low = self.spectra
        # the scan's own (0, 0) sits `lift` cells in, so all of it lands on non-negative cells;
        # an offset d of the correlation is then the cell of the cart minus lift, in [-lift, shape - lift)
        lift = int(math.ceil(SCAN_RANGE[1] / CELL)) + 2
        free = scan.free[scan.free[:, 2] < LOW_LAYERS, :2]
        weight = len(scan.points) / max(len(free), 1)       # both parts count as fractions
        places = []
        for deg in np.arange(0.0, 360.0, yaw_step_deg):
            yaw = math.radians(deg)
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, s], [-s, c]])

            def image(pts, value):
                ij = np.floor(pts @ rotation / CELL).astype(int) + lift
                ok = (ij >= 0).all(axis=1) & (ij < np.array(shape)).all(axis=1)
                img = np.zeros(shape, dtype=np.float32)
                img[ij[ok, 0], ij[ok, 1]] = value
                return sfft.rfft2(img, workers=FFT_WORKERS)

            on_walls = walls * np.conj(image(scan.points, 1.0))
            spectra = [on_walls]
            if len(free):
                spectra.append(on_walls - low * np.conj(image(free, weight)))
            for spectrum in spectra:
                cor = sfft.irfft2(spectrum, s=shape, workers=FFT_WORKERS)
                for k in np.argpartition(cor.ravel(), -keep)[-keep:]:
                    d = np.array(np.unravel_index(int(k), shape))
                    d = np.where(d >= np.array(shape) - lift, d - np.array(shape), d)   # undo the wrap
                    x, y = self.origin + (d + lift) * CELL
                    places.append((float(x), float(y), yaw))
        return places

    def refine(self, scan, pose, iterations=(1.0, 0.7, 0.5, 0.4, 0.3)):
        """2D ICP of the scan onto the map's wall points."""
        x, y, yaw = pose
        for gate in iterations:
            c, s = math.cos(yaw), math.sin(yaw)
            p = scan.points @ np.array([[c, s], [-s, c]]) + (x, y)
            d, i = self.tree.query(p, distance_upper_bound=gate)
            m = np.isfinite(d)
            if m.sum() < 30:
                break
            a, b = p[m], self.walls[i[m]]
            ma, mb_ = a.mean(axis=0), b.mean(axis=0)
            H = (a - ma).T @ (b - mb_)
            dtheta = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            cd, sd = math.cos(dtheta), math.sin(dtheta)
            R = np.array([[cd, -sd], [sd, cd]])
            t = mb_ - R @ ma
            x, y = R @ np.array([x, y]) + t
            yaw += dtheta
        return (float(x), float(y), float((yaw + math.pi) % (2 * math.pi) - math.pi))


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def distinct(candidates, apart_m=3.0, apart_rad=math.radians(20)):
    """Best first, one per place-and-heading."""
    out = []
    for sc, p in sorted(candidates, key=lambda c: -c[0]):
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > apart_m or angle_diff(p[2], q[2]) > apart_rad
               for _, q in out):
            out.append((sc, p))
    return out


def locate(raster, scan, priors, log=print, accept=0.5, accept_saved=0.5, accept_global=0.55,
           gps=None, gps_radius=10.0):
    """Search the priors in order -> poses to try, best first: [(pose, score, how)].

    priors: [(name, (x, y, yaw or None), radius)], name 'saved', 'gps' or 'global'.
    A saved pose has to fit well on its own, and lie near the GPS fix when there is one
    (the cart may have been moved while it was off). Places that fit almost as well as the
    best one go in the list too, so localization can reject the first and take the next.
    """
    for name, centre, radius in priors:
        t0 = time.monotonic()
        refine_count = 8
        if name == 'global':
            places = raster.search_global(scan)
            rough = []
            for yaw in sorted({p[2] for p in places}):       # scored together, per heading
                same = [p for p in places if p[2] == yaw]
                sc = raster._scores(scan, np.array([p[:2] for p in same]), yaw)
                rough += [(float(v), p) for v, p in zip(sc, same)]
            cands = distinct(rough)[:24]
            refine_count = 24
        elif centre[2] is None:            # heading unknown: coarse grid, ICP does the rest
            yaws = [math.radians(d) for d in np.arange(0, 360, 3.0)]
            res = raster.search_window(scan, centre[:2], min(radius, 10.0), 1.0, yaws)
            cands = [(r[3], r[:3]) for r in res]
        else:
            yaws = [centre[2] + math.radians(d) for d in np.arange(-12, 12.1, 1.0)]
            res = raster.search_window(scan, centre[:2], radius, 0.25, yaws)
            cands = [(r[3], r[:3]) for r in res]
        refined = []
        for _, p in distinct(cands)[:refine_count]:
            q = raster.refine(scan, p)
            refined.append((float(raster.score(scan, [q])[0]), q))
        ranked = distinct(refined)
        top_score, top = ranked[0]
        rival = '%.2f' % ranked[1][0] if len(ranked) > 1 else '-'
        log('%s: score %.2f (next other place %s) at x %.2f y %.2f yaw %.1f deg, %.1f s'
            % (name, top_score, rival, top[0], top[1], math.degrees(top[2]), time.monotonic() - t0))
        if name == 'saved':
            far = gps is not None and math.hypot(top[0] - gps[0], top[1] - gps[1]) > gps_radius
            if top_score >= accept_saved and not far:
                return [(top, top_score, name)]
            if far:
                log('saved: ห่างจาก GPS %.0f m -> รถอาจถูกย้ายตอนปิด' % math.hypot(top[0] - gps[0], top[1] - gps[1]))
            continue
        need = accept_global if name == 'global' else accept
        good = [(p, sc, name) for sc, p in ranked if sc >= need and sc >= top_score - 0.1][:3]
        if good:
            return good
    return []


def scan_to_2d(points_base):
    """Lidar points in base_link (n, 3) -> Scan."""
    return Scan(points_base)


# ======================================================================
#  ROS node
# ======================================================================
def main():
    import rclpy
    import yaml
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import NavSatFix, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Float64
    import tf2_ros

    class AutoInitialPose(Node):
        def __init__(self):
            super().__init__('auto_initial_pose')
            self.declare_parameter('map_path', '')
            self.declare_parameter('cloud_topic', '/ouster/points')
            self.declare_parameter('fix_topic', '/ublox_gps_node/fix')
            self.declare_parameter('accept_score', 0.5)
            self.declare_parameter('confirm_inlier', 0.6)     # = min_inlier_ratio of localization.yaml
            self.declare_parameter('save_period_sec', 2.0)
            self.map_path = str(self.get_parameter('map_path').value)
            self.map_name = os.path.splitext(os.path.basename(self.map_path))[0]
            self.accept = float(self.get_parameter('accept_score').value)
            self.confirm = float(self.get_parameter('confirm_inlier').value)
            self.frame = self.load_frame()
            self.raster = None
            self.cloud = None           # the latest scan message, turned into a Scan only for a search
            self.fix = None
            self.pose = None            # (time, x, y, z, yaw) of /pcl_pose
            self.inliers = []           # (time, ratio)
            self.state = 'loading'      # loading -> waiting -> searching -> checking -> done | manual
            self.sent = None
            self.tried = set()
            self.queue = []
            self.last_save = 0.0
            self.last_note = 0.0
            self.point_cloud2 = point_cloud2

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            # its own callback group: 10 Hz of big messages must not hold up the inlier ratio
            self.cloud_sub = self.create_subscription(PointCloud2, str(self.get_parameter('cloud_topic').value),
                                                      self.on_cloud, qos_profile_sensor_data,
                                                      callback_group=MutuallyExclusiveCallbackGroup())
            self.create_subscription(NavSatFix, str(self.get_parameter('fix_topic').value),
                                     self.on_fix, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', self.on_pose,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(Float64, '/match_inlier_ratio', self.on_inlier, 10)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
            self.create_timer(0.5, self.tick)
            threading.Thread(target=self.load_map, daemon=True).start()

        def load_frame(self):
            with open(os.path.join(HERE, 'map_frames.yaml'), encoding='utf-8') as f:
                frames = yaml.safe_load(f)
            frame = frames['maps'].get(self.map_name)
            if frame is None:
                return None
            return frame, frames['enu_origin']

        def load_map(self):
            t0 = time.monotonic()
            if not os.path.isfile(self.map_path):
                self.get_logger().error(f'ไม่พบแผนที่ {self.map_path}: ตั้งตำแหน่งเองไม่ได้')
                self.state = 'done'
                return
            self.get_logger().info(f'เตรียมแผนที่ {self.map_name} สำหรับหาตำแหน่งเริ่ม...')
            self.raster = MapRaster.from_pcd(self.map_path)
            self.get_logger().info(f'แผนที่พร้อม ({time.monotonic() - t0:.1f} s) รอสแกนจาก lidar')
            self.state = 'waiting'

        # ---------- inputs ----------
        def on_cloud(self, msg):
            if self.state in ('waiting', 'checking'):
                self.cloud = msg

        def make_scan(self, msg):
            """Scan message -> Scan in base_link, or None while TF is not up."""
            try:
                tf = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                q, t = tf.transform.rotation, tf.transform.translation
                R = np.array([
                    [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                    [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
                    [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)]])
                T = np.array([t.x, t.y, t.z])
            except Exception:
                return None
            # read_points, not read_points_numpy: Ouster clouds mix float32 and integer fields
            arr = self.point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float)
            r = np.linalg.norm(pts, axis=1)
            pts = pts[(r > SCAN_RANGE[0]) & (r < SCAN_RANGE[1])]
            return scan_to_2d(pts @ R.T + T)

        def on_fix(self, msg):
            if math.isfinite(msg.latitude) and msg.status.status >= 0:
                self.fix = (time.monotonic(), msg.latitude, msg.longitude, msg.status.status,
                            msg.position_covariance[0])

        def on_pose(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (time.monotonic(), p.x, p.y, p.z, yaw)

        def on_inlier(self, msg):
            now = time.monotonic()
            self.inliers = [(t, v) for t, v in self.inliers if now - t < 5.0] + [(now, float(msg.data))]

        def on_initialpose(self, msg):
            if self.sent is not None and time.monotonic() - self.sent[0] < 2.0:
                p = msg.pose.pose.position
                if math.hypot(p.x - self.sent[1], p.y - self.sent[2]) < 0.05:
                    return              # our own message
            if self.state not in ('done', 'manual'):
                self.get_logger().info('มีการตั้ง 2D Pose Estimate เอง -> หยุดหาตำแหน่งอัตโนมัติ')
            self.state = 'manual'

        # ---------- the steps ----------
        def localization_running(self, now):
            """The localization node is active: it publishes an inlier ratio for every scan it aligns."""
            return bool(self.inliers) and now - self.inliers[-1][0] < 3.0

        def tick(self):
            now = time.monotonic()
            self.save_pose(now)
            if self.state in ('done', 'manual') and self.cloud_sub is not None:
                self.destroy_subscription(self.cloud_sub)   # scans are only needed for the search
                self.cloud_sub = None
            if self.state == 'waiting' and self.cloud is not None:
                # a pose sent before the node is active (it loads the map for several seconds)
                # would be checked against nothing
                if not self.localization_running(now):
                    if now - self.last_note > 30.0:
                        self.last_note = now
                        self.get_logger().info('รอ localization เริ่มทำงาน (ยังไม่มี /match_inlier_ratio)')
                    return
                self.state = 'searching'
                threading.Thread(target=self.search, daemon=True).start()
            elif self.state == 'checking' and now - self.sent[0] > 6.0:
                recent = [v for t, v in self.inliers if t > self.sent[0] + 2.0]
                if len(recent) < 3 and now - self.sent[0] < 30.0:
                    return              # no verdict from the localization yet
                ratio = float(np.median(recent)) if recent else float('nan')
                if recent and ratio >= self.confirm:
                    self.get_logger().info(f'ตั้งตำแหน่งเริ่มเองสำเร็จ ({self.sent[3]}) inlier ratio {ratio:.2f}')
                    self.state = 'done'
                elif not recent:
                    self.get_logger().error('localization เงียบไป (ไม่มี /match_inlier_ratio 30 s): '
                                            'ตรวจขั้น Localization แล้วกด 2D Pose Estimate ใน RViz')
                    self.state = 'done'
                elif self.queue:
                    self.get_logger().warn(f'localization ไม่รับตำแหน่งนี้ (inlier {ratio:.2f}) -> ลองตำแหน่งถัดไป')
                    self.send(*self.queue.pop(0))
                elif self.priors():
                    self.get_logger().warn(f'localization ไม่รับตำแหน่งนี้ (inlier {ratio:.2f}) -> ค้นกว้างขึ้น')
                    self.state = 'waiting'
                else:
                    self.get_logger().error('หาตำแหน่งเริ่มเองไม่สำเร็จ: กด 2D Pose Estimate ใน RViz')
                    self.state = 'done'

        def gps_xy(self):
            if not (self.fix and self.frame and time.monotonic() - self.fix[0] < 10.0):
                return None, None
            radius = 3.0 if self.fix[3] >= 2 else min(max(3.0 * math.sqrt(max(self.fix[4], 0.0)), 5.0), 15.0)
            return self.gps_to_map(self.fix[1], self.fix[2]), radius

        def priors(self):
            out = []
            saved = self.load_saved()
            if saved and 'saved' not in self.tried:
                out.append(('saved', saved, 1.5))
            gps, radius = self.gps_xy()
            if gps and 'gps' not in self.tried:
                out.append(('gps', (gps[0], gps[1], None), radius))
            if 'global' not in self.tried:
                out.append(('global', None, None))
            return out

        def search(self):
            scan = self.make_scan(self.cloud)
            if scan is None or len(scan) < 150:
                if time.monotonic() - self.last_note > 30.0:
                    self.last_note = time.monotonic()
                    self.get_logger().warn('สแกนยังใช้ไม่ได้ (ไม่มี TF base_link หรือเห็นผนังน้อยเกินไป) รอสแกนถัดไป')
                self.state = 'waiting'
                return
            priors = self.priors()
            gps, radius = self.gps_xy()
            found = locate(self.raster, scan, priors, log=self.get_logger().info, accept=self.accept,
                           gps=gps, gps_radius=max(10.0, 3.0 * (radius or 0.0)))
            # the priors searched: all of them if nothing fits, else up to the one that did
            names = [name for name, _, _ in priors]
            self.tried.update(names if not found else names[:names.index(found[0][2]) + 1])
            if self.state == 'manual':
                return
            if not found:
                self.get_logger().error('หาตำแหน่งเริ่มเองไม่ได้ (ไม่มีที่ไหนตรงกับสแกนพอ): '
                                        'กด 2D Pose Estimate ใน RViz')
                self.state = 'done'
                return
            pose = found[0][0]
            current = self.pose
            if (current and time.monotonic() - current[0] < 1.0
                    and math.hypot(current[1] - pose[0], current[2] - pose[1]) < 0.5
                    and angle_diff(current[4], pose[2]) < math.radians(5)):
                self.get_logger().info('localization อยู่ตรงตำแหน่งที่หาได้อยู่แล้ว ไม่ต้องตั้งใหม่')
                self.state = 'done'
                return
            self.queue = list(found[1:])
            self.send(*found[0])

        def send(self, pose, score, how):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
            msg.pose.pose.position.z = self.pose[3] if self.pose else 0.0
            msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
            msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.25
            cov[35] = 0.03
            msg.pose.covariance = cov
            self.sent = (time.monotonic(), pose[0], pose[1], how)
            self.pub.publish(msg)
            self.get_logger().info(f'ตั้งตำแหน่งเริ่มจาก {how} (score {score:.2f}): x {pose[0]:.2f} '
                                   f'y {pose[1]:.2f} yaw {math.degrees(pose[2]):.1f} deg')
            self.state = 'checking'

        # ---------- remembering where the cart is ----------
        def load_saved(self):
            try:
                with open(STATE_FILE, encoding='utf-8') as f:
                    s = json.load(f)
                if s.get('map') == self.map_name:
                    return (float(s['x']), float(s['y']), float(s['yaw']))
            except (OSError, ValueError, KeyError):
                pass
            return None

        def save_pose(self, now):
            if self.state not in ('done', 'manual') or self.pose is None or now - self.last_save < float(
                    self.get_parameter('save_period_sec').value):
                return
            recent = [v for t, v in self.inliers if now - t < 2.0]
            if now - self.pose[0] > 1.0 or not recent or np.median(recent) < self.confirm:
                return
            self.last_save = now
            data = {'map': self.map_name, 'x': round(self.pose[1], 3), 'y': round(self.pose[2], 3),
                    'z': round(self.pose[3], 3), 'yaw': round(self.pose[4], 4),
                    'inlier': round(float(np.median(recent)), 3), 'saved': time.strftime('%Y-%m-%d %H:%M:%S')}
            try:
                os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
                fd, temp = tempfile.mkstemp(dir=os.path.dirname(STATE_FILE), suffix='.json')
                with os.fdopen(fd, 'w', encoding='utf-8') as f:
                    json.dump(data, f)
                os.replace(temp, STATE_FILE)
            except OSError:
                pass

        def gps_to_map(self, lat, lon):
            frame, origin = self.frame
            a, e2 = 6378137.0, 6.69437999014e-3
            lat0 = math.radians(origin['lat'])
            s = math.sin(lat0)
            east = math.radians(lon - origin['lon']) * a / math.sqrt(1 - e2 * s * s) * math.cos(lat0)
            north = math.radians(lat - origin['lat']) * a * (1 - e2) / (1 - e2 * s * s) ** 1.5
            yaw = math.radians(frame['yaw_deg'])
            return (math.cos(yaw) * east - math.sin(yaw) * north + frame['tx'],
                    math.sin(yaw) * east + math.cos(yaw) * north + frame['ty'])

    rclpy.init()
    node = AutoInitialPose()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
