#!/usr/bin/env python3
"""The golf cart at its real size: COLLADA meshes and the URDF that hangs them on base_link.

    python3 cart_model.py     # reads cart.yaml, rewrites golf_cart.urdf and the two .dae beside it

robot_state_publisher (lidar_localization.launch.py) serves golf_cart.urdf, and the
RobotModel display in RViz draws it. The sizes and where base_link sits on the cart
come from cart.yaml; nothing here is read while the cart runs.

The cart is modelled in its own frame, the URDF link cart_body: origin on the ground
under the middle of the rear axle, x forward, y left, z up.
"""
import argparse
import math
import os

import numpy as np
import yaml

PACKAGE_URL = 'package://lidar_localization_ros2/urdf/'

# colour, lit, glow. RViz lights the scene from the camera only, with no ambient
# light: a face shows colour * (glow + lit * how squarely it faces the camera).
# The glow keeps a white roof seen at a shallow angle from going grey and the dark
# parts from sinking into a dark background; lit + glow stays near 1 so that the
# faces turned to the camera do not all burn out to the same white.
MATERIALS = {
    'body': ((0.95, 0.96, 0.97), 0.60, 0.40),
    'roof': ((0.96, 0.97, 0.97), 0.44, 0.56),
    'seat': ((0.24, 0.25, 0.28), 0.85, 0.25),
    'trim': ((0.15, 0.16, 0.18), 0.80, 0.35),
    'tire': ((0.085, 0.085, 0.095), 0.80, 0.40),
    'metal': ((0.78, 0.80, 0.83), 0.70, 0.25),
    'lamp': ((1.0, 0.95, 0.75), 0.15, 0.85),
    'tail': ((0.90, 0.13, 0.11), 0.35, 0.65),
}
# A link has one transparency in RViz, whatever its mesh says, so the windscreen
# is a link of its own, coloured by the URDF.
GLASS = (0.66, 0.84, 0.95, 0.22)


def _outward(tris):
    """A closed surface whose triangles all wind the same way, facing out."""
    volume = np.einsum('ij,ij->i', tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()
    return tris if volume > 0 else tris[:, ::-1]


def _loft(rings):
    """The surface through rings of equal point count, capped at both ends."""
    rings = np.asarray(rings, dtype=float)
    a, b = rings[:-1], rings[1:]
    a2, b2 = np.roll(a, -1, axis=1), np.roll(b, -1, axis=1)
    parts = [np.stack([a, a2, b2], axis=2).reshape(-1, 3, 3),
             np.stack([a, b2, b], axis=2).reshape(-1, 3, 3)]
    for ring, first in ((rings[0], True), (rings[-1], False)):
        fan = np.stack(
            [np.broadcast_to(ring.mean(axis=0), ring.shape), ring, np.roll(ring, -1, axis=0)],
            axis=1)
        parts.append(fan[:, ::-1] if first else fan)
    return _outward(np.concatenate(parts))


def _section(hv, w0, w1, r_top, r_bottom, n):
    """Outline (v, w) of the rectangle |v| <= hv, w0 <= w <= w1 with rounded corners."""
    limit = min(hv, (w1 - w0) / 2)
    rt, rb = (max(min(r, limit), 1e-4) for r in (r_top, r_bottom))
    corners = ((hv - rt, w1 - rt, rt, 0.0), (rt - hv, w1 - rt, rt, 0.5),
               (rb - hv, w0 + rb, rb, 1.0), (hv - rb, w0 + rb, rb, 1.5))
    arcs = []
    for cv, cw, r, start in corners:
        t = (start + np.linspace(0.0, 0.5, n + 1)) * math.pi
        arcs.append(np.stack([cv + r * np.cos(t), cw + r * np.sin(t)], axis=1))
    return np.concatenate(arcs)


def _place(axis, a, v, w):
    """Points of a cross-section (v, w) at station a along the axis."""
    a = np.full_like(v, a)
    return np.stack({'x': (a, v, w), 'y': (v, a, w), 'z': (w, v, a)}[axis], axis=1)


def _block(axis, a0, a1, hv, w0, w1, r=0.03, r_end=None, n=3, v=0.0):
    """A box with rounded edges: the cross-section (v, w) swept from a0 to a1.

    For axis x the section is (y, z), for y it is (x, z), for z it is (y, x);
    r rounds the edges along the axis, r_end the ones around its two ends.
    """
    r_end = r if r_end is None else r_end
    r_end = min(r_end, (a1 - a0) / 2, hv, (w1 - w0) / 2)
    if r_end > 1e-4:
        t = np.linspace(0.0, math.pi / 2, n + 1)
        along = np.concatenate([a0 + r_end * (1 - np.cos(t)), a1 - r_end * (1 - np.cos(t[::-1]))])
        inset = np.concatenate([r_end * (1 - np.sin(t)), r_end * (1 - np.sin(t[::-1]))])
    else:
        along, inset = np.array([a0, a1]), np.zeros(2)
    rings = []
    for a, d in zip(along, inset):
        ring = _section(hv - d, w0 + d, w1 - d, r - d, r - d, n)
        rings.append(_place(axis, a, ring[:, 0] + v, ring[:, 1]))
    return _loft(rings)


def _panel(xs, hw, z0, z1, r_top, r_bottom, n=4):
    """A body panel swept along x: one half-width, underside, top and pair of
    corner radii per station."""
    return _loft([
        _place('x', x, *_section(*args, n).T)
        for x, *args in zip(xs, hw, z0, z1, r_top, r_bottom)])


def _fillet(xs, end, r, inward):
    """How far a rounded end pulls a panel in at each station; inward is +1 for
    the end the panel starts at, -1 for the one it stops at."""
    s = np.clip(inward * (xs - end) / r, 0.0, 1.0)
    return r * (1 - np.sqrt(1 - (1 - s) ** 2))


def _stations(x0, x1, step, *extra):
    """Stations from x0 to x1 no further apart than step, plus the extra ones
    that are not right next to another."""
    xs = np.unique(np.round(np.clip(np.concatenate(
        [np.linspace(x0, x1, int(math.ceil((x1 - x0) / step)) + 1), *extra]), x0, x1), 5))
    kept = [xs[0]]
    for x in xs[1:-1]:
        if x - kept[-1] >= 0.012 and xs[-1] - x >= 0.012:
            kept.append(x)
    return np.array(kept + [xs[-1]])


def _basis(u):
    u = np.asarray(u, dtype=float)
    u = u / np.linalg.norm(u)
    e = np.cross(u, (0.0, 0.0, 1.0))
    if np.linalg.norm(e) < 1e-6:
        e = np.cross(u, (1.0, 0.0, 0.0))
    e = e / np.linalg.norm(e)
    return u, e, np.cross(u, e)


def _tube(p0, p1, radius, n=12):
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    _, e, f = _basis(p1 - p0)
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    circle = radius * (np.outer(np.cos(t), e) + np.outer(np.sin(t), f))
    return _loft([p0 + circle, p1 + circle])


def _revolve(profile, center, axis, n=32):
    """A closed outline of (radius, along-axis) points turned around the axis."""
    u, e, f = _basis(axis)
    rings = np.array([
        np.asarray(center, dtype=float) +
        np.outer(profile[:, 0], math.cos(t) * e + math.sin(t) * f) + np.outer(profile[:, 1], u)
        for t in np.linspace(0.0, 2 * math.pi, n, endpoint=False)])
    a, b = rings, np.roll(rings, -1, axis=0)
    a2, b2 = np.roll(a, -1, axis=1), np.roll(b, -1, axis=1)
    return _outward(np.concatenate([
        np.stack([a, a2, b2], axis=2).reshape(-1, 3, 3),
        np.stack([a, b2, b], axis=2).reshape(-1, 3, 3)]))


def _tilt(tris, degrees, pivot):
    """Turned about the y axis through the pivot; positive leans the top forward."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    rotation = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    pivot = np.asarray(pivot, dtype=float)
    return (tris - pivot) @ rotation.T + pivot


def build_parts(dims):
    """({material: [triangles]}, [windscreen triangles]) of the cart in its own frame."""
    wb = float(dims['wheelbase'])
    nose = wb + float(dims['front_overhang'])
    tail = nose - float(dims['length'])
    tire_r = float(dims['tire_diameter']) / 2
    tire_w = float(dims['tire_width'])
    wheel_y = float(dims['width']) / 2 - tire_w / 2
    hb = wheel_y + tire_w / 2 - 0.06          # body half-width: the tyres stand a little proud
    inner = wheel_y - tire_w / 2 - 0.035      # half-width of what sits between the wheels
    lidar_x = float(dims['lidar']['ahead_of_rear_axle'])
    lidar_z = float(dims['lidar']['height'])
    sensor_base = lidar_z - 0.038             # os_lidar is 38 mm above the foot of an OS1
    roof_top = sensor_base - 0.006            # the sensor's plate lies on it, a little proud
    roof_bottom = roof_top - 0.05
    roof_front = float(dims['roof_front'])
    roof_rear = roof_front - float(dims['roof_length'])
    sill = 0.20                               # lower edge of the body panels
    fender = 2 * tire_r + 0.045               # top of the wheel arches
    arch = fender - sill                      # their radius, about a point on the sill line
    deck = fender + 0.16                      # top of the body tub the seats sit on
    tub_rear, tub_front = tail + 0.23, 0.62
    dash, cap = wb - 0.50, nose - 0.03        # the cowl runs from the dashboard to its flat nose

    parts = {name: [] for name in MATERIALS}

    # Wheels: tyre, rim and hub cap, on axles.
    tyre = _section(tire_w / 2, tire_r * 0.47, tire_r, 0.05, 0.05, 4)[:, ::-1]
    for x in (0.0, wb):
        parts['trim'].append(_tube((x, -wheel_y, tire_r), (x, wheel_y, tire_r), 0.028, 10))
        for side in (1.0, -1.0):
            y = side * wheel_y
            parts['tire'].append(_revolve(tyre, (x, y, tire_r), (0, 1, 0)))
            parts['metal'].append(_tube((x, y - 0.075, tire_r), (x, y + 0.075, tire_r), tire_r * 0.48, 20))
            parts['trim'].append(
                _tube((x, y + side * 0.07, tire_r), (x, y + side * 0.092, tire_r), tire_r * 0.2, 12))

    def underside(xs):
        """The lower edge of the body panels: the sill, arched over each wheel."""
        z = np.full_like(xs, sill)
        for axle in (0.0, wb):
            z = np.maximum(z, sill + np.sqrt(np.maximum(arch ** 2 - (xs - axle) ** 2, 0.0)))
        return z

    def over_wheel(axle):
        return axle - arch * np.cos(np.linspace(0.0, math.pi, 25))

    # Body tub under the seats.
    xs = _stations(tub_rear, tub_front, 0.06, over_wheel(0.0),
                   tub_rear + 0.05 * (1 - np.cos(np.linspace(0.0, math.pi / 2, 5))),
                   tub_front - 0.05 * (1 - np.cos(np.linspace(0.0, math.pi / 2, 5))))
    d = _fillet(xs, tub_rear, 0.05, 1.0) + _fillet(xs, tub_front, 0.05, -1.0)
    parts['body'].append(_panel(xs, hb - d, underside(xs) + 0.5 * d, deck - d, 0.05 - d, 0.025 - d))

    # Cowl: full width over the front wheels, sloping down to a rounded nose.
    nose_r = 0.22
    xs = _stations(dash, cap, 0.05, over_wheel(wb),
                   dash + 0.03 * (1 - np.cos(np.linspace(0.0, math.pi / 2, 4))),
                   cap - nose_r * (1 - np.sin(np.linspace(0.0, math.pi / 2, 14))))
    t = (xs - dash) / (cap - dash)
    d = _fillet(xs, dash, 0.03, 1.0)
    round_nose = nose_r * (1 - np.sqrt(1 - np.clip((xs - (cap - nose_r)) / nose_r, 0.0, 1.0) ** 2))
    parts['body'].append(_panel(
        xs, hb - 0.05 * t - 0.9 * round_nose - d, underside(xs) + 0.1 * round_nose + 0.5 * d,
        deck + 0.27 - 0.25 * t ** 1.5 - 0.35 * round_nose - d, 0.10 - d, 0.03 - d))

    # Floor between them, and the dark frame the body sits on.
    parts['body'].append(_block('x', tub_front - 0.04, dash + 0.04, hb, sill, 0.31, 0.03))
    parts['trim'] += [
        _block('x', tail + 0.30, tub_front - 0.02, inner, 0.13, fender - 0.03, 0.04),
        _block('x', tub_front - 0.07, dash + 0.08, inner, 0.13, 0.24, 0.04),
        _block('x', dash + 0.03, nose - 0.09, inner, 0.13, fender - 0.03, 0.04),
        _block('x', nose - 0.11, nose, inner + 0.07, 0.24, 0.35, 0.04),             # front bumper
        _block('x', nose - 0.07, cap + 0.006, 0.30, 0.425, 0.555, 0.02),            # lamp panel
        _block('x', tub_front - 0.02, dash + 0.03, hb - 0.07, 0.31, 0.322, 0.005),  # floor mat
        _block('x', tail, tail + 0.28, hb - 0.09, 0.30, 0.34, 0.015),               # rear footrest
        _block('x', dash - 0.07, dash + 0.03, hb - 0.03, deck + 0.10, deck + 0.255, 0.03),  # dashboard
    ]
    for side in (1.0, -1.0):
        parts['lamp'].append(_block(
            'x', nose - 0.05, cap + 0.016, 0.075, 0.452, 0.528, 0.015, v=side * 0.195))
        parts['tail'].append(_block(
            'x', tub_rear - 0.012, tub_rear + 0.02, 0.06, fender + 0.04, fender + 0.12, 0.012,
            v=side * (hb - 0.13)))
        parts['trim'].append(_tube(                                                 # footrest stay
            (tail + 0.02, side * (hb - 0.12), 0.32), (tail + 0.32, side * (inner - 0.03), 0.30), 0.014, 8))

    # Seats: a bench facing forward and one facing back, sharing the middle.
    parts['seat'] += [
        _block('x', 0.07, 0.60, hb - 0.04, deck, deck + 0.12, 0.045),
        _tilt(_block('x', 0.02, 0.11, hb - 0.04, deck + 0.14, deck + 0.56, 0.04), -8, (0.065, 0, deck + 0.14)),
        _block('x', tail + 0.26, -0.11, hb - 0.04, deck, deck + 0.12, 0.045),
        _tilt(_block('x', -0.19, -0.10, hb - 0.04, deck + 0.14, deck + 0.52, 0.04), 8, (-0.145, 0, deck + 0.14)),
    ]
    for side in (1.0, -1.0):                                                        # hip rails
        y = side * (hb - 0.02)
        for x0, x1 in ((0.10, 0.52), (tail + 0.30, -0.14)):
            parts['trim'] += [
                _tube((x0, y, deck + 0.02), (x0, y, deck + 0.24), 0.012, 8),
                _tube((x0, y, deck + 0.24), (x1, y, deck + 0.24), 0.012, 8),
                _tube((x1, y, deck + 0.24), (x1, y, deck + 0.02), 0.012, 8),
            ]

    # Steering wheel, on the left.
    wheel_at = np.array((wb - 0.75, 0.28, deck + 0.34))
    column = np.array((wb - 0.54, 0.28, deck + 0.20))
    spin = wheel_at - column
    t = np.linspace(0.0, 2 * math.pi, 8, endpoint=False)
    rim = np.stack([0.17 + 0.015 * np.cos(t), 0.015 * np.sin(t)], axis=1)
    parts['trim'] += [
        _tube(column, wheel_at, 0.018, 10),
        _revolve(rim, wheel_at, spin, 28),
        _tube(wheel_at - 0.02 * spin / np.linalg.norm(spin), wheel_at, 0.045, 12),
    ]
    _, e, f = _basis(spin)
    for direction in (e, -0.5 * e + 0.87 * f, -0.5 * e - 0.87 * f):
        parts['trim'].append(_tube(wheel_at, wheel_at + 0.17 * direction, 0.009, 6))

    # Roof on four struts, with a windscreen between the front pair.
    strut_foot = np.array((dash + 0.01, hb - 0.05, deck + 0.24))
    strut_top = np.array((dash - 0.10, hb - 0.07, roof_bottom + 0.01))
    rear_foot = np.array((tail + 0.29, hb - 0.04, deck))
    rear_top = np.array((max(roof_rear + 0.10, tail + 0.23), hb - 0.07, roof_bottom + 0.01))
    mirror = np.array((1.0, -1.0, 1.0))
    for flip in (np.ones(3), mirror):
        parts['trim'] += [
            _tube(strut_foot * flip, strut_top * flip, 0.017, 10),
            _tube(rear_foot * flip, rear_top * flip, 0.017, 10),
        ]
    for share in (0.0, 0.55, 1.0):                                                  # windscreen frame
        bar = strut_foot + share * (strut_top - strut_foot)
        parts['trim'].append(_tube(bar, bar * mirror, 0.011, 8))
    lean = math.degrees(math.atan2(strut_foot[0] - strut_top[0], strut_top[2] - strut_foot[2]))
    height = float(np.linalg.norm(strut_top - strut_foot)) - 0.04
    foot = (strut_foot[0], 0.0, strut_foot[2] + 0.02)
    glass = _tilt(_block(
        'x', foot[0] - 0.004, foot[0] + 0.004, hb - 0.08, foot[2], foot[2] + height, 0.003), -lean, foot)
    parts['roof'].append(_block('z', roof_bottom, roof_top, hb - 0.02, roof_rear, roof_front, 0.16, 0.02, 6))

    # The lidar where it really is, on a plate off the front edge of the roof.
    parts['trim'] += [
        _block('x', min(roof_front - 0.10, lidar_x - 0.07), lidar_x + 0.07, 0.07, sensor_base - 0.014, sensor_base, 0.005),
        _tube((lidar_x, 0, sensor_base + 0.018), (lidar_x, 0, sensor_base + 0.056), 0.040, 24),
    ]
    parts['metal'] += [
        _tube((lidar_x, 0, sensor_base), (lidar_x, 0, sensor_base + 0.018), 0.0435, 24),
        _tube((lidar_x, 0, sensor_base + 0.056), (lidar_x, 0, sensor_base + 0.0735), 0.0435, 24),
    ]

    # The RTK antenna on its pole: a foot plate, the pole, and the puck on top.
    mount = rtk_mount(dims)
    if mount is not None:
        base, length = mount
        top = base + np.array((0.0, 0.0, length))
        disc = lambda r, h: np.array(((0.0, 0.0), (r, 0.0), (r, h), (0.0, h)))  # noqa: E731
        parts['trim'] += [
            _revolve(disc(0.045, 0.012), base, (0, 0, 1), 24),
            _revolve(disc(0.065, 0.022), top, (0, 0, 1), 32),
        ]
        parts['metal'].append(_tube(base, top, 0.013, 12))
    return parts, [glass]


def rtk_mount(dims):
    """(foot of the RTK pole in the cart's frame, pole length), or None without an rtk entry."""
    rtk = dims.get('rtk')
    if not rtk:
        return None
    base = np.array((float(rtk.get('ahead_of_rear_axle', 0.0)), float(rtk.get('left_of_center', 0.0)),
                     float(rtk['base_height'])))
    return base, float(rtk['pole_length'])


def cart_in_base_link(dims):
    """(xyz, rpy) of the cart's frame in base_link, as a URDF origin."""
    link, lidar = dims['base_link'], dims['lidar']
    roll, pitch = math.radians(float(link['roll_deg'])), math.radians(float(link['pitch_deg']))

    def rot(axis, angle):
        c, s = math.cos(angle), math.sin(angle)
        i, j = {'x': (1, 2), 'y': (2, 0)}[axis]
        m = np.eye(3)
        m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
        return m

    # A level cart heading the way base_link does, seen from a base_link that is
    # itself rolled and pitched with the lidar's mount.
    rotation = rot('x', -roll) @ rot('y', -pitch)
    lidar_on_cart = np.array((float(lidar['ahead_of_rear_axle']), 0.0, float(lidar['height'])))
    position = np.array((float(link['behind_lidar']), 0.0, 0.0)) - rotation @ lidar_on_cart
    rpy = (math.atan2(rotation[2, 1], rotation[2, 2]), -math.asin(rotation[2, 0]),
           math.atan2(rotation[1, 0], rotation[0, 0]))
    return position, rpy


def _smooth_normals(tris, crease_degrees=50.0):
    """A normal for every corner: its faces averaged, except across sharp edges."""
    edges = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    area = np.linalg.norm(edges, axis=1)
    tris, edges, area = tris[area > 1e-12], edges[area > 1e-12], area[area > 1e-12]
    face = edges / area[:, None]
    # Each face counts by the angle it fills at the corner, so a narrow strip
    # next to a wide one bends the surface as much as the wide one does.
    side = np.roll(tris, -1, axis=1) - tris
    side /= np.linalg.norm(side, axis=2, keepdims=True)
    angle = np.arccos(np.clip(-np.einsum('ijk,ijk->ij', side, np.roll(side, 1, axis=1)), -1.0, 1.0)).reshape(-1)
    _, vertex = np.unique(np.round(tris.reshape(-1, 3) * 1e4).astype(np.int64), axis=0, return_inverse=True)
    vertex = vertex.reshape(-1)
    order = np.argsort(vertex, kind='stable')
    starts = np.flatnonzero(np.r_[True, np.diff(vertex[order]) != 0, True])
    normals = np.repeat(face, 3, axis=0)
    limit = math.cos(math.radians(crease_degrees))
    for a, b in zip(starts[:-1], starts[1:]):
        corners = order[a:b]                      # the corners that meet at one point
        if len(corners) == 1:
            continue
        f = face[corners // 3]
        together = (f @ f.T) > limit
        summed = (together * angle[corners]) @ f
        normals[corners] = summed / np.linalg.norm(summed, axis=1, keepdims=True)
    return tris, normals.reshape(-1, 3, 3)


def _floats(values):
    return ' '.join('%.5g' % v for v in np.asarray(values, dtype=float).ravel())


def _write_dae(path, parts, materials):
    """One COLLADA file, a geometry per material."""
    effects, library, geometries, nodes = [], [], [], []
    for name, tris in parts.items():
        if not tris:
            continue
        tris, normals = _smooth_normals(np.concatenate(tris))
        colour, lit, glow = materials[name]
        effects.append(
            '<effect id="%s-fx"><profile_COMMON><technique sid="common"><phong>'
            '<emission><color>%s 1</color></emission>'
            '<ambient><color>%s 1</color></ambient><diffuse><color>%s 1</color></diffuse>'
            '<specular><color>0.08 0.08 0.08 1</color></specular><shininess><float>12</float></shininess>'
            '</phong></technique></profile_COMMON></effect>' % (
                name, _floats(np.multiply(colour, glow)), _floats(np.multiply(colour, lit)),
                _floats(np.multiply(colour, lit))))
        library.append(
            '<material id="%s-mat" name="%s"><instance_effect url="#%s-fx"/></material>' % (name, name, name))
        source = (
            '<source id="{n}-{k}"><float_array id="{n}-{k}-a" count="{c}">{v}</float_array><technique_common>'
            '<accessor source="#{n}-{k}-a" count="{m}" stride="3"><param name="X" type="float"/>'
            '<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>')
        corners, index = np.unique(
            np.round(np.concatenate([tris, normals], axis=2).reshape(-1, 6), 4), axis=0, return_inverse=True)
        geometries.append(
            '<geometry id="%s-geo" name="%s"><mesh>%s%s<vertices id="%s-vtx">'
            '<input semantic="POSITION" source="#%s-pos"/></vertices>'
            '<triangles count="%d" material="%s-mat"><input semantic="VERTEX" source="#%s-vtx" offset="0"/>'
            '<input semantic="NORMAL" source="#%s-nrm" offset="0"/><p>%s</p></triangles></mesh></geometry>' % (
                name, name,
                source.format(n=name, k='pos', c=corners.size // 2, m=len(corners), v=_floats(corners[:, :3])),
                source.format(n=name, k='nrm', c=corners.size // 2, m=len(corners), v=_floats(corners[:, 3:])),
                name, name, len(tris), name, name, name, ' '.join(map(str, index.reshape(-1)))))
        nodes.append(
            '<instance_geometry url="#%s-geo"><bind_material><technique_common>'
            '<instance_material symbol="%s-mat" target="#%s-mat"/></technique_common></bind_material>'
            '</instance_geometry>' % (name, name, name))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(
            '<?xml version="1.0" encoding="utf-8"?>\n'
            '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
            '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>\n'
            '<library_effects>%s</library_effects>\n<library_materials>%s</library_materials>\n'
            '<library_geometries>%s</library_geometries>\n'
            '<library_visual_scenes><visual_scene id="scene"><node id="cart" name="cart">%s</node>'
            '</visual_scene></library_visual_scenes>\n'
            '<scene><instance_visual_scene url="#scene"/></scene>\n</COLLADA>\n' % (
                '\n'.join(effects), '\n'.join(library), '\n'.join(geometries), '\n'.join(nodes)))


URDF = """<?xml version="1.0"?>
<!-- Written by cart_model.py from cart.yaml: change the sizes there, not here. -->
<robot name="golf_cart">

  <!-- The frame the localization moves: at the lidar's height, behind it.
       os_sensor and imu_link hang on it from the static_transform_publisher
       nodes in lidar_localization.launch.py, not from here. -->
  <link name="base_link"/>

  <!-- The cart's own frame: on the ground under the middle of the rear axle,
       level, x forward. No <material>: with one, RViz would paint the whole
       mesh in it. -->
  <link name="cart_body">
    <visual>
      <geometry><mesh filename="{url}golf_cart.dae"/></geometry>
    </visual>
  </link>
  <joint name="cart_body_joint" type="fixed">
    <parent link="base_link"/>
    <child link="cart_body"/>
    <origin xyz="{xyz}" rpy="{rpy}"/>
  </joint>

  <link name="cart_glass">
    <visual>
      <geometry><mesh filename="{url}golf_cart_glass.dae"/></geometry>
      <material name="glass"><color rgba="{glass}"/></material>
    </visual>
  </link>
  <joint name="cart_glass_joint" type="fixed">
    <parent link="cart_body"/>
    <child link="cart_glass"/>
  </joint>
{rtk}
</robot>
"""

# The RTK pole, level like cart_body: its foot, and the antenna on top, which is
# the frame_id u-blox puts on /ublox_gps_node/fix.
RTK_URDF = """
  <link name="rtk_pole_base"/>
  <joint name="rtk_pole_base_joint" type="fixed">
    <parent link="cart_body"/>
    <child link="rtk_pole_base"/>
    <origin xyz="{base}" rpy="0 0 0"/>
  </joint>

  <link name="gps"/>
  <joint name="gps_joint" type="fixed">
    <parent link="rtk_pole_base"/>
    <child link="gps"/>
    <origin xyz="0 0 {length}" rpy="0 0 0"/>
  </joint>
"""


def build(dims, out_dir, url=PACKAGE_URL):
    """Writes the meshes and the URDF into out_dir; returns the URDF's path."""
    materials = dict(MATERIALS)
    for name, colour in (dims.get('colors') or {}).items():
        materials[name] = (tuple(float(c) for c in colour), *MATERIALS[name][1:])
    parts, glass = build_parts(dims)
    os.makedirs(out_dir, exist_ok=True)
    _write_dae(os.path.join(out_dir, 'golf_cart.dae'), parts, materials)
    _write_dae(os.path.join(out_dir, 'golf_cart_glass.dae'), {'glass': glass}, {'glass': (GLASS[:3], 1.0, 0.0)})
    xyz, rpy = cart_in_base_link(dims)
    mount = rtk_mount(dims)
    rtk = '' if mount is None else RTK_URDF.format(
        base='%.4f %.4f %.4f' % tuple(mount[0]), length='%.4f' % mount[1])
    urdf = os.path.join(out_dir, 'golf_cart.urdf')
    with open(urdf, 'w', encoding='utf-8') as f:
        f.write(URDF.format(
            url=url, xyz='%.4f %.4f %.4f' % tuple(xyz), rpy='%.5f %.5f %.5f' % tuple(rpy),
            glass='%g %g %g %g' % GLASS, rtk=rtk))
    return urdf


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--dims', default=os.path.join(here, 'cart.yaml'))
    parser.add_argument('--out', default=here, help='directory the URDF and the meshes are written to')
    parser.add_argument('--url', default=PACKAGE_URL, help='what the URDF puts in front of the mesh names')
    args = parser.parse_args()
    with open(args.dims, encoding='utf-8') as f:
        dims = yaml.safe_load(f)
    print(build(dims, args.out, args.url))


if __name__ == '__main__':
    main()
