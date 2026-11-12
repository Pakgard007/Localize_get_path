#!/usr/bin/env python3
"""
Add the pose_stamp column that nav_path.py needs in order to hand a speed
profile to the controller.

    python3 scripts/add_pose_stamp.py loop2.csv
    python3 scripts/add_pose_stamp.py loop2.csv -o loop2_with_speed.csv

nav_path.py's load_csv() copies pose_stamp into each PoseStamped header, and
pagard_adaptive_fixed.py's compute_path_segment_speeds() turns consecutive
header stamps back into a per-segment speed:

    speed = distance(p1, p0) / (stamp[p1] - stamp[p0])

so the timestamps ARE the speed profile -- a v_target column on its own is
never read by anything. Rows whose pose_stamp is missing or <= 0 are treated
as "not set" and fall back to the previous valid segment speed.

Source of the timestamps, in order of preference:
  1. an existing t_s column (cumulative seconds along the path), offset to a
     positive epoch value;
  2. otherwise integrated from v_target and the distance between points.

The script refuses to write a file whose implied speeds disagree with v_target,
because that silently mis-shapes the speed profile rather than failing loudly.
"""

import argparse
import csv
import math
import os
import sys
import time


def to_float(value):
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def read_rows(path):
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            sys.exit(f'{path}: no header row')
        return list(reader), list(reader.fieldnames)


def segment_lengths(rows):
    lengths = []
    for a, b in zip(rows, rows[1:]):
        ax, ay = to_float(a.get('x')), to_float(a.get('y'))
        bx, by = to_float(b.get('x')), to_float(b.get('y'))
        if None in (ax, ay, bx, by):
            lengths.append(None)
        else:
            lengths.append(math.hypot(bx - ax, by - ay))
    return lengths


def times_from_t_s(rows):
    values = [to_float(r.get('t_s')) for r in rows]
    if any(v is None for v in values):
        return None
    for a, b in zip(values, values[1:]):
        if b < a:
            sys.exit('t_s is not monotonically increasing - refusing to guess')
    return values


def times_from_v_target(rows, lengths):
    speeds = [to_float(r.get('v_target')) for r in rows]
    if any(v is None or v <= 0.0 for v in speeds):
        return None
    times = [0.0]
    for i, length in enumerate(lengths):
        if length is None:
            sys.exit(f'row {i + 1}: missing x/y, cannot integrate v_target')
        # trapezoidal: the segment is driven at the mean of its endpoint targets
        mean_speed = 0.5 * (speeds[i] + speeds[i + 1])
        times.append(times[-1] + length / mean_speed)
    return times


def percentile(values, p):
    values = sorted(values)
    return values[int(p * (len(values) - 1))]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input_csv')
    parser.add_argument('-o', '--output', help='default: <input>_with_speed.csv')
    parser.add_argument('--epoch', type=float, default=None,
                        help='epoch seconds to offset t_s by (default: now)')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='allowed median relative disagreement with v_target (default 0.05)')
    parser.add_argument('--force', action='store_true',
                        help='write even if the v_target cross-check fails')
    args = parser.parse_args()

    rows, fieldnames = read_rows(args.input_csv)
    if len(rows) < 2:
        sys.exit(f'{args.input_csv}: need at least 2 rows, found {len(rows)}')

    lengths = segment_lengths(rows)
    usable = [d for d in lengths if d is not None]
    print(f'{args.input_csv}: {len(rows)} rows, path length {sum(usable):.1f} m')

    times = times_from_t_s(rows)
    if times is not None:
        print('  timestamps: from the existing t_s column')
    else:
        times = times_from_v_target(rows, lengths)
        if times is None:
            sys.exit('  need either a complete t_s column or a complete positive '
                     'v_target column to build pose_stamp')
        print('  timestamps: integrated from v_target (no usable t_s column)')

    # Cross-check: the speed the controller will derive must match v_target,
    # otherwise the profile in the file is not the profile the cart will drive.
    targets = [to_float(r.get('v_target')) for r in rows]
    errors = []
    derived = []
    for i, length in enumerate(lengths):
        dt = times[i + 1] - times[i]
        if length is None or dt <= 0:
            continue
        speed = length / dt
        derived.append(speed)
        # v_target is a per-point column, so the segment between two points is
        # driven at the mean of its endpoints -- comparing against the start
        # point alone reports a false ~8% error wherever the profile ramps.
        a, b = targets[i], targets[i + 1]
        expected = 0.5 * (a + b) if (a is not None and b is not None) else a
        if expected and expected > 0.1:
            errors.append(abs(speed - expected) / expected)

    if derived:
        print(f'  derived segment speed: med={percentile(derived, .5):.2f} '
              f'p95={percentile(derived, .95):.2f} max={max(derived):.2f} m/s')
    if errors:
        median_error = percentile(errors, .5)
        print(f'  cross-check vs v_target: median disagreement {100 * median_error:.1f}% '
              f'over {len(errors)} segments')
        if median_error > args.tolerance and not args.force:
            sys.exit(f'  ABORT: disagreement exceeds {100 * args.tolerance:.0f}%. '
                     f'The timestamps would not reproduce v_target. Use --force to '
                     f'write anyway.')
    else:
        print('  no v_target column to cross-check against')

    epoch = args.epoch if args.epoch is not None else time.time()
    out_path = args.output or (os.path.splitext(args.input_csv)[0] + '_with_speed.csv')

    out_fields = list(fieldnames)
    if 'pose_stamp' not in out_fields:
        # keep it next to index if that column exists, matching get_path.py's layout
        insert_at = out_fields.index('index') + 1 if 'index' in out_fields else 0
        out_fields.insert(insert_at, 'pose_stamp')

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=out_fields)
        writer.writeheader()
        for row, t in zip(rows, times):
            row = dict(row)
            row['pose_stamp'] = f'{epoch + t:.6f}'
            writer.writerow(row)

    print(f'  wrote {out_path}  (pose_stamp = {epoch:.0f} + t_s, '
          f'span {times[-1]:.1f} s)')
    print()
    print('Next: point nav_path.py at it --')
    print(f"  csv_path default in src/sim/nav_path.py, or")
    print(f"  python3 src/sim/nav_path.py --ros-args -p csv_path:={os.path.abspath(out_path)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
