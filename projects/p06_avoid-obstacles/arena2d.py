#!/usr/bin/env python3
"""The arena in 2D, from make_arena.py: a fake /scan (ray casting), the distance to the nearest obstacle, and a kinematic
robot that follows reactive.step. Used to tune reactive.py without Gazebo, by the tests, and by analyze.py for clearance.
usage: python3 projects/p06_avoid-obstacles/arena2d.py [--method vfh|apf] [--start X Y YAW_DEG] [--goal X Y] [--set KEY=VALUE ...]
Prints the outcome of one run of the kinematic model (no gait, no sensor noise, no delay)."""
import argparse, math, pathlib, sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'src/go2_nav_sim/scripts'))
import make_arena as arena  # noqa: E402
import reactive  # noqa: E402

RECTS = [w[:4] for w in arena.WALLS] + [b[:4] for b in arena.BOXES]      # centre x, y, size x, size y
CIRCLES = [c[:3] for c in arena.CYLS]                                     # centre x, y, radius
# /scan as configured in src/go2_nav_sim/config/scan.yaml
ANGLE_MIN, ANGLE_INC, RANGE_MIN, RANGE_MAX = -3.14159, 0.00698, 0.4, 15.0
N_BEAMS = int((3.14159 - ANGLE_MIN) / ANGLE_INC) + 1


def raycast(x, y, yaw):
    """Ranges of a scan taken at (x, y, yaw); inf where a beam hits nothing within RANGE_MAX."""
    a = yaw + ANGLE_MIN + ANGLE_INC * np.arange(N_BEAMS)
    dx, dy = np.cos(a), np.sin(a)
    best = np.full(N_BEAMS, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for cx, cy, sx, sy in RECTS:                                   # slab test against an axis-aligned box
            tx1, tx2 = (cx - sx / 2 - x) / dx, (cx + sx / 2 - x) / dx
            ty1, ty2 = (cy - sy / 2 - y) / dy, (cy + sy / 2 - y) / dy
            t0 = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            t1 = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = (t1 >= t0) & (t0 > 0)
            best[hit] = np.minimum(best[hit], t0[hit])
        for cx, cy, rad in CIRCLES:                                    # |p + t d - c| = r, nearer root
            b = dx * (x - cx) + dy * (y - cy)
            disc = b ** 2 - ((x - cx) ** 2 + (y - cy) ** 2 - rad ** 2)
            t = -b - np.sqrt(disc)
            hit = (disc >= 0) & (t > 0)
            best[hit] = np.minimum(best[hit], t[hit])
    best[best > RANGE_MAX] = np.inf
    return best


def clearance(x, y):
    """Distance from (x, y) to the nearest obstacle surface (walls included). Works on arrays."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    d = np.full(np.broadcast(x, y).shape, np.inf)
    for cx, cy, sx, sy in RECTS:
        d = np.minimum(d, np.hypot(np.maximum(abs(x - cx) - sx / 2, 0), np.maximum(abs(y - cy) - sy / 2, 0)))
    for cx, cy, rad in CIRCLES:
        d = np.minimum(d, np.hypot(x - cx, y - cy) - rad)
    return d


def run(start, goal, method='vfh', p=reactive.P, dt=0.1, t_max=200.0):
    """Kinematic unicycle driven by reactive.step at 10 Hz (the scan rate). Returns (outcome, time, path xy array)."""
    x, y, yaw = start
    prog, path, prev = reactive.Progress(), [(x, y)], None
    for k in range(int(t_max / dt)):
        v, w, info = reactive.step(raycast(x, y, yaw), ANGLE_MIN, ANGLE_INC, RANGE_MIN, RANGE_MAX, (x, y, yaw), goal, method, prev, p)
        prev = info['dir']
        if info['dist'] < p['goal_tol']:
            return 'reached', k * dt, np.array(path)
        if prog.stuck(k * dt, info['dist']):
            return 'stuck', k * dt, np.array(path)
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
        path.append((x, y))
    return 'timeout', t_max, np.array(path)


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('--start', type=float, nargs=3, default=(*arena.A, 0.0))
    ap.add_argument('--goal', type=float, nargs=2, default=arena.B)
    ap.add_argument('--method', choices=('vfh', 'apf'), default='vfh')
    ap.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE', help='override a value of reactive.P')
    a = ap.parse_args()
    p = dict(reactive.P, **{k: float(v) for k, v in (kv.split('=') for kv in a.set)})
    out, t, xy = run((a.start[0], a.start[1], math.radians(a.start[2])), a.goal, a.method, p)
    seg = np.hypot(*np.diff(xy, axis=0).T).sum()
    print(f'{out} after {t:.1f} s, path {seg:.2f} m, min clearance {clearance(*xy.T).min():.2f} m, '
          f'end ({xy[-1, 0]:.2f}, {xy[-1, 1]:.2f})')


if __name__ == '__main__':
    main()
