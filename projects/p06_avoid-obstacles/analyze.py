#!/usr/bin/env python3
"""Score the trials on ground truth, compare with Nav2, draw the figure.
usage: python3 projects/p06_avoid-obstacles/analyze.py [--runs ab1 ab2 ...] [--fail ba4 ...] [--nav2-bags bags/w1 ...]
                                                       [--out figures/trials.png]
  --runs       results/NAME.csv of the trials to score (default: every ab*.csv and ba*.csv). After an optional my_ or
               apf_ prefix, a name starting with ab was a run to B, any other a run to A
  --fail       runs to draw in the failure panel
  --nav2-bags  read /odom_gt from these Nav2 bags (needs ROS sourced) and save it to results/nav2_*.csv first
Every run is scored the same way, on /odom_gt only: it starts when the robot has moved 5 cm from where it stood and ends
when it last moved 5 cm from where it finished. Path is the length of the true track over that time, clearance the
smallest distance from the true base_footprint position to an obstacle surface (arena2d.clearance)."""
import argparse, math, pathlib, sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
RES = HERE / 'results'
sys.path.insert(0, str(HERE))
import arena2d  # noqa: E402

A, B = arena2d.arena.A, arena2d.arena.B


def leg(name):
    """'ab' for a run to B, 'ba' for a run to A: my_ab3 -> ab, apf_ab1 -> ab."""
    return name.removeprefix('my_').removeprefix('apf_')[:2]


def load(name):
    """(t, est x, est y, true x, true y) of one of our runs."""
    d = np.loadtxt(RES / f'{name}.csv', delimiter=',', skiprows=1, ndmin=2)
    return d[:, 0], d[:, 1:3], d[:, 4:6]


def start_heading(name):
    """True heading in degrees on the first scan: it decides the route (README, Results)."""
    return math.degrees(np.loadtxt(RES / f'{name}.csv', delimiter=',', skiprows=1, ndmin=2)[0, 6])


def score(t, gt, goal):
    moved = np.flatnonzero(np.hypot(*(gt - gt[0]).T) > 0.05)
    still = np.flatnonzero(np.hypot(*(gt - gt[-1]).T) > 0.05)
    i, j = moved[0], still[-1] + 1
    seg = gt[i:j + 1]
    return dict(time=t[j] - t[i], path=np.hypot(*np.diff(seg, axis=0).T).sum(),
                clear=float(arena2d.clearance(*seg.T).min()), miss=math.dist(gt[-1], goal))


def read_bag(bag):
    """/odom_gt of a Nav2 bag as (t, x, y) in the world frame."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/odom_gt']))
    rows = []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x + A[0], p.y + A[1]))
    return np.array(sorted(rows))


def draw_arena(ax):
    from matplotlib.patches import Circle, Rectangle
    for cx, cy, sx, sy in arena2d.RECTS:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='0.55', lw=0))
    for cx, cy, r in arena2d.CIRCLES:
        ax.add_patch(Circle((cx, cy), r, color='0.55', lw=0))
    for (x, y), lab in ((A, 'A'), (B, 'B')):
        ax.add_patch(Circle((x, y), 0.4, fill=False, ls=':', color='k'))
        ax.text(x, y, lab, ha='center', va='center', fontsize=11)
    ax.set_aspect('equal'); ax.set_xlim(-6.2, 6.2); ax.set_ylim(-5.2, 5.2)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('--runs', nargs='*')
    ap.add_argument('--fail', nargs='*', default=[])
    ap.add_argument('--nav2-bags', nargs='*', default=[])
    ap.add_argument('--out', default='figures/trials.png', help='relative to this folder')
    a = ap.parse_args()
    for bag in a.nav2_bags:
        g = read_bag(bag)
        out = RES / f'nav2_{pathlib.Path(bag).name}.csv'
        np.savetxt(out, g, delimiter=',', fmt='%.4f', header='t,x,y', comments='')
        print(f'{bag}: {len(g)} /odom_gt messages -> {out.relative_to(HERE.parents[1])}')
    runs = a.runs or sorted(p.stem for p in RES.glob('a[b]*.csv')) + sorted(p.stem for p in RES.glob('b[a]*.csv'))

    print(f'{"run":8} {"goal":4} {"start deg":>9} {"time s":>7} {"path m":>7} {"min clear m":>11} {"true miss m":>11} '
          f'{"odom err m":>10}')
    rows = {}
    for n in runs:
        t, est, gt = load(n)
        goal = B if leg(n) == 'ab' else A
        s = score(t, gt, goal)
        s['odom'] = math.dist(est[-1], gt[-1])
        rows[n] = s
        print(f'{n:8} {"B" if goal == B else "A":4} {start_heading(n):9.0f} {s["time"]:7.1f} {s["path"]:7.2f} {s["clear"]:11.2f} {s["miss"]:11.2f} '
              f'{s["odom"]:10.2f}')
    nav2 = sorted(RES.glob('nav2_*.csv'))
    for f in nav2:
        g = np.loadtxt(f, delimiter=',', skiprows=1)
        s = score(g[:, 0], g[:, 1:3], B)
        print(f'{f.stem:8} {"B":4} {"":>9} {s["time"]:7.1f} {s["path"]:7.2f} {s["clear"]:11.2f} {s["miss"]:11.2f} {"":>10}')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(20, 5.8), constrained_layout=True)
    for ax, way, goal in ((axs[0], 'ab', 'A to B'), (axs[1], 'ba', 'B to A')):
        draw_arena(ax)
        if way == 'ab':
            for f in nav2:
                g = np.loadtxt(f, delimiter=',', skiprows=1)
                ax.plot(g[:, 1], g[:, 2], color='#d08c00', lw=1.2, label='Nav2 with map, true path' if f == nav2[0] else None)
        mine = [n for n in runs if leg(n) == way and not n.startswith(('apf_', 'my_apf_')) and rows[n]['miss'] < 0.5]
        for k, n in enumerate(mine):
            t, est, gt = load(n)
            ax.plot(*gt.T, color='0.15', lw=1.2, label='reactive (vfh), true path' if k == 0 else None)
            ax.plot(*est.T, color='#2a6fdb', lw=0.9, ls='--', label='reactive, its own estimate (/odom)' if k == 0 else None)
        ax.set_title(f'{goal}: {len(mine)} reactive runs that arrived' + (f', {len(nav2)} Nav2 runs' if way == 'ab' else ''))
        ax.legend(loc='upper left', fontsize=8)
    ax = axs[2]; draw_arena(ax)
    for n, ls in zip(a.fail, ('-', '--', ':', '-.') * 3):     # true paths: black, told apart by line style
        t, est, gt = load(n)
        print(f'failure {n}: gave up at ({gt[-1, 0]:.2f}, {gt[-1, 1]:.2f}) after {t[-1]:.1f} s, '
              f'min clearance {arena2d.clearance(*gt.T).min():.2f} m')
        ax.plot(*gt.T, color='0.15', ls=ls, lw=1.4, label=f'{n}, true path')
        ax.plot(*gt[-1], 'x', color='k', ms=9)
    ax.set_title('Failures: where the robot gave up (x)')
    if a.fail:
        ax.legend(loc='upper left', fontsize=8)
    out = HERE / a.out
    fig.savefig(out, dpi=110)
    print(f'wrote {out.relative_to(HERE.parents[1])}')


if __name__ == '__main__':
    main()
