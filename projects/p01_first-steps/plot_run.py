#!/usr/bin/env python3
"""Draw a run recorded by measure.py --csv on top of the arena.
usage: python3 projects/p01_first-steps/plot_run.py run.csv OUT.png
Obstacles come from src/go2_nav_sim/scripts/make_arena.py, the same list that generated the Gazebo world and the map.
Ground truth is grey and the estimate (/odom) blue, as everywhere else in this repository."""
import csv, math, pathlib, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'src/go2_nav_sim/scripts'))
import make_arena as arena


def main():
    src, out = sys.argv[1], sys.argv[2]
    runs = {}
    for r in csv.DictReader(open(src)):
        runs.setdefault(r['source'], []).append((float(r['x']), float(r['y']), float(r['yaw'])))

    fig, ax = plt.subplots(figsize=(8, 6.8))
    for x, y, sx, sy in arena.WALLS:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='0.35'))
    for x, y, sx, sy, *_ in arena.BOXES:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='0.75'))
    for x, y, r, *_ in arena.CYLS:
        ax.add_patch(Circle((x, y), r, color='0.75'))
    for (x, y), name, c in ((arena.A, 'A', '#2a9d3a'), (arena.B, 'B', '#d0402b')):
        ax.add_patch(Circle((x, y), 0.4, fill=False, ec=c, lw=1.5))
        ax.text(x, y - 0.65, name, color=c, ha='center', va='top')

    # /odom starts at 0 where the estimator started, /odom_gt where the robot was spawned (before it settled). Shift the
    # estimate so both start at the same point; what is left is the estimator's error during this run.
    if runs.get('gt') and runs.get('odom'):
        dx, dy = runs['gt'][0][0] - runs['odom'][0][0], runs['gt'][0][1] - runs['odom'][0][1]
        runs['odom'] = [(x + dx, y + dy, a) for x, y, a in runs['odom']]

    # The two paths lie within a few cm of each other, so truth is a wide grey band and the estimate a thin blue line on it.
    for key, label, c, lw in (('gt', 'ground truth (/odom_gt)', '0.6', 5), ('odom', 'estimate (/odom), start aligned', '#2563eb', 1.4)):
        p = runs.get(key)
        if not p:
            continue
        xs, ys, yaws = zip(*p)
        ax.plot(xs, ys, color=c, lw=lw, label=label, solid_capstyle='round')
        x, y, a = p[-1]
        ax.annotate('', (x + 0.35 * math.cos(a), y + 0.35 * math.sin(a)), (x, y),
                    arrowprops=dict(arrowstyle='->', color=c, lw=min(lw, 2.5)))  # final heading

    ax.set_aspect('equal'); ax.set_xlim(-6.3, 6.3); ax.set_ylim(-5.3, 5.3)
    ax.set_xlabel('x, world frame (m)'); ax.set_ylabel('y (m)'); ax.grid(alpha=0.3)
    ax.legend(loc='upper right'); ax.set_title(pathlib.Path(src).stem)
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
