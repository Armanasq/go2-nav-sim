#!/usr/bin/env python3
"""Figure and results table from results/*.json and .npz.
usage: python3 projects/p08_navigate-with-nav2/plot.py OUT.png RUN [RUN ...]      e.g. patrol.png patrol_1 box_1
One panel per run; prints the results table in Markdown."""
import json, pathlib, sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parents[1] / 'src/go2_nav_sim/scripts'))
import make_arena as arena  # noqa: E402
import patrol_logic as pl  # noqa: E402

GT, ODOM, AMCL, PLAN = '#7f7f7f', '#1f6fd0', '#7b4fb8', '#e0a020'   # house colours: truth grey, odometry blue, AMCL violet, plan amber


def draw_arena(ax, waypoints):
    for cx, cy, sx, sy, *_ in [(*w, 0) for w in arena.WALLS] + arena.BOXES:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='0.25'))
    for cx, cy, r, *_ in arena.CYLS:
        ax.add_patch(Circle((cx, cy), r, color='0.25'))
    for i, (x, y, _) in enumerate(waypoints):
        ax.plot(x, y, 'o', ms=14, mfc='white', mec='black'); ax.text(x, y, str(i), ha='center', va='center', fontsize=9)
    ax.set_xlim(-6.3, 6.3); ax.set_ylim(-5.3, 5.3); ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')


def plans_of(d):
    p = d['plans']; cut = np.flatnonzero(np.diff(p[:, 0])) + 1
    return [(seg[0, 0], seg[:, 1:]) for seg in np.split(p, cut)]


def main():
    out, names = sys.argv[1], sys.argv[2:]
    runs = [(json.loads((HERE / f'results/{n}.json').read_text()), np.load(HERE / f'results/{n}.npz')) for n in names]
    fig, axes = plt.subplots(1, len(runs), figsize=(6.5 * len(runs), 5.6), squeeze=False); axes = axes[0]
    for ax, (j, d) in zip(axes, runs):
        draw_arena(ax, j['waypoints'])
        gt = d['gt'][(d['gt'][:, 0] >= j['t_start']) & (d['gt'][:, 0] <= j['t_end'])]
        am = d['amcl'][d['amcl'][:, 0] >= j['t_start']]
        ax.plot(gt[:, 1], gt[:, 2], color=GT, lw=2.5, label='ground truth (/odom_gt)')
        if 'odom' in d and len(d['odom']):
            od = d['odom'][(d['odom'][:, 0] >= j['t_start']) & (d['odom'][:, 0] <= j['t_end'])]
            ax.plot(od[:, 1], od[:, 2], color=ODOM, lw=1, label='lidar-inertial odometry (/odom)')
        ax.plot(am[:, 1], am[:, 2], '.', color=AMCL, ms=3, label='AMCL (/amcl_pose)')
        b = j.get('box_run')
        if b:
            ax.add_patch(Rectangle((b['x'] - pl.BOX / 2, b['y'] - pl.BOX / 2), pl.BOX, pl.BOX, fill=False, hatch='///', ec='black', lw=1.5,
                                   label='spawned box (not on the map)'))
            plans = plans_of(d)
            before = [p for t, p in plans if t <= b['t']][-1]
            after = next(p for t, p in plans if abs(t - b['t_replan']) < 1e-6)
            ax.plot(*before.T, '--', color=PLAN, lw=2, label=f"last plan before the box ({b['plan_before_len_m']} m)")
            ax.plot(*after.T, '-', color=PLAN, lw=2, label=f"first plan around it, {b['replan_after_s']} s later ({b['plan_after_len_m']} m)")
            ax.plot(*b['robot'], 'x', color='black', ms=9, mew=2, label='robot when the box appeared')
        ax.set_title(f"{j['name']}: {j['result']}, missed {j['missed_waypoints']}, {j['time_s']} s", fontsize=10)
        ax.legend(loc='lower center', fontsize=8, bbox_to_anchor=(0.5, -0.3), ncol=2)
    fig.tight_layout(); (HERE / 'figures').mkdir(exist_ok=True)
    fig.savefig(HERE / 'figures' / out, dpi=110, bbox_inches='tight'); print('figures/' + out)

    print('| Run | Box | Result | Missed | Time (s) | Legs 0-3 (s) | Walked (m) | End error (m) | AMCL RMSE / max (m) |')
    print('|---|---|---|---|---|---|---|---|---|')
    for j, _ in runs:
        print(f"| {j['name']} | {'yes' if j['box'] else 'no'} | {j['result']} | {j['missed_waypoints']} | {j['time_s']} | "
              f"{', '.join(map(str, j['leg_times_s']))} | {j['path_m']} | {j['final_error_m']} | {j['amcl_rmse_m']} / {j['amcl_max_m']} |")
    for j, _ in runs:
        if j.get('box_run'):
            print(json.dumps({k: v for k, v in j['box_run'].items() if not k.startswith('t')}))


if __name__ == '__main__':
    main()
