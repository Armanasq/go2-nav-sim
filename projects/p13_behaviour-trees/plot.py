#!/usr/bin/env python3
"""Draw the recovery timeline of the two unreachable-goal runs and the patrol path.
usage:  python3 projects/p13_behaviour-trees/plot.py      (reads results/, writes figures/recovery_timeline.png, patrol_path.png)"""
import csv, math, pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import bt_tools

HERE = pathlib.Path(__file__).resolve().parent
R, F = HERE / 'results', HERE / 'figures'
COLOUR = {'Spin': '#4a7fb5', 'Wait': '#b0b0b0', 'BackUp': '#d9902f'}


def timeline():
    runs = [('default tree', 'default.log'), ('BackUp and Spin swapped', 'swapped.log')]
    fig, ax = plt.subplots(figsize=(8, 2.4))
    for row, (title, name) in enumerate(runs):
        events = bt_tools.parse_log(open(R / name))
        for node, t0, t1 in bt_tools.intervals(events):
            ax.barh(row, t1 - t0, left=t0, height=0.5, color=COLOUR[node], edgecolor='white', linewidth=2)
            ax.text((t0 + t1) / 2, row, node, ha='center', va='center', fontsize=9)
        end = max(t for t, n, p, c in events if n == 'NavigateRecovery' and c == 'FAILURE')
        ax.plot([end, end], [row - 0.3, row + 0.3], color='black', lw=1.5)
        ax.text(end + 0.2, row, f'FAILED {end:.1f} s', va='center', fontsize=9)
    ax.set_yticks(range(len(runs)), [t for t, _ in runs]); ax.invert_yaxis()
    ax.set_xlabel('simulated time since the goal was sent (s), from /behavior_tree_log')
    ax.set_xlim(0, ax.get_xlim()[1] + 3)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    fig.tight_layout(); fig.savefig(F / 'recovery_timeline.png', dpi=110)


def patrol():
    img = plt.imread(str(HERE.parents[1] / 'src/go2_nav_sim/maps/arena.pgm'))
    runs = [('first try: W2 at (-3.0, -1.0), stopped after 590 s', 'patrol_first_try_gt.csv', 'patrol_tight_w2.xml'),
            ('W2 moved to (-3.5, -1.5): two laps in 154 s', 'patrol_gt.csv', 'patrol.xml')]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True)
    for ax, (title, csv_name, tree) in zip(axes, runs):
        rows = list(csv.DictReader(open(R / csv_name)))
        ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=(-6.5, -6.5 + img.shape[1] * 0.05, -5.5, -5.5 + img.shape[0] * 0.05))
        ax.plot([float(r['x']) for r in rows], [float(r['y']) for r in rows], color='0.3', lw=1.2, label='ground truth (/odom_gt)')
        for (wx, wy, yaw), name in zip(bt_tools.waypoints(HERE / 'trees' / tree), ('W1', 'W2', 'A (home)')):
            ax.plot(wx, wy, 'o', ms=9, mfc='#d9902f', mec='black')
            ax.annotate('', (wx + 0.5 * math.cos(yaw), wy + 0.5 * math.sin(yaw)), (wx, wy), arrowprops=dict(arrowstyle='->', lw=1.2))
            ax.text(wx + 0.25, wy - 0.45, name, fontsize=10)
        ax.set_xlim(-6.2, 2.0); ax.set_ylim(-5.2, 1.0); ax.set_aspect('equal'); ax.set_title(title, fontsize=10)
        ax.set_xlabel('x (m, map)')
    axes[0].set_ylabel('y (m, map)'); axes[1].legend(loc='upper right', fontsize=9)
    fig.tight_layout(); fig.savefig(F / 'patrol_path.png', dpi=100)


if __name__ == '__main__':
    timeline()
    patrol()
