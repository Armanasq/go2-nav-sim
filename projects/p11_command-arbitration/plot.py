#!/usr/bin/env python3
"""The figure of this project: one run of each measurement in measure.py.
usage: python3 projects/p11_command-arbitration/plot.py [--dir DIR]
  reads override_1.csv, estop_1.csv and box_1.csv from DIR (default: results/) and writes arbitration_runs.png
  (default: figures/). Speeds come from /odom_gt: distance over a centred 0.5 s window."""
import argparse, csv, math, pathlib, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from arbitration_check import box_extent, load, polygon

GT, NAV, KEY, OUT = '#777777', '#c27c0e', '#222222', '#1f6fd1'   # ground truth grey, Nav2 amber, teleop black, to CHAMP blue


def read(path):
    d = {}
    for r in csv.DictReader(open(path)):
        d.setdefault(r['src'], []).append((float(r['sim']), float(r['a']), float(r['b'])))
    return {k: np.array(v) for k, v in d.items()}


def speed(gt, win=0.5):
    """(t, |v|) from (t, x, y) samples on a 20 Hz grid."""
    t = np.arange(gt[0, 0], gt[-1, 0], 0.05)
    x, y = np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2])
    k = int(round(win / 0.05 / 2))
    v = np.hypot(x[2 * k:] - x[:-2 * k], y[2 * k:] - y[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    return t[k:-k], v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', help='read the CSVs from DIR and write the figure there')
    a = ap.parse_args()
    src = pathlib.Path(a.dir).expanduser() if a.dir else HERE / 'results'
    dst = pathlib.Path(a.dir).expanduser() if a.dir else HERE / 'figures'
    fig, ax = plt.subplots(1, 3, figsize=(14, 3.8))

    d = read(src / 'override_1.csv')
    t0 = d['teleop/cmd_vel'][d['teleop/cmd_vel'][:, 2] > 0.9][0, 0]
    for k, c, lab in (('nav/cmd_vel', NAV, 'Nav2 (nav/cmd_vel, priority 10)'), ('teleop/cmd_vel', KEY, "key 'j' (teleop/cmd_vel, 100)"),
                      ('cmd_vel', OUT, 'to CHAMP (cmd_vel)')):
        m = (d[k][:, 0] > t0 - 2) & (d[k][:, 0] < t0 + 5)
        ax[0].step(d[k][m, 0] - t0, d[k][m, 2], where='post', color=c, lw=2 if k == 'cmd_vel' else 1.2, label=lab,
                   ls='--' if k == 'teleop/cmd_vel' else '-', zorder=3 if k == 'teleop/cmd_vel' else 2)
    ax[0].set(xlabel='simulated time from the first key event (s)', ylabel='yaw rate command (rad/s)', title='Key overrides a live Nav2 goal')
    ax[0].set_ylim(top=1.65); ax[0].legend(fontsize=8, loc='upper right')    # headroom: the legend covers no trace

    d = read(src / 'estop_1.csv')
    lock = d['e_stop'][d['e_stop'][:, 1] > 0.5][0, 0]
    free = d['e_stop'][(d['e_stop'][:, 1] < 0.5) & (d['e_stop'][:, 0] > lock)][0, 0]
    t, v = speed(d['gt'])
    m = (t > lock - 3) & (t < free + 5)
    ax[1].axvspan(0, free - lock, color='#eeeeee', label='e_stop locked')
    ax[1].plot(t[m] - lock, v[m], color=GT, lw=2, label='true speed (/odom_gt)')
    c = d['cmd_vel']; m = (c[:, 0] > lock - 3) & (c[:, 0] < free + 5)
    ax[1].step(c[m, 0] - lock, c[m, 1], where='post', color=OUT, lw=1.2, label='cmd_vel linear.x')
    ax[1].set(xlabel='simulated time from the lock (s)', ylabel='speed (m/s)', title='Lock stops the robot')
    ax[1].set_ylim(top=0.42); ax[1].legend(fontsize=8, loc='upper right')

    d = read(src / 'box_1.csv')
    start = d['teleop/cmd_vel'][0, 0]
    gt = d['gt'][d['gt'][:, 0] >= start]
    heading = math.atan2(gt[-1, 2] - gt[0, 2], gt[-1, 1] - gt[0, 1])
    along = lambda x, y: (x - gt[0, 1]) * math.cos(heading) + (y - gt[0, 2]) * math.sin(heading)
    t, v = speed(gt)
    s = along(np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2]))
    face = 2.0                                                    # measure.py box --dist
    cm = load('collision_monitor.yaml')
    for name, ls in (('PolygonSlow', ':'), ('PolygonStop', '--')):
        front = box_extent(polygon(cm, name))[1]
        ax[2].axvline(face - front, color='#444444', ls=ls, lw=1, label=f'box face enters {name} (x = {front:.2f} m)')
    ax[2].axvline(face - 0.325, color='#b3261e', lw=1.5, label='body front touches the box')
    ax[2].plot(s, v, color=GT, lw=2, label='true speed (/odom_gt)')
    ax[2].set(xlabel='distance walked towards the box (m)', ylabel='true speed (m/s)', title='Collision Monitor stops before a box', xlim=(0, face))
    ax[2].set_ylim(top=0.5); ax[2].legend(fontsize=7, loc='upper right')
    for x in ax:
        x.grid(color='#e5e5e5'); x.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    out = dst / 'arbitration_runs.png'
    fig.savefig(out, dpi=90)
    print(out)


if __name__ == '__main__':
    main()
