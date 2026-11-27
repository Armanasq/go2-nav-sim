#!/usr/bin/env python3
"""Figures for this project.
usage: python3 projects/p02_teleop-and-safety/plot.py BAG_DIR
  BAG_DIR: a bag of the teleop drive with /odom_gt, /odom, /teleop/cmd_vel, /cmd_vel  -> figures/teleop_drive.png
  results/*.csv from measure_stop.py                                                     -> figures/stop_time.png"""
import csv, math, pathlib, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src/go2_nav_sim/scripts'))
import make_arena as arena                       # walls, boxes, cylinders, A, B: the same list that built the world
from measure_stop import stop_time

GT, EST, CMD, OUT = '#777777', '#1f6fd1', '#c27c0e', '#1f6fd1'   # ground truth grey, odometry blue, teleop amber, safety output blue


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        name, data, t_ns = r.read_next()
        if name in out:
            out[name].append((t_ns * 1e-9, deserialize_message(data, get_message(types[name]))))
    return out


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def draw_arena(ax):
    for x, y, sx, sy in arena.WALLS:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='#bbbbbb'))
    for x, y, sx, sy, *_ in arena.BOXES:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='#d9c9a8'))
    for x, y, r, *_ in arena.CYLS:
        ax.add_patch(Circle((x, y), r, color='#d9c9a8'))
    for (x, y), name in ((arena.A, 'A'), (arena.B, 'B')):
        ax.add_patch(Circle((x, y), 0.4, fill=False, ec='#444444', ls=':'))
        ax.text(x, y, name, ha='center', va='center', color='#444444')
    ax.set_xlim(-6.2, 6.2); ax.set_ylim(-5.2, 5.2); ax.set_aspect('equal')
    ax.set_xlabel('x (m, world)'); ax.set_ylabel('y (m, world)')


def drive_figure(bag):
    d = read_bag(bag, ['/odom_gt', '/odom', '/teleop/cmd_vel', '/cmd_vel'])
    # /odom_gt is relative to the spawn point A; shift it into the world frame
    gt = np.array([(t, m.pose.pose.position.x + arena.A[0], m.pose.pose.position.y + arena.A[1]) for t, m in d['/odom_gt']])
    od = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)) for t, m in d['/odom']])
    g0 = d['/odom_gt'][0][1].pose.pose
    # align /odom to ground truth at the first sample (rotation + translation), so the plot shows drift during the drive only
    dth = yaw(g0.orientation) - od[0, 3]
    c, s = math.cos(dth), math.sin(dth)
    ex = gt[0, 1] + c * (od[:, 1] - od[0, 1]) - s * (od[:, 2] - od[0, 2])
    ey = gt[0, 2] + s * (od[:, 1] - od[0, 1]) + c * (od[:, 2] - od[0, 2])
    # Twist has no stamp: put every message on the simulated clock through the bag receive time of /odom_gt
    t_bag = np.array([t for t, _ in d['/odom_gt']])
    t_sim = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in d['/odom_gt']])
    sim = lambda t: np.interp(t, t_bag, t_sim) - t_sim[0]
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw={'width_ratios': [1.1, 1]})
    draw_arena(ax)
    ax.plot(gt[:, 1], gt[:, 2], color=GT, lw=3, label='ground truth (/odom_gt)')
    ax.plot(ex, ey, color=EST, lw=1.2, label='lidar-inertial odometry (/odom), aligned at start')
    ax.plot(*gt[0, 1:], 'o', color='k', ms=4); ax.plot(*gt[-1, 1:], 's', color='k', ms=4)
    ax.legend(loc='upper left', fontsize=8, frameon=False)
    ax.set_title('Keyboard drive through the safety node (circle start, square end)', fontsize=10)
    tk = np.array([(sim(t), m.linear.x, m.angular.z) for t, m in d['/teleop/cmd_vel']])   # dots: a gap is silence
    bx.plot(tk[:, 0], tk[:, 1], '.', ms=3, color=CMD, label='teleop/cmd_vel forward (keyboard)')
    bx.plot(tk[:, 0], tk[:, 2], '.', ms=3, color='#e0b060', label='teleop/cmd_vel yaw rate')
    tv = np.array([(sim(t), m.linear.x, m.angular.z) for t, m in d['/cmd_vel']])
    bx.plot(tv[:, 0], tv[:, 1], color=OUT, lw=1.4, label='cmd_vel forward (safety output)')
    bx.plot(tv[:, 0], tv[:, 2], color=OUT, lw=1.4, ls='--', label='cmd_vel yaw rate')
    _, speeds = stop_time([(ts, x, y) for ts, (_, x, y) in zip(t_sim, gt)], t_sim[0])
    bx.plot([t - t_sim[0] for t, _ in speeds], [v for _, v in speeds], color=GT, lw=1.2, label='true speed, 0.5 s window')
    bx.set_xlabel('time (s, simulated)'); bx.set_ylabel('m/s  or  rad/s'); bx.legend(fontsize=7, frameon=False, loc='upper right')
    bx.set_ylim(-1.1, 1.25); bx.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(HERE / 'figures/teleop_drive.png', dpi=120)
    err = math.hypot(ex[-1] - gt[-1, 1], ey[-1] - gt[-1, 2])
    path = np.sum(np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])))
    print(f'drive: {t_sim[-1] - t_sim[0]:.1f} s simulated, true path {path:.2f} m, odometry end error {err:.3f} m, '
          f'end at ({gt[-1, 1]:.2f}, {gt[-1, 2]:.2f})')


def stop_figure():
    runs = [('direct_zero', 'straight to CHAMP, one zero twist', '#444444'),
            ('direct_silence', 'straight to CHAMP, then silence', '#b03030'),
            ('safety_silence', 'through safety_node.py, then silence', '#1f6fd1')]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for name, label, col in runs:
        for k, p in enumerate(sorted(HERE.glob(f'results/{name}*.csv'))):
            rows = list(csv.reader(open(p)))[1:]
            t_last = float(rows[0][1]); gt = [(float(r[1]), float(r[2]), float(r[3])) for r in rows if r[0] == 'gt']
            t_stop, speeds = stop_time(gt, t_last)
            ax.plot([t - t_last for t, _ in speeds], [v for _, v in speeds], color=col, lw=1, label=None if k else label)
            i0 = min(range(len(gt)), key=lambda i: abs(gt[i][0] - t_last))
            i1 = min(range(len(gt)), key=lambda i: abs(gt[i][0] - t_stop)) if t_stop else len(gt) - 1
            dist = math.hypot(gt[i1][1] - gt[i0][1], gt[i1][2] - gt[i0][2])
            if t_stop:
                print(f'{p.stem:18s} standstill {t_stop - t_last:.2f} s after the last command, {dist:.3f} m')
            else:
                print(f'{p.stem:18s} still walking {gt[-1][0] - t_last:.1f} s after the last command, {dist:.2f} m, '
                      f'{speeds[-1][1]:.3f} m/s')
    ax.axvline(0, color='k', lw=0.8); ax.text(0.05, 0.33, 'last command', fontsize=8)
    ax.set_xlim(-2, 6); ax.set_ylim(0, 0.35)
    ax.set_xlabel('time since the last command (s, simulated)'); ax.set_ylabel('true speed, 0.5 s window (m/s)')
    ax.legend(fontsize=8, frameon=False, loc='center right'); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(HERE / 'figures/stop_time.png', dpi=120)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        drive_figure(sys.argv[1])
    stop_figure()
