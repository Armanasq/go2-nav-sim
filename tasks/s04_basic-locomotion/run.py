#!/usr/bin/env python3
"""Stage 4 acceptance tests: basic locomotion with CHAMP on /cmd_vel, scored on /odom_gt and the Gazebo link poses.
usage: python3 run.py <task-id>|all [--keep]        e.g. python3 run.py 4.10
Run after `source env.sh`. Without --keep the stack is restarted first (scripts/stop.sh, then launch/sim_gait.launch.py with
worlds/flat.world and odom:=gt), so the robot stands on marker A. 4.16 and 4.17 restart it with a changed gait.yaml and
restore the default at the end. Writes results/<id>.json (+ .npz raw data), figures/4-NN_*.png, media/4-03_walk_in_place.mp4."""
import argparse
import contextlib
import fcntl
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates

import gait as g

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'modules'))
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

ACC = json.loads((HERE / 'acceptance.json').read_text())
RES, FIG, MEDIA = HERE / 'results', HERE / 'figures', HERE / 'media'
P02 = ROOT / 'projects/p02_teleop-and-safety'
PY = sys.executable
GAIT = {'default': HERE / 'params/gait.yaml', 'stance035': HERE / 'params/gait_stance035.yaml',
        'swing008': HERE / 'params/gait_swing008.yaml'}
STATE = {'gait': None}                 # gait of the running stack (None: unknown, --keep assumes default)
FOOT_C = dict(lf=fs.C['est'], rf=fs.C['plan'], lh=fs.C['good'], rh=fs.C['bad'])


# ---- bring-up -----------------------------------------------------------------------------------------------------------
def ok(cmd):
    return subprocess.run(cmd, shell=True, cwd=ROOT, capture_output=True, text=True).returncode == 0


def wait_for(check, seconds, what):
    t_end = time.time() + seconds
    while time.time() < t_end:
        if check():
            return
        time.sleep(2)
    sys.exit(f'TIMEOUT waiting for {what}')


def bring_up(gait='default'):
    subprocess.run(['bash', 'scripts/stop.sh'], cwd=ROOT)
    port = os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345').rsplit(':', 1)[1]
    wait_for(lambda: not ok(f'ss -ltn | grep -q ":{port} "'), 40, 'the Gazebo port to be free')
    (ROOT / 'logs').mkdir(exist_ok=True)
    subprocess.Popen(['ros2', 'launch', str(HERE / 'launch/sim_gait.launch.py'), f'gait:={GAIT[gait]}'], cwd=ROOT,
                     stdout=open(ROOT / 'logs/s04_sim.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    wait_for(lambda: ok('timeout 10 gz stats -d 1 | grep -q Factor'), 300, 'gazebo')
    wait_for(lambda: ok('timeout 10 ros2 topic echo --once /imu/data --no-daemon'), 150, '/imu/data')
    want = '0.35' if gait == 'stance035' else '0.25'
    wait_for(lambda: ok(f'ros2 param get /quadruped_controller_node gait.stance_duration | grep -q {want}'), 60, 'CHAMP params')
    time.sleep(3)                      # let the robot settle after the drop from 0.275 m
    STATE['gait'] = gait
    print(f'stack up, gait {gait}', flush=True)


def ensure(gait):
    if STATE['gait'] != gait:
        bring_up(gait)


# ---- recording ----------------------------------------------------------------------------------------------------------
class Rec(Node):
    """Logs /odom_gt (pose, at its stamp) and the four foot centres from /gazebo/link_states (at the clock), publishes /cmd_vel."""

    def __init__(self):
        super().__init__('s04_rec', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.gt, self.feet, self.idx = [], [], None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, 50)
        self.create_subscription(LinkStates, '/gazebo/link_states', self.on_links, 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, g.yaw_of(q.x, q.y, q.z, q.w)))

    def on_links(self, m):
        if self.idx is None:
            self.idx = [m.name.index(f'go2::{f}_lower_leg_link') for f in g.FEET]
        row = [self.now()]
        for i in self.idx:
            p, q = m.pose[i].position, m.pose[i].orientation
            row += g.foot_from_calf(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        self.feet.append(row)

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(m)

    def spin_until(self, cond, timeout=120.0):
        t_end = time.time() + timeout
        while not cond():
            if time.time() > t_end:
                sys.exit('TIMEOUT: no /clock, /odom_gt or /gazebo/link_states')
            time.sleep(0.05)


NODE = None


def node():
    global NODE
    if NODE is None:
        rclpy.init()
        NODE = Rec()
        # spin in a thread: spin_once in the drive loop fell behind the 1 kHz /clock and delivered link states up to 1 s late
        ex = SingleThreadedExecutor()
        ex.add_node(NODE)
        threading.Thread(target=ex.spin, daemon=True).start()
    return NODE


def drive(name, segs, pre=1.0, tail=2.0):
    """Stand for pre seconds, run segs = [(seconds, vx, vy, wz)] at 20 Hz on simulated time, then zeros for tail seconds.
    Returns the run in the start frame (pose at the end of pre): dict of arrays, also saved to results/<name>.npz."""
    n = node()
    n.spin_until(lambda: n.now() > 0 and n.gt and n.feet)
    n.gt.clear(); n.feet.clear()
    plan = [(pre, 0, 0, 0)] + list(segs) + [(tail, 0, 0, 0)]
    ends = np.cumsum([s[0] for s in plan])
    t0, tick, cmd = n.now(), 0.0, []
    while True:
        time.sleep(0.002)
        t = n.now() - t0
        if t >= ends[-1]:
            break
        if t >= tick:
            k = int(np.searchsorted(ends, t, side='right'))
            n.send(*plan[k][1:]); cmd.append((t, *plan[k][1:]))
            tick += 0.05
    for _ in range(3):
        n.send()
    gt, feet = np.array(n.gt), np.array(n.feet)
    gt[:, 0] -= t0; feet[:, 0] -= t0
    i0 = int(np.searchsorted(gt[:, 0], pre))
    x0, y0, yaw0 = gt[i0, 1], gt[i0, 2], gt[i0, 4]
    pose = np.array([g.to_start(x, y, yaw, x0, y0, yaw0) for x, y, yaw in gt[:, [1, 2, 4]]])
    yaw_u = np.unwrap(pose[:, 2])
    stand = feet[:, 0] < pre
    clear = np.stack([feet[:, 3 + 3 * k] - np.median(feet[stand, 3 + 3 * k]) for k in range(4)], axis=1)
    run = dict(t=gt[:, 0], x=pose[:, 0], y=pose[:, 1], yaw=yaw_u, z=gt[:, 3], ft=feet[:, 0], feet=feet[:, 1:], clear=clear,
               cmd=np.array(cmd), pre=pre, t_end=float(ends[-2]), gait=STATE['gait'] or 'default')
    RES.mkdir(exist_ok=True)
    np.savez_compressed(RES / f'{name}.npz', **run)
    return run


def load(name):
    d = np.load(RES / f'{name}.npz')
    return {k: (d[k].item() if d[k].ndim == 0 else d[k]) for k in d.files}


def swings_of(run):
    return {f: g.swings(run['clear'][:, k]) for k, f in enumerate(g.FEET)}


@contextlib.contextmanager
def render_lock():
    with open('/tmp/go2_render.lock', 'w') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        yield


@contextlib.contextmanager
def safety_node():
    p = subprocess.Popen([PY, str(P02 / 'safety_node.py')], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    time.sleep(4)
    try:
        yield p
    finally:
        p.send_signal(signal.SIGINT)             # the node publishes one zero twist as it exits
        p.wait(timeout=20)


# ---- figures ------------------------------------------------------------------------------------------------------------
def save(fig, stem):
    fs.save(fig, FIG / stem)
    return [f'figures/{stem}.png']


def clearance_axes(ax, run, ta, tb, sw=None):
    m = (run['ft'] >= ta) & (run['ft'] <= tb)
    for k, f in enumerate(g.FEET):
        ax.plot(run['ft'][m], 100 * run['clear'][m, k], color=FOOT_C[f], lw=1.2, label=f)
    ax.axvspan(run['pre'], run['t_end'], color=fs.C['gt'], alpha=.12, lw=0, label='command on')
    ax.set_xlabel('time [s]'); ax.set_ylabel('foot clearance [cm]')


def footfall_axes(ax, run, ta, tb):
    sw = swings_of(run)
    for k, f in enumerate(g.FEET):
        ax.broken_barh([(run['ft'][a], run['ft'][b] - run['ft'][a]) for a, b, p in sw[f] if ta <= run['ft'][a] <= tb],
                       (3 - k - 0.35, 0.7), color=FOOT_C[f])
    ax.set_yticks([3, 2, 1, 0], g.FEET); ax.set_xlim(ta, tb); ax.set_xlabel('time [s] (bars: swing)')


def path_figure(stem, runs, title):
    """runs: [(label, run, (ux, uy) or None, distance)] in the start frame."""
    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    for label, run, u, d in runs:
        line, = ax.plot(run['x'], run['y'], lw=1.6, label=label)
        if u is not None:
            ax.plot([0, u[0] * d], [0, u[1] * d], color=line.get_color(), ls=':', lw=1)
    ax.plot(0, 0, 'k^', ms=6)
    ax.set_aspect('equal', adjustable='datalim'); ax.set_xlabel('x, start frame [m]'); ax.set_ylabel('y [m]')
    ax.set_title(title); ax.legend(fontsize=7.5, loc='best')
    return save(fig, stem)


# ---- tasks: each returns (metrics, outputs) -----------------------------------------------------------------------------
def t401():
    run = drive('4.01', [(0.25, 0.1, 0, 0)])
    sw = swings_of(run)
    stepping = {f for f in g.FEET if sw[f]}
    pk = [max(p for a, b, p in sw[f]) for f in ('rf', 'lh') if sw[f]]
    met = dict(stepping_feet=sorted(stepping), stepping_pair_is_rf_lh=stepping == {'rf', 'lh'},
               liftoffs_per_stepping_foot=max((len(sw[f]) for f in stepping), default=0),
               stepping_peak_clearance_m=min(pk) if len(pk) == 2 else 0.0,
               other_pair_max_clearance_m=float(max(run['clear'][:, 0].max(), run['clear'][:, 3].max())),
               swing_time_s={f: [round(float(run['ft'][b] - run['ft'][a]), 3) for a, b, p in sw[f]] for f in stepping},
               body_displacement_m=float(math.hypot(run['x'][-1], run['y'][-1])))
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    clearance_axes(ax, run, 0.6, 2.0); ax.set_title('4.01 one footstep: vx 0.1 m/s for 0.25 s'); ax.legend(fontsize=7.5, loc='upper right')
    return met, save(fig, '4-01_footstep')


def t402():
    run = drive('4.02', [(4.0, 0.2, 0, 0)])
    s = g.gait_stats(run['ft'], swings_of(run), 2.0, run['t_end'])
    met = dict(stride_period_s=s['period'], duty_factor=s['duty'], phase_rf=s['phase']['rf'], phase_lh=s['phase']['lh'],
               phase_rh_wrapped=g.wrapped_phase(s['phase']['rh']), n_lifts=s['n_lifts'])
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.4, 4.4), sharex=True, gridspec_kw=dict(height_ratios=[2, 1]))
    clearance_axes(a1, run, 2.0, 3.5); a1.legend(fontsize=7.5, ncol=5); a1.set_xlabel('')
    a1.set_title(f"4.02 one gait cycle at 0.2 m/s: period {s['period']:.3f} s, duty factor {s['duty']:.2f}")
    footfall_axes(a2, run, 2.0, 3.5)
    return met, save(fig, '4-02_gait_cycle')


def t403():
    if STATE['gait'] is not None and math.hypot(*node_xy()) > 0.3:
        bring_up()                                   # the side camera looks at marker A
    rec = None
    with render_lock():
        MEDIA.mkdir(exist_ok=True)
        rec = subprocess.Popen([PY, str(ROOT / 'projects/common/record_cam.py'), str(MEDIA / '4-03_walk_in_place.mp4'),
                                '--topic', '/side/cam/image_raw', '--fps', '20', '--width', '640'], start_new_session=True)
        time.sleep(3)
        run = drive('4.03', [(10.0, 0.001, 0, 0)])
        rec.send_signal(signal.SIGINT); rec.wait(timeout=60)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-ss', '6', '-i', str(MEDIA / '4-03_walk_in_place.mp4'),
                    '-frames:v', '1', str(MEDIA / '4-03_walk_in_place.png')])
    sw = swings_of(run)
    n = {f: sum(1 for a, b, p in sw[f] if run['pre'] <= run['ft'][a] <= run['t_end']) for f in g.FEET}
    met = dict(drift_m=float(math.hypot(run['x'][-1], run['y'][-1])), yaw_change_deg=math.degrees(run['yaw'][-1]),
               min_liftoffs_per_foot=min(n.values()), liftoffs=n)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.4, 4.0), gridspec_kw=dict(height_ratios=[1, 1.4]))
    footfall_axes(a1, run, 0.5, run['t_end'] + 1); a1.set_title(f'4.03 walk in place, vx 0.001 m/s for 10 s: '
                                                               f'{min(n.values())}-{max(n.values())} steps per foot')
    a2.plot(run['t'], 100 * run['x'], label='x'); a2.plot(run['t'], 100 * run['y'], label='y')
    a2.set_xlabel('time [s]'); a2.set_ylabel('trunk drift [cm]'); a2.legend(fontsize=7.5)
    return met, save(fig, '4-03_walk_in_place') + ['media/4-03_walk_in_place.mp4']


def node_xy():
    n = node()
    n.spin_until(lambda: n.gt)
    n.gt.clear(); n.spin_until(lambda: n.gt)
    return (n.gt[-1][1], n.gt[-1][2])                # /odom_gt is relative to marker A


TRANS = {'4.04': ('forward 0.5 m', 2.5, 0.2, 0.0), '4.05': ('forward 1 m', 5.0, 0.2, 0.0),
         '4.06': ('backward 0.5 m', 2.5, -0.2, 0.0), '4.07': ('left 0.5 m', 2.5, 0.0, 0.2),
         '4.08': ('diagonal 0.5 m', 2.5, 0.1414, 0.1414)}


def translation(tid):
    label, T, vx, vy = TRANS[tid]
    run = drive(tid, [(T, vx, vy, 0)])
    v = math.hypot(vx, vy); u = (vx / v, vy / v); d = v * T
    along, cross = g.line_errors(run['x'], run['y'], *u)
    met = dict(target_m=d, along_m=along, distance_ratio=along / d, max_cross_track_m=cross,
               cross_track_over_distance=cross / d, heading_change_deg=math.degrees(run['yaw'][-1]))
    runs = []
    for k, (lab, Tk, ax_, ay_) in TRANS.items():
        if (RES / f'{k}.npz').exists():
            vk = math.hypot(ax_, ay_)
            runs.append((f'{k} {lab}', load(k), (ax_ / vk, ay_ / vk), vk * Tk))
    return met, path_figure('4-04_translations', runs, '4.04-4.08 open-loop walks (dotted: commanded)')


TURNS = {'4.09': 45, '4.10': 90, '4.11': 180}


def turn(tid):
    deg = TURNS[tid]
    T = math.radians(deg) / 0.5
    run = drive(tid, [(T, 0, 0, 0.5)])
    met = dict(final_heading_deg=math.degrees(run['yaw'][-1]), heading_error_deg=math.degrees(run['yaw'][-1]) - deg,
               position_drift_m=float(np.max(np.hypot(run['x'], run['y']))))
    fig, ax = plt.subplots(figsize=(6.0, 3.4))
    for k, dg in TURNS.items():
        if (RES / f'{k}.npz').exists():
            r = load(k)
            line, = ax.plot(r['t'], np.degrees(r['yaw']), label=f'{k} turn {dg} deg')
            ax.axhline(dg, color=line.get_color(), ls=':', lw=1)
    ax.set_xlabel('time [s]'); ax.set_ylabel('heading [deg]'); ax.set_title('4.09-4.11 wz 0.5 rad/s for angle / 0.5 s (dotted: target)')
    ax.legend(fontsize=7.5)
    return met, save(fig, '4-09_turns')


def t412():
    v, w = 0.2, 0.3
    T = (math.pi / 2) / w
    run = drive('4.12', [(T, v, 0, w)])
    r = v / w
    dev = g.arc_deviation(run['x'], run['y'], r)
    met = dict(radius_m=r, max_radial_deviation_m=float(dev.max()), rms_radial_deviation_m=float(np.sqrt(np.mean(dev ** 2))),
               end_heading_error_deg=math.degrees(run['yaw'][-1]) - 90.0,
               end_angle_on_arc_deg=math.degrees(math.atan2(run['x'][-1], r - run['y'][-1])))
    th = np.linspace(0, math.pi / 2, 60)
    fig, ax = plt.subplots(figsize=(4.6, 4.2))
    ax.plot(r * np.sin(th), r - r * np.cos(th), color=fs.C['plan'], ls=':', label='commanded arc, r = 0.667 m')
    ax.plot(run['x'], run['y'], color=fs.C['gt'], lw=1.6, label='ground truth')
    ax.set_aspect('equal'); ax.set_xlabel('x, start frame [m]'); ax.set_ylabel('y [m]'); ax.legend(fontsize=7.5)
    ax.set_title(f"4.12 curve: max deviation {100 * dev.max():.1f} cm")
    return met, save(fig, '4-12_curve')


def t413():
    v0, T = 0.3, 5.0
    run = drive('4.13', [(T, v0, 0, 0)], tail=3.0)
    tc, v = g.windowed_speed(run['t'], run['x'], run['y'])
    t_on, t_off = run['pre'], run['t_end']
    steady = float(np.mean(v[(tc > t_off - 2.0) & (tc < t_off - 0.25)]))
    r10, r90 = g.crossing_time(tc, v, 0.1 * steady, t_on - 0.3), g.crossing_time(tc, v, 0.9 * steady, t_on - 0.3)
    f90 = g.crossing_time(tc, v, 0.9 * steady, t_off - 0.3, rising=False)
    f10 = g.crossing_time(tc, v, 0.1 * steady, t_off - 0.3, rising=False)
    met = dict(steady_speed_m_s=steady, steady_ratio=steady / v0, rise_time_s=r90 - r10, fall_time_s=f10 - f90,
               delay_on_s=r10 - t_on, delay_off_s=f90 - t_off,
               window_note='0.5 s centred window: a perfect step would show a 0.4 s 10-90 % time')
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    c = run['cmd']
    ax.step(c[:, 0], c[:, 1], where='post', color=fs.C['plan'], label='command vx')
    ax.plot(tc, v, color=fs.C['gt'], lw=1.6, label='ground-truth speed, 0.5 s window')
    ax.set_xlabel('time [s]'); ax.set_ylabel('speed [m/s]'); ax.legend(fontsize=7.5)
    ax.set_title(f'4.13 step 0 to 0.3 m/s and back: rise {r90 - r10:.2f} s, fall {f10 - f90:.2f} s')
    return met, save(fig, '4-13_accel')


SWEEP = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]


def sweep_run():
    if (RES / '4.14.npz').exists() and ARGS.id == 'all' and '4.14' in DONE:
        return load('4.14')
    return drive('4.14', [(6.0, v, 0, 0) for v in SWEEP])


def t414():
    run = sweep_run()
    rows = []
    for k, vc in enumerate(SWEEP):
        ta, tb = run['pre'] + 6.0 * k + 2.0, run['pre'] + 6.0 * (k + 1)
        i, j = np.searchsorted(run['t'], [ta, tb])
        v = math.hypot(run['x'][j - 1] - run['x'][i], run['y'][j - 1] - run['y'][i]) / (run['t'][j - 1] - run['t'][i])
        rows.append(dict(cmd=vc, speed=v, error_pct=100 * (v - vc) / vc))
    met = dict(steps=rows, max_tracking_error_pct=max(abs(r['error_pct']) for r in rows))
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    ax.plot([0, 0.32], [0, 0.32], color=fs.C['plan'], ls=':', label='speed = command')
    ax.plot([r['cmd'] for r in rows], [r['speed'] for r in rows], 'o', color=fs.C['gt'], label='measured, last 4 s of 6 s')
    for r in rows:
        ax.annotate(f"{r['error_pct']:+.0f} %", (r['cmd'], r['speed']), textcoords='offset points', xytext=(6, -10), fontsize=7.5)
    ax.set_xlabel('commanded vx [m/s]'); ax.set_ylabel('ground-truth speed [m/s]'); ax.legend(fontsize=7.5)
    ax.set_title('4.14 speed sweep')
    return met, save(fig, '4-14_speeds')


def t415():
    run = sweep_run()
    sw = swings_of(run)
    st = {}
    for k in (0, len(SWEEP) - 1):
        ta, tb = run['pre'] + 6.0 * k + 2.0, run['pre'] + 6.0 * (k + 1)
        s = g.gait_stats(run['ft'], sw, ta, tb)
        st[SWEEP[k]] = dict(period_s=s['period'], duty=s['duty'], phase={f: s['phase'][f] for f in g.FEET}, window=(ta, tb))
    slow, fast = st[SWEEP[0]], st[SWEEP[-1]]
    changes = abs(slow['phase']['lh'] - fast['phase']['lh']) > 0.15 or abs(slow['duty'] - fast['duty']) > 0.1
    met = dict(slow=slow, fast=fast, gait_changes_with_speed=bool(changes),
               source='phase_generator.h: leg_clocks offsets 0, 0.5, 0.5, 0 of the stride and a fixed 0.25 s swing; no gait-type parameter')
    fig, axs = plt.subplots(2, 1, figsize=(6.4, 3.8))
    for ax, s, vc in zip(axs, (slow, fast), (SWEEP[0], SWEEP[-1])):
        footfall_axes(ax, run, s['window'][0], s['window'][0] + 2.0)
        ax.set_title(f"vx {vc} m/s: duty factor {s['duty']:.2f}, lh phase {s['phase']['lh']:.2f}", fontsize=9)
    fig.suptitle('4.15 footfall at the slowest and fastest command: the same trot', fontsize=10)
    return met, save(fig, '4-15_gait_types')


def stride(run):
    """Stride period and length per foot from the lift-off times and the foot position at each touchdown."""
    sw = swings_of(run)
    s = g.gait_stats(run['ft'], sw, 2.0, run['t_end'])
    lens = [math.dist(run['feet'][b1, 3 * k:3 * k + 2], run['feet'][b0, 3 * k:3 * k + 2])
            for k, f in enumerate(g.FEET) for (_, b0, _), (a1, b1, _) in zip(sw[f], sw[f][1:]) if 2.0 <= run['ft'][a1] <= run['t_end']]
    peaks = [p for f in g.FEET for a, b, p in sw[f] if 2.0 <= run['ft'][a] <= run['t_end']]
    tc, v = g.windowed_speed(run['t'], run['x'], run['y'])
    speed = float(np.mean(v[(tc > 2.0) & (tc < run['t_end'] - 0.25)]))
    return dict(period_s=s['period'], stride_m=float(np.median(lens)), peak_clearance_m=float(np.mean(peaks)), speed_m_s=speed,
                duty=s['duty'])


def gait_pair(tid, changed):
    ensure('default')
    a = drive(f'{tid}_default', [(6.0, 0.2, 0, 0)])
    bring_up(changed)
    b = drive(f'{tid}_{changed}', [(6.0, 0.2, 0, 0)])
    bring_up('default')
    return a, b


def t416():
    a, b = gait_pair('4.16', 'stance035')
    sa, sb = stride(a), stride(b)
    met = dict(default=sa, stance035=sb, period_error_default_s=sa['period_s'] - 0.5, period_error_changed_s=sb['period_s'] - 0.6,
               stride_length_increase_pct=100 * (sb['stride_m'] / sa['stride_m'] - 1))
    fig, axs = plt.subplots(2, 1, figsize=(6.4, 3.8), sharex=True)
    for ax, r, lab, s in zip(axs, (a, b), ('stance_duration 0.25 s', 'stance_duration 0.35 s'), (sa, sb)):
        clearance_axes(ax, r, 3.0, 5.0); ax.set_xlim(3.0, 5.0); ax.set_xlabel('')
        ax.set_title(f"{lab}: period {s['period_s']:.3f} s, stride {100 * s['stride_m']:.1f} cm, speed {s['speed_m_s']:.3f} m/s", fontsize=9)
    axs[0].legend(fontsize=7, ncol=5); axs[1].set_xlabel('time [s]')
    return met, save(fig, '4-16_step_length')


def t417():
    a, b = gait_pair('4.17', 'swing008')
    sa, sb = stride(a), stride(b)
    met = dict(default=sa, swing008=sb, clearance_increase_m=sb['peak_clearance_m'] - sa['peak_clearance_m'],
               changed_clearance_m=sb['peak_clearance_m'])
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    for r, lab, ls in ((a, 'swing_height 0.04 m', '-'), (b, 'swing_height 0.08 m', '-')):
        m = (r['ft'] > 3.0) & (r['ft'] < 5.0)
        ax.plot(r['ft'][m], 100 * r['clear'][m, 0], lw=1.4, ls=ls, label=f'lf, {lab}')
    ax.set_xlabel('time [s]'); ax.set_ylabel('foot clearance [cm]'); ax.legend(fontsize=7.5)
    ax.set_title(f"4.17 mean peak clearance {100 * sa['peak_clearance_m']:.1f} -> {100 * sb['peak_clearance_m']:.1f} cm")
    return met, save(fig, '4-17_clearance')


STOP_RE = re.compile(r'standstill ([\d.]+) s after the last command, ([\d.]+) m')
NOTSTOP_RE = re.compile(r'NOT stopped ([\d.]+) s .*walked ([\d.]+) m since, speed at end ([\d.]+)')


def measure_stop(topic, csv, zero=False):
    cmd = [PY, str(P02 / 'measure_stop.py'), '--topic', topic, '--csv', str(RES / csv)] + (['--zero'] if zero else [])
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=300).stdout.strip().splitlines()[-1]
    print(out, flush=True)
    m = STOP_RE.search(out)
    if m:
        return dict(stopped=True, time_s=float(m[1]), distance_m=float(m[2]), line=out)
    m = NOTSTOP_RE.search(out)
    return dict(stopped=False, watched_s=float(m[1]), distance_m=float(m[2]), speed_end_m_s=float(m[3]), line=out)


def stop_curve(ax, csv, label):
    rows = [ln.strip().split(',') for ln in open(RES / csv)][1:]
    t_last = float(next(r[1] for r in rows if r[0] == 't_last'))
    gt = np.array([[float(v) for v in r[1:4]] for r in rows if r[0] == 'gt'])
    tc, v = g.windowed_speed(gt[:, 0], gt[:, 1], gt[:, 2])
    ax.plot(tc - t_last, v, lw=1.5, label=label)


def stop_figure():
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    for csv, lab in (('4.18_zero.csv', '4.18 one zero twist'), ('4.20_no_node.csv', '4.20 silence, no safety node'),
                     ('4.20_safety.csv', '4.20 silence, through safety_node.py')):
        if (RES / csv).exists():
            stop_curve(ax, csv, lab)
    ax.axvline(0, color=fs.C['gt'], lw=1); ax.axhline(0.02, color=fs.C['gt'], ls=':', lw=1)
    ax.set_xlim(-2, 6); ax.set_xlabel('time after the last command [s]'); ax.set_ylabel('speed [m/s]'); ax.legend(fontsize=7.5)
    ax.set_title('4.18 and 4.20: stopping from 0.3 m/s (dotted: 0.02 m/s standstill)')
    return save(fig, '4-18_stops')


def t418():
    r = measure_stop('/cmd_vel', '4.18_zero.csv', zero=True)
    met = dict(stop_time_s=r.get('time_s', math.inf), stop_distance_m=r['distance_m'], output=r['line'])
    return met, stop_figure()


def t419():
    n = node()
    with safety_node():
        n.gt.clear()
        p = subprocess.Popen([PY, str(P02 / 'key_drive.py'), 'i 3, - 3, j 3, - 3'], stdout=subprocess.PIPE, text=True)
        p.wait()
        lines = p.stdout.read().strip().splitlines()
    print('\n'.join(lines), flush=True)
    starts = [float(ln.split()[0]) for ln in lines if re.match(r'\s*[\d.]+\s+\S+ for', ln)]
    gt = np.array(n.gt)
    at = lambda t: gt[min(np.searchsorted(gt[:, 0], t), len(gt) - 1)]  # noqa: E731
    p0, p1, p2, p3 = at(starts[0]), at(starts[2]), at(starts[2]), at(starts[3] + 3.0)
    yaw0 = p0[4]
    fwd = (p1[1] - p0[1]) * math.cos(yaw0) + (p1[2] - p0[2]) * math.sin(yaw0)
    turn_deg = math.degrees(g.wrap(p3[4] - p2[4]))
    m = gt[:, 0] > starts[3] + 2.0
    tail = gt[m]
    moved = math.hypot(tail[-1, 1] - tail[0, 1], tail[-1, 2] - tail[0, 2])
    spun = abs(math.degrees(g.wrap(tail[-1, 4] - tail[0, 4])))
    met = dict(forward_on_i_m=fwd, turn_on_j_deg=turn_deg, last_1s_move_m=moved, last_1s_turn_deg=spun,
               stopped_after_release=bool(moved < 0.02 and spun < 1.0), key_log=lines)
    fig, ax = plt.subplots(figsize=(5.0, 4.0))
    sel = gt[:, 0] >= starts[0] - 1
    ax.plot(gt[sel, 1], gt[sel, 2], color=fs.C['gt'], lw=1.6, label='ground truth')
    for t, lab in zip(starts, ('i', '-', 'j', '-')):
        q = at(t); ax.plot(q[1], q[2], 'o', color=fs.C['plan'], ms=4); ax.annotate(f'{lab} at {t:.1f} s', q[1:3], fontsize=7.5, xytext=(5, 5), textcoords='offset points')
    ax.set_aspect('equal', adjustable='datalim'); ax.set_xlabel('x from marker A [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'4.19 keyboard: {fwd:.2f} m on i, {turn_deg:.0f} deg on j')
    return met, save(fig, '4-19_keyboard')


def t420():
    base = measure_stop('/cmd_vel', '4.20_no_node.csv')
    for _ in range(3):
        node().send()
    with safety_node():
        r = measure_stop('/teleop/cmd_vel', '4.20_safety.csv')
    met = dict(safe_stop_time_s=r.get('time_s', math.inf), safe_stop_distance_m=r['distance_m'], without_node_stopped=base['stopped'],
               without_node=base['line'], with_node=r['line'])
    return met, stop_figure()


TASKS = {'4.01': t401, '4.02': t402, '4.03': t403, **{k: (lambda k=k: translation(k)) for k in TRANS},
         **{k: (lambda k=k: turn(k)) for k in TURNS}, '4.12': t412, '4.13': t413, '4.14': t414, '4.15': t415,
         '4.16': t416, '4.17': t417, '4.18': t418, '4.19': t419, '4.20': t420}
ORDER = ['4.03', '4.01', '4.02'] + [k for k in TASKS if k not in ('4.01', '4.02', '4.03')]   # 4.03 first: robot still on marker A
DONE = set()


# ---- scoring ------------------------------------------------------------------------------------------------------------
def rule(k, r):
    if 'range' in r:
        return f'{r["range"][0]} <= {k} <= {r["range"][1]}', lambda v: r['range'][0] <= v <= r['range'][1]
    if 'abs_max' in r:
        return f'|{k}| <= {r["abs_max"]}', lambda v: abs(v) <= r['abs_max']
    if 'max' in r:
        return f'{k} <= {r["max"]}', lambda v: v <= r['max']
    if 'min' in r:
        return f'{k} >= {r["min"]}', lambda v: v >= r['min']
    return f'{k} == {json.dumps(r["equals"])}', lambda v: v == r['equals']


def score(tid, fn):
    t_wall = time.time()
    met, outputs = fn()
    checks = {}
    for k, r in ACC[tid]['checks'].items():
        txt, f = rule(k, r)
        v = met.get(k)
        checks[k] = dict(value=v, rule=txt, passed=bool(v is not None and f(v)))
    res = dict(id=tid, title=ACC[tid]['title'], status='verified' if all(c['passed'] for c in checks.values()) else 'failed',
               metrics=met, acceptance='; '.join(c['rule'] for c in checks.values()), checks=checks,
               command=f'python3 run.py {tid}', outputs=outputs, gait=STATE['gait'], wall_s=round(time.time() - t_wall, 1))
    RES.mkdir(exist_ok=True)
    (RES / f'{tid}.json').write_text(json.dumps(res, indent=2, default=float) + '\n')
    DONE.add(tid)
    print(f"{tid} {res['status']}: " + ', '.join(f"{k}={v['value']:.4g}" if isinstance(v['value'], float) else f"{k}={v['value']}"
                                             for k, v in checks.items()), flush=True)
    return res


def main():
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument('id', help='task id (4.01 ... 4.20) or all')
    ap.add_argument('--keep', action='store_true', help='use the running stack (default gait) instead of restarting it')
    ARGS = ap.parse_args()
    ids = ORDER if ARGS.id == 'all' else [ARGS.id]
    if ARGS.keep:
        STATE['gait'] = 'default'
    else:
        bring_up()
    for tid in ids:
        score(tid, TASKS[tid])
    node().send()
    rclpy.try_shutdown()


ARGS = None
if __name__ == '__main__':
    main()
