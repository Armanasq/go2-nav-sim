#!/usr/bin/env python3
"""Stage 8 acceptance tests: timed and feedback movement of the Go2 in Gazebo, scored on /odom_gt.
usage: python3 run.py <task-id>|all [--keep]        e.g. python3 run.py 8.06
Run from any folder after `source env.sh`. Each run restarts Gazebo and the estimator (scripts/stop.sh, sim.launch.py with
worlds/arena_overhead.world, estimation.launch.py), so the robot starts on marker A with /odom at zero. --keep uses the stack
that is already running instead. Writes results/<id>.json and .csv, figures/<id>.png; 8.10 also media/8.10_push.mp4."""
import argparse
import csv
import fcntl
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import ApplyLinkWrench

import motion as m
sys.path.insert(0, str(m.ROOT / 'modules'))
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
ROOT = m.ROOT
ACC = json.loads((HERE / 'acceptance.json').read_text())
PUSH = yaml.safe_load((HERE / 'params/push.yaml').read_text())
RES, FIG, MEDIA = HERE / 'results', HERE / 'figures', HERE / 'media'
TIMEOUT = 240.0                                   # s of simulated time per run
RESCORE = False                                   # --rescore: score and draw the logged CSVs again, no simulator


# ---- bring-up -----------------------------------------------------------------------------------------------------------
def wait_for(check, seconds, what):
    t_end = time.time() + seconds
    while time.time() < t_end:
        if check():
            return
        time.sleep(2)
    sys.exit(f'TIMEOUT waiting for {what}')


def ok(cmd):
    return subprocess.run(cmd, shell=True, cwd=ROOT, capture_output=True, text=True).returncode == 0


def bring_up():
    """What scripts/up.sh does for sim and estimation (Nav2 is not needed here), with this stage's world and gzserver."""
    env = dict(os.environ, PATH=f"{HERE / 'bin'}:{os.environ['PATH']}")      # bin/gzserver adds /apply_link_wrench
    subprocess.run(['bash', 'scripts/stop.sh'], cwd=ROOT, env=env)
    port = os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345').rsplit(':', 1)[1]
    wait_for(lambda: not ok(f'ss -ltn | grep -q ":{port} "'), 40, 'the Gazebo port to be free')
    (ROOT / 'logs').mkdir(exist_ok=True)
    launch = lambda args, log: subprocess.Popen(['ros2', 'launch', 'go2_nav_sim', *args], cwd=ROOT, env=env,  # noqa: E731
                                                stdout=open(ROOT / 'logs' / log, 'w'), stderr=subprocess.STDOUT,
                                                start_new_session=True)
    launch(['sim.launch.py', f"world:={HERE / 'worlds/arena_overhead.world'}"], 's08_sim.log')
    wait_for(lambda: ok('timeout 10 gz stats -d 1 | grep -q Factor'), 300, 'gazebo')
    wait_for(lambda: ok('timeout 10 ros2 topic echo --once /imu/data --no-daemon'), 150, '/imu/data')
    launch(['estimation.launch.py'], 's08_est.log')
    wait_for(lambda: ' pose=' in (ROOT / 'logs/s08_est.log').read_text(), 120, 'lidar odometry')


# ---- one run ------------------------------------------------------------------------------------------------------------
def pose_of(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), p.z


class Driver(Node):
    """20 Hz on simulated time: pose from /odom into the start frame, one controller step, /cmd_vel, one logged row."""

    def __init__(self, ctl, push=False):
        super().__init__('s08_driver', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ctl, self.push = ctl, push
        self.odom = self.gt = self.t0 = self.t_push = None
        self.rows, self.done = [], False
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', lambda msg: setattr(self, 'odom', pose_of(msg)), 10)
        self.create_subscription(Odometry, 'odom_gt', lambda msg: setattr(self, 'gt', pose_of(msg)), 10)
        self.cli = self.create_client(ApplyLinkWrench, 'apply_link_wrench')
        self.create_timer(0.05, self.tick)

    def send_push(self, t):
        """start_time must be set: with start_time 0 this gazebo_ros version accepts the call and applies nothing."""
        start = self.get_clock().now().nanoseconds + 50_000_000
        req = ApplyLinkWrench.Request(link_name=PUSH['link'])
        req.wrench.force.y = float(PUSH['force_y_n'])
        req.start_time.sec, req.start_time.nanosec = divmod(start, 10**9)
        req.duration.sec, req.duration.nanosec = divmod(int(PUSH['duration_s'] * 1e9), 10**9)
        self.cli.call_async(req)
        self.t_push = start * 1e-9 - self.t0

    def tick(self):
        if self.done or self.odom is None or self.gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0, self.o0, self.g0 = t, self.odom[:3], self.gt[:3]
        x, y, yaw = m.to_start(*self.odom[:3], *self.o0)
        gx, gy, gyaw = m.to_start(*self.gt[:3], *self.g0)
        if self.push and self.t_push is None and x >= PUSH['at_m']:
            self.send_push(t)
        cmd = self.ctl.step(t - self.t0, x, y, yaw)
        if cmd is None or t - self.t0 > TIMEOUT:
            cmd, self.done = (0.0, 0.0, 'stop' if cmd is None else 'timeout'), True
        v, w, label = cmd
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.pub.publish(tw)
        self.rows.append(dict(t=t - self.t0, label=label, v=v, w=w, x=x, y=y, yaw=yaw, gx=gx, gy=gy, gyaw=gyaw, gz=self.gt[3]))


def load(name):
    rows = [{k: (v if k == 'label' else float(v) if v else None) for k, v in r.items()} for r in csv.DictReader(open(RES / f'{name}.csv'))]
    return rows, rows[0].get('t_push')


def drive(name, ctl, keep, push=False, video=None):
    """Fresh stack (unless keep), wait 2 s standing, run the controller to the end, log to results/<name>.csv.
    With --rescore nothing is driven: the rows come from the CSV of an earlier run."""
    if RESCORE:
        return load(name)
    if not keep:
        bring_up()
    node = Driver(m.Timed([(2.0, 0.0, 0.0, 'wait')]))
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rec = None
    if video:                             # p03's record.py is projects/common/record_cam.py with a reliable subscription
        rec = subprocess.Popen([sys.executable, str(ROOT / 'projects/p03_move-in-shapes/record.py'), video, '--topic',
                                '/overhead/cam/image_raw'], start_new_session=True)
        time.sleep(3)
    node = Driver(ctl, push)
    if push:
        node.cli.wait_for_service(timeout_sec=30)
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(5):                    # CHAMP keeps walking on the last command
        node.pub.publish(Twist())
    rows, t_push = node.rows, node.t_push
    for r in rows:
        r['t_push'] = '' if t_push is None else t_push
    node.destroy_node()
    if rec:
        time.sleep(2)
        rec.send_signal(signal.SIGINT)
        rec.wait(timeout=60)
    RES.mkdir(exist_ok=True)
    with open(RES / f'{name}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f'{name}: {len(rows)} rows, {rows[-1]["t"]:.1f} s, ended with {rows[-1]["label"]}')
    return rows, t_push


# ---- figures ------------------------------------------------------------------------------------------------------------
def fig_name(tid):
    return 's' + tid.replace('.', '_')            # no dot: figstyle.save would read '.01' as a file extension


def save(fig, tid):
    fs.save(fig, FIG / fig_name(tid))
    return [f'figures/{fig_name(tid)}.png']


def xy_figure(tid, runs, plan=None, title='', goals_only=False):
    """runs: [(label, rows, colour key)]; ground truth solid, /odom dashed (both in their own start frame)."""
    fig, ax = plt.subplots(figsize=(5.6, 4.0))
    if plan:
        px, py = zip(*plan)
        ax.plot(px, py, color=fs.C['plan'], lw=1.2, marker='o' if len(plan) < 30 else None, ms=5 if goals_only else 3, ls='' if goals_only else '-',
                label='goals' if goals_only else 'plan')
    for label, rows, c in runs:
        ax.plot([r['gx'] for r in rows], [r['gy'] for r in rows], color=fs.C[c], label=f'{label}: ground truth')
        ax.plot([r['x'] for r in rows], [r['y'] for r in rows], color=fs.C[c], ls='--', lw=1, alpha=.7, label=f'{label}: /odom')
    ax.plot(0, 0, 'k^', ms=6)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x, start frame [m]'); ax.set_ylabel('y [m]'); ax.set_title(f'{tid} {title}')
    ax.legend(loc='best', fontsize=7.5)
    return save(fig, tid)


# ---- tasks: each returns (metrics, outputs) ------------------------------------------------------------------------------
def heading_err(r, target):
    return math.degrees(m.wrap(r['gyaw'] - target))


def t801(keep):
    rows, _ = drive('8.01', m.plan('8.01'), keep)
    walk = [r for r in rows if r['label'] == 'walk']
    speed = math.hypot(walk[-1]['gx'] - walk[0]['gx'], walk[-1]['gy'] - walk[0]['gy']) / (walk[-1]['t'] - walk[0]['t'])
    out = xy_figure('8.01', [('timed', rows, 'gt')], [(0, 0), m.POINT], 'timed commands to (1.5, 0.5)')
    return dict(final_pos_err_m=m.dist(rows[-1], m.POINT), odom_final_err_m=m.dist(rows[-1], m.POINT, ''),
                heading_after_turn_deg=math.degrees(walk[0]['gyaw']), commanded_heading_deg=math.degrees(math.atan2(0.5, 1.5)),
                achieved_speed_m_s=speed, commanded_speed_m_s=0.25, time_s=rows[-1]['t']), out


def t802(keep):
    prev = RES / '8.01.json'
    if not prev.exists():
        sys.exit('8.02 compares with 8.01: run 8.01 first')
    rows, _ = drive('8.02', m.plan('8.02'), keep)
    timed, _ = load('8.01')
    out = xy_figure('8.02', [('timed (8.01)', timed, 'gt'), ('feedback', rows, 'est')], [(0, 0), m.POINT],
                    'timed vs feedback to (1.5, 0.5)')
    e, e01 = m.dist(rows[-1], m.POINT), json.loads(prev.read_text())['metrics']['final_pos_err_m']
    return dict(final_pos_err_m=e, err_8_01_m=e01, better_than_8_01=e < e01, odom_final_err_m=m.dist(rows[-1], m.POINT, ''),
                odom_gap_m=m.odom_gap(rows[-1]), time_s=rows[-1]['t']), out


def t803(keep):
    rows, _ = drive('8.03', m.plan('8.03'), keep)
    ends = [m.last_with(rows, f'settle{k}') for k in range(len(m.HEADINGS))]
    errs = [heading_err(r, math.radians(h)) for r, h in zip(ends, m.HEADINGS)]
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    ax.plot([r['t'] for r in rows], [math.degrees(r['gyaw']) for r in rows], color=fs.C['gt'], label='ground truth')
    ax.plot([r['t'] for r in rows], [math.degrees(r['yaw']) for r in rows], color=fs.C['est'], ls='--', label='/odom')
    for r, h in zip(ends, m.HEADINGS):
        ax.plot(r['t'], h, 'o', color=fs.C['plan'])
    ax.set_xlabel('time [s]'); ax.set_ylabel('heading [deg]'); ax.set_title('8.03 turns on the spot to +90, -90, 0 deg')
    ax.legend()
    save(fig, '8.03')
    return dict(heading_errs_deg=errs, max_heading_err_deg=max(map(abs, errs)),
                odom_heading_errs_deg=[math.degrees(m.wrap(r['yaw'] - math.radians(h))) for r, h in zip(ends, m.HEADINGS)],
                trunk_moved_m=[math.hypot(r['gx'], r['gy']) for r in ends], time_s=rows[-1]['t']), [f'figures/{fig_name("8.03")}.png']


def poses_task(tid, goals, title, keep):
    rows, _ = drive(tid, m.plan(tid), keep)
    out = xy_figure(tid, [('feedback', rows, 'est')], [g[:2] for g in goals], title, goals_only=True)
    r = rows[-1]
    return dict(final_pos_err_m=m.dist(r, goals[-1]), final_heading_err_deg=heading_err(r, goals[-1][2]),
                odom_final_err_m=m.dist(r, goals[-1], ''), odom_gap_m=m.odom_gap(r), time_s=r['t'],
                timeouts=sum(1 for x in rows if x['label'] == 'timeout')), out


def t805(keep):
    rows, _ = drive('8.05', m.plan('8.05'), keep)
    out = xy_figure('8.05', [('feedback', rows, 'est')], [(0, 0), (m.LINE, 0)], 'straight line, 2.5 m')
    return dict(max_cross_track_m=max(abs(r['gy']) for r in rows), final_along_err_m=rows[-1]['gx'] - m.LINE,
                odom_gap_m=m.odom_gap(rows[-1]), time_s=rows[-1]['t']), out


def corners_task(tid, pts, title, keep, key='max_corner_err_m'):
    rows, _ = drive(tid, m.plan(tid), keep)
    errs = [m.dist(m.last_with(rows, f'corner{k}'), p) for k, p in enumerate(pts)
            if any(r['label'] == f'corner{k}' for r in rows)]
    out = xy_figure(tid, [('feedback', rows, 'est')], [(0, 0)] + list(pts), title)
    res = dict(corner_errs_m=errs, n_reached=len(errs), **{key: max(errs)}, odom_gap_m=m.odom_gap(rows[-1]), time_s=rows[-1]['t'])
    if tid == '8.06':                             # only the square turns back to its start heading
        res['final_heading_err_deg'] = heading_err(rows[-1], 0)
    return res, out


def t807(keep):
    rows, _ = drive('8.07', m.plan('8.07'), keep)
    e = m.radial([r for r in rows if r['label'] == 'circle'], (0, m.CIRCLE_R), m.CIRCLE_R)
    ring = [(m.CIRCLE_R * math.sin(a / 50 * 2 * math.pi), m.CIRCLE_R * (1 - math.cos(a / 50 * 2 * math.pi))) for a in range(51)]
    out = xy_figure('8.07', [('feedback', rows, 'est')], ring, 'circle, r = 0.6 m')
    return dict(rms_radial_err_m=math.sqrt(sum(v * v for v in e) / len(e)), max_radial_err_m=max(map(abs, e)),
                mean_radial_err_m=sum(e) / len(e), final_pos_err_m=m.dist(rows[-1], (0, 0)), odom_gap_m=m.odom_gap(rows[-1]),
                time_s=rows[-1]['t']), out


def t808(keep):
    rows, _ = drive('8.08', m.plan('8.08'), keep)
    out = xy_figure('8.08', [('feedback', rows, 'est')], [(0, 0)] + m.shapes.figure8_points(), 'figure-eight, r = 0.45 m')
    return dict(crossing_err_m=m.dist(m.last_with(rows, 'walk11'), (0, 0)), final_pos_err_m=m.dist(rows[-1], (0, 0)),
                final_heading_err_deg=heading_err(rows[-1], 0), odom_gap_m=m.odom_gap(rows[-1]), time_s=rows[-1]['t']), out


def t810(keep):
    MEDIA.mkdir(exist_ok=True)
    raw = f'/tmp/s08_push_raw_{os.getpid()}.mp4'
    if RESCORE:
        fb, tp = drive('8.10', None, keep)
        speed = json.loads((RES / '8.10.json').read_text())['metrics']['video_speedup']
    else:
        with open('/tmp/go2_render.lock', 'w') as lock:              # one render at a time on this machine
            fcntl.flock(lock, fcntl.LOCK_EX)
            fb, tp = drive('8.10', m.plan('8.10'), keep, push=True, video=raw)
            dur = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', raw],
                                       capture_output=True, text=True).stdout)
            speed = max(1.0, dur / 18.0)                               # the clip stays under 20 s; the crop is the line, 4 m wide
            subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', raw, '-vf', f'crop=720:405:120:100,scale=960:540,setpts=PTS/{speed:.3f}', '-r', '20',
                            '-c:v', 'libx264', '-crf', '28', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an',
                            str(MEDIA / '8.10_push.mp4')], check=True)
    ol, tpo = drive('8.10_open', m.plan('8.10', 'open'), keep, push=True)
    peak, rec = m.line_recovery(fb, tp)
    peak_o, _ = m.line_recovery(ol, tpo)
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.6))
    for rows, t_p, c, lab in ((fb, tp, 'est', 'feedback'), (ol, tpo, 'bad', 'timed')):
        ax[0].plot([r['gx'] for r in rows], [r['gy'] for r in rows], color=fs.C[c], label=f'{lab}: ground truth')
        ax[1].plot([r['t'] - t_p for r in rows], [r['gy'] for r in rows], color=fs.C[c], label=lab)
    ax[0].plot([0, m.PUSH_LINE], [0, 0], color=fs.C['plan'], label='line')
    ax[0].annotate(f"push {PUSH['force_y_n']:.0f} N, {PUSH['duration_s']} s", (PUSH['at_m'], 0), (0.25, 0.2),
                   arrowprops=dict(arrowstyle='->'), fontsize=8)
    ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]'); ax[0].legend(fontsize=7.5)
    ax[0].set_title('8.10 lateral push 1 m along a 3 m line')
    ax[1].axhspan(-0.05, 0.05, color=fs.C['fill'])
    ax[1].axvline(0, color=fs.C['muted'], lw=.8)
    ax[1].set_xlabel('time since the push [s]'); ax[1].set_ylabel('cross-track y [m]'); ax[1].legend(fontsize=7.5)
    out = save(fig, '8.10')
    after = [r for r in fb if r['t'] >= tp]
    return dict(push_force_n=PUSH['force_y_n'], push_duration_s=PUSH['duration_s'], push_peak_dev_m=peak, recovery_time_s=rec,
                final_cross_track_m=abs(fb[-1]['gy']), final_along_err_m=fb[-1]['gx'] - m.PUSH_LINE,
                fell=min(r['gz'] for r in fb + ol) < 0.12, max_odom_gap_after_push_m=max(m.odom_gap(r) for r in after),
                open_loop_peak_dev_m=peak_o, open_loop_final_cross_track_m=abs(ol[-1]['gy']), time_s=fb[-1]['t'],
                video_speedup=round(speed, 2)), out + ['media/8.10_push.mp4']


TASKS = {
    '8.01': t801, '8.02': t802, '8.03': t803,
    '8.04': lambda k: poses_task('8.04', [m.POSE], 'polar controller to (1.5, 0.5, 90 deg)', k),
    '8.05': t805,
    '8.06': lambda k: corners_task('8.06', m.shapes.square_corners(), 'square, 1.5 m', k),
    '8.07': t807, '8.08': t808,
    '8.09': lambda k: corners_task('8.09', m.WAYPOINTS, 'five waypoints', k, 'max_waypoint_err_m'),
    '8.10': t810,
    '8.11': lambda k: poses_task('8.11', m.OUT_AND_BACK, 'out to (2, 1, 180 deg) and back to the start', k),
}


def check(rule, v):
    (op, x), = rule.items()
    return {'max': lambda: v <= x, 'min': lambda: v >= x, 'equals': lambda: v == x, 'abs_max': lambda: abs(v) <= x}[op]()


def rule_text(key, rule):
    (op, x), = rule.items()
    return {'max': f'{key} <= {x}', 'min': f'{key} >= {x}', 'equals': f'{key} == {x}', 'abs_max': f'|{key}| <= {x}'}[op]


def run(tid, keep):
    t0 = time.time()
    metrics, outputs = TASKS[tid](keep)
    acc = ACC[tid]
    checks = {k: dict(value=metrics[k], rule=rule_text(k, r), passed=check(r, metrics[k])) for k, r in acc['checks'].items()}
    res = dict(id=tid, title=acc['title'], status='verified' if all(c['passed'] for c in checks.values()) else 'failed',
               metrics=metrics, acceptance='; '.join(c['rule'] for c in checks.values()), checks=checks,
               command=f'python3 run.py {tid}' + (f'; scored again from results/{tid}.csv with --rescore' if RESCORE else ''), outputs=outputs, wall_s=round(time.time() - t0, 1))
    (RES / f'{tid}.json').write_text(json.dumps(res, indent=2) + '\n')
    print(f"{tid} {res['status']}: " + ', '.join(f"{k}={c['value']:.3f}" if isinstance(c['value'], float) else f"{k}={c['value']}"
                                          for k, c in checks.items()))


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('task', choices=[*TASKS, 'all'])
    ap.add_argument('--keep', action='store_true', help='use the running stack, do not restart')
    ap.add_argument('--rescore', action='store_true', help='score results/<id>.csv again without the simulator')
    a = ap.parse_args()
    global RESCORE
    RESCORE = a.rescore
    rclpy.init()
    try:
        for tid in (TASKS if a.task == 'all' else [a.task]):
            run(tid, a.keep)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
