#!/usr/bin/env python3
"""Repeat the stop in front of the wood pallet at (2.5, -2.5) and measure it on ground truth.
For each run: walk (keeping the lane y = -2.5, the pallet's centre line, and heading +x) until the true gap to the pallet's west
face is `--start` m, stand still, run stop_at_obstacle.py, record /odom_gt, /front_range and /cmd_vel, and report where the robot really stopped.
usage: python3 projects/p05_sense-and-stop/measure_stops.py --speeds 0.1,0.2,0.3 --runs 3 --out ~/p05_runs [-- -p lead:=0.05]
       python3 projects/p05_sense-and-stop/measure_stops.py --goto-start    (only walk to the start gap and stand)
Arguments after `--` go to stop_at_obstacle.py. Needs the stack and sense.py running, and the robot west of the pallet, facing +x."""
import argparse, csv, math, os, pathlib, re, subprocess, sys, time

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src/go2_nav_sim/scripts'))
import make_arena as arena
sys.path.insert(0, str(HERE))
from obstacle import FRONT

_px, LANE_Y, _sx = arena.BOXES[2][:3]          # the wood pallet: centre (2.5, -2.5), 1.2 m square
FACE_X = _px - _sx / 2                         # its west face, x = 1.9
STILL = 0.02                                   # m/s: below this the robot counts as standing (as in project 2)


def clip(v, m):
    return max(-m, min(m, v))


class Rec(Node):
    def __init__(self):
        super().__init__('measure_stops', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.gt, self.rng, self.cmd = [], [], []
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, 50)
        self.create_subscription(Range, 'front_range', lambda m: self.rng.append((self.now(), stamp(m), m.range)), 10)  # type: ignore[reportAttributeAccessIssue]  # rclpy: create_subscription leaves msg_type untyped, so the callback argument is typed object
        self.create_subscription(Twist, 'cmd_vel', lambda m: self.cmd.append((self.now(), m.linear.x)), 10)  # type: ignore[reportAttributeAccessIssue]  # rclpy: create_subscription leaves msg_type untyped, so the callback argument is typed object

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = p.x + arena.A[0], p.y + arena.A[1]              # /odom_gt is relative to A; back to the world frame
        self.gt.append((stamp(m), x, y, yaw, FACE_X - x - FRONT * math.cos(yaw)))   # last field: true gap

    def send(self, vx, vy=0.0, wz=0.0):
        t = Twist(); t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz); self.pub.publish(t)

    def spin_for(self, dt):
        t0 = self.now()
        while self.now() - t0 < dt:
            rclpy.spin_once(self, timeout_sec=0.05)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def speed_at(gt, t, w=0.5):
    """True speed from positions over the window [t - w, t]."""
    s = [g for g in gt if t - w <= g[0] <= t]
    return math.hypot(s[-1][1] - s[0][1], s[-1][2] - s[0][2]) / (s[-1][0] - s[0][0]) if len(s) > 1 else math.nan


def reposition(n, start):
    """Walk to the start gap, keeping the lane and the heading, then stand still for 2 s."""
    while not n.gt:
        rclpy.spin_once(n, timeout_sec=0.1)
    while True:
        _, x, y, yaw, gap = n.gt[-1]
        if abs(gap - start) < 0.1 and abs(y - LANE_Y) < 0.05 and abs(yaw) < 0.05:
            break
        n.send(clip(gap - start, 0.2), clip(-1.0 * (y - LANE_Y), 0.15), clip(-1.5 * yaw, 0.3))
        n.spin_for(0.1)
    n.send(0.0)
    n.spin_for(2.0)


def run_once(n, v, extra):
    n.gt.clear(); n.rng.clear(); n.cmd.clear()
    cmd = [sys.executable, str(HERE / 'stop_at_obstacle.py'), '--ros-args', '-p', f'speed:={v}'] + extra
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    t_wall = time.time()
    while proc.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - t_wall > 300:
            proc.terminate()
    assert proc.stdout is not None
    out = proc.stdout.read()
    m = re.search(r'STOP t=(\S+) range=(\S+) gap=(\S+) age=(\S+)', out)
    n.send(0.0)
    n.spin_for(2.0)                                            # keep recording while the gait comes to rest
    if not m:
        why = re.search(r'TIMEOUT.*', out)
        print(f'v={v}: no STOP line ({why.group(0) if why else "no TIMEOUT line either"}); true gap at the end {n.gt[-1][4]:.3f} m')
        return None
    t_trig, rng, gap_sensed, age = map(float, m.groups())
    gt = n.gt
    i = min(range(len(gt)), key=lambda k: abs(gt[k][0] - t_trig))
    still = math.nan                                           # standing = speed stays below STILL from here to the end
    for t, _, _, _, _ in gt:
        if t <= t_trig:
            continue
        if not speed_at(gt, t) < STILL:
            still = math.nan
        elif math.isnan(still):
            still = t
    gap_end = gt[-1][4]
    return dict(speed_cmd=v, v_true=round(speed_at(gt, t_trig), 3), scan_age=age, gap_sensed=gap_sensed,
                gap_true_at_trigger=round(gt[i][4], 3), gap_final=round(gap_end, 3),
                coast=round(gt[i][4] - gap_end, 3), t_to_still=round(still - t_trig, 2),
                y_end=round(gt[-1][2], 3), yaw_end_deg=round(math.degrees(gt[-1][3]), 1)), (list(gt), list(n.rng), list(n.cmd))


def main():
    argv = sys.argv[1:]
    extra = argv[argv.index('--') + 1:] if '--' in argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', default='0.1,0.2,0.3'); ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--start', type=float, default=2.0, help='true gap (m) at which each run starts')
    ap.add_argument('--out', default=os.path.expanduser('~/p05_runs')); ap.add_argument('--tag', default='stop')
    ap.add_argument('--goto-start', action='store_true', help='walk to the start gap, stand, and exit')
    a = ap.parse_args(argv[:argv.index('--')] if '--' in argv else argv)
    out = pathlib.Path(a.out); (out / 'runs').mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[]); n = Rec()                            # args=[]: the arguments after -- are not ours
    if a.goto_start:
        reposition(n, a.start)
        _, x, y, yaw, gap = n.gt[-1]
        print(f'at the start: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):+.1f} deg, true gap {gap:.3f} m')
        return
    table = out / f'{a.tag}.csv'
    new = not table.exists()
    with open(table, 'a', newline='') as f:
        w = None
        for v in map(float, a.speeds.split(',')):
            for k in range(a.runs):
                reposition(n, a.start)
                r = run_once(n, v, extra)
                if r is None:
                    continue
                row, (gt, rng, cmd) = r
                print(' '.join(f'{key}={val}' for key, val in row.items()), flush=True)
                if w is None:
                    w = csv.DictWriter(f, fieldnames=['tag', 'run'] + list(row))
                    if new:
                        w.writeheader()
                w.writerow(dict(tag=a.tag, run=f'{v}_{k}', **row)); f.flush()
                with open(out / 'runs' / f'{a.tag}_{v}_{k}.csv', 'w', newline='') as g:
                    c = csv.writer(g); c.writerow(['source', 't', 'a', 'b', 'c'])
                    c.writerows(('gt', t, x, y, gap) for t, x, y, _, gap in gt)
                    c.writerows(('range', t, ts, r_, '') for t, ts, r_ in rng)
                    c.writerows(('cmd', t, vx, '', '') for t, vx in cmd)
    n.send(0.0)
    print(f'wrote {table}')


if __name__ == '__main__':
    main()
