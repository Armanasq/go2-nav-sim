#!/usr/bin/env python3
"""Walk the Go2 to a goal with no map and no Nav2: steer on /scan and /odom only (reactive.py), log the run, score it.
usage: python3 projects/p06_avoid-obstacles/avoid.py [--method vfh|apf] [--goal X Y] [--name NAME] [--timeout S]
                                                      [--set KEY=VALUE ...]
The goal is in the world frame of make_arena.py (default: marker B). /odom_gt is read once at the start, to place the odom
frame in the world (the same shortcut go_to_goal.py uses to start AMCL), and afterwards only logged for scoring. The run ends
when the estimate is within goal_tol of the goal (REACHED), when the distance has not shrunk by 0.25 m in 30 s (STUCK), or
at the timeout. Every scan is one row of projects/p06_avoid-obstacles/results/NAME.csv."""
import argparse, csv, math, pathlib, sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import arena2d  # noqa: E402  (arena geometry, used only to score the run)
import reactive  # noqa: E402

A = arena2d.arena.A


def pose_of(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Avoid(Node):
    def __init__(self, a):
        super().__init__('avoid', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a, self.p = a, dict(reactive.P, **{k: float(v) for k, v in (kv.split('=') for kv in a.set)})
        self.odom = self.gt = self.anchor = self.t0 = None
        self.t_scan, self.result, self.prog, self.prev = None, None, reactive.Progress(), None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', pose_of(m)), 10)
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, 10)
        self.create_timer(0.1, self.watchdog)
        (HERE / 'results').mkdir(exist_ok=True)
        self.f = open(HERE / 'results' / f'{a.name}.csv', 'w', newline='')
        self.log = csv.writer(self.f)
        self.log.writerow(['t', 'x', 'y', 'yaw', 'gt_x', 'gt_y', 'gt_yaw', 'v', 'w', 'th', 'front', 'dist'])
        self.get_logger().info(f'{a.method}: goal ({a.goal[0]:.2f}, {a.goal[1]:.2f}) in the world frame')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        x, y, yaw = pose_of(m)
        self.gt = (x + A[0], y + A[1], yaw)                      # /odom_gt is relative to A; back to the world frame

    def on_scan(self, m):
        if self.result or self.odom is None or self.gt is None:
            return
        t = self.now()
        self.t_scan = t
        if self.anchor is None:                                  # once: where the odom frame sits in the world
            self.anchor, self.t0 = reactive.compose(self.gt, reactive.inverse(self.odom)), t
            self.get_logger().info('start at ({:.2f}, {:.2f}, {:.0f} deg)'.format(*self.gt[:2], math.degrees(self.gt[2])))
        pose = reactive.compose(self.anchor, self.odom)
        v, w, info = reactive.step(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                                   pose, self.a.goal, self.a.method, self.prev, self.p)
        self.prev = info['dir']
        if info['dist'] < self.p['goal_tol']:
            self.result, v, w = 'REACHED', 0.0, 0.0
        elif self.prog.stuck(t, info['dist']):
            self.result, v, w = 'STUCK', 0.0, 0.0
        elif t - self.t0 > self.a.timeout:
            self.result, v, w = 'TIMEOUT', 0.0, 0.0
        self.send(v, w)
        th = math.nan if info['th'] is None else info['th']
        self.log.writerow([f'{t - self.t0:.2f}', *(f'{c:.4f}' for c in pose + self.gt),
                           f'{v:.3f}', f'{w:.3f}', f'{th:.3f}', f'{info["front"]:.3f}', f'{info["dist"]:.3f}'])

    def watchdog(self):
        if self.t_scan is not None and not self.result and self.now() - self.t_scan > 0.5:
            self.send(0.0, 0.0)                                  # no scan for 5 periods: do not walk blind

    def send(self, v, w):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.pub.publish(tw)

    def report(self):
        d = np.loadtxt(self.f.name, delimiter=',', skiprows=1, ndmin=2)
        t, est, gt = d[-1, 0], d[:, 1:3], d[:, 4:6]
        path = np.hypot(*np.diff(gt, axis=0).T).sum()
        print(f'RESULT {self.result} method={self.a.method} time={t:.1f}s path={path:.2f}m '
              f'min_clearance={arena2d.clearance(*gt.T).min():.2f}m '
              f'true_dist_to_goal={math.dist(gt[-1], self.a.goal):.2f}m odom_err_end={math.dist(gt[-1], est[-1]):.2f}m',
              flush=True)


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('--method', choices=('vfh', 'apf'), default='vfh')
    ap.add_argument('--goal', type=float, nargs=2, default=arena2d.arena.B)
    ap.add_argument('--name', default='run')
    ap.add_argument('--timeout', type=float, default=240.0)
    ap.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE', help='override a value of reactive.P')
    a = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = Avoid(a)
    try:
        while rclpy.ok() and not node.result:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.result = 'INTERRUPTED'
    for _ in range(5):                                           # CHAMP keeps walking on the last command: end on zero
        node.send(0.0, 0.0)
    node.f.close()
    node.report()


if __name__ == '__main__':
    main()
