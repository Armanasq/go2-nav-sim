#!/usr/bin/env python3
"""Live odometry evaluation against simulator ground truth. The estimate is rigidly aligned to ground truth on the first sample
(the robot settles a few cm after spawning, so the two origins differ slightly).
usage: odom_eval.py [seconds=30] [estimate_topic=/odom]   -> prints ATE RMSE, final error, path length, drift [% of path]"""
import math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init(); n = Node('odom_eval'); est, gt, rows = {}, {}, []
    n.create_subscription(Odometry, sys.argv[2] if len(sys.argv) > 2 else '/odom', lambda m: est.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y, a=yaw(m.pose.pose.orientation)), 10)
    n.create_subscription(Odometry, '/odom_gt', lambda m: gt.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y, a=yaw(m.pose.pose.orientation)), 10)
    t0 = time.time(); last = 0
    while time.time() - t0 < float(sys.argv[1] if len(sys.argv) > 1 else 30):
        rclpy.spin_once(n, timeout_sec=0.05)
        if est and gt and time.time() - last > 0.2:
            last = time.time(); rows.append((gt['x'], gt['y'], gt['a'], est['x'], est['y'], est['a']))
    r = np.array(rows)
    d = r[0, 2] - r[0, 5]; c, s = np.cos(d), np.sin(d)                      # rotate estimate by the first yaw offset, then translate onto GT
    ex, ey = r[:, 3] - r[0, 3], r[:, 4] - r[0, 4]
    r[:, 3], r[:, 4], r[:, 5] = r[0, 0] + c * ex - s * ey, r[0, 1] + s * ex + c * ey, r[:, 5] + d
    e = np.hypot(r[:, 0] - r[:, 3], r[:, 1] - r[:, 4]); ea = np.degrees(np.abs((r[:, 2] - r[:, 5] + np.pi) % (2 * np.pi) - np.pi))
    path = np.hypot(*np.diff(r[:, :2], axis=0).T).sum()
    print(f'samples {len(r)}  path {path:.2f} m  ATE RMSE {np.sqrt((e**2).mean()):.3f} m  max {e.max():.3f} m  final {e[-1]:.3f} m  '
          f'yaw err RMSE {np.sqrt((ea**2).mean()):.2f} deg  drift {100*e[-1]/max(path,1e-6):.2f} % of path')


if __name__ == '__main__':
    main()
