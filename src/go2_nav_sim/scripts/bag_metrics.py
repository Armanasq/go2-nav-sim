#!/usr/bin/env python3
"""Evaluation numbers from a recorded run.   usage: bag_metrics.py BAG_DIR [--json OUT.json]

Ground truth is the simulator pose (/odom_gt, shifted by the start marker A) and is never used by the robot.
  odometry : /odom (lidar odometry + IMU, EKF) rigidly aligned to ground truth on the first sample, because the robot settles a few
             centimetres after spawning. ATE = RMSE of position error; drift = final error / path length.
  AMCL     : /amcl_pose is already expressed in the ground-truth (map) frame, no alignment."""
import argparse, json, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

A = (-4.5, -3.5)
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('--json'); a = ap.parse_args()
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    cur, gt, odo, amcl = {}, [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic not in ('/odom', '/odom_gt', '/amcl_pose'):
            continue
        p = deserialize_message(data, types[topic]).pose.pose; s = (p.position.x, p.position.y, yaw(p.orientation))
        if topic == '/odom_gt':
            g = (s[0] + A[0], s[1] + A[1], s[2]); gt.append((t * 1e-9, *g))
            if 'odom' in cur: odo.append((*cur['odom'], *g))
            if 'amcl' in cur: amcl.append((*cur['amcl'][:2], *g[:2]))
        elif topic == '/odom':
            cur['odom'] = s
        else:
            cur['amcl'] = s
    g, o, m = np.array(gt), np.array(odo), np.array(amcl)
    path = float(np.hypot(*np.diff(g[:, 1:3], axis=0).T).sum()); dur = float(g[-1, 0] - g[0, 0])
    d = o[0, 5] - o[0, 2]; c, s = math.cos(d), math.sin(d)                      # align estimate to ground truth on the first sample
    ex, ey = o[:, 0] - o[0, 0], o[:, 1] - o[0, 1]
    ax, ay = o[0, 3] + c * ex - s * ey, o[0, 4] + s * ex + c * ey
    e_odo = np.hypot(ax - o[:, 3], ay - o[:, 4]); e_amcl = np.hypot(m[:, 0] - m[:, 2], m[:, 1] - m[:, 3])
    res = dict(duration_s=round(dur, 1), path_m=round(path, 2), mean_speed_ms=round(path / dur, 3),
               odom_ate_rmse_cm=round(100 * float(np.sqrt((e_odo ** 2).mean())), 1), odom_final_err_cm=round(100 * float(e_odo[-1]), 1),
               odom_drift_pct=round(100 * float(e_odo[-1]) / path, 2), amcl_rmse_cm=round(100 * float(np.sqrt((e_amcl ** 2).mean())), 1),
               amcl_max_cm=round(100 * float(e_amcl.max()), 1), goal_error_m=round(float(math.hypot(g[-1, 1] - 4.0, g[-1, 2] - 3.0)), 2))
    print(json.dumps(res, indent=2))
    if a.json:
        open(a.json, 'w').write(json.dumps(res, indent=2) + '\n')


if __name__ == '__main__':
    main()
