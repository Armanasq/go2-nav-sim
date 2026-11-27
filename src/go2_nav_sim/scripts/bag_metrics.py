#!/usr/bin/env python3
"""Evaluation numbers from a recorded run.   usage: bag_metrics.py BAG_DIR [--json OUT.json]

Ground truth is the simulator pose (/odom_gt, shifted by the start marker A). It is never used by the robot, except once by
go_to_goal.py to initialise AMCL. All pairing is by message header stamp (simulation time): ground truth is interpolated at the
stamp of each estimate, so a slowly updating estimate is not compared against a ground truth that has moved on.

  odometry  /odom (lidar odometry + IMU, EKF), in its own odom frame.
            ATE aligned  : RMSE after the best rigid SE(2) alignment (Umeyama, no scale), the standard definition.
            ATE anchored : RMSE after aligning only the first pose; it includes the drift that alignment would hide.
            RPE          : translational error over 1 m segments of travelled ground-truth path, in % of 1 m.
  AMCL      /amcl_pose is already in the ground-truth (map) frame; scored at its own update stamps only.
  goal      distance from B at the end of the recording. Nav2's xy_goal_tolerance is 0.25 m, so values near that reflect the
            tolerance plus localisation error, not navigation accuracy.
Sturm et al., IROS 2012, define ATE and RPE; this is their planar version."""
import argparse, json, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

A, B = (-4.5, -3.5), (4.0, 3.0)
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9


def umeyama_se2(P, Q):
    """R, t minimising sum |R p + t - q|^2 (no scale)."""
    mp, mq = P.mean(0), Q.mean(0); U, _, Vt = np.linalg.svd((Q - mq).T @ (P - mp))
    R = U @ np.diag([1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R, mq - R @ mp


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('--json'); a = ap.parse_args()
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    gt, odo, amcl = [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in ('/odom', '/odom_gt', '/amcl_pose'):
            continue
        m = deserialize_message(data, types[topic]); p = m.pose.pose; row = (stamp(m.header), p.position.x, p.position.y, yaw(p.orientation))
        (gt if topic == '/odom_gt' else odo if topic == '/odom' else amcl).append(row)
    g = np.array(sorted(gt)); g[:, 1] += A[0]; g[:, 2] += A[1]
    gi = lambda T: np.stack([np.interp(T, g[:, 0], g[:, 1]), np.interp(T, g[:, 0], g[:, 2])], 1)
    o = np.array(sorted(odo)); o = o[(o[:, 0] >= g[0, 0]) & (o[:, 0] <= g[-1, 0])]; G = gi(o[:, 0])
    R, t = umeyama_se2(o[:, 1:3], G); e_al = np.linalg.norm(o[:, 1:3] @ R.T + t - G, axis=1)
    th = np.interp(o[0, 0], g[:, 0], np.unwrap(g[:, 3])) - o[0, 3]; R0 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    e_an = np.linalg.norm((o[:, 1:3] - o[0, 1:3]) @ R0.T + G[0] - G, axis=1)
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(G, axis=0), axis=1))]); rpe = []      # 1 m segments along the true path
    for i in range(len(s)):
        j = np.searchsorted(s, s[i] + 1.0)
        if j < len(s):
            rpe.append(np.linalg.norm((o[j, 1:3] - o[i, 1:3]) @ R.T - (G[j] - G[i])))
    m = np.array(amcl); m = m[(m[:, 0] >= g[0, 0]) & (m[:, 0] <= g[-1, 0])]; e_am = np.linalg.norm(m[:, 1:3] - gi(m[:, 0]), axis=1)
    path = float(s[-1]); dur = float(g[-1, 0] - g[0, 0])
    res = dict(duration_s=round(dur, 1), path_m=round(path, 2), mean_speed_ms=round(path / dur, 3),
               odom_ate_aligned_cm=round(100 * float(np.sqrt((e_al ** 2).mean())), 1), odom_ate_anchored_cm=round(100 * float(np.sqrt((e_an ** 2).mean())), 1),
               odom_final_err_cm=round(100 * float(e_an[-1]), 1), odom_drift_pct=round(100 * float(e_an[-1]) / path, 2),
               odom_rpe_1m_pct=round(100 * float(np.sqrt(np.mean(np.square(rpe)))), 2) if rpe else None,
               amcl_rmse_cm=round(100 * float(np.sqrt((e_am ** 2).mean())), 1), amcl_max_cm=round(100 * float(e_am.max()), 1), amcl_updates=int(len(m)),
               goal_error_m=round(float(math.hypot(g[-1, 1] - B[0], g[-1, 2] - B[1])), 2), goal_tolerance_m=0.25)
    print(json.dumps(res, indent=2))
    if a.json:
        open(a.json, 'w').write(json.dumps(res, indent=2) + '\n')


if __name__ == '__main__':
    main()
