#!/usr/bin/env python3
"""Score one recorded A-to-B run:  python3 projects/p12_tune-nav2/score.py BAG_DIR --setting NAME
Runs src/go2_nav_sim/scripts/bag_metrics.py on the bag and adds what tuning needs: the mission outcome (BAG_DIR.log), the length
of the first plan, how many plans were published, how long the robot took to get 1 m away from A, and the clearance of the true path (robot centre, and the 0.65 x 0.30 m body)
to the obstacles in maps/arena.pgm. Writes BAG_DIR.json, and BAG_DIR.npz (first global costmap, first plan, true path, its time since the first plan) for plot.py."""
import argparse, json, math, pathlib, subprocess, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tuning import body_clearance, distance_grid, load_map, lookup, parse_mission, path_length

ROOT = pathlib.Path(__file__).resolve().parents[2]
A = (-4.5, -3.5)
BODY = [[0.325, 0.15], [0.325, -0.15], [-0.325, -0.15], [-0.325, 0.15]]
stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(bag):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    gt, gt_t, plans, plan0_t, cmap = [], [], [], None, None
    while r.has_next():
        topic, data, _ = r.read_next()                                  # times below are header stamps (simulated time)
        if topic == '/odom_gt':
            m = deserialize_message(data, types[topic]); p = m.pose.pose
            gt.append((p.position.x + A[0], p.position.y + A[1], yaw(p.orientation))); gt_t.append(stamp(m.header))
        elif topic == '/plan':
            m = deserialize_message(data, types[topic]); plan0_t = stamp(m.header) if plan0_t is None else plan0_t
            plans.append([(s.pose.position.x, s.pose.position.y) for s in m.poses])
        elif topic == '/global_costmap/costmap_raw' and cmap is None and plans:     # the costmap the first plan was made on
            m = deserialize_message(data, types[topic]).metadata
            c = np.frombuffer(deserialize_message(data, types[topic]).data, np.uint8).reshape(m.size_y, m.size_x)
            cmap = (c, (m.origin.position.x, m.origin.position.y), m.resolution)
    return np.array(gt), np.array(gt_t), plans, plan0_t, cmap


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('--setting', required=True); a = ap.parse_args()
    bag = str(pathlib.Path(a.bag).resolve())
    out = subprocess.run([sys.executable, str(ROOT / 'src/go2_nav_sim/scripts/bag_metrics.py'), bag], capture_output=True, text=True)
    res = dict(setting=a.setting, run=pathlib.Path(bag).name)
    log = pathlib.Path(bag + '.log')
    res.update(parse_mission(log.read_text() if log.exists() else ''))
    res.update(json.loads(out.stdout) if out.returncode == 0 else dict(bag_metrics_error=out.stderr.strip().splitlines()[-1:]))
    gt, gt_t, plans, plan0_t, cmap = read(bag)
    away = np.hypot(gt[:, 0] - A[0], gt[:, 1] - A[1]) > 1.0            # first moment 1 m from A, counted from the first plan
    if plans and away.any():
        res['leave_A_s'] = round(float(gt_t[np.argmax(away)] - plan0_t), 1)
    occ, origin, r = load_map(ROOT / 'src/go2_nav_sim/maps/arena.yaml'); dist = distance_grid(occ, r)
    moving = gt[::5]                                                     # 10 Hz of the 50 Hz ground truth is enough
    res.update(plans=len(plans), first_plan_m=round(path_length(plans[0]), 2) if plans else None,
               min_centre_clearance_m=round(float(lookup(dist, origin, r, moving[:, :2]).min()), 3),
               min_body_clearance_m=round(body_clearance(moving, BODY, dist, origin, r), 3))
    if plans:
        res['first_plan_min_clearance_m'] = round(float(lookup(dist, origin, r, np.array(plans[0])).min()), 3)
    pathlib.Path(bag + '.json').write_text(json.dumps(res, indent=2) + '\n')
    if cmap is not None:
        np.savez_compressed(bag + '.npz', costmap=cmap[0], origin=cmap[1], res=cmap[2], plan=np.array(plans[0]), gt=gt, t=gt_t - plan0_t)
    print(json.dumps(res))


if __name__ == '__main__':
    main()
