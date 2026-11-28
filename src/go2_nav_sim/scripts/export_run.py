#!/usr/bin/env python3
"""Export a recorded run to compact files for the website (no ROS runtime needed besides rosbag2_py).

usage: export_run.py BAG OUT_DIR [--snapshots N]

Writes
  run.json        10 Hz timeline in the world frame: ground truth, odometry (aligned to ground truth on the first sample, as in
                  bag_metrics.py), AMCL, commanded velocity, errors, Nav2 plans, arena geometry, metrics
  imu.json        IMU at 25 Hz (accelerometer, gyro, roll, pitch)
  slam_map.png    the final /slam/map in the world frame (+ slam_map.json with its placement)
  snap_K.jpg / snap_K_depth.png / snap_K.bin    (with --snapshots) onboard RGB, colour-mapped depth and the lidar sweep
                  (float32 x y z, lidar frame) at N evenly spaced instants, listed in run.json under "snapshots"
"""
import argparse, importlib.util, json, math, pathlib
import cv2, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('arena', HERE / 'make_arena.py'); assert spec is not None and spec.loader is not None; arena = importlib.util.module_from_spec(spec); spec.loader.exec_module(arena)
A, B = arena.A, arena.B
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
TURBO = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(256, 3)        # BGR


def rnd(v, n=3):
    return [round(float(x), n) for x in v]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('out'); ap.add_argument('--snapshots', type=int, default=0)
    ap.add_argument('--metrics'); a = ap.parse_args()
    out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    info = rosbag2_py.Info().read_metadata(a.bag, 'mcap'); t0 = info.starting_time.nanoseconds; dur = info.duration.nanoseconds * 1e-9
    snap_times = [dur * (k + 1) / (a.snapshots + 1) for k in range(a.snapshots)]; snaps = []; pending = {}
    cur = {}; rows = []; imu = []; plans = []; nxt = 0.0; nxt_imu = 0.0; last_map = None; odom0 = None
    while r.has_next():
        topic, data, tns = r.read_next(); t = (tns - t0) * 1e-9
        if topic not in ('/odom_gt', '/odom', '/amcl_pose', '/cmd_vel', '/plan', '/imu/data', '/slam/map', '/camera/image_raw',
                         '/camera/depth/image_raw', '/lidar/points'):
            continue
        m = deserialize_message(data, types[topic])
        if topic == '/odom_gt':
            p = m.pose.pose; cur['gt'] = (p.position.x + A[0], p.position.y + A[1], yaw_of(p.orientation))
        elif topic == '/odom':
            p = m.pose.pose; cur['odom'] = (p.position.x, p.position.y, yaw_of(p.orientation))
        elif topic == '/amcl_pose':
            p = m.pose.pose; cur['amcl'] = (p.position.x, p.position.y, yaw_of(p.orientation))
        elif topic == '/cmd_vel':
            cur['cmd'] = (m.linear.x, m.angular.z)
        elif topic == '/plan' and m.poses:
            pts = [(q.pose.position.x, q.pose.position.y) for q in m.poses][::4]
            if not plans or t - plans[-1]['t'] > 1.0:
                plans.append({'t': round(t, 2), 'pts': [rnd(p, 2) for p in pts]})
        elif topic == '/slam/map':
            last_map = (np.array(m.data, np.int8).reshape(m.info.height, m.info.width), m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
        elif topic == '/imu/data' and t >= nxt_imu:
            nxt_imu = t + 0.04; q = m.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)); pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
            imu.append([round(t, 2), *rnd((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z, m.angular_velocity.x,
                                            m.angular_velocity.y, m.angular_velocity.z, math.degrees(roll), math.degrees(pitch)), 3)])
        elif topic in ('/camera/image_raw', '/camera/depth/image_raw', '/lidar/points'):
            pending[topic] = m
        if snap_times and t >= snap_times[0] and all(k in pending for k in ('/camera/image_raw', '/camera/depth/image_raw', '/lidar/points')) and 'gt' in cur:
            k = len(snaps); snap_times.pop(0)
            im = pending['/camera/image_raw']; rgb = np.frombuffer(im.data, np.uint8).reshape(im.height, im.width, 3)
            cv2.imwrite(str(out / f'snap_{k}.jpg'), rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 88])
            dm = pending['/camera/depth/image_raw']; d = np.frombuffer(dm.data, np.float32).reshape(dm.height, dm.width); ok = np.isfinite(d) & (d > 0)
            n = np.zeros(d.shape, np.uint8); n[ok] = np.clip((d[ok] - 0.3) / 7.7 * 255, 0, 255).astype(np.uint8)
            col = TURBO[255 - n]; col[~ok] = (29, 22, 18); cv2.imwrite(str(out / f'snap_{k}_depth.png'), col)
            pc = pending['/lidar/points']; P = np.frombuffer(pc.data, np.float32).reshape(-1, pc.point_step // 4)[:, :3]; P = P[np.isfinite(P).all(1)]
            P.astype(np.float32).tofile(out / f'snap_{k}.bin')
            snaps.append({'t': round(t, 2), 'pose': rnd(cur['gt']), 'points': int(len(P)), 'depth_valid': round(float(ok.mean()), 3),
                          'depth_range_m': [0.3, 8.0]})
        if t >= nxt and 'gt' in cur:
            nxt = t + 0.1
            if 'odom' in cur and odom0 is None:
                odom0 = (cur['odom'], cur['gt'])
            row = {'t': round(t, 2), 'gt': rnd(cur['gt'])}
            if odom0:
                (ox, oy, oa), (gx, gy, ga) = odom0; d = ga - oa; c, s = math.cos(d), math.sin(d); ex, ey = cur['odom'][0] - ox, cur['odom'][1] - oy
                row['odom'] = rnd((gx + c * ex - s * ey, gy + s * ex + c * ey, cur['odom'][2] + d))
            if 'amcl' in cur: row['amcl'] = rnd(cur['amcl'])
            if 'cmd' in cur: row['cmd'] = rnd(cur['cmd'], 2)
            rows.append(row)
    # final SLAM map, placed in the world frame through the estimator's start pose (as in render_dashboard.py)
    if last_map is not None and odom0:
        g, res, mx, my = last_map; (ox, oy, _), (gx, gy, _) = odom0
        rgba = np.zeros((*g.shape, 4), np.uint8); rgba[g == 0] = (31, 159, 209, 70); rgba[g >= 50] = (31, 159, 209, 255)
        cv2.imwrite(str(out / 'slam_map.png'), cv2.cvtColor(rgba[::-1], cv2.COLOR_RGBA2BGRA))
        (out / 'slam_map.json').write_text(json.dumps({'origin': rnd((gx - ox + mx, gy - oy + my)), 'resolution': res, 'width': g.shape[1], 'height': g.shape[0]}))
    metrics = json.loads(pathlib.Path(a.metrics).read_text()) if a.metrics else {}
    geo = {'A': A, 'B': B, 'walls': [list(w) for w in arena.WALLS], 'boxes': [list(b[:5]) for b in arena.BOXES], 'cylinders': [list(c[:4]) for c in arena.CYLS]}
    (out / 'run.json').write_text(json.dumps({'duration': round(dur, 2), 'metrics': metrics, 'arena': geo, 'samples': rows, 'plans': plans, 'snapshots': snaps},
                                             separators=(',', ':')))
    (out / 'imu.json').write_text(json.dumps({'fields': ['t', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'roll_deg', 'pitch_deg'], 'rows': imu}, separators=(',', ':')))
    print(f'{out}: {len(rows)} samples, {len(plans)} plans, {len(imu)} imu rows, {len(snaps)} snapshots')


if __name__ == '__main__':
    main()
