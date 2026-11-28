"""Leg odometry offline, from a bag: base velocity from the stance feet, compared with ground truth (chapter 9, Task 1).

    v_base = -omega x p_f - J(q) dq      (p_f foot position in base_link, omega the gyro, J the foot Jacobian)

Stance is guessed without contact sensors: of the two trot diagonals, the pair whose feet are lower in base_link is on the
ground. Record the bag with
    ros2 bag record -o bags/legs /joint_states /imu/data /odom_gt /odom/raw /robot_description
and run
    /usr/bin/python3 docs/img/c09-leg-odometry.py bags/legs docs/img/c09-leg-odometry.png
"""
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LEGS = ('lf', 'rf', 'lh', 'rh')


def read_bag(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    out = {t: [] for t in topics}
    while r.has_next():
        name, data, _ = r.read_next()
        out[name].append(deserialize_message(data, get_message(types[name])))
    return out


def leg_chains(urdf):
    """Joint origin and axis for hip, upper leg, lower leg and the foot offset, per leg, all from the URDF."""
    joints = {j.get('name'): j for j in ET.fromstring(urdf).findall('joint')}

    def origin(j):
        o = j.find('origin')
        assert o is None or np.allclose([float(v) for v in o.get('rpy', '0 0 0').split()], 0), 'rotated joint not handled'
        return np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3)

    def axis(j):
        a = j.find('axis')
        assert a is not None, 'joint without axis'
        return np.array([float(v) for v in a.attrib['xyz'].split()])
    return {leg: ([(origin(joints[f'{leg}_{p}_joint']), axis(joints[f'{leg}_{p}_joint']))
                   for p in ('hip', 'upper_leg', 'lower_leg')], origin(joints[f'{leg}_foot_joint'])) for leg in LEGS}


def rot(axis, a):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k


def foot_and_jacobian(chain, q):
    """Foot position in base_link (base_link = trunk here) and its 3x3 Jacobian J_i = z_i x (p_f - o_i)."""
    links, foot = chain
    R, p, origins, axes = np.eye(3), np.zeros(3), [], []
    for (offset, axis), qi in zip(links, q):
        p = p + R @ offset
        origins.append(p); axes.append(R @ axis)
        R = R @ rot(axis, qi)
    pf = p + R @ foot
    return pf, np.stack([np.cross(z, pf - o) for z, o in zip(axes, origins)], 1)


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main(bag, png):
    d = read_bag(bag, ['/robot_description', '/joint_states', '/imu/data', '/odom_gt', '/odom/raw'])
    chains = leg_chains(d['/robot_description'][0].data)
    st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    imu_t = np.array([st(m) for m in d['/imu/data']])
    gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in d['/imu/data']])

    js = [m for m in d['/joint_states'] if imu_t[0] <= st(m) <= imu_t[-1]]
    t = np.array([st(m) for m in js])
    P, Jqd = np.zeros((len(js), 4, 3)), np.zeros((len(js), 4, 3))      # foot positions and J(q) dq from the velocity field
    for i, m in enumerate(js):
        pos, vel = dict(zip(m.name, m.position)), dict(zip(m.name, m.velocity))
        for k, leg in enumerate(LEGS):
            names = [f'{leg}_{p}_joint' for p in ('hip', 'upper_leg', 'lower_leg')]
            P[i, k], J = foot_and_jacobian(chains[leg], [pos[n] for n in names])
            Jqd[i, k] = J @ np.array([vel[n] for n in names])
    # J(q) dq is the time derivative of p_f(q). The velocity field of /joint_states is chatter from the effort controller
    # (several rad/s while the robot stands still), so differentiate the foot positions instead.
    Pdot = np.gradient(P, t, axis=0)
    still = slice(0, int(np.searchsorted(t, t[0] + 5)))                     # first 5 s: the robot stands
    print(f'standing, first 5 s: mean J(q)dq from the velocity field {np.abs(Jqd[still]).mean():.3f} m/s, '
          f'from differentiated positions {np.abs(Pdot[still]).mean():.3f} m/s')
    w = np.stack([np.interp(t, imu_t, gyro[:, k]) for k in range(3)], 1)
    vfeet = -np.cross(w[:, None, :], P) - Pdot                              # base velocity implied by each foot
    lower_first = (P[:, 0, 2] + P[:, 3, 2]) < (P[:, 1, 2] + P[:, 2, 2])   # lf+rh lower than rf+lh: that pair is on the ground
    v = np.where(lower_first[:, None], vfeet[:, [0, 3]].mean(1), vfeet[:, [1, 2]].mean(1))

    gt = d['/odom_gt']
    gt_t = np.array([st(m) for m in gt])
    gt_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in gt])
    gt_yaw = np.unwrap([yaw_of(m.pose.pose.orientation) for m in gt])
    gt_vw = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y] for m in gt])          # p3d: world frame
    gt_fwd = gt_vw[:, 0] * np.cos(gt_yaw) + gt_vw[:, 1] * np.sin(gt_yaw)                         # into the heading

    # Integrate: yaw from the gyro, position from the leg velocity rotated by that yaw; anchored on the true start.
    yaw = gt_yaw[0] + np.concatenate([[0], np.cumsum(np.diff(imu_t) * gyro[1:, 2])])
    yaw_t = np.interp(t, imu_t, yaw)
    dt = np.diff(t, prepend=t[0])
    vx_w = v[:, 0] * np.cos(yaw_t) - v[:, 1] * np.sin(yaw_t)
    vy_w = v[:, 0] * np.sin(yaw_t) + v[:, 1] * np.cos(yaw_t)
    i0 = np.searchsorted(t, gt_t[0])
    leg_xy = gt_xy[0] + np.cumsum(np.stack([vx_w * dt, vy_w * dt], 1)[i0:], 0)
    lt = t[i0:]

    # Compare on 0.1 s bins (one gait half-cycle is ~0.13 s at CHAMP's settings, so the bin removes the swing switch spikes).
    bins = np.arange(gt_t[0], gt_t[-1], 0.1)
    idx = np.digitize(t, bins)
    leg_fwd = np.array([v[idx == k, 0].mean() if np.any(idx == k) else np.nan for k in range(1, len(bins))])
    mid = bins[:-1] + 0.05
    true_fwd = np.interp(mid, gt_t, gt_fwd)
    ok = ~np.isnan(leg_fwd)
    rmse_v = np.sqrt(np.mean((leg_fwd[ok] - true_fwd[ok]) ** 2))
    path = np.sum(np.linalg.norm(np.diff(gt_xy, axis=0), axis=1))
    gt_at_leg = np.stack([np.interp(lt, gt_t, gt_xy[:, k]) for k in range(2)], 1)
    err = np.linalg.norm(leg_xy - gt_at_leg, axis=1)
    raw = d['/odom/raw']
    raw_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in raw])
    raw_path = np.sum(np.linalg.norm(np.diff(raw_xy, axis=0), axis=1))
    print(f'samples {len(t)}  duration {gt_t[-1] - gt_t[0]:.1f} s  ground-truth path {path:.2f} m')
    print(f'leg forward velocity: mean {np.nanmean(leg_fwd[true_fwd > 0.1]):.3f} m/s vs ground truth '
          f'{true_fwd[true_fwd > 0.1].mean():.3f} m/s while walking; RMSE {rmse_v:.3f} m/s (0.1 s bins)')
    print(f'leg + gyro path: final error {err[-1]:.2f} m ({100 * err[-1] / path:.1f} % of path), max {err.max():.2f} m')
    print(f'CHAMP /odom/raw: {len(raw)} messages, path {raw_path:.2f} m')

    plt.rcParams.update({'font.size': 11})
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={'width_ratios': [1.5, 1]})
    t0 = gt_t[0]
    a.plot(gt_t - t0, gt_fwd, color='#7a7f8a', lw=2, label='ground truth (/odom_gt)')
    a.plot(mid - t0, leg_fwd, color='#1f77b4', lw=1.4, label='legs: −ω×p − ṗ_f, lower diagonal pair')
    a.plot(np.array([st(m) for m in raw]) - t0, [m.twist.twist.linear.x for m in raw], color='#c0392b', lw=1.2,
           label='CHAMP /odom/raw')
    a.set_xlabel('sim time since start of recording (s)'); a.set_ylabel('forward velocity (m/s)')
    a.set_title(f'Forward velocity: RMSE {rmse_v:.3f} m/s', loc='left'); a.grid(alpha=.3); a.legend(loc='lower right', fontsize=9)
    b.plot(gt_xy[:, 0], gt_xy[:, 1], color='#7a7f8a', lw=3, label='ground truth')
    b.plot(leg_xy[:, 0], leg_xy[:, 1], color='#1f77b4', lw=1.5, label='legs + gyro yaw')
    b.plot(*raw_xy[-1], 'x', color='#c0392b', ms=10, mew=2, label=f'CHAMP /odom/raw (path {raw_path:.2f} m)')
    b.set_aspect('equal'); b.set_xlabel('x (m)'); b.set_ylabel('y (m)'); b.grid(alpha=.3); b.legend(fontsize=9)
    b.set_title(f'Path: final error {err[-1]:.2f} m over {path:.1f} m', loc='left')
    fig.tight_layout(); fig.savefig(png, dpi=110)


if __name__ == '__main__':
    main(*sys.argv[1:3])
