#!/usr/bin/env python3
"""Planar LiDAR-inertial odometry: scan-to-local-map point-to-line ICP in SE(2).

  /lidar/points --TF--> gravity-aligned base_footprint frame -> ground removal (height band) -> voxel downsample
  prediction : constant velocity, yaw increment replaced by the integrated gyro (/imu/data)
  registration: Gauss-Newton on point-to-line residuals against a keyframed voxel map, Geman-McClure robust weights,
                map normals from local PCA (line-like neighbourhoods weighted up)
  output     : /lidar_odom (odom -> base_footprint, no TF; the EKF fuses it with the IMU and owns the TF)
Design notes: 2D is enough for a ground robot and is cheap (~5 ms/scan); the pitch/roll of a walking robot is removed by
projecting with the TF chain instead of ignoring it. Known limits: degenerate in long featureless corridors (no 1-DoF
detection implemented), no loop closure (slam_toolbox provides map->odom correction)."""
import collections, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


class LidarOdom(Node):
    def __init__(self):
        super().__init__('lidar_odom')
        P = self.declare_parameter
        self.voxel = P('voxel', 0.15).value
        self.min_h, self.max_h = P('min_height', 0.30).value, P('max_height', 1.0).value
        self.min_r, self.max_r = P('min_range', 0.5).value, P('max_range', 12.0).value
        self.map_radius = P('map_radius', 12.0).value
        self.key_t, self.key_r = P('keyframe_trans', 0.15).value, P('keyframe_rot', 0.10).value
        self.sigma = P('robust_sigma', 0.12).value           # Geman-McClure scale [m]
        self.fix_yaw = P('fix_yaw', True).value               # yaw from the integrated gyro, ICP solves translation only (gyro error ~1 deg per 6 s here)
        self.trans_prior = P('trans_prior', 10.0).value      # weak pull toward the predicted translation: only matters in directions the scan does not constrain
        self.reseed_after = P('reseed_after', 8).value       # consecutive rejected registrations before the local map is rebuilt
        self.base, self.lidar = P('base_frame', 'base_footprint').value, P('lidar_frame', 'lidar_link').value
        self.tf = Buffer(); TransformListener(self.tf, self, spin_thread=True)   # own thread: a blocking lookup in the callback must not starve TF input
        self.pub = self.create_publisher(Odometry, 'lidar_odom', 10)
        # Separate callback groups + a multi-threaded executor: a slow registration must not delay IMU processing.
        self.create_subscription(PointCloud2, 'lidar/points', self.on_cloud, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())
        self.pose = np.zeros(3)                               # x, y, yaw in odom
        self.vel = np.zeros(3)                                # body-frame vx, vy, wz
        self.t_last = None
        self.cum_yaw = 0.0; self.imu_prev = None; self.yaw_hist = collections.deque(maxlen=800)   # (t, integrated gyro z): lookups by time, so callback timing does not matter
        self.map = np.empty((0, 2)); self.nrm = np.empty((0, 2)); self.wgt = np.empty(0); self.tree = None
        self.key_pose = None; self.n = 0; self.fails = 0; self.rej = 0

    # ---- inputs -------------------------------------------------------------------------------------------------
    def on_imu(self, m):
        t, w = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z
        if self.imu_prev is not None and 0 < t - self.imu_prev[0] < 0.1:
            self.cum_yaw += 0.5 * (w + self.imu_prev[1]) * (t - self.imu_prev[0])        # trapezoid
        self.imu_prev = (t, w); self.yaw_hist.append((t, self.cum_yaw))

    def gyro_dyaw(self, t0, t1):
        h = np.array(self.yaw_hist)
        if len(h) < 10 or h[0, 0] > t0 or h[-1, 0] < t1 - 0.05:                           # not enough IMU coverage
            return None
        return float(np.interp(t1, h[:, 0], h[:, 1]) - np.interp(t0, h[:, 0], h[:, 1]))

    def to_base(self, msg):
        """PointCloud2 -> Nx3 in gravity-aligned base frame (TF carries gait roll/pitch)."""
        a = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)[:, :3]
        a = a[np.isfinite(a).all(1)]
        try:
            T = self.tf.lookup_transform(self.base, msg.header.frame_id, Time.from_msg(msg.header.stamp), Duration(seconds=0.3)).transform
        except Exception:                                                # TF a little late (loaded machine): use latest instead of dropping the scan
            T = self.tf.lookup_transform(self.base, msg.header.frame_id, Time()).transform
        R = Rotation.from_quat([T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]).as_matrix()
        return a @ R.T + np.array([T.translation.x, T.translation.y, T.translation.z])

    def preprocess(self, p):
        r = np.hypot(p[:, 0], p[:, 1])
        p = p[(p[:, 2] > self.min_h) & (p[:, 2] < self.max_h) & (r > self.min_r) & (r < self.max_r)][:, :2]   # drop ground + own body
        _, i = np.unique(np.floor(p / self.voxel).astype(np.int64), axis=0, return_index=True)
        return p[i]

    # ---- map ----------------------------------------------------------------------------------------------------
    def rebuild_map(self):
        self.tree = cKDTree(self.map)
        k = min(8, len(self.map))
        d, idx = self.tree.query(self.map, k=k)
        nb = self.map[idx] - self.map[idx].mean(1, keepdims=True)          # (M,k,2)
        c = np.einsum('mki,mkj->mij', nb, nb) / k
        ang = 0.5 * np.arctan2(2 * c[:, 0, 1], c[:, 0, 0] - c[:, 1, 1])    # principal direction
        self.nrm = np.stack([-np.sin(ang), np.cos(ang)], 1)
        tr = c[:, 0, 0] + c[:, 1, 1]; det = c[:, 0, 0] * c[:, 1, 1] - c[:, 0, 1] ** 2
        lam_min = 0.5 * (tr - np.sqrt(np.maximum(tr * tr - 4 * det, 0)))
        self.wgt = np.clip(1.0 - lam_min / (0.25 * tr + 1e-9), 0.15, 1.0)  # line-like -> 1, blob -> small

    def add_keyframe(self, pts_odom):
        m = np.vstack([self.map, pts_odom]) if len(self.map) else pts_odom
        m = m[np.hypot(*(m - self.pose[:2]).T) < self.map_radius]
        _, i = np.unique(np.floor(m / self.voxel).astype(np.int64), axis=0, return_index=True)
        self.map = m[i]; self.rebuild_map(); self.key_pose = self.pose.copy()

    # ---- registration -------------------------------------------------------------------------------------------
    def icp(self, src, guess):
        x = guess.copy(); matches = 0; rms = float('nan')
        for it in range(12):
            R = rot(x[2]); q = src @ R.T + x[:2]
            tau = max(0.25, 0.8 - 0.1 * it)
            d, j = self.tree.query(q, distance_upper_bound=tau)
            ok = np.isfinite(d)
            if ok.sum() < 25:
                return x, int(ok.sum()), float('nan'), False
            n = self.nrm[j[ok]]; res = np.einsum('ij,ij->i', n, q[ok] - self.map[j[ok]])
            w = self.wgt[j[ok]] * (self.sigma ** 2 / (self.sigma ** 2 + res ** 2)) ** 2
            perp = np.stack([-(q[ok][:, 1] - x[1]), q[ok][:, 0] - x[0]], 1)
            J = np.column_stack([n[:, 0], n[:, 1], np.einsum('ij,ij->i', n, perp)])
            H = J.T @ (J * w[:, None]) + 1e-6 * np.eye(3); g = J.T @ (w * res)
            H[0, 0] += self.trans_prior; H[1, 1] += self.trans_prior                                # Tikhonov toward the prediction
            g[0] += self.trans_prior * (x[0] - guess[0]); g[1] += self.trans_prior * (x[1] - guess[1])
            if self.fix_yaw:                                                    # solve [dx, dy] only; yaw stays at the gyro prediction
                dx = np.zeros(3); dx[:2] = -np.linalg.solve(H[:2, :2], g[:2])
            else:
                dx = -np.linalg.solve(H, g)
            x += dx; x[2] = wrap(x[2]); matches = int(ok.sum()); rms = float(np.sqrt(np.mean(res ** 2)))
            if np.linalg.norm(dx[:2]) < 1e-4 and abs(dx[2]) < 1e-4:
                break
        return x, matches, rms, True

    # ---- main loop ----------------------------------------------------------------------------------------------
    def on_cloud(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            src = self.preprocess(self.to_base(msg))
        except Exception as e:                                           # TF not ready yet
            self.get_logger().warn(f'skip scan: {e}', throttle_duration_sec=5.0); return
        if len(src) < 30:
            return
        if self.tree is None:
            self.add_keyframe(src @ rot(self.pose[2]).T + self.pose[:2]); self.t_last = t; return
        dt = max(t - self.t_last, 1e-3)
        dyaw = self.gyro_dyaw(self.t_last, t)
        if dyaw is None:
            dyaw = self.vel[2] * dt
        guess = self.pose.copy()
        guess[:2] += rot(self.pose[2]) @ (self.vel[:2] * dt)             # constant-velocity translation
        guess[2] = wrap(self.pose[2] + dyaw)
        x, matches, rms, ok = self.icp(src, guess)
        step = np.hypot(*(x[:2] - self.pose[:2])); turn = abs(wrap(x[2] - self.pose[2]))
        if ok and (step > 0.6 * dt + 0.05 or turn > 0.8 * dt + 0.05):      # implausible jump: keep the prediction
            ok = False
        self.rej = 0 if ok else self.rej + 1
        if self.rej >= self.reseed_after:                                    # lost: restart the local map at the predicted pose
            self.get_logger().warn('registration lost, re-seeding the local map')
            self.map = np.empty((0, 2)); self.tree = None; self.rej = 0; self.pose = guess; self.t_last = t; return
        if not ok:
            self.fails += 1; x = guess; self.vel *= 0.5
            self.get_logger().warn(f'registration rejected ({matches} matches), using prediction', throttle_duration_sec=2.0)
        d = x - self.pose; d[2] = wrap(d[2])
        self.vel = np.array([*(rot(-self.pose[2]) @ d[:2] / dt), d[2] / dt])
        self.pose = x; self.t_last = t; self.n += 1
        if (np.hypot(*(self.pose[:2] - self.key_pose[:2])) > self.key_t) or abs(wrap(self.pose[2] - self.key_pose[2])) > self.key_r:
            self.add_keyframe(src @ rot(self.pose[2]).T + self.pose[:2])
        self.publish(msg.header.stamp, ok, matches, rms)

    def publish(self, stamp, ok, matches, rms):
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = 'odom'; o.child_frame_id = self.base
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.pose[0]), float(self.pose[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = (float(v) for v in self.vel)
        s = 1.0 if ok else 100.0                                          # inflate covariance when registration failed
        pc = np.zeros(36); pc[0] = pc[7] = 0.004 * s; pc[14] = pc[21] = pc[28] = 1e3; pc[35] = 0.002 * s
        tc = np.zeros(36); tc[0] = tc[7] = 0.02 * s; tc[35] = 0.01 * s
        o.pose.covariance = pc.tolist(); o.twist.covariance = tc.tolist()
        self.pub.publish(o)
        if self.n % 50 == 0:
            self.get_logger().info(f'#{self.n} pose=({self.pose[0]:.2f},{self.pose[1]:.2f},{math.degrees(self.pose[2]):.0f}deg) matches={matches} rms={rms:.3f} map={len(self.map)} fails={self.fails}')


def main():
    rclpy.init(); node = LidarOdom()
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(node); ex.spin()


if __name__ == '__main__':
    main()
