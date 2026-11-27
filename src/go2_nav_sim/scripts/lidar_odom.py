#!/usr/bin/env python3
"""Planar LiDAR-inertial odometry: scan-to-local-map point-to-line ICP in SE(2) (registration.py).

  /lidar/points --TF--> base_footprint -> height band -> voxel downsample
  prediction : constant velocity for translation; yaw increment from the gyro, tilt-compensated with the IMU roll and pitch
  registration: translation-only Gauss-Newton on point-to-line residuals against a keyframed local map, Geman-McClure weights,
                a prior toward the prediction only along directions whose information per point is below a threshold (degeneracy)
  output     : /lidar_odom with covariance sigma^2 H^-1 from the registration (no TF; the EKF owns odom -> base_footprint)
Known limits: in this setup TF base_footprint -> base_link is a constant (0, 0, 0.426) with no tilt, so the height band is not
gravity-aligned (chapter 7); no loop closure; slam_toolbox runs passively and does not correct odom."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from registration import GyroYaw, LineMap, register, rot, voxel_downsample, wrap


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
        self.gyro = GyroYaw()                                 # integrated gyro z, looked up by time (callback order does not matter)
        self.map = np.empty((0, 2)); self.lmap = None
        self.key_pose = None; self.n = 0; self.fails = 0; self.rej = 0

    # ---- inputs -------------------------------------------------------------------------------------------------
    def on_imu(self, m):
        q = m.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)); pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.gyro.add(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z, m.angular_velocity.y, roll, pitch)

    def to_base(self, msg):
        """PointCloud2 -> Nx3 in the base frame given by TF."""
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
        return voxel_downsample(p, self.voxel)

    # ---- map ----------------------------------------------------------------------------------------------------
    def add_keyframe(self, pts_odom):
        m = np.vstack([self.map, pts_odom]) if len(self.map) else pts_odom
        m = m[np.hypot(*(m - self.pose[:2]).T) < self.map_radius]
        self.map = voxel_downsample(m, self.voxel); self.lmap = LineMap(self.map); self.key_pose = self.pose.copy()

    # ---- main loop ----------------------------------------------------------------------------------------------
    def on_cloud(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            src = self.preprocess(self.to_base(msg))
        except Exception as e:                                           # TF not ready yet
            self.get_logger().warn(f'skip scan: {e}', throttle_duration_sec=5.0); return
        if len(src) < 30:
            return
        if self.lmap is None:
            self.add_keyframe(src @ rot(self.pose[2]).T + self.pose[:2]); self.t_last = t; return
        dt = max(t - self.t_last, 1e-3)
        dyaw = self.gyro.dyaw(self.t_last, t)
        if dyaw is None:
            dyaw = self.vel[2] * dt
        guess = self.pose.copy()
        guess[:2] += rot(self.pose[2]) @ (self.vel[:2] * dt)             # constant-velocity translation
        guess[2] = wrap(self.pose[2] + dyaw)
        x, matches, rms, ok, cov = register(src, self.lmap, guess, self.sigma, self.trans_prior, self.fix_yaw)
        step = np.hypot(*(x[:2] - self.pose[:2])); turn = abs(wrap(x[2] - self.pose[2]))
        if ok and (step > 0.6 * dt + 0.05 or turn > 0.8 * dt + 0.05):      # implausible jump: keep the prediction
            ok = False
        self.rej = 0 if ok else self.rej + 1
        if self.rej >= self.reseed_after:                                    # lost: restart the local map at the predicted pose
            self.get_logger().warn('registration lost, re-seeding the local map')
            self.map = np.empty((0, 2)); self.lmap = None; self.rej = 0; self.pose = guess; self.t_last = t; return
        if not ok:
            self.fails += 1; x = guess; self.vel *= 0.5
            self.get_logger().warn(f'registration rejected ({matches} matches), using prediction', throttle_duration_sec=2.0)
        d = x - self.pose; d[2] = wrap(d[2])
        self.vel = np.array([*(rot(-self.pose[2]) @ d[:2] / dt), d[2] / dt])
        self.pose = x; self.t_last = t; self.n += 1
        if (np.hypot(*(self.pose[:2] - self.key_pose[:2])) > self.key_t) or abs(wrap(self.pose[2] - self.key_pose[2])) > self.key_r:
            self.add_keyframe(src @ rot(self.pose[2]).T + self.pose[:2])
        self.publish(msg.header.stamp, ok, matches, rms, cov)

    def publish(self, stamp, ok, matches, rms, cov=None):
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = 'odom'; o.child_frame_id = self.base
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.pose[0]), float(self.pose[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = (float(v) for v in self.vel)
        pc = np.zeros(36); pc[14] = pc[21] = pc[28] = 1e3; pc[35] = 1e3       # z, roll, pitch unused; yaw is the gyro's, fused by the EKF from /imu
        if ok and cov is not None:
            c = np.maximum(cov, 0) + np.diag([1e-4, 1e-4])                       # sigma^2 H^-1 from the registration, floored at 1 cm
            pc[0], pc[1], pc[6], pc[7] = c[0, 0], c[0, 1], c[1, 0], c[1, 1]
        else:
            pc[0] = pc[7] = 1.0                                                  # failed registration: prediction only, 1 m std
        s = 1.0 if ok else 100.0                                                 # twist is not fused by the EKF; kept for other consumers
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
