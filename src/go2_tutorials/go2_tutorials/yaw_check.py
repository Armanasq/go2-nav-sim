"""Compare /imu_yaw with the ground-truth yaw on /odom_gt and print the difference every 10 s of sim time (chapter 4).
usage: ros2 run go2_tutorials yaw_check --ros-args -p use_sim_time:=true -p duration:=60.0"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class YawCheck(Node):
    def __init__(self):
        super().__init__('yaw_check')
        self.duration = self.declare_parameter('duration', 60.0).value
        self.imu = None                      # latest /imu_yaw
        self.gt = self.gt_raw = None         # unwrapped ground-truth yaw, last raw value
        self.t0 = self.imu0 = self.gt0 = None
        self.next_print = 0.0
        self.create_subscription(Float64, 'imu_yaw', lambda m: setattr(self, 'imu', m.data), 10)  # type: ignore[reportAttributeAccessIssue]  # rclpy types the callback message as an unbound TypeVar
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, 10)

    def on_gt(self, m):
        raw = yaw_of(m.pose.pose.orientation)
        if self.gt is None:
            self.gt = raw
        else:
            assert self.gt_raw is not None   # set together with self.gt
            self.gt = self.gt + wrap(raw - self.gt_raw)
        self.gt_raw = raw
        if self.imu is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0, self.imu0, self.gt0 = t, self.imu, self.gt
        el = t - self.t0
        if el >= self.next_print:
            assert self.gt0 is not None      # set with self.t0
            d_gt, d_imu = self.gt - self.gt0, self.imu - self.imu0
            print(f't {el:5.1f} s  gt turned {math.degrees(d_gt):8.2f} deg  imu_yaw turned {math.degrees(d_imu):8.2f} deg  '
                  f'error {math.degrees(d_imu - d_gt):6.2f} deg', flush=True)
            self.next_print += 10.0
            if el >= self.duration:
                raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(YawCheck())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
