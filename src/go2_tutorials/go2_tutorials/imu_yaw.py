"""Integrate the IMU yaw rate into a heading and publish it on /imu_yaw (chapter 4)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class YawIntegrator:
    """Sums rate * dt between message stamps. Plain Python, so the test needs no running ROS graph."""

    def __init__(self, max_gap=0.5):
        self.yaw, self.t_last, self.max_gap = 0.0, None, max_gap

    def add(self, t, rate):
        if self.t_last is not None:
            dt = t - self.t_last
            if dt <= 0.0:                  # repeated or out-of-order stamp: ignore the sample
                return self.yaw
            if dt < self.max_gap:          # a longer gap (paused sim, lost messages) is skipped, not bridged
                self.yaw += rate * dt
        self.t_last = t
        return self.yaw


class ImuYaw(Node):
    def __init__(self):
        super().__init__('imu_yaw')
        rate = self.declare_parameter('publish_rate', 10.0).value
        self.integ = YawIntegrator()
        self.pub = self.create_publisher(Float64, 'imu_yaw', 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'integrating imu/data, publishing imu_yaw at {rate} Hz')

    def on_imu(self, m):
        self.integ.add(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z)

    def on_timer(self):
        self.pub.publish(Float64(data=self.integ.yaw))
        self.get_logger().debug(f'yaw {self.integ.yaw:.4f} rad', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    try:
        rclpy.spin(ImuYaw())
    except KeyboardInterrupt:              # Ctrl-C: exit without a traceback
        pass


if __name__ == '__main__':
    main()
