#!/usr/bin/env python3
"""Publish what is in front of the robot.
  /scan                  -> /front_range   (sensor_msgs/Range, base_footprint): nearest return within +-half_angle of straight ahead
  /camera/depth/image_raw -> /centre_depth (sensor_msgs/Range, camera_optical): median depth of the 5x5 patch at the image centre
usage: python3 projects/p05_sense-and-stop/sense.py [--ros-args -p half_angle:=20.0]"""
import math, os, sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, Range

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obstacle import centre_depth, front_min


class Sense(Node):
    def __init__(self):
        super().__init__('sense', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.half = math.radians(self.declare_parameter('half_angle', 20.0).value)   # degrees on the command line
        self.pub_scan = self.create_publisher(Range, 'front_range', 10)
        self.pub_depth = self.create_publisher(Range, 'centre_depth', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # A 640x360 float image is 0.9 MB. Subscribe reliable (the camera publishes reliable): best effort lost most of them
        # on a loaded machine (project 2, record_reliable.py).
        self.create_subscription(Image, 'camera/depth/image_raw', self.on_depth,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        self.last = {'scan': (math.nan, math.nan), 'depth': math.nan}
        self.create_timer(1.0, self.report)
        self.get_logger().info(f'/scan -> /front_range (+-{math.degrees(self.half):.0f} deg), '
                               '/camera/depth/image_raw -> /centre_depth')

    def on_scan(self, m):
        r, a = front_min(m.ranges, m.angle_min, m.angle_increment, self.half, m.range_min, m.range_max)
        self.pub_scan.publish(Range(header=m.header, radiation_type=Range.INFRARED, field_of_view=2 * self.half,
                                    min_range=m.range_min, max_range=m.range_max, range=r))
        self.last['scan'] = (r, a)

    def on_depth(self, m):
        if m.encoding != '32FC1':
            self.get_logger().error(f'expected 32FC1 depth, got {m.encoding}', once=True)
            return
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        r = centre_depth(d)
        self.pub_depth.publish(Range(header=m.header, radiation_type=Range.INFRARED, field_of_view=0.01,
                                     min_range=0.3, max_range=8.0, range=r))   # 0.3-8 m: the camera's clip in sensors.xacro
        self.last['depth'] = r

    def report(self):
        (r, a), d = self.last['scan'], self.last['depth']
        self.get_logger().info(f'front {r:.2f} m at {math.degrees(a):+.1f} deg   centre depth {d:.2f} m')


def main():
    rclpy.init()
    try:
        rclpy.spin(Sense())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
