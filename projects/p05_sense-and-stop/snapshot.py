#!/usr/bin/env python3
"""Save one /scan, one /camera/depth/image_raw and the true pose to an .npz file, for plot.py.
usage: python3 projects/p05_sense-and-stop/snapshot.py OUT.npz"""
import math, sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

rclpy.init()
n = rclpy.create_node('snapshot')
got = {}
n.create_subscription(LaserScan, 'scan', lambda m: got.setdefault('scan', m), qos_profile_sensor_data)
n.create_subscription(Image, 'camera/depth/image_raw', lambda m: got.setdefault('depth', m),
                      QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(Odometry, 'odom_gt', lambda m: got.setdefault('gt', m), 10)
while len(got) < 3:
    rclpy.spin_once(n, timeout_sec=0.5)
s, d, g = got['scan'], got['depth'], got['gt']
q = g.pose.pose.orientation
np.savez_compressed(sys.argv[1], ranges=np.array(s.ranges, np.float32), angle_min=s.angle_min, angle_inc=s.angle_increment,
                    range_min=s.range_min, range_max=s.range_max,
                    depth=np.frombuffer(d.data, np.float32).reshape(d.height, d.step // 4)[:, :d.width],
                    pose=[g.pose.pose.position.x, g.pose.pose.position.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])
print(f'wrote {sys.argv[1]}: scan {len(s.ranges)} beams, depth {d.width}x{d.height}, /odom_gt x={g.pose.pose.position.x:.3f}')
