#!/usr/bin/env python3
"""Find the floor in base_footprint from the lowest lidar ring (chapter 5, task 1.5).
Takes one /lidar/points message, keeps the points of the lowest elevation angle, moves them to base_footprint with TF,
and prints their horizontal distance and height next to the true trunk height from /odom_gt.
Run with the stack up and the robot standing:  python3 docs/exercises/floor_ring.py --ros-args -p use_sim_time:=true"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('floor_ring')
    buf = Buffer()
    listener = TransformListener(buf, node)
    got = {}
    node.create_subscription(PointCloud2, '/lidar/points', lambda m: got.setdefault('cloud', m), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom_gt', lambda m: got.setdefault('gt', m), 10)
    while len(got) < 2 or not buf.can_transform('base_footprint', 'lidar_link', Time()):
        rclpy.spin_once(node, timeout_sec=1.0)
    p = point_cloud2.read_points_numpy(got['cloud'], field_names=('x', 'y', 'z'), skip_nans=True)
    horiz = np.hypot(p[:, 0], p[:, 1])
    elev = np.round(np.degrees(np.arctan2(p[:, 2], horiz)))            # one value per ring, whole degrees
    ring = elev == elev.min()
    dz = buf.lookup_transform('base_footprint', 'lidar_link', Time()).transform.translation.z
    z = p[ring, 2] + dz                                                # no rotation on this chain (task 1.5)
    print(f'lowest ring ({elev.min():.0f} deg): {ring.sum()} pts  horizontal distance median {np.median(horiz[ring]):.3f} m  '
          f'z in base_footprint median {np.median(z):.3f}  mean {z.mean():.3f}  (lidar_link z in base_footprint {dz:.3f})')
    print(f'gt z {got["gt"].pose.pose.position.z:.4f}')
    del listener
    rclpy.shutdown()


if __name__ == '__main__':
    main()
