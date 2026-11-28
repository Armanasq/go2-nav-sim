#!/usr/bin/env python3
"""Print map -> odom and odom -> base_footprint as x, y, yaw, and the two composed (chapter 5, task 1.3).
Run with the stack up:  python3 docs/exercises/map_odom.py --ros-args -p use_sim_time:=true"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def xyyaw(buf, parent, child):
    t = buf.lookup_transform(parent, child, Time()).transform      # the latest of each edge
    q = t.rotation
    return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    rclpy.init()
    node = Node('map_odom')
    buf = Buffer()
    listener = TransformListener(buf, node, spin_thread=True)
    time.sleep(3.0)                                                  # wall seconds; let the buffer fill
    a, b = xyyaw(buf, 'map', 'odom'), xyyaw(buf, 'odom', 'base_footprint')
    x = a[0] + math.cos(a[2]) * b[0] - math.sin(a[2]) * b[1]         # map <- odom <- base_footprint, in 2-D
    y = a[1] + math.sin(a[2]) * b[0] + math.cos(a[2]) * b[1]
    print('map->odom (%.3f, %.3f, yaw %.3f rad)  odom->base_footprint (%.3f, %.3f, yaw %.3f rad)  robot in map (%.3f, %.3f)'
          % (*a, *b, x, y))
    del listener
    rclpy.shutdown()


if __name__ == '__main__':
    main()
