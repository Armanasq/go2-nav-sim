#!/usr/bin/env python3
"""How often /front_range and /centre_depth arrive, and how old they are on arrival, both in simulated time.
usage: python3 projects/p05_sense-and-stop/sensor_age.py [wall seconds, default 40]    (needs sense.py running)"""
import sys, time

import numpy as np
import rclpy
import rclpy.parameter
from sensor_msgs.msg import Range

rclpy.init()
n = rclpy.create_node('sensor_age', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
got = {'front_range': [], 'centre_depth': []}
for k in got:
    n.create_subscription(Range, k, lambda m, k=k: got[k].append(
        (n.get_clock().now().nanoseconds * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), 10)  # type: ignore[reportAttributeAccessIssue]  # rclpy: create_subscription leaves msg_type untyped, so the callback argument is typed object
t0 = time.time()
while time.time() - t0 < float(sys.argv[1] if len(sys.argv) > 1 else 40):
    rclpy.spin_once(n, timeout_sec=0.1)
for k, v in got.items():
    if len(v) < 2:
        print(f'{k}: {len(v)} messages'); continue
    v = np.array(v); gap = np.diff(v[:, 0]); age = v[:, 0] - v[:, 1]       # receive time minus header stamp
    print(f'{k}: {len(v)} messages in {v[-1, 0] - v[0, 0]:.1f} s; interval mean {gap.mean():.3f} max {gap.max():.3f} s, '
          f'{(gap > 0.5).sum()} over 0.5 s; age mean {age.mean():.3f} max {age.max():.3f} s')
