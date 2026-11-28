#!/usr/bin/env python3
"""Ask TF for odom -> base_footprint at four times and print what comes back (chapter 5, task 2).
Run with the stack up:  python3 docs/exercises/tf_lookup.py"""
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

TARGET, SOURCE = 'odom', 'base_footprint'


def ask(buf, label, when, timeout=Duration()):
    try:
        t = buf.lookup_transform(TARGET, SOURCE, when, timeout)
        s = t.header.stamp
        print(f'{label:<22} ok, stamp {s.sec + s.nanosec * 1e-9:.3f}')
    except TransformException as e:
        print(f'{label:<22} {type(e).__name__}: {e}')


def main():
    rclpy.init()
    node = Node('tf_lookup', parameter_overrides=[Parameter('use_sim_time', value=True)])   # stamps are simulated seconds
    buf = Buffer()                                         # keeps 10 s of history by default
    TransformListener(buf, node, spin_thread=True)         # spins the node: /tf, /tf_static and /clock
    ask(buf, 'at once, now', node.get_clock().now())       # the listener has heard nothing yet
    time.sleep(3.0)                                        # wall seconds; let the buffer fill
    now = node.get_clock().now()
    print(f'now = {now.nanoseconds * 1e-9:.3f} s (simulated)')
    ask(buf, 'latest, Time()', Time())
    ask(buf, 'now - 30 s', now - Duration(seconds=30.0))
    ask(buf, 'now + 0.5 s', now + Duration(seconds=0.5))
    ask(buf, 'now + 0.5 s, wait 3 s', now + Duration(seconds=0.5), Duration(seconds=3.0))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
