#!/usr/bin/env python3
"""Record a camera topic to MP4 and stop cleanly on Ctrl-C (or SIGTERM).
usage: python3 projects/p05_sense-and-stop/record.py OUT.mp4 [--topic /spectator/cam/image_raw] [--fps 10] [--width 960]
The recorder of projects/common/record_cam.py, with a reliable subscription (as project 2's record_reliable.py) and its own
signal handling: Ctrl-C only sets a flag, the loop returns, and the file is closed and 'OUT.mp4: N frames' printed. With
rclpy's handler, Ctrl-C could end in an ExternalShutdownException or RuntimeError traceback before that line."""
import argparse, os, signal, sys

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
import record_cam

record_cam.qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)   # read by Rec at subscribe time
ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('--topic', default='/spectator/cam/image_raw')
ap.add_argument('--fps', type=int, default=10); ap.add_argument('--width', type=int, default=960)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node, stop = record_cam.Rec(ap.parse_args()), []
for s in (signal.SIGINT, signal.SIGTERM):
    signal.signal(s, lambda *_: stop.append(1))
while not stop:
    rclpy.spin_once(node, timeout_sec=0.1)
node.close()
