#!/usr/bin/env python3
"""projects/common/record_cam.py with a RELIABLE subscription. Same arguments.
On a loaded machine a best-effort subscriber to the 1.5 MB spectator images received 1 frame in 25 s and a reliable one 116
(Cyclone DDS on loopback drops fragments of large best-effort samples). The camera publisher is reliable, so this costs nothing."""
import os, sys
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
import record_cam

record_cam.qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
record_cam.main()
