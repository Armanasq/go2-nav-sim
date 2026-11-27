#!/usr/bin/env python3
"""projects/common/record_cam.py with a reliable subscription instead of best effort.
usage: same as record_cam.py, e.g.  python3 projects/p03_move-in-shapes/record.py OUT.mp4 --topic /spectator/cam/image_raw
A 960x540 image is 1.5 MB, which DDS splits into many UDP fragments. On a loaded machine the best-effort recorder lost almost
every image (2 frames in 60 s of recording); with a reliable reader lost fragments are resent (222 frames in 25 s)."""
import pathlib, sys
sys.dont_write_bytecode = True                              # keep projects/common free of __pycache__
from rclpy.qos import QoSProfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'common'))
import record_cam  # noqa: E402

record_cam.qos_profile_sensor_data = QoSProfile(depth=5)    # ponytail: patch the one name record_cam reads, no copy of it
record_cam.main()
