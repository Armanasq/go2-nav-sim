#!/usr/bin/env python3
"""Record a camera topic to an MP4: projects/common/record_cam.py with a reliable subscription instead of best effort.
usage: python3 projects/p01_first-steps/record.py OUT.mp4 [--topic /spectator/cam/image_raw] [--fps 10] [--width 960]
Stop with Ctrl-C; the file is finalised on exit and the frame count printed.
A 960x540 image is 1.5 MB, which DDS splits into many UDP fragments. On a loaded machine a best-effort reader loses the whole
image when one fragment is lost (record_cam.py as shipped wrote 1 frame); a reliable reader has lost fragments resent."""
import pathlib, sys
sys.dont_write_bytecode = True                              # keep projects/common free of __pycache__
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'common'))
import record_cam  # noqa: E402

record_cam.qos_profile_sensor_data = 10                     # ponytail: patch the one name record_cam reads; 10 = reliable, depth 10
record_cam.main()
