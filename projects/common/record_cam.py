#!/usr/bin/env python3
"""Record a camera topic to an MP4 (H.264), paced by simulation time.
usage: python3 projects/common/record_cam.py OUT.mp4 [--topic /spectator/cam/image_raw] [--fps 10] [--width 960]
Stop with Ctrl-C (or SIGTERM); the file is finalised on exit. Cameras in this simulation: /spectator/cam/image_raw (fixed view
of the whole arena), /chase/cam/image_raw (behind the robot), /camera/image_raw (onboard RGB)."""
import argparse, signal, subprocess, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Rec(Node):
    def __init__(self, a):
        super().__init__('record_cam', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a, self.ff, self.t0, self.n = a, None, None, 0
        self.create_subscription(Image, a.topic, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        if self.ff is None:
            w = self.a.width - self.a.width % 2; h = int(round(m.height * w / m.width / 2)) * 2
            self.ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{m.width}x{m.height}',
                                        '-r', str(self.a.fps), '-i', '-', '-vf', f'scale={w}:{h}', '-c:v', 'libx264', '-preset', 'veryfast',
                                        '-crf', '26', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', self.a.out], stdin=subprocess.PIPE)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.t0 = t if self.t0 is None else self.t0
        while self.n <= int((t - self.t0) * self.a.fps):             # repeat frames if rendering is slower than fps
            self.ff.stdin.write(img.tobytes()); self.n += 1

    def close(self):
        if self.ff:
            self.ff.stdin.close(); self.ff.wait()
        print(f'{self.a.out}: {self.n} frames', file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('--topic', default='/spectator/cam/image_raw')
    ap.add_argument('--fps', type=int, default=10); ap.add_argument('--width', type=int, default=960); a = ap.parse_args()
    rclpy.init(); node = Rec(a)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()


if __name__ == '__main__':
    main()
