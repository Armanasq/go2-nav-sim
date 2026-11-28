#!/usr/bin/env python3
"""Walk straight ahead and stop when the gap between the robot's front and the nearest obstacle reaches `gap`.
Needs sense.py running (it publishes /front_range). Exits once it has stopped.
usage: python3 projects/p05_sense-and-stop/stop_at_obstacle.py [--ros-args -p speed:=0.3 -p gap:=0.5 -p front:=0.28]
  topic   /front_range (scan) or /centre_depth (depth camera; then use front:=0.0, the camera is the front)
  lead    extra metres added to `gap` at the trigger, to cancel a known overshoot (exercise 1)
  timeout s of simulated time without a reading while walking before it stops anyway (default 0.5; the scan comes at 10 Hz)"""
import os, signal, sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from obstacle import FRONT, should_stop


class StopAtObstacle(Node):
    def __init__(self):
        super().__init__('stop_at_obstacle', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        p = lambda name, default: self.declare_parameter(name, default).value
        self.speed, self.gap, self.front, self.lead = p('speed', 0.3), p('gap', 0.5), p('front', FRONT), p('lead', 0.0)
        self.timeout = p('timeout', 0.5)
        topic = p('topic', '/front_range')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Range, topic, self.on_range, 10)
        self.state, self.t_reading, self.t_stop = 'wait', None, None
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'{topic}: walk at {self.speed} m/s, stop at a {self.gap} m gap '
                               f'(front {self.front} m ahead of the sensor frame, lead {self.lead} m)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, vx):
        t = Twist(); t.linear.x = float(vx); self.pub.publish(t)

    def on_range(self, m):
        self.t_reading = self.now()
        if self.state == 'wait':
            self.state = 'walk'
        if self.state == 'walk' and should_stop(m.range, self.gap + self.lead, self.front):
            self.send(0.0)                                         # at once, not at the next timer tick
            age = self.t_reading - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.state, self.t_stop = 'stopped', self.t_reading
            print(f'STOP t={self.t_reading:.2f} range={m.range:.3f} gap={m.range - self.front:.3f} age={age:.3f}', flush=True)

    def on_timer(self):
        t = self.now()
        if self.state == 'walk':
            if t - self.t_reading > self.timeout:                  # no news is not good news: stop
                print(f'TIMEOUT t={t:.2f}: no reading for {t - self.t_reading:.2f} s, stopping', flush=True)
                self.state, self.t_stop = 'stopped', t
                self.send(0.0)
            else:
                self.send(self.speed)                              # CHAMP holds the last command; resend it anyway
        elif self.state == 'stopped':
            self.send(0.0)                                         # zeros for 1 s, then exit
            if t - self.t_stop > 1.0:
                raise KeyboardInterrupt


def main():
    # Our own SIGINT/SIGTERM handling, so the final zero can still be published (rclpy's handler invalidates the context).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, signal.default_int_handler)
    node = StopAtObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.send(0.0)


if __name__ == '__main__':
    main()
