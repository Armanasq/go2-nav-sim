#!/usr/bin/env python3
"""Safety layer between a teleop and the robot:  teleop/cmd_vel -> [limits + dead-man] -> cmd_vel
usage: python3 projects/p02_teleop-and-safety/safety_node.py [--ros-args -p timeout:=0.6 -p rate:=20.0 ...]
Parameters: v_max [vx, vy, wz], a_max [ax, ay, alpha], timeout (s), rate (Hz). Runs on simulated time."""
import os, signal, sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from limiter import Limiter


class Safety(Node):
    def __init__(self):
        super().__init__('cmd_safety', parameter_overrides=[Parameter('use_sim_time', value=True)])
        v_max = self.declare_parameter('v_max', [0.3, 0.25, 0.5]).value
        a_max = self.declare_parameter('a_max', [0.5, 0.5, 1.0]).value
        timeout = self.declare_parameter('timeout', 0.6).value
        rate = self.declare_parameter('rate', 20.0).value
        assert v_max is not None and a_max is not None and timeout is not None and rate is not None
        self.lim = Limiter(tuple(v_max), tuple(a_max), timeout)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'teleop/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.was_stale = True
        self.get_logger().info(f'teleop/cmd_vel -> cmd_vel  v_max {list(v_max)}  a_max {list(a_max)}  timeout {timeout} s  {rate} Hz')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.lim.command(self.now(), (m.linear.x, m.linear.y, m.angular.z))

    def on_timer(self):
        t = self.now()
        if self.lim.t_out is not None and t < self.lim.t_out:
            self.get_logger().warn(f'simulated time went back {self.lim.t_out - t:.1f} s (simulator restarted): '
                                   'last command dropped')
        stale = self.lim.stale(t)
        if stale and not self.was_stale:
            self.get_logger().warn(f'no command for {self.lim.timeout} s: stopping')
        self.was_stale = stale
        out = self.lim.step(t)
        if out is not None:
            self.send(*out)

    def send(self, vx, vy, wz):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
        self.pub.publish(m)


def main():
    # own the signals, so the context is still valid when we publish the final zero. SIGINT is set explicitly because a
    # process started in the background inherits it as ignored.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, signal.default_int_handler)
    node = Safety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.send(0.0, 0.0, 0.0)            # leave the robot standing, not walking on our last output
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
