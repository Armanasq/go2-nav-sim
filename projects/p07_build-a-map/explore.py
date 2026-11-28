#!/usr/bin/env python3
"""Walk the Go2 along the mapping route in route.py with project 4's polar controller, steering on /odom, and log the run.
usage: python3 projects/p07_build-a-map/explore.py [--name NAME]
The waypoints are in the world frame. /odom_gt is read once at the start, to place the odom frame in the world (the same
shortcut as project 6); after that the robot steers on /odom only, and /odom_gt is only logged.
Each 20 Hz tick is one row of projects/p07_build-a-map/results/NAME.csv (default my_map): the command, /odom, the true pose
(world frame) and slam_toolbox's estimate (TF map -> base_footprint, nan until slam_toolbox publishes it).
A leg that takes longer than 90 s of simulated time is given up and the next one starts.
Ctrl-C or kill -TERM stops the robot: the node sends zero velocity before it exits."""
import argparse, csv, math, pathlib, signal, sys
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / 'p04_go-to-pose'), str(HERE.parent / 'p06_avoid-obstacles')]
import polar  # noqa: E402   project 4: GoToPose
import reactive  # noqa: E402   project 6: compose, inverse (SE(2))
import route  # noqa: E402

TIMEOUT = 90.0                       # s of simulated time per leg
NAN3 = (math.nan,) * 3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_of(m):
    p = m.pose.pose.position
    return p.x, p.y, yaw_of(m.pose.pose.orientation)


class Explorer(Node):
    def __init__(self, name):
        super().__init__('explore', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.odom = self.gt = self.anchor = None
        self.goals, self.ctl, self.leg, self.done = [], None, 0, False
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', pose_of(m)), 10)
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, 10)
        self.tf = Buffer(); TransformListener(self.tf, self)
        (HERE / 'results').mkdir(exist_ok=True)
        self.f = open(HERE / 'results' / f'{name}.csv', 'w', newline='')
        self.log = csv.writer(self.f)
        self.log.writerow(['t', 'leg', 'phase', 'v_cmd', 'w_cmd', 'x', 'y', 'yaw', 'gt_x', 'gt_y', 'gt_yaw',
                           'slam_x', 'slam_y', 'slam_yaw'])
        self.create_timer(0.05, self.tick)

    def on_gt(self, m):
        x, y, yaw = pose_of(m)
        self.gt = (x + route.A[0], y + route.A[1], yaw)          # /odom_gt is relative to A; back to the world frame

    def slam(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time()).transform
        except Exception:                                          # slam_toolbox not up yet, or no scan matched yet
            return NAN3
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def start_leg(self, t):
        world, tol = self.goals[self.leg]
        self.ctl = polar.GoToPose(reactive.compose(reactive.inverse(self.anchor), world), pos_tol=tol)
        self.t_leg = t

    def tick(self):
        if self.done or self.odom is None or self.gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.anchor is None:                                    # once: where the odom frame sits in the world
            self.anchor, self.t0 = reactive.compose(self.gt, reactive.inverse(self.odom)), t
            self.goals = route.legs(self.gt[:2])
            self.get_logger().info(f'start at world ({self.gt[0]:.2f}, {self.gt[1]:.2f}); {len(route.WAYPOINTS)} waypoints, '
                                   f'{route.length(self.gt[:2]):.1f} m')
            self.start_leg(t)
        v, w, phase = self.ctl.step(*self.odom)
        if phase != 'done' and t - self.t_leg > TIMEOUT:
            v, w, phase = 0.0, 0.0, 'timeout'
        tw = Twist(); tw.linear.x, tw.angular.z = float(v), float(w)
        self.pub.publish(tw)
        self.log.writerow([f'{t - self.t0:.3f}', self.leg, phase, f'{v:.3f}', f'{w:.3f}',
                           *(f'{c:.4f}' for c in self.odom + self.gt + self.slam())])
        if phase in ('done', 'timeout'):
            if self.leg % 2:                                       # odd legs are the walks; even legs the turns
                self.get_logger().info(f'waypoint {self.leg // 2 + 1}/{len(route.WAYPOINTS)} {phase} at {t - self.t0:.0f} s, '
                                       f'true pose ({self.gt[0]:.2f}, {self.gt[1]:.2f})')
            self.leg += 1
            if self.leg == len(self.goals):
                self.done = True
            else:
                self.start_leg(t)

    def stop(self):
        for _ in range(5):                  # CHAMP keeps walking on the last command, so always end on zero
            self.pub.publish(Twist())
        self.f.close()


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('--name', default='my_map')
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # project 4 explains why: zero twist on Ctrl-C
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = Explorer(args.name)
    try:
        while not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted: sending zero velocity')
    finally:
        node.stop()
    print(f'log: {node.f.name}')


if __name__ == '__main__':
    main()
