#!/usr/bin/env python3
"""Walk the Go2 to five poses in turn with the polar controller in polar.py, steering on /odom, and log the run.
usage: python3 projects/p04_go-to-pose/go_to_pose.py [--name NAME] [--gains K_RHO K_ALPHA K_BETA] [--goals 1 2 ...] [--v-min V]
  --goals picks some of the five goals (numbered from 1); --v-min sets the speed floor (default 0.1 m/s).
  ROS arguments work too, e.g. --ros-args -r odom:=odom_gt.
Goals are relative to the robot's pose when the script starts (x forward, y left, heading in degrees). After each goal the
robot stands for 1.5 s, then the position and heading error are printed, measured on /odom and on /odom_gt.
A goal ends in 'timeout' after 90 s. The run stops with 'abort' if /odom puts the robot 0.5 m further from the goal than
when the goal was set: the controller is walking away from it, which happens when the /odom heading has drifted.
Ctrl-C or kill -TERM stops the robot: the node sends zero velocity before it exits.
Every 20 Hz tick of simulated time is one row of projects/p04_go-to-pose/results/NAME.csv (default NAME my_run)."""
import argparse, csv, math, pathlib, signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import polar

GOALS = [(2.0, 0.0, 0), (4.0, 0.8, 90), (2.5, 0.3, 180), (0.5, 1.0, 90), (0.0, 0.0, 0)]   # x m, y m, heading deg
SETTLE, TIMEOUT = 1.5, 90.0                                                                # s of simulated time
LEASH = 0.5                    # m further from the goal than when it was set, on /odom: give up the run
RUNS = pathlib.Path(__file__).resolve().parent / 'results'


def pose_of(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def place(goal, start):
    """A goal given relative to the start pose -> the same goal in the frame the start pose is measured in."""
    (gx, gy, gdeg), (x0, y0, a0) = goal, start
    return (x0 + gx * math.cos(a0) - gy * math.sin(a0), y0 + gx * math.sin(a0) + gy * math.cos(a0),
            polar.wrap(a0 + math.radians(gdeg)))


def errors(pose, goal):
    return math.hypot(pose[0] - goal[0], pose[1] - goal[1]), math.degrees(polar.wrap(pose[2] - goal[2]))


class Runner(Node):
    def __init__(self, args):
        super().__init__('go_to_pose', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.gains, self.v_min, self.todo = tuple(args.gains), args.v_min, [GOALS[i - 1] for i in args.goals]
        self.ids = list(args.goals)
        self.odom = self.gt = self.start = None
        self.ctl, self.t_goal, self.t_done, self.done = None, 0.0, None, False
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', pose_of(m)), 10)
        self.create_subscription(Odometry, 'odom_gt', lambda m: setattr(self, 'gt', pose_of(m)), 10)
        RUNS.mkdir(exist_ok=True)
        self.f = open(RUNS / f'{args.name}.csv', 'w', newline='')
        self.log = csv.writer(self.f)
        self.log.writerow(['t', 'goal', 'phase', 'v_cmd', 'w_cmd', 'x', 'y', 'yaw', 'gt_x', 'gt_y', 'gt_yaw',
                           'goal_x', 'goal_y', 'goal_yaw', 'gt_goal_x', 'gt_goal_y', 'gt_goal_yaw'])
        self.get_logger().info(f'gains k_rho={self.gains[0]} k_alpha={self.gains[1]} k_beta={self.gains[2]}, '
                               f'strongly stable: {polar.strong_stability(*self.gains)}')
        self.create_timer(0.05, self.tick)

    def next_goal(self, t):
        self.goal = self.todo.pop(0)
        self.target, self.gt_target = place(self.goal, self.start[0]), place(self.goal, self.start[1])
        self.ctl, self.t_goal, self.t_done = polar.GoToPose(self.target, self.gains, v_min=self.v_min), t, None
        self.d0 = errors(self.odom, self.target)[0]

    def tick(self):
        if self.done or self.odom is None or self.gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start, self.t0 = (self.odom, self.gt), t
            self.get_logger().info(f'start at odom ({self.odom[0]:.2f}, {self.odom[1]:.2f}, '
                                   f'{math.degrees(self.odom[2]):.1f} deg)')
            self.next_goal(t)
        v, w, phase = self.ctl.step(*self.odom)
        if phase == 'drive' and errors(self.odom, self.target)[0] > self.d0 + LEASH:
            phase = 'abort'
        elif phase not in ('done', 'abort') and t - self.t_goal > TIMEOUT:
            phase = 'timeout'
        if phase in ('timeout', 'abort'):
            v, w = 0.0, 0.0
            self.ctl.phase = phase
        tw = Twist(); tw.linear.x, tw.angular.z = float(v), float(w)
        self.pub.publish(tw)
        self.log.writerow([f'{t - self.t0:.3f}', self.ids[0], phase, f'{v:.3f}', f'{w:.3f}',
                           *(f'{c:.4f}' for c in self.odom + self.gt + self.target + self.gt_target)])
        if phase in ('done', 'timeout', 'abort'):
            self.t_done = self.t_done or t
            if t - self.t_done >= SETTLE:
                self.report(t, phase)      # after logging, so the last row of a goal is the one its errors come from

    def report(self, t, phase):
        (de, he), (dt_, ht) = errors(self.odom, self.target), errors(self.gt, self.gt_target)
        self.get_logger().info(f'goal {self.ids[0]} {self.goal}: {phase} after {self.t_done - self.t_goal:.1f} s; '
                               f'error on odom {de:.3f} m {he:+.1f} deg, true {dt_:.3f} m {ht:+.1f} deg')
        if phase == 'abort':
            self.get_logger().warn(f'walking away from the goal on /odom ({de:.2f} m, {self.d0:.2f} m when it was set): '
                                   'the /odom heading has probably drifted. Remaining goals skipped; restart the stack')
            self.todo = []
        if self.todo:
            self.ids.pop(0)
            self.next_goal(t)
        else:
            self.done = True

    def stop(self):
        for _ in range(5):                  # CHAMP keeps walking on the last command, so always end on zero
            self.pub.publish(Twist())
        self.f.close()


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument('--name', default='my_run')
    ap.add_argument('--gains', type=float, nargs=3, default=polar.GAINS)
    ap.add_argument('--v-min', type=float, default=0.1)
    ap.add_argument('--goals', type=int, nargs='+', default=[1, 2, 3, 4, 5], choices=range(1, len(GOALS) + 1))
    args = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])
    # rclpy's own Ctrl-C handler shuts the context down before our code runs, and then stop() cannot publish the zero
    # twist. Without it, Ctrl-C is a plain KeyboardInterrupt; kill -TERM is turned into one too.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = Runner(args)
    try:
        while not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted: sending zero velocity')
    finally:
        node.stop()                         # on any exit, also an exception, so the robot never keeps walking
    print(f'log: {node.f.name}')


if __name__ == '__main__':
    main()
