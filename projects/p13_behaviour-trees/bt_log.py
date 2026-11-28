#!/usr/bin/env python3
"""Print bt_navigator's node status changes (/behavior_tree_log) as one line each, with simulated time since the first message.
Skipped, to keep the log readable: changes back to IDLE (the tree resetting a node) and GoalUpdated returning FAILURE (the
ReactiveFallback checks it on every tick while a recovery runs, about 100 lines per second).
usage:  python3 bt_log.py [--gt results/patrol_gt.csv]       Ctrl-C to stop.
--gt also logs the true pose (/odom_gt, world frame) at 5 Hz for plotting."""
import argparse, sys
import rclpy
import rclpy.parameter
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Odometry

A = (-4.5, -3.5)                                   # /odom_gt starts at 0 on marker A; world = odom_gt + A (as go_to_goal.py)


class BtLog(Node):
    def __init__(self, gt_path):
        super().__init__('bt_log', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.t0, self.gt, self.last_gt = None, open(gt_path, 'w') if gt_path else None, -1.0
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.on_log, 100)
        if self.gt:
            self.gt.write('t,x,y\n')
            self.create_subscription(Odometry, '/odom_gt', self.on_gt, 10)

    def stamp(self, s):
        t = s.sec + s.nanosec * 1e-9
        self.t0 = t if self.t0 is None else self.t0
        return t - self.t0

    def on_log(self, msg):
        t = self.stamp(msg.timestamp)              # bt_navigator's clock (sim time); event.timestamp is wall-clock time
        for e in msg.event_log:
            if e.current_status != 'IDLE' and not (e.node_name == 'GoalUpdated' and e.current_status == 'FAILURE'):
                print(f'{t:8.2f}  {e.node_name:28s} {e.previous_status:8s} -> {e.current_status}', flush=True)

    def on_gt(self, m):
        if self.t0 is None:
            return                                 # start the path with the first tree event
        t = self.stamp(m.header.stamp)
        if t - self.last_gt >= 0.2:
            self.last_gt = t
            assert self.gt is not None
            self.gt.write(f'{t:.2f},{m.pose.pose.position.x + A[0]:.3f},{m.pose.pose.position.y + A[1]:.3f}\n'); self.gt.flush()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--gt'); a = ap.parse_args()
    rclpy.init(); node = BtLog(a.gt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print('bt_log: stopped', file=sys.stderr)


if __name__ == '__main__':
    main()
