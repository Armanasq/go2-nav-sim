#!/usr/bin/env python3
"""Walk forward, stop sending commands, and measure how long the robot takes to stand still (from /odom_gt).
usage: python3 projects/p02_teleop-and-safety/measure_stop.py --topic /cmd_vel            # straight to CHAMP
       python3 projects/p02_teleop-and-safety/measure_stop.py --topic /teleop/cmd_vel     # through safety_node.py
       add --zero to end with one zero twist (the 'k' key) instead of silence
Times are simulated seconds. The script always sends a zero on /cmd_vel at the end, so the robot is left standing."""
import argparse, csv, math, pathlib
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

STILL = 0.02     # m/s: below this, averaged over WIN, the robot counts as standing
WIN = 0.5        # s


def stop_time(rows, t_last):
    """First time after t_last from which the windowed speed stays below STILL; None if it never does.
    rows: (t, x, y) samples of the true position. Returns (t_stop, [(t, speed)]), each speed stamped at its window's centre."""
    speeds = []
    j = 0
    for t, x, y in rows:
        while rows[j][0] < t - WIN:
            j += 1
        if t - rows[j][0] >= 0.8 * WIN:
            speeds.append(((t + rows[j][0]) / 2, math.hypot(x - rows[j][1], y - rows[j][2]) / (t - rows[j][0])))
    t_stop = None
    for t, v in speeds:
        if t < t_last:
            continue
        if v >= STILL:
            t_stop = None
        elif t_stop is None:
            t_stop = t
    return t_stop, speeds


class MeasureStop(Node):
    def __init__(self, a):
        super().__init__('measure_stop', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.a = a
        self.cmd_pub = self.create_publisher(Twist, a.topic, 10)
        self.robot_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gt, self.sent, self.robot = [], [], []
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 50)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.robot.append((self.now(), m.linear.x)), 50)  # type: ignore[reportAttributeAccessIssue]  # rclpy: create_subscription leaves msg_type untyped, so the callback argument is typed object
        self.t0 = self.t_last = None
        self.create_timer(0.1, self.on_timer)        # 10 Hz, the rate of a held key in teleop_twist_keyboard is ~30 Hz
        self.done = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p = m.pose.pose.position
        self.gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y))

    def on_timer(self):
        t = self.now()
        if t == 0.0 or not self.gt:
            return                                   # no /clock or no ground truth yet
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 < self.a.hold:
            m = Twist(); m.linear.x = self.a.vx
            self.cmd_pub.publish(m); self.sent.append((t, self.a.vx)); self.t_last = t
        elif self.a.zero and self.sent[-1][1] != 0.0:
            self.cmd_pub.publish(Twist()); self.sent.append((t, 0.0)); self.t_last = t
        elif t - self.t_last > self.a.watch:
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/cmd_vel'); ap.add_argument('--vx', type=float, default=0.3)
    ap.add_argument('--hold', type=float, default=4.0, help='seconds of commands')
    ap.add_argument('--watch', type=float, default=6.0, help='seconds to watch after the last command')
    ap.add_argument('--zero', action='store_true', help='end with one zero twist instead of silence')
    ap.add_argument('--csv', help='write t, x, y of /odom_gt and the commands to this file')
    a = ap.parse_args()
    rclpy.init(); n = MeasureStop(a)
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    for _ in range(3):
        n.robot_pub.publish(Twist())                 # leave it standing
        rclpy.spin_once(n, timeout_sec=0.1)
    t_stop, speeds = stop_time(n.gt, n.t_last)
    i_last = min(range(len(n.gt)), key=lambda i: abs(n.gt[i][0] - n.t_last))
    x0, y0 = n.gt[i_last][1:]
    if t_stop is None:
        drift = math.hypot(n.gt[-1][1] - x0, n.gt[-1][2] - y0)
        v_end = speeds[-1][1]
        print(f'{a.topic} zero={a.zero}: NOT stopped {a.watch:.1f} s after the last command; '
              f'walked {drift:.2f} m since, speed at end {v_end:.3f} m/s')
    else:
        k = min(range(len(n.gt)), key=lambda i: abs(n.gt[i][0] - t_stop))
        drift = math.hypot(n.gt[k][1] - x0, n.gt[k][2] - y0)
        print(f'{a.topic} zero={a.zero}: standstill {t_stop - n.t_last:.2f} s after the last command, '
              f'{drift:.2f} m walked in that time')
    if a.csv:
        pathlib.Path(a.csv).parent.mkdir(parents=True, exist_ok=True)
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['kind', 't', 'a', 'b'])
            w.writerow(['t_last', n.t_last, '', '']); w.writerow(['t_stop', t_stop if t_stop else '', '', ''])
            w.writerows(['gt', *r] for r in n.gt)
            w.writerows(['sent', t, v, ''] for t, v in n.sent)
            w.writerows(['robot', t, v, ''] for t, v in n.robot)
    n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
