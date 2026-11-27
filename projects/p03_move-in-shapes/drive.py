#!/usr/bin/env python3
"""Walk a square or a figure-eight, open loop (timed commands) or closed loop (on /odom), and log the run.
usage: python3 projects/p03_move-in-shapes/drive.py {square,figure8} {open,closed} [v_real w_real]
  v_real w_real: open-loop square only, the forward and turn speeds the robot really reaches; side and turn times use them.
  ROS arguments work too, e.g. --ros-args -r odom:=odom_gt to close the loop on ground truth.
The shape starts at the robot's current pose. Every control tick (20 Hz of simulated time) one CSV row is written to
projects/p03_move-in-shapes/results/<shape>_<mode>.csv (plus a suffix for the options above): the command, the estimated pose (/odom) and the true pose (/odom_gt)."""
import csv, math, pathlib, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import shapes

RUNS = pathlib.Path(__file__).resolve().parent / 'results'


def pose_of(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self, shape, mode, real):
        super().__init__('drive_shapes', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.shape, self.mode, self.real = shape, mode, real
        self.odom = self.gt = self.t0 = self.plan = None
        self.done = False
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        sub = self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', pose_of(m)), 10)
        self.create_subscription(Odometry, 'odom_gt', lambda m: setattr(self, 'gt', pose_of(m)), 10)
        name = f'{shape}_{mode}' + ('_calibrated' if real else '')
        if sub.topic_name != '/odom':                       # remapped, e.g. odom:=odom_gt
            name += '_on' + sub.topic_name.replace('/', '_')
        RUNS.mkdir(exist_ok=True)
        self.f = open(RUNS / f'{name}.csv', 'w', newline='')
        self.log = csv.writer(self.f)
        self.log.writerow(['t', 'label', 'v_cmd', 'w_cmd', 'x', 'y', 'yaw', 'gt_x', 'gt_y', 'gt_yaw'])
        self.create_timer(0.05, self.tick)

    def start(self, t):
        """Fix the plan at the current pose: closed-loop waypoints are moved from the start frame into the odom frame."""
        x0, y0, a0 = self.odom
        if self.mode == 'open':
            self.plan = shapes.square_schedule(**self.real) if self.shape == 'square' else shapes.figure8_schedule()
        else:
            pts = shapes.square_corners() if self.shape == 'square' else shapes.figure8_points()
            pts = [(x0 + px * math.cos(a0) - py * math.sin(a0), y0 + px * math.sin(a0) + py * math.cos(a0)) for px, py in pts]
            self.plan = (shapes.Follower(pts, stop_each=True, final_yaw=a0) if self.shape == 'square' else
                         shapes.Follower(pts, stop_each=False, v=0.18, lookahead=0.25, reach_tol=0.05))
        self.t0 = t
        self.get_logger().info(f'{self.shape} {self.mode}: start at odom ({x0:.2f}, {y0:.2f}, {math.degrees(a0):.1f} deg)')

    def tick(self):
        if self.done or self.odom is None or self.gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.start(t)
        cmd = (shapes.command_at(self.plan, t - self.t0) if self.mode == 'open' else self.plan.step(t, *self.odom))
        if cmd is None:
            cmd, self.done = (0.0, 0.0, 'stop'), True
        v, w, label = cmd
        tw = Twist(); tw.linear.x, tw.angular.z = float(v), float(w)
        self.pub.publish(tw)
        self.log.writerow([f'{t - self.t0:.3f}', label, f'{v:.3f}', f'{w:.3f}', *(f'{c:.4f}' for c in self.odom + self.gt)])
        if self.done:
            self.get_logger().info(f'finished after {t - self.t0:.1f} s of simulated time')

    def stop(self):
        for _ in range(5):                  # CHAMP keeps walking on the last command, so always end on zero
            self.pub.publish(Twist())
        self.f.close()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) not in (2, 4) or argv[0] not in ('square', 'figure8') or argv[1] not in ('open', 'closed'):
        sys.exit(__doc__)
    real = {} if len(argv) == 2 else {'v_real': float(argv[2]), 'w_real': float(argv[3])}
    rclpy.init()
    node = Driver(argv[0], argv[1], real)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.stop()
    print(f'log: {node.f.name}')


if __name__ == '__main__':
    main()
