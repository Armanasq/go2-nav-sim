#!/usr/bin/env python3
"""Patrol four waypoints with Nav2's FollowWaypoints, optionally dropping a box on the plan that the map does not know about.
usage: python3 projects/p08_navigate-with-nav2/patrol.py NAME [--box]
Needs scripts/up.sh (Gazebo, estimation, Nav2 + AMCL) and the robot standing on marker A.
Writes results/NAME.json (times, outcome, replanning) and results/NAME.npz (true track, AMCL track, every global plan)."""
import argparse, json, math, pathlib, sys, threading, time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.srv import SpawnEntity
from nav2_simple_commander.robot_navigator import BasicNavigator

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import patrol_logic as pl  # noqa: E402

A = (-4.5, -3.5)                       # /odom_gt starts at zero on marker A; add A to get map coordinates
LATCHED = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class Log(Node):
    """Records the true pose, the AMCL pose and every global plan, stamped in simulated time. Spun in its own thread."""
    def __init__(self):
        super().__init__('p08_log', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt, self.amcl, self.odom, self.plans = [], [], [], []
        self.create_subscription(Odometry, '/odom_gt', lambda m: self.gt.append(
            (self.now(), m.pose.pose.position.x + A[0], m.pose.pose.position.y + A[1],
             2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))), 50)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: self.amcl.append(       # stamped with its scan
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y)), LATCHED)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom.append(                              # EKF, odom frame = A
            (self.now(), m.pose.pose.position.x + A[0], m.pose.pose.position.y + A[1])), 20)
        self.create_subscription(Path, '/plan', lambda m: self.plans.append(
            (self.now(), np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]).reshape(-1, 2))), 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def localise(log):
    """Set AMCL to the true pose (the simulation shortcut used throughout this course) and wait until /amcl_pose agrees.
    A zero stamp makes AMCL use the latest transform; one message can be lost to a TF race, so repeat."""
    for _ in range(40):
        t, x, y, yaw = log.gt[-1]
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05; m.pose.covariance[35] = 0.03
        log.init.publish(m); time.sleep(0.75)
        if log.amcl and log.amcl[-1][0] >= t - 0.5 and math.hypot(log.amcl[-1][1] - x, log.amcl[-1][2] - y) < 0.15:
            return True
    return False


def pose(nav, x, y, yaw):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('name'); ap.add_argument('--box', action='store_true')
    ap.add_argument('--limit', type=float, default=400.0, help='cancel the patrol after this many simulated seconds')
    a = ap.parse_args()
    rclpy.init()
    log = Log(); ex = SingleThreadedExecutor(); ex.add_node(log)
    threading.Thread(target=ex.spin, daemon=True).start()
    nav = BasicNavigator()                             # spins itself inside isTaskComplete(); keep it off the executor
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    nav._waitForNodeToActivate('amcl')
    while not log.gt:
        time.sleep(0.1)
    out = dict(name=a.name, box=a.box, waypoints=pl.WAYPOINTS, localised=localise(log))
    nav._waitForNodeToActivate('bt_navigator')

    t0 = log.now(); arrivals, box, leg_start = [], {}, None
    nav.followWaypoints([pose(nav, *w) for w in pl.WAYPOINTS])
    nav.info(f'patrol started, {len(pl.WAYPOINTS)} waypoints, box={a.box}')
    while not nav.isTaskComplete():
        fb, (t, x, y, _) = nav.getFeedback(), log.gt[-1]
        if fb is not None and fb.current_waypoint > len(arrivals):          # the follower moved on: previous waypoint reached
            arrivals.append(t); nav.info(f'waypoint {len(arrivals) - 1} reached at {t - t0:.1f} s')
        if len(arrivals) == pl.SPAWN_LEG and leg_start is None:
            leg_start = (x, y)
        if a.box and not box and leg_start and math.hypot(x - leg_start[0], y - leg_start[1]) > pl.SPAWN_AFTER and log.plans:
            bx, by = pl.point_ahead(log.plans[-1][1], (x, y), pl.SPAWN_AHEAD)
            req = SpawnEntity.Request(name='surprise_box', xml=pl.box_sdf('surprise_box'))
            req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = bx, by, pl.BOX / 2
            fut = log.spawn.call_async(req)                                 # the executor thread completes it
            while not fut.done():
                time.sleep(0.02)
            box = dict(x=round(bx, 2), y=round(by, 2), t=log.now(), ok=fut.result().success, robot=[round(x, 2), round(y, 2)])
            nav.info(f'box spawned at ({bx:.2f}, {by:.2f}): {fut.result().status_message}')
        if log.now() - t0 > a.limit:
            nav.cancelTask(); nav.warn('time limit, patrol cancelled')
    t_end = log.now()
    for _ in range(5):                                                      # CHAMP keeps walking on the last command
        log.cmd.publish(Twist()); time.sleep(0.1)
    res = nav.result_future.result().result if nav.result_future else None

    gt, amcl, odom = np.array(log.gt), np.array(log.amcl), np.array(log.odom)
    walk = gt[(gt[:, 0] >= t0) & (gt[:, 0] <= t_end)]
    i = np.searchsorted(gt[:, 0], amcl[:, 0]).clip(0, len(gt) - 1)          # nearest later true sample to each AMCL pose
    amcl_err = np.hypot(*(amcl[:, 1:3] - gt[i, 1:3]).T)[amcl[:, 0] >= t0]
    out.update(result=nav.getResult().name, missed_waypoints=list(res.missed_waypoints) if res else None,
               time_s=round(t_end - t0, 1), leg_times_s=pl.leg_times(t0, arrivals, t_end), path_m=round(pl.path_length(walk[:, 1:3]), 2),
               final_error_m=round(math.hypot(walk[-1, 1] - pl.WAYPOINTS[-1][0], walk[-1, 2] - pl.WAYPOINTS[-1][1]), 2),
               amcl_rmse_m=round(float(np.sqrt(np.mean(amcl_err ** 2))), 3), amcl_max_m=round(float(amcl_err.max()), 3),
               plans=len(log.plans), t_start=t0, arrivals=arrivals, t_end=t_end)
    if box:
        c = (box['x'], box['y'])
        before = [p for p in log.plans if p[0] <= box['t']][-1]
        k = pl.first_clear_plan(log.plans, box['t'], c)
        box.update(plan_before_clear_m=round(pl.box_clearance(before[1], c), 2), plan_before_len_m=round(pl.path_length(before[1]), 2))
        if k is not None:
            after = log.plans[k]
            box.update(replan_after_s=round(after[0] - box['t'], 1), plan_after_clear_m=round(pl.box_clearance(after[1], c), 2),
                       plan_after_len_m=round(pl.path_length(after[1]), 2), t_replan=after[0])
        box['robot_clear_m'] = round(pl.box_clearance(gt[gt[:, 0] >= box['t'], 1:3], c), 2)   # true body centre vs box faces
        out['box_run'] = box
    res_dir = HERE / 'results'; res_dir.mkdir(exist_ok=True)
    (res_dir / f'{a.name}.json').write_text(json.dumps(out, indent=1))
    plans = np.vstack([np.c_[np.full(len(p), t), p] for t, p in log.plans if len(p)]) if log.plans else np.zeros((0, 3))
    np.savez_compressed(res_dir / f'{a.name}.npz', gt=gt, amcl=amcl, odom=odom, plans=plans)
    print(json.dumps({k: v for k, v in out.items() if k not in ('arrivals', 't_start', 't_end')}))
    ex.shutdown(); nav.destroy_node(); log.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
