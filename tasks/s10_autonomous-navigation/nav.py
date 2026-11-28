#!/usr/bin/env python3
"""Stage-10 Nav2 missions with Simple Commander, scored against simulator ground truth (/odom_gt).
usage: python3 nav.py <mode> --world <world> --out <run.json> [mode options]
  goto      x y yaw                one goal (10.01-10.07, 10.15)
  replan                           goal B; a box is spawned on the plan 1.5 m ahead once the robot has walked 1.0 m (10.08)
  reject    x y yaw [x y yaw ...]  goals Nav2 should refuse, one after another; reports each action status code (10.09)
  cancel    --n 3                  goal B, cancelled after 0.8 m of walking, n times in a row (10.10)
  waypoints x,y,yaw x,y,yaw ...    FollowWaypoints (10.12)
  return    x y yaw                go to x y yaw, then back to A (10.14)
AMCL is first set from the true pose by go_to_goal.py, sent to the start pose (the same shortcut as every run in this repo).
Writes <run.json> (numbers) and <run>.csv (true track: t, x, y, yaw)."""
import argparse, json, math, pathlib, subprocess, sys, threading, time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.srv import SpawnEntity
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE / 'worlds')); sys.path.insert(0, str(HERE))
import make_worlds as mw
import score
A = mw.A
STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}          # action_msgs/GoalStatus
BOX = (0.3, 1.6, 0.8)                                            # replan box: depth along the path, width, height (m)


class Log(Node):
    """True track, AMCL estimate, global plans and velocity commands, all stamped in simulated time."""
    def __init__(self):
        super().__init__('s10_log', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt, self.amcl, self.plans, self.cmd, self.odom = [], [], [], [], []
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 50)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom.append(      # EKF output, odom frame = A at start
            (self.now(), m.pose.pose.position.x + A[0], m.pose.pose.position.y + A[1])), 50)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: self.amcl.append(
            (self.now(), m.pose.pose.position.x, m.pose.pose.position.y)),
            QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Path, '/plan', lambda m: self.plans.append(
            (self.now(), np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]))), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.cmd.append((self.now(), m.linear.x, m.angular.z)), 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt.append((self.now(), p.x + A[0], p.y + A[1], 2 * math.atan2(q.z, q.w)))

    def wait(self, secs):
        t0 = self.now()
        while self.now() - t0 < secs:
            time.sleep(0.02)

    def walked_since(self, t0):
        """Straight-line distance of the true position from where it was at t0."""
        g0 = next(g for g in self.gt if g[0] >= t0)
        return math.hypot(self.gt[-1][1] - g0[1], self.gt[-1][2] - g0[2])


def pose(nav, x, y, yaw):
    p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def run_task(nav, log, start, on_tick=None, limit=300.0):
    """Send one action (start() does it), wait for its end; on_tick(t) may act while it runs. Returns a result dict."""
    t0 = log.now(); start()
    while not nav.isTaskComplete():
        if on_tick:
            on_tick(log.now() - t0)
        if log.now() - t0 > limit:
            nav.cancelTask()
            while not nav.isTaskComplete():
                time.sleep(0.05)
            break
        time.sleep(0.05)
    res = nav.getResult()
    return dict(timed_out=log.now() - t0 > limit, result=res.name, status=int(nav.status), status_name=STATUS.get(int(nav.status), str(nav.status)),
                sim_time_s=round(log.now() - t0, 2), t_start=t0, t_end=log.now())


def spawn_box(log, x, y, yaw):
    sdf = (f"<sdf version='1.6'><model name='s10_box'><static>true</static><link name='l'>"
           f"<collision name='c'><geometry><box><size>{BOX[0]} {BOX[1]} {BOX[2]}</size></box></geometry></collision>"
           f"<visual name='v'><geometry><box><size>{BOX[0]} {BOX[1]} {BOX[2]}</size></box></geometry>"
           f"<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material></visual></link></model></sdf>")
    req = SpawnEntity.Request(name='s10_box', xml=sdf)
    req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = x, y, BOX[2] / 2
    req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    cli = log.create_client(SpawnEntity, '/spawn_entity')
    if not cli.wait_for_service(timeout_sec=20):
        sys.exit('service /spawn_entity not available')
    f = cli.call_async(req)
    while not f.done():
        time.sleep(0.05)
    return f.result().success


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['goto', 'replan', 'reject', 'cancel', 'waypoints', 'return'])
    ap.add_argument('args', nargs='*')
    ap.add_argument('--world', required=True, help='geometry used for the clearance numbers (worlds/make_worlds.py)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=3)
    a = ap.parse_args()
    w = mw.WORLDS[a.world]

    # AMCL from the true pose: go_to_goal.py does it (with its retry loop), then walks to its goal: here the start pose itself
    loc = subprocess.run([sys.executable, str(ROOT / 'src/go2_nav_sim/scripts/go_to_goal.py'), str(A[0]), str(A[1]), '0.0'],
                         capture_output=True, text=True, timeout=240)
    rclpy.init()
    log = Log(); ex = SingleThreadedExecutor(); ex.add_node(log)
    threading.Thread(target=ex.spin, daemon=True).start()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    nav._waitForNodeToActivate('bt_navigator')
    while not log.gt:
        time.sleep(0.1)
    out = dict(mode=a.mode, world=a.world, localise=[l for l in loc.stdout.splitlines() if l.startswith('RESULT')][-1:])
    triples = [[float(v) for v in a.args[i:i + 3]] for i in range(0, len(a.args) - 2, 3)]
    goals = {'goto': triples[:1], 'reject': triples, 'replan': [list(w['goal'])], 'return': triples[:1] + [[A[0], A[1], 0.0]]}
    legs = []

    if a.mode in goals:
        box = {}
        def on_tick(t):                           # replan: spawn once, on the latest plan 1.5 m ahead of the robot
            if a.mode != 'replan' or box or not log.plans or log.walked_since(t_go) < 1.0:
                return
            _, x, y, _ = log.gt[-1]
            pts = log.plans[-1][1]
            s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
            i0 = int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y)))
            i = min(int(np.searchsorted(s, s[i0] + 1.5)), len(pts) - 2)
            yaw = math.atan2(*(pts[i + 1] - pts[i])[::-1])
            box.update(x=float(pts[i, 0]), y=float(pts[i, 1]), yaw=yaw, t=log.now(), ok=spawn_box(log, pts[i, 0], pts[i, 1], yaw))
        for goal in goals[a.mode]:
            if legs:
                log.wait(2.0)
            t_go = log.now()
            legs.append(run_task(nav, log, lambda: nav.goToPose(pose(nav, *goal)), on_tick, limit=120 if a.mode == 'reject' else 240))
            legs[-1]['goal'] = goal
        if box:
            out['box'] = box
    elif a.mode == 'cancel':
        for k in range(a.n):
            target = list(w['goal']) if k % 2 == 0 else [A[0], A[1], 0.0]       # alternate, so the robot stays in the room
            state = {}
            def on_tick(t, state=state):
                if 'cancel' not in state and log.walked_since(state.setdefault('t0', log.now())) >= 0.8:
                    state['cancel'] = log.now(); nav.cancelTask()
            r = run_task(nav, log, lambda: nav.goToPose(pose(nav, *target)), on_tick)
            r.update(goal=target, t_cancel=state.get('cancel'))
            log.wait(6.0)                         # watch the robot stop
            legs.append(r)
    elif a.mode == 'waypoints':
        wps = [[float(v) for v in s.split(',')] for s in a.args]
        reached = []
        def on_tick(t):
            fb = nav.getFeedback()
            if fb is not None and fb.current_waypoint > len(reached):
                reached.append(log.gt[-1][1:3])   # the follower moved on: the previous waypoint was reached here
        legs.append(run_task(nav, log, lambda: nav.followWaypoints([pose(nav, *p) for p in wps]), on_tick, limit=400))
        res = nav.result_future.result().result if nav.result_future else None
        legs[-1].update(goal=wps[-1], waypoints=wps, missed=list(res.missed_waypoints) if res else None,
                        reached_at=[list(map(float, r)) for r in reached])
    log.wait(3.0)

    for leg in legs:                              # score each leg on the true track
        leg.update(score.leg(log.gt, log.amcl, leg['t_start'], leg['t_end'], leg['goal'], w))
        leg['odom_end_error_m'] = score.odom_end_error(log.gt, log.odom, leg['t_end'])
        if leg.get('t_cancel'):
            leg.update(score.stop(log.gt, leg['t_cancel']))
    if 'box' in out:
        out['box'].update(score.replan(log.plans, log.gt, out['box'], BOX, legs[0]['t_end']))
    out['legs'] = legs
    out['after_end_walked_m'] = round(log.walked_since(legs[-1]['t_end']), 3)
    path = pathlib.Path(a.out); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1))
    np.savetxt(path.with_suffix('.csv'), np.array(log.gt), delimiter=',', fmt='%.3f', header='t,x,y,yaw', comments='')
    for suffix, rows, head in [('_amcl', log.amcl, 't,x,y'), ('_cmd', log.cmd, 't,vx,wz'), ('_odom', log.odom, 't,x,y')]:
        if rows:
            np.savetxt(path.with_name(path.stem + suffix + '.csv'), np.array(rows), delimiter=',', fmt='%.3f', header=head, comments='')
    if log.plans and a.mode == 'replan':          # plans: one row per pose, first column the plan's stamp
        np.savetxt(path.with_name(path.stem + '_plans.csv'), np.vstack([np.c_[np.full(len(p), t), p] for t, p in log.plans if len(p)]),
                   delimiter=',', fmt='%.3f', header='t,x,y', comments='')
    print(json.dumps({k: v for k, v in out.items() if k != 'legs'}), '\n' + '\n'.join(
        f"leg {i}: {l['status_name']} {l['sim_time_s']} s, final error {l['final_error_m']} m, clearance {l['min_clearance_m']} m"
        for i, l in enumerate(legs)))
    ex.shutdown(); nav.destroy_node(); log.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
