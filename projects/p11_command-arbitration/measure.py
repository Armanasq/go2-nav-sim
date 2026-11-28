#!/usr/bin/env python3
"""Measure the command chain of arbitration.launch.py against ground truth.
usage: python3 projects/p11_command-arbitration/measure.py override --csv ~/p11_runs/override.csv
       python3 projects/p11_command-arbitration/measure.py estop    --csv ~/p11_runs/estop.csv   [--goal -4.5 -3.5 3.14]
       python3 projects/p11_command-arbitration/measure.py box      --csv ~/p11_runs/box.csv     [--dist 2.0]
override: Nav2 drives to a goal; a scripted keyboard holds 'j' for 2 s. Prints the delay from key to each topic.
estop:    Nav2 drives to a goal; e_stop goes true for 8 s. Prints the time to standstill (from /odom_gt), then releases it.
box:      a box is spawned --dist m ahead; the script sends 0.3 m/s on teleop/cmd_vel. Prints where the robot stopped.
Delays between topics are wall-clock (what the processes see); robot motion is in simulated seconds."""
import argparse, csv, math, os, pathlib, pty, subprocess, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / 'p02_teleop-and-safety'))
from measure_stop import stop_time          # p02: first time the 0.5 s windowed true speed stays below 0.02 m/s
from arbitration_check import first_after, stop_gap

A = (-4.5, -3.5)                             # /odom_gt starts at 0 on marker A (go_to_goal.py, make_arena.py)
TOPICS = ['teleop/cmd_vel', 'nav/cmd_vel', 'cmd_vel_mux', 'cmd_vel_cm', 'cmd_vel']
BOX = (0.4, 0.8, 0.5)                        # depth along the walk, width, height (m)


class Probe(Node):
    def __init__(self):
        super().__init__('p11_measure', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.log = {k: [] for k in TOPICS}  # (wall, sim, vx, wz)
        self.gt = []                         # (sim, x, y, yaw) in map coordinates
        for k in TOPICS:
            self.create_subscription(Twist, k, lambda m, k=k: self.log[k].append(
                (time.monotonic(), self.now(), m.linear.x, m.angular.z)), 50)
        self.create_subscription(Odometry, 'odom_gt', self.on_gt, 50)
        self.teleop = self.create_publisher(Twist, 'teleop/cmd_vel', 10)
        self.lock = self.create_publisher(Bool, 'e_stop', 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt.append((self.now(), p.x + A[0], p.y + A[1], 2 * math.atan2(q.z, q.w)))

    def wait(self, secs):
        t0 = self.now()
        while self.now() - t0 < secs:
            time.sleep(0.02)

    def until(self, cond, secs, what):
        t0 = self.now()
        while not cond():
            if self.now() - t0 > secs:
                sys.exit(f'timeout after {secs} s simulated: {what}')
            time.sleep(0.05)

    def nav_driving(self, secs=3.0):
        """Nav2 has sent forward commands above 0.15 m/s for the last `secs` simulated seconds."""
        log = self.log['nav/cmd_vel']
        t = self.now()
        recent = [vx for _, ts, vx, _ in log if ts > t - secs]
        return len(recent) > 10 * secs and min(recent) > 0.15

    def call(self, srv_type, name, req):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=20):
            sys.exit(f'service {name} not available')
        f = cli.call_async(req)
        while not f.done():
            time.sleep(0.05)
        return f.result()


def start_goal(goal):
    cmd = [sys.executable, str(ROOT / 'src/go2_nav_sim/scripts/go_to_goal.py'), *map(str, goal)]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def finish_goal(p, timeout=240):
    try:
        out, _ = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.terminate(); out, _ = p.communicate()
    res = [l for l in out.splitlines() if l.startswith('RESULT')]
    print('nav2:', res[0] if res else 'no RESULT line (go_to_goal.py did not finish)')


def ms(a, b):
    return f'{(b - a) * 1000:6.0f} ms' if a is not None and b is not None else '   none'


def override(n, a):
    goal = start_goal(a.goal)
    n.until(n.nav_driving, 120, 'Nav2 walking (nav/cmd_vel > 0.15 m/s for 3 s)')
    master, slave = pty.openpty()
    teleop = subprocess.Popen(['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', '--ros-args',
                               '-r', 'cmd_vel:=teleop/cmd_vel'], stdin=slave, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    n.wait(3.0)                                         # teleop starts and connects; Nav2 keeps driving
    t_key = time.monotonic(); s_key = n.now()
    while n.now() - s_key < 2.0:                        # hold 'j' (turn left at 1.0 rad/s) for 2 simulated seconds
        os.write(master, b'j'); time.sleep(1 / 30)
    s_rel = n.now()
    n.wait(4.0)
    os.write(master, b'\x03'); teleop.wait(timeout=10)  # Ctrl-C: teleop sends one zero as it exits
    turn = lambda w: w > 0.9                            # Nav2 never turns faster than 0.5 rad/s, so 1.0 is the key
    hit = {k: first_after([(w, z) for w, _, _, z in n.log[k]], t_key, turn) for k in TOPICS}
    for k in ['teleop/cmd_vel', 'cmd_vel_mux', 'cmd_vel_cm', 'cmd_vel']:
        if n.log[k]:                                    # cmd_vel_cm is empty without the dead-man
            print(f'key -> {k:15s}{ms(t_key, (hit[k] or [None])[0])}')
    last_key = max(s for _, s, _, z in n.log['teleop/cmd_vel'] if turn(z))
    back = first_after([(s, z) for _, s, _, z in n.log['cmd_vel']], last_key, lambda z: not turn(z))
    print(f'Nav2 back on cmd_vel {back[0] - last_key:.2f} s (simulated) after the last key event' if back else 'Nav2 never came back')
    nav_during = [1 for _, s, _, _ in n.log['nav/cmd_vel'] if s_key <= s <= s_rel]
    print(f'Nav2 kept publishing during the override: {len(nav_during)} messages in {s_rel - s_key:.1f} s')
    finish_goal(goal)


def estop(n, a):
    goal = start_goal(a.goal)
    n.until(n.nav_driving, 120, 'Nav2 walking (nav/cmd_vel > 0.15 m/s for 3 s)')
    n.lock.publish(Bool(data=True)); t_lock = n.now()
    n.wait(a.hold)
    t_stop, _ = stop_time([g[:3] for g in n.gt], t_lock)
    after = [g for g in n.gt if g[0] >= t_lock]
    walked = math.hypot(after[-1][1] - after[0][1], after[-1][2] - after[0][2])
    out_after = [1 for _, s, _, _ in n.log['cmd_vel'] if s > t_lock + 0.5]
    print(f'lock at {t_lock:.2f} s: ' + (f'standstill {t_stop - t_lock:.2f} s after the lock' if t_stop else
          f'NOT stopped {a.hold:.0f} s after the lock') + f', {walked:.2f} m walked; '
          f'{len(out_after)} messages on cmd_vel from 0.5 s after the lock on')
    n.lock.publish(Bool(data=False)); t_un = n.now()
    n.wait(3.0)
    back = first_after([(s, vx) for _, s, vx, _ in n.log['cmd_vel']], t_un, lambda v: abs(v) > 0.05)
    print(f'unlock: Nav2 moving the robot again {back[0] - t_un:.2f} s later' if back else 'unlock: no motion command within 3 s')
    finish_goal(goal)


def box(n, a):
    n.until(lambda: n.gt, 10, '/odom_gt')
    _, x, y, yaw = n.gt[-1]
    c = a.dist + BOX[0] / 2
    bx, by = x + c * math.cos(yaw), y + c * math.sin(yaw)
    sdf = (f"<sdf version='1.6'><model name='p11_box'><static>true</static><link name='l'>"
           f"<collision name='c'><geometry><box><size>{BOX[0]} {BOX[1]} {BOX[2]}</size></box></geometry></collision>"
           f"<visual name='v'><geometry><box><size>{BOX[0]} {BOX[1]} {BOX[2]}</size></box></geometry>"
           f"<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material></visual></link></model></sdf>")
    req = SpawnEntity.Request(name='p11_box', xml=sdf)
    req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = bx, by, BOX[2] / 2
    req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    r = n.call(SpawnEntity, '/spawn_entity', req)
    print(f'box spawned: {r.status_message}; face {a.dist:.2f} m ahead of base_footprint at ({bx:.2f}, {by:.2f})')
    n.wait(1.0)
    fwd = Twist(); fwd.linear.x = 0.3
    t0 = n.now(); still_since = None
    while n.now() - t0 < a.timeout:                     # a held 'i' key, through the mux at priority 100
        n.teleop.publish(fwd); time.sleep(0.1)
        t_stop, _ = stop_time([g[:3] for g in n.gt], t0 + 2.0)
        if t_stop is not None and n.now() - t_stop > 2.0:
            break
    s_end = n.now()
    for _ in range(3):
        n.teleop.publish(Twist()); time.sleep(0.1)
    along = lambda g: (g[1] - x) * math.cos(yaw) + (g[2] - y) * math.sin(yaw)
    d_end = along(n.gt[-1])
    slow = first_after([(s, vx) for _, s, vx, _ in n.log['cmd_vel']], t0 + 1.0, lambda v: 0.0 < v < 0.2)
    zero = first_after([(s, vx) for _, s, vx, _ in n.log['cmd_vel']], t0 + 1.0, lambda v: v == 0.0)
    pos_at = lambda t: along(min(n.gt, key=lambda g: abs(g[0] - t)))
    if slow:
        print(f'slowdown: cmd_vel {slow[1]:.3f} m/s at {pos_at(slow[0]):.2f} m walked (box face {a.dist - pos_at(slow[0]):.2f} m from base)')
    if zero:
        print(f'stop: first zero on cmd_vel at {pos_at(zero[0]):.2f} m walked (box face {a.dist - pos_at(zero[0]):.2f} m from base)')
    print(f'end after {s_end - t0:.1f} s: walked {d_end:.3f} m, base {a.dist - d_end:.3f} m from the box face, '
          f'gap body front to box {stop_gap(d_end, a.dist):.3f} m')
    if not a.keep:
        n.call(DeleteEntity, '/delete_entity', DeleteEntity.Request(name='p11_box'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['override', 'estop', 'box'])
    ap.add_argument('--csv', required=True)
    ap.add_argument('--goal', type=float, nargs=3, default=[4.0, 3.0, 0.65], help='x y yaw in map (default: marker B)')
    ap.add_argument('--hold', type=float, default=8.0, help='estop: seconds the lock is held')
    ap.add_argument('--dist', type=float, default=2.0, help='box: box face distance ahead of base_footprint')
    ap.add_argument('--timeout', type=float, default=25.0, help='box: longest walk, simulated seconds')
    ap.add_argument('--keep', action='store_true', help='box: leave the box in the world')
    a = ap.parse_args()
    rclpy.init()
    n = Probe()
    spin = threading.Thread(target=rclpy.spin, args=(n,), daemon=True); spin.start()
    n.until(lambda: n.now() > 0, 30, '/clock')
    try:
        {'override': override, 'estop': estop, 'box': box}[a.mode](n, a)
    finally:
        n.lock.publish(Bool(data=False)); n.teleop.publish(Twist())
        path = pathlib.Path(a.csv).expanduser(); path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['src', 'wall', 'sim', 'a', 'b'])
            for k in TOPICS:
                w.writerows([k, f'{wa:.4f}', f'{s:.4f}', f'{vx:.4f}', f'{z:.4f}'] for wa, s, vx, z in n.log[k])
            w.writerows(['gt', '', f'{s:.4f}', f'{x:.4f}', f'{y:.4f}'] for s, x, y, _ in n.gt)
        rclpy.try_shutdown(); spin.join(timeout=2)


if __name__ == '__main__':
    main()
