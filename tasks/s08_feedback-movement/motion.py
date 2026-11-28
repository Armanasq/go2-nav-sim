"""Stage 8 logic with no ROS in it: the plans of every task, the controller adapters and the scores.
All poses are in the start frame of a run: origin at the robot's start pose, x forward, y to the left.
The controllers come from project 3 (shapes.py: timed schedules, the carrot Follower) and project 4 (polar.py: GoToPose)."""
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'projects/p03_move-in-shapes'), str(ROOT / 'projects/p04_go-to-pose')]
import shapes  # noqa: E402
import polar  # noqa: E402

wrap = shapes.wrap
POINT = (1.5, 0.5)                    # 8.01, 8.02
LINE = 2.5                            # 8.05, m along x
CIRCLE_R = 0.6                        # 8.07
WAYPOINTS = [(1.0, 0.0), (2.0, 0.6), (1.2, 1.4), (0.3, 0.8), (0.0, 0.0)]     # 8.09
POSE = (1.5, 0.5, math.radians(90))   # 8.04
OUT_AND_BACK = [(2.0, 1.0, math.radians(180)), (0.0, 0.0, 0.0)]            # 8.11
HEADINGS = [90, -90, 0]               # 8.03, deg
PUSH_LINE = 3.0                       # 8.10: line length, m (the push is in params/push.yaml)
SETTLE = 1.5                          # s standing still after a goal before its error is taken


def to_start(x, y, yaw, x0, y0, a0):
    """Pose in a frame -> the same pose in the start frame given by (x0, y0, a0) in that frame."""
    c, s = math.cos(a0), math.sin(a0)
    return c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0), wrap(yaw - a0)


# ---- controller adapters: every plan is step(t, x, y, yaw) -> (v, w, label) or None when finished ----------------------
class Timed:
    """A project 3 schedule of (duration, v, w, label), played on time since the first step."""
    def __init__(self, schedule):
        self.schedule, self.t0 = schedule, None

    def step(self, t, x, y, yaw):
        self.t0 = t if self.t0 is None else self.t0
        return shapes.command_at(self.schedule, t - self.t0)


class Poses:
    """Project 4's GoToPose to each goal in turn, standing SETTLE s after each one (label settle<k>)."""
    def __init__(self, goals, timeout=90.0):
        self.goals, self.k, self.ctl, self.t_goal, self.t_done, self.timeout = list(goals), 0, None, None, None, timeout

    def step(self, t, x, y, yaw):
        if self.k == len(self.goals):
            return None
        if self.ctl is None:
            self.ctl, self.t_goal, self.t_done = polar.GoToPose(self.goals[self.k]), t, None
        v, w, phase = self.ctl.step(x, y, yaw)
        if phase != 'done' and t - self.t_goal > self.timeout:
            self.ctl.phase, v, w, phase = 'timeout', 0.0, 0.0, 'timeout'
        if phase in ('done', 'timeout'):
            self.t_done = self.t_done or t
            if t - self.t_done >= SETTLE:
                self.k, self.ctl = self.k + 1, None
            return 0.0, 0.0, f'settle{self.k}'
        return v, w, f'{phase}{self.k}'


def timed_point(p=POINT, v=0.25, w=0.4):
    """Turn towards p, then walk the distance, both from time: theta = w T, d = v T."""
    th, d = math.atan2(p[1], p[0]), math.hypot(*p)
    return Timed([(abs(th) / w, 0.0, math.copysign(w, th), 'turn'), (d / v, v, 0.0, 'walk'), (SETTLE, 0.0, 0.0, 'settle')])


class Circle:
    """One counter-clockwise circle of radius r through the start, centre r to the left. Feedforward turn rate v / r plus
    feedback on the radial error e = d - r and on the heading error to the tangent: w = v / r + k_e v e + k_h theta_e.
    Linearised, e'' + k_h e' + k_e v^2 e = 0: with v = 0.18, k_e = 10, k_h = 1.5 that is omega_n = 0.57 rad/s, zeta = 1.3.
    Stops when the robot has gone 2 pi around the centre."""
    def __init__(self, r=CIRCLE_R, v=0.18, k_e=10.0, k_h=1.5):
        self.r, self.v, self.k_e, self.k_h, self.phi, self.swept = r, v, k_e, k_h, None, 0.0

    def step(self, t, x, y, yaw):
        qx, qy = x, y - self.r
        phi = math.atan2(qy, qx)
        self.swept += 0.0 if self.phi is None else wrap(phi - self.phi)
        self.phi = phi
        if self.swept >= 2 * math.pi:
            return None
        e, th = math.hypot(qx, qy) - self.r, wrap(phi + math.pi / 2 - yaw)
        return self.v, self.v / self.r + self.k_e * self.v * e + self.k_h * th, 'circle'


def plan(tid, mode='closed'):
    """The controller for a task. mode 'open' exists for 8.10 (the same line under timed commands)."""
    F = shapes.Follower
    return {
        '8.01': lambda: timed_point(),
        '8.02': lambda: F([POINT]),
        '8.03': lambda: Headings([math.radians(h) for h in HEADINGS]),
        '8.04': lambda: Poses([POSE]),
        '8.05': lambda: F([(LINE, 0.0)]),
        '8.06': lambda: F(shapes.square_corners(), final_yaw=0.0),
        '8.07': lambda: Circle(),
        '8.08': lambda: F(shapes.figure8_points(), stop_each=False, v=0.18, lookahead=0.25, reach_tol=0.05),   # as p03 drive.py
        '8.09': lambda: F(WAYPOINTS),
        '8.10': lambda: F([(PUSH_LINE, 0.0)]) if mode == 'closed' else Timed([(PUSH_LINE / 0.25, 0.25, 0.0, 'walk'),
                                                                              (SETTLE, 0.0, 0.0, 'settle')]),
        '8.11': lambda: Poses(OUT_AND_BACK),
    }[tid]()


class Headings:
    """Turn on the spot to each heading in turn with the follower's turn law, standing SETTLE s after each (settle<k>)."""
    def __init__(self, yaws, tol=math.radians(2)):
        self.yaws, self.tol, self.k, self.t_done = yaws, tol, 0, None
        self.turn = shapes.Follower([])._turn

    def step(self, t, x, y, yaw):
        if self.k == len(self.yaws):
            return None
        label = f'settle{self.k}'
        if self.t_done is None:
            err = wrap(self.yaws[self.k] - yaw)
            if abs(err) > self.tol:
                return 0.0, self.turn(err), f'turn{self.k}'
            self.t_done = t
        if t - self.t_done >= SETTLE:
            self.k, self.t_done = self.k + 1, None
        return 0.0, 0.0, label


# ---- scores: rows are dicts with t, label, x, y, yaw (/odom) and gx, gy, gyaw (/odom_gt), all in the start frame --------
def last_with(rows, label):
    return [r for r in rows if r['label'] == label][-1]


def dist(r, p, key='g'):
    return math.hypot(r[key + 'x'] - p[0], r[key + 'y'] - p[1])


def odom_gap(r):
    """How far /odom is from the truth at this row (both in the start frame)."""
    return math.hypot(r['x'] - r['gx'], r['y'] - r['gy'])


def radial(rows, centre, r0):
    return [math.hypot(r['gx'] - centre[0], r['gy'] - centre[1]) - r0 for r in rows]


def line_recovery(rows, t_push, band=0.05):
    """For a line along x: peak |y| after the push, and the time after the push from which |y| stays inside band."""
    after = [r for r in rows if r['t'] >= t_push]
    peak = max(abs(r['gy']) for r in after)
    out = [r['t'] for r in after if abs(r['gy']) > band]
    return peak, (out[-1] - t_push if out else 0.0)


def unicycle(ctl, x=0.0, y=0.0, yaw=0.0, dt=0.05, t_max=300.0):
    """Drive a controller on a perfect robot (does exactly what it is told) and return the rows; for the tests."""
    rows, t = [], 0.0
    while t < t_max:
        cmd = ctl.step(t, x, y, yaw)
        if cmd is None:
            break
        v, w, label = cmd
        rows.append(dict(t=t, label=label, x=x, y=y, yaw=yaw, gx=x, gy=y, gyaw=yaw))
        x, y, yaw, t = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, wrap(yaw + w * dt), t + dt
    rows.append(dict(t=t, label='stop', x=x, y=y, yaw=yaw, gx=x, gy=y, gyaw=yaw))
    return rows
