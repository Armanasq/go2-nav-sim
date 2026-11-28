"""The pure logic of project 03, with no ROS in it so it can be tested on its own.
Shapes are given in the start frame: origin at the robot, x forward, y to the left."""
import math

SIDE = 1.5      # square side, m
RADIUS = 0.45   # figure-eight lobe radius, m


def wrap(a):
    """Angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def square_corners(side=SIDE):
    """Corners of a square walked counter-clockwise, ending back at the start."""
    return [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]


def figure8_points(r=RADIUS, step_deg=30):
    """Points on two circles that touch at the start: the left lobe counter-clockwise, then the right lobe clockwise."""
    pts = []
    for sign in (1, -1):                                   # +1: centre (0, r); -1: centre (0, -r)
        for k in range(1, 360 // step_deg + 1):
            a = math.radians(k * step_deg)
            pts.append((r * math.sin(a), sign * (r - r * math.cos(a))))
    return pts


# ---------------------------------------------------------------------------------------------------------- open loop
def square_schedule(side=SIDE, v=0.25, w=0.4, settle=1.5, v_real=None, w_real=None):
    """Timed commands (duration s, v m/s, w rad/s, label). Each side takes side / v seconds, each turn (pi/2) / w.
    v_real, w_real: the speeds the robot is known to reach for v and w; if given, the durations are computed from them."""
    T_side, T_turn = side / (v_real or v), math.pi / 2 / (w_real or w)
    seg = []
    for k in range(4):
        seg += [(T_side, v, 0.0, f'side{k}'), (settle, 0.0, 0.0, f'corner{k}'),
                (T_turn, 0.0, w, f'turn{k}'), (settle, 0.0, 0.0, f'turned{k}')]
    return seg


def figure8_schedule(r=RADIUS, v=0.18, settle=1.5):
    """One full circle each way at constant v and w = v / r, so each lobe takes 2 pi r / v seconds."""
    w, T = v / r, 2 * math.pi * r / v
    return [(T, v, w, 'lobe0'), (T, v, -w, 'lobe1'), (settle, 0.0, 0.0, 'end')]


def command_at(schedule, t):
    """(v, w, label) for time t since the start, or None once the schedule is over."""
    for dur, v, w, label in schedule:
        if t < dur:
            return v, w, label
        t -= dur
    return None


# -------------------------------------------------------------------------------------------------------- closed loop
def clip(x, hi):
    return max(-hi, min(hi, x))


class Follower:
    """Visit waypoints using the measured pose. Each leg runs from the previous waypoint to the next one. The robot steers
    towards a carrot: the point `lookahead` metres further along the leg's line than the robot's own projection onto it, so
    the heading converges onto the line instead of chasing a point it is about to reach.
    Turn on the spot until the heading error is below turn_tol, then walk until the distance left along the leg is below
    reach_tol. stop_each=True stands still for `settle` s at every waypoint (square); False goes straight on to the next leg
    and turns on the spot only when the heading error exceeds pass_tol (figure-eight). final_yaw is turned to at the end."""

    def __init__(self, pts, stop_each=True, final_yaw=None, v=0.25, w_max=0.5, w_min=0.12, k_turn=1.5, k_head=1.0,
                 lookahead=0.4, turn_tol=math.radians(2), pass_tol=math.radians(45), reach_tol=0.02, settle=1.5):
        self.pts, self.stop_each, self.final_yaw = list(pts), stop_each, final_yaw
        self.v, self.w_max, self.w_min, self.k_turn, self.k_head = v, w_max, w_min, k_turn, k_head
        self.lookahead, self.turn_tol, self.pass_tol = lookahead, turn_tol, pass_tol
        self.reach_tol, self.settle = reach_tol, settle
        self.i, self.mode, self.t_mode, self.prev = 0, 'turn', None, None

    def _turn(self, err):
        w = clip(self.k_turn * err, self.w_max)
        return math.copysign(max(abs(w), self.w_min), err)     # floor, so the last degrees of a turn do not crawl

    def leg(self, x, y, yaw):
        """(distance left along the leg, heading error towards the carrot) for the current leg."""
        assert self.prev is not None
        (px, py), (tx, ty) = self.prev, self.pts[self.i]
        L = math.hypot(tx - px, ty - py) or 1e-9
        ux, uy = (tx - px) / L, (ty - py) / L
        s = (x - px) * ux + (y - py) * uy                       # robot's projection onto the leg
        cx, cy = px + (s + self.lookahead) * ux, py + (s + self.lookahead) * uy
        return L - s, wrap(math.atan2(cy - y, cx - x) - yaw)

    def step(self, t, x, y, yaw):
        """(v, w, label) for the current pose, or None when finished."""
        if self.prev is None:
            self.prev = (x, y)
        if self.mode == 'settle':
            if t - self.t_mode < self.settle:
                return 0.0, 0.0, f'corner{self.i - 1}'
            self.mode = 'turn'
        if self.i == len(self.pts):                               # all points visited: final heading, then stop
            err = 0.0 if self.final_yaw is None else wrap(self.final_yaw - yaw)
            if abs(err) < self.turn_tol:
                return None
            return 0.0, self._turn(err), 'final_turn'
        left, err = self.leg(x, y, yaw)
        if self.mode == 'turn':
            if abs(err) > self.turn_tol:
                return 0.0, self._turn(err), f'turn{self.i}'
            self.mode = 'walk'
        if left < self.reach_tol:                                 # reached (or passed) this waypoint
            self.prev, self.i = self.pts[self.i], self.i + 1
            if self.stop_each:
                self.mode, self.t_mode = 'settle', t
            elif self.i < len(self.pts) and abs(self.leg(x, y, yaw)[1]) > self.pass_tol:
                self.mode = 'turn'
            return self.step(t, x, y, yaw)
        v = min(self.v, max(0.1, left)) if self.stop_each else self.v     # slow down only before a stop
        return v, clip(self.k_head * err, self.w_max), f'walk{self.i}'
