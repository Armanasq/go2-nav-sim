"""Reactive steering from one 2D scan and a pose estimate. Pure logic, no ROS, so it can be tested and run in arena2d.py.

Two ways to pick the direction to walk in:
  apf  potential field: a constant pull towards the goal plus a push away from every return closer than d0.
  vfh  free-direction histogram (the idea of VFH, simplified): of the directions whose corridor of half-width r_safe is free
       for `look` metres, take the one nearest to the goal.
Both then use the same rule to turn that direction into (v, w) for CHAMP."""
import math

import numpy as np

# Tuned in arena2d.py, then checked in Gazebo. The speed limits are CHAMP's: 0.3 m/s forward, 0.5 rad/s turn.
P = dict(v_max=0.3, w_max=0.5, k_w=1.2, front_half=math.radians(35), stop=0.5, slow=1.2, goal_tol=0.3,
         k_att=1.0, k_rep=1.0, d0=1.5,                                # apf
         r_safe=0.55, look=2.0, n_dir=180, k_turn=0.3, k_prev=0.5)    # vfh


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """Pose b expressed in a's frame -> the same pose in the frame a is expressed in (SE(2))."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def goal_in_robot(pose, goal):
    """Goal (world x, y) seen from pose (x, y, yaw): x forward, y left."""
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def valid_returns(ranges, angle_min, angle_inc, range_min, range_max):
    """Ranges and bearings of the beams that hit something (inf = no return, outside [min, max] = unusable)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return r[ok], a[ok]


def apf_direction(r, a, g, angle_inc, p=P):
    """Bearing of att + rep. Each return at d < d0 pushes with k_rep * (1/d - 1/d0) / d^2 (Khatib's potential,
    differentiated), weighted by the beam's angular width so the sum does not depend on the scan resolution."""
    att = p['k_att'] * np.asarray(g) / max(math.hypot(*g), 1e-9)
    near = r < p['d0']
    d, b = r[near], a[near]
    mag = p['k_rep'] * (1 / d - 1 / p['d0']) / d ** 2 * angle_inc
    f = att - np.array([np.sum(mag * np.cos(b)), np.sum(mag * np.sin(b))])
    return math.atan2(f[1], f[0])


def free_directions(r, a, look, r_safe, n_dir):
    """Candidate bearings (n_dir around the robot) and whether each corridor (half-width r_safe, length look) is free.
    A return at (d, b) blocks bearing t when it lies ahead along t (0 < d cos(b - t) < look) and
    closer than r_safe to the centre line (|d sin(b - t)| < r_safe)."""
    t = np.linspace(-math.pi, math.pi, n_dir, endpoint=False)
    rel = a[None, :] - t[:, None]
    along, side = r * np.cos(rel), np.abs(r * np.sin(rel))
    blocked = ((along > 0) & (along < look) & (side < r_safe)).any(axis=1)
    return t, ~blocked


def vfh_direction(r, a, g, prev=None, p=P):
    """Free bearing with the lowest cost: angle to the goal, plus k_turn x angle from straight ahead, plus k_prev x angle
    from the bearing chosen last time (prev, robot frame). The last term stops the choice flipping between two gaps of
    nearly equal cost. None if every bearing is blocked."""
    dist = math.hypot(*g)
    t, free = free_directions(r, a, min(p['look'], dist), p['r_safe'], int(p['n_dir']))
    if not free.any():
        return None
    ang = lambda x: np.abs(np.arctan2(np.sin(t - x), np.cos(t - x)))
    cost = ang(math.atan2(g[1], g[0])) + p['k_turn'] * np.abs(t) + (0 if prev is None else p['k_prev'] * ang(prev))
    cost[~free] = np.inf
    return float(t[np.argmin(cost)])


def command(th, front, p=P):
    """(v, w) that turns towards bearing th and walks forward more slowly the more it has to turn and the nearer the
    closest return in front. th None (nowhere to go): turn on the spot."""
    if th is None:
        return 0.0, p['w_max']
    w = max(-p['w_max'], min(p['w_max'], p['k_w'] * th))
    brake = min(1.0, max(0.0, (front - p['stop']) / (p['slow'] - p['stop'])))
    return p['v_max'] * max(0.0, math.cos(th)) * brake, w


def step(ranges, angle_min, angle_inc, range_min, range_max, pose, goal, method='vfh', prev=None, p=P):
    """One control step: scan + pose estimate + goal -> (v, w, info). prev: info['dir'] of the last step (world frame).
    The caller keeps prev, so this function holds no state."""
    r, a = valid_returns(ranges, angle_min, angle_inc, range_min, range_max)
    g = goal_in_robot(pose, goal)
    if method == 'apf':
        th = apf_direction(r, a, g, angle_inc, p)
    else:
        th = vfh_direction(r, a, g, None if prev is None else wrap(prev - pose[2]), p)
    front = float(r[np.abs(a) <= p['front_half']].min(initial=math.inf))
    v, w = command(th, front, p)
    return v, w, dict(dist=math.hypot(*g), front=front, th=th, dir=None if th is None else wrap(th + pose[2]))


class Progress:
    """Local-minimum detector: stuck when the best distance to the goal has not improved by eps for window seconds."""

    def __init__(self, window=30.0, eps=0.25):
        self.window, self.eps, self.best, self.t_best = window, eps, math.inf, None

    def stuck(self, t, dist):
        if self.t_best is None or dist < self.best - self.eps:
            self.best, self.t_best = dist, t
        return t - self.t_best > self.window
