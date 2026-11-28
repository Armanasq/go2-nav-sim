"""Stage 4 scoring, no ROS: foot position from a calf pose, swing detection, gait timing, speed, path errors.
Everything here is tested by tests/test_gait.py on synthetic data."""
import math
import numpy as np

FEET = ('lf', 'rf', 'lh', 'rh')
CALF = 0.213          # m, calf_length in go2_description/xacro/const.xacro: foot centre = lower_leg_link origin - 0.213 z_calf
UP, DOWN = 0.004, 0.002   # m of clearance: lift-off above UP, touchdown below DOWN (hysteresis against contact jitter)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def foot_from_calf(px, py, pz, qx, qy, qz, qw):
    """Foot centre in the world from the pose of <leg>_lower_leg_link: p + R(q) (0, 0, -CALF)."""
    return (px - CALF * 2 * (qx * qz + qw * qy),
            py - CALF * 2 * (qy * qz - qw * qx),
            pz - CALF * (1 - 2 * (qx * qx + qy * qy)))


def to_start(x, y, yaw, x0, y0, yaw0):
    """Pose in the frame of the start pose (x forward, y left)."""
    c, s = math.cos(yaw0), math.sin(yaw0)
    dx, dy = x - x0, y - y0
    return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - yaw0)


def swings(c, up=UP, down=DOWN):
    """Swing phases of one foot from its clearance samples: [(i_lift, i_land, peak)]. A swing still open at the end is dropped."""
    out, lift, peak = [], None, 0.0
    for i, ci in enumerate(c):
        if lift is None:
            if ci > up:
                lift, peak = i, ci
        else:
            peak = max(peak, ci)
            if ci < down:
                out.append((lift, i, peak))
                lift = None
    return out


def gait_stats(t, sw, ta=-math.inf, tb=math.inf):
    """Timing of a gait from the swings of the four feet (dict foot -> swings(), indices into t) between ta and tb.
    period: median time between lift-offs of the same foot; duty factor: 1 - median swing time / period;
    phase of a foot: lift-off time after the last lf lift-off, as a fraction of the period, circular mean."""
    lifts = {f: [t[a] for a, b, p in sw[f] if ta <= t[a] <= tb] for f in FEET}
    durs = [t[b] - t[a] for f in FEET for a, b, p in sw[f] if ta <= t[a] <= tb]
    diffs = [d for f in FEET for d in np.diff(lifts[f])]
    if not diffs or len(lifts['lf']) < 2:
        return None
    period = float(np.median(diffs))
    phase = {}
    for f in FEET:
        ang = []
        for tf in lifts[f]:
            before = [t0 for t0 in lifts['lf'] if t0 <= tf + 1e-9]
            if before:
                ang.append(2 * math.pi * ((tf - before[-1]) / period % 1.0))
        phase[f] = (math.atan2(np.mean(np.sin(ang)), np.mean(np.cos(ang))) / (2 * math.pi)) % 1.0 if ang else None
    return dict(period=period, duty=1 - float(np.median(durs)) / period, phase=phase,
                n_lifts={f: len(lifts[f]) for f in FEET})


def wrapped_phase(p):
    """Phase in [-0.5, 0.5): 0.98 -> -0.02."""
    return (p + 0.5) % 1.0 - 0.5


def windowed_speed(t, x, y, win=0.5):
    """Speed over a sliding window of win seconds (one stride, so the gait bounce averages out), stamped at the window centre."""
    t, x, y = map(np.asarray, (t, x, y))
    j = np.searchsorted(t, t - win)
    dt = t - t[j]
    ok = dt >= 0.8 * win
    v = np.hypot(x - x[j], y - y[j])[ok] / dt[ok]
    return ((t + t[j]) / 2)[ok], v


def crossing_time(t, v, level, after, rising=True):
    """First time >= after at which v crosses level (upwards if rising), linearly interpolated; None if never."""
    for i in range(1, len(t)):
        if t[i] < after:
            continue
        a, b = v[i - 1], v[i]
        if (rising and a < level <= b) or (not rising and a > level >= b):
            return float(t[i - 1] + (level - a) / (b - a) * (t[i] - t[i - 1]))
    return None


def line_errors(x, y, ux, uy):
    """Along-track distance of the last point and the largest cross-track distance from the line through 0 along u."""
    x, y = np.asarray(x), np.asarray(y)
    return float(x[-1] * ux + y[-1] * uy), float(np.max(np.abs(x * uy - y * ux)))


def arc_deviation(x, y, r):
    """Distance of each point from the circle of radius r centred on (0, r): the path of vx = r wz starting along +x."""
    return np.abs(np.hypot(np.asarray(x), np.asarray(y) - r) - r)
