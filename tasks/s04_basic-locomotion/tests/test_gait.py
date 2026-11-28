"""Fast checks of gait.py on synthetic data (no ROS, no simulator):  python3 -m pytest -q tests"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import gait as g  # noqa: E402


def trot(t, period=0.5, swing=0.25, offsets=(0.0, 0.5, 0.5, 0.0), h=0.03):
    """Clearance of four feet: a half-sine bump of height h during each swing, swing starting at offset * period."""
    out = {}
    for f, o in zip(g.FEET, offsets):
        ph = ((t / period - o) % 1.0) * period
        out[f] = np.where(ph < swing, h * np.sin(np.pi * ph / swing), 0.0)
    return out


def test_foot_from_identity_and_rotated_calf():
    assert np.allclose(g.foot_from_calf(1, 2, 0.3, 0, 0, 0, 1), (1, 2, 0.3 - g.CALF))
    s = math.sin(math.pi / 4)                      # calf pitched +90 deg about y: its -z axis points along -x
    assert np.allclose(g.foot_from_calf(0, 0, 0.3, 0, s, 0, s), (-g.CALF, 0, 0.3), atol=1e-12)


def test_swings_hysteresis_and_open_swing_dropped():
    c = [0, 0.003, 0.006, 0.02, 0.003, 0.001, 0, 0.01, 0.02]   # 0.003 neither lifts nor lands; last swing never lands
    assert g.swings(c) == [(2, 5, 0.02)]


def test_gait_stats_recovers_a_trot():
    t = np.arange(0, 5, 0.01)
    c = trot(t + 0.003)                            # shift off the sample grid
    s = g.gait_stats(t, {f: g.swings(c[f]) for f in g.FEET})
    assert abs(s['period'] - 0.5) < 0.011
    assert abs(s['duty'] - 0.5) < 0.06             # thresholds shorten the measured swing a little
    assert abs(s['phase']['rf'] - 0.5) < 0.02 and abs(s['phase']['lh'] - 0.5) < 0.02
    assert abs(g.wrapped_phase(s['phase']['rh'])) < 0.02


def test_gait_stats_tells_a_walk_from_a_trot():
    t = np.arange(0, 8, 0.01)
    c = trot(t, period=1.0, swing=0.25, offsets=(0.0, 0.5, 0.75, 0.25))   # lateral-sequence walk, duty 0.75
    s = g.gait_stats(t, {f: g.swings(c[f]) for f in g.FEET})
    assert abs(s['phase']['lh'] - 0.75) < 0.02 and s['duty'] > 0.7


def test_speed_crossing_line_and_arc():
    t = np.arange(0, 4, 0.02)
    x = np.where(t < 1, 0, 0.3 * (t - 1))
    tc, v = g.windowed_speed(t, x, 0 * x)
    assert abs(v[-1] - 0.3) < 1e-9
    assert abs(g.crossing_time(tc, v, 0.15, 0.0) - 1.0) < 0.02      # centred 0.5 s window: half speed at the step itself
    rise = g.crossing_time(tc, v, 0.27, 0.0) - g.crossing_time(tc, v, 0.03, 0.0)
    assert abs(rise - 0.4) < 0.03                                  # the window alone gives a 10-90 % rise of 0.8 x 0.5 s
    along, cross = g.line_errors([0, 0.5, 1.0], [0, 0.02, -0.01], 1, 0)
    assert along == 1.0 and cross == 0.02
    th = np.linspace(0, math.pi / 2, 50)
    assert np.max(g.arc_deviation(0.5 * np.sin(th), 0.5 - 0.5 * np.cos(th), 0.5)) < 1e-12


def test_to_start_and_wrap():
    x, y, yaw = g.to_start(1, 1, math.pi, 1, 0, math.pi / 2)
    assert np.allclose((x, y, yaw), (1, 0, math.pi / 2))
    assert abs(g.wrap(3 * math.pi) - math.pi) < 1e-12 or abs(g.wrap(3 * math.pi) + math.pi) < 1e-12
