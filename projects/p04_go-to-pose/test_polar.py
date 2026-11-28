"""Unit tests for polar.py on an ideal unicycle (it does exactly what it is commanded). Run:
  python3 -m pytest -q projects/p04_go-to-pose"""
import math, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import polar


def simulate(goal, start=(0.0, 0.0, 0.0), gains=polar.GAINS, dt=0.05, t_max=120.0):
    x, y, yaw = start
    ctl, peak = polar.GoToPose(goal, gains), (0.0, 0.0)
    for _ in range(int(t_max / dt)):
        v, w, phase = ctl.step(x, y, yaw)
        if phase == 'done':
            break
        peak = (max(peak[0], abs(v)), max(peak[1], abs(w)))
        x, y, yaw = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw + w * dt
    return ctl, (x, y, yaw), peak


def test_polar_coordinates():
    rho, alpha, beta = polar.polar(0, 0, 0, 1, 1, math.pi / 2)
    assert math.isclose(rho, math.sqrt(2)) and math.isclose(alpha, math.pi / 4) and math.isclose(beta, math.pi / 4)
    assert math.isclose(polar.wrap(3 * math.pi / 2), -math.pi / 2)


def test_saturate_keeps_the_ratio():
    v, w = polar.saturate(1.2, 1.0)
    assert math.isclose(v, 0.3) and math.isclose(w, 0.25)
    assert polar.saturate(0.1, 0.2) == (0.1, 0.2)


def test_stability_condition():
    assert polar.strong_stability(*polar.GAINS) and polar.strong_stability(*polar.HIGH_GAINS)
    assert not polar.strong_stability(0.5, 1.5, 0.5)            # k_beta must be negative


def test_reaches_goals_in_every_direction():
    for goal in [(2, 0, 0), (2, 1, math.pi / 2), (1, -1.5, -math.pi), (-1.5, 0.5, 0), (0, 1, math.pi)]:
        ctl, (x, y, yaw), (v_peak, w_peak) = simulate(goal)
        assert ctl.phase == 'done', goal
        assert math.hypot(x - goal[0], y - goal[1]) < 0.05 and abs(polar.wrap(yaw - goal[2])) < math.radians(3)
        assert v_peak <= polar.V_MAX + 1e-9 and w_peak <= polar.W_MAX + 1e-9


def test_goal_behind_is_reached_walking_backward():
    ctl, _, _ = simulate((-1.5, 0.0, 0.0))
    assert ctl.direction == -1
