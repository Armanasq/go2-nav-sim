"""Unit tests for limiter.py; no ROS needed:  python3 -m pytest -q projects/p02_teleop-and-safety"""
import os, sys
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from limiter import Limiter

DT = 0.05   # 20 Hz, the node's default rate


def run(lim, t0, t1):
    """Step the limiter from t0 to t1 and return the outputs."""
    out, n = [], round((t1 - t0) / DT)
    for i in range(n + 1):
        out.append(lim.step(t0 + i * DT))
    return out


def test_velocity_clip():
    lim = Limiter(a_max=(100.0, 100.0, 100.0))           # acceleration out of the way
    lim.command(0.0, (0.5, -1.0, 2.0))
    assert lim.target == (0.3, -0.25, 0.5)
    assert run(lim, 0.0, 0.5)[-1] == pytest.approx((0.3, -0.25, 0.5))


def test_acceleration_ramp():
    lim = Limiter(timeout=10.0)                           # a_max 0.5 m/s^2 forward, no dead-man here
    lim.step(0.0)
    lim.command(0.0, (0.3, 0.0, 0.0))
    out = run(lim, DT, 1.0)
    vx = [o[0] for o in out]
    assert vx[0] == pytest.approx(0.5 * DT)               # one step of a_max * dt
    assert max(b - a for a, b in zip(vx, vx[1:])) <= 0.5 * DT + 1e-12
    assert vx[-1] == pytest.approx(0.3)                   # 0.3 / 0.5 = 0.6 s to reach it


def test_deadman_stops_and_goes_quiet():
    lim = Limiter(timeout=0.6)
    lim.step(0.0)
    for i in range(1, 41):                                # 2 s of commands at 20 Hz
        lim.command(i * DT, (0.3, 0.0, 0.0)); lim.step(i * DT)
    assert lim.out[0] == pytest.approx(0.3)
    t_last = 40 * DT
    out = run(lim, t_last + DT, t_last + 3.0)
    vx = [None if o is None else o[0] for o in out]
    # still 0.3 up to the timeout, then down at 0.5 m/s^2 (0.6 s), then five zeros, then nothing
    assert vx[round(0.6 / DT) - 2] == pytest.approx(0.3)
    first_zero = vx.index(0.0)
    assert (first_zero + 1) * DT == pytest.approx(0.6 + 0.6, abs=2 * DT)
    assert vx[first_zero:first_zero + 5] == [0.0] * 5
    assert all(v is None for v in vx[first_zero + 5:])


def test_new_command_after_timeout_moves_again():
    lim = Limiter(a_max=(100.0, 100.0, 100.0), timeout=0.6)
    lim.command(0.0, (0.2, 0.0, 0.0)); run(lim, 0.0, 2.0)
    assert lim.out == (0.0, 0.0, 0.0)
    lim.command(2.0, (0.2, 0.0, 0.0))
    assert lim.step(2.05) == pytest.approx((0.2, 0.0, 0.0))


def test_nothing_sent_before_first_command():
    lim = Limiter()
    out = run(lim, 0.0, 1.0)
    assert out[:5] == [(0.0, 0.0, 0.0)] * 5 and all(o is None for o in out[5:])


def test_reverse_command_passes_through_zero_at_a_max():
    lim = Limiter(timeout=10.0)
    lim.command(0.0, (0.3, 0.0, 0.0)); run(lim, 0.0, 1.0)
    lim.command(1.0, (-0.3, 0.0, 0.0))
    out = run(lim, 1.0 + DT, 3.0)
    vx = [o[0] for o in out]
    assert vx[-1] == pytest.approx(-0.3)
    assert (vx.index(-0.3) + 1) * DT == pytest.approx(1.2, abs=DT)   # 0.6 m/s change at 0.5 m/s^2
