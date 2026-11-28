import math

import numpy as np

import arena2d
import reactive
from reactive import P

INC = math.radians(1.0)


def scan(fill=math.inf):
    """360 beams from -180 deg in 1 deg steps; index 180 is straight ahead."""
    return np.full(360, fill), -math.pi


def wall_ahead(d, half_width=1.0):
    """Returns of a wall square to the robot, d ahead, from y = -half_width to +half_width."""
    r, a0 = scan()
    for i in range(360):
        b = a0 + i * INC
        if math.cos(b) > 0 and abs(d * math.tan(b)) <= half_width:
            r[i] = d / math.cos(b)
    return r, a0


def test_invalid_returns_are_dropped():
    r, a0 = scan(5.0)
    r[0], r[1], r[2], r[3] = math.inf, math.nan, 0.2, 30.0
    rr, aa = reactive.valid_returns(r, a0, INC, 0.4, 15.0)
    assert len(rr) == 356 and math.isclose(aa[0], a0 + 4 * INC)


def test_goal_in_robot_frame():
    gx, gy = reactive.goal_in_robot((1.0, 1.0, math.pi / 2), (1.0, 3.0))   # facing +y, goal 2 m further along +y
    assert math.isclose(gx, 2.0) and abs(gy) < 1e-9


def test_compose_inverse_is_identity():
    a = (1.0, -2.0, 0.7)
    assert np.allclose(reactive.compose(a, reactive.inverse(a)), (0, 0, 0), atol=1e-12)


def test_wall_blocks_straight_ahead_only():
    r, a0 = wall_ahead(1.0)
    rr, aa = reactive.valid_returns(r, a0, INC, 0.4, 15.0)
    t, free = reactive.free_directions(rr, aa, look=2.0, r_safe=0.55, n_dir=180)
    assert not free[t == 0.0][0]                               # into the wall
    assert free[np.argmin(abs(t - math.pi / 2))]               # along it, to the left
    assert free[np.argmin(abs(t + math.pi))]                   # back


def test_vfh_goes_round_the_wall_and_commits():
    r, a0 = wall_ahead(1.0)
    rr, aa = reactive.valid_returns(r, a0, INC, 0.4, 15.0)
    th = reactive.vfh_direction(rr, aa, (3.0, 0.0))            # goal straight behind the wall
    # the corridor must clear the wall end (1 m to the side) by r_safe: |th| >= atan((1 + 0.55) / 1)
    assert abs(th) >= math.atan2(1.55, 1.0) - math.radians(2)
    # with the last choice on the left, it stays left; on the right, it stays right
    assert reactive.vfh_direction(rr, aa, (3.0, 0.0), prev=1.0) > 0
    assert reactive.vfh_direction(rr, aa, (3.0, 0.0), prev=-1.0) < 0


def test_apf_symmetric_wall_points_straight_at_it():
    """The local minimum: goal behind a symmetric wall, push and pull on one line, nothing to turn the robot."""
    r, a0 = wall_ahead(0.9)
    rr, aa = reactive.valid_returns(r, a0, INC, 0.4, 15.0)
    assert abs(reactive.apf_direction(rr, aa, (3.0, 0.0), INC)) < 1e-6


def test_apf_obstacle_on_the_left_steers_right():
    r, a0 = scan()
    r[180 + 30] = 0.8                                          # one return 30 deg to the left
    rr, aa = reactive.valid_returns(r, a0, INC, 0.4, 15.0)
    assert reactive.apf_direction(rr, aa, (5.0, 0.0), INC) < 0


def test_command_brakes_and_turns():
    v, w = reactive.command(0.0, math.inf)
    assert (v, w) == (P['v_max'], 0.0)
    assert reactive.command(0.0, P['stop'])[0] == 0.0          # something at the stop distance: no forward speed
    assert reactive.command(None, 2.0) == (0.0, P['w_max'])    # nowhere to go: turn on the spot
    assert reactive.command(math.pi, 2.0) == (0.0, P['w_max'])  # behind: turn, do not walk


def test_progress_flags_a_stall_only():
    p = reactive.Progress(window=30.0, eps=0.25)
    assert not any(p.stuck(t, 10.0 - 0.1 * t) for t in range(60))      # closing 0.1 m/s: never stuck
    p = reactive.Progress(window=30.0, eps=0.25)
    assert [p.stuck(t, 5.0) for t in (0, 29, 31)] == [False, False, True]


def test_raycast_and_clearance_at_A():
    r = arena2d.raycast(*arena2d.arena.A, 0.0)
    ahead = r[np.argmin(abs(arena2d.ANGLE_MIN + arena2d.ANGLE_INC * np.arange(len(r))))]
    assert math.isclose(ahead, 5.9 + 4.5, abs_tol=0.01)       # east wall inner face at x = 5.9
    assert math.isclose(float(arena2d.clearance(*arena2d.arena.A)), 1.4, abs_tol=1e-9)   # south and west walls


def test_kinematic_runs():
    """vfh reaches B from A; apf stops in front of the long brick box with the goal straight behind it."""
    out, _, xy = arena2d.run((*arena2d.arena.A, 0.0), arena2d.arena.B, 'vfh')
    assert out == 'reached' and arena2d.clearance(*xy.T).min() > 0.5
    out, _, xy = arena2d.run((-1.0, 0.0, 0.0), (3.0, 0.0), 'apf')
    assert out == 'stuck' and xy[-1, 0] < 0.0                  # never got past x = 0, the brick face is at x = 0.6
