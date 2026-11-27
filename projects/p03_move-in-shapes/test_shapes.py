"""Tests for the ROS-free logic.  /usr/bin/python3 -m pytest -q projects/p03_move-in-shapes"""
import math
import shapes


def test_wrap():
    assert math.isclose(shapes.wrap(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(shapes.wrap(-0.1), -0.1)


def test_shapes_close_on_the_start():
    assert shapes.square_corners()[-1] == (0.0, 0.0)
    pts = shapes.figure8_points()
    assert len(pts) == 24
    assert all(math.hypot(*p) < 1e-9 for p in (pts[11], pts[23]))      # both lobes end at the start
    assert max(y for _, y in pts) > 0.89 and min(y for _, y in pts) < -0.89   # lobes 2r = 0.9 m tall, one each side


def test_schedule_timing():
    s = shapes.square_schedule(side=1.5, v=0.25, w=0.4, settle=1.5)
    assert shapes.command_at(s, 0.0) == (0.25, 0.0, 'side0')
    assert shapes.command_at(s, 6.5) == (0.0, 0.0, 'corner0')                # side lasts 1.5 / 0.25 = 6 s
    assert shapes.command_at(s, 7.6)[2] == 'turn0'
    total = sum(d for d, *_ in s)
    assert math.isclose(total, 4 * (6 + 1.5 + math.pi / 2 / 0.4 + 1.5))
    assert shapes.command_at(s, total + 0.01) is None
    v, w, _ = shapes.command_at(shapes.figure8_schedule(r=0.45, v=0.18), 1.0)
    assert math.isclose(v / w, 0.45)                                          # radius = v / w


def simulate(f, steps=4000, dt=0.05, x=0.0, y=0.0, yaw=0.0):
    """A perfect unicycle: the robot does exactly what it is told."""
    path, t = [], 0.0
    for _ in range(steps):
        cmd = f.step(t, x, y, yaw)
        if cmd is None:
            return path, (x, y, yaw)
        v, w, _ = cmd
        yaw += w * dt
        x, y = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        path.append((x, y))
        t += dt
    raise AssertionError('follower did not finish')


def test_follower_walks_the_square_on_a_perfect_robot():
    f = shapes.Follower(shapes.square_corners(), stop_each=True, final_yaw=0.0)
    path, (x, y, yaw) = simulate(f)
    assert math.hypot(x, y) < 0.05 and abs(shapes.wrap(yaw)) < math.radians(3)
    for cx, cy in shapes.square_corners():                                   # passes within 5 cm of every corner
        assert min(math.hypot(px - cx, py - cy) for px, py in path) < 0.05


def test_follower_corrects_a_start_offset():
    # start 0.2 m to the right of the first leg and 20 degrees off: the carrot pulls the robot back onto the line
    f = shapes.Follower([(2.0, 0.0)], stop_each=True)
    f.prev = (0.0, 0.0)
    path, (x, y, _) = simulate(f, y=-0.2, yaw=math.radians(-20))
    assert abs(x - 2.0) < 0.05 and abs(y) < 0.05


def test_follower_walks_the_figure8():
    f = shapes.Follower(shapes.figure8_points(), stop_each=False, v=0.18, lookahead=0.25, reach_tol=0.05)
    path, (x, y, _) = simulate(f)
    assert math.hypot(x, y) < 0.08
    assert max(p[1] for p in path) > 0.8 and min(p[1] for p in path) < -0.8
