"""Every stage 8 plan on a perfect robot, and the scores. No simulator: python3 -m pytest -q tasks/s08_feedback-movement/tests"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import motion as m  # noqa: E402


def end(tid):
    return m.unicycle(m.plan(tid))


def test_to_start_frame():
    x, y, yaw = m.to_start(1.0, 2.0, math.pi, 1.0, 1.0, math.pi / 2)    # 1 m to the left of a robot facing +y
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9


def test_timed_point_exact_on_perfect_robot():
    r = end('8.01')[-1]
    assert m.dist(r, m.POINT) < 0.04            # each segment ends on a 0.05 s tick: up to 1 deg of extra turn


def test_feedback_point_line_square_waypoints():
    assert m.dist(end('8.02')[-1], m.POINT) < 0.03
    rows = end('8.05')
    assert max(abs(r['gy']) for r in rows) < 1e-6 and abs(rows[-1]['gx'] - m.LINE) < 0.03
    rows = end('8.06')
    for k, c in enumerate(m.shapes.square_corners()):
        assert m.dist(m.last_with(rows, f'corner{k}'), c) < 0.05
    rows = end('8.09')
    assert max(m.dist(m.last_with(rows, f'corner{k}'), p) for k, p in enumerate(m.WAYPOINTS)) < 0.05


def test_headings_and_poses():
    rows = end('8.03')
    for k, h in enumerate(m.HEADINGS):
        assert abs(math.degrees(m.wrap(m.last_with(rows, f'settle{k}')['gyaw'])) - h) < 2.5
    r = m.last_with(end('8.04'), 'settle0')
    assert m.dist(r, m.POSE) < 0.06 and abs(math.degrees(m.wrap(r['gyaw'] - m.POSE[2]))) < 3.5
    r = end('8.11')[-1]
    assert m.dist(r, (0, 0)) < 0.06 and abs(math.degrees(r['gyaw'])) < 3.5


def test_circle_and_recovery_scores():
    rows = end('8.07')
    e = m.radial(rows, (0.0, m.CIRCLE_R), m.CIRCLE_R)
    assert max(abs(v) for v in e) < 0.005 and m.dist(rows[-1], (0, 0)) < 0.02
    rows = [dict(t=t / 10, gy=y) for t, y in enumerate([0, 0, 0.2, 0.1, 0.06, 0.04, 0.0])]
    peak, rec = m.line_recovery(rows, 0.1)
    assert peak == 0.2 and abs(rec - 0.3) < 1e-9
