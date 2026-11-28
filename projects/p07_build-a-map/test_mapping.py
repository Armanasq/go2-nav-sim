"""Tests for route.py and mapcmp.py. No ROS needed: /usr/bin/python3 -m pytest -q projects/p07_build-a-map"""
import math, pathlib, sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / 'p06_avoid-obstacles')]
import arena2d  # noqa: E402   project 6: clearance() from the make_arena.py geometry
import mapcmp  # noqa: E402
import route  # noqa: E402

TRUTH = HERE.parents[1] / 'src/go2_nav_sim/maps/arena.yaml'


def test_route_keeps_clear_of_every_obstacle():
    pts = [route.A] + route.WAYPOINTS
    for p, q in zip(pts, pts[1:]):
        s = np.linspace(0, 1, 200)[:, None]
        seg = np.asarray(p) + s * (np.asarray(q) - np.asarray(p))
        # Go2 half width 0.16 m plus 0.3 m margin for drift and the gait's sway
        assert arena2d.clearance(seg[:, 0], seg[:, 1]).min() > 0.45, (p, q)


def test_route_ends_at_start_and_legs_alternate_turn_and_walk():
    assert route.WAYPOINTS[-1] == route.A
    g = route.legs(route.A)
    assert len(g) == 2 * len(route.WAYPOINTS)
    for i in range(0, len(g), 2):
        (turn, tol_t), (walk, tol_w) = g[i], g[i + 1]
        assert turn[2] == walk[2] and tol_t > tol_w                     # turn to the heading the walk keeps
        assert math.isclose(math.atan2(walk[1] - turn[1], walk[0] - turn[0]), walk[2])


def test_load_truth_map():
    cells, res, origin = mapcmp.load(TRUTH)
    assert cells.shape == (220, 260) and res == 0.05 and list(origin) == [-6.5, -5.5]
    x, y = (np.array([1.0, 0.0]) - origin) / res                         # centre of the tall brick box: occupied
    assert cells[int(y), int(x)] == mapcmp.OCC
    x, y = (np.array(route.A) - origin) / res                            # marker A: free
    assert cells[int(y), int(x)] == mapcmp.FREE


def fake_slam(truth, pose):
    """What a perfect mapper would save: the visible outline as occupied, reachable free space as free, the rest unknown,
    drawn in a map frame that sits at pose in the world."""
    outline, reach = mapcmp.visible_outline(truth, route.A)
    H, W = truth[0].shape
    frame = (np.zeros((H + 40, W + 40), np.int8), truth[1], np.array([-7.5, -6.5]))
    inv = lambda p: (p - pose[:2]) @ mapcmp.registration.rot(pose[2])   # world -> map frame
    cells = np.full(frame[0].shape, mapcmp.UNKNOWN, np.int8)
    for mask, val in ((reach, mapcmp.FREE), (outline, mapcmp.OCC)):
        c, r = np.floor((inv(mapcmp.centres(truth, mask)) - frame[2]) / truth[1]).astype(int).T
        cells[r, c] = val
    return cells, truth[1], frame[2]


def test_align_recovers_a_known_offset_and_scores_near_perfect():
    truth = mapcmp.load(TRUTH)
    pose = np.array([0.25, -0.15, math.radians(2.5)])
    slam = fake_slam(truth, pose)
    outline, _ = mapcmp.visible_outline(truth, route.A)
    found, _, _ = mapcmp.align(slam, mapcmp.centres(truth, outline), guess=[0.0, 0.0, 0.0])
    assert np.hypot(*(found[:2] - pose[:2])) < 0.02 and abs(found[2] - pose[2]) < math.radians(0.3)
    s = mapcmp.scores(mapcmp.resample(slam, found, truth), truth, route.A)
    assert s['precision_1cell'] > 0.95 and s['recall_1cell'] > 0.95 and s['dist_median_m'] < 0.03
    # without the fit the same map scores badly: the metric sees the 0.29 m / 2.5 deg offset
    s0 = mapcmp.scores(mapcmp.resample(slam, [0.0, 0.0, 0.0], truth), truth, route.A)
    assert s0['precision_1cell'] < 0.5
