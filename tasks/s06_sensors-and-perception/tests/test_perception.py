"""Fast checks of perception.py, no ROS:  python3 -m pytest -q tasks/s06_sensors-and-perception/tests"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import perception as pc


def test_rpy_and_rot_agree():
    r, p, y = 0.1, -0.2, 2.5
    R = pc.rot(r, p, y)
    w = math.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2                 # quaternion from R
    q = ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    assert np.allclose(pc.rpy(*q), (r, p, y))


def test_tilt_from_accel():
    R = pc.rot(0.02, -0.01, 0.7)
    assert np.allclose(pc.tilt_from_accel(R.T @ [0, 0, 9.8]), (0.02, -0.01))


def test_allan_white_noise_slope():
    sigma, dt = 2e-4, 0.01
    x = np.random.default_rng(0).normal(0, sigma, 200000)
    ad = pc.allan_deviation(x, dt, [0.1, 1.0])
    N = sigma * math.sqrt(dt)
    assert abs(ad[0] * math.sqrt(0.1) / N - 1) < 0.05 and abs(ad[1] * math.sqrt(1.0) / N - 1) < 0.1


def test_integrate_constant_rate():
    t = np.linspace(0, 2, 201)
    assert abs(pc.integrate(t, np.full_like(t, 0.5))[-1] - 1.0) < 1e-12


def test_floor_depth_and_backproject_round_trip():
    fx = fy = 467.7; cx, cy, h = 320.0, 180.0, 0.25
    v = np.arange(360)
    z = pc.floor_depth(v, cy, fy, h)
    assert np.isnan(z[:181]).all() and abs(z[200] - h * fy / 20) < 1e-12
    depth = np.tile(z[:, None], (1, 640))
    P = pc.backproject(depth, fx, fy, cx, cy)
    assert np.allclose(P[:, 1], h)                                      # every floor point lies h below the camera


def test_ransac_finds_floor_and_platform():
    rng = np.random.default_rng(1)
    floor = np.column_stack([rng.uniform(-1, 1, 3000), np.full(3000, 0.25), rng.uniform(0.7, 3, 3000)])
    top = np.column_stack([rng.uniform(-0.5, 0.5, 1500), np.full(1500, 0.13), rng.uniform(1.5, 2.5, 1500)])
    wall = np.column_stack([np.full(3000, 1.2), rng.uniform(-0.6, 0.25, 3000), rng.uniform(1.5, 4, 3000)])   # vertical, right
    P = np.vstack([floor, top, wall]) + rng.normal(0, 0.005, (7500, 3))
    r = pc.platform_from_points(P)
    assert r['detected'] and abs(r['height'] - 0.12) < 0.002 and r['angle_deg'] < 1.0
    assert r['top_mask'][3000:4500].mean() > 0.95 and r['top_mask'][:3000].mean() < 0.01
    assert r['top_mask'][4500:].sum() == 0                              # the wall slice at the same height is not the top
    g = pc.ground_frame(P[r['top_mask']], *r['floor'])
    assert abs(np.percentile(g[:, 0], 1) - 1.5) < 0.03 and abs(np.percentile(g[:, 2], 50) - 0.12) < 0.005
    assert abs(np.percentile(g[:, 1], 99) - 0.5) < 0.03                 # ground y is left = optical -x


def test_remove_floor_and_body():
    P = np.array([[2.0, 0, -0.21], [2.0, 0, 0.1], [0.3, 0.1, 0.0], [0.5, 0.5, -0.19]])
    keep = pc.remove_floor_and_body(P, np.array([0, 0, 1.0]), 0.215)
    assert keep.tolist() == [False, True, False, False]


def test_cluster_two_groups_and_noise():
    xy = np.vstack([np.column_stack([np.linspace(0, 1, 50), np.zeros(50)]),
                    np.column_stack([np.linspace(3, 4, 50), np.ones(50)]), [[10, 10]]])
    lab = pc.cluster(xy, 0.15, 5)
    assert set(lab[:50]) == {0} and set(lab[50:100]) == {1} and lab[100] == -1


def test_footprints_and_rays():
    box, cyl = ('box', 2.0, 0.0, 1.0, 2.0), ('cyl', 0.0, 3.0, 0.5)
    assert pc.footprint_dist(np.array([[1.5, 0.3], [0.5, 0.0]]), box).tolist() == [0.0, 1.0]
    assert abs(pc.ray_hit((0, 0), (1, 0), box) - 1.5) < 1e-12 and pc.ray_hit((0, 0), (-1, 0), box) == math.inf
    assert abs(pc.ray_hit((0, 0), (0, 1), cyl) - 2.5) < 1e-12
    hits = pc.visible((0, 0), [box, ('box', 4.0, 0.0, 1.0, 1.0), cyl])
    assert hits[0] > 0 and hits[1] == 0 and hits[2] > 0                   # the second box hides behind the first


def test_floor_depth_with_pitch_matches_rotated_ray():
    fy, cy, h, p = 467.7, 180.0, 0.25, -0.0068                          # 0.39 deg nose up
    v = 200.0
    z = pc.floor_depth(v, cy, fy, h, p)
    ray = pc.rot(0, p, 0) @ np.array([1.0, 0.0, -(v - cy) / fy])        # body frame: x forward, z up
    assert abs(z * -ray[2] - h) < 1e-12
