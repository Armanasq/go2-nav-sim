"""Registration and gyro integration on synthetic scans of the real arena geometry (make_arena.py)."""
import importlib.util, math, pathlib, sys
import numpy as np
import pytest

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from registration import GyroYaw, LineMap, register, rot, voxel_downsample, wrap  # noqa: E402

spec = importlib.util.spec_from_file_location('arena', SCRIPTS / 'make_arena.py'); assert spec is not None and spec.loader is not None; arena = importlib.util.module_from_spec(spec); spec.loader.exec_module(arena)


def outline_points(step=0.03):
    """Points on the 2D outline of every wall, box and cylinder."""
    pts = []
    for cx, cy, sx, sy, *_ in arena.WALLS + arena.BOXES:
        for t in np.arange(-0.5, 0.5, step / max(sx, sy)):
            pts += [(cx + t * sx, cy - sy / 2), (cx + t * sx, cy + sy / 2), (cx - sx / 2, cy + t * sy), (cx + sx / 2, cy + t * sy)]
    for cx, cy, r, *_ in arena.CYLS:
        for a in np.arange(0, 2 * math.pi, step / r):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(pts)


WORLD = voxel_downsample(outline_points(), 0.15)
LMAP = LineMap(WORLD)


def scan_from(pose, noise=0.01, rng=np.random.default_rng(0), max_range=12.0):
    """What the robot sees at pose: world points within range, expressed in the robot frame, with range noise."""
    d = WORLD - pose[:2]; near = np.hypot(*d.T) < max_range
    local = d[near] @ rot(pose[2])                              # world -> robot: R^T (p - t)
    r, b = np.hypot(*local.T), np.floor(np.degrees(np.arctan2(local[:, 1], local[:, 0])) / 0.4)
    order = np.lexsort((r, b)); keep = order[np.r_[True, np.diff(b[order]) != 0]]   # nearest point per bearing bin: occlusion
    local = local[keep]
    return local + rng.normal(0, noise, local.shape)


@pytest.mark.parametrize('pose', [(-4.4, -3.5, 0.0), (-1.0, 1.0, 1.2), (3.0, 2.5, -2.5)])
def test_translation_recovered_with_yaw_fixed(pose):
    true = np.array(pose); src = scan_from(true)
    guess = true + np.array([0.20, -0.15, 0.0])
    x, matches, rms, ok = register(src, LMAP, guess, fix_yaw=True)[:4]
    assert ok and matches > 100
    assert np.hypot(*(x[:2] - true[:2])) < 0.02


def test_full_se2_recovers_rotation_and_translation():
    true = np.array([-2.0, -2.0, 0.4]); src = scan_from(true)
    x, _, _, ok = register(src, LMAP, true + np.array([0.10, 0.10, 0.05]), fix_yaw=False, trans_prior=0.0)[:4]
    assert ok
    assert np.hypot(*(x[:2] - true[:2])) < 0.02 and abs(math.degrees(wrap(x[2] - true[2]))) < 0.5


def test_robust_to_twenty_percent_outliers():
    rng = np.random.default_rng(1); true = np.array([0.5, -3.0, 0.3]); src = scan_from(true, rng=rng)
    out = rng.uniform(-6, 6, (len(src) // 5, 2))                 # clutter that is not in the map
    x, _, _, ok = register(np.vstack([src, out]), LMAP, true + np.array([0.15, 0.1, 0.0]))[:4]
    assert ok and np.hypot(*(x[:2] - true[:2])) < 0.03


def test_reports_failure_when_scan_does_not_overlap_map():
    src = np.random.default_rng(2).uniform(40, 50, (300, 2))
    _, matches, _, ok = register(src, LMAP, np.zeros(3))[:4]
    assert not ok and matches < 25


def test_corridor_prior_stops_sliding_but_corrects_across():
    """Two parallel walls: along the corridor the scan carries no information. The prior keeps the prediction there,
    while the across-corridor offset is still corrected."""
    xs = np.arange(-15, 15, 0.03)
    walls = voxel_downsample(np.vstack([np.c_[xs, np.full_like(xs, -1.0)], np.c_[xs, np.full_like(xs, 1.0)]]), 0.15)
    cmap = LineMap(walls); near = walls[np.hypot(*walls.T) < 12]; src = near + np.random.default_rng(3).normal(0, 0.01, near.shape)
    x, _, _, ok = register(src, cmap, np.array([0.3, 0.2, 0.0]))[:4]
    assert ok
    assert abs(x[1]) < 0.02                      # across: corrected from 0.2 to ~0
    assert abs(x[0] - 0.3) < 0.02                # along: held at the prediction instead of drifting


def test_gyro_yaw_is_independent_of_arrival_order():
    g = GyroYaw(); ts = np.arange(0, 3, 0.01); rng = np.random.default_rng(5)
    for t in rng.permutation(ts):                               # 0.5 rad/s for 3 s, samples delivered in random order
        g.add(float(t), 0.5)
    assert g.dyaw(1.0, 2.0) == pytest.approx(0.5, abs=1e-3)
    assert g.dyaw(1.0, 9.0) is None                             # not covered by IMU data yet


def test_returns_a_covariance_that_shrinks_with_information():
    true = np.array([-1.0, 1.0, 1.2]); src = scan_from(true)
    *_, cov = register(src, LMAP, true + np.array([0.05, 0.0, 0.0]))
    assert cov.shape == (2, 2) and np.all(np.linalg.eigvalsh(cov) > 0) and np.sqrt(np.trace(cov)) < 0.01


def test_tilt_compensated_yaw_rate():
    """psi_dot = (sin(roll) w_y + cos(roll) w_z) / cos(pitch). At 3.6 deg roll with a pure yaw rotation of the world frame,
    integrating w_z alone underestimates yaw by 1 - cos(3.6 deg) = 0.2 %."""
    roll = math.radians(3.6); psi_dot = 0.5
    wy, wz = math.sin(roll) * psi_dot, math.cos(roll) * psi_dot   # body rates of a pure yaw rotation, robot rolled
    assert GyroYaw.yaw_rate(wy, wz, roll, 0.0) == pytest.approx(psi_dot, rel=1e-12)
    assert 1 - wz / psi_dot == pytest.approx(1 - math.cos(roll), rel=1e-9)
