"""Fast checks for stage 3 (no rendering, no long simulation): the first-principles CoM, statics, support margin and trunk
clearance against MuJoCo or closed forms, the planner's rejections, and that every results file agrees with acceptance.json.
run: python -m pytest -q tests"""
import json
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import lib  # noqa: E402
import mujoco  # noqa: E402
import sim  # noqa: E402


@pytest.fixture(scope='module')
def placed():
    """Standing pose placed kinematically on the floor (no simulation), with a Posture anchored on it."""
    m, d = sim.load()
    sim.place_on_ground(m, d, lib.Q_STAND)
    return m, d, lib.Posture(m, d)


def test_com_matches_mujoco(placed):
    m, d, pose = placed
    d2, rng = mujoco.MjData(m), np.random.default_rng(0)
    for _ in range(20):
        d2.qpos[:] = d.qpos
        d2.qpos[3:7] = rng.normal(size=4); d2.qpos[3:7] /= np.linalg.norm(d2.qpos[3:7])
        d2.qpos[pose.idx.qpos] = np.concatenate([rng.uniform(L.lo, L.hi) for L in pose.L])
        mujoco.mj_forward(m, d2)
        R = d2.xmat[pose.idx.base].reshape(3, 3)
        c = d2.xpos[pose.idx.base] + R @ pose.com_body(d2.qpos[pose.idx.qpos])
        assert np.abs(c - d2.subtree_com[pose.idx.base]).max() < 1e-9
    assert abs(pose.M - mujoco.mj_getTotalmass(m)) < 1e-12


def test_force_distribution_statics():
    rng = np.random.default_rng(1)
    for _ in range(50):
        dxy = rng.uniform(-0.2, 0.2, (4, 2)) + [[0.19, 0.14], [0.19, -0.14], [-0.19, 0.14], [-0.19, -0.14]]
        c = rng.uniform(-0.03, 0.03, 2)
        f = lib.force_distribution(dxy - c, np.ones(4), 150.0)
        if (f > 0).all():                                      # no clipping: equilibrium holds exactly
            assert abs(f.sum() - 150) < 1e-9 and np.abs(f @ (dxy - c)).max() < 1e-9
    tri = np.array([[0.19, -0.14], [-0.19, 0.14], [-0.19, -0.14]])
    c = tri.mean(0)
    f3 = lib.force_distribution(np.vstack([[0.19, 0.14], tri]) - c, np.array([0, 1, 1, 1.]), 90.0)
    assert f3[0] == 0 and np.allclose(f3[1:], 30.0)           # centroid of a triangle: equal shares


def test_support_margin():
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    assert abs(lib.support_margin(sq, [0.5, 0.5]) - 0.5) < 1e-12
    assert abs(lib.support_margin(sq[[2, 0, 3, 1]], [0.2, 0.5]) - 0.2) < 1e-12     # order of the points does not matter
    assert abs(lib.support_margin(sq, [1.3, 0.5]) + 0.3) < 1e-12


def test_rpy_roundtrip():
    rng = np.random.default_rng(2)
    for _ in range(50):
        r = rng.uniform([-1, -1.4, -3], [1, 1.4, 3])
        R = lib.rpy_to_R(r)
        assert np.allclose(lib.R_to_rpy(R), r) and np.allclose(R @ R.T, np.eye(3))


def test_trunk_clearance_matches_mujoco(placed):
    m, _, _ = placed
    d2, rng = mujoco.MjData(m), np.random.default_rng(3)
    for _ in range(20):
        mujoco.mj_resetData(m, d2)
        rpy = rng.uniform(-0.3, 0.3, 3)
        d2.qpos[:3] = [0, 0, rng.uniform(0.15, 0.35)]
        mujoco.mju_mat2Quat(d2.qpos[3:7], lib.rpy_to_R(rpy).ravel())
        d2.qpos[7:] = lib.Q_STAND
        mujoco.mj_forward(m, d2)
        assert abs(lib.trunk_clearance(d2.qpos[:3], lib.rpy_to_R(rpy)) - lib.mujoco_clearance(m, d2)) < 1e-6


def test_planner_rejects_before_motion(placed):
    m, d, pose = placed
    qpos = d.qpos.copy()
    for z, reason in ((0.42, 'too_far'), (0.12, 'clearance')):
        with pytest.raises(lib.Rejected) as e:
            pose.set_body_pose(p=np.r_[pose.p[:2], z], T=0.5)
        assert e.value.reason == reason
    with pytest.raises(lib.Rejected) as e:
        pose.shift_com(pose.P[:, :2].mean(0) + [0.25, 0], T=0.5)
    assert e.value.reason == 'margin'
    with pytest.raises(lib.Rejected) as e:
        pose.lift_foot('FL', 0.05)                              # CoM near the FR-RL diagonal: no margin on three legs
    assert e.value.reason == 'margin'
    assert np.array_equal(d.qpos, qpos) and d.time == 0


def test_results_agree_with_acceptance():
    acc = json.loads((HERE / 'acceptance.json').read_text())
    ids = [k for k in acc if k[0].isdigit()]
    assert len(ids) == 12
    for tid in ids:
        res = json.loads((HERE / 'results' / f'{tid}.json').read_text())
        ok = True
        for key, rule in acc[tid]['checks'].items():
            (op, x), = rule.items()
            v = res['measured'][key]
            ok &= bool(v <= x if op == 'max' else v >= x if op == 'min' else v == x)
        assert res['status'] == ('verified' if ok else 'failed'), tid
