"""Fast checks for stage 2 (no rendering): the support, the first-principles dynamics terms, IK rejection, step metrics, and
that every results file passes the thresholds currently in acceptance.json.  run: python -m pytest -q tests"""
import json
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import lib  # noqa: E402
import mujoco  # noqa: E402


@pytest.fixture(scope='module')
def model():
    return lib.load_supported()


def test_trunk_is_fixed_and_feet_are_free(model):
    m, d = model
    assert m.nq == m.nv == 12 and mujoco.mjtJoint.mjJNT_FREE not in m.jnt_type
    lib.simulate(m, d, None, 0.5)                          # zero torque: the legs fall under gravity, the trunk cannot
    assert np.allclose(d.xpos[lib.sim.index(m).base], [0, 0, lib.BASE_Z])
    assert d.ncon == 0 and d.geom_xpos[lib.sim.index(m).feet, 2].min() > 0.2


def test_gravity_and_inertia_match_mujoco(model):
    m, d = model
    rng, M = np.random.default_rng(0), np.zeros((12, 12))
    Ls = lib.legs(m)
    for _ in range(20):
        d.qpos[:] = np.concatenate([rng.uniform(L.lo, L.hi) for L in Ls]); d.qvel[:] = 0
        mujoco.mj_forward(m, d); mujoco.mj_fullM(m, d, M)
        for i, L in enumerate(Ls):
            q = d.qpos[3 * i:3 * i + 3]
            assert np.allclose(L.gravity(q), d.qfrc_bias[3 * i:3 * i + 3], atol=1e-10)
            assert np.allclose([L.inertia(q, j) for j in range(3)], np.diag(M)[3 * i:3 * i + 3], atol=1e-12)


def test_ik_checked_reasons(model):
    m, _ = model
    L = lib.LegModel(m, 'FL')
    p0 = L.fk(lib.Q_HOLD[:3])
    cases = {'too_far': p0 + [0, 0, -.2], 'too_close': L.hip + [0, L.d, -.06], 'inside_offset': L.hip + [0, .03, -.03],
             'joint_limit': L.hip + [0, .37, -.05]}
    for reason, p in cases.items():
        assert lib.ik_checked(L, p) == (None, reason)
    rng = np.random.default_rng(1)
    for _ in range(100):
        target = L.fk(rng.uniform(L.lo, L.hi))
        q, why = lib.ik_checked(L, target)
        assert why is None and np.linalg.norm(L.fk(q) - target) < 1e-9


def test_step_metrics_against_closed_form():
    t = np.arange(0, 1, 1e-4)
    for zeta in (0.3, 0.546, 0.8):
        y, z, wn = lib.second_order_step(1.0, 2 * zeta * 50, 2500.0, t)
        assert abs(z - zeta) < 1e-12 and abs(wn - 50) < 1e-12
        os_pct = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta ** 2))
        assert abs(lib.step_metrics(t, y, 0, 1)['overshoot_pct'] - os_pct) < 0.01
        assert abs(lib.zeta_from_overshoot(os_pct) - zeta) < 1e-9


def test_pd_step_matches_prediction():
    import run
    r = run.step_trial(100.0, 0.0, T=0.4)[0]
    assert abs(r['measured']['overshoot_pct'] - r['predicted']['overshoot_pct']) < 5


def test_results_pass_current_thresholds():
    acc = json.loads((HERE / 'acceptance.json').read_text())
    ids = [k for k in acc if k[0].isdigit()]
    assert len(ids) == 14
    for tid in ids:
        res = json.loads((HERE / 'results' / f'{tid}.json').read_text())
        for key, rule in acc[tid]['checks'].items():
            (op, x), = rule.items()
            v = res['measured'][key]
            assert (v <= x if op == 'max' else v >= x if op == 'min' else v == x), (tid, key, v, rule)
        assert res['status'] == 'verified', tid
