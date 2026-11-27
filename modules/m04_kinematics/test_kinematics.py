"""Module 04 acceptance tests: first-principles kinematics must match MuJoCo and Pinocchio to machine precision."""
import pathlib, sys
import numpy as np
import pytest
mujoco = pytest.importorskip('mujoco'); pin = pytest.importorskip('pinocchio')
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1])); sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from common import GO2_XML, LEGS  # noqa: E402
from kinematics import Leg  # noqa: E402

pytestmark = pytest.mark.skipif(not GO2_XML.exists(), reason='run modules/fetch_go2.sh first')


@pytest.fixture(scope='module')
def model():
    return mujoco.MjModel.from_xml_path(str(GO2_XML))


@pytest.mark.parametrize('leg', LEGS)
def test_fk_matches_mujoco(model, leg):
    L = Leg(model, leg); d = mujoco.MjData(model); rng = np.random.default_rng(1)
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{p}_joint')] for p in ('hip', 'thigh', 'calf')]
    g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)
    for _ in range(200):
        q = rng.uniform(L.lo, L.hi); d.qpos[:] = 0; d.qpos[3] = 1; d.qpos[adr] = q
        mujoco.mj_kinematics(model, d)
        assert np.linalg.norm(d.geom_xpos[g] - L.fk(q)) < 1e-12


@pytest.mark.parametrize('leg', LEGS)
def test_jacobian_matches_finite_differences(model, leg):
    L = Leg(model, leg); rng = np.random.default_rng(2); h = 1e-6
    for _ in range(100):
        q = rng.uniform(L.lo, L.hi)
        Jfd = np.stack([(L.fk(q + h * e) - L.fk(q - h * e)) / (2 * h) for e in np.eye(3)], 1)
        assert np.abs(L.jacobian(q) - Jfd).max() < 1e-7


def test_fk_matches_pinocchio(model):
    pm = pin.buildModelFromMJCF(str(GO2_XML)); L = Leg(model, 'FL')
    fid = pm.addFrame(pin.Frame('FL_foot', pm.getJointId('FL_calf_joint'), pin.SE3(np.eye(3), L.foot), pin.FrameType.OP_FRAME)); pd = pm.createData()
    idx = [pm.joints[pm.getJointId(f'FL_{p}_joint')].idx_q for p in ('hip', 'thigh', 'calf')]; rng = np.random.default_rng(3)
    for _ in range(200):
        q = rng.uniform(L.lo, L.hi); pq = pin.neutral(pm); pq[idx] = q
        pin.framesForwardKinematics(pm, pd, pq)
        assert np.linalg.norm(pd.oMf[fid].translation - L.fk(q)) < 1e-12


def test_multistart_ik_reaches_99_percent_of_reachable_targets(model):
    L = Leg(model, 'FL'); rng = np.random.default_rng(4); stand = np.array([0.0, 0.9, -1.8]); n = 300
    ok = sum(L.ik_multistart(L.fk(rng.uniform(L.lo, L.hi)), stand)[3] for _ in range(n))
    assert ok / n >= 0.99


def test_ik_solution_is_exact_when_it_converges(model):
    L = Leg(model, 'RR'); target = L.fk(np.array([0.2, 0.6, -1.5]))
    q, _, err, ok = L.ik_multistart(target, np.array([0.0, 0.9, -1.8]))
    assert ok and np.linalg.norm(L.fk(q) - target) < 1e-6


@pytest.mark.parametrize('leg', LEGS)
def test_closed_form_ik_recovers_the_generating_angles(model, leg):
    L = Leg(model, leg); rng = np.random.default_rng(5)
    for _ in range(500):
        q = rng.uniform(L.lo, L.hi); S = L.ik_closed_form(L.fk(q))
        assert len(S) and np.abs(S - q).max(1).min() < 1e-9
        assert all(np.linalg.norm(L.fk(s) - L.fk(q)) < 1e-12 for s in S) and np.all((S >= L.lo - 1e-12) & (S <= L.hi + 1e-12))


def test_closed_form_ik_rejects_unreachable_targets(model):
    L = Leg(model, 'FL')
    assert len(L.ik_closed_form(L.hip + [0, L.d, -0.5])) == 0          # beyond l1 + l2 = 0.426 m
    assert len(L.ik_closed_form(L.hip + [0, 0.05, 0])) == 0             # inside the lateral offset circle, |(y, z)| < d


def test_closed_form_agrees_with_numerical_ik(model):
    L = Leg(model, 'FL'); rng = np.random.default_rng(6)
    for _ in range(100):
        t = L.fk(rng.uniform(L.lo, L.hi)); q, _, _, ok = L.ik_multistart(t, np.array([0.0, 0.9, -1.8]))
        if ok and abs(L.det_jacobian(q)) > 1e-4:                       # away from the s_z = 0 singularity (README, failure analysis)
            q = L.ik(t, q, tol=1e-13, max_iter=100)[0]
            assert np.abs(L.ik_closed_form(t) - q).max(1).min() < 1e-9


def test_worked_standing_example(model):
    L = Leg(model, 'FL'); f = L.fk([0, 0.9, -1.8])
    assert np.allclose(f, [0.192157, 0.142, -0.266373], atol=1e-6)
    assert np.allclose(L.ik_closed_form(f), [[0, 0.9, -1.8]], atol=1e-12)
    S = L.ik_closed_form(L.hip + [0, L.d, -0.225])                    # reviewer's case: foot 0.225 m under the thigh joint
    assert np.allclose(S, [[0, 1.014403, -2.038124]], atol=1e-6)


def test_det_jacobian_formula_and_knee_limit_bound(model):
    L = Leg(model, 'RL'); rng = np.random.default_rng(7)
    for _ in range(500):
        q = rng.uniform(L.lo, L.hi); assert abs(L.det_jacobian(q) - np.linalg.det(L.jacobian(q))) < 1e-15
    k = np.linspace(L.lo[2], L.hi[2], 10001)
    assert np.isclose(np.abs(np.sin(k + L.gamma)).min(), np.sin(-L.lo[2] - L.gamma), atol=1e-9)   # worst case: fully folded knee
    assert np.sin(-L.lo[2] - L.gamma) > 0.41


def test_statics_match_mujoco_standing_equilibrium():
    from analytic import statics
    s = statics(mujoco.MjModel.from_xml_path(str(GO2_XML.parent / 'scene.xml')))
    assert abs(s['total_mass_kg'] - 15.206408) < 1e-6
    assert abs(sum(r['contact_force_world_N'][2] for r in s['legs'].values()) - s['total_mass_kg'] * s['g']) < 1e-2
    assert all(r['max_abs_residual'] < 1e-4 for r in s['legs'].values())
    assert abs(s['legs']['FL']['contact_force_world_N'][2] - s['lever_rule_front_foot_N']) < 0.05
    assert all(abs(t) < lim for tau in s['trot_tau'].values() for t, lim in zip(tau, s['limits']))
