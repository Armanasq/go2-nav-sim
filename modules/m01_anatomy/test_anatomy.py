"""Acceptance tests for module 01, same thresholds as validate.py (ACCEPT). Run: .venv-modules/bin/python -m pytest -q modules/m01_anatomy"""
import json, pathlib, sys
import numpy as np
import pytest
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import importlib.util  # noqa: E402
# every module has a validate.py: load this one under its own name so pytest over all modules does not pick up another's
_spec = importlib.util.spec_from_file_location('m01_validate', HERE / 'validate.py')
V = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(V)      # imports sim first, which sets MUJOCO_GL
import anatomy as A  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
from common import GO2_XML  # noqa: E402


def thr(k):
    return V.ACCEPT[k][1]


@pytest.fixture(scope='module')
def models():
    return A.parse(), mujoco.MjModel.from_xml_path(str(GO2_XML)), pin.buildModelFromMJCF(str(GO2_XML))


def test_parse_matches_mujoco_and_pinocchio(models):
    res = V.fundamental(*models)
    assert res['count_mismatches_vs_mujoco_pinocchio'] == 0
    for k in ('parse_bodies_vs_mujoco', 'parse_joints_vs_mujoco', 'parse_actuators_vs_mujoco', 'total_mass_vs_mujoco_kg',
              'joint_limits_vs_pinocchio', 'armature_damping_vs_pinocchio'):
        assert res[k] < thr(k), k
    assert abs(res['total_mass_kg'] - 15.2064) < 1e-4


def test_dof_counts_and_selection_matrix(models):
    r = models[0]; dm = A.dof(r); B = A.selection(r)
    assert (dm['nq'], dm['nv'], dm['nu'], dm['unactuated']) == (19, 18, 12, 6)
    assert np.all(B[:6] == 0) and np.linalg.matrix_rank(B) == 12
    assert np.array_equal(B[6:], np.eye(12))                     # gear 1, actuator k on joint k


def test_kinematics_momentum_and_transmission(models):
    res = V.intermediate(*models, n=60)
    for k in ('fk_body_pose_vs_mujoco', 'fk_joint_anchor_axis_vs_mujoco', 'body_velocity_vs_mujoco', 'com_vs_pinocchio_m',
              'centroidal_momentum_vs_pinocchio', 'actuator_force_vs_mujoco', 'armature_in_mass_matrix'):
        assert res[k] < thr(k), k


def test_actuator_force_clips_to_ctrlrange(models):
    r = models[0]; u = np.full(12, 100.0)
    tau = A.actuator_force(r, u)
    assert np.all(tau[:6] == 0) and np.allclose(tau[6:], np.tile([23.7, 23.7, 45.43], 4))


def test_flight_momentum_rate_is_gravity_whatever_the_torques(models):
    res = V.underactuation(*models, n=40)
    assert res['qfrc_actuator_base_rows_max_abs'] == 0.0
    assert res['flight_momentum_rate_vs_Mg_fd'] < thr('flight_momentum_rate_vs_Mg_fd')
    assert res['flight_momentum_rate_vs_Mg_pinocchio'] < thr('flight_momentum_rate_vs_Mg_pinocchio')
    assert res['flight_torque_effect_on_base_ang_acc_median'] > 1.0        # the torques do turn the base


def test_stance_momentum_rate_equals_gravity_plus_contacts():
    res, X = V.stance_balance()
    assert res['stance_momentum_rate_vs_wrench'] < thr('stance_momentum_rate_vs_wrench')
    assert res['stance_error_without_contacts_min_N'] > 50.0              # leaving out the contacts is far off


def test_zero_momentum_loop_reverses():
    f, b = V.reorient(cycles=2), V.reorient(cycles=2, sign=-1.0)
    df, db = np.degrees(f[-1, 2] - f[0, 2]), np.degrees(b[-1, 2] - b[0, 2])
    assert abs(df) > 2.0 and max(f[:, 4].max(), b[:, 4].max()) < thr('reorient_max_ang_momentum')
    lo, hi = thr('reorient_reverse_ratio'); assert lo <= db / df <= hi


def test_camera_model_against_segmentation(models):
    if not V.can_render(models[1]):
        pytest.skip('offscreen rendering needs MUJOCO_GL=glfw and a live X display')
    views, m, d = V.render_views(models[0], lift=0.4)
    err, n, _ = V.projection_check(m, d, views)
    assert n > 10000 and max(err.values()) <= thr('camera_projection_bbox_px')


def test_results_json_passes():
    p = HERE / 'results.json'
    if not p.exists():
        pytest.skip('run validate.py first')
    checks = json.loads(p.read_text())['checks']
    assert set(checks) <= set(V.ACCEPT) and set(V.ACCEPT) - set(checks) <= set(V.GL_CHECKS)   # GL checks absent without a display
    assert all(c['passed'] for c in checks.values()), [k for k, c in checks.items() if not c['passed']]


def test_other_region_loop_is_point_reflection_within_knee_limits():
    t = np.linspace(0, 1, 101)
    a = np.array([V.loop()(s) for s in t]); b = np.array([V.loop(region=-1.0)(s) for s in t])
    q0 = V.sim.Q_STAND
    assert np.allclose(b - q0, -(a - q0))                                  # same area, same direction, other side of the stand pose
    knee = b[:, 2::3]; assert knee.min() >= -2.7227 and knee.max() <= -0.83776


# Direct recomputation of the thresholds that test_results_json_passes would otherwise only read from a stored results.json.

def test_stand_torque_within_budget(models):
    tb = V.torque_budget(models[0])
    assert tb['stand_torque_utilisation_max'] < thr('stand_torque_utilisation_max')


def test_flight_drift_euler_first_order_rk4_small():
    D, _ = V.drift(T=0.3)                       # 0.3 s instead of 0.9 s: ratio 2.00, RK4 1.9e-6 kg m^2/s at both horizons
    lo, hi = thr('flight_drift_order_euler'); assert lo <= D['flight_drift_order_euler'] <= hi
    assert D['flight_drift_rk4_ang_kgm2s'] < thr('flight_drift_rk4_ang_kgm2s')
    assert all(D[k] == 0 for k in D if k.endswith('floor_contacts'))


def test_reorientation_pitch_speed_and_region():
    """Same runs as validate.py (5 cycles at 1 Hz, at 0.5 Hz, over the other region), about 10 s of CPU."""
    pitch = lambda X: np.degrees(X[-1, 2] - X[0, 2])  # noqa: E731
    pf, ph, po = (pitch(X) for X in (V.reorient(), V.reorient(f=0.5), V.reorient(region=-1.0)))
    assert abs(pf) > thr('reorient_pitch_5_cycles_deg')
    lo, hi = thr('reorient_half_speed_ratio'); assert lo <= ph / pf <= hi
    assert abs(po / pf - 1) > thr('reorient_other_region_rel_diff')
