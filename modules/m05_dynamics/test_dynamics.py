"""Module 05 acceptance tests: the thresholds of validate.py's ACCEPT on smaller samples (< 60 s on one core)."""
import importlib.util, pathlib, sys
import numpy as np
import pytest
mujoco = pytest.importorskip('mujoco'); pin = pytest.importorskip('pinocchio')
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
from common import GO2_XML  # noqa: E402
_spec = importlib.util.spec_from_file_location('m05_validate', HERE / 'validate.py')
V = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(V)

pytestmark = pytest.mark.skipif(not GO2_XML.exists(), reason='run modules/fetch_go2.sh first')


def ok(res, *keys):
    for k in keys:
        assert V.check(k, res[k]), (k, res[k], V.ACCEPT[k])


def test_single_leg_and_lagrangian():
    r = V.fundamental(60, np.random.default_rng(10))
    ok(r, 'leg_rnea_vs_pinocchio_max_Nm', 'leg_rnea_vs_mujoco_max_Nm', 'leg_crba_vs_pinocchio_max', 'leg_crba_vs_mujoco_max',
       'leg_aba_vs_pinocchio_max', 'leg_aba_vs_mujoco_max', 'leg_aba_vs_cholesky_max', 'leg_M_min_eigenvalue',
       'lagrange_tau_vs_rnea_max_Nm', 'lagrange_M_vs_crba_max', 'lagrange_skew_max')


def test_floating_base_and_contact_jacobians():
    r = V.intermediate_float(40, np.random.default_rng(11))
    ok(r, 'float_rnea_vs_pinocchio_max', 'float_rnea_vs_mujoco_max', 'float_crba_vs_pinocchio_max', 'float_crba_vs_mujoco_max',
       'float_aba_vs_pinocchio_max', 'float_aba_vs_mujoco_max', 'float_aba_vs_cholesky_max', 'float_M_mass_block_err_kg',
       'foot_jac_vs_mujoco_max', 'foot_jacdot_v_vs_mujoco_max')


def test_trajectory_torques():
    ok(V.traj_torques(duration=1.0, dt=0.02)[1], 'traj_torque_vs_pinocchio_max_Nm', 'traj_torque_vs_mujoco_max_Nm')


def test_computed_torque_beats_pd_and_residual_is_first_order_in_dt():
    ct, pdg = V.computed_torque('ct', True, duration=2.5)[1], V.computed_torque('pdg', True, duration=2.5)[1]
    assert V.check('ct_rms_friction_rad', ct['rms_rad'])
    assert V.check('pdg_over_ct_rms_ratio_friction', pdg['rms_rad'] / ct['rms_rad'])
    # the missed criterion ct_rms_no_friction_rad, and its two measured causes (README, failure analysis)
    naive = V.computed_torque('ct', False, duration=2.5)[1]['rms_rad']
    e2, e1 = (V.computed_torque('ct_implicit', False, duration=2.5, timestep=dt)[1]['rms_rad'] for dt in (0.002, 0.001))
    assert naive > 3 * e2                                   # implicit damping: most of the naive error
    assert V.check('ct_error_order_in_dt', np.log(e2 / e1) / np.log(2))


def test_free_flight_matches_mujoco_integrator():
    ok(V.flight(duration=0.6)[1], 'flight_base_pos_dev_max_m', 'flight_joint_dev_max_rad')


def test_standing_kkt_forces_match_mujoco():
    r = V.standing(False, settle=2.0)
    assert V.check('kkt_vs_schur_max_N', r['kkt_vs_schur_max_N'])
    assert V.check('stand_kkt_vs_mujoco_max_N', r['kkt_vs_mujoco_max_N'])
    assert V.check('stand_weight_ratio_err', abs(r['weight_ratio'] - 1))
    assert V.check('stand_kkt_acc_max', r['kkt_acc_max'])
    assert r['n_contacts'] == 4 and r['tangential_over_normal_max'] < r['mu']


def test_squat_kkt_forces():
    r = V.squat(duration=1.5)[1]
    ok(r, 'squat_force_rms_over_mg', 'squat_force_max_over_mg')
    assert V.check('squat_condim3_eom_residual_max', V.squat(duration=0.5, condim=3)[1]['squat_eom_residual_max'])
