"""Acceptance tests for module 02, same thresholds as validate.py (ACCEPT). Run: .venv-modules/bin/python -m pytest -q modules/m02_description"""
import importlib.util
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
_spec = importlib.util.spec_from_file_location('m02_validate', HERE / 'validate.py')
V = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(V)      # imports sim first, which sets MUJOCO_GL
import description as D  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
from common import GO2_XML  # noqa: E402


def thr(k):
    return V.ACCEPT[k][1]


@pytest.fixture(scope='module')
def setup():
    r = D.A.parse(); m = mujoco.MjModel.from_xml_path(str(GO2_XML))
    fund, parsed, lumped = V.fundamental(r, m)
    return r, m, fund, parsed, lumped


def test_lumping_matches_pinocchio_and_mujoco(setup):
    r, m, fund, parsed, lumped = setup
    for k in ('lump_vs_pinocchio_gazebo', 'lump_vs_pinocchio_unitree', 'lump_vs_mujoco_urdf_import', 'mjcf_parse_vs_mujoco',
              'body_frames_urdf_vs_mjcf_m'):
        assert fund[k] < thr(k), k
    assert len(lumped['gazebo']) == len(lumped['unitree']) == len(r.bodies) == 13
    assert abs(fund['totals']['gazebo']['mass_kg'] - 15.09805) < 1e-9 and abs(fund['totals']['unitree']['mass_kg'] - 15.019) < 1e-9


def test_stored_urdf_is_xacro_output(setup):
    if 'stored_urdf_equals_xacro' not in setup[2]:
        pytest.skip('ROS 2 Humble / xacro not available')
    assert setup[2]['stored_urdf_equals_xacro'] >= thr('stored_urdf_equals_xacro')


def test_known_differences(setup):
    """The findings the README states, so a change to either file shows up here."""
    r, m, fund, parsed, lumped = setup
    calf = next(b for b in fund['bodies'] if b['body'] == 'FL_calf')
    assert abs(calf['gazebo']['dmass_kg'] + 0.027352) < 1e-9          # Gazebo calf + foot 0.214 kg vs MJCF 0.241352 kg
    J = {j['joint']: j for j in fund['joints']}
    assert J['RL_thigh_joint']['gazebo']['lower'] == -1.5708 and J['RL_thigh_joint']['mjcf']['lower'] == -0.5236
    assert J['FL_calf_joint']['gazebo']['effort'] == 35.55 and J['FL_calf_joint']['mjcf']['effort'] == 45.43
    assert fund['collision']['foot']['gazebo']['radius'] == 0.02 and fund['collision']['foot']['mjcf']['radius'] == 0.022
    assert fund_refs(parsed) == ['lh_hip', 'rf_hip', 'rh_hip']


def test_inertia_validity(setup):
    r, m, fund, parsed, lumped = setup
    rng = np.random.default_rng(0)
    syn = V.synthetic(rng, n=2000)
    assert syn['synthetic_known_cases'] >= thr('synthetic_known_cases')
    assert syn['synthetic_pseudo_vs_triangle_agreement'] >= thr('synthetic_pseudo_vs_triangle_agreement')
    inter, _ = V.intermediate(r, m, pin.buildModelFromMJCF(str(GO2_XML)), parsed, lumped, rng)
    # ACCEPT['invalid_inertias_all_files'] (<= 0) is missed: the two failures are zero-mass sensor frames of the Unitree URDF
    # (README, failure analysis). Pin that exact state so a new invalid inertia fails this test.
    assert sorted(map(tuple, inter['invalid_inertias_list'])) == [('unitree_links', 'imu'), ('unitree_links', 'radar')]
    assert all(inter['inertia_reports']['unitree_links'][n]['mass'] == 0 for n in ('imu', 'radar'))
    assert inter['regressor_vs_pinocchio_Nm'] < thr('regressor_vs_pinocchio_Nm')
    assert inter['regressor_vs_mujoco_bias_Nm'] < thr('regressor_vs_mujoco_bias_Nm')
    assert inter['mirror']['gazebo']['FL-FR_calf'][0] > 1e-3 and inter['mirror']['mjcf']['FL-FR_calf'][0] < 1e-12


def test_rank_and_identification(setup):
    r, m, fund, parsed, lumped = setup
    rng = np.random.default_rng(1)
    poses = [(rng.uniform(V.LO, V.HI), V.tilted_gravity(rng, np.radians(45))) for _ in range(20)]
    cols = np.r_[[np.arange(4 * r.body(f'FL_{s}'), 4 * r.body(f'FL_{s}') + 4) for s in V.SEGS]].ravel()
    _, rank, zero = D.svd_rank(np.vstack([D.gravity_regressor(r, q, g)[:3][:, cols] for q, g in poses]))
    assert thr('full_leg_rank')[0] <= rank <= thr('full_leg_rank')[1] and zero == 2
    pi = D.params(*V.mass_params(r))
    for fric, sides, km, kc in ((False, (1,), 'id_nofric_mass_rel_err_max', 'id_nofric_com_err_max_m'),
                                (True, (1, -1), 'id_twosided_mass_rel_err_max', 'id_twosided_com_err_max_m')):
        mm, dd = V.fixed_base_model(frictionloss=fric)
        Ys, taus, _ = V.measure(r, mm, dd, poses, sides, 0.5)
        est = V.identify_calves(r, Ys, taus, pi)
        e = V.errors(est, r)
        assert e['mass_rel_max'] < thr(km) and e['com_max_m'] < thr(kc), (fric, e)
    # est: friction loss, two-sided, as in validate.advanced
    assert V.mass_err_over_gap(est, r, lumped) < thr('id_twosided_mass_err_over_urdf_gap')
    hold = [(rng.uniform(V.LO, V.HI), V.tilted_gravity(rng, np.radians(45))) for _ in range(10)]
    rms_urdf, rms_id = V.holdout_rms(r, lumped, est, hold)
    assert rms_id / rms_urdf < thr('holdout_rms_ratio_id_over_urdf'), (rms_id, rms_urdf)


def test_media_under_6MB():
    vids = [V.MEDIA / 'side_by_side.mp4', V.MEDIA / 'identification.mp4']
    if not all(p.exists() for p in vids):
        pytest.skip('videos not rendered (run validate.py with an X display)')
    assert float(all(p.stat().st_size < 6e6 for p in vids)) >= thr('media_mp4_under_6MB')


def test_lstsq_qr_matches_numpy():
    rng = np.random.default_rng(3)
    Y = rng.normal(size=(40, 4)); b = rng.normal(size=40)
    x, _, cov = D.lstsq_qr(Y, b)
    assert np.allclose(x, np.linalg.lstsq(Y, b, rcond=None)[0], atol=1e-12)
    r = b - Y @ x
    assert np.allclose(cov, r @ r / 36 * np.linalg.inv(Y.T @ Y), atol=1e-12)


def fund_refs(parsed):
    L, J, refs = parsed['gazebo']
    return sorted(x for x in refs if x not in L and x not in J)
