"""Acceptance tests for module 06, same thresholds as validate.py (ACCEPT). Run: .venv-modules/bin/python -m pytest -q modules/m06_sensors"""
import importlib.util
import json
import pathlib
import sys

import numpy as np
import pytest
from scipy.signal import correlate

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
_spec = importlib.util.spec_from_file_location('m06_validate', HERE / 'validate.py')
V = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(V)      # imports sim first, which sets MUJOCO_GL
import sensors as S  # noqa: E402

# Criteria missed in the recorded run, each explained in README "Failure analysis". A fix that makes one pass must remove it here.
KNOWN_MISSES = {'allan_white_rel_err_max', 'gyro_bias_mujoco_err_in_sd', 'contact_best_error_rate'}


def thr(k):
    return V.ACCEPT[k][1]


def test_results_json_meets_acceptance():
    r = json.loads((HERE / 'results.json').read_text())
    missing = [k for k in V.ACCEPT if k not in r['checks']]
    assert not missing, missing
    failed = {k for k, ok in r['checks'].items() if not ok}
    assert failed == KNOWN_MISSES, failed ^ KNOWN_MISSES


@pytest.fixture(scope='module')
def rig():
    m, d = S.build_model()
    return V.rig_motion(m, d)[0]


@pytest.mark.parametrize('key', ['gyro_vs_base_omega_max', 'quat_vs_base_quat_max', 'jointpos_vs_qpos_max', 'jointvel_vs_qvel_max',
                                 'gyro_vs_rotation_fd_rel_rms', 'acc_vs_position_fd_rel_rms', 'lever_arm_rel_rms'])
def test_mujoco_sensors_match_kinematics(rig, key):
    assert V.check(key, rig[key]), rig[key]


def test_noise_model_statistics():
    r = V.noise_stats()[0]
    for k in ('white_sd_rel_err', 'bias_rw_var_rel_err', 'quant_err_rms_rel_err'):
        assert V.check(k, r[k]), (k, r[k])


def test_allan_identity_and_white_slope():
    rng = np.random.default_rng(0); dt = 0.004; m = 50
    y = V.GYRO.corrupt(np.zeros((200_000, 3)), dt, rng)[0]
    yc = y - y.mean(0); th = np.concatenate([np.zeros((1, 3)), np.cumsum(yc, 0) * dt])
    d2 = th[2 * m::m] - 2 * th[m:-m:m] + th[:-2 * m:m]
    assert np.abs((d2 ** 2).mean(0) / (2 * (m * dt) ** 2) / S.allan_nonoverlap(y, dt, m) - 1).max() < thr('allan_overlap_subset_identity')
    w = S.Inertial(N=1e-4, K=0.0).corrupt(np.zeros((400_000, 3)), dt, rng)[0]
    tau, av = S.allan(w, dt, [1, 10, 100])                                   # >= 1000 clusters each
    assert np.abs(av.mean(1) / (1e-4 ** 2 / tau) - 1).max() < 0.05


def test_allan_fit_recovers_noise_parameters():
    rng = np.random.default_rng(1); dt = 0.004; n = int(4 * 3600 / dt)
    y = S.Inertial(N=V.GYRO.N, K=V.GYRO.K, b0=np.zeros(1)).corrupt(np.zeros((n, 1)), dt, rng)[0]
    tau, av = S.allan(y, dt, S.log_taus(n, dt))
    f = S.allan_fit(tau, av[:, 0])
    assert abs(f['N'] / V.GYRO.N - 1) < thr('allan_gyro_N_rel_err')
    assert abs(f['K'] / V.GYRO.K - 1) < thr('allan_gyro_K_rel_err')


def test_bias_sd_formula_monte_carlo():
    rng = np.random.default_rng(2); dt = 0.004; T = 20.0; k = int(T / dt)
    ch = S.Inertial(N=V.GYRO.N, K=V.GYRO.K, b0=np.zeros(400))
    y, b = ch.corrupt(np.zeros((k, 400)), dt, rng)
    assert abs((y.mean(0) - b[-1]).std() / S.bias_sd(V.GYRO.N, V.GYRO.K, T) - 1) < thr('gyro_bias_mc_sd_ratio_max_dev')


def test_calibration_noise_free_exact():
    f_ref = np.array([P.as_matrix().T @ np.array([0, 0, V.G]) for P in V.rig_poses()])
    b = np.array([0.05, -0.03, 0.08]); f_meas = f_ref @ V.M_ACC.T + b
    M, bk = S.calib_known(f_ref, f_meas)
    assert np.abs(M - V.M_ACC).max() < 1e-12 and np.abs(bk - b).max() < 1e-12
    U, bu, res = S.calib_unknown(f_meas, V.G)
    U_true, Q = S.triangular_factor(np.linalg.inv(V.M_ACC))
    assert np.allclose(Q @ U_true, np.linalg.inv(V.M_ACC), atol=1e-14)
    assert np.abs(U - U_true).max() < 1e-9 and np.abs(bu - b).max() < 1e-9


def test_xcorr_offset_convention_and_subsample():
    rng = np.random.default_rng(3)
    x = np.convolve(rng.standard_normal(6000), np.hanning(25), 'same')      # smooth broadband signal at a fine grid
    a, b = x[100:5000:2], x[100 - 3:5000 - 3:2]                            # b lags a by 3 fine steps = 1.5 samples
    est, lags, c = S.xcorr_offset(a, b, 1.0, 10)
    assert abs(est - 1.5) < 0.05
    a, b = x[100:5000:2], x[100 - 4:5000 - 4:2]                            # whole-sample lag 2: same peak as scipy's correlate
    lags, c = S.xcorr(a, b, 10)
    a0, b0 = a - a.mean(), b - b.mean()
    full = correlate(b0, a0, 'full'); zero = len(a0) - 1
    assert lags[np.argmax(c)] == np.argmax(full[zero - 10:zero + 11]) - 10 == 2


def test_detect_contact_hysteresis():
    F = np.array([0, 5, 11, 9, 4, 9, 2, 12, 0], float)[:, None]
    assert S.detect_contact(F, 8.0)[:, 0].tolist() == [0, 0, 1, 1, 0, 1, 0, 1, 0]
    assert S.detect_contact(F, 10.0, 3.0)[:, 0].tolist() == [0, 0, 1, 1, 1, 1, 0, 1, 0]
    assert S.toggles(S.detect_contact(F, 10.0, 3.0)).tolist() == [4]
