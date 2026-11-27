"""Module 00 advanced-level tests, optimisation part: Gauss-Newton and Levenberg-Marquardt on SE(2) registration and on the Go2
FL leg calibration, against the closed form, scipy.optimize.least_squares and MuJoCo. Thresholds are the ones in validate.ACCEPT."""
import pathlib, sys
import numpy as np
import pytest
from scipy.optimize import least_squares
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import validate as V  # noqa: E402
from lie import se2_plus  # noqa: E402
from optim import (cauchy_weights, gauss_newton, leg_fk, levenberg_marquardt, registration_closed_form,  # noqa: E402
                   registration_problem)


def thr(k):
    return V.ACCEPT[k][1]


@pytest.fixture(scope='module')
def scan():
    P, Q, _ = V.make_scan(np.random.default_rng(2)); return P, Q, registration_problem(P, Q), registration_closed_form(P, Q)


@pytest.fixture(scope='module')
def leg():
    import sim
    m, d = sim.load(); return V.FLLeg(m, d)


def test_registration_jacobian(scan):
    P, Q, f, _ = scan; T0 = np.array([0.1, 0.2, 0.3])
    assert np.abs(f(T0)[1] - V.fd(lambda h: f(se2_plus(T0, h))[0], np.zeros(3))).max() < thr('registration_jacobian_vs_fd')


def test_registration_matches_closed_form_and_scipy(scan):
    P, Q, f, cf = scan; x0 = np.array([0.0, 0.0, np.deg2rad(170.0)])
    assert V.pose_err(gauss_newton(f, x0, se2_plus)['x'], cf) < thr('registration_gn_vs_closed_form')
    assert V.pose_err(levenberg_marquardt(f, x0, se2_plus)['x'], cf) < thr('registration_lm_vs_closed_form')
    S = least_squares(lambda x: f(x)[0], x0, jac=lambda x: V._jac_vec(P, x), method='lm', xtol=1e-15, ftol=1e-15, gtol=1e-15)
    assert V.pose_err(S.x, cf) < thr('registration_scipy_vs_closed_form')


def test_registration_converges_from_every_heading(scan):
    """All starts converge (acceptance rows met). The <= 10 iteration row is missed at truth - 180 deg, where the cost has its
    maximum and the GN step is small (README, failure analysis); the test pins that this is the only slow start."""
    P, Q, f, cf = scan; iters = {}
    for th in np.arange(-180, 180, 5.0):                         # the validate.py grid, which contains -145 deg
        r = gauss_newton(f, np.array([0.0, 0.0, np.deg2rad(th)]), se2_plus); iters[th] = r['iters']
        assert V.pose_err(r['x'], cf) < 1e-9
        assert V.pose_err(levenberg_marquardt(f, np.array([0.0, 0.0, np.deg2rad(th)]), se2_plus)['x'], cf) < 1e-9
    slow = {th for th, k in iters.items() if k > thr('registration_gn_max_iters')}
    assert slow == {-145.0}


def test_robust_kernel_with_outliers():
    P, Q, _ = V.make_scan(np.random.default_rng(3), 0.2)
    ls = gauss_newton(registration_problem(P, Q), np.zeros(3), se2_plus)
    ca = gauss_newton(registration_problem(P, Q, cauchy_weights(5 * V.SIGMA_SCAN)), ls['x'], se2_plus, max_iter=200, tol_grad=1e-9)
    rot = lambda x: np.rad2deg(abs(V.wrap(x[2] - V.T_TRUE[2])))  # noqa: E731
    assert rot(ca['x']) < thr('registration_cauchy_20pct_outliers_rot_err_deg')
    assert np.linalg.norm(ca['x'][:2] - V.T_TRUE[:2]) < thr('registration_cauchy_20pct_outliers_trans_err_m')
    assert rot(ls['x']) > 1.0                                   # the failure case: plain least squares is pulled off by > 1 deg


def test_leg_fk_matches_mujoco(leg):
    q = leg.poses(np.random.default_rng(13), 300)
    assert np.abs(leg_fk(q, np.r_[0, 0, 0, V.P_TRUE[3:]], leg.hip, leg.dy, leg.fx)[0] - leg.measure(q)).max() < thr('calib_fk_vs_mujoco_m')


def test_calibration_matches_scipy_and_truth(leg):
    rng = np.random.default_rng(14); f = leg.problem(leg.poses(rng), rng)
    assert np.abs(f(V.P_NOMINAL)[1] - V.fd(lambda p: f(p)[0], V.P_NOMINAL)).max() < thr('calib_jacobian_vs_fd')
    G = gauss_newton(f, V.P_NOMINAL); L = levenberg_marquardt(f, V.P_NOMINAL)
    S = least_squares(lambda p: f(p)[0], V.P_NOMINAL, jac=lambda p: f(p)[1], method='lm', xtol=1e-15, ftol=1e-15, gtol=1e-15)
    assert np.abs(G['x'] - S.x).max() < thr('calib_gn_vs_scipy') and np.abs(L['x'] - S.x).max() < thr('calib_lm_vs_scipy')
    assert abs(G['cost'][-1] - S.cost) / S.cost < thr('calib_cost_rel_diff_vs_scipy')
    sd, _ = V.predicted_sd(f, S.x)
    assert np.max(np.abs(S.x - V.P_TRUE) / sd) < thr('calib_max_err_in_sd')


def test_calibration_mirrored_solution_has_the_same_cost(leg):
    """Negative link lengths with the thigh offset turned by pi fit the data exactly as well: the parameterisation is not unique."""
    rng = np.random.default_rng(15); f = leg.problem(leg.poses(rng), rng)
    ref = gauss_newton(f, V.P_NOMINAL)
    mir = gauss_newton(f, ref['x'] * [1, 1, 1, -1, -1] + [0, np.pi, 0, 0, 0])
    assert mir['x'][3] < 0 and mir['x'][4] < 0
    assert abs(mir['cost'][-1] - ref['cost'][-1]) < 1e-9 * ref['cost'][-1]


def test_lm_on_vectors_and_rejects_bad_steps():
    """Rosenbrock as residuals r = (10 (x2 - x1^2), 1 - x1) from (-1.2, 1): LM reaches (1, 1) and the accepted cost never rises."""
    fun = lambda x: (np.array([10 * (x[1] - x[0] ** 2), 1 - x[0]]), np.array([[-20 * x[0], 10.0], [-1.0, 0.0]]))  # noqa: E731
    L = levenberg_marquardt(fun, np.array([-1.2, 1.0]))
    assert np.abs(L['x'] - 1).max() < 1e-8 and all(np.diff(L['cost']) <= 0)


@pytest.fixture(scope='module')
def calibration_run():
    """The full calibration experiment of validate.py (same seeds, 200 NEES draws, 60 basin starts per magnitude, 30 draws per pose
    spread); about 5 s on one core."""
    import sim
    return V.calibration(*sim.load())[0]


def test_calibration_covariance_is_consistent(calibration_run):
    c = calibration_run; lo, hi = thr('calib_mean_nees')
    assert c['calib_nees_trials'] == 200 and lo <= c['calib_mean_nees'] <= hi
    assert c['calib_rms_over_predicted_sd_max'] < thr('calib_rms_over_predicted_sd_max')


def test_calibration_basin(calibration_run):
    c = calibration_run; b = c['calib_basin']
    assert b['mag_rad'][1] == 0.5 and b['trials'] == 60
    assert c['calib_gn_success_at_0p5rad'] >= thr('calib_gn_success_at_0p5rad')
    assert c['calib_lm_success_at_0p5rad'] >= thr('calib_lm_success_at_0p5rad')
    assert c['calib_lm_not_worse_than_gn'] >= thr('calib_lm_not_worse_than_gn')
    assert all(lm >= gn for gn, lm in zip(b['gn'], b['lm']))
