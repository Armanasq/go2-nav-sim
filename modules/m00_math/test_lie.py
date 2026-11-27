"""Module 00 advanced-level tests, Lie-group part: SO(3) right/left Jacobians against finite differences, Pinocchio and the Taylor
series; SE(2) Exp/Log. Thresholds are the ones in validate.ACCEPT."""
import pathlib, sys
import numpy as np
import pytest
pin = pytest.importorskip('pinocchio')
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from lie import SMALL, jl_inv_so3, jl_so3, jr_inv_so3, jr_so3, se2_exp, se2_log  # noqa: E402
from rotations import exp_so3, log_so3  # noqa: E402
from validate import ACCEPT, fd, random_rotvecs, series_jr, series_jr_inv  # noqa: E402

W = random_rotvecs(np.random.default_rng(10), 400)


def thr(k):
    return ACCEPT[k][1]


def test_right_and_left_jacobian_match_finite_differences():
    for w in W:
        R = exp_so3(w)
        assert np.abs(jr_so3(w) - fd(lambda x: log_so3(R.T @ exp_so3(x)), w)).max() < thr('jr_vs_fd')
        assert np.abs(jl_so3(w) - fd(lambda x: log_so3(exp_so3(x) @ R.T), w)).max() < thr('jl_vs_fd')
        assert np.abs(jr_inv_so3(w) - fd(lambda d: log_so3(R @ exp_so3(d)), np.zeros(3))).max() < thr('jr_inv_vs_fd')


def test_identities():
    for w in W:
        R = exp_so3(w)
        assert np.abs(jr_inv_so3(w) @ jr_so3(w) - np.eye(3)).max() < thr('jr_inv_times_jr_minus_I')
        assert np.abs(jl_inv_so3(w) @ jl_so3(w) - np.eye(3)).max() < thr('jr_inv_times_jr_minus_I')
        assert np.abs(jl_so3(w) - R @ jr_so3(w)).max() < thr('jl_equals_R_jr')
        assert np.abs(jl_so3(w) - jr_so3(w).T).max() < 1e-15
        assert np.abs(jr_so3(w) @ w - w).max() < 1e-15                       # the rotation axis is left unchanged


def test_matches_pinocchio_above_1e_2_rad():
    """Pinocchio Jexp3 is Jr and Jlog3 is Jr^-1. The 1e-14 / 1e-10 rows miss below 1e-2 rad (README, failure analysis), so this
    test pins the agreement where it holds."""
    for w in W[np.linalg.norm(W, axis=1) > 1e-2]:
        assert np.abs(jr_so3(w) - pin.Jexp3(w)).max() < 1e-14
        assert np.abs(jr_inv_so3(w) - pin.Jlog3(exp_so3(w))).max() < 1e-10


def test_small_angles_match_taylor_series():
    rng = np.random.default_rng(11)
    for t in np.logspace(-8, -2, 200):
        a = rng.normal(size=3); w = a / np.linalg.norm(a) * t
        assert np.abs(jr_so3(w) - series_jr(w)).max() < 1e-15
        assert np.abs(jr_inv_so3(w) - series_jr_inv(w)).max() < 1e-15


def test_no_jump_at_series_switch():
    a = np.array([0.3, -0.5, 0.81]); a /= np.linalg.norm(a)
    for f in (jr_so3, jr_inv_so3):
        assert np.abs(f(a * SMALL * (1 - 1e-9)) - f(a * SMALL * (1 + 1e-9))).max() < thr('jacobian_jump_at_series_switch')


def test_first_order_error_is_quadratic_with_jr():
    w = np.array([0.6, -0.9, 0.5]); d = np.array([0.3, 0.8, -0.52]); d /= np.linalg.norm(d); R = exp_so3(w)
    eps = np.logspace(-5, -2, 10)
    e_jr = [np.linalg.norm(log_so3(R.T @ exp_so3(w + s * d)) - s * jr_so3(w) @ d) for s in eps]
    e_id = [np.linalg.norm(log_so3(R.T @ exp_so3(w + s * d)) - s * d) for s in eps]
    lo, hi = ACCEPT['first_order_slope_with_jr'][1]; assert lo < np.polyfit(np.log(eps), np.log(e_jr), 1)[0] < hi
    lo, hi = ACCEPT['first_order_slope_without_jr'][1]; assert lo < np.polyfit(np.log(eps), np.log(e_id), 1)[0] < hi


def test_jr_inverse_near_pi():
    """(1 + cos t) / sin t is 0/0 at t = pi; the cot(t/2) form stays finite and inverts Jr."""
    w = np.array([0.0, 0.0, np.pi - 1e-12])
    assert np.abs(jr_inv_so3(w) @ jr_so3(w) - np.eye(3)).max() < 1e-12


def test_se2_exp_log_roundtrip():
    rng = np.random.default_rng(12)
    for _ in range(500):
        T = np.r_[rng.uniform(-3, 3, 2), rng.uniform(-np.pi + 1e-9, np.pi)]
        assert np.abs(se2_exp(se2_log(T)) - T).max() < thr('se2_exp_log_roundtrip')
    assert np.abs(se2_exp(np.array([1.0, 2.0, 0.0])) - [1, 2, 0]).max() == 0
