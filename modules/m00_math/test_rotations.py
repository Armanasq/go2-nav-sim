"""Module 00 acceptance tests: the NumPy rotations must equal scipy.spatial.transform.Rotation and Pinocchio on random samples."""
import pathlib, sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
pin = pytest.importorskip('pinocchio')
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from rotations import (R_LINK_OPTICAL, R_to_quat, R_to_rpy, axis_angle_R, exp_so3, log_so3, quat_to_R, rpy_rates, rpy_to_R,  # noqa: E402
                       se2_compose, se2_inverse, se3, se3_inverse)

N = 2000
rng = np.random.default_rng(0)
QUATS = Rotation.random(N, random_state=1)


def test_rodrigues_matches_scipy():
    for _ in range(N):
        a = rng.normal(size=3); a /= np.linalg.norm(a); th = rng.uniform(-np.pi, np.pi)
        assert np.abs(axis_angle_R(a, th) - Rotation.from_rotvec(th * a).as_matrix()).max() < 1e-14


def test_quaternion_to_R_and_back():
    for r in QUATS:
        x, y, z, w = r.as_quat(); R = r.as_matrix()
        assert np.abs(quat_to_R([w, x, y, z]) - R).max() < 1e-14
        q = R_to_quat(R); ref = np.array([w, x, y, z]) * np.sign(w)
        assert np.abs(q - ref).max() < 1e-14
        assert np.abs(q - np.roll(pin.Quaternion(R).coeffs(), 1) * np.sign(pin.Quaternion(R).w)).max() < 1e-14


def test_R_to_quat_near_pi():
    for a in np.eye(3):                                  # 180 deg about each axis: trace = -1, the w branch would divide by 0
        R = axis_angle_R(a, np.pi); assert np.abs(quat_to_R(R_to_quat(R)) - R).max() < 1e-15


def test_rpy_matches_scipy_and_pinocchio():
    for _ in range(N):
        rpy = rng.uniform([-np.pi, -np.pi / 2 + 1e-3, -np.pi], [np.pi, np.pi / 2 - 1e-3, np.pi])
        R = rpy_to_R(*rpy)
        assert np.abs(R - Rotation.from_euler('ZYX', rpy[::-1]).as_matrix()).max() < 1e-14
        assert np.abs(R - pin.rpy.rpyToMatrix(*rpy)).max() < 1e-14
        assert np.abs(R_to_rpy(R) - rpy).max() < 1e-10
        assert np.abs(R_to_rpy(R) - pin.rpy.matrixToRpy(R)).max() < 1e-10


@pytest.mark.parametrize('pitch', [np.pi / 2, -np.pi / 2])
def test_gimbal_lock_reconstructs_R(pitch):
    for _ in range(200):
        R = rpy_to_R(rng.uniform(-np.pi, np.pi), pitch, rng.uniform(-np.pi, np.pi))
        rpy = R_to_rpy(R)
        assert rpy[2] == 0 and np.abs(rpy_to_R(*rpy) - R).max() < 1e-12


def test_exp_log_match_scipy_and_pinocchio():
    for r in QUATS:
        w = r.as_rotvec(); R = r.as_matrix()
        assert np.abs(exp_so3(w) - R).max() < 1e-14 and np.abs(exp_so3(w) - pin.exp3(w)).max() < 1e-14
        assert np.abs(log_so3(R) - w).max() < 1e-9 and np.abs(log_so3(R) - pin.log3(R)).max() < 1e-9
    for a in np.eye(3):                                  # theta = pi exactly and just below
        for th in (np.pi, np.pi - 1e-8):
            assert np.abs(exp_so3(log_so3(axis_angle_R(a, th))) - axis_angle_R(a, th)).max() < 1e-12
    assert np.abs(log_so3(exp_so3([1e-14, 0, 0])) - [1e-14, 0, 0]).max() < 1e-28


def test_se2_compose_inverse_match_matrix_form():
    def M(p):
        return se3(Rotation.from_euler('z', p[2]).as_matrix(), [p[0], p[1], 0])
    for _ in range(N):
        a, b = rng.uniform(-3, 3, 3), rng.uniform(-3, 3, 3)
        assert np.abs(M(se2_compose(a, b)) - M(a) @ M(b)).max() < 1e-13
        assert np.abs(M(se2_inverse(a)) - np.linalg.inv(M(a))).max() < 1e-13
        assert np.abs(se2_compose(a, se2_inverse(a))).max() < 1e-13


def test_se3_inverse_matches_pinocchio():
    for r in QUATS[:500]:
        t = rng.normal(size=3); T = se3(r.as_matrix(), t)
        assert np.abs(se3_inverse(T) - pin.SE3(r.as_matrix(), t).inverse().homogeneous).max() < 1e-14


def test_rpy_rates_match_pinocchio_and_finite_differences():
    """omega_body = J_local(rpy) rpy_dot (Pinocchio); also omega from log(R(t)^T R(t+h)) / h via scipy."""
    for _ in range(500):
        rpy = rng.uniform([-0.5, -1.2, -3], [0.5, 1.2, 3]); rates = rng.normal(size=3)
        omega = pin.rpy.computeRpyJacobian(rpy, pin.LOCAL) @ rates
        assert np.abs(rpy_rates(rpy, omega) - rates).max() < 1e-12
        h = 1e-6
        om_fd = Rotation.from_matrix(rpy_to_R(*(rpy - h * rates)).T @ rpy_to_R(*(rpy + h * rates))).as_rotvec() / (2 * h)
        assert np.abs(om_fd - omega).max() < 1e-7


def test_yaw_from_omega_z_alone_underestimates_by_cos_roll():
    """Steady turn at 3.6 deg roll, zero pitch: omega_z = cos(phi) psi_dot, so integrating omega_z loses 1 - cos(3.6 deg) = 0.197%, 0.710 deg per turn."""
    phi = np.deg2rad(3.6); omega = pin.rpy.computeRpyJacobian(np.array([phi, 0, 0]), pin.LOCAL) @ [0, 0, 1.0]
    assert np.isclose(omega[2], np.cos(phi)) and np.isclose(rpy_rates([phi, 0, 0], omega)[2], 1.0)
    assert np.isclose(np.rad2deg(2 * np.pi * (1 - omega[2])), 0.7104, atol=1e-4)


def test_optical_frame_axes():
    R = R_LINK_OPTICAL                                   # columns: optical axes expressed in the link frame
    assert np.allclose(R, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-15)
    assert np.allclose(R, pin.rpy.rpyToMatrix(-np.pi / 2, 0, -np.pi / 2), atol=1e-15)
    assert np.allclose(R[:, 2], [1, 0, 0]) and np.allclose(R[:, 0], [0, -1, 0]) and np.allclose(R[:, 1], [0, 0, -1])


def test_tilted_point_height_shift():
    assert np.isclose(3 * np.sin(np.deg2rad(3.6)), 0.1884, atol=1e-4)
    p = rpy_to_R(0, -np.deg2rad(3.6), 0) @ [3, 0, 0]   # z up, y left: negative pitch is nose up
    assert np.isclose(p[2], 3 * np.sin(np.deg2rad(3.6)))
