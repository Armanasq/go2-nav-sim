#!/usr/bin/env python3
"""Experiment for module 00: max error of rotations.py against scipy and Pinocchio on random samples, and the worked numbers.
usage: python validate.py [--n 100000] [--json results.json]"""
import argparse, json, pathlib, sys
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from rotations import (R_LINK_OPTICAL, R_to_quat, R_to_rpy, exp_so3, log_so3, quat_to_R, rpy_rates, rpy_to_R,  # noqa: E402
                       se3, se3_inverse)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=100000); ap.add_argument('--json'); a = ap.parse_args()
    rng = np.random.default_rng(0); rots = Rotation.random(a.n, random_state=0); err = dict.fromkeys(
        ['quat_to_R_vs_scipy', 'R_to_quat_vs_scipy', 'rpy_to_R_vs_pinocchio', 'R_to_rpy_vs_pinocchio', 'exp_vs_pinocchio',
         'log_vs_pinocchio', 'log_vs_scipy', 'se3_inverse_vs_pinocchio', 'rpy_rates_vs_pinocchio'], 0.0)

    def upd(k, e):
        err[k] = max(err[k], float(np.abs(e).max()))
    for r in rots:
        x, y, z, w = r.as_quat(); R = r.as_matrix(); rv = r.as_rotvec(); t = rng.normal(size=3)
        upd('quat_to_R_vs_scipy', quat_to_R([w, x, y, z]) - R)
        upd('R_to_quat_vs_scipy', R_to_quat(R) - np.sign(w) * np.array([w, x, y, z]))
        rpy = pin.rpy.matrixToRpy(R)
        upd('rpy_to_R_vs_pinocchio', rpy_to_R(*rpy) - pin.rpy.rpyToMatrix(rpy))
        upd('R_to_rpy_vs_pinocchio', R_to_rpy(R) - rpy)
        upd('exp_vs_pinocchio', exp_so3(rv) - pin.exp3(rv))
        upd('log_vs_pinocchio', log_so3(R) - pin.log3(R)); upd('log_vs_scipy', log_so3(R) - rv)
        upd('se3_inverse_vs_pinocchio', se3_inverse(se3(R, t)) - pin.SE3(R, t).inverse().homogeneous)
        if abs(rpy[1]) < 1.5:                                       # rates are singular at |pitch| = pi/2
            rates = rng.normal(size=3)
            upd('rpy_rates_vs_pinocchio', rpy_rates(rpy, pin.rpy.computeRpyJacobian(rpy, pin.LOCAL) @ rates) - rates)
    for k, v in err.items():
        print(f'{k:28s} max error {v:.1e}')

    # Yaw from omega_z alone during a steady turn at constant roll phi and pitch theta: omega_z = cos(phi) cos(theta) psi_dot.
    # Checked by integrating omega_z sampled from the true rotation R(t) = Rz(psi(t)) Ry(theta) Rx(phi), with scipy's log.
    yaw = {}
    for roll_deg, pitch_deg in ((1.0, 0.0), (2.1, 0.0), (3.6, 0.0), (3.6, 2.0), (10.0, 0.0)):
        phi, th = np.deg2rad([roll_deg, pitch_deg]); dt = 1e-3; ts = np.arange(0, 2 * np.pi, dt)     # 1 rad/s, one full turn
        Rs = [rpy_to_R(phi, th, t) for t in ts]
        wz = [Rotation.from_matrix(R0.T @ R1).as_rotvec()[2] / dt for R0, R1 in zip(Rs[:-1], Rs[1:])]
        sim = np.rad2deg(ts[-1] - np.sum(wz) * dt)
        yaw[f'roll {roll_deg} pitch {pitch_deg}'] = dict(predicted_deg_per_turn=float(np.rad2deg(2 * np.pi * (1 - np.cos(phi) * np.cos(th)))),
                                                      integrated_deg_per_turn=float(sim), relative=float(1 - np.cos(phi) * np.cos(th)))
        print(f'yaw from omega_z, roll {roll_deg:4.1f} deg pitch {pitch_deg:3.1f} deg: '
              f'{yaw[f"roll {roll_deg} pitch {pitch_deg}"]["predicted_deg_per_turn"]:.3f} deg/turn predicted, {sim:.3f} integrated')
    h = 3 * np.sin(np.deg2rad(3.6))
    print('R_link_optical =', R_LINK_OPTICAL.round(12).tolist(), f'| point 3 m away, 3.6 deg pitch: {h:.4f} m height shift')
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(dict(n=a.n, max_error=err, yaw_from_omega_z=yaw, R_link_optical=R_LINK_OPTICAL.round(12).tolist(),
                                                        height_shift_3m_3p6deg_m=h), indent=2) + '\n')


if __name__ == '__main__':
    main()
