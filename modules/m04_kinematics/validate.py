#!/usr/bin/env python3
"""Experiment for module 04: check this kinematics against MuJoCo, Pinocchio and finite differences, measure IK, plot the workspace.
usage: python validate.py [--n 1000] [--json results.json] [--plot workspace.png]"""
import argparse, json, pathlib, sys, time
import numpy as np
import mujoco, pinocchio as pin
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common import LEGS, require_model  # noqa: E402
from kinematics import Leg  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=1000); ap.add_argument('--json'); ap.add_argument('--plot'); a = ap.parse_args()
    xml = require_model(); m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)
    pm = pin.buildModelFromMJCF(xml); pd = pm.createData(); rng = np.random.default_rng(0); res = {}
    for leg in LEGS:
        L = Leg(m, leg); jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{leg}_{p}_joint') for p in ('hip', 'thigh', 'calf')]
        adr = [m.jnt_qposadr[j] for j in jid]; gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, leg)
        pj = pm.getJointId(f'{leg}_calf_joint'); fid = pm.addFrame(pin.Frame(f'{leg}_foot', pj, pin.SE3(np.eye(3), L.foot), pin.FrameType.OP_FRAME))
        pd = pm.createData()
        e_mj, e_pin, e_fd, e_jpin = [], [], [], []
        for _ in range(a.n):
            q = rng.uniform(L.lo, L.hi)
            f = L.fk(q)
            qpos = np.zeros(m.nq); qpos[3] = 1.0; qpos[[a_ for a_ in adr]] = q   # base at the origin, identity orientation
            d.qpos[:] = qpos; mujoco.mj_kinematics(m, d); e_mj.append(np.linalg.norm(d.geom_xpos[gid] - f))
            pq = pin.neutral(pm); pq[[pm.joints[pm.getJointId(f'{leg}_{p}_joint')].idx_q for p in ('hip', 'thigh', 'calf')]] = q
            pin.framesForwardKinematics(pm, pd, pq); e_pin.append(np.linalg.norm(pd.oMf[fid].translation - f))
            J = L.jacobian(q); h = 1e-6
            Jfd = np.stack([(L.fk(q + h * np.eye(3)[i]) - L.fk(q - h * np.eye(3)[i])) / (2 * h) for i in range(3)], 1)
            e_fd.append(np.abs(J - Jfd).max())
            Jp = pin.computeFrameJacobian(pm, pd, pq, fid, pin.LOCAL_WORLD_ALIGNED)[:3, [pm.joints[pm.getJointId(f'{leg}_{p}_joint')].idx_v for p in ('hip', 'thigh', 'calf')]]
            e_jpin.append(np.abs(J - Jp).max())
        # IK: targets are FK of random reachable configurations, solved from the nominal standing pose
        q_stand = np.clip(np.array([0.0, 0.9, -1.8]), L.lo, L.hi); ok = ok1 = 0; its = []; errs = []
        targets = [L.fk(rng.uniform(L.lo, L.hi)) for _ in range(a.n)]
        for target in targets:
            ok1 += L.ik(target, q_stand)[3]
        t0 = time.perf_counter()
        for target in targets:
            q, it, err, conv = L.ik_multistart(target, q_stand)
            ok += conv; its.append(it); errs.append(err)
        dt = (time.perf_counter() - t0) / a.n
        res[leg] = dict(fk_vs_mujoco_max_m=float(max(e_mj)), fk_vs_pinocchio_max_m=float(max(e_pin)), jac_vs_finite_diff_max=float(max(e_fd)),
                        jac_vs_pinocchio_max=float(max(e_jpin)), ik_single_seed_success=ok1 / a.n, ik_success=ok / a.n, ik_max_error_m=float(max(errs)) if ok == a.n else None, ik_median_iterations=float(np.median(its)),
                        ik_p99_iterations=float(np.percentile(its, 99)), ik_mean_ms=round(dt * 1e3, 3))
        print(f'{leg}: FK vs MuJoCo {max(e_mj):.1e} m, vs Pinocchio {max(e_pin):.1e} m | J vs FD {max(e_fd):.1e}, vs Pinocchio {max(e_jpin):.1e} | '
              f'IK single seed {100 * ok1 / a.n:.1f}%, multi-start {100 * ok / a.n:.1f}% converged, median {np.median(its):.0f} it, {dt * 1e3:.2f} ms')
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps({'n_per_leg': a.n, 'legs': res}, indent=2) + '\n')
    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        L = Leg(m, 'FL'); P = np.array([L.fk(rng.uniform(L.lo, L.hi)) for _ in range(6000)])
        fig, ax = plt.subplots(1, 2, figsize=(9, 4.2), dpi=150)
        for k, (i, j, lab) in enumerate(((0, 2, 'x forward [m]'), (1, 2, 'y left [m]'))):
            ax[k].scatter(P[:, i], P[:, j], s=1.5, alpha=0.35, color='#0b74a8', lw=0)
            ax[k].plot(*L.fk(np.array([0, 0.9, -1.8]))[[i, j]], 'o', color='#c98410', ms=6, label='standing foot')
            ax[k].set_xlabel(lab); ax[k].set_ylabel('z up [m]'); ax[k].set_aspect('equal'); ax[k].grid(alpha=0.25)
        ax[0].legend(loc='lower left', frameon=False); fig.suptitle('Go2 front-left foot workspace, base frame (6000 random joint configurations)', fontsize=10)
        fig.tight_layout(); fig.savefig(a.plot)


if __name__ == '__main__':
    main()
