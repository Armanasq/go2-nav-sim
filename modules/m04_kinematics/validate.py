#!/usr/bin/env python3
"""Experiment for module 04, all three levels.
  fundamental: FK and Jacobian (kinematics.py) against MuJoCo, Pinocchio and finite differences; numerical IK success.
  intermediate: closed-form IK, Jacobian determinant and statics (analytic.py), statics against a MuJoCo standing equilibrium.
  advanced: differential IK as a box-constrained QP (diffik.py) against scipy.optimize.lsq_linear; tracking a foot path through
      the outer workspace boundary and one through the inner, singular boundary, against unconstrained damped least squares;
      a figure-eight tracked in MuJoCo.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--n 1000] [--no-media]"""
import argparse, json, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
from scipy.optimize import lsq_linear, minimize  # noqa: E402
from common import LEGS, SCENE_XML, require_model  # noqa: E402
from kinematics import Leg  # noqa: E402
import analytic  # noqa: E402
from diffik import QD_MAX, box, distance_to_workspace, dls_step, qp_active_set, qp_step, reach_radius, step_bounds, track  # noqa: E402

C = fs.C

FIG, MEDIA = HERE / 'figures', HERE / 'media'
STAND = np.array([0.0, 0.9, -1.8])
DT, LAM = 0.002, 0.01                       # control period [s] (= MuJoCo step), damping [m/rad] shared by DLS and QP
ADAPT = (0.02, 0.05)                        # (eps, lambda_max) [m/rad] of the adaptive damping; from a 3-setting sweep, README

# Acceptance thresholds, fixed before the full run (MODULE_SPEC.md). key: (op, threshold)
ACCEPT = {
    # fundamental (max over the four legs)
    'fk_vs_mujoco_max_m': ('<', 1e-12), 'fk_vs_pinocchio_max_m': ('<', 1e-12), 'jac_vs_finite_diff_max': ('<', 1e-7),
    'jac_vs_pinocchio_max': ('<', 1e-12), 'ik_multistart_success_min': ('>=', 0.99),
    # intermediate
    'closed_form_success_min': ('>=', 1.0), 'closed_form_max_q_error_rad': ('<', 1e-9),
    'closed_form_max_fk_residual_m': ('<', 1e-12), 'det_formula_vs_numpy_max': ('<', 1e-15),
    'statics_max_abs_residual_Nm': ('<', 1e-4),
    # advanced
    'qp_vs_lsq_linear_max': ('<', 1e-9), 'qp_kkt_residual_max': ('<', 1e-9), 'qp_max_iterations': ('<=', 10),
    'qp_ticks_vs_lsq_linear_max': ('<', 1e-9), 'workspace_distance_vs_scipy_max_m': ('<', 1e-6),
    'boundary_qp_limit_violation_rad': ('<', 1e-12), 'boundary_qp_rms_error_reachable_m': ('<', 1e-4),
    'boundary_qp_excess_error_max_m': ('<', 1e-3),
    'singular_qp_speed_over_limit_max': ('<=', 1.0 + 1e-9), 'singular_qp_rms_error_reachable_m': ('<', 1e-3),
    'singular_qp_excess_error_max_m': ('<', 5e-3),
    'fig8_ik_max_error_m': ('<', 1e-4), 'fig8_mujoco_rms_error_m': ('<', 5e-3), 'fig8_mujoco_max_error_m': ('<', 1e-2),
    'media_mp4_under_6MB': ('>=', 1.0),
}


def check(key, val):
    op, thr = ACCEPT[key]
    return bool({'<': val < thr, '<=': val <= thr, '>=': val >= thr}[op])


# ---- fundamental -------------------------------------------------------------------------------------------------------------

def fundamental(n):
    xml = require_model(); m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)
    pm = pin.buildModelFromMJCF(xml); rng = np.random.default_rng(0); res = {}
    for leg in LEGS:
        L = Leg(m, leg); jn = [f'{leg}_{p}_joint' for p in ('hip', 'thigh', 'calf')]
        adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in jn]
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, leg)
        fid = pm.addFrame(pin.Frame(f'{leg}_foot', pm.getJointId(jn[2]), pin.SE3(np.eye(3), L.foot), pin.FrameType.OP_FRAME))
        pd = pm.createData(); iq = [pm.joints[pm.getJointId(j)].idx_q for j in jn]; iv = [pm.joints[pm.getJointId(j)].idx_v for j in jn]
        e_mj, e_pin, e_fd, e_jpin = [], [], [], []
        for _ in range(n):
            q = rng.uniform(L.lo, L.hi); f = L.fk(q)
            d.qpos[:] = 0; d.qpos[3] = 1; d.qpos[adr] = q                  # base at the origin, identity orientation
            mujoco.mj_kinematics(m, d); e_mj.append(np.linalg.norm(d.geom_xpos[gid] - f))
            pq = pin.neutral(pm); pq[iq] = q
            pin.framesForwardKinematics(pm, pd, pq); e_pin.append(np.linalg.norm(pd.oMf[fid].translation - f))
            J = L.jacobian(q); h = 1e-6
            Jfd = np.stack([(L.fk(q + h * u) - L.fk(q - h * u)) / (2 * h) for u in np.eye(3)], 1)
            e_fd.append(np.abs(J - Jfd).max())
            e_jpin.append(np.abs(J - pin.computeFrameJacobian(pm, pd, pq, fid, pin.LOCAL_WORLD_ALIGNED)[:3, iv]).max())
        targets = [L.fk(rng.uniform(L.lo, L.hi)) for _ in range(n)]       # IK targets: FK of random in-limit configurations
        ok1 = sum(L.ik(t, STAND)[3] for t in targets)
        t0 = time.perf_counter(); sol = [L.ik_multistart(t, STAND) for t in targets]; dt = (time.perf_counter() - t0) / n
        its = [s[1] for s in sol]
        res[leg] = dict(fk_vs_mujoco_max_m=max(e_mj), fk_vs_pinocchio_max_m=max(e_pin), jac_vs_finite_diff_max=max(e_fd),
                        jac_vs_pinocchio_max=max(e_jpin), ik_single_seed_success=ok1 / n, ik_success=sum(s[3] for s in sol) / n,
                        ik_median_iterations=float(np.median(its)), ik_p99_iterations=float(np.percentile(its, 99)), ik_mean_ms=dt * 1e3)
        print(f'{leg}: FK vs MuJoCo {max(e_mj):.1e} m, vs Pinocchio {max(e_pin):.1e} m | J vs FD {max(e_fd):.1e}, vs Pinocchio '
              f'{max(e_jpin):.1e} | IK single seed {100 * ok1 / n:.1f}%, multi-start {100 * res[leg]["ik_success"]:.1f}%, {dt * 1e3:.2f} ms')
    return res


def intermediate(n):
    m = mujoco.MjModel.from_xml_path(require_model()); rng = np.random.default_rng(0)
    return dict(ik={leg: analytic.ik_comparison(Leg(m, leg), n, rng) for leg in LEGS}, worked_FL=analytic.worked_example(Leg(m, 'FL')),
                singularity=analytic.singularity(Leg(m, 'FL'), rng), statics=analytic.statics(mujoco.MjModel.from_xml_path(str(SCENE_XML))))


# ---- advanced: QP solver checks -------------------------------------------------------------------------------------------

def qp_checks(n=2000):
    """Random box QPs of the differential-IK shape: own active set vs scipy's bounded least squares (BVLS), and the KKT conditions."""
    rng = np.random.default_rng(10); dx, kkt, its = 0.0, 0.0, 0
    for _ in range(n):
        J = rng.normal(size=(3, 3)) * 0.2; e = rng.normal(size=3) * 0.01
        lo, hi = -rng.uniform(0, 0.03, 3), rng.uniform(0, 0.03, 3)
        H, g = 2 * (J.T @ J + LAM ** 2 * np.eye(3)), -2 * J.T @ e; A, b = box(lo, hi)
        x, y, it = qp_active_set(H, g, A, b, np.zeros(3)); its = max(its, it)
        r = lsq_linear(np.vstack([J, LAM * np.eye(3)]), np.concatenate([e, np.zeros(3)]), bounds=(lo, hi), method='bvls', tol=1e-15)
        dx = max(dx, np.abs(x - r.x).max())
        kkt = max(kkt, np.abs(H @ x + g + A.T @ y).max(), -y.min(), np.abs(y * (A @ x - b)).max(), (A @ x - b).max())
    return dict(n=n, qp_vs_lsq_linear_max=dx, qp_kkt_residual_max=kkt, qp_max_iterations_random=its)


def ticks_vs_lsq_linear(L, run, p_ref, q0):
    """Re-solve every tick of a QP run with scipy and compare the step."""
    q_prev = np.vstack([q0, run['q'][:-1]]); worst = 0.0
    for q, qn, p in zip(q_prev, run['q'], p_ref):
        J = L.jacobian(q); e = p - L.fk(q); lo, hi = step_bounds(L, q, DT)
        r = lsq_linear(np.vstack([J, LAM * np.eye(3)]), np.concatenate([e, np.zeros(3)]), bounds=(lo, hi), method='bvls', tol=1e-15)
        worst = max(worst, np.abs((qn - q) - r.x).max())
    return worst


def distance_vs_scipy(L, pts, rng):
    """Distance to the reachable set by multi-start L-BFGS-B over the joint box, against the closed-form distance."""
    worst = 0.0
    for p in pts:
        f = lambda q: np.sum((L.fk(q) - p) ** 2)                                     # noqa: E731
        jac = lambda q: 2 * L.jacobian(q).T @ (L.fk(q) - p)                          # noqa: E731
        best = min(minimize(f, rng.uniform(L.lo, L.hi), jac=jac, method='L-BFGS-B', bounds=list(zip(L.lo, L.hi)),
                            options=dict(ftol=1e-20, gtol=1e-14, maxiter=2000)).fun for _ in range(12))
        worst = max(worst, abs(np.sqrt(best) - distance_to_workspace(L, p)))
    return worst


# ---- advanced: trajectories ------------------------------------------------------------------------------------------------

def traj_boundary(L, T=2.0, R=0.075, cycles=2):
    """Circle of radius R in the leg's sagittal plane, top point at the standing foot. Its lower part leaves the workspace."""
    t = np.arange(1, int(cycles * T / DT) + 1) * DT; w = 2 * np.pi / T; p0 = L.fk(STAND)
    return t, p0 + np.stack([R * np.sin(w * t), 0 * t, -R * (1 - np.cos(w * t))], 1), STAND.copy()


def traj_singular(L, T=2.0, x=0.20, z=-0.01, cycles=2):
    """Lateral oscillation 0.20 m ahead of the hip that dips 2 cm into the cylinder |(y, z)| < d around the abduction axis, the
    inner workspace boundary, where s_z = 0 and det J = 0. Starts on the foot-below-hip branch."""
    t = np.arange(1, int(cycles * T / DT) + 1) * DT; w = 2 * np.pi / T; s = np.sign(L.d)
    p = L.hip + np.stack([x + 0 * t, s * (abs(L.d) + 0.01 + 0.03 * np.cos(w * t)), z + 0 * t], 1)
    q0 = L.ik_closed_form(L.hip + [x, s * (abs(L.d) + 0.04), z])[0]
    return t, p, q0


METHODS = {'pinv': dict(method='pinv'), 'dls': dict(method='dls', lam=LAM), 'dls_l05': dict(method='dls', lam=0.05),
           'dls_clip': dict(method='dls_clip', lam=LAM), 'qp': dict(method='qp', lam=LAM),
           'qp_adaptive': dict(method='qp', lam=LAM, adapt=ADAPT)}


def evaluate(L, t, p_ref, q0):
    dist = distance_to_workspace(L, p_ref); reach = dist == 0; out, runs = {}, {}
    for name, kw in METHODS.items():
        t0 = time.perf_counter(); r = track(L, p_ref, q0, DT, **kw); el = (time.perf_counter() - t0) / len(t)
        err = np.linalg.norm(r['p'] - p_ref, axis=1)
        qd = np.diff(np.vstack([q0, r['q']]), axis=0) / DT
        viol = float(max(np.maximum(r['q'] - L.hi, 0).max(), np.maximum(L.lo - r['q'], 0).max()))
        sig = np.array([np.linalg.svd(L.jacobian(q), compute_uv=False)[-1] for q in r['q']])
        r.update(err=err, qd=qd, sigma_min=sig); runs[name] = r
        out[name] = dict(rms_error_reachable_m=float(np.sqrt(np.mean(err[reach] ** 2))), max_error_reachable_m=float(err[reach].max()),
                         excess_error_max_m=float((err - dist).max()), excess_error_mean_m=float((err - dist).mean()),
                         limit_violation_rad=viol, max_speed_rad_s=np.abs(qd).max(0).tolist(),
                         speed_over_limit_max=float((np.abs(qd) / QD_MAX).max()), sigma_min_min=float(sig.min()),
                         mean_max_speed_unreachable_rad_s=float(np.abs(qd[~reach]).max(1).mean()) if (~reach).any() else 0.0,
                         qp_iterations_max=int(r['qp_iters'].max()), us_per_tick=el * 1e6)
    return dict(n_ticks=len(t), dt=DT, unreachable_fraction=float(1 - reach.mean()), max_distance_to_workspace_m=float(dist.max()),
                methods=out), runs, dist


# ---- advanced: figure-eight in MuJoCo --------------------------------------------------------------------------------------

def fixed_base_model(path, base_z=0.55):
    """Go2 with the free joint removed (base welded at base_z), the reference path drawn as small amber spheres, and a mocap
    sphere for the current target."""
    s = mujoco.MjSpec.from_file(str(SCENE_XML)); b = s.body('base')
    for j in list(b.joints):
        s.delete(j)
    for k in list(s.keys):
        s.delete(k)
    b.pos = [0, 0, base_z]
    for p in path:
        s.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.003, 0, 0], pos=p + [0, 0, base_z],
                             rgba=[0.79, 0.52, 0.06, 1], contype=0, conaffinity=0, group=0)
    tb = s.worldbody.add_body(name='target', mocap=True)
    tb.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.008, 0, 0], rgba=[0.79, 0.52, 0.06, 0.6], contype=0, conaffinity=0)
    m = s.compile(); m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    return m


def fig8_ref(p0, T=2.0, A=0.08, B=0.045, C=0.03, ramp=1.0):
    """Figure-eight about the standing foot: x = A sin wt, z = B sin 2wt, plus a lateral C sin wt; amplitude ramped up by a
    minimum-jerk profile over the first `ramp` seconds so the reference starts at rest."""
    w = 2 * np.pi / T
    def p(t):
        s = np.clip(t / ramp, 0, 1); a = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        return p0 + a * np.array([A * np.sin(w * t), C * np.sin(w * t), B * np.sin(2 * w * t)])
    return p


def run_figure_eight(duration=7.0, mp4=None, still=None, kp=150.0, kd=4.0):
    """FL foot tracks the figure-eight; the QP integrates q_ref each 2 ms tick, a joint PD with MuJoCo's bias force (gravity and
    Coriolis) as feed-forward follows it. The other legs hold the standing pose. Error = MuJoCo foot centre vs reference."""
    mtmp = mujoco.MjModel.from_xml_path(require_model()); L = Leg(mtmp, 'FL'); p0 = L.fk(STAND); ref = fig8_ref(p0)
    m = fixed_base_model([ref(t) for t in np.linspace(1.0, 3.0, 120)]); d = mujoco.MjData(m); idx = sim.index(m)
    d.qpos[idx.qpos] = np.tile(STAND, 4); mujoco.mj_forward(m, d)
    base = d.xpos[idx.base].copy(); fl = idx.legs['FL']; state = dict(q=STAND.copy(), qd=np.zeros(3))
    q_des = np.tile(STAND, 4); log = []

    def ctrl(m, d):
        tn = d.time + DT; q = state['q']; J = L.jacobian(q); e = ref(tn) - L.fk(q)
        dq, _ = qp_step(J, e, LAM, *step_bounds(L, q, DT))
        state['q'], state['qd'] = q + dq, dq / DT
        q_des[:3] = state['q']; qd_des = np.zeros(12); qd_des[:3] = state['qd']
        d.mocap_pos[0] = base + ref(tn)
        return kp * (q_des - d.qpos[idx.qpos]) + kd * (qd_des - d.qvel[idx.qvel]) + d.qfrc_bias[idx.qvel]

    def on_step(m, d):
        log.append((d.time, *(d.geom_xpos[fl['foot']] - base), *ref(d.time), *L.fk(state['q']), *d.qpos[fl['qpos']]))
        if rec is not None:
            rec(m, d)

    rec = sim.Recorder(m, mp4, width=960, height=540, distance=0.75, azimuth=-105, elevation=-8) if mp4 else None
    if rec is not None:                                 # base is welded: a fixed camera aimed at the leg, from the robot's left
        rec.cam.type = mujoco.mjtCamera.mjCAMERA_FREE; rec.cam.lookat[:] = base + p0 + [-0.05, 0, 0.12]
    video = None
    if rec is None:
        sim.run(m, d, ctrl, duration, on_step=on_step)
    else:
        with rec:
            sim.run(m, d, ctrl, duration, on_step=on_step)
            video = rec.close()
            if still:
                rec.still(d, still)
    a = np.array(log); t, pm, pr, pk = a[:, 0], a[:, 1:4], a[:, 4:7], a[:, 7:10]
    err, eik = np.linalg.norm(pm - pr, axis=1), np.linalg.norm(pk - pr, axis=1)
    out = dict(duration_s=duration, kp=kp, kd=kd, fig8_ik_max_error_m=float(eik.max()), fig8_mujoco_rms_error_m=float(np.sqrt(np.mean(err ** 2))),
               fig8_mujoco_max_error_m=float(err.max()), fig8_mujoco_rms_error_after_ramp_m=float(np.sqrt(np.mean(err[t > 1] ** 2))))
    if video:
        out['mp4_bytes'] = pathlib.Path(video).stat().st_size
    return out, a


# ---- figures ---------------------------------------------------------------------------------------------------------------

def reachable_grid(L, plane, u, v):
    """Boolean grid: closed-form IK has an in-limit solution at each point of a plane slice (base frame)."""
    G = np.zeros((len(v), len(u)), bool)
    for i, vv in enumerate(v):
        for j, uu in enumerate(u):
            G[i, j] = len(L.ik_closed_form(plane(uu, vv))) > 0
    return G


def fig_workspace(L, A, B):
    (tA, pA, runsA), (tB, pB, runsB) = A, B
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 4.4))
    yA = L.hip[1] + L.d; u = np.linspace(-0.45, 0.45, 181) + L.hip[0]; v = np.linspace(-0.45, 0.45, 181)
    G = reachable_grid(L, lambda x, z: np.array([x, yA, z]), u, v)
    ax[0].contourf(u, v, G, levels=[0.5, 1.5], colors=[C['fill']]); ax[0].contour(u, v, G, levels=[0.5], colors=[C['muted']], linewidths=0.6)
    th = np.linspace(0, 2 * np.pi, 400); rr = np.sqrt(reach_radius(L) ** 2 - L.d ** 2)
    ax[0].plot(L.hip[0] + rr * np.cos(th), rr * np.sin(th), '--', color=C['gt'], lw=0.8, label=f'|p - hip| = {reach_radius(L):.4f} m (knee limit)')
    ax[0].plot(pA[:, 0], pA[:, 2], color=C['plan'], lw=2.2, label='reference')
    ax[0].plot(runsA['dls']['p'][:, 0], runsA['dls']['p'][:, 2], color=C['bad'], lw=1.0, label='DLS (breaks knee limit)')
    ax[0].plot(runsA['dls_clip']['p'][:, 0], runsA['dls_clip']['p'][:, 2], color=C['est2'], lw=1.0, label='DLS, then clipped')
    ax[0].plot(runsA['qp']['p'][:, 0], runsA['qp']['p'][:, 2], color=C['est'], lw=1.2, label='QP')
    ax[0].plot(*L.fk(STAND)[[0, 2]], 'o', color=C['ink'], ms=4); ax[0].plot(L.hip[0], 0, '+', color=C['ink'], ms=8)
    ax[0].set(xlabel='x forward [m]', ylabel='z up [m]', title=f'Sagittal slice y = {yA:.4f} m: path A', aspect='equal')
    ax[0].legend(loc='upper left', fontsize=7.5)
    ins = ax[0].inset_axes([0.62, 0.62, 0.36, 0.36])
    ins.plot(L.hip[0] + rr * np.cos(th), rr * np.sin(th), '--', color=C['gt'], lw=0.8); ins.plot(pA[:, 0], pA[:, 2], color=C['plan'], lw=2.2)
    for n, c in (('dls', C['bad']), ('dls_clip', C['est2']), ('qp', C['est'])):
        ins.plot(runsA[n]['p'][:, 0], runsA[n]['p'][:, 2], color=c, lw=1.0)
    ins.set(xlim=(0.12, 0.27), ylim=(-0.42, -0.37), aspect='equal'); ins.tick_params(labelsize=6)
    xB = pB[0, 0]; u = np.linspace(-0.45, 0.45, 181) + L.hip[1]; v = np.linspace(-0.45, 0.45, 181)
    G = reachable_grid(L, lambda y, z: np.array([xB, y, z]), u, v)
    ax[1].contourf(u, v, G, levels=[0.5, 1.5], colors=[C['fill']]); ax[1].contour(u, v, G, levels=[0.5], colors=[C['muted']], linewidths=0.6)
    ax[1].plot(L.hip[1] + abs(L.d) * np.cos(th), abs(L.d) * np.sin(th), '--', color=C['bad'], lw=0.8, label=f'|(y, z)| = d: s_z = 0, det J = 0')
    ax[1].plot(pB[:, 1], pB[:, 2], color=C['plan'], lw=2.2, label='reference')
    ax[1].plot(runsB['dls']['p'][:, 1], runsB['dls']['p'][:, 2], color=C['bad'], lw=0.8, label='DLS')
    ax[1].plot(runsB['qp']['p'][:, 1], runsB['qp']['p'][:, 2], color=C['est'], lw=1.2, label='QP')
    ax[1].plot(L.hip[1], 0, '+', color=C['ink'], ms=8)
    ax[1].set(xlabel='y left [m]', ylabel='z up [m]', title=f'Frontal slice x = {xB:.4f} m (hip + 0.20 m): path B', aspect='equal')
    ax[1].legend(loc='lower left', fontsize=7.5)
    ins = ax[1].inset_axes([0.62, 0.62, 0.36, 0.36])
    ins.plot(L.hip[1] + abs(L.d) * np.cos(th), abs(L.d) * np.sin(th), '--', color=C['bad'], lw=0.8)
    ins.plot(pB[:, 1], pB[:, 2], color=C['plan'], lw=2.2); ins.plot(runsB['dls']['p'][:, 1], runsB['dls']['p'][:, 2], color=C['bad'], lw=0.6)
    ins.plot(runsB['qp']['p'][:, 1], runsB['qp']['p'][:, 2], color=C['est'], lw=1.0)
    ins.set(xlim=(L.hip[1] + 0.06, L.hip[1] + 0.15), ylim=(-0.04, 0.03), aspect='equal'); ins.tick_params(labelsize=6)
    fs.save(fig, FIG / 'workspace')


def fig_qp_step(L, runsA, pA, k=None):
    """Joint-space picture of one tick at the knee limit: cost contours over (thigh, knee) steps with the hip step held at the QP
    value, the box of feasible steps, and the DLS, clipped and QP steps."""
    qs = np.vstack([STAND, runsA['qp']['q'][:-1]])
    if k is None:                                                       # first tick where the knee bound is active and e is large
        act = [i for i in range(len(pA)) if qs[i][2] >= L.hi[2] - 1e-12 and np.linalg.norm(pA[i] - L.fk(qs[i])) > 0.01]
        k = act[0]
    q = qs[k]; J = L.jacobian(q); e = pA[k] - L.fk(q); lo, hi = step_bounds(L, q, DT)
    x_dls = dls_step(J, e, LAM); x_clip = np.clip(x_dls, lo, hi); x_qp, _ = qp_step(J, e, LAM, lo, hi)
    cost = lambda x: np.sum((J @ x - e) ** 2) + LAM ** 2 * np.sum(x ** 2)       # noqa: E731
    s = 1e3; span = 1.3 * max(np.abs(x_dls[1:]).max(), np.abs(hi[1:] - lo[1:]).max())
    a = np.linspace(-span, span, 241) + x_qp[1]; b = np.linspace(-span, span, 241) + x_qp[2]
    Z = np.array([[cost(np.array([x_qp[0], aa, bb])) for aa in a] for bb in b])
    fig, ax = plt.subplots(figsize=(5.6, 4.6))
    ax.contour(a * s, b * s, np.sqrt(Z) * 1e3, levels=12, colors=[C['grid']], linewidths=0.7)
    ax.add_patch(plt.Rectangle((lo[1] * s, lo[2] * s), (hi[1] - lo[1]) * s, (hi[2] - lo[2]) * s, fc='none', ec=C['ink'], lw=1.0))
    ax.text(lo[1] * s, lo[2] * s, ' feasible steps: velocity limits, knee limit on top', fontsize=7.5, va='top', color=C['ink'])
    for x, c, n in ((x_dls, C['bad'], 'DLS'), (x_clip, C['est2'], 'DLS clipped'), (x_qp, C['est'], 'QP')):
        ax.plot(x[1] * s, x[2] * s, 'o', color=c, ms=6)
        ax.annotate(f'{n}: residual {np.linalg.norm(J @ x - e) * 1e3:.2f} mm', (x[1] * s, x[2] * s), xytext={'DLS': (8, 0), 'DLS clipped': (-6, 12), 'QP': (8, 10)}[n], ha='right' if n == 'DLS clipped' else 'left',
                    textcoords='offset points', fontsize=7.5, color=c)
    ax.set(xlabel='thigh step [mrad]', ylabel='knee step [mrad]', aspect='equal',
           title=f'One tick of path A (t = {(k + 1) * DT:.3f} s), hip step {x_qp[0] * s:.2f} mrad')
    ax.text(0.02, 0.02, 'grey: contours of the QP cost, hip step held at the QP value', transform=ax.transAxes, fontsize=7, color=C['muted'])
    fs.save(fig, FIG / 'qp_step')
    return dict(tick=k, e_norm_m=float(np.linalg.norm(e)), dls=x_dls.tolist(), clip=x_clip.tolist(), qp=x_qp.tolist(),
                residual_m={n: float(np.linalg.norm(J @ x - e)) for n, x in (('dls', x_dls), ('clip', x_clip), ('qp', x_qp))})


def fig_tracking(name, L, t, runs, dist, title, speed_panel, err_max=None):
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True)
    ax[0].fill_between(t, 0, dist * 1e3, color=C['fill'], lw=0); ax[0].plot(t, dist * 1e3, '--', color=C['gt'], lw=1.0, label='distance to workspace (lower bound)')
    style = dict(pinv=(C['bad'], ':', 'pseudo-inverse'), dls=(C['bad'], '-', f'DLS, λ = {LAM}'), dls_l05=(C['muted'], '-', 'DLS, λ = 0.05'),
                 dls_clip=(C['est2'], '-', 'DLS, then clipped'), qp=(C['est'], '-', 'QP'),
                 qp_adaptive=(C['good'], '-', 'QP, adaptive damping'))
    for n, r in runs.items():
        c, ls, lab = style[n]; ax[0].plot(t, r['err'] * 1e3, ls, color=c, lw=1.0 if n != 'qp' else 1.4, label=lab)
    ax[0].set(ylabel='foot error [mm]', title=title); ax[0].legend(fontsize=7, ncol=3, loc='upper center')
    if err_max:
        ax[0].set_ylim(0, err_max); ax[0].text(t[len(t) // 2], 0.45 * err_max, f'pseudo-inverse peaks at {runs["pinv"]["err"].max() * 1e3:.0f} mm,\naxis cut at {err_max} mm',
                                                ha='center', va='center', fontsize=7, color=C['bad'])
    if speed_panel:
        for n, r in runs.items():
            c, ls, lab = style[n]; ax[1].semilogy(t, np.abs(r['qd']).max(1), ls, color=c, lw=0.9 if n != 'qp' else 1.4)
        ax[1].axhline(QD_MAX.max(), color=C['plan'], lw=0.8); ax[1].text(t[0], QD_MAX.max() * 1.3, ' 30.1 rad/s hip and thigh limit', fontsize=7, color=C['plan'])
        ax[1].set(ylabel='largest joint speed [rad/s]', xlabel='time [s]')
    else:
        for n, r in runs.items():
            c, ls, lab = style[n]; ax[1].plot(t, r['q'][:, 2], ls, color=c, lw=1.0 if n != 'qp' else 1.4)
        ax[1].axhline(L.hi[2], color=C['plan'], lw=0.8); ax[1].text(t[0], L.hi[2] + 0.02, ' knee upper limit −0.838 rad', fontsize=7, color=C['plan'])
        ax[1].set(ylabel='knee angle [rad]', xlabel='time [s]')
    fs.save(fig, FIG / name)


def fig_figure_eight(a):
    t, pm, pr = a[:, 0], a[:, 1:4], a[:, 4:7]; err = np.linalg.norm(pm - pr, axis=1)
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.8), gridspec_kw=dict(width_ratios=[1, 1.3]))
    ax[0].plot(pr[:, 0], pr[:, 2], color=C['plan'], lw=2.4, label='reference'); ax[0].plot(pm[:, 0], pm[:, 2], color=C['gt'], lw=0.9, label='MuJoCo foot')
    ax[0].set(xlabel='x forward [m]', ylabel='z up [m]', title='FL foot, base frame', aspect='equal'); ax[0].legend(fontsize=7.5)
    ax[1].plot(t, err * 1e3, color=C['gt']); ax[1].axvline(1.0, color=C['muted'], lw=0.6, ls=':')
    ax[1].set(xlabel='time [s]', ylabel='|foot − reference| [mm]', title='Tracking error in MuJoCo (QP IK + joint PD)')
    fs.save(fig, FIG / 'figure_eight')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=1000); ap.add_argument('--no-media', action='store_true')
    ap.add_argument('--json', default=str(HERE / 'results.json')); a = ap.parse_args(); t0 = time.time()
    res = dict(n_per_leg=a.n, fundamental=fundamental(a.n)); print('intermediate ...')
    res['intermediate'] = intermediate(a.n); print('advanced ...')
    m = mujoco.MjModel.from_xml_path(require_model()); L = Leg(m, 'FL'); rng = np.random.default_rng(11)
    adv = dict(qp_random=qp_checks())
    tA, pA, qA = traj_boundary(L); adv['boundary'], runsA, dA = evaluate(L, tA, pA, qA)
    tB, pB, qB = traj_singular(L); adv['singular'], runsB, dB = evaluate(L, tB, pB, qB)
    adv['adaptive_sweep'] = []                                                       # how ADAPT was chosen
    for eps, lmax in ((0.02, 0.05), (0.03, 0.05), (0.04, 0.08)):
        r = track(L, pB, qB, DT, 'qp', lam=LAM, adapt=(eps, lmax)); err = np.linalg.norm(r['p'] - pB, axis=1); un = dB > 0
        qd = np.abs(np.diff(np.vstack([qB, r['q']]), axis=0)) / DT
        adv['adaptive_sweep'].append(dict(eps=eps, lam_max=lmax, rms_error_reachable_m=float(np.sqrt(np.mean(err[~un] ** 2))),
                                          excess_error_max_m=float((err - dB).max()), mean_max_speed_unreachable_rad_s=float(qd[un].max(1).mean())))
    adv['qp_ticks_vs_lsq_linear_max'] = max(ticks_vs_lsq_linear(L, runsA['qp'], pA, qA), ticks_vs_lsq_linear(L, runsB['qp'], pB, qB))
    far = np.concatenate([pA[dA > 0][::25], pB[dB > 0][::25]])
    adv['workspace_distance_vs_scipy_max_m'] = distance_vs_scipy(L, far, rng); adv['workspace_distance_n_points'] = len(far)
    adv['reach_radius_m'] = float(reach_radius(L))
    print('figure-eight in MuJoCo ...')
    adv['figure_eight'], log8 = run_figure_eight(mp4=None if a.no_media else MEDIA / 'figure_eight.mp4', still=MEDIA / 'figure_eight.png')
    adv['figure_eight_gain_x2'] = run_figure_eight(kp=300.0, kd=8.0)[0]            # cause of the MuJoCo error: does it halve with the gains?
    res['advanced'] = adv
    fig_workspace(L, (tA, pA, runsA), (tB, pB, runsB)); adv['qp_step_figure'] = fig_qp_step(L, runsA, pA)
    fig_tracking('tracking_boundary', L, tA, {k: runsA[k] for k in ('dls', 'dls_clip', 'qp')}, dA,
                 'Path A: circle through the outer workspace boundary', speed_panel=False)
    fig_tracking('tracking_singular', L, tB, {k: runsB[k] for k in ('pinv', 'dls', 'dls_l05', 'dls_clip', 'qp', 'qp_adaptive')}, dB,
                 'Path B: lateral motion into the singular inner boundary', speed_panel=True, err_max=80)
    fig_figure_eight(log8)

    F, I = res['fundamental'], res['intermediate']
    flat = dict(
        fk_vs_mujoco_max_m=max(r['fk_vs_mujoco_max_m'] for r in F.values()), fk_vs_pinocchio_max_m=max(r['fk_vs_pinocchio_max_m'] for r in F.values()),
        jac_vs_finite_diff_max=max(r['jac_vs_finite_diff_max'] for r in F.values()), jac_vs_pinocchio_max=max(r['jac_vs_pinocchio_max'] for r in F.values()),
        ik_multistart_success_min=min(r['ik_success'] for r in F.values()),
        closed_form_success_min=min(r['closed_form_success'] for r in I['ik'].values()),
        closed_form_max_q_error_rad=max(r['closed_form_max_q_error_rad'] for r in I['ik'].values()),
        closed_form_max_fk_residual_m=max(r['closed_form_max_fk_residual_m'] for r in I['ik'].values()),
        det_formula_vs_numpy_max=I['singularity']['det_formula_vs_numpy_max'],
        statics_max_abs_residual_Nm=max(r['max_abs_residual'] for r in I['statics']['legs'].values()),
        qp_vs_lsq_linear_max=adv['qp_random']['qp_vs_lsq_linear_max'], qp_kkt_residual_max=adv['qp_random']['qp_kkt_residual_max'],
        qp_max_iterations=max(adv['qp_random']['qp_max_iterations_random'], adv['boundary']['methods']['qp']['qp_iterations_max'],
                              adv['singular']['methods']['qp']['qp_iterations_max']),
        qp_ticks_vs_lsq_linear_max=adv['qp_ticks_vs_lsq_linear_max'], workspace_distance_vs_scipy_max_m=adv['workspace_distance_vs_scipy_max_m'],
        boundary_qp_limit_violation_rad=adv['boundary']['methods']['qp']['limit_violation_rad'],
        boundary_qp_rms_error_reachable_m=adv['boundary']['methods']['qp']['rms_error_reachable_m'],
        boundary_qp_excess_error_max_m=adv['boundary']['methods']['qp']['excess_error_max_m'],
        singular_qp_speed_over_limit_max=adv['singular']['methods']['qp']['speed_over_limit_max'],
        singular_qp_rms_error_reachable_m=adv['singular']['methods']['qp']['rms_error_reachable_m'],
        singular_qp_excess_error_max_m=adv['singular']['methods']['qp']['excess_error_max_m'],
        **{k: adv['figure_eight'][k] for k in ('fig8_ik_max_error_m', 'fig8_mujoco_rms_error_m', 'fig8_mujoco_max_error_m')})
    if 'mp4_bytes' in adv['figure_eight']:
        flat['media_mp4_under_6MB'] = float(adv['figure_eight']['mp4_bytes'] < 6e6)
    res['checks'] = {k: dict(measured=flat[k], op=op, threshold=thr, passed=check(k, flat[k])) for k, (op, thr) in ACCEPT.items() if k in flat}
    print('\nmethod comparison (FL):')
    for path in ('boundary', 'singular'):
        for n, r in adv[path]['methods'].items():
            print(f'  {path:8s} {n:8s} rms(reach) {r["rms_error_reachable_m"] * 1e3:7.3f} mm  excess max {r["excess_error_max_m"] * 1e3:8.2f} mm  '
                  f'viol {r["limit_violation_rad"]:9.3g} rad  speed/limit {r["speed_over_limit_max"]:9.3g}  '
                  f'unreachable mean speed {r["mean_max_speed_unreachable_rad_s"]:9.3g}  {r["us_per_tick"]:.0f} us/tick')
    print(json.dumps(adv['figure_eight'], indent=1))
    print('\nacceptance:')
    for k, c in res['checks'].items():
        print(f'  {"PASS" if c["passed"] else "MISS"}  {k:40s} {c["measured"]:.3g}  ({c["op"]} {c["threshold"]})')
    res['wall_time_s'] = round(time.time() - t0, 1)
    pathlib.Path(a.json).write_text(json.dumps(res, indent=2, default=float) + '\n')
    print(f'wrote {a.json} in {res["wall_time_s"]} s')


if __name__ == '__main__':
    main()
