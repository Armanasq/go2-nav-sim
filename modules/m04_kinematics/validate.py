#!/usr/bin/env python3
"""Experiment for module 04, all three levels.
  fundamental: FK and Jacobian (kinematics.py) against MuJoCo, Pinocchio and finite differences; numerical IK success.
  intermediate: closed-form IK, Jacobian determinant and statics (analytic.py), statics against a MuJoCo standing equilibrium.
  advanced: differential IK as a box-constrained QP (diffik.py) against scipy.optimize.lsq_linear; tracking a foot path through
      the outer workspace boundary and one through the inner, singular boundary, against unconstrained damped least squares;
      a figure-eight tracked in MuJoCo.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--n 1000] [--no-media]"""
import argparse, json, os, pathlib, sys, time
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
from diffik import QD_MAX, box, distance_to_workspace, dls_step, fold_radius, qp_active_set, qp_step, reach_radius, step_bounds, track  # noqa: E402

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
    'closed_vs_numerical_agree_min': ('>=', 1.0),                      # missed on RR (README, failure analysis)
    'closed_vs_numerical_agree_nonsingular_min': ('>=', 1.0),          # post-hoc: |det J| > 1e-4 subset, added after the RR miss
    # advanced
    'qp_vs_lsq_linear_max': ('<', 1e-9), 'qp_kkt_residual_max': ('<', 1e-9), 'qp_max_iterations': ('<=', 10),
    'qp_ticks_vs_lsq_linear_max': ('<', 1e-9), 'workspace_distance_vs_scipy_max_m': ('<', 1e-6),
    'workspace_distance_folded_vs_scipy_max_m': ('<', 1e-6),
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
        qt = [rng.uniform(L.lo, L.hi) for _ in range(n)]; targets = [L.fk(q) for q in qt]   # IK targets: FK of random in-limit q
        single = [L.ik(t, STAND) for t in targets]; ok = np.array([s[3] for s in single]); ok1 = int(ok.sum())
        th = np.array([q[1] for q in qt]); bins = dict(below_0=th < 0, from_0_to_1p6=(th >= 0) & (th <= 1.6),
                                                        from_1p6_to_2p6=(th > 1.6) & (th <= 2.6), above_2p6=th > 2.6)
        at_limit = [bool(np.any((s[0] <= L.lo) | (s[0] >= L.hi))) for s in single if not s[3]]
        t0 = time.perf_counter(); sol = [L.ik_multistart(t, STAND) for t in targets]; dt = (time.perf_counter() - t0) / n
        its = [s[1] for s in sol]
        res[leg] = dict(fk_vs_mujoco_max_m=max(e_mj), fk_vs_pinocchio_max_m=max(e_pin), jac_vs_finite_diff_max=max(e_fd),
                        jac_vs_pinocchio_max=max(e_jpin), ik_single_seed_success=ok1 / n,
                        ik_single_seed_by_thigh={k: dict(n=int(b.sum()), n_ok=int(ok[b].sum())) for k, b in bins.items()},
                        ik_single_seed_failures_at_limit=dict(n=len(at_limit), n_at_limit=int(sum(at_limit))), ik_success=sum(s[3] for s in sol) / n,
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
    return max(abs(scipy_distance(L, p, rng) - distance_to_workspace(L, p)) for p in pts)


def scipy_distance(L, p, rng):
    f = lambda q: np.sum((L.fk(q) - p) ** 2)                                         # noqa: E731
    jac = lambda q: 2 * L.jacobian(q).T @ (L.fk(q) - p)                              # noqa: E731
    return np.sqrt(min(minimize(f, rng.uniform(L.lo, L.hi), jac=jac, method='L-BFGS-B', bounds=list(zip(L.lo, L.hi)),
                                options=dict(ftol=1e-20, gtol=1e-14, maxiter=2000)).fun for _ in range(12)))


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
        gap = np.linalg.norm((r['p'] - L.hip)[:, 1:], axis=1) - abs(L.d)              # foot distance outside the cylinder |(y, z)| = d
        r.update(err=err, qd=qd, sigma_min=sig); runs[name] = r
        out[name] = dict(rms_error_reachable_m=float(np.sqrt(np.mean(err[reach] ** 2))), max_error_reachable_m=float(err[reach].max()),
                         max_error_m=float(err.max()), t_max_error_s=float(t[err.argmax()]), distance_at_max_error_m=float(dist[err.argmax()]),
                         excess_error_max_m=float((err - dist).max()), excess_error_mean_m=float((err - dist).mean()),
                         limit_violation_rad=viol, max_speed_rad_s=np.abs(qd).max(0).tolist(),
                         speed_over_limit_max=float((np.abs(qd) / QD_MAX).max()), sigma_min_min=float(sig.min()),
                         mean_max_speed_unreachable_rad_s=float(np.abs(qd[~reach]).max(1).mean()) if (~reach).any() else 0.0,
                         cylinder_gap_min_m=float(gap.min()), cylinder_gap_max_unreachable_m=float(gap[~reach].max()) if (~reach).any() else 0.0,
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


# Colour per method, the same in every figure. C['bad'] is kept for fail marks (MODULE_SPEC), so the unconstrained baselines are
# neutral grey and the clipped baseline is ink; the QP is the estimate (blue), the adaptive QP the second estimate (violet). The
# distance to the workspace, the best error any in-limit method can reach, is the reference (amber).
STYLE = dict(pinv=(C['muted'], ':', 'pseudo-inverse'), dls=(C['muted'], '-', f'DLS, λ = {LAM}'), dls_l05=(C['muted'], '-.', 'DLS, λ = 0.05'),
             dls_clip=(C['ink'], '-', 'DLS, then clipped'), qp=(C['est'], '-', 'QP'), qp_adaptive=(C['est2'], '-', 'QP, adaptive damping'))


def fig_workspace(L, A, B):
    (tA, pA, runsA), (tB, pB, runsB) = A, B
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 5.2))
    yA = L.hip[1] + L.d; u = np.linspace(-0.45, 0.45, 181) + L.hip[0]; v = np.linspace(-0.45, 0.45, 181)
    G = reachable_grid(L, lambda x, z: np.array([x, yA, z]), u, v)
    ax[0].contourf(u, v, G, levels=[0.5, 1.5], colors=[C['fill']]); ax[0].contour(u, v, G, levels=[0.5], colors=[C['gt']], linewidths=0.6)
    th = np.linspace(0, 2 * np.pi, 400); rr = np.sqrt(reach_radius(L) ** 2 - L.d ** 2)
    ax[0].plot(L.hip[0] + rr * np.cos(th), rr * np.sin(th), '--', color=C['gt'], lw=0.8, label=f'|p - hip| = {reach_radius(L):.4f} m (knee straight limit)')
    rf = np.sqrt(fold_radius(L) ** 2 - L.d ** 2)
    ax[0].plot(L.hip[0] + rf * np.cos(th), rf * np.sin(th), ':', color=C['gt'], lw=1.0, label=f'|p - hip| = {fold_radius(L):.4f} m (knee folded limit)')
    ax[0].plot(pA[:, 0], pA[:, 2], color=C['plan'], lw=2.2, label='reference')
    ax[0].plot(runsA['dls']['p'][:, 0], runsA['dls']['p'][:, 2], color=STYLE['dls'][0], lw=1.0, label='DLS (breaks knee limit)')
    ax[0].plot(runsA['dls_clip']['p'][:, 0], runsA['dls_clip']['p'][:, 2], color=STYLE['dls_clip'][0], lw=1.0, label='DLS, then clipped')
    ax[0].plot(runsA['qp']['p'][:, 0], runsA['qp']['p'][:, 2], color=C['est'], lw=1.2, label='QP')
    ax[0].plot(*L.fk(STAND)[[0, 2]], 'o', color=C['ink'], ms=4); ax[0].plot(L.hip[0], 0, '+', color=C['ink'], ms=8)
    ax[0].set(xlabel='x forward [m]', ylabel='z up [m]', title=f'Sagittal slice y = {yA:.4f} m: path A', aspect='equal')
    ax[0].legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=2, fontsize=7.5)
    ins = ax[0].inset_axes([0.62, 0.62, 0.36, 0.36])
    ins.plot(L.hip[0] + rr * np.cos(th), rr * np.sin(th), '--', color=C['gt'], lw=0.8); ins.plot(pA[:, 0], pA[:, 2], color=C['plan'], lw=2.2)
    for n in ('dls', 'dls_clip', 'qp'):
        ins.plot(runsA[n]['p'][:, 0], runsA[n]['p'][:, 2], color=STYLE[n][0], lw=1.0)
    ins.set(xlim=(0.12, 0.27), ylim=(-0.42, -0.37), aspect='equal'); ins.tick_params(labelsize=6)
    xB = pB[0, 0]; u = np.linspace(-0.45, 0.45, 181) + L.hip[1]; v = np.linspace(-0.45, 0.45, 181)
    G = reachable_grid(L, lambda y, z: np.array([xB, y, z]), u, v)
    ax[1].contourf(u, v, G, levels=[0.5, 1.5], colors=[C['fill']]); ax[1].contour(u, v, G, levels=[0.5], colors=[C['gt']], linewidths=0.6)
    ax[1].plot(L.hip[1] + abs(L.d) * np.cos(th), abs(L.d) * np.sin(th), '-.', color=C['gt'], lw=0.8, label=f'|(y, z)| = d: s_z = 0, det J = 0')
    ax[1].plot(pB[:, 1], pB[:, 2], color=C['plan'], lw=2.2, label='reference')
    ax[1].plot(runsB['dls']['p'][:, 1], runsB['dls']['p'][:, 2], color=STYLE['dls'][0], lw=0.8, label='DLS')
    ax[1].plot(runsB['qp']['p'][:, 1], runsB['qp']['p'][:, 2], color=C['est'], lw=1.2, label='QP')
    ax[1].plot(L.hip[1], 0, '+', color=C['ink'], ms=8)
    ax[1].set(xlabel='y left [m]', ylabel='z up [m]', title=f'Frontal slice x = {xB:.4f} m (hip + 0.20 m): path B', aspect='equal')
    ax[1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=2, fontsize=7.5)
    ins = ax[1].inset_axes([0.62, 0.62, 0.36, 0.36])
    ins.plot(L.hip[1] + abs(L.d) * np.cos(th), abs(L.d) * np.sin(th), '-.', color=C['gt'], lw=0.8)
    ins.plot(pB[:, 1], pB[:, 2], color=C['plan'], lw=2.2); ins.plot(runsB['dls']['p'][:, 1], runsB['dls']['p'][:, 2], color=STYLE['dls'][0], lw=0.6)
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
    for x, c, n in ((x_dls, STYLE['dls'][0], 'DLS'), (x_clip, STYLE['dls_clip'][0], 'DLS clipped'), (x_qp, STYLE['qp'][0], 'QP')):
        ax.plot(x[1] * s, x[2] * s, 'o', color=c, ms=6)
        ax.annotate(f'{n}: residual {np.linalg.norm(J @ x - e) * 1e3:.2f} mm', (x[1] * s, x[2] * s), xytext={'DLS': (8, 0), 'DLS clipped': (8, -12), 'QP': (8, 10)}[n], ha='left',
                    textcoords='offset points', fontsize=7.5, color=c)
    ax.set(xlabel='thigh step [mrad]', ylabel='knee step [mrad]', aspect='equal',
           title=f'One tick of path A (t = {(k + 1) * DT:.3f} s), hip step {x_qp[0] * s:.2f} mrad')
    ax.text(0.02, 0.02, 'grey: contours of the QP cost, hip step held at the QP value', transform=ax.transAxes, fontsize=7, color=C['muted'])
    fs.save(fig, FIG / 'qp_step')
    return dict(tick=k, e_norm_m=float(np.linalg.norm(e)), dls=x_dls.tolist(), clip=x_clip.tolist(), qp=x_qp.tolist(),
                residual_m={n: float(np.linalg.norm(J @ x - e)) for n, x in (('dls', x_dls), ('clip', x_clip), ('qp', x_qp))})


def fig_tracking(name, L, t, runs, dist, title, speed_panel, err_max=None, bump=None):
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True)
    ax[0].fill_between(t, 0, dist * 1e3, color=C['fill'], lw=0); ax[0].plot(t, dist * 1e3, '--', color=C['plan'], lw=1.2, label='distance to workspace (lower bound)')
    style = STYLE
    for n, r in runs.items():
        c, ls, lab = style[n]; ax[0].plot(t, r['err'] * 1e3, ls, color=c, lw=1.0 if n != 'qp' else 1.4, label=lab)
    ax[0].set(ylabel='foot error [mm]', title=title); ax[0].legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.01, 1.0))
    if err_max:
        ax[0].set_ylim(0, err_max); ax[0].text(t[len(t) // 2], 0.85 * err_max, f'pseudo-inverse peaks at {runs["pinv"]["err"].max() * 1e3:.0f} mm,\naxis cut at {err_max} mm',
                                                ha='center', va='center', fontsize=7, color=C['ink'], bbox=dict(fc=C['paper'], ec='none', alpha=0.9))
    if bump:                                                            # zoom on the first error bump of the adaptive QP
        t0 = bump['t_s']; ins = ax[0].inset_axes([0.42, 0.14, 0.18, 0.5]); w = (t > t0 - 0.12) & (t < t0 + 0.12)
        ins.fill_between(t[w], 0, dist[w] * 1e3, color=C['fill'], lw=0); ins.plot(t[w], dist[w] * 1e3, '--', color=C['plan'], lw=1.0)
        for n in ('dls_clip', 'qp', 'qp_adaptive'):
            c, ls, _ = style[n]; ins.plot(t[w], runs[n]['err'][w] * 1e3, ls, color=c, lw=1.2)
        ins.set(xlim=(t0 - 0.12, t0 + 0.12), ylim=(0, 1.3 * bump['error_m'] * 1e3), xticks=np.round([t0 - 0.1, t0 + 0.1], 2)); ins.tick_params(labelsize=6)
        ins.set_title(f'zoom: adaptive QP, {bump["error_m"] * 1e3:.1f} mm', fontsize=6.5, loc='center', color=C['est2'], pad=2)
    if speed_panel:
        for n, r in runs.items():
            c, ls, lab = style[n]; ax[1].semilogy(t, np.abs(r['qd']).max(1), ls, color=c, lw=0.9 if n != 'qp' else 1.4)
        ax[1].axhline(QD_MAX.max(), color=C['plan'], lw=0.8); ax[1].text(t[0], QD_MAX.max() * 1.3, ' 30.1 rad/s hip and thigh limit', fontsize=7, color=C['plan'], bbox=dict(fc=C['paper'], ec='none', alpha=0.9))
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


def fig_leg_diagram(L, q=(0.3, 0.9, -1.8), ox=0.50, fx_scale=10):
    """Method diagram, true scale for the FL leg: frontal view (abduction q1 about x, offset d, cylinder |(y, z)| = d) and the leg
    plane with q1 undone (thigh q2 and knee q3 about y, l1, l2, gamma, s, s_z, knee-limit shells, foot force)."""
    q1, q2, q3 = q; d, l1, l2 = abs(L.d), L.l1, L.l2; fx, fz = L.foot[0] * fx_scale, L.foot[2]
    ry = lambda th, v: np.array([v[0] * np.cos(th) + v[1] * np.sin(th), -v[0] * np.sin(th) + v[1] * np.cos(th)])   # noqa: E731  R_y on (x, z)
    knee = ry(q2, (0, -l1)); foot = knee + ry(q2 + q3, (fx, fz)); sz = float(knee[1] + ry(q2 + q3, (L.foot[0], fz))[1])
    fig, ax = fs.diagram(9.6, 4.6, (-0.17, 0.88), (-0.46, 0.13))
    arc = lambda c, r, a0, a1, **kw: ax.plot(c[0] + r * np.sin(np.linspace(a0, a1, 60)), c[1] - r * np.cos(np.linspace(a0, a1, 60)), **kw)  # noqa: E731
    # frontal view: y right, z up, x out of the page; angles measured from -z
    ax.text(-0.16, 0.115, 'Frontal view, from the front (x out of page)', fontsize=9, weight='semibold', va='top')
    th = np.linspace(0, 2 * np.pi, 200); ax.plot(d * np.cos(th), d * np.sin(th), '-.', color=C['gt'], lw=0.8)
    fs.label(ax, (d * np.cos(3.9), d * np.sin(3.9)), 'cylinder |(y, z)| = d:\n$s_z = 0$, det J = 0', xytext=(-6, -32), color=C['gt'], ha='center', fontsize=7.5)
    c1, s1 = np.cos(q1), np.sin(q1); T = np.array([d * c1, d * s1]); P = np.array([d * c1 - sz * s1, d * s1 + sz * c1])
    ax.plot(*zip((0, 0), T), color=C['ink'], lw=2.2); ax.plot(*zip(T, P), color=C['ink'], lw=2.2, solid_capstyle='round')
    ax.plot(0, 0, 'o', ms=9, mfc=C['paper'], mec=C['ink']); ax.plot(0, 0, '.', color=C['ink'], ms=3)
    fs.label(ax, (0, 0), 'hip (abduction)\naxis x', xytext=(-22, -20), ha='right', fontsize=7.5)
    ax.plot(*T, 'o', ms=5, color=C['ink']); fs.label(ax, T, 'thigh joint', xytext=(14, 10), fontsize=7.5)
    ax.annotate(f'$d$ = {d:.4f} m', (T / 2), xytext=(-12, 9), textcoords='offset points', ha='center', fontsize=8)
    ax.annotate(f'$|s_z|$ = {-sz:.3f} m', ((T + P) / 2), xytext=(8, 0), textcoords='offset points', ha='left', fontsize=8)
    ax.plot([0, 0.12], [0, 0], ':', color=C['muted'], lw=0.7); ang = np.linspace(0, q1, 30)
    ax.plot(0.075 * np.cos(ang), 0.075 * np.sin(ang), color=C['muted'], lw=0.8); ax.text(0.08, 0.012, '$q_1$', fontsize=8, color=C['muted'])
    fs.frame_axes(ax, (-0.14, -0.43), length=0.05, names=('y', 'z'))
    fs.arrow(ax, P, P + [0, 0.07], color=C['est'], lw=1.4); ax.text(P[0] + 0.008, P[1] + 0.045, '$F$ on foot', color=C['est'], fontsize=8)
    ax.plot(*P, 'o', ms=6, color=C['ink'])
    ax.text(-0.16, -0.36, '$p = R_x(q_1)\\,(s_x,\\ d,\\ s_z)$', fontsize=8.5)
    # leg plane: x right, z up, y into the page; origin at the thigh joint
    O = np.array([ox, 0.0]); K, Pf = O + knee, O + foot
    ax.text(ox - 0.12, 0.115, 'Leg plane, $q_1$ undone (y into page)', fontsize=9, weight='semibold', va='top')
    rs = [np.sqrt(fold_radius(L) ** 2 - L.d ** 2), np.sqrt(reach_radius(L) ** 2 - L.d ** 2)]
    for r, n in zip(rs, ('folded', 'straight')):
        arc(O, r, -0.8, 0.75, color=C['gt'], ls='--' if n == 'straight' else ':', lw=0.9)
    ax.text(O[0] + 0.25, O[1] - 0.33, f'knee straight limit:\n|s| = {rs[1]:.4f} m', color=C['gt'], fontsize=7.5)
    fs.label(ax, O + rs[0] * np.array([np.sin(0.6), -np.cos(0.6)]), f'folded limit:\n|s| = {rs[0]:.4f} m', xytext=(40, 26), color=C['gt'], fontsize=7.5)
    ax.plot([O[0], O[0]], [O[1], O[1] - 0.12], ':', color=C['muted'], lw=0.7); arc(O, 0.06, 0, -q2, color=C['muted'], lw=0.8)
    ax.text(O[0] - 0.055, O[1] - 0.085, '$q_2$', fontsize=8, color=C['muted'])
    ext = K + (K - O) / l1 * 0.09; ax.plot(*zip(K, ext), ':', color=C['muted'], lw=0.7)
    ka = np.arctan2((K - O)[0], -(K - O)[1]); arc(K, 0.035, -ka, -ka - q3, color=C['muted'], lw=0.8)    # thigh extension to calf
    ax.text(K[0] - 0.085, K[1] - 0.03, '$q_3 < 0$', fontsize=8, color=C['muted'])
    calf = K + ry(q2 + q3, (0, -l2)); ax.plot(*zip(K, calf), ':', color=C['muted'], lw=0.7)
    fs.label(ax, (calf + Pf) / 2, f'$\\gamma$ = {np.rad2deg(L.gamma):.2f}° from calf $-z$\n($f_x$ drawn {fx_scale}×)', xytext=(18, -14), fontsize=7.5)
    ax.plot(*zip(O, K), color=C['ink'], lw=2.2); ax.plot(*zip(K, Pf), color=C['ink'], lw=2.2)
    ax.plot(*zip(O, Pf), '--', color=C['est2'], lw=1.0); ax.text(*(O + Pf) / 2 + [0.006, 0.0], '$s$', color=C['est2'], fontsize=9)
    ax.annotate(f'$l_1$ = {l1:.3f} m', (O + K) / 2, xytext=(-8, 6), textcoords='offset points', ha='right', fontsize=8)
    ax.annotate(f'$l_2$ = {l2:.4f} m', (K + Pf) / 2, xytext=(-10, -12), textcoords='offset points', ha='right', fontsize=8)
    for j, n in ((O, 'thigh $q_2$'), (K, 'knee $q_3$')):
        ax.plot(*j, 'o', ms=9, mfc=C['paper'], mec=C['ink']); ax.plot(*j, 'x', color=C['ink'], ms=4)
    ax.text(O[0] + 0.012, O[1] + 0.02, 'thigh joint, axis y', fontsize=7.5)
    ax.text(K[0] - 0.012, K[1] + 0.012, 'knee, axis y', fontsize=7.5, ha='right')
    xd = Pf[0] + 0.10; ax.plot([O[0], xd + 0.01], [O[1], O[1]], ':', color=C['muted'], lw=0.7); ax.plot([Pf[0], xd + 0.01], [Pf[1], Pf[1]], ':', color=C['muted'], lw=0.7)
    fs.arrow(ax, (xd, O[1]), (xd, Pf[1]), style='<|-|>', color=C['ink'], lw=0.8)
    ax.text(xd + 0.008, Pf[1] / 2, f'$s_z$ = {sz:.3f} m', fontsize=8, va='center')
    fs.arrow(ax, Pf - [0, 0.08], Pf - [0, 0.012], color=C['est'], lw=1.4)    # pushes up on the foot from below, clear of the s vector
    ax.text(Pf[0] - 0.008, Pf[1] - 0.065, '$F$, $\\tau = g(q) - J^\\top F$', ha='right', color=C['est'], fontsize=8)
    ax.plot(*Pf, 'o', ms=6, color=C['ink']); fs.label(ax, Pf, 'foot centre', xytext=(-14, -12), ha='right', fontsize=7.5)
    fs.frame_axes(ax, (ox + 0.24, 0.03), length=0.05, names=('x', 'z'))
    fs.save(fig, FIG / 'leg_diagram')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=1000); ap.add_argument('--no-media', action='store_true')
    ap.add_argument('--json', default=str(HERE / 'results.json')); a = ap.parse_args(); t0 = time.time()
    res = dict(n_per_leg=a.n, load_average_start=os.getloadavg(), fundamental=fundamental(a.n)); print('intermediate ...')
    res['intermediate'] = intermediate(a.n); print('advanced ...')
    m = mujoco.MjModel.from_xml_path(require_model()); L = Leg(m, 'FL'); rng = np.random.default_rng(11)
    adv = dict(qp_random=qp_checks())
    tA, pA, qA = traj_boundary(L); adv['boundary'], runsA, dA = evaluate(L, tA, pA, qA)
    tB, pB, qB = traj_singular(L); adv['singular'], runsB, dB = evaluate(L, tB, pB, qB)
    # Adaptive damping on path B: the error bump after the target leaves the cylinder (largest error in the next 0.2 s), and the
    # ticks where the catch-up step hits a speed limit, with sigma_min at the configuration the damping was computed from.
    r, un = runsB['qp_adaptive'], dB > 0; leave = np.flatnonzero(un[:-1] & ~un[1:]) + 1
    sig_pre = np.concatenate([[np.linalg.svd(L.jacobian(qB), compute_uv=False)[-1]], r['sigma_min'][:-1]])
    sat = np.flatnonzero((np.abs(r['qd']) / QD_MAX).max(1) >= 1 - 1e-9)
    adv['singular']['methods']['qp_adaptive'].update(
        bump_after_leaving=[dict(t_leave_s=float(tB[i]), t_s=float(tB[k]), error_m=float(r['err'][k]))
                            for i in leave for k in [i + int(np.argmax(r['err'][i:i + 100]))]],
        speed_limit_ticks=dict(t_s=tB[sat].tolist(), sigma_min_before_step=sig_pre[sat].tolist(), all_reachable=bool((~un[sat]).all())))
    adv['adaptive_sweep'] = []                                                       # how ADAPT was chosen
    for eps, lmax in ((0.02, 0.05), (0.03, 0.05), (0.04, 0.08)):
        r = track(L, pB, qB, DT, 'qp', lam=LAM, adapt=(eps, lmax)); err = np.linalg.norm(r['p'] - pB, axis=1); un = dB > 0
        qd = np.abs(np.diff(np.vstack([qB, r['q']]), axis=0)) / DT
        adv['adaptive_sweep'].append(dict(eps=eps, lam_max=lmax, rms_error_reachable_m=float(np.sqrt(np.mean(err[~un] ** 2))),
                                          excess_error_max_m=float((err - dB).max()), mean_max_speed_unreachable_rad_s=float(qd[un].max(1).mean())))
    adv['qp_ticks_vs_lsq_linear_max'] = max(ticks_vs_lsq_linear(L, runsA['qp'], pA, qA), ticks_vs_lsq_linear(L, runsB['qp'], pB, qB))
    far = np.concatenate([pA[dA > 0][::25], pB[dB > 0][::25]])
    adv['workspace_distance_vs_scipy_max_m'] = distance_vs_scipy(L, far, rng); adv['workspace_distance_n_points'] = len(far)
    adv['reach_radius_m'] = float(reach_radius(L)); adv['fold_radius_m'] = float(fold_radius(L))
    # Folded-knee boundary: points 0.06 m from the thigh joint in the leg plane (|p - hip| = 0.1132 m < fold_radius), foot below
    # the thigh joint, for two hip angles; and two points above and behind it, where the thigh limits bind and delta is only a bound.
    def leg_plane(q1, phi, r=0.06):
        sx, sz = r * np.sin(phi), r * np.cos(phi); c, s = np.cos(q1), np.sin(q1)
        return L.hip + np.array([sx, c * L.d - s * sz, s * L.d + c * sz])
    below = [leg_plane(q1, np.deg2rad(ph)) for q1 in (0.0, 0.5) for ph in (90, 120, 150, 180, -150, -120, -90)]
    adv['workspace_distance_folded_vs_scipy_max_m'] = distance_vs_scipy(L, below, rng); adv['workspace_distance_folded_n_points'] = len(below)
    notch = [leg_plane(0.0, np.deg2rad(ph)) for ph in (-30, 0)]
    adv['workspace_distance_notch'] = [dict(phi_deg=ph, delta_m=float(distance_to_workspace(L, p)), scipy_m=float(scipy_distance(L, p, rng)))
                                       for ph, p in zip((-30, 0), notch)]
    print('figure-eight in MuJoCo ...')
    adv['figure_eight'], log8 = run_figure_eight(mp4=None if a.no_media else MEDIA / 'figure_eight.mp4', still=MEDIA / 'figure_eight.png')
    adv['figure_eight_gain_x2'] = run_figure_eight(kp=300.0, kd=8.0)[0]            # cause of the MuJoCo error: does it halve with the gains?
    res['advanced'] = adv
    fig_workspace(L, (tA, pA, runsA), (tB, pB, runsB)); adv['qp_step_figure'] = fig_qp_step(L, runsA, pA)
    fig_tracking('tracking_boundary', L, tA, {k: runsA[k] for k in ('dls', 'dls_clip', 'qp')}, dA,
                 'Path A: circle through the outer workspace boundary', speed_panel=False)
    fig_tracking('tracking_singular', L, tB, {k: runsB[k] for k in ('pinv', 'dls', 'dls_l05', 'dls_clip', 'qp', 'qp_adaptive')}, dB,
                 'Path B: lateral motion into the singular inner boundary', speed_panel=True, err_max=80,
                 bump=adv['singular']['methods']['qp_adaptive']['bump_after_leaving'][0])
    fig_figure_eight(log8); fig_leg_diagram(L)

    F, I = res['fundamental'], res['intermediate']
    flat = dict(
        fk_vs_mujoco_max_m=max(r['fk_vs_mujoco_max_m'] for r in F.values()), fk_vs_pinocchio_max_m=max(r['fk_vs_pinocchio_max_m'] for r in F.values()),
        jac_vs_finite_diff_max=max(r['jac_vs_finite_diff_max'] for r in F.values()), jac_vs_pinocchio_max=max(r['jac_vs_pinocchio_max'] for r in F.values()),
        ik_multistart_success_min=min(r['ik_success'] for r in F.values()),
        closed_form_success_min=min(r['closed_form_success'] for r in I['ik'].values()),
        closed_form_max_q_error_rad=max(r['closed_form_max_q_error_rad'] for r in I['ik'].values()),
        closed_form_max_fk_residual_m=max(r['closed_form_max_fk_residual_m'] for r in I['ik'].values()),
        det_formula_vs_numpy_max=I['singularity']['det_formula_vs_numpy_max'],
        closed_vs_numerical_agree_min=min(r['agree_to_1e9_fraction'] for r in I['ik'].values()),
        closed_vs_numerical_agree_nonsingular_min=min(r['agree_to_1e9_fraction_nonsingular'] for r in I['ik'].values()),
        statics_max_abs_residual_Nm=max(r['max_abs_residual'] for r in I['statics']['legs'].values()),
        qp_vs_lsq_linear_max=adv['qp_random']['qp_vs_lsq_linear_max'], qp_kkt_residual_max=adv['qp_random']['qp_kkt_residual_max'],
        qp_max_iterations=max(adv['qp_random']['qp_max_iterations_random'], adv['boundary']['methods']['qp']['qp_iterations_max'],
                              adv['singular']['methods']['qp']['qp_iterations_max']),
        qp_ticks_vs_lsq_linear_max=adv['qp_ticks_vs_lsq_linear_max'], workspace_distance_vs_scipy_max_m=adv['workspace_distance_vs_scipy_max_m'],
        workspace_distance_folded_vs_scipy_max_m=adv['workspace_distance_folded_vs_scipy_max_m'],
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
    res['wall_time_s'] = round(time.time() - t0, 1); res['load_average_end'] = os.getloadavg()
    pathlib.Path(a.json).write_text(json.dumps(res, indent=2, default=float) + '\n')
    print(f'wrote {a.json} in {res["wall_time_s"]} s')


if __name__ == '__main__':
    main()
