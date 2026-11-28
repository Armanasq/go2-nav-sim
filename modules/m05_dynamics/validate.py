#!/usr/bin/env python3
"""Experiment for module 05, all three levels.
  fundamental: RNEA, CRBA and ABA (dynamics.py) for one fixed-base leg against Pinocchio and MuJoCo; the Lagrangian two-link
      thigh-calf model against RNEA and CRBA; skew symmetry of M_dot - 2C.
  intermediate: the same algorithms for the floating-base robot (18 dof) against Pinocchio and MuJoCo; contact-point Jacobian
      and J_dot v against mj_jac / mj_jacDot; computed-torque control of the welded robot against PD + gravity; a free-flying
      robot integrated with the own ABA against MuJoCo's integrator.
  advanced: contact-constrained dynamics as a KKT system: standing on four feet and a squat, contact forces against MuJoCo's.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--n 500] [--no-media]"""
import argparse, json, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
from common import G, GO2_XML, JOINTS, LEGS, SCENE_XML, quat_to_R  # noqa: E402
from dynamics import (Tree, TwoLink, from_mj_acc, from_mj_force, from_mj_vel, to_mj_acc, to_mj_force,  # noqa: E402
                      to_mj_matrix, to_pin_q)

C = fs.C
FIG, MEDIA = HERE / 'figures', HERE / 'media'
STAND = np.array([0.0, 0.9, -1.8])
NO_CONSTRAINT = mujoco.mjtDisableBit.mjDSBL_CONSTRAINT

# Acceptance thresholds, fixed before the full run (MODULE_SPEC.md). key: (op, threshold). Units in the key.
ACCEPT = {
    # fundamental: one leg (FL), fixed base, random states
    'leg_rnea_vs_pinocchio_max_Nm': ('<', 1e-10), 'leg_rnea_vs_mujoco_max_Nm': ('<', 1e-10),
    'leg_crba_vs_pinocchio_max': ('<', 1e-12), 'leg_crba_vs_mujoco_max': ('<', 1e-12),
    'leg_aba_vs_pinocchio_max': ('<', 1e-9), 'leg_aba_vs_mujoco_max': ('<', 1e-9), 'leg_aba_vs_cholesky_max': ('<', 1e-9),
    'leg_M_min_eigenvalue': ('>', 0.0),
    'lagrange_tau_vs_rnea_max_Nm': ('<', 1e-12), 'lagrange_M_vs_crba_max': ('<', 1e-13),
    'lagrange_skew_max': ('<', 1e-7),
    # intermediate: floating base
    'float_rnea_vs_pinocchio_max': ('<', 1e-9), 'float_rnea_vs_mujoco_max': ('<', 1e-9),
    'float_crba_vs_pinocchio_max': ('<', 1e-12), 'float_crba_vs_mujoco_max': ('<', 1e-12),
    'float_aba_vs_pinocchio_max': ('<', 1e-8), 'float_aba_vs_mujoco_max': ('<', 1e-8), 'float_aba_vs_cholesky_max': ('<', 1e-8),
    'float_M_mass_block_err_kg': ('<', 1e-12),
    'foot_jac_vs_mujoco_max': ('<', 1e-12), 'foot_jacdot_v_vs_mujoco_max': ('<', 1e-10),
    'traj_torque_vs_pinocchio_max_Nm': ('<', 1e-10), 'traj_torque_vs_mujoco_max_Nm': ('<', 1e-10),
    'ct_rms_no_friction_rad': ('<', 1e-3),                 # missed: implicit damping + Euler discretisation (README)
    'ct_rms_friction_rad': ('<', 0.02), 'pdg_over_ct_rms_ratio_friction': ('>=', 3.0),
    'ct_error_order_in_dt': ('>=', 0.9),                   # post-hoc, added after the miss: the residual is first order in dt
    'flight_base_pos_dev_max_m': ('<', 1e-8), 'flight_joint_dev_max_rad': ('<', 1e-8),
    # advanced: contact KKT
    'kkt_vs_schur_max_N': ('<', 1e-9), 'stand_kkt_vs_mujoco_max_N': ('<', 0.1), 'stand_weight_ratio_err': ('<', 1e-3),
    'stand_kkt_acc_max': ('<', 0.1),
    'squat_force_rms_over_mg': ('<', 0.02), 'squat_force_max_over_mg': ('<', 0.1),
    'squat_condim3_eom_residual_max': ('<', 1e-9),         # post-hoc, added after finding the rolling-friction torque (README)
    'media_mp4_under_6MB': ('>=', 1.0),
}


def check(key, val):
    op, thr = ACCEPT[key]
    return bool({'<': val < thr, '<=': val <= thr, '>=': val >= thr, '>': val > thr}[op])


# ---- models for checking ---------------------------------------------------------------------------------------------------

def no_passive(m):
    """Rigid-body terms only: damping off, all constraints off (joint limits, friction loss, contacts)."""
    m.dof_damping[:] = 0; m.opt.disableflags |= NO_CONSTRAINT
    return m


def leg_models(leg='FL'):
    """One leg with the base welded at the origin: MuJoCo (MjSpec, other legs deleted) and Pinocchio (reduced model)."""
    s = mujoco.MjSpec.from_file(str(GO2_XML))
    for k in list(s.keys):
        s.delete(k)
    for a in list(s.actuators):
        if not a.name.startswith(leg):
            s.delete(a)
    for other in LEGS:
        if other != leg:
            s.delete(s.body(f'{other}_hip'))
    for j in list(s.body('base').joints):
        s.delete(j)
    s.body('base').pos = [0, 0, 0]
    m = no_passive(s.compile())
    pm = pin.buildModelFromMJCF(str(GO2_XML))
    lock = [pm.getJointId(n) for n in list(pm.names)[1:] if not n.startswith(leg)]
    return m, pin.buildReducedModel(pm, lock, pin.neutral(pm))


def welded_model(base_z=0.6, scene=True):
    """Go2 with the free joint removed, base welded at base_z (legs hang free above the floor)."""
    s = mujoco.MjSpec.from_file(str(SCENE_XML if scene else GO2_XML))
    for k in list(s.keys):
        s.delete(k)
    b = s.body('base')
    for j in list(b.joints):
        s.delete(j)
    b.pos = [0, 0, base_z]
    m = s.compile(); m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    return m


def pin_full(m=None):
    pm = pin.buildModelFromMJCF(str(GO2_XML))
    return pm


def sym(Mp):
    return np.triu(Mp) + np.triu(Mp, 1).T          # Pinocchio's crba fills the upper triangle


def mj_M(m, d):
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M)
    return M


def random_joints(m, rng, jnt0):
    lo, hi = m.jnt_range[jnt0:].T
    return rng.uniform(lo, hi)


# ---- fundamental -------------------------------------------------------------------------------------------------------------

def fundamental(n, rng):
    m, pl = leg_models('FL'); d = mujoco.MjData(m); pd = pl.createData(); T = Tree(m)
    e = {k: 0.0 for k in ('rnea_p', 'rnea_m', 'crba_p', 'crba_m', 'aba_p', 'aba_m', 'aba_c')}; mineig = np.inf
    for _ in range(n):
        q = random_joints(m, rng, 0); v = rng.normal(0, 2, 3); a = rng.normal(0, 5, 3); tau = rng.normal(0, 3, 3)
        t = T.rnea(q, v, a); M = T.crba(q); qdd = T.aba(q, v, tau)
        d.qpos[:], d.qvel[:], d.qacc[:] = q, v, a; mujoco.mj_inverse(m, d)
        e['rnea_p'] = max(e['rnea_p'], np.abs(t - pin.rnea(pl, pd, q, v, a)).max())
        e['rnea_m'] = max(e['rnea_m'], np.abs(t - d.qfrc_inverse).max())
        e['crba_p'] = max(e['crba_p'], np.abs(M - sym(pin.crba(pl, pd, q))).max())
        e['crba_m'] = max(e['crba_m'], np.abs(M - mj_M(m, d)).max())
        d.qfrc_applied[:] = tau; mujoco.mj_forward(m, d)
        e['aba_p'] = max(e['aba_p'], np.abs(qdd - pin.aba(pl, pd, q, v, tau)).max())
        e['aba_m'] = max(e['aba_m'], np.abs(qdd - d.qacc).max())
        e['aba_c'] = max(e['aba_c'], np.abs(qdd - T.fd(q, v, tau)).max())
        mineig = min(mineig, np.linalg.eigvalsh(M).min())
    res = dict(n_states=n, leg_rnea_vs_pinocchio_max_Nm=e['rnea_p'], leg_rnea_vs_mujoco_max_Nm=e['rnea_m'],
               leg_crba_vs_pinocchio_max=e['crba_p'], leg_crba_vs_mujoco_max=e['crba_m'], leg_aba_vs_pinocchio_max=e['aba_p'],
               leg_aba_vs_mujoco_max=e['aba_m'], leg_aba_vs_cholesky_max=e['aba_c'], leg_M_min_eigenvalue=float(mineig))
    # Lagrangian two-link: hip abduction held at zero, thigh and calf free; rows 1, 2 of the 3-dof leg
    L = TwoLink(m, 'FL'); et = eM = es = 0.0; h = 1e-6
    for _ in range(n):
        th = random_joints(m, rng, 1); dth = rng.normal(0, 2, 2); ddth = rng.normal(0, 5, 2)
        q, v, a = np.r_[0, th], np.r_[0, dth], np.r_[0, ddth]
        et = max(et, np.abs(L.tau(th, dth, ddth) - T.rnea(q, v, a)[1:]).max())
        eM = max(eM, np.abs(L.M(th) - T.crba(q)[1:, 1:]).max())
        Mdot = (L.M(th + h * dth) - L.M(th - h * dth)) / (2 * h); x = rng.normal(size=2)
        es = max(es, abs(x @ (Mdot - 2 * L.C(th, dth)) @ x))
    res.update(lagrange_tau_vs_rnea_max_Nm=et, lagrange_M_vs_crba_max=eM, lagrange_skew_max=es)
    # worked example: standing angles, hanging leg
    th = STAND[1:]
    res['worked'] = dict(l1_m=L.l1, m_thigh_kg=L.m1, m_calf_kg=L.m2, c_thigh_xz_m=L.c1.tolist(), c_calf_xz_m=L.c2.tolist(),
                         Iyy_thigh=L.I1, Iyy_calf=L.I2, armature=L.arm, M_stand=L.M(th).tolist(), g_stand_Nm=L.gvec(th).tolist(),
                         M_leg_stand=T.crba(STAND).tolist(), g_leg_stand_Nm=T.gravity_torque(STAND).tolist())
    return res


# ---- intermediate: floating base ---------------------------------------------------------------------------------------------

def random_float_state(m, rng):
    qpos = np.zeros(19); qpos[:3] = rng.normal(0, 1, 3); u = rng.normal(size=4); qpos[3:7] = u / np.linalg.norm(u)
    qpos[7:] = random_joints(m, rng, 1)
    return qpos, rng.normal(0, 1, 18) * np.r_[np.ones(6), 2 * np.ones(12)], rng.normal(0, 5, 18)


def intermediate_float(n, rng):
    m = no_passive(mujoco.MjModel.from_xml_path(str(GO2_XML))); d = mujoco.MjData(m); T = Tree(m)
    pm = pin_full(); pd = pm.createData(); mass = m.body_subtreemass[1]
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, l) for l in LEGS]
    e = {k: 0.0 for k in ('rnea_p', 'rnea_m', 'crba_p', 'crba_m', 'aba_p', 'aba_m', 'aba_c', 'mass', 'jac', 'jdot')}
    t0 = time.perf_counter(); timing = {'rnea': 0.0, 'crba': 0.0, 'aba': 0.0, 'fd': 0.0}
    for _ in range(n):
        qpos, qvel, qacc = random_float_state(m, rng)
        v, a = from_mj_vel(qpos, qvel), from_mj_acc(qpos, qvel, qacc); tau = rng.normal(0, 3, 18)
        s = time.perf_counter(); t = T.rnea(qpos, v, a); timing['rnea'] += time.perf_counter() - s
        s = time.perf_counter(); M = T.crba(qpos); timing['crba'] += time.perf_counter() - s
        s = time.perf_counter(); qdd = T.aba(qpos, v, tau); timing['aba'] += time.perf_counter() - s
        s = time.perf_counter(); qdd_c = T.fd(qpos, v, tau); timing['fd'] += time.perf_counter() - s
        qp = to_pin_q(qpos)
        e['rnea_p'] = max(e['rnea_p'], np.abs(t - pin.rnea(pm, pd, qp, v, a)).max())
        e['crba_p'] = max(e['crba_p'], np.abs(M - sym(pin.crba(pm, pd, qp))).max())
        e['aba_p'] = max(e['aba_p'], np.abs(qdd - pin.aba(pm, pd, qp, v, tau)).max())
        e['aba_c'] = max(e['aba_c'], np.abs(qdd - qdd_c).max())
        e['mass'] = max(e['mass'], np.abs(M[:3, :3] - mass * np.eye(3)).max())
        d.qpos[:], d.qvel[:], d.qacc[:] = qpos, qvel, qacc; mujoco.mj_inverse(m, d)
        e['rnea_m'] = max(e['rnea_m'], np.abs(to_mj_force(qpos, t) - d.qfrc_inverse).max())
        e['crba_m'] = max(e['crba_m'], np.abs(to_mj_matrix(qpos, M) - mj_M(m, d)).max())
        d.qfrc_applied[:] = to_mj_force(qpos, tau); mujoco.mj_forward(m, d)
        e['aba_m'] = max(e['aba_m'], np.abs(to_mj_acc(qpos, v, qdd) - d.qacc).max())
        D = np.eye(18); D[:3, :3] = quat_to_R(qpos[3:7]).T          # J_mj = J_native T^-1
        R = quat_to_R(qpos[3:7]); w_x_v = R @ np.cross(v[3:6], v[:3])
        for k, g in enumerate(feet):
            p = d.geom_xpos[g]; body = m.geom_bodyid[g]
            Jm, Jd = np.zeros((3, 18)), np.zeros((3, 18))
            mujoco.mj_jac(m, d, Jm, None, p, body); mujoco.mj_jacDot(m, d, Jd, None, p, body)
            e['jac'] = max(e['jac'], np.abs(T.point_jacobian(qpos, body - 1, p) @ D - Jm).max())
            # MuJoCo's J_dot multiplies its own qvel; with native accelerations the base term R (w x v_b) moves into the bias
            e['jdot'] = max(e['jdot'], np.abs(T.point_bias_acceleration(qpos, v, body - 1, p) - w_x_v - Jd @ qvel).max())
    res = dict(n_states=n, float_rnea_vs_pinocchio_max=e['rnea_p'], float_rnea_vs_mujoco_max=e['rnea_m'],
               float_crba_vs_pinocchio_max=e['crba_p'], float_crba_vs_mujoco_max=e['crba_m'],
               float_aba_vs_pinocchio_max=e['aba_p'], float_aba_vs_mujoco_max=e['aba_m'], float_aba_vs_cholesky_max=e['aba_c'],
               float_M_mass_block_err_kg=e['mass'], foot_jac_vs_mujoco_max=e['jac'], foot_jacdot_v_vs_mujoco_max=e['jdot'],
               timing_ms_per_call={k: 1e3 * v / n for k, v in timing.items()}, total_mass_kg=float(mass),
               wall_s=time.perf_counter() - t0)
    qs = np.r_[0, 0, 0.27, 1, 0, 0, 0, np.tile(STAND, 4)]
    res['M_stand'] = T.crba(qs).tolist()
    return res


# ---- intermediate: torque along a trajectory and computed-torque control -----------------------------------------------------

AMP = np.array([0.25, 0.5, 0.6])           # rad, hip / thigh / calf swing amplitude
FREQ = 1.5                                 # Hz
PHASE = dict(FL=0.0, FR=np.pi, RL=np.pi, RR=0.0)


def q_des(t, ramp=1.0):
    s = np.clip(t / ramp, 0, 1); env = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return np.concatenate([STAND + env * AMP * np.sin(2 * np.pi * FREQ * t + PHASE[l] + np.array([0, 0.5, 1.0])) for l in LEGS])


def q_des_derivs(t, h=1e-4):
    qm, q0, qp = q_des(t - h), q_des(t), q_des(t + h)
    return q0, (qp - qm) / (2 * h), (qp - 2 * q0 + qm) / h ** 2


def traj_torques(duration=4.0, dt=0.01):
    """Inverse dynamics of the desired swing on the welded robot: own RNEA, Pinocchio, MuJoCo; plus the M a / C v / g split."""
    m = no_passive(welded_model(scene=False)); d = mujoco.MjData(m); T = Tree(m)
    pm = pin_full(); pl = pin.buildReducedModel(pm, [pm.getJointId('base_free')], pin.neutral(pm)); pd = pl.createData()
    rows = []
    for t in np.arange(0, duration, dt):
        q, v, a = q_des_derivs(t)
        tau = T.rnea(q, v, a); g = T.gravity_torque(q); hv = T.bias(q, v) - g
        d.qpos[:], d.qvel[:], d.qacc[:] = q, v, a; mujoco.mj_inverse(m, d)
        rows.append(np.concatenate([[t], tau, pin.rnea(pl, pd, q, v, a), d.qfrc_inverse, tau - hv - g, hv, g]))
    A = np.array(rows)
    return A, dict(traj_torque_vs_pinocchio_max_Nm=float(np.abs(A[:, 1:13] - A[:, 13:25]).max()),
                   traj_torque_vs_mujoco_max_Nm=float(np.abs(A[:, 1:13] - A[:, 25:37]).max()),
                   traj_torque_abs_max_Nm=float(np.abs(A[:, 1:13]).max()))


def computed_torque(mode, friction, duration=4.0, kp=400.0, kd=40.0, mp4=None, still=None, timestep=None):
    """Welded robot tracks q_des. mode 'ct': tau = RNEA(q, dq, a_c) + b dq with a_c = a_d + kp e + kd de.
    mode 'ct_implicit': adds dt b a_c, which cancels the implicit damping of MuJoCo's Euler step (M + dt B) a = f (README).
    mode 'pdg': tau = diag(M(q_stand)) (kp e + kd de) + g(q) + b dq, the same stiffness per joint without the model's coupling."""
    m = welded_model(); d = mujoco.MjData(m); idx = sim.index(m); T = Tree(m)
    if not friction:
        m.dof_frictionloss[:] = 0
    if timestep:
        m.opt.timestep = timestep
    dt = m.opt.timestep
    b = m.dof_damping[idx.qvel]; Md = np.diag(T.crba(np.tile(STAND, 4)))
    d.qpos[idx.qpos] = q_des(0); mujoco.mj_forward(m, d); log = []

    def ctrl(m, d):
        q, dq = d.qpos[idx.qpos], d.qvel[idx.qvel]; qd, vd, ad = q_des_derivs(d.time)
        e, de = qd - q, vd - dq
        if mode in ('ct', 'ct_implicit'):
            ac = ad + kp * e + kd * de
            return T.rnea(q, dq, ac) + b * dq + (dt * b * ac if mode == 'ct_implicit' else 0)
        return Md * (kp * e + kd * de) + T.gravity_torque(q) + b * dq

    def on_step(m, d):
        log.append(np.concatenate([[d.time], q_des(d.time) - d.qpos[idx.qpos], d.actuator_force[idx.act]]))
        if rec is not None:
            rec(m, d)
    rec = sim.Recorder(m, mp4, distance=1.5, azimuth=120, elevation=-12) if mp4 else None
    if rec is None:
        sim.run(m, d, ctrl, duration, on_step=on_step)
    else:
        with rec:
            sim.run(m, d, ctrl, duration, on_step=on_step); rec.close()
            if still:
                rec.still(d, still)
    A = np.array(log); after = A[:, 0] > 1.0; err = A[after, 1:13]
    return A, dict(rms_rad=float(np.sqrt(np.mean(err ** 2))), max_rad=float(np.abs(err).max()),
                   torque_max_Nm=float(np.abs(A[:, 13:]).max()))


# ---- intermediate: free flight, own ABA against MuJoCo's integrator ----------------------------------------------------------

def quat_integrate(qu, w, dt):
    """MuJoCo's mju_quatIntegrate: q <- q * exp(w dt / 2), w in the body frame."""
    ang = np.linalg.norm(w) * dt
    if ang < 1e-15:
        return qu
    ax = w / np.linalg.norm(w); dq = np.r_[np.cos(ang / 2), np.sin(ang / 2) * ax]
    w0, x0, y0, z0 = qu; w1, x1, y1, z1 = dq
    out = np.array([w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1, w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
                    w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1, w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1])
    return out / np.linalg.norm(out)


def flight(duration=3.0, mp4=None, still=None):
    """Zero gravity, no constraints, joint PD to a swinging target. MuJoCo mj_step (semi-implicit Euler) against the own ABA
    with the same integrator. Both start from the same state; the PD torque is computed from each simulation's own state."""
    m, dm = sim.load(); no_passive(m); m.opt.gravity[:] = 0; idx = sim.index(m); T = Tree(m)
    target = lambda t: np.concatenate([STAND + 0.6 * np.sin(2 * np.pi * 0.8 * t + k + np.array([0, 0.7, 1.4])) * [0.5, 1, 1]  # noqa: E731
                                       for k, l in enumerate(LEGS)])
    pd_tau = lambda t, q, dq: 20.0 * (target(t) - q) - 0.5 * dq  # noqa: E731
    mujoco.mj_resetData(m, dm); dm.qpos[:7] = [0, 0, 0.6, 1, 0, 0, 0]; dm.qpos[7:] = np.tile(STAND, 4); mujoco.mj_forward(m, dm)
    qpos, qvel = dm.qpos.copy(), dm.qvel.copy(); dt = m.opt.timestep
    dr = mujoco.MjData(m)                                   # render data for the own simulation
    rec = sim.Recorder(m, mp4, distance=1.4, azimuth=100, elevation=-10) if mp4 else None
    log = []
    for k in range(round(duration / dt)):
        t = k * dt
        dm.qfrc_applied[6:] = pd_tau(t, dm.qpos[7:], dm.qvel[6:]); mujoco.mj_step(m, dm)
        tau = np.r_[np.zeros(6), pd_tau(t, qpos[7:], qvel[6:])]
        v = from_mj_vel(qpos, qvel); qacc = to_mj_acc(qpos, v, T.aba(qpos, v, tau))
        qvel = qvel + dt * qacc
        qpos = qpos.copy(); qpos[:3] += dt * qvel[:3]; qpos[3:7] = quat_integrate(qpos[3:7], qvel[3:6], dt); qpos[7:] += dt * qvel[6:]
        Rm, Ro = quat_to_R(dm.qpos[3:7]), quat_to_R(qpos[3:7])
        ang = np.arccos(np.clip((np.trace(Rm.T @ Ro) - 1) / 2, -1, 1))
        log.append([t + dt, np.linalg.norm(dm.qpos[:3] - qpos[:3]), ang, np.abs(dm.qpos[7:] - qpos[7:]).max(),
                    np.degrees(2 * np.arccos(np.clip(abs(qpos[3]), 0, 1)))])
        if rec is not None and rec.ok:
            dr.qpos[:] = qpos; dr.time = t + dt; mujoco.mj_forward(m, dr); rec(m, dr)
    if rec is not None:
        with rec:
            rec.close()
            if still:
                rec.still(dr, still)
    A = np.array(log)
    return A, dict(flight_base_pos_dev_max_m=float(A[:, 1].max()), flight_base_rot_dev_max_rad=float(A[:, 2].max()),
                   flight_joint_dev_max_rad=float(A[:, 3].max()), flight_base_rotation_final_deg=float(A[-1, 4]))


# ---- advanced: contact-constrained dynamics ----------------------------------------------------------------------------------

def contact_points(m, d, idx):
    """[(tree body of the calf, contact position in world)] for every foot in contact, LEGS order; and the leg indices."""
    pts, legs = [], []
    for k, g in enumerate(idx.feet):
        for i in range(d.ncon):
            c = d.contact[i]
            if g in (c.geom1, c.geom2):
                pts.append((m.geom_bodyid[g] - 1, c.pos.copy())); legs.append(k); break
    return pts, legs


def kkt_at(m, d, T, idx):
    """KKT contact forces and accelerations at MuJoCo's current state, with the torques MuJoCo applies (actuators and damping)."""
    qpos, qvel = d.qpos.copy(), d.qvel.copy(); v = from_mj_vel(qpos, qvel)
    tau = np.zeros(18); tau[idx.qvel] = d.actuator_force[idx.act]; tau[6:] -= m.dof_damping[6:] * qvel[6:]
    pts, legs = contact_points(m, d, idx)
    a, lam, Jc = T.contact_kkt(qpos, v, tau, pts)
    F = np.zeros((4, 3)); F[legs] = lam
    return a, F, (qpos, v, tau, pts)


def standing(frictionloss, settle=2.0):
    m, d = sim.load(); idx = sim.index(m); T = Tree(m)
    if not frictionloss:
        m.dof_frictionloss[:] = 0
    sim.stand(m, d, settle=settle)
    mujoco.mj_forward(m, d)
    a, F, (qpos, v, tau, pts) = kkt_at(m, d, T, idx)
    Fm = sim.foot_forces(m, d); mg = m.body_subtreemass[1] * G
    a2, lam2 = T.contact_schur(qpos, v, tau, pts)
    mu = m.geom_friction[idx.feet[0], 0]
    out = dict(kkt_vs_mujoco_max_N=float(np.abs(F - Fm).max()), weight_ratio=float(F[:, 2].sum() / mg),
               kkt_acc_max=float(np.abs(a).max()), mujoco_qacc_max=float(np.abs(from_mj_acc(qpos, d.qvel, d.qacc)).max()),
               kkt_vs_schur_max_N=float(max(np.abs(lam2.ravel() - F[F[:, 2] != 0].ravel()).max(), np.abs(a2 - a).max())),
               tangential_over_normal_max=float((np.linalg.norm(F[:, :2], axis=1) / F[:, 2]).max()), mu=float(mu),
               F_kkt_N=F.tolist(), F_mujoco_N=Fm.tolist(), n_contacts=len(pts),
               joint_friction_force_max_Nm=float(np.abs(d.qfrc_constraint[6:] - contact_joint_force(T, qpos, pts, Fm)).max()))
    return out


def contact_joint_force(T, qpos, pts, Fm):
    """Joint rows of J_c^T F for MuJoCo's own contact forces: subtracting it from qfrc_constraint leaves friction loss."""
    Jc = np.vstack([T.point_jacobian(qpos, b, p) for b, p in pts]); legs = [k for k in range(4) if Fm[k, 2] > 0]
    return (Jc.T @ Fm[legs].ravel())[6:]


def squat(duration=4.0, mp4=None, still=None, condim=None):
    """Frictionloss off. Joint PD tracks a squat (thigh +0.35, calf -0.7 rad at 0.75 Hz); at every step the KKT forces are
    computed at MuJoCo's state with MuJoCo's torques and compared with MuJoCo's contact forces. Also logged: the residual of
    the equations of motion with MuJoCo's own accelerations and foot forces, |M a + h - tau - Jc^T F_mujoco|, which is zero only
    if point forces are all MuJoCo applies at the feet. condim: override the feet's contact dimensionality (MJCF: 6)."""
    m, d = sim.load(); idx = sim.index(m); T = Tree(m); m.dof_frictionloss[:] = 0
    if condim:
        m.geom_condim[idx.feet] = condim
    sim.stand(m, d, settle=1.0); t0 = d.time; mg = m.body_subtreemass[1] * G
    dq = np.tile([0.0, 0.35, -0.7], 4)
    qref = lambda t: sim.Q_STAND + dq * 0.5 * (1 - np.cos(2 * np.pi * 0.75 * (t - t0)))  # noqa: E731
    calf = [idx.legs[l]['calf'] for l in LEGS]; log = []

    def on_step(m, d):
        mujoco.mj_forward(m, d)                        # contact forces and actuator forces at the state after the step
        a, F, (qpos, v, tau, pts) = kkt_at(m, d, T, idx); Fm = sim.foot_forces(m, d)
        wc = np.array([np.linalg.norm(d.cvel[b][:3]) for b in calf])
        Jc = np.vstack([T.point_jacobian(qpos, b, p) for b, p in pts])
        eom = T.crba(qpos) @ from_mj_acc(qpos, d.qvel, d.qacc) + T.bias(qpos, v) - tau - Jc.T @ Fm[Fm[:, 2] != 0].ravel()
        log.append(np.concatenate([[d.time - t0, d.qpos[2]], F.ravel(), Fm.ravel(), wc, [np.abs(eom).max()]]))
        if rec is not None:
            rec(m, d)
    rec = sim.Recorder(m, mp4, contacts=True, distance=1.2, azimuth=150, elevation=-12) if mp4 else None
    if rec is not None:
        rec.n = int(np.ceil(t0 * rec.fps))           # Recorder counts frames from sim time zero
    if rec is None:
        sim.run(m, d, sim.joint_pd(m, qref, kp=80, kd=3), duration, on_step=on_step)
    else:
        with rec:
            sim.run(m, d, sim.joint_pd(m, qref, kp=80, kd=3), duration, on_step=on_step); rec.close()
            if still:
                rec.still(d, still)
    A = np.array(log); Fk, Fm = A[:, 2:14].reshape(-1, 4, 3), A[:, 14:26].reshape(-1, 4, 3)
    err = np.linalg.norm(Fk - Fm, axis=2)                # per foot, N
    tot = np.linalg.norm((Fk - Fm).sum(1), axis=1)        # net force difference
    return A, dict(squat_force_rms_over_mg=float(np.sqrt(np.mean(err ** 2)) / mg), squat_force_max_over_mg=float(err.max() / mg),
                   squat_force_rms_N=float(np.sqrt(np.mean(err ** 2))), squat_force_max_N=float(err.max()),
                   squat_net_force_rms_N=float(np.sqrt(np.mean(tot ** 2))), squat_base_z_range_m=[float(A[:, 1].min()), float(A[:, 1].max())],
                   squat_calf_rate_max=float(A[:, 26:30].max()), squat_eom_residual_max=float(A[:, 30].max()),
                   squat_rolling_normal_acc_bound=float(A[:, 26:30].max() ** 2 * sim.FOOT_RADIUS))


# ---- figures ---------------------------------------------------------------------------------------------------------------

def fig_two_link(L, foot_xz, th=(0.9, -1.8)):
    """Sagittal-plane diagram of the thigh-calf chain with the Lagrangian's symbols."""
    R = lambda a: np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])  # noqa: E731  rotation about +y in (x, z)
    t1, t12 = th[0], th[0] + th[1]
    knee = R(t1) @ [0, -L.l1]; c1 = R(t1) @ L.c1; c2 = knee + R(t12) @ L.c2
    foot = knee + R(t12) @ foot_xz
    fig, ax = fs.diagram(6.4, 4.0, (-0.25, 0.27), (-0.33, 0.06))
    ax.plot([0, knee[0], foot[0]], [0, knee[1], foot[1]], color=C['ink'], lw=3, solid_capstyle='round')
    ax.plot([0, 0], [0, -0.12], ':', color=C['muted'], lw=0.8); ax.plot([knee[0], knee[0] - 0.09 * np.sin(t1)], [knee[1], knee[1] - 0.09 * np.cos(t1)], ':', color=C['muted'], lw=0.8)
    for p, name in ((c1, r'$m_1, I_1$'), (c2, r'$m_2, I_2$')):
        ax.plot(*p, 'o', ms=7, mfc=C['paper'], mec=C['est'], mew=1.5); fs.label(ax, p, name, xytext=(14, 4), color=C['est'])
    ax.plot(0, 0, 'o', ms=6, color=C['ink']); ax.plot(*knee, 'o', ms=6, color=C['ink'])
    fs.label(ax, (0, 0), r'thigh joint, $\theta_1$', xytext=(14, 10)); fs.label(ax, knee, r'knee, $\theta_2$', xytext=(-70, -6))
    fs.label(ax, (knee / 2), r'$l_1$ = %.3f m' % L.l1, xytext=(-80, 0), color=C['muted'])
    fs.frame_axes(ax, (-0.21, -0.03), length=0.05, names=('x', 'z'))
    fs.label(ax, (-0.21, -0.045), 'x forward, z up (base frame)', color=C['muted'], fontsize=7.5)
    ax.plot(*foot, 'o', ms=4, color=C['ink']); fs.label(ax, foot, 'foot', xytext=(10, -2))
    fs.label(ax, (0.005, -0.10), r'$\theta_1$ from $-z$', color=C['muted'], fontsize=7.5)
    fs.label(ax, knee + [-0.075, -0.07], r'$\theta_2$ from the thigh line', color=C['muted'], fontsize=7.5, ha='right')
    fs.arrow(ax, (0.2, 0.0), (0.2, -0.07), r'$g$', color=C['muted'], offset=(8, 0), ha='left')
    fs.label(ax, (0.06, -0.24), '$c_1, c_2$: COM in each link frame (x, z)\nrotations about $+y$ (into the page)', color=C['muted'], fontsize=8)
    ax.set_title('Planar thigh-calf chain used for the Lagrangian (FL leg, standing angles)')
    fs.save(fig, FIG / 'two_link_diagram')


def fig_mass_matrix(M, Mleg):
    M = np.asarray(M); fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.4, 3.6), gridspec_kw=dict(width_ratios=[1.6, 1]))
    Z = np.log10(np.abs(M) + 1e-16); Z[np.abs(M) < 1e-12] = np.nan
    im = a1.imshow(Z, cmap='Blues', vmin=-6, vmax=np.log10(np.abs(M).max()))
    names = ['vx', 'vy', 'vz', 'ωx', 'ωy', 'ωz'] + [f'{l} {j[0]}' for l in LEGS for j in JOINTS]
    a1.set_xticks(range(18)); a1.set_xticklabels(names, rotation=90, fontsize=6.5); a1.set_yticks(range(18)); a1.set_yticklabels(names, fontsize=6.5)
    for k in (5.5, 8.5, 11.5, 14.5):
        a1.axhline(k, color=C['muted'], lw=0.5); a1.axvline(k, color=C['muted'], lw=0.5)
    a1.grid(False); fig.colorbar(im, ax=a1, shrink=0.8, label=r'$\log_{10}|M_{ij}|$ (empty: exactly 0)')
    a1.set_title('Floating-base M(q), standing pose')
    Ml = np.asarray(Mleg); a2.imshow(Ml, cmap='Blues'); a2.grid(False)
    for i in range(3):
        for j in range(3):
            a2.text(j, i, f'{Ml[i, j]:.4f}', ha='center', va='center', fontsize=8, color=C['ink'] if Ml[i, j] < 0.6 * Ml.max() else C['paper'])
    a2.set_xticks(range(3)); a2.set_xticklabels(JOINTS); a2.set_yticks(range(3)); a2.set_yticklabels(JOINTS)
    a2.set_title('FL leg, fixed base [kg m²]')
    fs.save(fig, FIG / 'mass_matrix')


def fig_torques(A):
    t = A[:, 0]; fig, ax = plt.subplots(3, 1, figsize=(6.4, 6.4), sharex=True)
    for j, name in enumerate(JOINTS):
        ax[0].plot(t, A[:, 1 + j], color=[C['est'], C['est2'], C['plan']][j], label=f'FL {name}, own RNEA')
        ax[0].plot(t[::6], A[::6, 13 + j], 'o', ms=3, mfc='none', color=C['gt'], label='Pinocchio rnea' if j == 0 else None)
        ax[0].plot(t[3::6], A[3::6, 25 + j], 'x', ms=3, color=C['ink'], label='MuJoCo mj_inverse' if j == 0 else None)
    ax[0].set(ylabel='torque [N m]', title='Inverse dynamics of the swing trajectory (welded base)', ylim=(-1.7, 3.4)); ax[0].legend(ncol=3, fontsize=7.2, loc='upper left')
    k = 2
    ax[1].plot(t, A[:, 37 + k], color=C['est'], label=r'$M(q)\ddot q$'); ax[1].plot(t, A[:, 49 + k], color=C['est2'], label=r'$C(q,\dot q)\dot q$')
    ax[1].plot(t, A[:, 61 + k], color=C['plan'], label=r'$g(q)$'); ax[1].plot(t, A[:, 1 + k], color=C['ink'], lw=0.8, label=r'$\tau$')
    ax[1].set(ylabel='FL calf [N m]', title='Split of the FL calf torque'); ax[1].legend(ncol=4, fontsize=7.5)
    ax[2].semilogy(t, np.abs(A[:, 1:13] - A[:, 13:25]).max(1) + 1e-18, color=C['gt'], label='|own − Pinocchio|, max over 12 joints')
    ax[2].semilogy(t, np.abs(A[:, 1:13] - A[:, 25:37]).max(1) + 1e-18, color=C['ink'], lw=0.8, label='|own − MuJoCo|')
    ax[2].axhline(ACCEPT['traj_torque_vs_mujoco_max_Nm'][1], ls='--', color=C['bad'], lw=0.8, label='acceptance')
    ax[2].set(xlabel='time [s]', ylabel='difference [N m]', ylim=(1e-17, 1e-9)); ax[2].legend(fontsize=7.5)
    fs.save(fig, FIG / 'torque_comparison')


def fig_ct(runs, sweep):
    fig = plt.figure(figsize=(6.8, 6.2)); gs = fig.add_gridspec(2, 2, height_ratios=[1.3, 1], width_ratios=[1.6, 1])
    a0, a1, a2 = fig.add_subplot(gs[0, :]), fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])
    style = dict(ct=(C['est'], 'computed torque'), ct_implicit=(C['est2'], 'computed torque + dt·B·a'), pdg=(C['muted'], 'PD + gravity'))
    for (mode, fr), (A, r) in runs.items():
        col, name = style[mode]
        a0.semilogy(A[:, 0], np.abs(A[:, 1:13]).max(1) + 1e-9, color=col, ls='-' if fr else ':', lw=1.0,
                    label=f'{name}, friction loss {"on" if fr else "off"} (RMS {r["rms_rad"]:.2g} rad)')
    a0.axhline(ACCEPT['ct_rms_no_friction_rad'][1], ls='--', color=C['bad'], lw=0.8, label='RMS acceptance, friction off')
    a0.set(xlabel='time [s]', ylabel='max joint error [rad]', ylim=(1e-5, 0.5),
           title='Tracking the swing on the welded robot, kp = 400 s⁻², kd = 40 s⁻¹'); a0.legend(fontsize=6.8, ncol=2)
    A = runs[('ct', True)][0]
    for j, name in enumerate(JOINTS):
        a1.plot(A[:, 0], A[:, 13 + j], color=[C['est'], C['est2'], C['plan']][j], lw=0.9, label=f'FL {name}')
    a1.set(xlabel='time [s]', ylabel='torque [N m]', title='Command, computed torque'); a1.legend(ncol=3, fontsize=7)
    dts = np.array([float(k) for k in sweep]); rms = np.array(list(sweep.values()))
    a2.loglog(dts * 1e3, rms, 'o-', color=C['est2'], label='measured, friction off')
    a2.loglog(dts * 1e3, rms[0] * dts / dts[0], '--', color=C['muted'], lw=0.8, label='slope 1')
    a2.set(xlabel='timestep [ms]', ylabel='RMS error [rad]', title='Residual against the step'); a2.legend(fontsize=7)
    fs.save(fig, FIG / 'computed_torque')


def fig_flight(A):
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    ax.semilogy(A[:, 0], A[:, 1] + 1e-18, color=C['est'], label='base position [m]')
    ax.semilogy(A[:, 0], A[:, 2] + 1e-18, color=C['est2'], label='base orientation [rad]')
    ax.semilogy(A[:, 0], A[:, 3] + 1e-18, color=C['plan'], label='joints, max [rad]')
    ax.axhline(ACCEPT['flight_base_pos_dev_max_m'][1], ls='--', color=C['bad'], lw=0.8, label='acceptance')
    ax.set(xlabel='time [s]', ylabel='own ABA − MuJoCo', title='Free flight in zero gravity: own forward dynamics against mj_step'); ax.legend(fontsize=7.5)
    fs.save(fig, FIG / 'flight_deviation')


def fig_standing(off, on):
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 3.2), sharey=True)
    for a, r, title in ((ax[0], off, 'friction loss off'), (ax[1], on, 'friction loss on (MJCF default, 0.2 N m)')):
        Fk, Fm = np.array(r['F_kkt_N']), np.array(r['F_mujoco_N']); x = np.arange(4)
        a.bar(x - 0.18, Fm[:, 2], 0.36, color=C['gt'], label='MuJoCo contact force')
        a.bar(x + 0.18, Fk[:, 2], 0.36, color=C['est'], label='KKT')
        for k in range(4):
            e = np.abs(Fk[k] - Fm[k]).max()
            a.text(x[k], max(Fm[k, 2], Fk[k, 2]) + 1, f'{e:.3f}', ha='center', fontsize=7.5,
                   color=C['good'] if e < ACCEPT['stand_kkt_vs_mujoco_max_N'][1] else C['bad'])
        a.set_xticks(x); a.set_xticklabels(LEGS); a.set_title(title); a.grid(axis='x', visible=False)
    ax[0].set_ylabel('vertical foot force [N]'); ax[0].legend(fontsize=7.5, loc='lower left')
    fig.suptitle('Standing: KKT contact forces from the applied torques (labels: max component |KKT − MuJoCo|, N)', fontsize=10)
    fs.save(fig, FIG / 'standing_forces')


def fig_squat(A, A3, mg):
    t = A[:, 0]; Fk, Fm = A[:, 2:14].reshape(-1, 4, 3), A[:, 14:26].reshape(-1, 4, 3)
    fig, ax = plt.subplots(3, 1, figsize=(6.4, 6.4), sharex=True)
    for k, l in enumerate(LEGS):
        ax[0].plot(t, Fm[:, k, 2], color=C['gt'], lw=2.2, alpha=0.6, label='MuJoCo' if k == 0 else None)
        ax[0].plot(t, Fk[:, k, 2], color=fs.LEGS[l], lw=0.9, label=f'KKT {l}')
    ax[0].set(ylabel='vertical force [N]', title='Squat at 0.75 Hz: contact forces, KKT against MuJoCo (feet as in the MJCF)')
    ax[0].legend(ncol=5, fontsize=7.2, loc='upper center', bbox_to_anchor=(0.5, -0.04)); ax[0].set_ylim(top=Fm[:, :, 2].max() + 3)
    e6 = np.linalg.norm(Fk - Fm, axis=2).max(1)
    e3 = np.linalg.norm(A3[:, 2:14].reshape(-1, 4, 3) - A3[:, 14:26].reshape(-1, 4, 3), axis=2).max(1)
    ax[1].semilogy(t, e6, color=C['bad'], lw=0.9, label='feet condim 6 (MJCF: rolling and torsional friction)')
    ax[1].semilogy(A3[:, 0], e3, color=C['est'], lw=0.9, label='feet condim 3 (point forces only)')
    ax[1].axhline(ACCEPT['squat_force_max_over_mg'][1] * mg, ls='--', color=C['muted'], lw=0.8, label='max acceptance (0.1 m g)')
    ax[1].set(ylabel='max foot |KKT − MuJoCo| [N]', ylim=(1e-3, 50)); ax[1].legend(fontsize=7.2, loc='lower right')
    ax[2].semilogy(t, A[:, 30], color=C['bad'], lw=0.9, label='condim 6'); ax[2].semilogy(A3[:, 0], A3[:, 30] + 1e-16, color=C['est'], lw=0.9, label='condim 3')
    ax[2].set(xlabel='time [s]', ylabel='EOM residual [N, N m]', title=r'$|M a + h - \tau - J_c^\top F|$ with MuJoCo\'s own a and F')
    ax[2].legend(fontsize=7.2, ncol=2)
    fs.save(fig, FIG / 'squat_forces')


# ---- main ------------------------------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=500); ap.add_argument('--no-media', action='store_true')
    args = ap.parse_args(); rng = np.random.default_rng(0); media = not args.no_media
    vid = lambda name: (MEDIA / f'{name}.mp4', MEDIA / f'{name}.png') if media else (None, None)  # noqa: E731
    t0 = time.time(); R = {}
    R['fundamental'] = fundamental(args.n, rng)
    R['intermediate'] = intermediate_float(args.n, rng)
    A_traj, r = traj_torques(); R['intermediate'].update(r)
    runs = {}
    for mode in ('ct', 'ct_implicit', 'pdg'):
        for fr in (False, True):
            mp4, png = vid('computed_torque') if (mode, fr) == ('ct', True) else (None, None)
            runs[(mode, fr)] = computed_torque(mode, fr, mp4=mp4, still=png)
    sweep = {dt: computed_torque('ct_implicit', False, timestep=dt)[1]['rms_rad'] for dt in (0.002, 0.001, 0.0005)}
    dts = np.array(list(sweep)); order = float(np.polyfit(np.log(dts), np.log(list(sweep.values())), 1)[0])
    R['intermediate']['ct_timestep_sweep_rms_rad'] = {str(k): v for k, v in sweep.items()}
    R['intermediate'].update(ct_implicit_rms_no_friction_rad=sweep[0.002], ct_error_order_in_dt=order)
    R['intermediate']['tracking'] = {f'{k[0]}_{"friction" if k[1] else "no_friction"}': v[1] for k, v in runs.items()}
    R['intermediate'].update(ct_rms_no_friction_rad=runs[('ct', False)][1]['rms_rad'], ct_rms_friction_rad=runs[('ct', True)][1]['rms_rad'],
                             pdg_over_ct_rms_ratio_friction=runs[('pdg', True)][1]['rms_rad'] / runs[('ct', True)][1]['rms_rad'])
    A_fl, r = flight(mp4=vid('flight')[0], still=vid('flight')[1]); R['intermediate'].update(r)
    off, on = standing(False), standing(True)
    A_sq, r = squat(mp4=vid('squat')[0], still=vid('squat')[1])
    A_sq3, r3 = squat(condim=3)
    r.update({k.replace('squat_', 'squat_condim3_'): v for k, v in r3.items()})
    R['advanced'] = dict(stand_frictionloss_off=off, stand_frictionloss_on=on, kkt_vs_schur_max_N=off['kkt_vs_schur_max_N'],
                         stand_kkt_vs_mujoco_max_N=off['kkt_vs_mujoco_max_N'], stand_weight_ratio_err=abs(off['weight_ratio'] - 1),
                         stand_kkt_acc_max=off['kkt_acc_max'], stand_frictionloss_on_kkt_vs_mujoco_max_N=on['kkt_vs_mujoco_max_N'], **r)
    sizes = {p.name: p.stat().st_size for p in MEDIA.glob('*.mp4')} if media else {}
    R['media_bytes'] = sizes
    flat = {**R['fundamental'], **R['intermediate'], **R['advanced'],
            'media_mp4_under_6MB': float(all(s < 6e6 for s in sizes.values()) if sizes else 1.0)}
    R['checks'] = {k: dict(value=flat[k], op=op, threshold=thr, passed=check(k, flat[k])) for k, (op, thr) in ACCEPT.items()}
    R['passed'] = sum(c['passed'] for c in R['checks'].values()); R['total'] = len(ACCEPT); R['wall_s'] = time.time() - t0
    (HERE / 'results.json').write_text(json.dumps(R, indent=1, default=float))
    mg = R['intermediate']['total_mass_kg'] * G
    mf = mujoco.MjModel.from_xml_path(str(GO2_XML))
    fig_two_link(TwoLink(mf), mf.geom_pos[mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_GEOM, 'FL')][[0, 2]])
    fig_mass_matrix(R['intermediate']['M_stand'], R['fundamental']['worked']['M_leg_stand'])
    fig_torques(A_traj); fig_ct(runs, R['intermediate']['ct_timestep_sweep_rms_rad']); fig_flight(A_fl); fig_standing(off, on); fig_squat(A_sq, A_sq3, mg)
    for k, c in R['checks'].items():
        print(f'{"PASS" if c["passed"] else "MISS"}  {k:42s} {c["value"]:.4g}  ({c["op"]} {c["threshold"]:g})')
    print(f'{R["passed"]} / {R["total"]} checks pass, {R["wall_s"]:.0f} s')


if __name__ == '__main__':
    main()
