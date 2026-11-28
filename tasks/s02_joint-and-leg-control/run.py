#!/usr/bin/env python3
"""Stage 2 experiments on the supported Go2 (trunk fixed in the air, lib.load_supported).
usage: python run.py <task-id>|all      e.g. python run.py 2.05
Writes results/<id>.json (measured values, thresholds from acceptance.json, pass/fail), figures/ and, for 2.11 and 2.14, media/.
Wrap runs that render (2.11, 2.14, all) in `flock /tmp/go2_render.lock`."""
import json
import sys
from typing import Any

import numpy as np

import lib
from lib import DT, LEGS, Q_HOLD
import figstyle as fs                                   # importable after lib put modules/ on sys.path
import matplotlib.pyplot as plt
import mujoco

HERE = lib.HERE
ACC = json.loads((HERE / 'acceptance.json').read_text())
FIG, MEDIA, RES = HERE / 'figures', HERE / 'media', HERE / 'results'
HIP, THIGH, KNEE = 0, 1, 2                              # FL joint columns in the 12-vector
TASKS = {}


def task(tid):
    def wrap(f):
        TASKS[tid] = f
        return f
    return wrap


def kidx(log, N):
    return np.minimum(np.round(log['t'] / DT).astype(int), N - 1)


def joint_traj(j, points, times, q=Q_HOLD):
    """(N, 12) reference: joint j moves by minimum-jerk segments through `points` (a repeated point is a hold); others hold q."""
    s = np.concatenate([[points[0]]] + [lib.segment([a], [b], T)[:, 0] for a, b, T in zip(points, points[1:], times)])
    Q = np.tile(np.asarray(q, float), (len(s), 1))
    Q[:, j] = s
    return Q


def step_ref(j, q0, q1, t_step, q=Q_HOLD):
    """Ideal step of joint j at t_step (dq* = 0 throughout, so no derivative kick)."""
    a, b = np.array(q, float), np.array(q, float)
    a[j], b[j] = q0, q1
    return lambda t: (b if t >= t_step else a, np.zeros(12))


# ---- 2.01 -------------------------------------------------------------------------------------------------------------------
@task('2.01')
def t201():
    m, d = lib.load_supported()
    kp, kd = 100.0, 2.0
    Q = joint_traj(HIP, [0, .3, .3, -.3, -.3, 0, 0], [1, .5, 1.5, .5, 1, .3])
    d.qpos[:] = Q[0]
    log = lib.simulate(m, d, lib.joint_controller(m, lib.array_ref(Q), kp, kd, gravity=False), (len(Q) - 1) * DT)
    k = kidx(log, len(Q))
    e = log['q'][:, HIP] - Q[k, HIP]
    FL = lib.LegModel(m, 'FL')
    holds = []
    for t0, t1 in ((1.3, 1.5), (3.3, 3.5)):                 # last 0.2 s of each hold
        w = (log['t'] > t0) & (log['t'] <= t1)
        qd = Q[k[w][-1]]
        pred = -FL.gravity(qd[:3])[HIP] / kp                # kp (q* - q) = g(q)  =>  q - q* = -g/kp
        holds.append(dict(q_ref_rad=float(qd[HIP]), measured_rad=float(e[w].mean()), predicted_rad=float(pred)))
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    ax[0].plot(log['t'], Q[k, HIP], color=fs.C['plan'], label='reference $q^*$')
    ax[0].plot(log['t'], log['q'][:, HIP], color=fs.C['est'], ls='--', label='measured $q$')
    ax[0].set_ylabel('FL hip angle [rad]'); ax[0].legend(loc='lower left')
    ax[1].plot(log['t'], 1e3 * e, color=fs.C['est'])
    for h, (t0, t1) in zip(holds, ((1.3, 1.5), (3.3, 3.5))):
        ax[1].hlines(1e3 * h['predicted_rad'], t0 - .2, t1, color=fs.C['gt'], lw=2, label='predicted $-g(q)/k_p$' if t0 < 2 else None)
    ax[1].set_ylabel('error $q - q^*$ [mrad]'); ax[1].set_xlabel('time [s]'); ax[1].legend(loc='upper right')
    ax[0].set_title('2.01 FL hip joint, PD without gravity compensation')
    fs.save(fig, FIG / '2-01_hip_tracking')
    return dict(kp=kp, kd=kd, rms_error_rad=lib.rms(e), max_error_rad=float(np.abs(e).max()), holds=holds,
                hold_error_vs_gravity_over_kp_rad=max(abs(h['measured_rad'] - h['predicted_rad']) for h in holds),
                max_abs_torque_nm=float(np.abs(log['tau'][:, HIP]).max())), ['figures/2-01_hip_tracking.png']


# ---- 2.02 -------------------------------------------------------------------------------------------------------------------
def clamp_request(q, lo, hi, margin=0.05):
    """Command layer: a request outside [lo + margin, hi - margin] is clamped and reported."""
    c = float(np.clip(q, lo + margin, hi - margin))
    return c, c != q


@task('2.02')
def t202():
    m, d = lib.load_supported()
    lo, hi = m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'FL_calf_joint')]
    requests = [-1.2, -2.4, -3.0, -0.5, -1.8]
    cmds = [clamp_request(r, lo, hi) for r in requests]
    pts = [Q_HOLD[KNEE]] + [c for c, _ in cmds for _ in (0, 1)]
    Q = joint_traj(KNEE, pts, [.6, .4] * len(cmds))
    log = lib.simulate(m, d, lib.joint_controller(m, lib.array_ref(Q), 150.0, 2.0, gravity=False), (len(Q) - 1) * DT)
    q = log['q'][:, KNEE]
    table = []
    for i, (r, (c, clamped)) in enumerate(zip(requests, cmds)):
        kend = round((i + 1) * 1.0 / DT) - 1               # last sample of hold i
        table.append(dict(request_rad=r, command_rad=c, clamped=bool(clamped), final_rad=float(q[kend]), error_rad=float(abs(q[kend] - c))))
    excess = float(max(0.0, (q - hi).max(), (lo - q).max()))
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.axhspan(hi, -0.3, color=fs.C['bad'], alpha=.08); ax.axhspan(-3.2, lo, color=fs.C['bad'], alpha=.08)
    ax.axhline(lo, color=fs.C['bad'], lw=.8); ax.axhline(hi, color=fs.C['bad'], lw=.8)
    ax.plot(log['t'], Q[kidx(log, len(Q)), KNEE], color=fs.C['plan'], label='command (after clamping)')
    ax.plot(log['t'], q, color=fs.C['est'], ls='--', label='measured')
    for i, row in enumerate(table):
        ax.plot(i + .8, row['request_rad'], 'x', color=fs.C['bad'] if row['clamped'] else fs.C['plan'], ms=7)
    ax.set_ylim(-3.2, -0.3); ax.set_xlabel('time [s]'); ax.set_ylabel('FL knee angle [rad]')
    ax.set_title('2.02 knee moves, requests (x) outside the range are clamped'); ax.legend(loc='lower right')
    fs.save(fig, FIG / '2-02_knee_limits')
    return dict(range_rad=[float(lo), float(hi)], margin_rad=0.05, moves=table, n_requests_clamped=sum(r['clamped'] for r in table),
                max_final_error_rad=max(r['error_rad'] for r in table), max_limit_excess_rad=excess,
                q_min_rad=float(q.min()), q_max_rad=float(q.max())), ['figures/2-02_knee_limits.png']


# ---- 2.03 -------------------------------------------------------------------------------------------------------------------
def velocity_loop(m, j, v_ref, kv, ki):
    """Override for joint j: tau = b v* + kv (v* - v) + ki * integral(v* - v) dt."""
    b, state = m.dof_damping[j], dict(i=0.0)

    def override(m, d):
        e = v_ref(d.time) - d.qvel[j]
        state['i'] += e * DT
        tau = np.zeros(12)
        tau[j] = b * v_ref(d.time) + kv * e + ki * state['i']
        return np.arange(12) == j, tau
    return override


@task('2.03')
def t203():
    switch = [0.2, 0.95, 1.7, 2.45, 3.2]                    # velocity command changes here
    v_ref = lambda t: 0.0 if t < switch[0] or t >= switch[-1] else 0.8 * (-1) ** int(np.searchsorted(switch, t, 'right') - 1)  # noqa: E731
    runs = {}
    for name, ki in (('PI', 100.0), ('P', 0.0)):
        m, d = lib.load_supported()
        q0 = Q_HOLD.copy(); q0[HIP] = -0.3; d.qpos[:] = q0
        log = lib.simulate(m, d, lib.joint_controller(m, lib.hold_ref(q0), override=velocity_loop(m, HIP, v_ref, 2.0, ki)), 3.4)
        vr = np.array([v_ref(t - DT) for t in log['t']])
        steady = np.zeros(len(vr), bool)
        for a, b in zip(switch, switch[1:]):
            steady |= (log['t'] > a + 0.15) & (log['t'] < b)
        runs[name] = dict(log=log, vr=vr, rms=lib.rms((log['dq'][:, HIP] - vr)[steady]),
                          mean=float((log['dq'][:, HIP] - vr)[steady].mean()))
    lo, hi = lib.LegModel(m, 'FL').lo[HIP], lib.LegModel(m, 'FL').hi[HIP]
    qa = runs['PI']['log']['q'][:, HIP]
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    t = runs['PI']['log']['t']
    ax[0].plot(t, runs['PI']['vr'], color=fs.C['plan'], label='command $\\dot q^*$')
    ax[0].plot(t, runs['PI']['log']['dq'][:, HIP], color=fs.C['est'], label='PI, measured')
    ax[0].plot(t, runs['P']['log']['dq'][:, HIP], color=fs.C['est2'], lw=1, label='P only, measured')
    ax[0].set_ylabel('FL hip velocity [rad/s]'); ax[0].set_ylim(-1.6, 1.2); ax[0].legend(loc='lower center', ncol=3)
    ax[1].plot(t, qa, color=fs.C['est'], label='PI'); ax[1].plot(t, runs['P']['log']['q'][:, HIP], color=fs.C['est2'], lw=1, label='P only')
    ax[1].set_ylabel('angle [rad]'); ax[1].set_xlabel('time [s]'); ax[1].legend(loc='lower left')
    ax[0].set_title('2.03 velocity control of the FL hip joint')
    fs.save(fig, FIG / '2-03_velocity')
    return dict(kv=2.0, ki=100.0, command_rad_s=0.8, pi_steady_rms_error_rad_s=runs['PI']['rms'], pi_steady_mean_error_rad_s=runs['PI']['mean'],
                p_only_steady_rms_error_rad_s=runs['P']['rms'], p_only_steady_mean_error_rad_s=runs['P']['mean'],
                p_only_final_angle_rad=float(runs['P']['log']['q'][-1, HIP]),
                max_limit_excess_rad=float(max(0, (qa - hi).max(), (lo - qa).max())), q_range_rad=[float(qa.min()), float(qa.max())]), \
        ['figures/2-03_velocity.png']


# ---- 2.04 -------------------------------------------------------------------------------------------------------------------
@task('2.04')
def t204():
    m, d = lib.load_supported()
    FL = lib.LegModel(m, 'FL')
    q0 = Q_HOLD.copy(); q0[HIP] = -0.6; d.qpos[:] = q0
    t_on, t_off, tau_cmd = 0.3, 0.6, 2.0
    torque = lambda m, d: (np.arange(12) == HIP if t_on <= d.time < t_off else np.zeros(12, bool), np.full(12, tau_cmd))  # noqa: E731
    log = lib.simulate(m, d, lib.joint_controller(m, lib.hold_ref(q0), override=torque), t_off)
    w = log['t'] > t_on + 1e-9
    t, q, dq = log['t'][w] - t_on, log['q'][w, HIP], log['dq'][w, HIP]
    J, b, f = FL.inertia(q0[:3], HIP), FL.damping[HIP], FL.frictionloss[HIP]
    g = lambda qh: FL.gravity(np.array([qh, q0[THIGH], q0[KNEE]]))[HIP]  # noqa: E731
    n = t <= 0.010 + 1e-9                                   # first 10 ms: q - q0 = v0 t + a t^2 / 2
    A = np.stack([t[n], t[n] ** 2 / 2], 1)
    a_meas = float(np.linalg.lstsq(A, q[n] - log['q'][~w][-1, HIP], rcond=None)[0][1])
    a_pred = (tau_cmd - g(q0[HIP]) - f) / J
    # inertia identified from the first 30 ms: J (v[k+1] - v[k]) / h = tau - g(q) - f - b v[k+1]  (MuJoCo: implicit damping)
    v = log['dq'][:, HIP]; k0 = int(np.argmax(w)); ks = np.arange(k0, k0 + 15)
    acc = (v[ks] - v[ks - 1]) / DT
    rhs = tau_cmd - np.array([g(x) for x in log['q'][ks, HIP]]) - f - b * v[ks]
    J_hat = float(acc @ rhs / (acc @ acc))
    a_first = float(acc[0])
    M = np.zeros((12, 12)); d2 = mujoco.MjData(m); d2.qpos[:] = q0; mujoco.mj_forward(m, d2); mujoco.mj_fullM(m, d2, M)
    J_free = 1 / np.linalg.inv(M[:3, :3])[0, 0]             # check only: inertia felt with the other FL joints free
    # model: J q'' = tau - g(q) - b q' - f sign(q'), explicit Euler at DT/20
    qm, vm, sub, model = q0[HIP], 0.0, 20, []
    for _ in range(len(t)):
        for _ in range(sub):
            net = tau_cmd - g(qm) - b * vm
            acc = (net - f * np.sign(vm)) / J if vm != 0 or abs(net) > f else 0.0
            vm += acc * DT / sub; qm += vm * DT / sub
        model.append((qm, vm))
    model = np.array(model)
    late = t > 0.2
    v_ss = (tau_cmd - np.array([g(x) for x in q[late]]) - f) / b
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    ax[0].plot(t, q, color=fs.C['est'], label='MuJoCo'); ax[0].plot(t, model[:, 0], color=fs.C['gt'], ls='--', label='model $J\\ddot q + b\\dot q = \\tau - g(q) - f$')
    ax[0].set_ylabel('FL hip angle [rad]'); ax[0].legend(loc='upper left')
    ax[1].plot(t, dq, color=fs.C['est']); ax[1].plot(t, model[:, 1], color=fs.C['gt'], ls='--')
    ax[1].plot(t[late], v_ss, color=fs.C['plan'], lw=2, label='$(\\tau - g(q) - f)/b$'); ax[1].legend(loc='lower right')
    ax[1].set_ylabel('velocity [rad/s]'); ax[1].set_xlabel(f'time since {tau_cmd:.1f} N m torque applied [s]')
    ax[0].set_title('2.04 open-loop torque on the FL hip joint')
    fs.save(fig, FIG / '2-04_torque_response')
    return dict(tau_cmd_nm=tau_cmd, q0_rad=float(q0[HIP]), J_locked_kgm2=J, J_free_kgm2_mujoco_check=float(J_free), gravity_at_q0_nm=float(g(q0[HIP])),
                inertia_identified_kgm2=J_hat, inertia_identified_rel_error=abs(J_hat - J) / J,
                first_step_accel_measured=a_first, first_step_accel_predicted=float((tau_cmd - g(q0[HIP]) - f) / (J + DT * b)),
                initial_accel_quadfit_measured=a_meas, initial_accel_predicted=float(a_pred),
                initial_accel_quadfit_rel_error=abs(a_meas - a_pred) / abs(a_pred),
                terminal_velocity_rel_error=float(np.abs(dq[late] - v_ss).mean() / np.abs(v_ss).mean()),
                trajectory_rms_vs_model_rad=lib.rms(q - model[:, 0]), final_velocity_rad_s=float(dq[-1]),
                angle_moved_rad=float(q[-1] - q0[HIP])), ['figures/2-04_torque_response.png']


# ---- 2.05 / 2.06 ------------------------------------------------------------------------------------------------------------
def step_trial(kp, kd, step=0.2, T=0.8, t_step=0.2) -> tuple[dict[str, Any], np.ndarray, np.ndarray, np.ndarray]:
    m, d = lib.load_supported()
    J = lib.LegModel(m, 'FL').inertia(Q_HOLD[:3], HIP)
    log = lib.simulate(m, d, lib.joint_controller(m, step_ref(HIP, 0.0, step, t_step), kp, kd), t_step + T)
    w = log['t'] >= t_step
    t, y = log['t'][w] - t_step, log['q'][w, HIP]
    meas = lib.step_metrics(t, y, 0.0, step)
    y_pred, zeta, wn = lib.second_order_step(J, kd + m.dof_damping[HIP], kp, t)
    pred = lib.step_metrics(t, step * y_pred, 0.0, step)
    return dict(kp=kp, kd=kd, J_kgm2=J, zeta_pred=zeta, wn_pred_rad_s=wn, zeta_meas=lib.zeta_from_overshoot(meas['overshoot_pct']),
                measured=meas, predicted=pred, max_abs_torque_nm=float(np.abs(log['tau'][w, HIP]).max())), t, y, step * y_pred


@task('2.05')
def t205():
    r, t, y, yp = step_trial(100.0, 0.0)
    me, pr = r['measured'], r['predicted']
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.axhspan(0.2 * 0.95, 0.2 * 1.05, color=fs.C['fill']); ax.axhline(0.2, color=fs.C['plan'], lw=1, label='reference')
    ax.plot(t, yp, color=fs.C['gt'], ls='--', label=f'predicted, $\\zeta$ = {r["zeta_pred"]:.3f}')
    ax.plot(t, y, color=fs.C['est'], label=f'MuJoCo, overshoot {me["overshoot_pct"]:.1f} %')
    ax.set_xlabel('time since step [s]'); ax.set_ylabel('FL hip angle [rad]'); ax.set_xlim(0, 0.4); ax.legend(loc='lower right')
    ax.set_title('2.05 PD step response, $k_p$ = 100 N m/rad, $k_d$ = 0 (only the joint damping b = 2)')
    fs.save(fig, FIG / '2-05_pd_step')
    r.update(zeta_rel_error=abs(r['zeta_meas'] - r['zeta_pred']) / r['zeta_pred'],
             overshoot_error_pp=abs(me['overshoot_pct'] - pr['overshoot_pct']),
             rise_rel_error=abs(me['rise_s'] - pr['rise_s']) / pr['rise_s'],
             settling_rel_error=abs(me['settling_s'] - pr['settling_s']) / pr['settling_s'], final_error_rad=me['final_error_rad'])
    return r, ['figures/2-05_pd_step.png']


@task('2.06')
def t206():
    gains = [(40.0, 0.0), (80.0, 0.0), (150.0, 0.0), (150.0, 2.0)]
    rows, curves = [], []
    for kp, kd in gains:
        r, t, y, yp = step_trial(kp, kd, step=0.15)
        m, d = lib.load_supported()                          # 1 Hz sine, amplitude 0.15 rad, from rest at 0
        w = 2 * np.pi
        ref = lambda t: (np.r_[0.15 * np.sin(w * t), Q_HOLD[1:]], np.r_[0.15 * w * np.cos(w * t), np.zeros(11)])  # noqa: E731
        log = lib.simulate(m, d, lib.joint_controller(m, ref, kp, kd), 3.0)
        late = log['t'] > 1.0
        r['sine_rms_error_rad'] = lib.rms(log['q'][late, HIP] - 0.15 * np.sin(w * (log['t'][late] - DT)))
        rows.append(r); curves.append((t, y, yp))
    shades = ['#9cc3dc', '#4f9cc8', '#0b74a8', '#7a45d6']
    fig, ax = plt.subplots(1, 2, figsize=(8.4, 3.4), width_ratios=[1.6, 1])
    for (t, y, yp), r, c in zip(curves, rows, shades):
        ax[0].plot(t, yp, color=fs.C['gt'], ls='--', lw=.9)
        ax[0].plot(t, y, color=c, label=f'$k_p$ {r["kp"]:.0f}, $k_d$ {r["kd"]:.0f}: OS {r["measured"]["overshoot_pct"]:.1f} %')
    ax[0].set_xlim(0, 0.3); ax[0].set_xlabel('time since step [s]'); ax[0].set_ylabel('FL hip angle [rad]'); ax[0].legend(loc='lower right')
    ax[0].set_title('2.06 step 0.15 rad (grey dashed: predicted)')
    ax[1].bar(range(4), [1e3 * r['sine_rms_error_rad'] for r in rows], color=shades)
    ax[1].set_xticks(range(4), [f'{r["kp"]:.0f}/{r["kd"]:.0f}' for r in rows]); ax[1].set_xlabel('$k_p$ / $k_d$')
    ax[1].set_ylabel('RMS error [mrad]'); ax[1].set_title('1 Hz sine tracking')
    fs.save(fig, FIG / '2-06_gains')
    os_err = [abs(r['measured']['overshoot_pct'] - r['predicted']['overshoot_pct']) for r in rows]
    rise = [r['measured']['rise_s'] for r in rows[:3]]
    sine = [r['sine_rms_error_rad'] for r in rows[:3]]
    return dict(sets=rows, max_overshoot_error_pp=max(os_err), rise_time_decreases_with_kp=bool(rise[0] > rise[1] > rise[2]),
                kd_overshoot_drop_pp=rows[2]['measured']['overshoot_pct'] - rows[3]['measured']['overshoot_pct'],
                sine_rms_decreases_with_kp=bool(sine[0] > sine[1] > sine[2])), ['figures/2-06_gains.png']


# ---- 2.07 -------------------------------------------------------------------------------------------------------------------
@task('2.07')
def t207():
    m, d = lib.load_supported()
    p_base, rng, per_leg = lib.base_origin(m), np.random.default_rng(7), {}
    for i, L in enumerate(lib.legs(m)):
        body = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{L.name}_{p}') for p in ('hip', 'thigh', 'calf')]
        foot = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, L.name)
        ef, eo = [], []
        for _ in range(500):
            q = rng.uniform(L.lo, L.hi)
            d.qpos[3 * i:3 * i + 3] = q; mujoco.mj_kinematics(m, d)
            origins, _, f = L.frames(q)
            ef.append(np.linalg.norm(p_base + f - d.geom_xpos[foot]))
            eo.append(np.abs(p_base + origins - d.xpos[body]).max())
        d.qpos[3 * i:3 * i + 3] = Q_HOLD[:3]
        per_leg[L.name] = dict(foot_max_m=float(max(ef)), joint_origin_max_m=float(max(eo)))
    return dict(n_per_leg=500, legs=per_leg, foot_max_error_m=max(v['foot_max_m'] for v in per_leg.values()),
                joint_origin_max_error_m=max(v['joint_origin_max_m'] for v in per_leg.values())), []


# ---- 2.08 -------------------------------------------------------------------------------------------------------------------
@task('2.08')
def t208():
    m, d = lib.load_supported()
    Ls = lib.legs(m)
    offs = np.array([(x, y, z) for x in (-.06, 0, .06) for y in (-.03, 0, .03) for z in (-.05, 0, .05)])
    sols, alg = np.zeros((len(offs), 12)), []
    for i, L in enumerate(Ls):
        p0 = L.fk(Q_HOLD[:3])
        for k, o in enumerate(offs):
            q = lib.ik_closest(L, p0 + o, Q_HOLD[:3])
            sols[k, 3 * i:3 * i + 3] = q
            alg.append(np.linalg.norm(L.fk(q) - (p0 + o)))
    Q = np.vstack([Q_HOLD[None]] + [np.vstack([lib.segment(a, b, .3), lib.segment(b, b, .3)]) for a, b in zip(np.vstack([Q_HOLD, sols[:-1]]), sols)])
    log = lib.simulate(m, d, lib.joint_controller(m, lib.array_ref(Q)), (len(Q) - 1) * DT)
    err = np.zeros((len(offs), 4))
    for k in range(len(offs)):
        s = round((k + 1) * 0.6 / DT) - 1                  # last sample of hold k
        for i, L in enumerate(Ls):
            err[k, i] = np.linalg.norm(log['feet'][s, i] - lib.base_origin(m) - (L.fk(Q_HOLD[:3]) + offs[k]))
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    for i, leg in enumerate(LEGS):
        ax.plot(np.arange(len(offs)) + (i - 1.5) * .15, 1e3 * err[:, i], 'o', color=fs.LEGS[leg], label=leg)
    ax.axhline(2.0, color=fs.C['bad'], lw=.8, label='acceptance 2 mm')
    ax.set_xlabel('target number (3 x 3 x 3 grid, 60/30/50 mm steps)'); ax.set_ylabel('settled foot error [mm]')
    ax.set_title('2.08 closed-form IK, foot position in MuJoCo after 0.3 s hold'); ax.legend(ncol=5, loc='upper right')
    fs.save(fig, FIG / '2-08_ik_settled')
    return dict(n_targets_per_leg=len(offs), algebraic_max_error_m=float(max(alg)), settled_max_error_m=float(err.max()),
                settled_median_error_m=float(np.median(err)), settled_max_error_by_leg_m=dict(zip(LEGS, err.max(0).tolist()))), \
        ['figures/2-08_ik_settled.png']


# ---- 2.09 to 2.12 and 2.14: foot paths ----------------------------------------------------------------------------------------
def run_paths(m, d, rel_paths, rec=None):
    """rel_paths {leg: function p0 -> (N, 3) base-frame path}; adds a 0.5 s lead-in from the standing foot. Returns log, paths, starts."""
    L = {x.name: x for x in lib.legs(m)}
    full, start = {}, {}
    for leg, make in rel_paths.items():
        p0 = L[leg].fk(Q_HOLD[3 * LEGS.index(leg):3 * LEGS.index(leg) + 3])
        full[leg], start[leg] = lib.lead_in(p0, make(p0))
    Q = lib.foot_ref(m, full)
    log = lib.simulate(m, d, lib.joint_controller(m, lib.array_ref(Q)), (len(Q) - 1) * DT, rec)
    return log, full, start


def path_figure(name, title, log, m, leg, P, start, extra=None):
    p, e_t, e_p = lib.foot_errors(log, m, leg, P, start)
    t = log['t'][kidx(log, len(P)) >= start]
    fig, ax = plt.subplots(1, 2, figsize=(8.4, 3.4), width_ratios=[1, 1.3])
    ax[0].plot(1e3 * P[start:, 0], 1e3 * P[start:, 2], color=fs.C['plan'], lw=2.2, label='reference')
    ax[0].plot(1e3 * p[:, 0], 1e3 * p[:, 2], color=fs.C['est'], lw=1, label='MuJoCo foot')
    ax[0].set_aspect('equal', 'datalim'); ax[0].set_xlabel('x forward [mm]'); ax[0].set_ylabel('z up [mm]'); ax[0].legend(loc='best')
    ax[0].set_title(f'{name} {leg} foot, base frame')
    ax[1].plot(t, 1e3 * e_t, color=fs.C['est'], label='tracking $|p - p^*(t)|$')
    ax[1].plot(t, 1e3 * e_p, color=fs.C['est2'], lw=1, label='distance to path')
    ax[1].set_xlabel('time [s]'); ax[1].set_ylabel('error [mm]'); ax[1].legend(loc='upper right'); ax[1].set_title(title)
    if extra:
        extra(ax)
    fs.save(fig, FIG / f'{name.replace(".", "-")}_{title.split()[0].lower()}')
    return p, e_t, e_p, f'figures/{name.replace(".", "-")}_{title.split()[0].lower()}.png'


def side_rec(m, name, P, feet, **kw):
    return lib.PathRecorder(m, MEDIA / f'{name}.mp4', [Pi + lib.base_origin(m) for Pi in P], feet, **kw)


@task('2.09')
def t209():
    m, d = lib.load_supported()
    log, P, s = run_paths(m, d, {'FL': lambda p0: lib.polyline([p0, p0 + [0, 0, .08]], [.6, .6], cycles=3)})
    p, e_t, e_p, fig = path_figure('2.09', 'Vertical stroke 80 mm', log, m, 'FL', P['FL'], s['FL'])
    lift = p[:, 2].max() - p[:, 2].min()
    return dict(rms_error_m=lib.rms(e_t), max_error_m=float(e_t.max()), path_rms_error_m=lib.rms(e_p),
                lift_measured_m=float(lift), lift_error_m=float(abs(lift - .08))), [fig]


@task('2.10')
def t210():
    m, d = lib.load_supported()
    log, P, s = run_paths(m, d, {'FL': lambda p0: lib.polyline([p0, p0 + [.06, 0, 0], p0, p0 - [.06, 0, 0]], [.5] * 4, cycles=3)})
    p, e_t, e_p, fig = path_figure('2.10', 'Horizontal stroke 120 mm', log, m, 'FL', P['FL'], s['FL'])
    stroke = p[:, 0].max() - p[:, 0].min()
    return dict(rms_error_m=lib.rms(e_t), max_error_m=float(e_t.max()), path_rms_error_m=lib.rms(e_p),
                stroke_measured_m=float(stroke), stroke_error_m=float(abs(stroke - .12))), [fig]


def triangle(p0, speed=0.3):
    V = [p0 + [-.06, 0, 0], p0 + [.06, 0, 0], p0 + [0, 0, .08]]
    return V, lib.polyline(V, [np.linalg.norm(V[(i + 1) % 3] - V[i]) / speed for i in range(3)], smooth=False, cycles=3)


@task('2.11')
def t211():
    m, d = lib.load_supported()
    L = lib.LegModel(m, 'FL')
    V, tri = triangle(L.fk(Q_HOLD[:3]))
    P, s = lib.lead_in(L.fk(Q_HOLD[:3]), tri)
    with side_rec(m, '2-11_triangle', [P[s:]], [0], lookat=np.mean(V, 0) + lib.base_origin(m), azimuth=-90, elevation=-8, distance=0.85) as rec:
        log, Pd, sd = run_paths(m, d, {'FL': lambda p0: tri}, rec=rec)
        rec.close(); rec.still(d, MEDIA / '2-11_triangle.png')
    p, e_t, e_p, fig = path_figure('2.11', 'Triangle at 0.3 m/s', log, m, 'FL', Pd['FL'], sd['FL'])
    miss = [float(np.linalg.norm(p - v, axis=1).min()) for v in V]
    return dict(edge_speed_m_s=0.3, rms_error_m=lib.rms(e_t), max_error_m=float(e_t.max()), path_rms_error_m=lib.rms(e_p),
                path_max_error_m=float(e_p.max()), vertex_miss_m=miss, max_vertex_miss_m=max(miss),
                max_abs_torque_nm=float(np.abs(log['tau'][:, :3]).max())), [fig, 'media/2-11_triangle.mp4', 'media/2-11_triangle.png']


@task('2.12')
def t212():
    m, d = lib.load_supported()
    r = 0.06
    log, P, s = run_paths(m, d, {'FL': lambda p0: lib.arc_cycle(p0, r, .5, .5, cycles=3)})
    p0 = lib.LegModel(m, 'FL').fk(Q_HOLD[:3])
    p, e_t, e_p, fig = path_figure('2.12', 'Arc r = 60 mm', log, m, 'FL', P['FL'], s['FL'])
    k = kidx(log, len(P['FL']))
    swing = P['FL'][k[k >= s['FL']], 2] > p0[2] + 1e-6
    radial = np.abs(np.hypot(p[swing, 0] - p0[0], p[swing, 2] - p0[2]) - r)
    return dict(radius_m=r, rms_error_m=lib.rms(e_t), max_error_m=float(e_t.max()), path_rms_error_m=lib.rms(e_p),
                radial_rms_error_m=lib.rms(radial), radial_max_error_m=float(radial.max()),
                apex_error_m=float(abs(p[:, 2].max() - p0[2] - r))), [fig]


# ---- 2.13 -------------------------------------------------------------------------------------------------------------------
@task('2.13')
def t213():
    from scipy.spatial import cKDTree  # type: ignore[reportAttributeAccessIssue]  # scipy re-exports it from the compiled _ckdtree, which has no .pyi here
    m, d = lib.load_supported()
    L = lib.LegModel(m, 'FL')
    p0 = L.fk(Q_HOLD[:3])
    designed = [('too_far', p0 + [0, 0, -.2]), ('too_close', L.hip + [0, L.d, -.06]), ('inside_offset', L.hip + [0, .03, -.03]),
                ('joint_limit', L.hip + [0, .37, -.05]), ('joint_limit', L.hip + [0, -.30, -.10])]
    rng = np.random.default_rng(13)
    cloud = cKDTree(np.array([L.fk(rng.uniform(L.lo, L.hi)) for _ in range(100000)]))   # brute-force workspace sample
    rows = []
    for expected, p in designed:
        q, why = lib.ik_checked(L, p)
        rows.append(dict(target_base_m=np.round(p, 4).tolist(), expected=expected, reason=why, rejected=q is None,
                         dls_converged=bool(L.ik_multistart(p, Q_HOLD[:3])[3]), nearest_workspace_sample_m=float(cloud.query(p)[0])))
    ok = 0
    for _ in range(200):                                    # reachable controls
        target = L.fk(rng.uniform(L.lo, L.hi))
        q, why = lib.ik_checked(L, target)
        ok += q is not None and np.linalg.norm(L.fk(q) - target) < 1e-9
    # in simulation: A (reachable) -> U (too_far: rejected, last valid command kept) -> B (reachable)
    seq = [p0 + [.03, 0, .03], designed[0][1], p0]
    q_cmd, Q, log_reason = Q_HOLD[:3].copy(), [Q_HOLD.copy()], []
    for p in seq:
        q, why = lib.ik_checked(L, p)
        log_reason.append(why)
        q_new = q_cmd if q is None else q
        Q += [np.r_[x, Q_HOLD[3:]] for x in np.vstack([lib.segment(q_cmd, q_new, .5), lib.segment(q_new, q_new, .5)])]
        q_cmd = q_new
    Q = np.array(Q)
    log = lib.simulate(m, d, lib.joint_controller(m, lib.array_ref(Q)), (len(Q) - 1) * DT)
    w = (log['t'] > 1.0) & (log['t'] <= 2.0)
    foot = log['feet'][w, 0]
    motion = float(np.linalg.norm(foot - foot[0], axis=1).max())
    # figure: front view (y-z) of the reachable set within 10 mm of the targets' x, and the designed targets
    C = cloud.data[np.abs(cloud.data[:, 0] - L.hip[0]) < 0.01]
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    ax.scatter(1e3 * C[:, 1], 1e3 * C[:, 2], s=1.5, color=fs.C['gt'], alpha=.35, lw=0, label='reachable foot samples, |x - x_hip| < 10 mm')
    ax.plot(1e3 * L.hip[1], 1e3 * L.hip[2], 'o', color=fs.C['ink'], ms=4, label='hip joint')
    for r, off in zip(rows, ((6, -4), (-70, -14), (-72, 6), (6, 4), (6, 4))):
        _, y, z = 1e3 * np.array(r['target_base_m'])
        ax.plot(y, z, 'x', color=fs.C['bad'] if r['rejected'] else fs.C['good'], ms=8, mew=1.6)
        ax.annotate(r['reason'], (y, z), xytext=off, textcoords='offset points', fontsize=8, color=fs.C['bad'])
    ax.set_xlabel('y left [mm]'); ax.set_ylabel('z up [mm]'); ax.set_aspect('equal', 'datalim'); ax.legend(loc='upper left')
    ax.set_title('2.13 FL targets rejected by ik_checked (front view, base frame)')
    fs.save(fig, FIG / '2-13_rejections')
    return dict(designed=rows, n_rejected_with_expected_reason=sum(r['rejected'] and r['reason'] == r['expected'] for r in rows),
                n_dls_also_failed=sum(not r['dls_converged'] for r in rows), reachable_accept_rate=ok / 200,
                sim_sequence_reasons=log_reason, foot_motion_during_rejected_m=motion,
                rho_min_m=L.rho_min, rho_max_m=L.rho_max, lateral_offset_m=float(L.d)), ['figures/2-13_rejections.png']


# ---- 2.14 -------------------------------------------------------------------------------------------------------------------
@task('2.14')
def t214():
    m, d = lib.load_supported()
    Ls = {x.name: x for x in lib.legs(m)}
    pair, r = ('FL', 'RR'), 0.06
    make = lambda p0: lib.arc_cycle(p0, r, .5, .5, cycles=4)  # noqa: E731
    pre = {}
    for leg in pair:
        p0 = Ls[leg].fk(Q_HOLD[:3]); pre[leg] = lib.lead_in(p0, make(p0))[0]
    with side_rec(m, '2-14_two_legs', [pre[x] for x in pair], [LEGS.index(x) for x in pair],
                  lookat=lib.base_origin(m) - [0, 0, 0.2], azimuth=-55, elevation=-12, distance=1.05) as rec:
        log, P, s = run_paths(m, d, {leg: make for leg in pair}, rec=rec)
        rec.close(); rec.still(d, MEDIA / '2-14_two_legs.png')
    z, rms_by = {}, {}
    for leg in pair:
        p, e_t, e_p = lib.foot_errors(log, m, leg, P[leg], s[leg])
        z[leg] = p[:, 2] - Ls[leg].fk(Q_HOLD[:3])[2]
        rms_by[leg] = lib.rms(e_t)
    a, b = z['FL'] - z['FL'].mean(), z['RR'] - z['RR'].mean()
    lags = np.arange(-50, 51)
    xc = [np.dot(a[max(0, -l):len(a) - max(0, l)], b[max(0, l):len(b) - max(0, -l)]) for l in lags]
    lag = float(lags[int(np.argmax(xc))] * DT)
    n = round(1.0 / DT)
    apex = [abs(int(np.argmax(z['FL'][c * n:(c + 1) * n])) - int(np.argmax(z['RR'][c * n:(c + 1) * n]))) * DT for c in range(4)]
    t = log['t'][kidx(log, len(P['FL'])) >= s['FL']]
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    ax.plot(t, 1e3 * (P['FL'][kidx(log, len(P['FL']))[kidx(log, len(P['FL'])) >= s['FL']], 2] - Ls['FL'].fk(Q_HOLD[:3])[2]),
            color=fs.C['plan'], lw=2.2, label='reference (both legs)')
    for leg in pair:
        ax.plot(t, 1e3 * z[leg], color=fs.LEGS[leg], lw=1, ls='--' if leg == 'RR' else '-', label=f'{leg} measured')
    ax.set_xlabel('time [s]'); ax.set_ylabel('foot height above standing [mm]'); ax.set_ylim(-3, 78); ax.legend(loc='upper center', ncol=3)
    ax.set_title(f'2.14 FL and RR in phase: lag {1e3 * lag:.0f} ms, apex difference <= {1e3 * max(apex):.0f} ms')
    fs.save(fig, FIG / '2-14_two_legs')
    # second run: FL and RL (front and rear legs are not mirror images for a forward path), RL half a cycle out of phase
    m, d = lib.load_supported()
    shifted = lambda p0: (lambda P: np.vstack([P[250], np.roll(P[1:], -250, axis=0)]))(make(p0))  # noqa: E731
    log2, P2, s2 = run_paths(m, d, {'FL': make, 'RL': shifted})
    z2, rms2 = {}, {}
    for leg in ('FL', 'RL'):
        p, e_t, _ = lib.foot_errors(log2, m, leg, P2[leg], s2[leg])
        z2[leg] = p[:, 2] - Ls[leg].fk(Q_HOLD[:3])[2]; rms2[leg] = lib.rms(e_t)
    a, b = z2['FL'] - z2['FL'].mean(), z2['RL'] - z2['RL'].mean()
    lags = np.arange(150, 351)                              # RL expected 250 samples (0.5 s) away from FL
    xc = [np.dot(a[:len(a) - l], b[l:]) for l in lags]
    lag2 = float(lags[int(np.argmax(xc))] * DT)
    apex2 = [abs((int(np.argmax(z2['RL'][c * n:(c + 1) * n])) - int(np.argmax(z2['FL'][c * n:(c + 1) * n]))) * DT) for c in range(4)]
    return dict(pair=list(pair), leg_rms_error_m=rms_by, max_leg_rms_error_m=max(rms_by.values()), xcorr_lag_s=abs(lag),
                max_height_difference_m=float(np.abs(z['FL'] - z['RR']).max()),
                apex_time_diff_s=apex, max_apex_time_diff_s=max(apex),
                fl_rl_rms_error_m=rms2, fl_rl_max_rms_error_m=max(rms2.values()), fl_rl_lag_s=lag2, fl_rl_phase_error_s=abs(lag2 - 0.5),
                fl_rl_apex_offsets_s=apex2, fl_rl_max_apex_offset_error_s=max(abs(x - 0.5) for x in apex2)), \
        ['figures/2-14_two_legs.png', 'media/2-14_two_legs.mp4', 'media/2-14_two_legs.png']


# ---- runner -----------------------------------------------------------------------------------------------------------------
def judge(tid, meas):
    checks, ok = {}, True
    for key, rule in ACC[tid]['checks'].items():
        v = meas[key]
        (op, x), = rule.items()
        p = bool(v <= x if op == 'max' else v >= x if op == 'min' else v == x)
        checks[key] = dict(value=v, rule=f'{op} {x}', passed=p)
        ok &= p
    return checks, ok


def run(tid):
    t0 = __import__('time').time()
    meas, outputs = TASKS[tid]()
    checks, ok = judge(tid, meas)
    out = dict(id=tid, title=ACC[tid]['title'], status='verified' if ok else 'failed', checks=checks, measured=meas,
               outputs=outputs, command=f'python run.py {tid}', mujoco=mujoco.__version__, wall_s=round(__import__('time').time() - t0, 1))
    RES.mkdir(exist_ok=True)
    (RES / f'{tid}.json').write_text(json.dumps(out, indent=2, default=float) + '\n')
    print(f'{tid} {out["status"]:8s} ' + ', '.join(f'{k}={v["value"]:.4g}' if isinstance(v['value'], float) else f'{k}={v["value"]}'
                                              for k, v in checks.items()))
    return ok


if __name__ == '__main__':
    ids = list(TASKS) if sys.argv[1:] == ['all'] else sys.argv[1:]
    if not ids or any(i not in TASKS for i in ids):
        raise SystemExit(f'usage: python run.py <task-id>|all   ids: {" ".join(TASKS)}')
    for i in ids:
        run(i)
