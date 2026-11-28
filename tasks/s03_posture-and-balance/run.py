#!/usr/bin/env python3
"""Stage 3 experiments: the Go2 on the floor, no support, posture controller from lib.py.
usage: python run.py <task-id>|all      e.g. python run.py 3.10
Writes results/<id>.json (measured values, thresholds from acceptance.json, pass/fail), figures/ and, for 3.01, 3.02, 3.08 and
3.11, media/. Wrap runs that render in `flock /tmp/go2_render.lock`; RENDER=0 skips rendering."""
import json
import os
import sys
import time

import numpy as np

import lib
from lib import DT, LEGS, Posture, Rejected
import sim
import figstyle as fs
import matplotlib.pyplot as plt
import mujoco

HERE = lib.HERE
ACC = json.loads((HERE / 'acceptance.json').read_text())
FIG, MEDIA, RES = HERE / 'figures', HERE / 'media', HERE / 'results'
RENDER = os.environ.get('RENDER', '1') != '0'
deg = np.rad2deg
TASKS = {}


def task(tid):
    def wrap(f):
        TASKS[tid] = f
        return f
    return wrap


def window(log, t0, t1=np.inf):
    return (log['t'] > t0 + 1e-9) & (log['t'] <= t1 + 1e-9)


def slip(log, w, legs=LEGS):
    """Largest horizontal displacement of the MuJoCo contact point of the given feet over window w (includes the sphere's
    rolling, r * change of calf angle)."""
    i = [LEGS.index(l) for l in legs]
    cp = log['cp'][w][:, i, :2]
    return float(np.nanmax(np.linalg.norm(cp - cp[0], axis=2)))


def in_contact(log, w, legs=LEGS):
    i = [LEGS.index(l) for l in legs]
    return float(np.mean((log['F'][w][:, i, 2] > 1.0).all(1)))


def forces_end(log, T=0.2):
    w = log['t'] > log['t'][-1] - T
    return {l: np.round(log['F'][w, i].mean(0), 2).tolist() for i, l in enumerate(LEGS)}


def margins(log, w, legs):
    i = [LEGS.index(l) for l in legs]
    return np.array([lib.support_margin(cp[i], c) for cp, c in zip(log['cp'][w], log['com'][w])])


def predicted_forces(pose, cp, com, legs):
    """Statics with measured contact points and CoM: weighted least-norm vertical forces (unique for three feet)."""
    i = [LEGS.index(l) for l in legs]
    w = np.zeros(4); w[i] = 1
    dxy = np.nan_to_num(cp[:, :2]) - com[:2]
    return lib.force_distribution(dxy, w, pose.M * 9.81)


def shade(ax, spans):
    for k, (a, b, txt) in enumerate(spans):
        if k % 2 == 0:
            ax.axvspan(a, b, color=fs.C['fill'], zorder=0)
        ax.text((a + b) / 2, 0.97, txt, transform=ax.get_xaxis_transform(), ha='center', va='top', fontsize=7.5, color=fs.C['muted'])


def with_video(name, pose, fn, **kw):
    """Run fn() with a Recorder attached to pose (skipped when RENDER=0); returns (fn(), [outputs])."""
    if not RENDER:
        return fn(), []
    with sim.Recorder(pose.m, MEDIA / f'{name}.mp4', **kw) as rec:
        pose.rec = rec
        rec.n = int(round(pose.d.time * rec.fps))      # sim.Recorder counts frames from t = 0; start the video now
        out = fn()
        pose.rec = None
        mp4 = rec.close()
        png = rec.still(pose.d, MEDIA / f'{name}.png')
    return out, [str(x.relative_to(HERE)) for x in (mp4, png) if x]


# ---- 3.01 -------------------------------------------------------------------------------------------------------------------
def contact_split(m, d):
    """Vertical force (world) the floor applies, summed per touching geom: feet by leg name, other geoms by body name."""
    out, f6 = {}, np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        mujoco.mj_contactForce(m, d, i, f6)
        g = c.geom2 if m.geom_bodyid[c.geom2] else c.geom1          # the robot geom (the floor belongs to the world body)
        sign = 1.0 if g == c.geom2 else -1.0
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        out[name] = out.get(name, 0.0) + sign * (c.frame.reshape(3, 3).T @ f6[:3])[2]
    return out


@task('3.01')
def t301():
    m, d = sim.load()
    pose = lib.rest(m, d, T=3.0)
    log = pose.log()
    mujoco.mj_forward(m, d)
    split = contact_split(m, d)
    W = pose.M * 9.81
    last = window(log, log['t'][-1] - 1.0)
    v = np.linalg.norm(np.diff(log['p'][window(log, log['t'][-1] - 0.5)], axis=0), axis=1) / DT
    rpy = deg(log['rpy'][-1])
    outputs = []
    if RENDER:
        with sim.Recorder(m, distance=1.1, azimuth=150, elevation=-10, contacts=True) as rec:
            outputs.append(str(rec.still(d, MEDIA / '3-01_rest.png').relative_to(HERE)))
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    names = list(split)
    ax.bar(names, split.values(), color=[fs.LEGS.get(n, fs.C['gt']) for n in names])
    ax.axhline(W / 4, color=fs.C['plan'], ls='--', lw=1, label='$mg/4$')
    ax.set_ylabel('vertical floor force [N]'); ax.legend(loc='upper left')
    ax.set_title(f'3.01 resting pose: who carries the {W:.1f} N weight (sum {sum(split.values()):.1f} N)')
    fs.save(fig, FIG / '3-01_rest_forces'); outputs.append('figures/3-01_rest_forces.png')
    tau_frac = np.abs(log['tau'][last]) / pose.limit
    return dict(q_rest=lib.Q_REST[:3].tolist(), base_height_m=float(log['p'][-1, 2]), roll_deg=float(rpy[0]), pitch_deg=float(rpy[1]),
                max_abs_roll_pitch_deg=float(np.abs(rpy[:2]).max()), contact_force_z_N={k: round(v_, 2) for k, v_ in split.items()},
                weight_N=W, weight_carried_rel_error=abs(sum(split.values()) / W - 1), trunk_share=sum(v_ for k, v_ in split.items() if k not in LEGS) / W,
                max_torque_fraction=float(tau_frac.max()), max_torque_nm=float(np.abs(log['tau'][last]).max()),
                base_speed_end_m_s=float(v.max()), foot_force_end_N=forces_end(log)), outputs


# ---- 3.02 -------------------------------------------------------------------------------------------------------------------
@task('3.02')
def t302():
    m, d = sim.load()
    pose = lib.rest(m, d, T=3.0)
    n0, t0 = len(pose._log), d.time
    L = pose.L
    h_kin = -np.mean([Lg.fk(lib.Q_STAND[:3])[2] for Lg in L])     # base height above the foot centres, Q_STAND
    h_pred = h_kin + 0.009                                           # foot centre 9 mm above the floor (MODULE_SPEC)

    def go():
        lib.stand_up(pose, T=1.5, settle=0.5)
        pose.hold(1.0)
    _, outputs = with_video('3-02_stand_up', pose, go, distance=1.2, azimuth=150, elevation=-12, contacts=True)
    log = pose.log(n0)
    up = window(log, t0, t0 + 2.0)
    end = window(log, log['t'][-1] - 0.2)
    z_end = float(log['p'][end, 2].mean())
    sumF = log['F'][:, :, 2].sum(1) / (pose.M * 9.81)
    t_lift = float(log['t'][np.argmax(log['n_other'] == 0)] - t0)
    fig, ax = plt.subplots(3, 1, figsize=(6.4, 5.6), sharex=True)
    t = log['t'] - t0
    ax[0].plot(t, log['p'][:, 2], color=fs.C['est'], label='base height (MuJoCo)')
    ax[0].axhline(h_pred, color=fs.C['plan'], ls='--', lw=1, label=f'predicted {h_pred:.4f} m')
    ax[0].set_ylabel('height [m]'); ax[0].legend(loc='lower right'); ax[0].set_title('3.02 stand up from rest: q_rest to Q_STAND in 1.5 s')
    ax[1].plot(t, deg(log['rpy'][:, 0]), color=fs.C['est'], label='roll'); ax[1].plot(t, deg(log['rpy'][:, 1]), color=fs.C['est2'], label='pitch')
    ax[1].set_ylabel('attitude [deg]'); ax[1].legend(loc='upper right')
    for i, l in enumerate(LEGS):
        ax[2].plot(t, log['F'][:, i, 2], color=fs.LEGS[l], lw=1, label=l)
    ax[2].plot(t, sumF * pose.M * 9.81 / 4, color=fs.C['gt'], lw=1, ls=':', label='(sum of feet) / 4')
    ax[2].set_ylabel('foot $F_z$ [N]'); ax[2].set_xlabel('time since the stand-up started [s]'); ax[2].legend(loc='lower right', ncol=5)
    shade(ax[0], [(0, 1.5, 'joint move'), (1.5, 2.0, 'settle'), (2.0, 3.0, 'posture hold')])
    fs.save(fig, FIG / '3-02_stand_up'); outputs.append('figures/3-02_stand_up.png')
    F = log['F'][end][:, :, 2]
    return dict(predicted_height_m=h_pred, kinematic_height_m=h_kin, final_height_m=z_end, height_error_m=abs(z_end - h_pred),
                max_abs_roll_pitch_deg=float(np.abs(deg(log['rpy'][up, :2])).max()),
                max_torque_fraction=float((np.abs(log['tau'][up]) / pose.limit).max()), max_torque_nm=float(np.abs(log['tau'][up]).max()),
                max_foot_slip_m=slip(log, up), trunk_lift_off_s=t_lift,
                n_feet_in_contact_end=int((F.mean(0) > 1).sum()), n_other_contacts_end=int(log['n_other'][-1]),
                feet_share_end=float(sumF[end].mean()), foot_force_end_N=forces_end(log)), outputs


# ---- 3.03 -------------------------------------------------------------------------------------------------------------------
@task('3.03')
def t303():
    m, d, pose = lib.standing()
    n0, t0 = len(pose._log), d.time
    pose.hold(10.0)
    log = pose.log(n0)
    t = log['t'] - t0
    w = t > 0.5
    k0 = np.argmax(w)
    dz = log['p'][w, 2] - log['p'][k0, 2]
    F = log['F'][w][:, :, 2]
    pred = predicted_forces(pose, log['cp'][-1], log['com'][-1], LEGS)
    fig, ax = plt.subplots(3, 1, figsize=(6.4, 5.4), sharex=True)
    ax[0].plot(t, 1e6 * (log['p'][:, 2] - log['p'][k0, 2]), color=fs.C['est']); ax[0].set_ylabel('height change [µm]')
    ax[0].set_title('3.03 hold standing for 10 s')
    ax[1].plot(t, deg(log['rpy'][:, 0]), color=fs.C['est'], label='roll'); ax[1].plot(t, deg(log['rpy'][:, 1]), color=fs.C['est2'], label='pitch')
    ax[1].set_ylabel('attitude [deg]'); ax[1].legend(loc='upper right')
    for i, l in enumerate(LEGS):
        ax[2].plot(t, log['F'][:, i, 2], color=fs.LEGS[l], lw=1, label=l)
        ax[2].axhline(pred[i], color=fs.LEGS[l], ls=':', lw=0.8)
    ax[2].set_ylabel('foot $F_z$ [N]'); ax[2].set_xlabel('time [s]'); ax[2].legend(loc='lower right', ncol=4)
    fs.save(fig, FIG / '3-03_hold')
    return dict(duration_s=10.0, max_height_deviation_m=float(np.abs(dz).max()), height_m=float(log['p'][k0, 2]),
                max_abs_roll_pitch_deg=float(np.abs(deg(log['rpy'][w, :2])).max()), contact_fraction=in_contact(log, w),
                max_foot_slip_m=slip(log, w), weight_carried_rel_error=float(abs(F.sum(1).mean() / (pose.M * 9.81) - 1)),
                foot_fz_mean_N={l: float(F[:, i].mean()) for i, l in enumerate(LEGS)},
                foot_fz_statics_N={l: float(pred[i]) for i, l in enumerate(LEGS)},
                foot_force_end_N=forces_end(log)), ['figures/3-03_hold.png']


# ---- 3.04 to 3.06: height ---------------------------------------------------------------------------------------------------
def height_error(log):
    return log['p'][:, 2] - log['ref_p'][:, 2]


def height_figure(name, title, log, t0, extra=None):
    t = log['t'] - t0
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    ax[0].plot(t, log['ref_p'][:, 2], color=fs.C['plan'], lw=2.2, label='reference')
    ax[0].plot(t, log['p'][:, 2], color=fs.C['est'], lw=1, label='MuJoCo base height')
    ax[0].set_ylabel('height [m]'); ax[0].legend(loc='best'); ax[0].set_title(title)
    ax[1].plot(t, 1e3 * height_error(log), color=fs.C['est'])
    ax[1].set_ylabel('error [mm]'); ax[1].set_xlabel('time [s]')
    if extra:
        extra(ax, t)
    fs.save(fig, FIG / name)
    return f'figures/{name}.png'


@task('3.04')
def t304():
    m, d, pose = lib.standing()
    n0, t0 = len(pose._log), d.time
    p0 = pose.p.copy()
    low = p0.copy(); low[2] = 0.18
    pose.set_body_pose(p=low, T=1.5); pose.hold(1.0)
    pose.set_body_pose(p=p0, T=1.5); pose.hold(1.0)
    log = pose.log(n0)
    e = height_error(log)
    end = window(log, log['t'][-1] - 0.2)
    fig = height_figure('3-04_crouch', '3.04 crouch: 0.095 m down and back, 1.5 s each', log, t0)
    return dict(start_height_m=float(p0[2]), crouch_height_m=0.18, rms_height_error_m=lib.s02.rms(e), max_height_error_m=float(np.abs(e).max()),
                max_abs_roll_pitch_deg=float(np.abs(deg(log['rpy'][:, :2])).max()),
                final_height_error_m=float(abs(log['p'][end, 2].mean() - p0[2])), contact_fraction=in_contact(log, slice(None)),
                max_torque_fraction=float((np.abs(log['tau']) / pose.limit).max()),
                foot_force_end_N=forces_end(log)), [fig]


def height_steps(pose, heights, T=1.0, hold=1.0):
    """Move to each height, hold; returns [(target, measured mean over the last 0.3 s, log of the hold)]."""
    out = []
    for h in heights:
        p = pose.p.copy(); p[2] = h
        pose.set_body_pose(p=p, T=T)
        lg = pose.hold(hold)
        w = lg['t'] > lg['t'][-1] - 0.3
        out.append((h, float(lg['p'][w, 2].mean()), lg))
    return out


@task('3.05')
def t305():
    m, d, pose = lib.standing()
    n0, t0 = len(pose._log), d.time
    h0 = float(pose.p[2])
    steps = height_steps(pose, [h0 + 0.02, h0 + 0.04, h0 + 0.06])
    # the reach limit, by bisection on the planner (no motion): highest base height every foot can still reach
    lo_, hi_ = h0, 0.5
    for _ in range(30):
        mid = (lo_ + hi_) / 2
        try:
            p = pose.p.copy(); p[2] = mid; pose.set_body_pose(p=p, T=0.2, dry_run=True); lo_ = mid
        except Rejected:
            hi_ = mid
    before = (d.qpos.copy(), sim.foot_positions(m, d))
    p = pose.p.copy(); p[2] = 0.42
    try:
        pose.set_body_pose(p=p, T=1.0); reason = None
    except Rejected as e:
        reason = e.reason
    lg = pose.hold(0.5)
    moved = float(max(np.abs(lg['feet'] - before[1]).max(), np.abs(lg['p'] - before[0][:3]).max()))
    pose.set_body_pose(p=np.r_[pose.p[:2], h0], T=1.0); pose.hold(0.5)
    log = pose.log(n0)
    errs = [abs(meas - h) for h, meas, _ in steps]
    fig = height_figure('3-05_raise', '3.05 raise the body: +20, +40, +60 mm', log, t0)
    return dict(start_height_m=h0, targets_m=[h for h, _, _ in steps], measured_m=[x for _, x, _ in steps], steady_errors_m=errs,
                max_steady_height_error_m=max(errs), max_reachable_height_m=lo_, request_m=0.42, rejected_reason=reason,
                unreachable_request_rejected=reason in ('too_far', 'joint_limit', 'too_close', 'inside_offset'),
                motion_during_rejected_m=moved, contact_fraction=in_contact(log, slice(None)),
                foot_force_end_N=forces_end(log)), [fig]


@task('3.06')
def t306():
    m, d, pose = lib.standing()
    n0, t0 = len(pose._log), d.time
    h0 = float(pose.p[2])
    rows = []
    for h in (0.22, 0.18, 0.15):
        (_, meas, lg), = height_steps(pose, [h])
        mujoco.mj_forward(m, d)
        p, _, R = sim.base_pose(d)
        rows.append(dict(target_m=h, measured_m=meas, error_m=abs(meas - h), trunk_clearance_mujoco_m=lib.mujoco_clearance(m, d),
                         trunk_clearance_predicted_m=lib.trunk_clearance(p, R),
                         legs_clearance_mujoco_m=lib.mujoco_clearance(m, d, lib.leg_bodies(m))))
    p = pose.p.copy(); p[2] = 0.12
    try:
        pose.set_body_pose(p=p, T=1.0); reason = None
    except Rejected as e:
        reason = e.reason
    pose.set_body_pose(p=np.r_[pose.p[:2], h0], T=1.5); pose.hold(0.5)
    log = pose.log(n0)
    fig = height_figure('3-06_lower', '3.06 lower the body to 0.22, 0.18, 0.15 m', log, t0)
    return dict(start_height_m=h0, steps=rows, max_steady_height_error_m=max(r['error_m'] for r in rows),
                min_trunk_clearance_m=min(r['trunk_clearance_mujoco_m'] for r in rows),
                clearance_prediction_error_m=max(abs(r['trunk_clearance_mujoco_m'] - r['trunk_clearance_predicted_m']) for r in rows),
                low_request_m=0.12, low_request_predicted_clearance_m=lib.trunk_clearance(np.r_[0, 0, 0.12], np.eye(3)),
                low_request_rejected=reason == 'clearance', n_other_contacts_max=int(log['n_other'].max()),
                contact_fraction=in_contact(log, slice(None)), max_torque_fraction=float((np.abs(log['tau']) / pose.limit).max()),
                foot_force_end_N=forces_end(log)), [fig]


# ---- 3.07, 3.08: attitude ---------------------------------------------------------------------------------------------------
def attitude_task(axis, name, title):
    m, d, pose = lib.standing()
    n0, t0 = len(pose._log), d.time
    a = np.radians(10.0)
    r0 = pose.rpy.copy()
    rp = lambda v: r0 + np.eye(3)[axis] * v  # noqa: E731
    holds = []
    for target, T in ((a, 1.0), (-a, 1.5), (0.0, 1.0)):
        pose.set_body_pose(rpy=rp(target), T=T)
        lg = pose.hold(1.0)
        w = lg['t'] > lg['t'][-1] - 0.3
        holds.append(dict(target_deg=float(deg(target)), steady_error_deg=float(abs(deg(lg['rpy'][w, axis] - lg['ref_rpy'][w, axis]).mean()))))
    ts = time.time()
    t_s = np.arange(1, int(round(4.0 / DT)) + 1) * DT
    n_sine = len(pose._log)
    pose.follow(rpy=r0 + np.outer(np.radians(8.0) * np.sin(2 * np.pi * 0.5 * t_s), np.eye(3)[axis]))
    pose.hold(0.5)
    log, sine = pose.log(n0), pose.log(n_sine)
    e = deg(log['rpy'][:, axis] - log['ref_rpy'][:, axis])
    es = deg(sine['rpy'][:, axis] - sine['ref_rpy'][:, axis])[:len(t_s)]
    lag = float(np.argmax(np.correlate(sine['rpy'][:len(t_s), axis] - r0[axis], sine['ref_rpy'][:len(t_s), axis] - r0[axis], 'full')) - (len(t_s) - 1)) * DT
    dz = log['p'][:, 2] - log['ref_p'][:, 2]
    t = log['t'] - t0
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    ax[0].plot(t, deg(log['ref_rpy'][:, axis]), color=fs.C['plan'], lw=2.2, label='reference')
    ax[0].plot(t, deg(log['rpy'][:, axis]), color=fs.C['est'], lw=1, label='MuJoCo trunk')
    ax[0].set_ylabel(f'{name} [deg]'); ax[0].legend(loc='lower left'); ax[0].set_title(title)
    ax[1].plot(t, e, color=fs.C['est'], label=f'{name} error')
    ax[1].plot(t, 1e2 * dz, color=fs.C['est2'], lw=1, label='height error [cm]')
    ax[1].set_ylabel('error [deg], [cm]'); ax[1].set_xlabel('time [s]'); ax[1].legend(loc='lower left')
    out = FIG / f'{"3-07" if axis == 1 else "3-08"}_{name}'
    fs.save(fig, out)
    other = [i for i in (0, 1) if i != axis][0]
    return pose, dict(holds=holds, max_steady_error_deg=max(h['steady_error_deg'] for h in holds), sine_amplitude_deg=8.0, sine_hz=0.5,
                      sine_rms_error_deg=lib.s02.rms(es), sine_lag_s=lag, max_error_deg=float(np.abs(e).max()),
                      max_other_axis_deg=float(np.abs(deg(log['rpy'][:, other] - r0[other])).max()),
                      max_height_deviation_m=float(np.abs(dz).max()), contact_fraction=in_contact(log, slice(None)),
                      max_torque_fraction=float((np.abs(log['tau']) / pose.limit).max()), wall_sine_s=round(time.time() - ts, 1),
                      foot_force_end_N=forces_end(log)), [f'figures/{out.name}.png']


@task('3.07')
def t307():
    _, meas, out = attitude_task(1, 'pitch', '3.07 pitch steps to +-10 deg, then a +-8 deg 0.5 Hz sine')
    return meas, out


@task('3.08')
def t308():
    _, meas, out = attitude_task(0, 'roll', '3.08 roll steps to +-10 deg, then a +-8 deg 0.5 Hz sine')
    # body pose sweep for the video: height, pitch, roll, yaw, one after the other, on a fresh standing robot
    m, d, pose = lib.standing()
    n0 = len(pose._log)
    p0, r0 = pose.p.copy(), pose.rpy.copy()
    seq = [(p0 + [0, 0, -0.06], r0, 1.2), (p0 + [0, 0, 0.04], r0, 1.5), (p0, r0, 1.0)]
    for i, a in ((1, 10), (0, 10), (2, 15)):
        e = np.radians(a) * np.eye(3)[i]
        seq += [(p0, r0 + e, 1.0), (p0, r0 - e, 1.5), (p0, r0, 1.0)]

    def go():
        for p, r, T in seq:
            pose.set_body_pose(p=p, rpy=r, T=T)
        pose.hold(0.5)
    _, vids = with_video('3-08_pose_sweep', pose, go, distance=1.3, azimuth=135, elevation=-15)
    lg = pose.log(n0)
    meas.update(sweep_duration_s=float(sum(T for *_, T in seq) + 0.5), sweep_rms_height_error_m=lib.s02.rms(lg['p'][:, 2] - lg['ref_p'][:, 2]),
                sweep_rms_attitude_error_deg={k: lib.s02.rms(deg(lg['rpy'][:, i] - lg['ref_rpy'][:, i])) for i, k in enumerate(('roll', 'pitch', 'yaw'))},
                sweep_contact_fraction=in_contact(lg, slice(None)))
    return meas, out + vids


# ---- 3.09 -------------------------------------------------------------------------------------------------------------------
def com_check(m, d, pose):
    """First-principles CoM (lib.Posture.com_body) against MuJoCo's subtree_com, on a copy of the state."""
    d2 = mujoco.MjData(m); d2.qpos[:] = d.qpos; mujoco.mj_forward(m, d2)
    R = d2.xmat[pose.idx.base].reshape(3, 3)
    return float(np.abs(d2.xpos[pose.idx.base] + R @ pose.com_body(d2.qpos[pose.idx.qpos]) - d2.subtree_com[pose.idx.base]).max())


@task('3.09')
def t309():
    m, d, pose = lib.standing()
    c0 = pose.com_world(pose.p, pose.rpy)[:2]
    rows, model_err = [], [com_check(m, d, pose)]
    q0, feet0, rpy0 = pose.q(), sim.foot_positions(m, d), pose.rpy.copy()
    for k in range(8):
        a = k * np.pi / 4
        target = c0 + 0.04 * np.array([np.cos(a), np.sin(a)])
        pose.shift_com(target, T=1.0)
        lg = pose.hold(1.0)
        w = lg['t'] > lg['t'][-1] - 0.3
        com = lg['com'][w].mean(0)
        F = lg['F'][w][:, :, 2].mean(0)
        cp = np.nanmean(lg['cp'][w], 0)
        cop = (F[:, None] * cp[:, :2]).sum(0) / F.sum()
        pred = predicted_forces(pose, cp, com, LEGS)
        # point-foot check: a sphere of radius r fixed to the calf and rolling without slip moves its centre by
        # r (dphi_y, -dphi_x), dphi = change of calf orientation (body attitude + hip abduction, thigh + knee)
        dq, drpy = lg['q'][-1] - q0, lg['rpy'][-1] - rpy0
        roll_pred = np.array([sim.FOOT_RADIUS * np.array([drpy[1] + dq[3 * i + 1] + dq[3 * i + 2], -(drpy[0] + dq[3 * i])]) for i in range(4)])
        foot_disp = lg['feet'][-1, :, :2] - feet0[:, :2]
        rows.append(dict(foot_centre_displacement_m=foot_disp.tolist(), rolling_prediction_m=roll_pred.tolist(),
                         rolling_model_error_m=float(np.linalg.norm(foot_disp - roll_pred, axis=1).max()),direction_deg=float(deg(a)), target=target.tolist(), com=com[:2].tolist(), com_error_m=float(np.linalg.norm(com[:2] - target)),
                         cop_minus_com_m=float(np.linalg.norm(cop - com[:2])), margin_measured_m=lib.support_margin(cp, com),
                         margin_predicted_m=lib.support_margin(pose.P, target), fz_N=F.tolist(), fz_statics_N=pred.tolist(),
                         fz_feedforward_N=lib.force_distribution(cp[:, :2] - com[:2], np.ones(4), pose.M * 9.81).tolist(), cp=cp[:, :2].tolist()))
        model_err.append(com_check(m, d, pose))
    pose.shift_com(c0, T=1.0)
    try:
        pose.shift_com(c0 + [0.20, 0.0], T=1.0); reason = None
    except Rejected as e:
        reason = e.reason
    pose.hold(0.3)
    for r in rows:
        r['margin_error_m'] = abs(r['margin_measured_m'] - r['margin_predicted_m'])
    fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.8), width_ratios=[1.25, 1])
    cp = np.array(rows[0]['cp'])
    hull = cp[[0, 1, 3, 2, 0]]
    ax[0].plot(1e3 * hull[:, 0], 1e3 * hull[:, 1], color=fs.C['gt'], lw=1.2, label='support polygon (MuJoCo contacts)')
    for i, l in enumerate(LEGS):
        ax[0].plot(*1e3 * cp[i], 'o', color=fs.LEGS[l]); ax[0].annotate(l, 1e3 * cp[i], xytext=(5, 4), textcoords='offset points', fontsize=8)
    T_ = 1e3 * np.array([r['target'] for r in rows]); C_ = 1e3 * np.array([r['com'] for r in rows])
    ax[0].plot(T_[:, 0], T_[:, 1], 's', mfc='none', color=fs.C['plan'], ms=7, label='CoM target')
    ax[0].plot(C_[:, 0], C_[:, 1], '.', color=fs.C['est'], label='CoM measured')
    ax[0].plot(*1e3 * c0, '+', color=fs.C['ink'], ms=8)
    ax[0].set_aspect('equal'); ax[0].set_xlabel('x [mm]'); ax[0].set_ylabel('y [mm]'); ax[0].legend(loc='upper center', bbox_to_anchor=(0.5, 0.9), fontsize=7.5)
    ax[0].set_title('3.09 CoM shifted 40 mm in 8 directions')
    Fz, Fp = np.array([r['fz_N'] for r in rows]), np.array([r['fz_feedforward_N'] for r in rows])
    for i, l in enumerate(LEGS):
        ax[1].plot(Fp[:, i], Fz[:, i], 'o', color=fs.LEGS[l], label=l)
    lim = [Fz.min() - 3, Fz.max() + 3]
    ax[1].plot(lim, lim, color=fs.C['gt'], lw=0.8)
    ax[1].set_xlabel('feed-forward $f_i$ [N]'); ax[1].set_ylabel('MuJoCo $F_z$ [N]'); ax[1].legend(loc='upper left'); ax[1].set_title('foot forces')
    fs.save(fig, FIG / '3-09_com_shift')
    ff_err = float(np.abs(Fz - Fp).max())
    return dict(shift_m=0.04, rows=rows, max_com_error_m=max(r['com_error_m'] for r in rows), max_cop_minus_com_m=max(r['cop_minus_com_m'] for r in rows),
                max_margin_error_m=max(r['margin_error_m'] for r in rows), min_margin_m=min(r['margin_measured_m'] for r in rows),
                max_fz_minus_feedforward_N=ff_err, max_rolling_model_error_m=max(r['rolling_model_error_m'] for r in rows),
                max_foot_centre_displacement_m=max(float(np.linalg.norm(r['foot_centre_displacement_m'], axis=1).max()) for r in rows), com_model_vs_mujoco_m=max(model_err),
                unsafe_request=(c0 + [0.20, 0.0]).tolist(), unsafe_rejected_reason=reason, unsafe_request_rejected=reason == 'margin',
                foot_force_end_N=forces_end(pose.log())), ['figures/3-09_com_shift.png']


# ---- 3.10 to 3.12: three legs -----------------------------------------------------------------------------------------------
LIFT, STEP = 0.06, np.array([0.06, 0.0])


def lift(pose):
    """CoM 0.04 m into the FR-RL-RR triangle, hold, unload and lift FL, hold 2 s. Returns the times of the phases."""
    t = [pose.d.time]
    pose.com_into_triangle('FL', margin=0.04, T=1.0); pose.hold(0.5); t.append(pose.d.time)
    pose.lift_foot('FL', LIFT, T_unload=0.5, T_lift=0.6); t.append(pose.d.time)
    pose.hold(2.0); t.append(pose.d.time)
    return t


def place(pose, xy, recentre=True):
    """Put the lifted FL foot at xy, reload it, and (recentre) move the CoM back to the same offset from the feet centroid."""
    t = [pose.d.time]
    pose.place_foot('FL', xy, T_move=0.6, T_lower=0.6, T_load=0.5); t.append(pose.d.time)
    if recentre:
        pose.shift_com(pose.P[:, :2].mean(0) + pose.c_off, T=1.0); pose.hold(1.0)
    t.append(pose.d.time)
    return t


def three_leg_stats(pose, log, w, legs=('FR', 'RL', 'RR')):
    mg = margins(log, w, legs)
    rpy, p = log['rpy'][w], log['p'][w]
    return dict(min_margin_m=float(mg.min()), stance_contact_fraction=in_contact(log, w, legs), max_stance_slip_m=slip(log, w, legs),
                max_attitude_change_deg=float(np.abs(deg(rpy[:, :2] - rpy[0, :2])).max()), max_height_change_m=float(np.abs(p[:, 2] - p[0, 2]).max())), mg


def lift_figure(name, title, log, t0, spans, mg_t, mg):
    t = log['t'] - t0
    fig, ax = plt.subplots(3, 1, figsize=(6.4, 6.0), sharex=True)
    for i, l in enumerate(LEGS):
        ax[0].plot(t, log['F'][:, i, 2], color=fs.LEGS[l], lw=1.1, label=l)
    ax[0].set_ylabel('foot $F_z$ [N]'); ax[0].legend(loc='upper left', bbox_to_anchor=(1.0, 1.0)); ax[0].set_title(title)
    ax[1].plot(t, 1e3 * (log['feet'][:, 0, 2] - log['feet'][0, 0, 2]), color=fs.LEGS['FL'], label='FL foot height')
    ax[1].plot(t, 1e3 * (log['feet'][:, 0, 0] - log['feet'][0, 0, 0]), color=fs.LEGS['FL'], ls='--', label='FL foot x')
    ax[1].set_ylabel('FL foot [mm]'); ax[1].legend(loc='upper left')
    ax[2].plot(mg_t - t0, 1e3 * mg, color=fs.C['est'], label='margin, FR-RL-RR triangle')
    ax[2].axhline(25, color=fs.C['bad'], ls='--', lw=0.8, label='acceptance 25 mm')
    ax[2].set_ylabel('margin [mm]'); ax[2].set_xlabel('time [s]'); ax[2].legend(loc='lower left')
    shade(ax[0], spans)
    fs.save(fig, FIG / name)
    return f'figures/{name}.png'


def stance_start(pose):
    pose.c_off = pose.com_world(pose.p, pose.rpy)[:2] - pose.P[:, :2].mean(0)
    return pose.P.copy(), pose.p.copy(), pose.rpy.copy()


@task('3.10')
def t310():
    m, d, pose = lib.standing()
    stance_start(pose)
    n0 = len(pose._log)
    t = lift(pose)
    log = pose.log(n0)
    w = window(log, t[1])
    st, mg = three_leg_stats(pose, log, w)
    hold = window(log, t[2] + 0.5, t[3])
    F = log['F'][hold][:, :, 2].mean(0)
    pred = predicted_forces(pose, np.nan_to_num(log['cp'][hold]).mean(0), log['com'][hold].mean(0), ('FR', 'RL', 'RR'))
    z_lift = log['feet'][window(log, t[2], t[3]), 0, 2] - pose.z0[0]
    fl_err = log['feet'][hold, 0] - log['ref_P'][hold, 0]                        # world: foot centre minus its target
    R = np.array([lib.rpy_to_R(r) for r in log['rpy'][hold]]); Rr = np.array([lib.rpy_to_R(r) for r in log['ref_rpy'][hold]])
    fl_body = np.einsum('kji,kj->ki', R, log['feet'][hold, 0] - log['p'][hold]) - np.einsum('kji,kj->ki', Rr, log['ref_P'][hold, 0] - log['ref_p'][hold])
    unload_end = np.argmin(np.abs(log['t'] - (t[1] + 0.5)))
    fl_zero = log['t'][np.argmax((log['t'] > t[1]) & (log['F'][:, 0, 2] <= 1.0))] - t[1]
    nos = no_shift_lift()
    fig = lift_figure('3-10_lift', '3.10 FL lifted 60 mm on three legs', log, t[0],
                      [(0, t[1] - t[0], 'CoM shift'), (t[1] - t[0], t[1] - t[0] + 0.5, 'unload'), (t[1] - t[0] + 0.5, t[2] - t[0], 'lift'),
                       (t[2] - t[0], t[3] - t[0], 'hold')], log['t'][w], mg)
    st.update(margin_target_m=0.04, lift_command_m=LIFT, min_lift_m=float(z_lift.min()), fz_hold_N=F.tolist(), fz_statics_N=pred.tolist(),
              max_force_prediction_error_n=float(np.abs(F - pred)[1:].max()), fl_fz_hold_max_N=float(log['F'][hold, 0, 2].max()),
              fl_fz_end_of_unload_N=float(log['F'][unload_end, 0, 2]), fl_fz_below_1N_after_s=float(fl_zero),
              fl_height_error_world_m=float(fl_err[:, 2].mean()), fl_error_body_frame_m=float(np.linalg.norm(fl_body, axis=1).max()),
              max_torque_fraction=float((np.abs(log['tau']) / pose.limit).max()), no_com_shift=nos[0], foot_force_end_N=forces_end(log))
    return st, [fig, nos[1]]


def no_shift_lift():
    """Failure case: lift FL from the centred stance, the margin guard switched off (min_margin = -1 m)."""
    m, d, pose = lib.standing()
    n0, t0 = len(pose._log), d.time
    mg0 = pose.margin(['FR', 'RL', 'RR'])
    pose.lift_foot('FL', LIFT, T_unload=0.5, T_lift=0.6, min_margin=-1.0)
    pose.hold(2.0)
    log = pose.log(n0)
    t = log['t'] - t0
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.2), sharex=True)
    ax[0].plot(t, deg(log['rpy'][:, 0] - log['rpy'][0, 0]), color=fs.C['est'], label='roll change')
    ax[0].plot(t, deg(log['rpy'][:, 1] - log['rpy'][0, 1]), color=fs.C['est2'], label='pitch change')
    ax[0].set_ylabel('[deg]'); ax[0].legend(loc='lower left')
    ax[0].set_title(f'3.10 failure case: FL lifted without the CoM shift (margin {1e3 * mg0:.1f} mm)')
    for i, l in enumerate(LEGS):
        ax[1].plot(t, log['F'][:, i, 2], color=fs.LEGS[l], lw=1.1, label=l)
    ax[1].set_ylabel('foot $F_z$ [N]'); ax[1].set_xlabel('time [s]'); ax[1].legend(loc='upper left', ncol=4)
    shade(ax[0], [(0, 0.5, 'unload'), (0.5, 1.1, 'lift'), (1.1, 3.1, 'hold')])
    fs.save(fig, FIG / '3-10_no_shift')
    end = window(log, log['t'][-1] - 0.2)
    return dict(predicted_margin_m=mg0, max_abs_roll_deg=float(np.abs(deg(log['rpy'][:, 0] - log['rpy'][0, 0])).max()),
                max_abs_pitch_deg=float(np.abs(deg(log['rpy'][:, 1] - log['rpy'][0, 1])).max()),
                fz_end_N=log['F'][end][:, :, 2].mean(0).tolist(), n_other_contacts_max=int(log['n_other'].max()),
                base_height_change_m=float(log['p'][-1, 2] - log['p'][0, 2])), 'figures/3-10_no_shift.png'


@task('3.11')
def t311():
    m, d, pose = lib.standing()
    P0, _, _ = stance_start(pose)
    n0 = len(pose._log)
    target = P0[0, :2] + STEP

    def go():
        return lift(pose), place(pose, target)
    (tl, tp), vids = with_video('3-11_three_leg_step', pose, go, distance=1.25, azimuth=120, elevation=-15, contacts=True)
    log = pose.log(n0)
    w = window(log, tl[1], tp[1])
    st, mg = three_leg_stats(pose, log, w)
    end = window(log, tp[1] - 0.3, tp[1])
    cp = np.nanmean(log['cp'][end, 0], 0)
    fin = window(log, log['t'][-1] - 0.2)
    fig = lift_figure('3-11_place', '3.11 FL lifted, moved 60 mm forward, placed, reloaded', log, tl[0],
                      [(0, tl[1] - tl[0], 'CoM'), (tl[1] - tl[0], tl[3] - tl[0], 'unload, lift, hold'), (tp[0] - tl[0], tp[1] - tl[0], 'place, load'),
                       (tp[1] - tl[0], tp[2] - tl[0], 'CoM back')], log['t'][w], mg)
    st.update(target_xy=target.tolist(), contact_xy=cp[:2].tolist(), placement_error_m=float(np.linalg.norm(cp[:2] - target)),
              placement_error_vs_foot_centre_m=float(np.linalg.norm(log['feet'][end, 0, :2].mean(0) - target)),
              n_feet_in_contact_end=int((log['F'][fin][:, :, 2].mean(0) > 1).sum()), fl_fz_end_N=float(log['F'][fin, 0, 2].mean()),
              max_torque_fraction=float((np.abs(log['tau']) / pose.limit).max()), duration_s=float(log['t'][-1] - tl[0]),
              foot_force_end_N=forces_end(log))
    return st, [fig] + vids


@task('3.12')
def t312():
    m, d, pose = lib.standing()
    P0, p0, r0 = stance_start(pose)
    q0 = pose.q()
    cp0 = contact_points(m, d)[:, :2]
    n0, t0 = len(pose._log), d.time
    cycles = []
    for k in range(3):
        try:
            lift(pose); place(pose, P0[0, :2] + STEP)
            lift(pose); place(pose, P0[0, :2], recentre=False)
            pose.set_body_pose(p=p0, rpy=r0, T=1.0); lg = pose.hold(1.0)
        except Rejected as e:
            cycles.append(dict(rejected=e.reason)); break
        w = lg['t'] > lg['t'][-1] - 0.3
        cycles.append(dict(base_position_error_m=float(np.linalg.norm(lg['p'][w].mean(0) - p0)),
                           base_position_error_xyz_m=(lg['p'][w].mean(0) - p0).tolist(),
                           attitude_error_deg=float(np.abs(deg(lg['rpy'][w].mean(0) - r0)).max()),
                           foot_position_error_m=float(np.linalg.norm(np.nanmean(lg['cp'][w], 0)[:, :2] - cp0, axis=1).max()),
                           joint_error_rad=float(np.abs(lg['q'][w].mean(0) - q0).max()), t_end=float(d.time - t0)))
    log = pose.log(n0)
    ok = [c for c in cycles if 'rejected' not in c]
    t = log['t'] - t0
    fig, ax = plt.subplots(2, 1, figsize=(6.4, 4.6), sharex=True)
    ax[0].plot(t, 1e3 * (log['cp'][:, 0, 0] - cp0[0, 0]), color=fs.LEGS['FL'], label='FL contact x')
    ax[0].plot(t, 1e3 * (log['p'][:, 0] - p0[0]), color=fs.C['ink'], lw=1, label='base x')
    ax[0].plot(t, 1e3 * (log['p'][:, 1] - p0[1]), color=fs.C['gt'], lw=1, label='base y')
    ax[0].set_ylabel('displacement [mm]'); ax[0].set_ylim(-55, 95); ax[0].legend(loc='upper center', ncol=3); ax[0].set_title('3.12 three step-out-and-back cycles')
    ax[1].plot(t, 1e3 * (log['p'][:, 2] - p0[2]), color=fs.C['est'], label='base height')
    ax[1].plot(t, deg(log['rpy'][:, 0] - r0[0]) * 10, color=fs.C['est2'], lw=1, label='roll x 10 [deg]')
    ax[1].plot(t, deg(log['rpy'][:, 1] - r0[1]) * 10, color=fs.C['plan'], lw=1, label='pitch x 10 [deg]')
    for c in ok:
        ax[0].axvline(c['t_end'], color=fs.C['gt'], lw=0.6); ax[1].axvline(c['t_end'], color=fs.C['gt'], lw=0.6)
    ax[1].set_ylabel('change [mm], [deg x 10]'); ax[1].set_xlabel('time [s]')
    ax[1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.32), ncol=3)
    fs.save(fig, FIG / '3-12_repeat')
    mx = lambda k: max(c[k] for c in ok) if ok else float('nan')  # noqa: E731
    return dict(cycles=cycles, n_cycles_completed=len(ok), max_base_position_error_m=mx('base_position_error_m'),
                max_attitude_error_deg=mx('attitude_error_deg'), max_foot_position_error_m=mx('foot_position_error_m'),
                max_joint_error_rad=mx('joint_error_rad'), foot_force_end_N=forces_end(log)), ['figures/3-12_repeat.png']


def contact_points(m, d):
    mujoco.mj_forward(m, d)
    return lib.contact_points(m, d)


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
    t0 = time.time()
    meas, outputs = TASKS[tid]()
    checks, ok = judge(tid, meas)
    out = dict(id=tid, title=ACC[tid]['title'], status='verified' if ok else 'failed', checks=checks, measured=meas,
               outputs=outputs, command=f'python run.py {tid}', mujoco=mujoco.__version__, wall_s=round(time.time() - t0, 1),
               load_avg_1min=round(os.getloadavg()[0], 1))
    RES.mkdir(exist_ok=True)
    (RES / f'{tid}.json').write_text(json.dumps(out, indent=2, default=float) + '\n')
    print(f'{tid} {out["status"]:8s} ' + ', '.join(f'{k}={v["value"]:.4g}' if isinstance(v['value'], float) else f'{k}={v["value"]}'
                                              for k, v in checks.items()), flush=True)
    return ok


if __name__ == '__main__':
    ids = list(TASKS) if sys.argv[1:] == ['all'] else sys.argv[1:]
    if not ids or any(i not in TASKS for i in ids):
        raise SystemExit(f'usage: python run.py <task-id>|all   ids: {" ".join(TASKS)}')
    for i in ids:
        run(i)
