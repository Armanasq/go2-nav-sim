#!/usr/bin/env python3
"""Experiment for module 06, all three levels.
  fundamental: MuJoCo's gyro, accelerometer, encoders and touch sensors against first-principles kinematics on a rig-driven
      motion and a static stand; statistics of the white noise, bias random walk and quantisation models; encoder quantisation
      during an open-loop trot.
  intermediate: Allan variance of 4 h static records, fitted noise parameters against the configured ones; gyro bias from static
      data against the predicted error; accelerometer calibration from 18 static rig poses with known orientation.
  advanced: orientation-free accelerometer calibration; time offset between two IMUs by cross-correlation; foot contact
      detection from noisy contact forces during the trot, error rate against the geometric contact.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--no-media]"""
import argparse, json, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
from scipy.signal import correlate  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402
import sensors as S  # noqa: E402  (adds the module 00 and 02 folders to sys.path)
from common import G, LEGS  # noqa: E402
from rotations import log_so3  # noqa: E402  (module 00)

FIG, MEDIA = HERE / 'figures', HERE / 'media'
DT = 0.002                                   # physics step (MuJoCo default, checked in run_log)
DT_IMU = S.IMU_RATE_DIV * DT                 # 250 Hz

# Configured sensor parameters (chosen for this module, of the order of a consumer MEMS IMU; not a Go2 datasheet).
GYRO = S.Inertial(N=1.0e-4, K=2.0e-5, q=np.deg2rad(2000) / 32768, limit=np.deg2rad(2000), b0=np.array([4e-3, -3e-3, 2e-3]))
M_ACC = np.diag([1.010, 0.992, 1.004]) @ (np.eye(3) + np.array([[0, 3e-3, -2e-3], [1e-3, 0, 4e-3], [2e-3, -1e-3, 0]]))
ACC = S.Inertial(N=1.0e-3, K=1.0e-4, q=16 * G / 32768, limit=16 * G, b0=np.array([0.05, -0.03, 0.08]), M=M_ACC)
ENC_RES = 2 * np.pi / 2 ** 14                # joint encoder step, rad
SIGMA_F = 3.0                                # foot force sensor white noise, N per sample at 500 Hz
OFFSET_TRUE = 0.006                          # s, delay of the lidar IMU: 3 physics steps, 1.5 IMU periods
TROT = dict(period=0.5, thigh=0.3, calf=-0.6, kp=60.0, kd=2.0)


def n_eff(ch, dt):
    """White density including per-sample quantisation (variance q^2/12 per sample): N_eff^2 = N^2 + q^2 dt / 12."""
    return np.sqrt(ch.N ** 2 + ch.q ** 2 * dt / 12)


# Acceptance thresholds, fixed before the first full run (MODULE_SPEC.md). (key, op, threshold). Keys in INFO are reported only.
ACCEPT = {
    # fundamental
    'gyro_vs_base_omega_max': ('<', 1e-12), 'quat_vs_base_quat_max': ('<', 1e-12), 'jointpos_vs_qpos_max': ('<', 1e-12),
    'jointvel_vs_qvel_max': ('<', 1e-12), 'touch_vs_foot_forces_max_N': ('<', 1e-6),
    'gyro_vs_rotation_fd_rel_rms': ('<', 1e-2), 'acc_vs_position_fd_rel_rms': ('<', 1e-2), 'lever_arm_rel_rms': ('<', 1e-2),
    'static_acc_norm_minus_g': ('<', 1e-3),
    'white_sd_rel_err': ('<', 0.01), 'bias_rw_var_rel_err': ('<', 0.10), 'quant_err_rms_rel_err': ('<', 0.02),
    'encoder_angle_err_rms_rel_err': ('<', 0.05), 'encoder_vel_quant_rms_rel_err': ('<', 0.10),
    # intermediate
    'allan_overlap_subset_identity': ('<', 1e-12), 'allan_white_rel_err_max': ('<', 0.05),
    'allan_gyro_N_rel_err': ('<', 0.03), 'allan_gyro_K_rel_err': ('<', 0.25),
    'allan_acc_N_rel_err': ('<', 0.03), 'allan_acc_K_rel_err': ('<', 0.25), 'allan_gazebo_N_rel_err': ('<', 0.03),
    'gyro_bias_mc_sd_ratio_max_dev': ('<', 0.15), 'gyro_bias_mujoco_err_in_sd': ('<', 3.0),
    'calib_known_M_err_max': ('<', 1e-3), 'calib_known_b_err_max': ('<', 3e-3),
    # advanced
    'calib_unknown_U_err_max': ('<', 2e-3), 'calib_unknown_b_err_max': ('<', 5e-3), 'calib_unknown_converged': ('>=', 1.0),
    'xcorr_vs_scipy_rel': ('<', 1e-10), 'time_offset_err_ms': ('<', 1.0), 'time_offset_sweep_max_err_ms': ('<', 1.0),
    'time_offset_with_transient_err_ms': ('<', 1.0),
    'contact_best_error_rate': ('<', 0.05), 'contact_hysteresis_toggle_ratio': ('<=', 1.1),
    'media_mp4_under_6MB': ('>=', 1.0),
}


def check(key, val):
    op, thr = ACCEPT[key]
    return bool({'<': val < thr, '<=': val <= thr, '>=': val >= thr}[op])


# ---- simulation logging --------------------------------------------------------------------------------------------------

def foot_dist(m, d, feet):
    """(4,) signed distance of each foot sphere to the floor from the contact list (+1 when no contact is listed)."""
    out = np.ones(4)
    for i in range(d.ncon):
        c = d.contact[i]
        for k, g in enumerate(feet):
            if g in (c.geom1, c.geom2):
                out[k] = min(out[k], c.dist)
    return out


def run_log(m, d, controller, duration, rec=None, rig=None):
    """sim.run with a logger. After each step mj_forward is called so sensordata, site poses and qpos all refer to d.time.
    rig(t) -> (pos, quat) drives the mocap body. Returns dict of arrays, one row per physics step."""
    assert abs(m.opt.timestep - DT) < 1e-15
    idx = sim.index(m); sid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n) for n in ('imu', 'imu_lidar')}
    L = {k: [] for k in ('t', 'sd', 'qpos', 'qvel', 'p_imu', 'p_lidar', 'R_imu', 'F', 'dist', 'rig_quat')}

    def on_step(m, d):
        if rig is not None:
            d.mocap_pos[0], d.mocap_quat[0] = rig(d.time)
        mujoco.mj_forward(m, d)
        for k, v in (('t', d.time), ('sd', d.sensordata), ('qpos', d.qpos), ('qvel', d.qvel), ('p_imu', d.site_xpos[sid['imu']]),
                     ('p_lidar', d.site_xpos[sid['imu_lidar']]), ('R_imu', d.site_xmat[sid['imu']].reshape(3, 3)),
                     ('F', sim.foot_forces(m, d)[:, 2]), ('dist', foot_dist(m, d, idx.feet)), ('rig_quat', d.mocap_quat[0])):
            L[k].append(np.array(v, copy=True))
        if rec is not None:
            rec(m, d)
    sim.run(m, d, controller, duration, on_step=on_step)
    out = {k: np.array(v) for k, v in L.items()}
    sl = S.sensor_slices(m)
    for n, s in sl.items():
        out[n] = out['sd'][:, s]
    out['q_enc'] = np.hstack([out[f'q_{l}_{p}'] for l in LEGS for p in ('hip', 'thigh', 'calf')])
    out['dq_enc'] = np.hstack([out[f'dq_{l}_{p}'] for l in LEGS for p in ('hip', 'thigh', 'calf')])
    out['touch'] = np.hstack([out[f'touch_{l}'] for l in LEGS])
    return out


def trot_qdes(t0):
    """Open-loop trot in place: diagonal pairs FL+RR and FR+RL alternate; a swing adds sin^2 bumps to thigh and calf."""
    P = TROT['period']

    def q(t):
        out = sim.Q_STAND.copy()
        if t < t0:
            return out
        ph = ((t - t0) % P) / P
        for i, leg in enumerate(LEGS):
            local = ph if leg in ('FL', 'RR') else (ph - 0.5) % 1.0
            s = np.sin(2 * np.pi * local) ** 2 if local < 0.5 else 0.0
            out[3 * i + 1] += TROT['thigh'] * s; out[3 * i + 2] += TROT['calf'] * s
        return out
    return q


# ---- fundamental ---------------------------------------------------------------------------------------------------------

def rig_motion(m, d):
    """Base welded to a mocap rig that moves along smooth sinusoids for 3 s. Checks MuJoCo's sensors against the kinematics."""
    sim.place_on_ground(m, d); d.eq_active[0] = 1

    def rig(t):                                          # amplitudes ramp in over 0.5 s (min-jerk) so the rig starts at rest
        e = np.clip(t / 0.5, 0, 1); e = 10 * e ** 3 - 15 * e ** 4 + 6 * e ** 5
        p = [e * 0.05 * np.sin(2 * np.pi * 1.0 * t), e * 0.04 * np.sin(2 * np.pi * 1.3 * t), 0.6 + e * 0.03 * np.sin(2 * np.pi * 0.7 * t)]
        rv = e * np.array([0.3 * np.sin(2 * np.pi * 0.5 * t), 0.2 * np.sin(2 * np.pi * 0.8 * t), 0.4 * np.sin(2 * np.pi * 0.3 * t)])
        return p, Rotation.from_rotvec(rv).as_quat(scalar_first=True)
    d.mocap_pos[0], d.mocap_quat[0] = rig(0.0)
    sim.run(m, d, sim.joint_pd(m, sim.Q_STAND), 0.5)          # let the weld pull the base onto the rig
    L = run_log(m, d, sim.joint_pd(m, sim.Q_STAND), 3.0, rig=lambda t: rig(t - 0.5))
    d.eq_active[0] = 0
    r = {}
    idx = sim.index(m)
    r['gyro_vs_base_omega_max'] = float(np.abs(L['gyro'] - L['qvel'][:, 3:6]).max())
    q = L['qpos'][:, 3:7] / np.linalg.norm(L['qpos'][:, 3:7], axis=1, keepdims=True)
    r['quat_vs_base_quat_max'] = float(np.abs(L['quat'] - q).max())
    r['jointpos_vs_qpos_max'] = float(np.abs(L['q_enc'] - L['qpos'][:, idx.qpos]).max())
    r['jointvel_vs_qvel_max'] = float(np.abs(L['dq_enc'] - L['qvel'][:, idx.qvel]).max())
    # gyro: MuJoCo's Euler step rotates by exp(h w_{k+1}), so w_k = Log(R_{k-1}^T R_k) / h exactly up to rounding
    R = L['R_imu']
    w_fd = np.array([log_so3(R[k - 1].T @ R[k]) / DT for k in range(1, len(R))])
    e = w_fd - L['gyro'][1:]
    r['gyro_vs_rotation_fd_rel_rms'] = float(np.sqrt((e ** 2).mean() / (L['gyro'] ** 2).mean()))
    # accelerometer: specific force R^T (a - g), a from the central second difference of the site position
    a = (L['p_imu'][2:] - 2 * L['p_imu'][1:-1] + L['p_imu'][:-2]) / DT ** 2
    f_fd = np.einsum('kji,kj->ki', R[1:-1], a - np.array([0, 0, -G]))
    e = f_fd - L['acc'][1:-1]
    r['acc_vs_position_fd_rel_rms'] = float(np.sqrt((e ** 2).mean() / (L['acc'][1:-1] ** 2).mean()))
    # lever arm: f_B = f_A + alpha x r + w x (w x r) in the base frame; alpha_k = (w_{k+1} - w_k) / h
    rr = np.einsum('kji,kj->ki', R, L['p_lidar'] - L['p_imu'])
    w = L['gyro']; al = np.zeros_like(w); al[:-1] = (w[1:] - w[:-1]) / DT
    f_B = L['acc'] + np.cross(al, rr) + np.cross(w, np.cross(w, rr))
    e = (f_B - L['acc_lidar'])[:-1]
    r['lever_arm_rel_rms'] = float(np.sqrt((e ** 2).mean() / (L['acc_lidar'][:-1] ** 2).mean()))
    r['lever_arm_m'] = rr.mean(0).tolist()
    r['lever_arm_acc_diff_rms'] = float(np.sqrt(((L['acc_lidar'] - L['acc']) ** 2).sum(1).mean()))
    return r, dict(t=L['t'] - L['t'][0], gyro=L['gyro'], w_fd=w_fd, acc=L['acc'], f_fd=f_fd, acc_lidar=L['acc_lidar'], f_B=f_B)


def static_stand(m, d, T=30.0):
    """Robot standing on the floor: settle 3 s, then log T s. Truth for the static IMU experiments."""
    sim.stand(m, d, settle=3.0)
    L = run_log(m, d, sim.joint_pd(m, sim.Q_STAND), T)
    g, a = L['gyro'][::S.IMU_RATE_DIV], L['acc'][::S.IMU_RATE_DIV]
    feet = sim.foot_positions(m, d)
    sim.run(m, d, sim.joint_pd(m, sim.Q_STAND), 60.0)                  # keep standing: does the creep stop?
    L2 = run_log(m, d, sim.joint_pd(m, sim.Q_STAND), 10.0)
    creep = {'static_pitch_rate_t93_103s': float(L2['gyro'][:, 1].mean()),
             'static_foot_xy_drift_60s_mm': float(np.abs(sim.foot_positions(m, d)[:, :2] - feet[:, :2]).max() * 1e3)}
    return dict(gyro=g, acc=a), {
        'static_gyro_truth_mean': g.mean(0).tolist(), 'static_gyro_truth_sd': g.std(0).tolist(),
        'static_acc_truth_mean': a.mean(0).tolist(), 'static_acc_norm_minus_g': float(abs(np.linalg.norm(a.mean(0)) - G)),
        'static_pitch_rate_first_10s': float(g[:2500, 1].mean()), 'static_pitch_rate_last_10s': float(g[-2500:, 1].mean()),
        'static_base_z_change_mm': float((L['qpos'][-1, 2] - L['qpos'][0, 2]) * 1e3),
        'static_pitch_change_deg': float(np.degrees(np.arcsin(-L['R_imu'][-1][2, 0]) - np.arcsin(-L['R_imu'][0][2, 0]))), **creep}


def noise_stats():
    """The discrete noise models reproduce their configured statistics (synthetic, fixed seed)."""
    rng = np.random.default_rng(1); r = {}
    w = S.Inertial(N=GYRO.N, K=0.0).corrupt(np.zeros((1_000_000, 3)), DT_IMU, rng)[0]
    r['white_sd_rel_err'] = float(np.abs(w.std(0) / (GYRO.N / np.sqrt(DT_IMU)) - 1).max())
    T = 100.0; n = int(T / DT_IMU)
    ends = np.array([S.Inertial(N=0.0, K=GYRO.K, b0=np.zeros(1)).corrupt(np.zeros((n, 1)), DT_IMU, rng)[1][-1, 0] for _ in range(2000)])
    r['bias_rw_var_rel_err'] = float(abs(ends.var() / (GYRO.K ** 2 * T) - 1))
    x = rng.uniform(-1, 1, 1_000_000)
    eq = S.quantise(x, GYRO.q) - x
    r['quant_err_rms_rel_err'] = float(abs(np.sqrt((eq ** 2).mean()) / (GYRO.q / np.sqrt(12)) - 1))
    # bias random walk curves for the figure: 200 realisations over 100 s
    paths = np.stack([S.Inertial(N=0.0, K=GYRO.K, b0=np.zeros(1)).corrupt(np.zeros((n, 1)), DT_IMU, rng)[1][:, 0] for _ in range(200)])
    return r, dict(eq=eq, paths=paths, T=T, n=n)


def encoder_check(L):
    """Encoder quantisation during the trot: angle error and the quantisation part of the backward-difference velocity."""
    qt = L['q_enc']; qm, vm = S.encoder(qt, ENC_RES, DT)
    ea = qm - qt
    ev = (np.diff(qm, axis=0) - np.diff(qt, axis=0)) / DT              # quantisation part of the velocity error
    etot = vm[1:] - L['dq_enc'][1:]                                      # against MuJoCo's instantaneous joint velocity
    return {'encoder_angle_err_rms_rel_err': float(abs(np.sqrt((ea ** 2).mean()) / (ENC_RES / np.sqrt(12)) - 1)),
            'encoder_vel_quant_rms_rel_err': float(abs(np.sqrt((ev ** 2).mean()) / (ENC_RES / (np.sqrt(6) * DT)) - 1)),
            'encoder_vel_quant_rms': float(np.sqrt((ev ** 2).mean())),
            'encoder_vel_quant_pred': float(ENC_RES / (np.sqrt(6) * DT)),
            'encoder_vel_total_rms': float(np.sqrt((etot ** 2).mean())),
            'encoder_vel_signal_rms': float(np.sqrt((L['dq_enc'] ** 2).mean())),
            'encoder_vel_quant_rms_hip': float(np.sqrt((ev[:, 0::3] ** 2).mean())),
            'encoder_vel_quant_rms_thigh_calf': float(np.sqrt((ev[:, np.r_[1:12:3, 2:12:3]] ** 2).mean())),
            'encoder_hip_speed_rms': float(np.sqrt((L['dq_enc'][:, 0::3] ** 2).mean())),
            'encoder_hip_step_per_sample_rms_in_res': float(np.sqrt((np.diff(qt[:, 0::3], axis=0) ** 2).mean()) / ENC_RES)}, \
        dict(ea=ea, ev=ev, vm=vm)


# ---- intermediate --------------------------------------------------------------------------------------------------------

def allan_experiment(static):
    """4 h static records: the noise models applied to the mean static truth of the MuJoCo stand (see README for why the
    4 h are not simulated step by step). Allan variance per axis, averaged over the three axes, fitted."""
    rng = np.random.default_rng(2); r = {}; curves = {}
    n = int(4 * 3600 / DT_IMU)
    ms = S.log_taus(n, DT_IMU)
    for name, ch, truth in (('gyro', GYRO, static['gyro'].mean(0)), ('acc', ACC, static['acc'].mean(0))):
        y = ch.corrupt(np.broadcast_to(truth, (n, 3)), DT_IMU, rng)[0]
        tau, av = S.allan(y, DT_IMU, ms)
        if name == 'gyro':
            m = 50
            sub = S.allan(y[:200_000], DT_IMU, [m])[1][0]
            yc = y[:200_000] - y[:200_000].mean(0)
            theta = np.concatenate([np.zeros((1, 3)), np.cumsum(yc, 0) * DT_IMU])
            d2 = theta[2 * m::m] - 2 * theta[m:-m:m] + theta[:-2 * m:m]
            ref = S.allan_nonoverlap(y[:200_000], DT_IMU, m)
            r['allan_overlap_subset_identity'] = float(np.abs((d2 ** 2).mean(0) / (2 * (m * DT_IMU) ** 2) / ref - 1).max())
            r['allan_overlap_vs_nonoverlap_rel'] = float(np.abs(sub / ref - 1).max())
        avm = av.mean(1)
        fit = S.allan_fit(tau, avm)
        Ne = n_eff(ch, DT_IMU)
        r[f'allan_{name}_fit'] = fit
        r[f'allan_{name}_N_cfg'], r[f'allan_{name}_N_eff'], r[f'allan_{name}_K_cfg'] = ch.N, float(Ne), ch.K
        r[f'allan_{name}_N_rel_err'] = float(abs(fit['N'] / Ne - 1))
        r[f'allan_{name}_N_rel_err_vs_unquantised'] = float(abs(fit['N'] / ch.N - 1))
        r[f'allan_{name}_K_rel_err'] = float(abs(fit['K'] / ch.K - 1))
        r[f'allan_{name}_tau_min_s'] = float(tau[np.argmin(avm)])
        curves[name] = dict(tau=tau, av=av, fit=fit, Neff=Ne, K=ch.K)
    # white noise alone: sigma^2 = N^2 / tau at every tau with >= 100 clusters
    y = S.Inertial(N=GYRO.N, K=0.0).corrupt(np.zeros((n // 4, 3)), DT_IMU, rng)[0]
    tau, av = S.allan(y, DT_IMU, S.log_taus(n // 4, DT_IMU, min_clusters=100))
    r['allan_white_rel_err_max'] = float(np.abs(av.mean(1) / (GYRO.N ** 2 / tau) - 1).max())
    r['allan_white_tau_max_s'] = float(tau[-1])
    # scatter of the estimator itself at that tau: 50 more seeds, one axis averaged over 3 as above
    sc = [S.allan(S.Inertial(N=GYRO.N, K=0.0).corrupt(np.zeros((n // 4, 3)), DT_IMU, np.random.default_rng(100 + s))[0],
                  DT_IMU, [int(round(tau[-1] / DT_IMU))])[1][0].mean() / (GYRO.N ** 2 / tau[-1]) - 1 for s in range(50)]
    r['allan_white_rel_err_sd_at_tau_max_50_seeds'] = float(np.std(sc))
    # Gazebo's IMU of this repository (go2_gazebo.urdf): 100 Hz, per-sample gaussian sd, no bias model
    sd_g, rate = gazebo_imu()
    dtg = 1 / rate; ng = int(3600 * rate)
    y = rng.standard_normal((ng, 3)) * sd_g
    tau_g, av_g = S.allan(y, dtg, S.log_taus(ng, dtg))
    fit_g = S.allan_fit(tau_g, av_g.mean(1))
    r['gazebo_gyro_sd_per_sample'], r['gazebo_rate_hz'] = sd_g, rate
    r['gazebo_gyro_N_from_sd'] = float(sd_g * np.sqrt(dtg)); r['allan_gazebo_fit'] = fit_g
    r['allan_gazebo_N_rel_err'] = float(abs(fit_g['N'] / (sd_g * np.sqrt(dtg)) - 1))
    curves['gazebo'] = dict(tau=tau_g, av=av_g.mean(1), N=sd_g * np.sqrt(dtg))
    return r, curves


def gazebo_imu():
    """Gyro noise sd and update rate of `imu_sensor` in the Gazebo URDF of module 02."""
    import xml.etree.ElementTree as ET
    import description as D
    s = [e for e in ET.parse(D.GAZEBO_URDF).getroot().iter('sensor') if e.get('type') == 'imu'][0]
    sd = {float(x.find('noise/stddev').text) for x in s.find('imu/angular_velocity')}
    assert len(sd) == 1
    return sd.pop(), float(s.find('update_rate').text)


def gyro_bias_experiment(static):
    """Bias = mean of static gyro over T. Monte Carlo on synthetic records, and once on the MuJoCo stand."""
    rng = np.random.default_rng(3); r = {}
    Ts = np.array([0.5, 1, 2, 5, 10, 20, 50, 100, 200])
    n = int(Ts[-1] / DT_IMU); ks = (Ts / DT_IMU).astype(int)
    errs = []
    for _ in range(4):                                                      # 4 x 100 realisations, one axis each
        x = np.zeros((n, 100))
        ch = S.Inertial(N=GYRO.N, K=GYRO.K, q=GYRO.q, b0=np.full(100, GYRO.b0[0]))
        y, b = ch.corrupt(x, DT_IMU, rng)
        cm = np.cumsum(y, axis=0)
        errs.append(np.stack([cm[k - 1] / k - b[k - 1] for k in ks]))
    errs = np.concatenate(errs, axis=1)
    sd_mc = errs.std(axis=1); sd_pred = S.bias_sd(n_eff(GYRO, DT_IMU), GYRO.K, Ts)
    r['gyro_bias_T_s'], r['gyro_bias_sd_mc'], r['gyro_bias_sd_pred'] = Ts.tolist(), sd_mc.tolist(), sd_pred.tolist()
    r['gyro_bias_mc_sd_ratio_max_dev'] = float(np.abs(sd_mc / sd_pred - 1).max())
    r['gyro_bias_T_opt_s'] = float(np.sqrt(3) * n_eff(GYRO, DT_IMU) / GYRO.K)
    # MuJoCo stand: 30 s of simulated truth, corrupted once
    y, b = GYRO.corrupt(static['gyro'], DT_IMU, np.random.default_rng(4))
    T = len(y) * DT_IMU
    err = y.mean(0) - b[-1]
    sd = S.bias_sd(n_eff(GYRO, DT_IMU), GYRO.K, T)
    r['gyro_bias_mujoco_T_s'] = T; r['gyro_bias_mujoco_err'] = err.tolist(); r['gyro_bias_mujoco_sd_pred'] = float(sd)
    r['gyro_bias_mujoco_err_in_sd'] = float(np.abs(err / sd).max())
    r['gyro_bias_mujoco_truth_mean_in_sd'] = float(np.abs(np.array(static['gyro']).mean(0) / sd).max())
    return r, dict(Ts=Ts, sd_mc=sd_mc, sd_pred=sd_pred)


def rig_poses(seed=5):
    """18 rig orientations: world up along +-x, +-y, +-z of the base and the 12 edge diagonals, random yaw each."""
    rng = np.random.default_rng(seed)
    ups = [s * e for e in np.eye(3) for s in (1, -1)]
    ups += [np.array(v) / np.sqrt(2) for i in range(3) for j in range(i + 1, 3) for si in (1, -1) for sj in (1, -1)
            for v in [np.eye(3)[i] * si + np.eye(3)[j] * sj]]
    out = []
    for u in ups:                                       # R^T e_z = u: R maps the body vector u onto world z
        R0 = Rotation.align_vectors([[0, 0, 1]], [u])[0]
        out.append(Rotation.from_euler('z', rng.uniform(-np.pi, np.pi)) * R0)
    return out


def calibration_rig(m, d, rec=None, move=0.4, hold=0.8, avg=0.6):
    """The base welded to the rig, which steps through rig_poses(); the accelerometer is averaged over the last `avg` s of each
    hold. Returns per-pose reference specific force (from the rig angle), MuJoCo truth, and the corrupted IMU average."""
    poses = rig_poses(); seg = move + hold
    sim.place_on_ground(m, d); d.eq_active[0] = 1
    keys = Rotation.concatenate([poses[0]] + poses)
    pos = np.array([0, 0, 0.6])

    def rig(t):
        i = min(max(int(t // seg), 0), len(poses) - 1); s = np.clip((t - i * seg) / move, 0, 1)
        s = np.clip(10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5, 0, 1)
        return pos, Slerp([0, 1], keys[[i, i + 1]])(s).as_quat(scalar_first=True)
    d.mocap_pos[0], d.mocap_quat[0] = rig(0.0)
    sim.run(m, d, sim.joint_pd(m, sim.Q_STAND), 0.5)
    t0 = d.time
    L = run_log(m, d, sim.joint_pd(m, sim.Q_STAND), seg * len(poses), rec=rec, rig=lambda t: rig(t - t0))
    d.eq_active[0] = 0
    t = L['t'] - t0
    acc_t = L['acc'][::S.IMU_RATE_DIV]; ti = t[::S.IMU_RATE_DIV]
    y, b = ACC.corrupt(acc_t, DT_IMU, np.random.default_rng(6))
    f_ref, f_true, f_meas, b_mean, sag = [], [], [], [], []
    for i, P in enumerate(poses):
        w = (ti > (i + 1) * seg - avg) & (ti <= (i + 1) * seg)
        f_ref.append(P.as_matrix().T @ np.array([0, 0, G])); f_true.append(acc_t[w].mean(0)); f_meas.append(y[w].mean(0))
        b_mean.append(b[w].mean(0))
        wl = (t > (i + 1) * seg - avg) & (t <= (i + 1) * seg)
        Rb = Rotation.from_quat(L['quat'][wl][-1], scalar_first=True)
        sag.append(np.degrees((P.inv() * Rb).magnitude()))
    return dict(f_ref=np.array(f_ref), f_true=np.array(f_true), f_meas=np.array(f_meas), b_mean=np.array(b_mean),
                sag=np.array(sag), t=ti, y=y, acc=acc_t)


def calib_experiment(C):
    r = {}
    M, b = S.calib_known(C['f_ref'], C['f_meas'])
    b_true = C['b_mean'].mean(0)
    r['calib_known_M_err_max'] = float(np.abs(M - ACC.M).max()); r['calib_known_b_err_max'] = float(np.abs(b - b_true).max())
    r['calib_known_M'] = M.tolist(); r['calib_known_b'] = b.tolist(); r['calib_b_true_mean'] = b_true.tolist()
    r['rig_sag_deg_max'] = float(C['sag'].max())
    r['rig_truth_vs_ref_max'] = float(np.abs(C['f_true'] - C['f_ref']).max())
    # orientation-free: upper-triangular factor of M^-1
    U, bu, res = S.calib_unknown(C['f_meas'], G)
    U_true, _ = S.triangular_factor(np.linalg.inv(ACC.M))
    r['calib_unknown_U_err_max'] = float(np.abs(U - U_true).max()); r['calib_unknown_b_err_max'] = float(np.abs(bu - b_true).max())
    r['calib_unknown_converged'] = float(res['status'] in ('grad', 'step')); r['calib_unknown_iters'] = int(res['iters'])
    r['calib_unknown_status'] = res['status']
    f_known = (C['f_meas'] - b) @ np.linalg.inv(M).T
    f_unk = (C['f_meas'] - bu) @ U.T
    nrm = lambda f: np.linalg.norm(f, axis=1) - G  # noqa: E731
    r['calib_norm_err_raw_max'] = float(np.abs(nrm(C['f_meas'])).max())
    r['calib_norm_err_known_max'] = float(np.abs(nrm(f_known)).max())
    r['calib_norm_err_unknown_max'] = float(np.abs(nrm(f_unk)).max())
    r['calib_dir_err_known_deg_max'] = float(np.degrees(np.arccos(np.clip(
        np.sum(f_known * C['f_ref'], 1) / np.linalg.norm(f_known, axis=1) / G, -1, 1))).max())
    # too few poses: the 6 axis-aligned poses only, unknown orientation (9 unknowns, 6 equations)
    U6, b6, res6 = S.calib_unknown(C['f_meas'][:6], G)
    r['calib_unknown_6pose_U_err_max'] = float(np.abs(U6 - U_true).max()); r['calib_unknown_6pose_status'] = res6['status']
    return r, dict(raw=nrm(C['f_meas']), known=nrm(f_known), unk=nrm(f_unk))


# ---- advanced ------------------------------------------------------------------------------------------------------------

def gait_run(m, d, rec=None, stand_s=1.0, trot_s=8.0):
    sim.stand(m, d, settle=0.5)
    t0 = d.time + stand_s
    L = run_log(m, d, sim.joint_pd(m, trot_qdes(t0), TROT['kp'], TROT['kd']), stand_s + trot_s, rec=rec)
    L['t_trot'] = t0
    return L


def time_offset_experiment(L):
    """Gyro of the base IMU (a) and of the lidar IMU (b), 250 Hz, b delayed: b_k = truth_b(t_k - d). |w| is used, so the
    relative mounting rotation does not have to be known."""
    rng = np.random.default_rng(7); r = {}
    wa, wb = L['gyro'], L['gyro_lidar']

    def pair(shift_steps, start):
        a = wa[start::S.IMU_RATE_DIV]; b = wb[start - shift_steps::S.IMU_RATE_DIV][:len(a)]
        a = GYRO.corrupt(a, DT_IMU, rng)[0]; b = GYRO.corrupt(b, DT_IMU, rng)[0]
        return np.linalg.norm(a, axis=1), np.linalg.norm(b, axis=1)
    k_trot = int(np.searchsorted(L['t'], L['t_trot']))
    start = k_trot + 50                                                # trot only, 0.1 s after its start
    a, b = pair(round(OFFSET_TRUE / DT), start)
    est, lags, c = S.xcorr_offset(a, b, DT_IMU, 25)
    r['time_offset_true_ms'] = OFFSET_TRUE * 1e3; r['time_offset_est_ms'] = est * 1e3
    r['time_offset_err_ms'] = float(abs(est - OFFSET_TRUE) * 1e3)
    # unnormalised sums against scipy.signal.correlate (lag convention check)
    a0, b0 = a - a.mean(), b - b.mean()
    full = correlate(b0, a0, mode='full'); zero = len(a0) - 1
    mine = np.array([a0[:len(a0) - k] @ b0[k:] if k >= 0 else a0[-k:] @ b0[:len(b0) + k] for k in lags])
    r['xcorr_vs_scipy_rel'] = float(np.abs(mine - full[zero + lags]).max() / np.abs(mine).max())
    errs = []
    sweep = list(range(0, 11))                                          # 0 .. 20 ms in physics steps
    for k in sweep:
        a_, b_ = pair(k, start)
        errs.append(S.xcorr_offset(a_, b_, DT_IMU, 25)[0] - k * DT)
    r['time_offset_sweep_true_ms'] = [k * DT * 1e3 for k in sweep]
    r['time_offset_sweep_err_ms'] = [e * 1e3 for e in errs]
    r['time_offset_sweep_max_err_ms'] = float(np.abs(errs).max() * 1e3)
    # failure case: a large offset (250 ms) on the periodic trot, search window +-0.6 s
    big = 125
    a_p, b_p = pair(big, k_trot + 1000)                                  # steady trot only (2 s after its start)
    est_p, lags_p, c_p = S.xcorr_offset(a_p, b_p, DT_IMU, 150)
    a_t, b_t = pair(big, 300)                                             # includes the stand and the trot start
    est_t, lags_t, c_t = S.xcorr_offset(a_t, b_t, DT_IMU, 150)
    pk = lambda lg, cc: sorted(cc[[i for i in range(1, len(cc) - 1) if cc[i] >= cc[i - 1] and cc[i] >= cc[i + 1]]])[-2:]  # noqa
    r['time_offset_periodic_true_ms'] = big * DT * 1e3
    r['time_offset_periodic_est_ms'] = est_p * 1e3; r['time_offset_periodic_err_ms'] = float(abs(est_p - big * DT) * 1e3)
    p2 = pk(lags_p, c_p); r['time_offset_periodic_peak_ratio'] = float(p2[0] / p2[1])
    r['time_offset_with_transient_est_ms'] = est_t * 1e3
    r['time_offset_with_transient_err_ms'] = float(abs(est_t - big * DT) * 1e3)
    p2 = pk(lags_t, c_t); r['time_offset_with_transient_peak_ratio'] = float(p2[0] / p2[1])
    return r, dict(lags=lags, c=c, est=est, sweep=sweep, errs=errs, lags_p=lags_p, c_p=c_p, est_p=est_p, lags_t=lags_t,
                   c_t=c_t, est_t=est_t)


def contact_experiment(L):
    """Truth: geometric contact, the foot sphere overlaps the floor (dist < 0). Sensor: MuJoCo's normal force plus white noise
    SIGMA_F at 500 Hz. Detector: threshold, and threshold with hysteresis."""
    rng = np.random.default_rng(8); r = {}
    k0 = int(np.searchsorted(L['t'], L['t_trot']))
    F, truth = L['F'][k0:], L['dist'][k0:] < 0
    r['touch_vs_foot_forces_max_N'] = float(np.abs(L['touch'] - L['F']).max())
    Fm = F + SIGMA_F * rng.standard_normal(F.shape)
    r['contact_truth_duty'] = truth.mean(0).tolist()
    r['contact_ideal_mismatch'] = float(((F > 0) != truth).mean())
    ths = np.arange(0.0, 60.5, 0.5)
    rate = np.array([(S.detect_contact(Fm, th) != truth).mean() for th in ths])
    fp = np.array([(S.detect_contact(Fm, th) & ~truth).mean() for th in ths])
    fn = np.array([(~S.detect_contact(Fm, th) & truth).mean() for th in ths])
    ideal = np.array([(S.detect_contact(F, th) != truth).mean() for th in ths])
    i = int(np.argmin(rate)); th = float(ths[i])
    r['contact_best_threshold_N'], r['contact_best_error_rate'] = th, float(rate[i])
    r['contact_best_fp'], r['contact_best_fn'] = float(fp[i]), float(fn[i])
    r['contact_error_rate_at_0p5N'] = float(rate[1]); r['contact_ideal_best'] = float(ideal.min())
    on, off = th + 2 * SIGMA_F, max(th - 2 * SIGMA_F, 0.5 * th)
    det_p, det_h = S.detect_contact(Fm, th), S.detect_contact(Fm, on, off)
    nt = S.toggles(truth).sum()
    r['contact_true_toggles'] = int(nt)
    r['contact_plain_toggle_ratio'] = float(S.toggles(det_p).sum() / nt)
    r['contact_hysteresis_on_off_N'] = [on, off]
    r['contact_hysteresis_error_rate'] = float((det_h != truth).mean())
    r['contact_hysteresis_toggle_ratio'] = float(S.toggles(det_h).sum() / nt)
    # where the errors are: distance in time to the nearest true transition
    edges = [np.flatnonzero(np.diff(truth[:, j].astype(int))) for j in range(4)]
    near = np.zeros_like(truth)
    for j in range(4):
        for e in edges[j]:
            near[max(e - 5, 0):e + 6, j] = True                       # +-10 ms
    err = det_h != truth
    r['contact_hysteresis_errors_within_10ms_of_transition'] = float((err & near).sum() / max(err.sum(), 1))
    r['contact_fn_force_quantiles_N'] = np.percentile(F[truth & (F < th)], [50, 90]).tolist() if (truth & (F < th)).any() else []
    return r, dict(t=L['t'][k0:] - L['t_trot'], F=F, Fm=Fm, truth=truth, det_p=det_p, det_h=det_h, ths=ths, rate=rate, fp=fp,
                   fn=fn, ideal=ideal, th=th, on=on, off=off)


# ---- figures -------------------------------------------------------------------------------------------------------------

def fig_diagram():
    fig, ax = fs.diagram(7.6, 3.4, (0, 7.6), (0, 3.4))
    y0 = 2.35
    fs.box(ax, (0.05, y0 - 0.3), 1.25, 0.6, 'MuJoCo truth\n$\\omega$, $f$ at site', fontsize=8)
    fs.box(ax, (1.65, y0 - 0.3), 1.05, 0.6, 'scale and\nmisalignment $M$', fontsize=8)
    fs.box(ax, (3.05, y0 - 0.3), 0.9, 0.6, '$+\\,b_k$\nbias walk', fontsize=8)
    fs.box(ax, (4.3, y0 - 0.3), 0.9, 0.6, '$+\\,n_k$\nwhite', fontsize=8)
    fs.box(ax, (5.55, y0 - 0.3), 0.85, 0.6, 'saturate,\nquantise', fontsize=8)
    for x0, x1 in ((1.3, 1.65), (2.7, 3.05), (3.95, 4.3), (5.2, 5.55), (6.4, 6.75)):
        fs.arrow(ax, (x0, y0), (x1, y0))
    ax.text(6.8, y0, '$y_k$', va='center')
    ax.text(3.5, y0 + 0.55, '$b_{k+1} = b_k + K\\sqrt{\\Delta t}\\,w_k$', ha='center', fontsize=8, color=C_MUTED)
    ax.text(4.75, y0 - 0.55, '$n_k \\sim \\mathcal{N}(0, N^2/\\Delta t)$', ha='center', fontsize=8, color=C_MUTED)
    ax.text(5.97, y0 + 0.55, 'step $q$', ha='center', fontsize=8, color=C_MUTED)
    y1 = 0.75
    fs.box(ax, (0.05, y1 - 0.3), 1.25, 0.6, 'joint angle $\\theta$', fontsize=8)
    fs.box(ax, (1.65, y1 - 0.3), 1.05, 0.6, 'quantise\nstep $\\delta$', fontsize=8)
    fs.box(ax, (3.05, y1 - 0.3), 0.9, 0.6, 'difference\n$/\\Delta t$', fontsize=8)
    fs.arrow(ax, (1.3, y1), (1.65, y1)); fs.arrow(ax, (2.7, y1), (3.05, y1)); fs.arrow(ax, (3.95, y1), (4.25, y1))
    ax.text(4.3, y1, '$\\hat\\theta$, $\\hat{\\dot\\theta}$', va='center')
    fs.box(ax, (5.0, y1 - 0.3), 0.9, 0.6, 'foot force\n$+\\,\\mathcal{N}(0,\\sigma_F^2)$', fontsize=8)
    fs.box(ax, (6.25, y1 - 0.3), 0.9, 0.6, 'threshold,\nhysteresis', fontsize=8)
    fs.arrow(ax, (5.9, y1), (6.25, y1))
    ax.text(0.05, 3.2, 'IMU (gyro, accelerometer), 250 Hz', fontsize=9, color=fs.C['ink'], weight='semibold')
    ax.text(0.05, 1.45, 'joint encoder, 500 Hz', fontsize=9, color=fs.C['ink'], weight='semibold')
    ax.text(5.0, 1.45, 'foot contact, 500 Hz', fontsize=9, color=fs.C['ink'], weight='semibold')
    fs.save(fig, FIG / 'sensor_model_diagram')


C_MUTED = fs.C['muted']


def fig_rig(R):
    fig, axs = plt.subplots(3, 1, figsize=(6.4, 5.6), sharex=True)
    t = R['t']
    for i, c in enumerate(('x', 'y', 'z')):
        axs[0].plot(t, R['gyro'][:, i], color=fs.C['gt'], lw=2.4, alpha=0.6, label='MuJoCo gyro' if i == 0 else None)
        axs[0].plot(t[1:], R['w_fd'][:, i], color=fs.C['est'], lw=0.9, ls='--', label='Log($R_{k-1}^T R_k$)/h' if i == 0 else None)
        axs[1].plot(t, R['acc'][:, i], color=fs.C['gt'], lw=2.4, alpha=0.6, label='MuJoCo accelerometer' if i == 0 else None)
        axs[1].plot(t[1:-1], R['f_fd'][:, i], color=fs.C['est'], lw=0.9, ls='--',
                    label='$R^T(\\ddot p - g)$, finite difference' if i == 0 else None)
        axs[2].plot(t, R['acc_lidar'][:, i] - R['acc'][:, i], color=fs.C['gt'], lw=2.4, alpha=0.6,
                    label='MuJoCo, lidar IMU minus base IMU' if i == 0 else None)
        axs[2].plot(t[:-1], (R['f_B'] - R['acc'])[:-1, i], color=fs.C['est'], lw=0.9, ls='--',
                    label='$\\alpha\\times r + \\omega\\times(\\omega\\times r)$' if i == 0 else None)
    axs[0].set_ylabel('angular rate [rad/s]'); axs[1].set_ylabel('specific force [m/s²]'); axs[1].set_ylim(top=17)
    axs[2].set_ylabel('difference [m/s²]'); axs[2].set_xlabel('time [s]')
    axs[0].set_title('Rig-driven base motion: MuJoCo sensors against kinematics (x, y, z axes)')
    for a in axs:
        a.legend(loc='upper right', ncol=2)
    fs.save(fig, FIG / 'mujoco_sensor_checks')


def fig_noise(N, E):
    fig, axs = plt.subplots(1, 3, figsize=(9.6, 3.2))
    t = np.arange(N['n']) * DT_IMU
    for p in N['paths'][:40]:
        axs[0].plot(t[::25], p[::25], color=fs.C['est'], lw=0.5, alpha=0.35)
    sd = np.sqrt(N['paths'].var(0))
    axs[0].plot(t[::25], 2 * sd[::25], color=fs.C['est2'], lw=1.4, label='±2 sd, 200 runs')
    axs[0].plot(t[::25], -2 * sd[::25], color=fs.C['est2'], lw=1.4)
    axs[0].plot(t, 2 * GYRO.K * np.sqrt(t), color=fs.C['plan'], ls='--', label='±2 K√t')
    axs[0].plot(t, -2 * GYRO.K * np.sqrt(t), color=fs.C['plan'], ls='--')
    axs[0].set_xlabel('time [s]'); axs[0].set_ylabel('gyro bias [rad/s]'); axs[0].set_title('Bias random walk')
    axs[0].legend(loc='upper left')
    q = GYRO.q
    axs[1].hist(N['eq'] / q, bins=60, density=True, color=fs.C['est'], alpha=0.7, label='quantiser error')
    axs[1].plot([-0.5, -0.5, 0.5, 0.5], [0, 1, 1, 0], color=fs.C['plan'], ls='--', label='uniform on ±q/2')
    axs[1].set_xlabel('error / q'); axs[1].set_ylabel('density'); axs[1].set_title('Quantisation, gyro LSB'); axs[1].legend()
    ev = E['ev'].ravel(); a = ENC_RES / DT
    axs[2].hist(ev, bins=np.linspace(-a, a, 41), density=True, color=fs.C['est'], alpha=0.7, label='12 joints, trot')
    axs[2].plot([-a, 0, a], [0, 1 / a, 0], color=fs.C['plan'], ls='--', label='triangle: difference of\ntwo uniform errors')
    axs[2].set_xlabel('velocity error from quantisation [rad/s]'); axs[2].set_ylabel('density')
    axs[2].set_title('Encoder velocity, 14 bit'); axs[2].legend(loc='upper left', fontsize=7.5)
    fs.save(fig, FIG / 'noise_models')


def fig_allan(Cv):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.8))
    for ax, name, unit in ((axs[0], 'gyro', 'rad/s'), (axs[1], 'acc', 'm/s²')):
        c = Cv[name]; tau = c['tau']
        for i in range(3):
            ax.loglog(tau, np.sqrt(c['av'][:, i]), color=fs.C['est'], lw=0.8, alpha=0.5, label='measured, 3 axes' if i == 0 else None)
        f = c['fit']
        ax.loglog(tau, np.sqrt(S.allan_model(tau, f['N'], f['K'], f['Q'])), color=fs.C['est2'], lw=1.3, label='fitted model')
        ax.loglog(tau, c['Neff'] / np.sqrt(tau), color=fs.C['plan'], ls='--', lw=1, label='configured $N_{eff}/\\sqrt{\\tau}$')
        ax.loglog(tau, c['K'] * np.sqrt(tau / 3), color=fs.C['plan'], ls=':', lw=1.2, label='configured $K\\sqrt{\\tau/3}$')
        ax.plot([1], [c['Neff']], 'o', color=fs.C['plan'], ms=4); ax.plot([3], [c['K']], 's', color=fs.C['plan'], ms=4)
        ax.set_xlabel('cluster time τ [s]'); ax.set_ylabel(f'Allan deviation [{unit}]')
        ax.set_title(('Gyro' if name == 'gyro' else 'Accelerometer') + ', 4 h static at 250 Hz')
    g = Cv['gazebo']
    axs[0].loglog(g['tau'], np.sqrt(g['av']), color=fs.C['gt'], lw=1.2, label='Gazebo IMU config, 1 h at 100 Hz')
    axs[0].legend(loc='lower left', fontsize=7.5); axs[1].legend(loc='lower left', fontsize=7.5)
    fs.save(fig, FIG / 'allan_deviation')


def fig_bias(B, r, static):
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(9.6, 3.4), gridspec_kw=dict(width_ratios=[1.2, 1]))
    tt = np.arange(len(static['gyro'])) * DT_IMU
    ax2.plot(tt, static['gyro'][:, 1] * 1e3, color=fs.C['gt'], lw=0.8, label='MuJoCo truth, pitch rate $\\omega_y$')
    ax2.axhline(0, color=fs.C['muted'], lw=0.6)
    ax2.axhspan(-r['gyro_bias_mujoco_sd_pred'] * 1e3, r['gyro_bias_mujoco_sd_pred'] * 1e3, color=fs.C['good'], alpha=0.15,
                label='±1 sd of the 30 s bias estimate')
    ax2.set_xlabel('time in the static record [s]'); ax2.set_ylabel('rate [mrad/s]'); ax2.legend(fontsize=7.5)
    ax2.set_title('Failure: the standing robot is not still')
    ax.loglog(B['Ts'], B['sd_pred'], color=fs.C['plan'], ls='--', label='predicted $\\sqrt{N^2/T + K^2T/3}$')
    ax.loglog(B['Ts'], B['sd_mc'], 'o-', color=fs.C['est'], ms=4, label='Monte Carlo, 400 runs')
    ax.axvline(r['gyro_bias_T_opt_s'], color=fs.C['muted'], lw=0.8)
    ax.text(r['gyro_bias_T_opt_s'] * 1.08, B['sd_pred'].max() * 0.8, f"$T^*$ = {r['gyro_bias_T_opt_s']:.1f} s", color=fs.C['muted'])
    ax.errorbar([r['gyro_bias_mujoco_T_s']], [np.abs(r['gyro_bias_mujoco_err']).max()], fmt='s', color=fs.C['est2'],
                label='MuJoCo stand, 30 s, max axis |error|')
    ax.set_xlabel('averaging time T [s]'); ax.set_ylabel('bias error sd [rad/s]'); ax.legend(fontsize=7.5)
    ax.set_title('Gyro bias from static data')
    fs.save(fig, FIG / 'gyro_bias_averaging')


def fig_calib(Nr, r):
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    x = np.arange(len(Nr['raw']))
    ax.bar(x - 0.27, Nr['raw'], 0.27, color=fs.C['gt'], label='raw')
    ax.bar(x, Nr['known'], 0.27, color=fs.C['est'], label='calibrated, known orientations')
    ax.bar(x + 0.27, Nr['unk'], 0.27, color=fs.C['est2'], label='calibrated, orientation-free')
    ax.set_xticks(x); ax.set_xlabel('rig pose'); ax.set_ylabel('|f| − g [m/s²]'); ax.legend(fontsize=7.5)
    ax.set_title('Accelerometer magnitude error per static pose')
    fs.save(fig, FIG / 'accel_calibration')


def fig_offset(T, r):
    fig, axs = plt.subplots(1, 3, figsize=(10.2, 3.3))
    ms = T['lags'] * DT_IMU * 1e3
    axs[0].plot(ms, T['c'], 'o-', color=fs.C['est'], ms=3)
    axs[0].axvline(OFFSET_TRUE * 1e3, color=fs.C['gt'], label='true 6 ms')
    axs[0].axvline(T['est'] * 1e3, color=fs.C['est2'], ls='--', label=f"estimate {T['est'] * 1e3:.2f} ms")
    axs[0].set_xlim(-30, 40); axs[0].set_xlabel('lag [ms]'); axs[0].set_ylabel('normalised correlation of |ω|')
    axs[0].set_title('Cross-correlation, trot'); axs[0].legend(fontsize=7.5)
    tr = np.array(T['sweep']) * DT * 1e3
    axs[1].plot(tr, np.array(T['errs']) * 1e3, 'o-', color=fs.C['est'], ms=4)
    axs[1].axhspan(-ACCEPT['time_offset_sweep_max_err_ms'][1], ACCEPT['time_offset_sweep_max_err_ms'][1], color=fs.C['good'], alpha=0.1)
    axs[1].axhline(0, color=fs.C['muted'], lw=0.6)
    axs[1].set_xlabel('true offset [ms]'); axs[1].set_ylabel('estimate − true [ms]'); axs[1].set_title('Offset sweep, 4 ms sampling')
    axs[2].plot(T['lags_p'] * DT_IMU, T['c_p'], color=fs.C['bad'], lw=1, label='steady trot only')
    axs[2].plot(T['lags_t'] * DT_IMU, T['c_t'], color=fs.C['est'], lw=1, label='with stand-to-trot start')
    axs[2].axvline(0.25, color=fs.C['gt'], label='true 250 ms')
    axs[2].set_xlabel('lag [s]'); axs[2].set_ylabel('normalised correlation'); axs[2].set_title('Failure: periodic motion')
    axs[2].legend(fontsize=7.5, loc='lower left')
    fs.save(fig, FIG / 'time_offset')


def fig_contact(Cc, r):
    fig, axs = plt.subplots(1, 2, figsize=(10.2, 3.5), gridspec_kw=dict(width_ratios=[1.5, 1]))
    t = Cc['t']; w = (t > 3.0) & (t < 4.5); j = 0
    axs[0].plot(t[w], Cc['Fm'][w, j], color=fs.C['est'], lw=0.6, label='measured force (σ = 3 N)')
    axs[0].plot(t[w], Cc['F'][w, j], color=fs.C['gt'], lw=1.2, label='MuJoCo normal force')
    axs[0].fill_between(t[w], -12, -4, where=Cc['truth'][w, j], color=fs.C['gt'], alpha=0.6, step='mid', label='true contact')
    axs[0].fill_between(t[w], -20, -12, where=Cc['det_h'][w, j], color=fs.C['est2'], alpha=0.6, step='mid', label='detected, hysteresis')
    axs[0].axhline(Cc['on'], color=fs.C['plan'], ls='--', lw=0.8); axs[0].axhline(Cc['off'], color=fs.C['plan'], ls=':', lw=0.8)
    axs[0].set_xlabel('time since trot start [s]'); axs[0].set_ylabel('FL normal force [N]'); axs[0].legend(fontsize=7, ncol=2, loc='upper right')
    axs[0].set_title('FL foot during the trot')
    axs[1].plot(Cc['ths'], Cc['rate'] * 100, color=fs.C['est'], label='total, noisy sensor')
    axs[1].plot(Cc['ths'], Cc['fp'] * 100, color=fs.C['bad'], lw=0.9, ls='--', label='false contact')
    axs[1].plot(Cc['ths'], Cc['fn'] * 100, color=fs.C['plan'], lw=0.9, ls='--', label='missed contact')
    axs[1].plot(Cc['ths'], Cc['ideal'] * 100, color=fs.C['gt'], lw=1, label='noise-free force')
    axs[1].axvline(Cc['th'], color=fs.C['muted'], lw=0.6)
    axs[1].set_xlabel('threshold [N]'); axs[1].set_ylabel('error rate [% of samples]'); axs[1].legend(fontsize=7.5)
    axs[1].set_ylim(0, 25); axs[1].set_title('Contact detection error, 4 feet, 8 s')
    fs.save(fig, FIG / 'contact_sensing')


# ---- main ----------------------------------------------------------------------------------------------------------------

def main(media=True):
    t_start = time.time()
    m, d = S.build_model()
    res = {}
    if media:
        fig_diagram()
    rr, Rg = rig_motion(m, d); res.update(rr)
    static, rs = static_stand(m, d); res.update(rs)
    rn, Nn = noise_stats(); res.update(rn)
    ra, Cv = allan_experiment(static); res.update(ra)
    rb, B = gyro_bias_experiment(static); res.update(rb)
    vids = []
    rec_c = sim.Recorder(m, MEDIA / 'calibration_rig.mp4', distance=1.6, azimuth=135, elevation=-15) if media else None
    Cal = calibration_rig(m, d, rec=rec_c)
    if rec_c is not None:
        vids.append(rec_c.close()); rec_c.still(d, MEDIA / 'calibration_rig.png'); rec_c.__exit__()
    rc, Nr = calib_experiment(Cal); res.update(rc)
    rec_g = sim.Recorder(m, MEDIA / 'trot_contacts.mp4', distance=1.4, azimuth=120, elevation=-15, contacts=True) if media else None
    L = gait_run(m, d, rec=rec_g)
    if rec_g is not None:
        vids.append(rec_g.close()); rec_g.still(d, MEDIA / 'trot_contacts.png'); rec_g.__exit__()
    p, _, R = sim.base_pose(d)
    res['trot_final_base_z'] = float(p[2]); res['trot_final_tilt_deg'] = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    re, E = encoder_check(L); res.update(re)
    rt, T = time_offset_experiment(L); res.update(rt)
    rk, Cc = contact_experiment(L); res.update(rk)
    sizes = [v.stat().st_size for v in vids if v is not None]
    res['media_bytes'] = sizes
    res['media_mp4_under_6MB'] = float(bool(sizes) and max(sizes) < 6e6) if media else float('nan')
    if media:
        fig_rig(Rg); fig_noise(Nn, E); fig_allan(Cv); fig_bias(B, res, static); fig_calib(Nr, res); fig_offset(T, res); fig_contact(Cc, res)
    res['wall_time_s'] = time.time() - t_start
    res['checks'] = {k: check(k, res[k]) for k in ACCEPT if k in res and not (isinstance(res[k], float) and np.isnan(res[k]))}
    res['passed'] = int(sum(res['checks'].values())); res['total'] = len(res['checks'])
    (HERE / 'results.json').write_text(json.dumps(res, indent=1, default=float))
    for k, ok in res['checks'].items():
        print(f"{'PASS' if ok else 'MISS'}  {k:45s} {res[k]:.4g}   ({ACCEPT[k][0]} {ACCEPT[k][1]})")
    print(f"{res['passed']} of {res['total']} checks pass; wall {res['wall_time_s']:.1f} s")
    return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--no-media', action='store_true')
    main(media=not ap.parse_args().no_media)
