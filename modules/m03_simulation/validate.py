#!/usr/bin/env python3
"""Experiment for module 03 (simulation), all three levels: controlled physics experiments with analytic answers.
  fundamental:  free fall of a sphere and of the Go2's centre of mass against the parabola and against the exact error of
                semi-implicit Euler; own explicit Euler, semi-implicit Euler, implicit-damping Euler and RK4 on an oscillator
                against the analytic energy factor per step and against MuJoCo's Euler, implicitfast, implicit and RK4.
  intermediate: energy drift of the tumbling Go2 (no gravity, no constraints) against timestep for the four integrators;
                a block on an incline against friction coefficient: threshold angle atan(mu), sliding acceleration, creep
                below the threshold, pyramidal against elliptic cones.
  advanced:     contact penetration against solref and solimp from the written-out soft-constraint model, checked row by
                row against MuJoCo's efc_* arrays, on a sphere and on the standing Go2's feet; actuator saturation on a
                knee bench against the closed-form torque-limited response and on the Go2 rising from a crouch; a tilting
                platform with three blocks and the Go2 (video).
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--no-media]"""
import argparse, json, os, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import physics as P  # noqa: E402
from common import G, LEGS  # noqa: E402

FIG, MEDIA = HERE / 'figures', HERE / 'media'
INTEGRATORS = ('Euler', 'implicitfast', 'implicit', 'RK4')
MJ_INT = dict(Euler=mujoco.mjtIntegrator.mjINT_EULER, implicitfast=mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
              implicit=mujoco.mjtIntegrator.mjINT_IMPLICIT, RK4=mujoco.mjtIntegrator.mjINT_RK4)
MUS = (0.2, 0.5, 0.8, 1.0)

# Acceptance thresholds, fixed before the first full run (MODULE_SPEC.md). key: (op, threshold)
ACCEPT = {
    # fundamental. Exact discrete answers are checked to round-off; the Go2 COM residual must be < 10% of the Euler offset.
    'ff_sphere_euler_vs_discrete_m': ('<', 1e-9), 'ff_sphere_rk4_vs_parabola_m': ('<', 1e-9),
    'ff_sphere_euler_offset_ratio_err': ('<', 1e-6), 'ff_sphere_euler_energy_ratio_err': ('<', 1e-6),
    'ff_go2_com_residual_m': ('<', 1e-3),
    'osc_explicit_euler_energy_rel_err': ('<', 1e-9), 'osc_rk4_energy_rel_err': ('<', 1e-9),
    'osc_symplectic_modified_energy_rel_drift': ('<', 1e-12), 'osc_symplectic_band_rel_err': ('<', 0.01),
    'osc_own_vs_mujoco_euler_m': ('<', 1e-12), 'osc_own_vs_mujoco_rk4_m': ('<', 1e-12),
    'osc_own_vs_mujoco_damped_euler_m': ('<', 1e-12), 'osc_own_vs_mujoco_damped_implicitfast_m': ('<', 1e-12),
    'osc_explicit_damping_growth_rel_err': ('<', 1e-9),
    # intermediate. Orders from theory (Euler 1, RK4 4); threshold angle within 0.5 deg; sliding acceleration within 2% g;
    # creep from the soft friction model within 10%; pyramidal cone at 45 deg follows mu/sqrt(2) within 0.5 deg.
    'energy_order_euler': ('range', (0.8, 1.2)), 'energy_order_implicitfast': ('range', (0.8, 1.2)),
    'energy_order_implicit': ('range', (0.8, 1.2)), 'energy_order_rk4': ('range', (3.5, 4.5)),
    'energy_implicitfast_equals_euler': ('<', 1e-10), 'energy_rk4_over_euler_at_2ms': ('<', 1e-2),
    'incline_threshold_err_deg_max': ('<', 0.5), 'incline_accel_err_over_g_max': ('<', 0.02),
    'incline_creep_rel_err_max': ('<', 0.10), 'incline_pyramid_45deg_threshold_err_deg_max': ('<', 0.5),
    # advanced
    'contact_model_vs_efc_rel_max': ('<', 1e-9), 'contact_invweight_vs_mujoco_rel': ('<', 1e-9),
    'penetration_sphere_rel_err_max': ('<', 1e-3), 'penetration_go2_rel_err_max': ('<', 0.02),
    'penetration_refsafe_rel_err': ('<', 1e-3),
    'bench_inertia_vs_mujoco_rel': ('<', 1e-9), 'bench_mujoco_force_clamped_Nm': ('<', 1e-12),
    'bench_saturated_vs_closed_form_rad': ('<', 1e-2), 'bench_saturated_vs_discrete_rad': ('<', 1e-9), 'bench_vs_reference_rad': ('<', 1e-2),
    'bench_sat_exit_time_err_s': ('<', 0.004),
    'rise_applied_over_limit_max': ('<=', 1.0), 'rise_demand_over_limit_max': ('>', 1.0),
    'platform_onset_err_deg_max': ('<', 1.0),
    'media_mp4_under_6MB': ('>=', 1.0),
}


def check(key, val):
    op, thr = ACCEPT[key]
    if op == 'range':
        return bool(thr[0] <= val <= thr[1])
    return bool({'<': val < thr, '<=': val <= thr, '>': val > thr, '>=': val >= thr}[op])


def model(xml):
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


# ==== fundamental ========================================================================================================

def free_fall_sphere(h=0.002, T=1.0, z0=10.0, mass=1.0):
    out, traj = {}, {}
    for integ in ('Euler', 'RK4'):
        m, d = model(P.sphere_xml(mass=mass, z0=z0, h=h, integrator=integ, floor=False))
        n = round(T / h); z, v = np.empty(n + 1), np.empty(n + 1); z[0], v[0] = d.qpos[2], d.qvel[2]
        for k in range(n):
            mujoco.mj_step(m, d); z[k + 1], v[k + 1] = d.qpos[2], d.qvel[2]
        traj[integ] = z
        t = np.arange(n + 1) * h
        zp, vp = P.free_fall(t, z0)
        if integ == 'Euler':
            out['ff_sphere_euler_vs_discrete_m'] = float(np.abs(z - (zp + P.free_fall_semi_implicit_offset(t, h))).max())
            out['ff_sphere_euler_offset_ratio_err'] = float(abs((z[-1] - zp[-1]) / P.free_fall_semi_implicit_offset(T, h) - 1))
            E = 0.5 * mass * v ** 2 + mass * G * z
            out['ff_sphere_euler_energy_ratio_err'] = float(abs((E[-1] - E[0]) / P.free_fall_semi_implicit_energy_error(T, h, mass) - 1))
            out['ff_sphere_euler_offset_at_1s_m'] = float(z[-1] - zp[-1])
        else:
            out['ff_sphere_rk4_vs_parabola_m'] = float(np.abs(z - zp).max())
    return out, traj


def free_fall_go2(h=0.002, T=1.0, seed=1):
    """Go2 without floor, joints given random velocities: internal forces cancel, so the COM follows the same parabola.
    The residual after removing the semi-implicit Euler offset measures how far the multibody step departs from it."""
    res = {}
    for hh in (h, h / 2, h / 4):
        m, d = sim.load(scene=False); m.opt.timestep = hh
        idx = sim.index(m)
        d.qpos[idx.qpos] = sim.Q_STAND
        r2 = np.random.default_rng(seed)
        d.qvel[idx.qvel] = r2.uniform(-2, 2, 12); d.qvel[3:6] = r2.uniform(-1, 1, 3); d.qvel[:3] = (0.3, -0.2, 1.0)
        mujoco.mj_forward(m, d)
        c0 = d.subtree_com[idx.base].copy()
        mujoco.mj_subtreeVel(m, d); vc0 = d.subtree_linvel[idx.base].copy()
        n = round(T / hh); err = np.empty(n + 1); dev = np.empty(n + 1); t = np.arange(n + 1) * hh
        for k in range(n + 1):
            if k:
                mujoco.mj_step(m, d); mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
            c = d.subtree_com[idx.base]
            pred = c0 + vc0 * t[k] + np.array([0, 0, -0.5 * G * t[k] ** 2 + P.free_fall_semi_implicit_offset(t[k], hh)])
            err[k] = np.linalg.norm(c - pred)
            dev[k] = c[2] - (c0[2] + vc0[2] * t[k] - 0.5 * G * t[k] ** 2)
        res[hh] = dict(t=t, residual=err, dev=dev)
    return {'ff_go2_com_residual_m': float(res[h]['residual'].max()),
            'ff_go2_com_residual_by_h': {f'{k * 1e3:g}ms': float(v['residual'].max()) for k, v in res.items()},
            'ff_go2_com_offset_at_1s_m': float(res[h]['dev'][-1])}, res


def oscillator(w=10.0, h=0.01, T=20.0, b=0.5, mass=1.0):
    """x'' = -w^2 x (k = m w^2). Own integrators against the analytic energy factor and against MuJoCo."""
    k = mass * w ** 2
    n = round(T / h); z = w * h
    acc = lambda x, v: -w ** 2 * x  # noqa: E731
    E = lambda tr: 0.5 * (tr[:, 1] ** 2 + w ** 2 * tr[:, 0] ** 2)  # noqa: E731
    own = {s: P.integrate(getattr(P, f'step_{s}'), 1.0, 0.0, h, n, acc) for s in ('explicit_euler', 'semi_implicit_euler', 'rk4')}
    kk = np.arange(n + 1)
    out = {}
    for s in ('explicit_euler', 'rk4'):
        pred = P.oscillator_energy_factor(z, s) ** kk
        out[f'osc_{s}_energy_rel_err'] = float(np.abs(E(own[s]) / E(own[s])[0] / pred - 1).max())
    tr = own['semi_implicit_euler']
    Em = P.symplectic_modified_energy(tr[:, 0], tr[:, 1], w, h)
    out['osc_symplectic_modified_energy_rel_drift'] = float(np.abs(Em / Em[0] - 1).max())
    band = np.abs(E(tr) / E(tr)[0] - 1).max()
    out['osc_symplectic_band_rel_err'] = float(abs(band / P.symplectic_energy_band(z) - 1))
    out['osc_symplectic_band_measured'] = float(band); out['osc_z'] = z

    def mj(integ, bb, flags=0):
        m, d = model(P.oscillator_xml(mass, k, bb, h, integ)); d.qpos[0] = 1.0
        m.opt.disableflags |= flags
        x = np.empty((n + 1, 2)); x[0] = 1.0, 0.0
        for i in range(n):
            mujoco.mj_step(m, d); x[i + 1] = d.qpos[0], d.qvel[0]
        return x
    out['osc_own_vs_mujoco_euler_m'] = float(np.abs(mj('Euler', 0.0)[:, 0] - tr[:, 0]).max())
    out['osc_own_vs_mujoco_rk4_m'] = float(np.abs(mj('RK4', 0.0)[:, 0] - own['rk4'][:, 0]).max())
    damped = P.integrate(P.step_euler_implicit_damping, 1.0, 0.0, h, n, lambda x: -k * x, mass, b)
    out['osc_own_vs_mujoco_damped_euler_m'] = float(np.abs(mj('Euler', b)[:, 0] - damped[:, 0]).max())
    out['osc_own_vs_mujoco_damped_implicitfast_m'] = float(np.abs(mj('implicitfast', b)[:, 0] - damped[:, 0]).max())
    # failure case: a stiff damper (b h / m = 3 > 2) integrated explicitly (MuJoCo with eulerdamp disabled) diverges by the
    # factor |1 - h b / m| per step on the velocity (spring removed, k = 0); with implicit damping it decays by 1/(1 + h b/m)
    bs = 3 * mass / h; ns = 20
    def damper(flags):
        m, d = model(P.oscillator_xml(mass, 0.0, bs, h, 'Euler')); d.qvel[0] = 1.0; m.opt.disableflags |= flags
        v = [1.0]
        for _ in range(ns):
            mujoco.mj_step(m, d); v.append(d.qvel[0])
        return np.array(v)
    v_exp, v_imp = damper(mujoco.mjtDisableBit.mjDSBL_EULERDAMP), damper(0)
    pred_exp, pred_imp = (1 - h * bs / mass) ** np.arange(ns + 1), (1 / (1 + h * bs / mass)) ** np.arange(ns + 1)
    out['osc_explicit_damping_growth_rel_err'] = float(max(np.abs(v_exp / pred_exp - 1).max(), np.abs(v_imp / pred_imp - 1).max()))
    out['osc_explicit_damping_v_after_20_steps'] = float(v_exp[-1])
    out['osc_implicit_damping_v_after_20_steps'] = float(v_imp[-1])
    traj = dict(t=kk * h, own=own, E=E, w=w, z=z, v_exp=v_exp, v_imp=v_imp)
    return out, traj


# ==== intermediate =======================================================================================================

def go2_energy(hs=(0.00025, 0.0005, 0.001, 0.002, 0.004), T=2.0, seed=0):
    """Tumbling Go2: gravity, damping, friction loss, limits and contacts off, so kinetic energy is the only energy and is
    conserved. Max |E(t) - E0| / E0 over T for each integrator and step."""
    rng = np.random.default_rng(seed)
    qd0 = np.concatenate([[0, 0, 0], rng.uniform(-1, 1, 3), rng.uniform(-2, 2, 12)])
    drift, traces = {}, {}
    for integ in INTEGRATORS:
        drift[integ] = []
        for h in hs:
            m, d = sim.load(scene=False)
            m.opt.timestep, m.opt.integrator = h, MJ_INT[integ]
            m.opt.gravity[:] = 0; m.dof_damping[:] = 0; m.dof_frictionloss[:] = 0
            m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
            m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
            d.qpos[sim.index(m).qpos] = sim.Q_STAND; d.qvel[:] = qd0
            mujoco.mj_forward(m, d); E0 = d.energy.sum()
            n = round(T / h); every = max(1, round(0.004 / h)); e = []
            for k in range(n):
                mujoco.mj_step(m, d)
                if (k + 1) % every == 0:
                    mujoco.mj_forward(m, d); e.append(d.energy.sum() / E0 - 1)
            e = np.array(e); drift[integ].append(float(np.abs(e).max()))
            if h == 0.002:
                traces[integ] = (np.arange(1, len(e) + 1) * every * h, e)
    # own kinetic energy 0.5 v^T M v against MuJoCo's at the initial state (checks what is being measured)
    m, d = sim.load(scene=False); d.qpos[sim.index(m).qpos] = sim.Q_STAND; d.qvel[:] = qd0
    m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY; m.opt.gravity[:] = 0; mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M)
    out = {f'energy_order_{k.lower()}': P.fit_order(hs, v) for k, v in drift.items()}
    out['energy_drift'] = {k: dict(zip([f'{h * 1e3:g}ms' for h in hs], v)) for k, v in drift.items()}
    out['energy_implicitfast_equals_euler'] = float(np.abs(np.array(drift['implicitfast']) - np.array(drift['Euler'])).max()
                                                    / max(drift['Euler']))
    i2 = list(hs).index(0.002)
    out['energy_rk4_over_euler_at_2ms'] = drift['RK4'][i2] / drift['Euler'][i2]
    out['energy_E0_J'] = float(d.energy.sum()); out['energy_own_kinetic_vs_mujoco_J'] = float(abs(0.5 * qd0 @ M @ qd0 - d.energy[1]))
    return out, dict(hs=np.array(hs), drift=drift, traces=traces)


def slide_trial(m, theta, phi=0.0, T0=0.5, T1=2.0, v0=0.0):
    """Gravity tilted by theta about the axis at phi + 90 deg: the plane becomes an incline whose downhill direction is phi
    from the contact frame's first tangent. Returns (acceleration along the slope by least squares over [T0, T1], speed at T1)."""
    d = mujoco.MjData(m)
    down = np.array([np.cos(phi), np.sin(phi)])
    m.opt.gravity[:] = (*(G * np.sin(theta) * down), -G * np.cos(theta))
    # start at the analytic resting depth to avoid a drop
    d.qpos[2] = -P.steady_penetration(m.body_mass[1] * G * np.cos(theta), 1 / m.body_mass[1], m.geom_solref[1], m.geom_solimp[1])
    d.qvel[:2] = v0 * down
    ts, vs = [], []
    for k in range(round(T1 / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if d.time >= T0 - 1e-9:
            ts.append(d.time); vs.append(d.qvel[:2] @ down)
    a = np.polyfit(ts, vs, 1)[0]
    return float(a), float(vs[-1]), d


def incline(mus=MUS):
    out = {'threshold_deg': {}, 'accel': {}, 'creep': {}}
    err_th, err_a, err_creep, err_pyr = [], [], [], []
    curves = {}
    for cone, ir in (('pyramidal', 1.0), ('elliptic', 1.0), ('elliptic', 100.0)):
        key = f'{cone}_ir{ir:g}'
        for mu in mus:
            m = mujoco.MjModel.from_xml_string(P.slider_xml(mu, cone, ir))
            for phi in ((0.0, np.pi / 4) if cone == 'pyramidal' else (0.0,)):
                mu_eff = mu * (P.pyramid_friction_ratio(phi) if cone == 'pyramidal' else 1.0)
                th_pred = P.incline_threshold(mu_eff)
                lo, hi = th_pred - np.radians(5), th_pred + np.radians(5)
                for _ in range(14):                      # bisection on 'accelerates' (> 0.01 m/s^2): 10 deg / 2^14 = 6e-4 deg
                    mid = 0.5 * (lo + hi)
                    lo, hi = (lo, mid) if slide_trial(m, mid, phi)[0] > 0.01 else (mid, hi)
                th_meas = 0.5 * (lo + hi)
                tag = f'{key}_mu{mu:g}' + ('_45deg' if phi else '')
                out['threshold_deg'][tag] = dict(measured=float(np.degrees(th_meas)), predicted=float(np.degrees(th_pred)),
                                                 atan_mu=float(np.degrees(np.arctan(mu))))
                (err_pyr if phi else err_th).append(abs(np.degrees(th_meas - th_pred)))
                if phi:
                    continue
                # sliding acceleration above the threshold, and creep below it
                ths = th_pred + np.radians(np.array([-10, -5, -2, 2, 5, 10, 20]))
                ths = ths[(ths > 0.02) & (ths < np.radians(80))]
                meas = np.array([slide_trial(m, t)[0] for t in ths])
                pred = P.incline_acceleration(ths, mu)
                sl = pred > 0
                err_a.append(float(np.abs(meas[sl] - pred[sl]).max() / G))
                curves[(key, mu)] = (np.degrees(ths), meas, pred)
                th_c = 0.9 * th_pred
                a_c, v_c, d = slide_trial(m, th_c)
                rec = dict(theta_deg=float(np.degrees(th_c)), speed_m_s=v_c)
                if cone == 'elliptic':
                    r = d.efc_pos[0] - d.efc_margin[0]
                    v_pred = P.creep_speed(m.body_mass[1] * G * np.sin(th_c), 1 / m.body_mass[1], r, m.geom_solref[1],
                                           m.geom_solimp[1], ir)
                    rec['predicted_m_s'] = float(v_pred); err_creep.append(abs(v_c / v_pred - 1))
                out['creep'][f'{key}_mu{mu:g}'] = rec
    out['incline_threshold_err_deg_max'] = float(max(err_th))
    out['incline_accel_err_over_g_max'] = float(max(err_a))
    out['incline_creep_rel_err_max'] = float(max(err_creep))
    out['incline_pyramid_45deg_threshold_err_deg_max'] = float(max(err_pyr))
    return out, curves


# ==== advanced ===========================================================================================================

def contact_rows(m, d, h):
    """Own K, B, d(r), d'(r) not compared (MuJoCo-internal), R and a_ref for every contact normal row, against efc_*."""
    errs = []
    for i in range(d.ncon):
        c = d.contact[i]; a = c.efc_address
        r, v = d.efc_pos[a] - d.efc_margin[a], d.efc_vel[a]
        K, B = P.stiffness_damping(c.solref, c.solimp, h)
        imp = P.impedance(r, c.solimp)
        R = P.regulariser(r, c.solimp, d.efc_diagA[a])
        aref = P.reference_acceleration(r, v, c.solref, c.solimp, h)
        for own, ref in ((K, d.efc_KBIP[a, 0]), (B, d.efc_KBIP[a, 1]), (imp, d.efc_KBIP[a, 2]), (R, d.efc_R[a]),
                         (aref, d.efc_aref[a])):
            errs.append(abs(own - ref) / max(abs(ref), 1e-12))
    return max(errs)


SPHERE_CASES = [(tc, imp) for tc in (0.005, 0.01, 0.02, 0.04, 0.08)
                for imp in ((0.9, 0.95, 0.001, 0.5, 2), (0.9, 0.9, 0.001, 0.5, 2), (0.5, 0.99, 0.01, 0.5, 2),
                            (0.015, 1.0, 0.022, 0.5, 2))]


def settle_sphere(tc, imp, h=0.002, T=2.0, mass=1.0, refsafe=True):
    m, d = model(P.sphere_xml(mass=mass, z0=0.05, h=h, solref=(tc, 1.0), solimp=imp))
    if not refsafe:
        m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_REFSAFE
    for _ in range(round(T / h)):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    return m, d, 0.05 - d.qpos[2]


def penetration_sphere():
    rows, errs, model_errs = [], [], []
    for tc, imp in SPHERE_CASES:
        m, d, depth = settle_sphere(tc, imp)
        pred = P.steady_penetration(G, 1.0, (tc, 1.0), imp)
        model_errs.append(contact_rows(m, d, 0.002))
        errs.append(abs(depth / pred - 1)); rows.append(dict(timeconst=tc, solimp=imp, measured_m=depth, predicted_m=pred))
    # refsafe: a timeconst below 2 h is raised to 2 h. Asking for 2 ms at h = 2 ms gives the 4 ms answer.
    imp = (0.9, 0.95, 0.001, 0.5, 2)
    _, _, depth = settle_sphere(0.002, imp)
    p_safe, p_nom = P.steady_penetration(G, 1.0, (0.002, 1.0), imp), P.steady_penetration(G, 1.0, (0.002, 1.0), imp, refsafe=False)
    _, d_un, depth_un = settle_sphere(0.002, imp, refsafe=False)
    out = {'penetration_sphere_rel_err_max': float(max(errs)), 'penetration_sphere_cases': rows,
           'penetration_refsafe_rel_err': float(abs(depth / p_safe - 1)),
           'refsafe': dict(asked_timeconst=0.002, measured_m=depth, predicted_refsafe_m=p_safe, predicted_nominal_m=p_nom,
                           measured_refsafe_off_m=float(depth_un), refsafe_off_vertical_speed=float(abs(d_un.qvel[2])))}
    return out, max(model_errs), rows


def go2_feet(variants=(('Menagerie foot', None), ('MuJoCo default solimp', (0.9, 0.95, 0.001, 0.5, 2)),
                       ('foot solimp, timeconst 0.01', 'tc0.01'))):
    """Standing Go2: foot penetration measured (radius - centre height) against steady_penetration with the measured normal
    force and the inverse weight computed here from J and M at qpos0."""
    out, errs, mdl = {'variants': {}}, [], []
    m0, d0 = sim.load(); idx = sim.index(m0)
    d0.qpos[:] = m0.qpos0; mujoco.mj_forward(m0, d0)
    M = np.zeros((m0.nv, m0.nv)); mujoco.mj_fullM(m0, d0, M)
    iw = []
    for g in idx.feet:
        b = m0.geom_bodyid[g]; jp = np.zeros((3, m0.nv)); mujoco.mj_jacBodyCom(m0, d0, jp, None, b)
        iw.append(P.inverse_weight(jp, M))
    iw = np.array(iw)
    out['contact_invweight_vs_mujoco_rel'] = float(np.abs(iw / m0.body_invweight0[m0.geom_bodyid[idx.feet], 0] - 1).max())
    for name, v in variants:
        m, d = sim.load()
        if isinstance(v, tuple):
            m.geom_solimp[idx.feet] = v
        elif v == 'tc0.01':
            m.geom_solref[idx.feet, 0] = 0.01
        sim.stand(m, d, settle=2.0); mujoco.mj_forward(m, d)
        mdl.append(contact_rows(m, d, m.opt.timestep))
        F = sim.foot_forces(m, d)[:, 2]
        meas = sim.FOOT_RADIUS - sim.foot_positions(m, d)[:, 2]
        pred = np.array([P.steady_penetration(F[i], iw[i], m.geom_solref[g], m.geom_solimp[g], m.geom_margin[g])
                         for i, g in enumerate(idx.feet)])
        errs.append(np.abs(meas / pred - 1).max())
        out['variants'][name] = dict(measured_mm=(meas * 1e3).tolist(), predicted_mm=(pred * 1e3).tolist(),
                                     normal_force_N=F.tolist(), base_height_m=float(d.qpos[2]))
    out['penetration_go2_rel_err_max'] = float(max(errs))
    out['go2_invweight_calf'] = iw.tolist()
    return out, max(mdl)


def knee_bench(kp=300.0, kd=5.0, q0=-2.6, q1=-1.0, T=0.25, h=0.002):
    m_go2, _ = sim.load(scene=False)
    m, d = model(P.knee_bench_xml(m_go2, h=h))
    tau_max = m.actuator_ctrlrange[0, 1]; b, fl, arm = m.dof_damping[0], m.dof_frictionloss[0], m.dof_armature[0]
    R = np.empty(9); mujoco.mju_quat2Mat(R, m.body_iquat[1]); R = R.reshape(3, 3)
    I_own = P.inertia_about_axis(m.body_mass[1], m.body_ipos[1], R @ np.diag(m.body_inertia[1]) @ R.T, m.jnt_axis[0], arm)
    mujoco.mj_forward(m, d); M = np.zeros((1, 1)); mujoco.mj_fullM(m, d, M)
    out = {'bench_inertia_vs_mujoco_rel': float(abs(I_own / M[0, 0] - 1)), 'bench_inertia_kgm2': I_own,
           'bench_tau_max_Nm': float(tau_max), 'bench_damping': float(b), 'bench_frictionloss_Nm': float(fl)}
    # MuJoCo's own clamp: a control of 300 N m produces the range limit
    d.ctrl[0] = 300.0; mujoco.mj_forward(m, d)
    out['bench_mujoco_force_clamped_Nm'] = float(abs(d.actuator_force[0] - tau_max))

    def run(limit):
        m2, d2 = model(P.knee_bench_xml(m_go2, h=h))
        if not limit:
            m2.actuator_ctrlrange[:] = (-1e6, 1e6)
        d2.qpos[0] = q0
        n = round(T / h); t = np.arange(n + 1) * h; q = np.empty(n + 1); dq = np.empty(n + 1); tau_d = np.empty(n); tau_a = np.empty(n)
        q[0], dq[0] = q0, 0.0
        for k in range(n):
            tau_d[k] = kp * (q1 - d2.qpos[0]) - kd * d2.qvel[0]
            d2.ctrl[0] = tau_d[k]; mujoco.mj_step(m2, d2)
            tau_a[k] = d2.actuator_force[0]; q[k + 1], dq[k + 1] = d2.qpos[0], d2.qvel[0]
        return t, q, dq, tau_d, tau_a
    t, q, dq, tau_d, tau_a = run(True)
    tu, qu, dqu, tau_du, _ = run(False)
    sat = np.abs(tau_d) >= tau_max
    k_exit = int(np.argmin(sat))                          # first unsaturated control step
    q_cf, dq_cf = P.saturated_response(t, I_own, tau_max - fl, b, q0)
    out['bench_saturated_vs_closed_form_rad'] = float(np.abs(q[:k_exit + 1] - q_cf[:k_exit + 1]).max())
    q_dc = P.saturated_response_discrete(np.arange(k_exit + 1), h, I_own, tau_max - fl, b, q0)[0]
    out['bench_saturated_vs_discrete_rad'] = float(np.abs(q[:k_exit + 1] - q_dc).max())
    ref = P.pd_saturated_reference(T, I_own, b, fl, kp, kd, q1, tau_max, q0)
    tt = t[t <= ref.t[-1]]
    out['bench_vs_reference_rad'] = float(np.abs(q[:len(tt)] - ref.sol(tt)[0]).max())
    # predicted exit from saturation: kp (q1 - q) - kd q' = tau_max on the closed-form branch
    from scipy.optimize import brentq
    t_exit = brentq(lambda s: kp * (q1 - P.saturated_response(s, I_own, tau_max - fl, b, q0)[0])
                    - kd * P.saturated_response(s, I_own, tau_max - fl, b, q0)[1] - tau_max, 1e-6, T)
    out['bench_sat_exit_time_pred_s'] = float(t_exit); out['bench_sat_exit_time_meas_s'] = float(t[k_exit])
    out['bench_sat_exit_time_err_s'] = float(abs(t[k_exit] - t_exit))
    out['bench_peak_demand_Nm'] = float(np.abs(tau_d).max())
    out['bench_applied_max_Nm'] = float(np.abs(tau_a).max())
    out['bench_terminal_speed_pred_rad_s'] = float((tau_max - fl) / b)
    out['bench_peak_speed_meas_rad_s'] = float(dq.max())
    ts90 = lambda qq: float(t[np.argmax(qq >= q0 + 0.9 * (q1 - q0))])  # noqa: E731
    out['bench_t90_limited_s'], out['bench_t90_unlimited_s'] = ts90(q), ts90(qu)
    out['bench_peak_demand_unlimited_Nm'] = float(np.abs(tau_du).max())
    return out, dict(t=t, q=q, dq=dq, tau_d=tau_d, tau_a=tau_a, q_cf=q_cf, k_exit=k_exit, t_exit=t_exit, tu=tu, qu=qu,
                     tau_du=tau_du, ref=ref, tau_max=tau_max, q1=q1)


def go2_rise(kp=150.0, kd=4.0, T=1.0):
    """Go2 from a crouch to the standing pose by a joint PD step (no trajectory): with the Menagerie torque limits and with
    the limits removed. Same gains, same start."""
    crouch = np.tile([0.0, 1.25, -2.6], 4)
    res = {}
    for name, limited in (('limited', True), ('unlimited', False)):
        m, d = sim.load(); idx = sim.index(m)
        lim = m.actuator_ctrlrange[idx.act, 1].copy()
        if not limited:
            m.actuator_ctrlrange[:] = (-1e4, 1e4)
        sim.place_on_ground(m, d, crouch)
        pd, demand = sim.joint_pd(m, sim.Q_STAND, kp, kd), [None]

        def ctrl(m, d):
            demand[0] = pd(m, d); return demand[0]
        log = sim.run(m, d, ctrl, T, on_step=lambda m, d: (d.time, d.qpos[2], np.abs(demand[0]) / lim,
                                                           np.abs(d.actuator_force[idx.act]) / lim))
        t, z = np.array([x[0] for x in log]), np.array([x[1] for x in log])
        dem, app = np.array([x[2] for x in log]), np.array([x[3] for x in log])
        res[name] = dict(t=t, z=z, dem=dem, app=app, t95=float(t[np.argmax(z >= z[0] + 0.95 * (z[-1] - z[0]))]))
    L = res['limited']
    return {'rise_applied_over_limit_max': float(L['app'].max()), 'rise_demand_over_limit_max': float(L['dem'].max()),
            'rise_unlimited_applied_over_limit_max': float(res['unlimited']['app'].max()),
            'rise_t95_limited_s': L['t95'], 'rise_t95_unlimited_s': res['unlimited']['t95'],
            'rise_knee_saturated_fraction_first_0p2s': float((L['app'][L['t'] <= 0.2][:, 2::3] >= 1 - 1e-9).mean()),
            'rise_final_height_m': float(L['z'][-1])}, res


PLATFORM_MUS = (0.2, 0.4, 0.6)
BLOCK_Y = (-0.2, -0.35, -0.5)                       # side by side across the slope, so no block pushes another
GO2_Y = 0.25
HINGE_Z = 1.0                                         # high enough that the tilted 2.4 m platform clears the floor at 40 deg
BOX_RGBA = ((0.04, 0.45, 0.66, 1), (0.48, 0.27, 0.84, 1), (0.79, 0.52, 0.06, 1))


def platform_model():
    """scene.xml + a 2.4 m x 1.2 m platform hinged about y at height HINGE_Z (top surface through the axis), three 10 cm
    blocks with mu = 0.2, 0.4, 0.6 (platform mu 0.05, so the block's mu wins MuJoCo's max rule) and the Go2 on it."""
    spec = mujoco.MjSpec.from_file(str(sim.require_model(scene=True)))
    plat = spec.worldbody.add_body(name='platform', pos=[0, 0, HINGE_Z])
    plat.add_joint(name='tilt', type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    plat.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[1.2, 0.6, 0.02], pos=[0, 0, -0.02], mass=200,
                  friction=[0.05, 0.005, 0.0001], rgba=[0.75, 0.77, 0.8, 1])
    for i, (mu, y) in enumerate(zip(PLATFORM_MUS, BLOCK_Y)):
        b = spec.worldbody.add_body(name=f'block{i}', pos=[-0.6, y, HINGE_Z + 0.05])
        b.add_freejoint()
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.05, 0.05], mass=1.0, friction=[mu, 0.005, 0.0001],
                   condim=3, rgba=BOX_RGBA[i])
    m = spec.compile()
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    return m, mujoco.MjData(m)


def platform(media=True, hold=1.5, ramp=16.0, theta_max=40.0):
    """Tilt the platform from 0 to theta_max along a minimum-jerk profile (hinge position and velocity prescribed every
    step, so contacts see the true surface velocity) while the Go2 holds its stance under joint PD. Onset of sliding: block
    speed relative to the surface, in the surface plane, above 2 mm/s."""
    m, d = platform_model(); idx = sim.index(m)
    jt = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'tilt'); qa, va = m.jnt_qposadr[jt], m.jnt_dofadr[jt]
    pb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'platform')
    blocks = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'block{i}') for i in range(3)]
    sim.place_on_ground(m, d, xy=(0.0, GO2_Y), clearance=HINGE_Z)
    for b in blocks:                                                 # blocks resting on the platform top
        a = m.jnt_qposadr[m.body_jntadr[b]]; d.qpos[a:a + 3] = m.body_pos[b]; d.qpos[a + 3:a + 7] = (1, 0, 0, 0)
    mujoco.mj_forward(m, d)
    pd = sim.joint_pd(m, sim.Q_STAND)
    th_max = np.radians(theta_max); T = hold + ramp + 1.0

    def theta(t):
        s = np.clip((t - hold) / ramp, 0, 1)
        return th_max * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5), th_max / ramp * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4)
    rec = sim.Recorder(m, MEDIA / 'incline.mp4', width=960, height=540, distance=3.2, azimuth=90.0, elevation=-14.0,
                       track='platform') if media else None
    onset, log, still_at, go2_ref = [None] * 3, [], None, None
    hinge = np.array([0, 0, HINGE_Z])
    lo, hi = m.actuator_ctrlrange[idx.act].T
    for k in range(round(T / m.opt.timestep)):
        th, dth = theta(d.time)
        d.qpos[qa], d.qvel[va] = th, dth
        d.ctrl[idx.act] = np.clip(pd(m, d), lo, hi)
        mujoco.mj_step(m, d)
        th, dth = theta(d.time); d.qpos[qa], d.qvel[va] = th, dth
        mujoco.mj_forward(m, d)
        Rp = d.xmat[pb].reshape(3, 3); nrm = Rp[:, 2]; om = np.array([0, dth, 0])
        rel = []
        for i, b in enumerate(blocks):
            a = m.jnt_dofadr[m.body_jntadr[b]]
            vr = d.qvel[a:a + 3] - np.cross(om, d.xpos[b] - hinge)
            rel.append(np.linalg.norm(vr - (vr @ nrm) * nrm))
            if onset[i] is None and rel[-1] > 2e-3 and d.time > hold:
                onset[i] = float(np.degrees(th))
        base_in_plat = Rp.T @ (d.qpos[:3] - hinge)
        if go2_ref is None and d.time >= hold:
            go2_ref = base_in_plat.copy()
            # rigid-stance tipping angle: COM over the front-foot line, (x_front - x_com) / (z_com - z_feet) in platform axes
            com = Rp.T @ (d.subtree_com[idx.base] - hinge); feet = (sim.foot_positions(m, d) - hinge) @ Rp
            tip = (feet[:2, 0].mean() - com[0], com[2] - feet[:, 2].mean() + sim.FOOT_RADIUS)
        F = sim.foot_forces(m, d) @ Rp                               # rows: foot force in platform axes
        ratio = np.hypot(F[:, 0], F[:, 1]) / np.maximum(F[:, 2], 1e-9)
        bx = [(Rp.T @ (d.xpos[b] - hinge))[0] for b in blocks]         # block position along the platform
        log.append((d.time, np.degrees(th), *rel, *(base_in_plat - (go2_ref if go2_ref is not None else base_in_plat)),
                    *ratio, *F[:, 2], *bx))
        if rec:
            rec(m, d)
            if still_at is None and onset[1] is not None and np.degrees(th) > onset[1] + 2.0:
                rec.still(d, MEDIA / 'incline.png'); still_at = float(np.degrees(th))
    if rec:
        rec.close(); rec.__exit__()
    L = np.array(log)
    disp = np.linalg.norm(L[:, 5:8], axis=1)
    after = L[:, 0] > hold
    rear_slip = L[after & (L[:, 10:12].min(1) >= 0.8 * (1 - 1e-3)), 1]
    rear_off = L[after & (L[:, 14:16].max(1) < 1.0), 1]
    pred = [float(np.degrees(np.arctan(mu))) for mu in PLATFORM_MUS]
    errs = [abs(o - p) if o is not None else 90.0 for o, p in zip(onset, pred)]
    return {'platform_onset_deg': onset, 'platform_onset_pred_deg': pred, 'platform_onset_err_deg_max': float(max(errs)),
            'platform_go2_rear_feet_at_friction_limit_deg': float(rear_slip[0]) if len(rear_slip) else None,
            'platform_go2_rear_feet_off_deg': float(rear_off[0]) if len(rear_off) else None,
            'platform_go2_tip_pred_deg': float(np.degrees(np.arctan2(*tip))),
            'platform_go2_displacement_at_20deg_m': float(disp[np.argmax(L[:, 1] >= 20.0)]),
            'platform_go2_displacement_final_m': float(disp[-1]),
            'platform_ramp_s': ramp, 'platform_duration_s': float(T), 'platform_still_at_deg': still_at}, L


# ==== figures ============================================================================================================

def fig_diagrams():
    """Method diagram: the incline (forces) and the soft contact (penetration where K d r balances the load)."""
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.9))
    ax = axs[0]; ax.set_aspect('equal'); ax.axis('off'); ax.grid(False)
    th = np.radians(25)
    t, n = np.array([np.cos(th), -np.sin(th)]), np.array([np.sin(th), np.cos(th)])
    p0 = np.array([0, 2.2]); p1 = p0 + 4.2 * t
    ax.fill([p0[0], p1[0], p0[0]], [p0[1], p1[1], p1[1]], color=fs.C['fill'], ec=fs.C['muted'], lw=0.8)
    c = p0 + 1.6 * t + 0.35 * n
    sq = np.array([c + 0.35 * (sx * t + sy * n) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    ax.fill(sq[:, 0], sq[:, 1], color=fs.C['est'], alpha=0.25, ec=fs.C['est'])
    fs.arrow(ax, c, c + np.array([0, -1.4]))
    fs.label(ax, c + np.array([0.08, -1.3]), 'm g')
    base = c - 0.35 * n
    fs.arrow(ax, base, base + 1.4 * n); fs.label(ax, base + 1.45 * n + np.array([0.05, 0.05]), 'N = m g cos θ')
    fs.arrow(ax, base, base - 1.1 * t); fs.label(ax, base - 1.1 * t + np.array([-0.15, 0.25]), 'F ≤ μ N', ha='center')
    ax.text(p1[0] - 0.95, p1[1] + 0.1, 'θ', color=fs.C['ink'])
    ax.text(0.0, -0.05, 'holds while tan θ ≤ μ;  slides with a = g (sin θ − μ cos θ)', color=fs.C['ink'], fontsize=9)
    ax.set_xlim(-0.3, 4.6); ax.set_ylim(-0.3, 3.6)
    ax.set_title('Block on an incline (Coulomb)')
    ax = axs[1]
    u = np.linspace(0, 0.03, 400)
    imp = (0.015, 1.0, 0.022, 0.5, 2)
    K, _ = P.stiffness_damping((0.02, 1.0), imp)
    dd = np.array([P.impedance(x, imp) for x in u])
    ax.plot(u * 1e3, K * dd * dd * u, color=fs.C['est'], label='spring side  K d(u)² u')
    for f, ls in ((33.4, '-'), (41.2, '--')):
        ax.plot(u * 1e3, (1 - dd) * 1.767 * f, color=fs.C['plan'], ls=ls, label=f'load side  (1 − d) Â f,  f = {f} N')
    ax.axvline(13.56, color=fs.C['gt'], lw=0.8); ax.text(21.5, 22, 'Go2 front foot at rest:\nu = 13.6 mm\n(12.6 mm depth + 1 mm margin)',
                                                          fontsize=8, color=fs.C['muted'])
    ax.set_xlabel('constraint violation u (mm)'); ax.set_ylabel('acceleration (m/s²)'); ax.set_ylim(0, 80)
    ax.legend(loc='upper left'); ax.set_title('Soft contact at rest: where the two sides meet')
    fs.save(fig, FIG / 'diagrams')


def fig_free_fall(ff_traj, go2_traj, h=0.002):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.4))
    n = len(ff_traj['Euler']); t = np.arange(n) * h
    zp = P.free_fall(t, 10.0)[0]
    ax = axs[0]
    ax.plot(t, (ff_traj['Euler'] - zp) * 1e3, color=fs.C['est'], label='MuJoCo Euler − parabola')
    ax.plot(t, P.free_fall_semi_implicit_offset(t, h) * 1e3, color=fs.C['plan'], ls='--', label='−g h t / 2')
    ax.plot(t, (ff_traj['RK4'] - zp) * 1e3, color=fs.C['est2'], label='MuJoCo RK4 − parabola')
    ax.set_xlabel('time (s)'); ax.set_ylabel('height error (mm)'); ax.legend(); ax.set_title('Sphere, h = 2 ms')
    ax = axs[1]
    for hh, col in zip(sorted(go2_traj, reverse=True), (fs.C['est'], fs.C['est2'], fs.C['plan'])):
        r = go2_traj[hh]
        ax.plot(r['t'], r['residual'] * 1e3, color=col, label=f'h = {hh * 1e3:g} ms')
    ax.set_xlabel('time (s)'); ax.set_ylabel('|COM − (parabola − g h t/2)| (mm)')
    ax.set_title('Go2 COM, joints moving, residual after the Euler offset'); ax.legend()
    fs.save(fig, FIG / 'free_fall')


def fig_oscillator(tr):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.4))
    t, E, z = tr['t'], tr['E'], tr['z']
    ax = axs[0]
    k = np.arange(len(t))
    for s, col, lab in (('explicit_euler', fs.C['bad'], 'explicit Euler'), ('semi_implicit_euler', fs.C['est'], 'semi-implicit Euler'),
                        ('rk4', fs.C['est2'], 'RK4')):
        e = E(tr['own'][s]); ax.plot(t, e / e[0], color=col, label=lab)
    ax.plot(t, P.oscillator_energy_factor(z, 'explicit_euler') ** k, color=fs.C['plan'], ls='--', lw=1, label='(1 + z²)^k')
    ax.set_yscale('log'); ax.set_xlabel('time (s)'); ax.set_ylabel('E / E₀'); ax.legend(); ax.set_title(f'Oscillator, z = ω h = {z:g}')
    ax = axs[1]
    e = E(tr['own']['semi_implicit_euler']); band = P.symplectic_energy_band(z)
    ax.plot(t[:300], e[:300] / e[0] - 1, color=fs.C['est'], label='semi-implicit Euler')
    ax.axhline(1 / (1 - z / 2) - 1, color=fs.C['plan'], ls='--', lw=1, label='band 1/(1 ∓ z/2) − 1')
    ax.axhline(1 / (1 + z / 2) - 1, color=fs.C['plan'], ls='--', lw=1)
    er = E(tr['own']['rk4']); ax.plot(t[:300], er[:300] / er[0] - 1, color=fs.C['est2'], label='RK4')
    ax.set_xlabel('time (s)'); ax.set_ylabel('E / E₀ − 1'); ax.legend(loc='lower left'); ax.set_title(f'Bounded energy error (max {band:.4f})')
    fs.save(fig, FIG / 'oscillator')


def fig_energy(en, diag):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.5))
    ax = axs[0]; hs = en['hs'] * 1e3
    cols = dict(Euler=fs.C['est'], implicitfast=fs.C['good'], implicit=fs.C['plan'], RK4=fs.C['est2'])
    for k, v in en['drift'].items():
        ax.loglog(hs, v, marker='o', color=cols[k], label=f'{k} (slope {P.fit_order(en["hs"], v):.2f})', ms=4,
                  ls='--' if k == 'implicitfast' else '-')
    ax.loglog(diag['hs_fixed'] * 1e3, diag['rk4_fixed'], 's:', color=fs.C['est2'], ms=4,
              label=f'RK4, base welded (slope {diag["order_rk4_fixed"]:.2f})')
    ax.set_xlabel('timestep h (ms)'); ax.set_ylabel('max |E − E₀| / E₀ over 2 s'); ax.legend(fontsize=7.5)
    ax.set_title('Tumbling Go2, energy drift')
    ax = axs[1]
    for k, (t, e) in en['traces'].items():
        if k != 'implicitfast':
            ax.plot(t, np.abs(e) + 1e-16, color=cols[k], label=k)
    ax.set_yscale('log'); ax.set_xlabel('time (s)'); ax.set_ylabel('|E − E₀| / E₀'); ax.legend(); ax.set_title('h = 2 ms')
    fs.save(fig, FIG / 'energy_drift')


def fig_incline(inc, curves):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.6))
    ax = axs[0]
    mus = np.linspace(0.05, 1.1, 100)
    ax.plot(mus, np.degrees(np.arctan(mus)), color=fs.C['plan'], label='atan μ')
    ax.plot(mus, np.degrees(np.arctan(mus / np.sqrt(2))), color=fs.C['plan'], ls='--', label='atan(μ/√2)')
    th = inc['threshold_deg']
    for key, mk, col, lab in (('elliptic_ir1', 'o', fs.C['est'], 'elliptic'), ('pyramidal_ir1', 's', fs.C['est2'], 'pyramidal, along t₁'),
                              ('pyramidal_ir1_45', 'x', fs.C['bad'], 'pyramidal, 45° to t₁')):
        tags = [k for k in th if k.startswith(key.replace('_45', '')) and (k.endswith('45deg') == key.endswith('_45'))]
        ax.plot([float(k.split('mu')[1].split('_')[0]) for k in tags], [th[k]['measured'] for k in tags], mk, color=col, label=lab)
    ax.set_xlabel('friction coefficient μ'); ax.set_ylabel('threshold angle (deg)'); ax.legend(); ax.set_title('Steepest angle that holds')
    ax = axs[1]
    for (key, mu), (deg, meas, pred) in curves.items():
        if key != 'elliptic_ir100':
            continue
        dd = np.linspace(deg.min(), deg.max(), 100)
        ax.plot(dd, P.incline_acceleration(np.radians(dd), mu), color=fs.C['plan'], lw=1)
        ax.plot(deg, meas, 'o', color=fs.C['est'], ms=4)
        ax.text(deg.max(), meas[-1], f' μ={mu:g}', fontsize=8, va='center', color=fs.C['muted'])
    ax.plot([], [], color=fs.C['plan'], label='g (sin θ − μ cos θ)'); ax.plot([], [], 'o', color=fs.C['est'], label='MuJoCo, elliptic')
    ax.set_xlabel('incline θ (deg)'); ax.set_ylabel('acceleration along slope (m/s²)'); ax.legend(); ax.set_title('Sliding acceleration')
    fs.save(fig, FIG / 'incline')


def fig_creep(inc):
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    cr = inc['creep']; mus = MUS
    for key, col, lab in (('pyramidal_ir1', fs.C['est2'], 'pyramidal, impratio 1'), ('elliptic_ir1', fs.C['est'], 'elliptic, impratio 1'),
                          ('elliptic_ir100', fs.C['good'], 'elliptic, impratio 100 (Go2 setting)')):
        ax.semilogy(mus, [cr[f'{key}_mu{mu:g}']['speed_m_s'] * 1e3 for mu in mus], 'o', color=col, label=lab)
        if 'elliptic' in key:
            ax.semilogy(mus, [cr[f'{key}_mu{mu:g}']['predicted_m_s'] * 1e3 for mu in mus], '-', color=fs.C['plan'], lw=1)
    ax.plot([], [], '-', color=fs.C['plan'], label='R_t f_t / B (written-out model)')
    ax.set_xlabel('μ (block held at 0.9 atan μ)'); ax.set_ylabel('creep speed (mm/s)'); ax.legend(); ax.set_title('Soft friction creeps below the threshold')
    fs.save(fig, FIG / 'creep')


def fig_penetration(rows, refsafe, feet):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.6))
    ax = axs[0]
    imps = sorted({tuple(r['solimp']) for r in rows}, key=lambda s: s[0])
    cols = [fs.C['est'], fs.C['est2'], fs.C['good'], fs.C['bad']]
    for imp, col in zip(imps, cols):
        rr = [r for r in rows if tuple(r['solimp']) == imp]
        tc = [r['timeconst'] for r in rr]
        ax.loglog(tc, [r['predicted_m'] * 1e3 for r in rr], '-', color=fs.C['plan'], lw=1)
        ax.loglog(tc, [r['measured_m'] * 1e3 for r in rr], 'o', color=col, label=f'solimp {imp[0]:g} {imp[1]:g} {imp[2]:g}')
    ax.loglog([refsafe['asked_timeconst']], [refsafe['measured_m'] * 1e3], 'x', color=fs.C['bad'], ms=7, label='timeconst 2 ms (= h): refsafe')
    ax.loglog([refsafe['asked_timeconst']], [refsafe['predicted_nominal_m'] * 1e3], '+', color=fs.C['gt'], ms=7, label='same, nominal prediction')
    ax.plot([], [], color=fs.C['plan'], label='steady_penetration()')
    ax.set_xlabel('solref timeconst (s)'); ax.set_ylabel('penetration (mm)'); ax.legend(fontsize=7); ax.set_title('1 kg sphere at rest')
    ax = axs[1]
    names = list(feet['variants']); x = np.arange(4)
    for i, (nm, col) in enumerate(zip(names, (fs.C['est'], fs.C['est2'], fs.C['good']))):
        v = feet['variants'][nm]
        ax.bar(x + (i - 1) * 0.26, v['measured_mm'], 0.24, color=col, label=nm)
        ax.plot(x + (i - 1) * 0.26, v['predicted_mm'], '_', color=fs.C['plan'], ms=14, mew=2)
    ax.plot([], [], '_', color=fs.C['plan'], ms=10, mew=2, label='predicted')
    ax.set_xticks(x, LEGS); ax.set_ylabel('foot penetration (mm)'); ax.set_ylim(0, 18)
    ax.legend(fontsize=7.5, ncol=2, loc='upper center'); ax.set_title('Standing Go2, per foot')
    fs.save(fig, FIG / 'penetration')


def fig_saturation(b, rise):
    fig, axs = plt.subplots(1, 3, figsize=(11.5, 3.4))
    ax = axs[0]; t = b['t']
    ax.plot(t, b['q'], color=fs.C['est'], label='MuJoCo, limited')
    ax.plot(t, b['qu'], color=fs.C['est2'], label='MuJoCo, limit removed')
    ks = b['k_exit'] + 1
    ax.plot(t[:ks], b['q_cf'][:ks], '--', color=fs.C['plan'], label='closed form, τ = τ_max')
    tt = np.linspace(0, b['ref'].t[-1], 300); ax.plot(tt, b['ref'].sol(tt)[0], ':', color=fs.C['gt'], label='scipy reference')
    ax.axhline(b['q1'], color=fs.C['gt'], lw=0.6); ax.axvline(b['t_exit'], color=fs.C['gt'], lw=0.6, ls='--')
    ax.set_xlabel('time (s)'); ax.set_ylabel('knee angle (rad)'); ax.legend(fontsize=7.5); ax.set_title('Knee bench, step −2.6 → −1.0 rad')
    ax = axs[1]
    ax.plot(t[:-1], b['tau_d'], color=fs.C['plan'], label='PD demand')
    ax.plot(t[:-1], b['tau_a'], color=fs.C['est'], label='applied (actuator_force)')
    ax.axhline(b['tau_max'], color=fs.C['bad'], lw=0.8, ls='--', label='ctrlrange ±45.43 N m'); ax.axhline(-b['tau_max'], color=fs.C['bad'], lw=0.8, ls='--')
    ax.set_xlim(0, 0.15); ax.set_xlabel('time (s)'); ax.set_ylabel('knee torque (N m)'); ax.legend(fontsize=7.5); ax.set_title('Demand against applied')
    ax = axs[2]
    for nm, col in (('limited', fs.C['est']), ('unlimited', fs.C['est2'])):
        r = rise[nm]; ax.plot(r['t'], r['z'], color=col, label=f'{nm} (t95 {r["t95"] * 1e3:.0f} ms)')
    ax.set_xlim(0, 0.4); ax.set_xlabel('time (s)'); ax.set_ylabel('base height (m)'); ax.legend(); ax.set_title('Go2 crouch to stand, PD step')
    fs.save(fig, FIG / 'saturation')


def fig_platform(L, pl):
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.4))
    ax = axs[0]
    for i, (mu, c) in enumerate(zip(PLATFORM_MUS, BOX_RGBA)):
        on = L[:, 16 + i] < 1.15                                     # until the block reaches the platform's lower edge
        ax.semilogy(L[on, 1], L[on, 2 + i] + 1e-7, color=c[:3], label=f'block μ = {mu:g}')
        ax.axvline(np.degrees(np.arctan(mu)), color=c[:3], ls='--', lw=0.8)
    ax.axhline(2e-3, color=fs.C['gt'], lw=0.6)
    ax.set_ylim(1e-6, 10); ax.set_xlabel('platform angle (deg)'); ax.set_ylabel('speed relative to surface (m/s)'); ax.legend(fontsize=7.5)
    ax.set_title('Tilting platform: dashed lines at atan μ')
    ax = axs[1]
    ax.plot(L[:, 1], np.linalg.norm(L[:, 5:8], axis=1) * 1e3, color=fs.C['est'])
    ax.set_xlabel('platform angle (deg)'); ax.set_ylabel('Go2 base displacement on platform (mm)'); ax.set_yscale('log')
    ax.axvline(np.degrees(np.arctan(0.8)), color=fs.C['plan'], ls='--', lw=0.8, label='atan 0.8 (foot μ)')
    ax.axvline(pl['platform_go2_rear_feet_at_friction_limit_deg'], color=fs.C['bad'], ls=':', lw=0.9, label='rear feet at |f_t| = μ f_n')
    ax.axvline(pl['platform_go2_rear_feet_off_deg'], color=fs.C['bad'], ls='--', lw=0.9, label='rear feet unloaded (< 1 N)')
    ax.legend(fontsize=7.5, loc='lower right')
    ax.set_title('Go2 under joint PD on the same platform')
    fs.save(fig, FIG / 'platform')


# ==== main ===============================================================================================================

def rk4_diagnostics():
    """Why the floating Go2's RK4 drift falls only as h^2: the same test with the base welded to the world, and the
    orientation error of a single tumbling box against a 10 us reference."""
    spec = mujoco.MjSpec.from_file(str(sim.require_model(scene=False)))
    for j in list(spec.body('base').joints):
        spec.delete(j)
    for k in list(spec.keys):
        spec.delete(k)
    mf = spec.compile()
    rng = np.random.default_rng(0); rng.uniform(-1, 1, 3); qf = rng.uniform(-2, 2, 12)   # same joint velocities as go2_energy
    hs = np.array([0.0005, 0.001, 0.002, 0.004]); e = []
    for h in hs:
        mf.opt.timestep, mf.opt.integrator = h, MJ_INT['RK4']
        mf.dof_damping[:] = 0; mf.dof_frictionloss[:] = 0; mf.opt.gravity[:] = 0
        mf.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT; mf.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
        d = mujoco.MjData(mf); d.qpos[:] = sim.Q_STAND; d.qvel[:] = qf; mujoco.mj_forward(mf, d); E0 = d.energy.sum(); mx = 0
        for k in range(round(2.0 / h)):
            mujoco.mj_step(mf, d)
            if (k + 1) % max(1, round(0.004 / h)) == 0:
                mujoco.mj_forward(mf, d); mx = max(mx, abs(d.energy.sum() / E0 - 1))
        e.append(mx)
    box = mujoco.MjModel.from_xml_string('<mujoco><option gravity="0 0 0" integrator="RK4"/><worldbody><body><freejoint/>'
                                         '<geom type="box" size="0.3 0.2 0.1" mass="1"/></body></worldbody></mujoco>')

    def tumble(h):
        box.opt.timestep = h; d = mujoco.MjData(box); d.qvel[3:] = (3, 0.2, 0.5)
        for _ in range(round(1.0 / h)):
            mujoco.mj_step(box, d)
        return d.qpos[3:7].copy()
    qr = tumble(1e-5); hb = np.array([0.001, 0.002, 0.004])
    eb = [2 * np.arccos(min(1.0, abs(tumble(h) @ qr))) for h in hb]
    return {'energy_order_rk4_fixed_base': P.fit_order(hs, e), 'rk4_fixed_base_drift': dict(zip([f'{h * 1e3:g}ms' for h in hs], e)),
            'rk4_box_orientation_order': P.fit_order(hb, eb), 'rk4_box_orientation_err_rad': dict(zip([f'{h * 1e3:g}ms' for h in hb], eb))}, \
        dict(hs_fixed=hs, rk4_fixed=np.array(e), order_rk4_fixed=P.fit_order(hs, e))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--no-media', action='store_true')
    ap.add_argument('--json', default=str(HERE / 'results.json')); a = ap.parse_args()
    t0 = time.time()
    res = {'fundamental': {}, 'intermediate': {}, 'advanced': {}}
    F, I, A = res['fundamental'], res['intermediate'], res['advanced']
    o, ff = free_fall_sphere(); F.update(o)
    o, go2ff = free_fall_go2(); F.update(o)
    o, osc = oscillator(); F.update(o)
    o, en = go2_energy(); I.update(o)
    o, diag = rk4_diagnostics(); I['rk4_diagnostics'] = o
    o, curves = incline(); I.update(o)
    I['accel'] = {f'{k}_mu{mu:g}': dict(theta_deg=c[0].tolist(), measured=c[1].tolist(), predicted=c[2].tolist())
                  for (k, mu), c in curves.items()}
    o, me_s, rows = penetration_sphere(); A.update(o)
    o, me_g = go2_feet(); A.update(o)
    A['contact_model_vs_efc_rel_max'] = float(max(me_s, me_g))
    o, bench = knee_bench(); A.update(o)
    o, rise = go2_rise(); A.update(o)
    o, L = platform(media=not a.no_media); A.update(o)
    FIG.mkdir(exist_ok=True)
    fig_diagrams(); fig_free_fall(ff, go2ff); fig_oscillator(osc); fig_energy(en, diag); fig_incline(I, curves); fig_creep(I)
    fig_penetration(rows, A['refsafe'], A); fig_saturation(bench, rise); fig_platform(L, A)
    mp4 = sorted(MEDIA.glob('*.mp4')) if MEDIA.exists() else []
    res['media_bytes'] = {p.name: p.stat().st_size for p in mp4}
    if mp4:
        res['media_mp4_under_6MB'] = float(all(p.stat().st_size < 6e6 for p in mp4))
    flat = {k: v for part in (F, I, A) for k, v in part.items()}
    if 'media_mp4_under_6MB' in res:
        flat['media_mp4_under_6MB'] = res['media_mp4_under_6MB']
    res['checks'] = {k: dict(measured=flat[k], op=op, threshold=thr, passed=check(k, flat[k])) for k, (op, thr) in ACCEPT.items()
                     if k in flat}
    res['skipped_checks'] = [k for k in ACCEPT if k not in flat]
    res['wall_time_s'] = time.time() - t0
    res['load_average_1min_at_end'] = os.getloadavg()[0]
    for k, c in res['checks'].items():
        print(f"{'PASS' if c['passed'] else 'MISS'}  {k:48s} {c['measured']!s:>24.24s}  {c['op']} {c['threshold']}")
    print(f"{sum(c['passed'] for c in res['checks'].values())} of {len(res['checks'])} checks pass; skipped {res['skipped_checks']}; "
          f"{res['wall_time_s']:.0f} s")
    pathlib.Path(a.json).write_text(json.dumps(res, indent=2, default=float) + '\n')


if __name__ == '__main__':
    main()
