#!/usr/bin/env python3
"""Experiment for module 02 (robot description), all three levels.
  fundamental:  the Gazebo URDF (xacro output) and the Unitree URDF parsed and lumped by description.py, checked against
                Pinocchio's URDF import and MuJoCo's URDF importer; module 01's MJCF parse checked against MuJoCo. Body frames,
                masses, centres of mass, inertias, joint origins, limits and collision geometry of URDF vs MJCF; side-by-side
                renders and a video; difference bar charts; calf diagram.
  intermediate: validity of every inertia in the three files (positive definite, triangle inequality, pseudo-inertia, size);
                left-right mirror symmetry; Gazebo reference names; triangle inequality vs pseudo-inertia on random tensors;
                gravity regressor vs Pinocchio and MuJoCo; gravity torque the URDF parameters predict vs the MJCF ones.
  advanced:     identification of each calf's mass and centre of mass from joint torques simulated in MuJoCo at static poses on
                a fixed base with tilted gravity: rank of the full-leg problem, friction bias, two-sided approach, held-out poses.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--no-media]"""
import argparse, json, os, pathlib, re, sys, time, warnings
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
import description as D  # noqa: E402
from common import G, GO2_XML, LEGS, SCENE_XML, axis_angle_R  # noqa: E402

A = D.A
FIG, MEDIA = HERE / 'figures', HERE / 'media'
SEGS = ('hip', 'thigh', 'calf')

# Acceptance thresholds, fixed before the first full run (MODULE_SPEC.md). (op, threshold)
ACCEPT = {
    # fundamental: the parsers and the lumping
    'stored_urdf_equals_xacro': ('>=', 1.0), 'lump_vs_pinocchio_gazebo': ('<', 1e-12), 'lump_vs_pinocchio_unitree': ('<', 1e-12),
    'lump_vs_mujoco_urdf_import': ('<', 1e-10), 'mjcf_parse_vs_mujoco': ('<', 1e-12), 'body_frames_urdf_vs_mjcf_m': ('<', 1e-9),
    # intermediate: validity tests and the regressor
    'synthetic_known_cases': ('>=', 1.0), 'synthetic_pseudo_vs_triangle_agreement': ('>=', 1.0),
    'invalid_inertias_all_files': ('<=', 0), 'regressor_vs_pinocchio_Nm': ('<', 1e-10), 'regressor_vs_mujoco_bias_Nm': ('<', 1e-10),
    # advanced: identification. Rank 6 is the count derived in the README; tolerances are set by what each case can reach:
    # no friction: only integration error; two-sided: friction bias cancels to first order
    'full_leg_rank': ('range', (6, 6)), 'id_nofric_mass_rel_err_max': ('<', 1e-3), 'id_nofric_com_err_max_m': ('<', 5e-4),
    'id_twosided_mass_rel_err_max': ('<', 0.05), 'id_twosided_com_err_max_m': ('<', 5e-3),
    'id_twosided_mass_err_over_urdf_gap': ('<', 0.25), 'holdout_rms_ratio_id_over_urdf': ('<', 0.5),
    'media_mp4_under_6MB': ('>=', 1.0),
}
GL_CHECKS = ('media_mp4_under_6MB',)
ROS_CHECKS = ('stored_urdf_equals_xacro',)


def check(key, val):
    op, thr = ACCEPT[key]
    return bool(thr[0] <= val <= thr[1]) if op == 'range' else bool({'<': val < thr, '<=': val <= thr, '>': val > thr,
                                                                       '>=': val >= thr}[op])


def can_render(m):
    try:
        with mujoco.Renderer(m, 8, 8):
            return True
    except Exception as e:
        warnings.warn(f'offscreen rendering unavailable ({type(e).__name__}: {e}): skipping renders and videos')
        return False


def mjcf_body_inertia(m, i):
    R = A.quat_to_R(m.body_iquat[i])
    return R @ np.diag(m.body_inertia[i]) @ R.T


def strip_urdf(text):
    """URDF without visuals (Collada meshes MuJoCo cannot load) and Gazebo / ros2_control blocks."""
    for tag in ('visual', 'gazebo', 'ros2_control'):
        text = re.sub(rf'<{tag}[ >].*?</{tag}>', '', text, flags=re.S)
    return text


def normalise_urdf(text):
    """URDF text without comments (the header names the xacro path) and with every absolute path up to an install space
    (mesh file:// URIs, the ros_control YAML) replaced by INSTALL/, so the comparison does not depend on the checkout path."""
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    return re.sub(r'(?:file://)?/[^\s"<>]*?/install/', 'INSTALL/', text)


def urdf_spec(path=D.GAZEBO_URDF):
    """MuJoCo's URDF importer on the stripped file, with a free joint on the root so it can stand next to the MJCF."""
    sp = mujoco.MjSpec.from_string(strip_urdf(pathlib.Path(path).read_text()))
    sp.body('base_link').add_freejoint()
    return sp


def by_name(bodies):
    return {b.name: b for b in bodies}


def leg_seg(name):
    leg, seg = name.split('_')
    return leg, seg


# ---- fundamental ---------------------------------------------------------------------------------------------------------

def fundamental(r, m):
    res = {}
    txt = D.xacro()
    if txt is None:
        warnings.warn('xacro / ROS 2 Humble not found: stored_urdf_equals_xacro skipped')
    else:
        res['stored_urdf_equals_xacro'] = float(normalise_urdf(txt) == normalise_urdf(D.GAZEBO_URDF.read_text()))
    files = {'gazebo': D.GAZEBO_URDF, 'unitree': D.UNITREE_URDF}
    parsed = {k: D.parse_urdf(p) for k, p in files.items()}
    lumped = {k: D.lump(L, J) for k, (L, J, _) in parsed.items()}
    # lumping vs Pinocchio (it merges fixed joints into the parent joint's inertia as well)
    for k, p in files.items():
        pm = pin.buildModelFromUrdf(str(p), pin.JointModelFreeFlyer())
        err = 0.0
        for b in lumped[k]:
            I = pm.inertias[pm.getJointId('root_joint' if b.joint is None else b.joint)]
            err = max(err, abs(I.mass - b.mass), np.abs(I.lever - b.com).max(), np.abs(I.inertia - b.I).max())
        res[f'lump_vs_pinocchio_{k}'] = float(err)
    # lumping vs MuJoCo's URDF importer (fusestatic), Gazebo file
    mu = urdf_spec().compile()
    err = 0.0
    for b in lumped['gazebo']:
        i = mujoco.mj_name2id(mu, mujoco.mjtObj.mjOBJ_BODY, b.link)
        err = max(err, abs(mu.body_mass[i] - b.mass), np.abs(mu.body_ipos[i] - b.com).max(),
                  np.abs(mjcf_body_inertia(mu, i) - b.I).max())
    res['lump_vs_mujoco_urdf_import'] = float(err)
    # module 01's MJCF parse vs the compiled model
    err = 0.0
    for i, b in enumerate(r.bodies):
        err = max(err, abs(m.body_mass[i + 1] - b.mass), np.abs(m.body_ipos[i + 1] - b.com).max(),
                  np.abs(mjcf_body_inertia(m, i + 1) - b.inertia).max())
    res['mjcf_parse_vs_mujoco'] = float(err)
    # body frames: URDF lumped body frame offsets vs MJCF body offsets (base: both at the root origin)
    mj = {b.name: b for b in r.bodies}
    err = 0.0
    for k in lumped:
        for b in lumped[k][1:]:
            mb = mj[b.name]
            err = max(err, np.abs(b.pos - mb.pos).max(), np.abs(b.R - mb.R).max())
    res['body_frames_urdf_vs_mjcf_m'] = float(err)
    # comparison tables
    res['bodies'] = body_table(r, lumped)
    res['joints'] = joint_table(r, parsed)
    res['collision'] = collision_table(m, lumped)
    res['totals'] = totals(r, lumped)
    return res, parsed, lumped


def body_table(r, lumped):
    rows = []
    for b in r.bodies:
        row = dict(body=b.name, mjcf_mass_kg=b.mass, mjcf_com_m=b.com.tolist(), mjcf_principal=np.linalg.eigvalsh(b.inertia).tolist())
        for k, L in lumped.items():
            u = by_name(L)[b.name]
            row[k] = dict(mass_kg=u.mass, dmass_kg=u.mass - b.mass, dmass_rel=(u.mass - b.mass) / b.mass, com_m=u.com.tolist(),
                          dcom_m=float(np.linalg.norm(u.com - b.com)),
                          dI_rel=float(np.linalg.norm(u.I - b.inertia) / np.linalg.norm(b.inertia)),
                          principal=np.linalg.eigvalsh(u.I).tolist(), members=u.members)
        rows.append(row)
    return rows


def joint_table(r, parsed):
    rows = []
    act = {r.joints[a.joint].name: a for a in r.actuators}
    for j in r.joints[1:]:
        leg, seg, _ = j.name.split('_')
        row = dict(joint=j.name, mjcf=dict(lower=j.range[0], upper=j.range[1], effort=act[j.name].ctrlrange[1], velocity=None,
                                           damping=j.damping, friction=j.frictionloss, armature=j.armature, axis=j.axis.tolist()))
        for k, (L, J, _) in parsed.items():
            uj = next(x for x in J.values() if x.type != 'fixed' and D.canonical(x.child) == f'{leg}_{seg}')
            row[k] = dict(name=uj.name, lower=uj.lower, upper=uj.upper, effort=uj.effort, velocity=uj.velocity,
                          damping=uj.damping, friction=uj.friction, armature=0.0, axis=uj.axis.tolist())
        rows.append(row)
    return rows


def collision_table(m, lumped):
    mc = D.mjcf_collisions(m)
    desc = lambda gs: [dict(type=g['type'], size=np.round(g['size'], 5).tolist(), pos=np.round(g['pos'], 5).tolist()) for g in gs]  # noqa: E731
    rows = []
    for name in ('base', 'FL_hip', 'FL_thigh', 'FL_calf'):
        row = dict(body=name, mjcf=desc(mc[name]), mjcf_volume_m3=float(sum(map(D.volume, mc[name]))),
                   mjcf_aabb=[v.tolist() for v in D.aabb(mc[name])])
        for k, L in lumped.items():
            gs = by_name(L)[name].collisions
            row[k] = desc(gs); row[f'{k}_volume_m3'] = float(sum(map(D.volume, gs)))
            row[f'{k}_aabb'] = [v.tolist() for v in D.aabb(gs)]
        rows.append(row)
    foot = lambda gs: next(g for g in gs if g['type'] == 'sphere')  # noqa: E731
    f_m = foot(mc['FL_calf']); out = dict(rows=rows, foot={'mjcf': dict(radius=float(f_m['size'][0]), centre=f_m['pos'].tolist())})
    for k, L in lumped.items():
        f = foot(by_name(L)['FL_calf'].collisions)
        out['foot'][k] = dict(radius=float(f['size'][0]), centre=f['pos'].tolist())
    # sole depth below the knee along the calf axis, the length that sets standing height: |z_centre| + radius
    out['foot_sole_below_knee_m'] = {k: abs(v['centre'][2]) + v['radius'] for k, v in out['foot'].items()}
    return out


def com_at(r, masses, coms, q):
    """Whole-robot centre of mass in the base frame at joint angles q (base at the identity pose), from module 01's tree FK."""
    R, p, _, _ = A.fk(r, np.r_[0, 0, 0, 1, 0, 0, 0, q])
    c = np.array([p[i] + R[i] @ coms[i] for i in range(len(masses))])
    return np.asarray(masses) @ c / np.sum(masses)


def mass_params(r, bodies=None):
    """(masses, coms) in the MJCF body order, from MJCF (bodies None) or a lumped URDF body list."""
    if bodies is None:
        return [b.mass for b in r.bodies], [b.com for b in r.bodies]
    u = by_name(bodies)
    return [u[b.name].mass for b in r.bodies], [u[b.name].com for b in r.bodies]


def totals(r, lumped):
    out = {'mjcf': dict(mass_kg=float(sum(b.mass for b in r.bodies)),
                        com_stand_m=com_at(r, *mass_params(r), sim.Q_STAND).tolist())}
    for k, L in lumped.items():
        mm, cc = mass_params(r, L)
        out[k] = dict(mass_kg=float(sum(mm)), com_stand_m=com_at(r, mm, cc, sim.Q_STAND).tolist())
        out[k]['dcom_stand_m'] = float(np.linalg.norm(np.subtract(out[k]['com_stand_m'], out['mjcf']['com_stand_m'])))
        out[k]['dmass_kg'] = out[k]['mass_kg'] - out['mjcf']['mass_kg']
    return out


# ---- intermediate --------------------------------------------------------------------------------------------------------

def intermediate(r, m, pm, parsed, lumped, rng):
    res = {}
    mc = D.mjcf_collisions(m)
    reports = {}
    for k, (L, J, _) in parsed.items():
        reports[f'{k}_links'] = {n: D.inertia_report(l.mass, l.com, l.I, l.collisions) for n, l in L.items() if l.has_inertial}
    reports['gazebo_lumped'] = {b.name: D.inertia_report(b.mass, b.com, b.I, b.collisions) for b in lumped['gazebo']}
    reports['mjcf'] = {b.name: D.inertia_report(b.mass, b.com, b.inertia, mc.get(b.name)) for b in r.bodies}
    res['inertia_reports'] = reports
    bad = [(k, n) for k, R in reports.items() for n, x in R.items() if not (x['pd'] and x['triangle'] and x['pseudo_pd'] and x['symmetric'])]
    res['invalid_inertias_all_files'] = len(bad); res['invalid_inertias_list'] = bad
    res['implausible_size'] = [(k, n, x['gyration_over_extent']) for k, R in reports.items() for n, x in R.items()
                               if x.get('gyration_over_extent', 0) > 1.0]
    res['min_triangle_margin'] = {k: min((x['triangle_margin'], n) for n, x in R.items()) for k, R in reports.items()}
    # mirror symmetry
    mir = {}
    srcs = {'mjcf': {b.name: (b.com, b.inertia) for b in r.bodies}}
    for k, L in lumped.items():
        srcs[k] = {b.name: (b.com, b.I) for b in L}
    for k, S in srcs.items():
        mir[k] = {f'{a}-{b}_{s}': D.mirror_error(*S[f'{a}_{s}'], *S[f'{b}_{s}']) for a, b in (('FL', 'FR'), ('RL', 'RR')) for s in SEGS}
    res['mirror'] = mir
    # Gazebo <gazebo reference> names that match no link or joint
    for k, (L, J, refs) in parsed.items():
        res[f'{k}_gazebo_refs_unknown'] = sorted(x for x in refs if x not in L and x not in J)
    res.update(synthetic(rng))
    res.update(regressor_checks(r, pm, rng))
    res['gravity_torque_urdf_vs_mjcf'], G_samples = gravity_difference(r, lumped, rng)
    return res, G_samples


def random_rotation(rng):
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    return A.quat_to_R(q)


def synthetic(rng, n=20000):
    """Known cases (thin rod: margin 0, solid sphere: 1/3, a tensor with d3 > d1 + d2 rejected) and n random tensors where
    pseudo-inertia positive definiteness must equal strict triangle inequality with positive moments (Wensing, Kim and Slotine 2018)."""
    L, mr = 0.4, 1.0
    rod = np.diag([mr * L ** 2 / 12, mr * L ** 2 / 12, 1e-12])
    sph = np.eye(3) * 0.4 * mr * 0.05 ** 2
    bad = np.diag([1.0, 1.0, 2.5]) * 1e-3
    known = (abs(D.triangle_margin(rod)) < 1e-9 and abs(D.triangle_margin(sph) - 1 / 3) < 1e-12 and D.triangle_margin(bad) < 0
             and not D.inertia_report(1.0, np.zeros(3), bad)['pseudo_pd'] and D.inertia_report(1.0, np.zeros(3), sph)['pseudo_pd'])
    agree, n_valid = 0, 0
    for _ in range(n):
        d = rng.uniform(0.0, 1.0, 3) * 10 ** rng.uniform(-5, -1)
        Rr = random_rotation(rng); Ic = Rr @ np.diag(d) @ Rr.T
        mm = rng.uniform(0.01, 10); c = rng.normal(scale=0.2, size=3)
        tri = d.min() > 0 and all(d.sum() - 2 * dk > 0 for dk in d)
        n_valid += tri
        agree += (D.is_pd(D.pseudo_inertia(mm, c, Ic)) == tri)
    return dict(synthetic_known_cases=float(known), synthetic_pseudo_vs_triangle_agreement=agree / n,
                synthetic_n=n, synthetic_fraction_valid=n_valid / n)


def tilted_gravity(rng, max_tilt):
    ax = rng.normal(size=3); ax[2] = 0
    return axis_angle_R(ax, rng.uniform(0, max_tilt)) @ np.array([0, 0, -G])


LO, HI = np.tile([-0.6, -0.3, -2.5], 4), np.tile([0.6, 1.8, -0.95], 4)   # inside both models' limits, legs clear of the body


def fixed_base_model(frictionloss=True, z=0.6):
    """go2 on scene.xml with the free joint removed (base welded at height z), contacts off so poses never touch anything."""
    sp = mujoco.MjSpec.from_file(str(SCENE_XML))
    sp.delete(sp.body('base').first_joint()); sp.delete(sp.key('home'))
    sp.body('base').pos = [0, 0, z]
    m = sp.compile()
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    if not frictionloss:
        m.dof_frictionloss[:] = 0
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    return m, mujoco.MjData(m)


def regressor_checks(r, pm, rng, n=200):
    pi = D.params(*mass_params(r))
    mf, df = fixed_base_model()
    pdat = pm.createData()
    e_pin = e_mj = 0.0
    for _ in range(n):
        q = rng.uniform(LO, HI); g = tilted_gravity(rng, np.pi)
        tau = D.gravity_regressor(r, q, g) @ pi
        pm.gravity.linear = g
        gp = pin.computeGeneralizedGravity(pm, pdat, np.r_[0, 0, 0, 0, 0, 0, 1, q])[6:]
        mf.opt.gravity[:] = g; df.qpos[:] = q; df.qvel[:] = 0; mujoco.mj_forward(mf, df)
        e_pin = max(e_pin, np.abs(tau - gp).max()); e_mj = max(e_mj, np.abs(tau - df.qfrc_bias).max())
    pm.gravity.linear = np.array([0, 0, -G])
    return dict(regressor_vs_pinocchio_Nm=float(e_pin), regressor_vs_mujoco_bias_Nm=float(e_mj), regressor_n=n)


def gravity_difference(r, lumped, rng, n=500):
    """Holding torque that each URDF's mass parameters predict vs the MJCF's, same poses, gravity upright (base level)."""
    pis = {'mjcf': D.params(*mass_params(r))}
    for k, L in lumped.items():
        pis[k] = D.params(*mass_params(r, L))
    g = np.array([0, 0, -G])
    T = {k: [] for k in pis}
    for _ in range(n):
        Y = D.gravity_regressor(r, rng.uniform(LO, HI), g)
        for k, p in pis.items():
            T[k].append(Y @ p)
    T = {k: np.array(v) for k, v in T.items()}
    out = {}
    for k in lumped:
        dT = T[k] - T['mjcf']
        out[k] = {s: dict(rms_diff_Nm=float(np.sqrt(np.mean(dT[:, j::3] ** 2))), max_diff_Nm=float(np.abs(dT[:, j::3]).max()),
                          rms_mjcf_Nm=float(np.sqrt(np.mean(T['mjcf'][:, j::3] ** 2))))
                  for j, s in enumerate(SEGS)}
    out['n_poses'] = n
    return out, T


# ---- advanced: identification --------------------------------------------------------------------------------------------

def hold(m, d, q, g, side, T, kp=150.0, kd=3.0, offset=0.08, on_step=None):
    """Start every joint `offset` rad off the target on one side, hold the target with joint PD for T s under gravity g, return
    the measured (joint angles, actuator torques, max |dq|) at the end: what encoders and torque sensing report."""
    mujoco.mj_resetData(m, d); m.opt.gravity[:] = g
    d.qpos[:] = q + side * offset
    mujoco.mj_forward(m, d)
    for _ in range(round(T / m.opt.timestep)):
        d.ctrl[:] = np.clip(kp * (q - d.qpos) - kd * d.qvel, *m.actuator_ctrlrange.T)
        mujoco.mj_step(m, d)
        if on_step:
            on_step(m, d)
    return d.qpos.copy(), d.actuator_force.copy(), float(np.abs(d.qvel).max())


def measure(r, m, d, poses, sides, T):
    """Regressors and torques of all poses x sides: lists of (Y (12 x 52), tau (12,))."""
    Ys, taus, dqs = [], [], []
    for q, g in poses:
        for s in sides:
            qm, tau, dq = hold(m, d, q, g, s, T)
            Ys.append(D.gravity_regressor(r, qm, g)); taus.append(tau); dqs.append(dq)
    return Ys, taus, max(dqs)


def calf_cols(r, leg):
    i = r.body(f'{leg}_calf')
    return np.arange(4 * i, 4 * i + 4)


def identify_calves(r, Ys, taus, pi_known):
    """Per leg: rows of that leg's 3 joints, unknowns = the calf's [m, m c]; every other body's parameters taken as known."""
    out = {}
    for li, leg in enumerate(LEGS):
        rows = slice(3 * li, 3 * li + 3)
        x, rms, cov = D.identify_body([Y[rows] for Y in Ys], [t[rows] for t in taus], pi_known, calf_cols(r, leg))
        J = np.vstack([[1, 0, 0, 0], np.c_[-x[1:, None] / x[0] ** 2, np.eye(3) / x[0]]])   # d(m, c)/d(m, h), c = h / m
        cov_mc = J @ cov @ J.T
        out[leg] = dict(mass=float(x[0]), com=(x[1:] / x[0]).tolist(), residual_rms_Nm=rms,
                        mass_std=float(np.sqrt(cov_mc[0, 0])), com_std=np.sqrt(np.diag(cov_mc)[1:]).tolist())
    return out


def errors(est, r):
    e = {}
    for leg, x in est.items():
        b = r.bodies[r.body(f'{leg}_calf')]
        e[leg] = dict(mass_rel=abs(x['mass'] - b.mass) / b.mass, com_m=float(np.linalg.norm(np.subtract(x['com'], b.com))))
    return dict(per_leg=e, mass_rel_max=max(v['mass_rel'] for v in e.values()), com_max_m=max(v['com_m'] for v in e.values()))


def mass_err_over_gap(est, r, lumped):
    """Largest identified calf mass error over the four legs, divided by the Gazebo URDF - MJCF calf mass gap."""
    gap = abs(by_name(lumped['gazebo'])['FL_calf'].mass - r.bodies[r.body('FL_calf')].mass)
    return max(abs(x['mass'] - r.bodies[r.body(f'{l}_calf')].mass) for l, x in est.items()) / gap


def holdout_rms(r, lumped, est, hold_poses):
    """(RMS gravity-torque error of the Gazebo URDF parameters, RMS error with the identified calves) against the MJCF."""
    pi_true = D.params(*mass_params(r)); pi_urdf = D.params(*mass_params(r, lumped['gazebo']))
    pi_id = pi_true.copy()
    for leg, x in est.items():
        pi_id[calf_cols(r, leg)] = [x['mass'], *(x['mass'] * np.asarray(x['com']))]
    dU, dI = [], []
    for q, g in hold_poses:
        Y = D.gravity_regressor(r, q, g)
        dU.append(Y @ (pi_urdf - pi_true)); dI.append(Y @ (pi_id - pi_true))
    return float(np.sqrt(np.mean(np.square(dU)))), float(np.sqrt(np.mean(np.square(dI))))


def advanced(r, lumped, rng, n_id=60, n_hold=30, T=0.5):
    res = {}
    pi_true = D.params(*mass_params(r)); pi_urdf = D.params(*mass_params(r, lumped['gazebo']))
    hip_thigh = np.r_[[np.arange(4 * r.body(f'{l}_{s}'), 4 * r.body(f'{l}_{s}') + 4) for l in LEGS for s in ('hip', 'thigh')]].ravel()
    res['urdf_vs_mjcf_hip_thigh_params_max'] = float(np.abs(pi_true[hip_thigh] - pi_urdf[hip_thigh]).max())
    poses = [(rng.uniform(LO, HI), tilted_gravity(rng, np.radians(45))) for _ in range(n_id)]
    hold_poses = [(rng.uniform(LO, HI), tilted_gravity(rng, np.radians(45))) for _ in range(n_hold)]
    # rank of the full-leg problem (hip, thigh, calf of FL unknown: 12 parameters), tilted and upright gravity
    cols = np.r_[[np.arange(4 * r.body(f'FL_{s}'), 4 * r.body(f'FL_{s}') + 4) for s in SEGS]].ravel()
    for tag, P in (('tilted', poses), ('upright', [(q, np.array([0, 0, -G])) for q, _ in poses])):
        Yf = np.vstack([D.gravity_regressor(r, q, g)[:3][:, cols] for q, g in P])
        sv, rank, zero = D.svd_rank(Yf)
        res[f'full_leg_{tag}'] = dict(rank=rank, zero_columns=zero, singular_values=sv.tolist())
        Yc = np.vstack([D.gravity_regressor(r, q, g)[:3][:, calf_cols(r, 'FL')] for q, g in P])
        res[f'calf_cond_{tag}'] = float(np.linalg.cond(Yc / np.linalg.norm(Yc, axis=0)))
    res['full_leg_rank'] = res['full_leg_tilted']['rank']
    # simulated measurements
    m0, d0 = fixed_base_model(frictionloss=False); m1, d1 = fixed_base_model(frictionloss=True)
    conds = {'nofric': (m0, d0, (1,)), 'single': (m1, d1, (1,)), 'twosided': (m1, d1, (1, -1))}
    est = {}
    for c, (m, d, sides) in conds.items():
        Ys, taus, dq = measure(r, m, d, poses, sides, T)
        est[c] = identify_calves(r, Ys, taus, pi_true)
        res[f'id_{c}'] = dict(estimates=est[c], errors=errors(est[c], r), rows=3 * len(Ys), max_dq_end=dq,
                              torque_residual_rms_Nm=float(np.sqrt(np.mean([(t - Y @ pi_true) ** 2 for Y, t in zip(Ys, taus)]))))
    for c in ('nofric', 'twosided'):
        res[f'id_{c}_mass_rel_err_max'] = res[f'id_{c}']['errors']['mass_rel_max']
        res[f'id_{c}_com_err_max_m'] = res[f'id_{c}']['errors']['com_max_m']
    res['urdf_mjcf_calf_mass_gap_kg'] = abs(by_name(lumped['gazebo'])['FL_calf'].mass - r.bodies[r.body('FL_calf')].mass)
    res['id_twosided_mass_err_over_urdf_gap'] = mass_err_over_gap(est['twosided'], r, lumped)
    # settle time sweep, single-sided: the failure case
    sweep = {}
    for Ts in (0.25, 0.5, 1.0, 2.0, 4.0):
        Ys, taus, dq = measure(r, m1, d1, poses, (1,), Ts)
        e = errors(identify_calves(r, Ys, taus, pi_true), r)
        Ys2, taus2, _ = measure(r, m1, d1, poses, (1, -1), Ts)
        e2 = errors(identify_calves(r, Ys2, taus2, pi_true), r)
        sweep[str(Ts)] = dict(single_mass_rel_max=e['mass_rel_max'], single_com_max_m=e['com_max_m'], max_dq_end=dq,
                              friction_residual_rms_Nm=float(np.sqrt(np.mean([(t - Y @ pi_true) ** 2 for Y, t in zip(Ys, taus)]))),
                              twosided_mass_rel_max=e2['mass_rel_max'], twosided_com_max_m=e2['com_max_m'])
    res['settle_sweep'] = sweep
    # held-out poses: gravity torque predicted by the URDF parameters and by the identified ones, against the MJCF truth
    res['holdout_rms_urdf_Nm'], res['holdout_rms_id_Nm'] = holdout_rms(r, lumped, est['twosided'], hold_poses)
    res['holdout_rms_ratio_id_over_urdf'] = res['holdout_rms_id_Nm'] / res['holdout_rms_urdf_Nm']
    res['n_id_poses'], res['n_holdout_poses'], res['settle_s'] = n_id, n_hold, T
    return res, poses


# ---- figures -------------------------------------------------------------------------------------------------------------

def fig_differences(res):
    """Six panels, URDF minus MJCF, for the base and the FL leg (the other legs repeat these values; mirror errors are in
    results.json): mass, centre of mass, inertia, joint position limits, effort limits, collision volume."""
    srcs = (('gazebo', fs.C['est'], 'Gazebo URDF'), ('unitree', fs.C['est2'], 'Unitree URDF'))
    sel = [x for x in res['bodies'] if x['body'] in ('base', 'FL_hip', 'FL_thigh', 'FL_calf')]
    names = [x['body'] for x in sel]
    fig, ax = plt.subplots(2, 3, figsize=(10.4, 6.0))
    x = np.arange(len(sel)); w = 0.38
    for k, (key, scale, lab) in enumerate((('dmass_rel', 100, 'mass (%)'), ('dcom_m', 1000, 'centre of mass offset (mm)'),
                                           ('dI_rel', 100, 'inertia |ΔI|/|I| (%)'))):
        a = ax[0, k]
        for o, (src, col, l) in enumerate(srcs):
            a.bar(x + (o - 0.5) * w, [s_[src][key] * scale for s_ in sel], w, color=col, label=l)
        a.set_xticks(x, names, rotation=30, ha='right'); a.set_ylabel(lab); a.axhline(0, color=fs.C['ink'], lw=0.6)
    ax[0, 0].legend(); ax[0, 0].set_title('URDF minus MJCF, per body')
    J = [j for j in res['joints'] if j['joint'].startswith(('FL', 'RL'))]
    jl = [j['joint'].replace('_joint', '') for j in J]
    for k, (keys, lab) in enumerate(((('lower', 'upper'), 'position limit (rad)'), (('effort',), 'effort limit (N m)'))):
        a = ax[1, k]
        labels = [f'{n} {q}' if len(keys) > 1 else n for n in jl for q in keys]
        xx = np.arange(len(labels))
        for o, (src, col, l) in enumerate(srcs):
            a.bar(xx + (o - 0.5) * w, [(j[src][q] or 0) - j['mjcf'][q] for j in J for q in keys], w, color=col)
        a.set_xticks(xx, labels, rotation=60, ha='right', fontsize=7.5); a.set_ylabel(lab); a.axhline(0, color=fs.C['ink'], lw=0.6)
    ax[1, 0].set_title('Joint limits, URDF minus MJCF')
    a = ax[1, 2]
    rows = res['collision']['rows']
    xx = np.arange(len(rows))
    for o, (src, col, l) in enumerate(srcs):
        a.bar(xx + (o - 0.5) * w, [100 * (r_[f'{src}_volume_m3'] / r_['mjcf_volume_m3'] - 1) for r_ in rows], w, color=col)
    a.set_xticks(xx, [r_['body'] for r_ in rows], rotation=30, ha='right'); a.set_ylabel('collision volume (%)')
    a.axhline(0, color=fs.C['ink'], lw=0.6); a.set_title('Collision geometry')
    fs.save(fig, FIG / 'differences')


def _outline(ax, g, color, ls='-'):
    """x-z section (body frame) of a primitive whose axis lies in the x-z plane."""
    s, R, p = g['size'], g['R'], g['pos']
    if g['type'] == 'sphere':
        ax.add_patch(plt.Circle((p[0], p[2]), s[0], fill=False, ec=color, ls=ls, lw=1.1)); return
    if g['type'] == 'box':
        u, v, hu, hv = R[:, 0], R[:, 2], s[0], s[2]
    else:
        u, v, hu, hv = R[:, 2], R[:, 0], s[1], s[0]
    c = np.array([p + a * hu * u + b * hv * v for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))])
    ax.plot(c[:, 0], c[:, 2], color=color, ls=ls, lw=1.1)


def fig_calf_diagram(r, m, parsed, lumped):
    """Calf in its x-z plane: collision outlines of both descriptions, the URDF parts' centres of mass, the lumped centre and
    the MJCF centre. Shows what lump() does and where the two files disagree."""
    L, J, _ = parsed['gazebo']
    fig, ax = plt.subplots(figsize=(4.4, 6.0))
    ax.set_aspect('equal'); ax.grid(False)
    for g in D.mjcf_collisions(m)['FL_calf']:
        _outline(ax, g, fs.C['gt'])
    for g in by_name(lumped['gazebo'])['FL_calf'].collisions:
        _outline(ax, g, fs.C['est'], '--')
    calf, foot = L['lf_lower_leg_link'], L['lf_foot_link']
    fj = J['lf_foot_joint']
    pts = [(calf.com, f'URDF calf link {calf.mass:.3f} kg', fs.C['est'], 'o'),
           (fj.xyz + foot.com, f'URDF foot link {foot.mass:.3f} kg', fs.C['est'], 's'),
           (by_name(lumped['gazebo'])['FL_calf'].com, f'lumped {by_name(lumped["gazebo"])["FL_calf"].mass:.3f} kg', fs.C['est2'], 'D'),
           (r.bodies[r.body('FL_calf')].com, f'MJCF calf {r.bodies[r.body("FL_calf")].mass:.4f} kg', fs.C['gt'], '*')]
    for p, t, col, mk in pts:
        ax.plot(p[0], p[2], mk, color=col, ms=7 if mk != '*' else 11, mfc=col if mk != 'o' else 'white', label=t)
    ax.plot([0, 0], [0.01, -0.24], color=fs.C['grid'], lw=0.8, zorder=0)
    ax.plot(0, 0, 'o', color=fs.C['ink'], ms=5); ax.annotate('knee axis (y)', (0, 0), (0.012, 0.004), fontsize=8)
    ax.plot([], [], color=fs.C['gt'], label='MJCF collision'); ax.plot([], [], color=fs.C['est'], ls='--', label='Gazebo URDF collision')
    ax.set_xlim(-0.06, 0.06); ax.set_ylim(-0.245, 0.015)
    ax.set_xlabel('x, calf frame (m)'); ax.set_ylabel('z, calf frame (m)')
    ax.set_title('FL calf: lumping calf + foot, and the MJCF body')
    ax.legend(loc='lower left', fontsize=7.5, bbox_to_anchor=(1.0, 0.0))
    fs.save(fig, FIG / 'calf_lumping')


def fig_inertia_validity(inter):
    R = inter['inertia_reports']
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 4.2))
    srcs = (('gazebo_links', fs.C['est'], 'Gazebo URDF links'), ('unitree_links', fs.C['est2'], 'Unitree URDF links'),
            ('mjcf', fs.C['gt'], 'MJCF bodies'))
    for k, (src, col, lab) in enumerate(srcs):
        n = sorted(R[src]); y = np.arange(len(n))
        ax[0].scatter([R[src][x]['triangle_margin'] for x in n], np.full(len(n), k) + np.linspace(-0.3, 0.3, len(n)), color=col, s=14, label=lab)
        ax[1].scatter([R[src][x].get('gyration_over_extent', np.nan) for x in n], np.full(len(n), k) + np.linspace(-0.3, 0.3, len(n)),
                      color=col, s=14)
        out = [x for x in n if R[src][x].get('gyration_over_extent', 0) > 1]
        if out:
            g = max(R[src][x]['gyration_over_extent'] for x in out)
            ax[1].annotate(', '.join(out), (g, k), (g, k + 0.38), fontsize=7.5, ha='right', color=fs.C['bad'])
    ax[0].axvline(0, color=fs.C['bad'], lw=0.8); ax[0].axvline(1 / 3, color=fs.C['muted'], lw=0.6, ls=':')
    ax[0].set_xlabel('triangle margin (tr I − 2 d_max) / tr I'); ax[0].set_yticks(range(3), [s[2] for s in srcs])
    ax[0].set_title('Triangle inequality: < 0 is impossible'); ax[0].annotate('sphere 1/3', (1 / 3, 2.4), fontsize=7.5, ha='right')
    ax[1].set_xscale('log'); ax[1].axvline(1, color=fs.C['bad'], lw=0.8)
    ax[1].set_xlabel('radius of gyration / extent of collision geometry'); ax[1].set_yticks(range(3), [s[2] for s in srcs])
    ax[1].set_title('Size check: > 1 needs mass outside the geometry')
    fs.save(fig, FIG / 'inertia_validity')


def fig_gravity_torque(T):
    fig, ax = plt.subplots(1, 3, figsize=(9.6, 3.4))
    for j, s in enumerate(SEGS):
        a = ax[j]
        x = T['mjcf'][:, j::3].ravel()
        a.scatter(x, T['gazebo'][:, j::3].ravel() - x, s=3, color=fs.C['est'], label='Gazebo URDF', alpha=0.6, rasterized=True)
        a.scatter(x, T['unitree'][:, j::3].ravel() - x, s=3, color=fs.C['est2'], label='Unitree URDF', alpha=0.6, rasterized=True)
        a.set_xlabel(f'{s} holding torque, MJCF (N m)'); a.axhline(0, color=fs.C['gt'], lw=0.8)
    ax[0].set_ylabel('URDF minus MJCF (N m)'); ax[0].legend(markerscale=3)
    ax[0].set_title('Gravity torque each URDF predicts, 500 poses, base level')
    fs.save(fig, FIG / 'gravity_torque_difference')


def fig_identification(adv, r, lumped):
    fig, ax = plt.subplots(1, 3, figsize=(10.4, 3.6))
    for tag, col in (('tilted', fs.C['est']), ('upright', fs.C['plan'])):
        sv = adv[f'full_leg_{tag}']['singular_values']
        ax[0].semilogy(np.arange(1, len(sv) + 1), np.maximum(sv, 1e-17), 'o-', color=col, label=f'gravity {tag}')
    ax[0].axvline(6.5, color=fs.C['muted'], lw=0.6, ls=':'); ax[0].set_xlabel('index'); ax[0].set_ylabel('singular value (columns normalised)')
    ax[0].set_title('FL leg, 12 mass parameters: rank 6'); ax[0].legend()
    true_m = np.array([r.bodies[r.body(f'{l}_calf')].mass for l in LEGS])
    urdf_m = np.array([by_name(lumped['gazebo'])[f'{l}_calf'].mass for l in LEGS])
    x = np.arange(4)
    ax[1].plot(x, true_m, '_', ms=26, mew=2, color=fs.C['gt'], label='MJCF (truth in sim)')
    ax[1].plot(x, urdf_m, '_', ms=26, mew=2, color=fs.C['plan'], label='Gazebo URDF')
    for o, (c, col, lab) in enumerate((('nofric', fs.C['good'], 'no friction'), ('twosided', fs.C['est'], 'friction, two-sided'),
                                       ('single', fs.C['bad'], 'friction, one-sided'))):
        e = adv[f'id_{c}']['estimates']
        ax[1].errorbar(x + (o - 1) * 0.15, [e[l]['mass'] for l in LEGS], yerr=[3 * e[l]['mass_std'] for l in LEGS], fmt='o',
                       color=col, ms=4, capsize=2, label=lab)
        ax[2].bar(x + (o - 1) * 0.27, [adv[f'id_{c}']['errors']['per_leg'][l]['com_m'] * 1000 for l in LEGS], 0.27, color=col, label=lab)
    ax[1].set_xticks(x, LEGS); ax[1].set_ylabel('calf mass (kg)'); ax[1].set_title('Identified calf mass (±3σ)')
    ax[1].set_ylim(0.195, 0.29); ax[1].legend(fontsize=7, ncol=2, loc='upper center')
    ax[2].set_xticks(x, LEGS); ax[2].set_yscale('log'); ax[2].set_ylabel('|c_id − c_MJCF| (mm)'); ax[2].set_title('Calf COM error')
    ax[2].axhline(5, color=fs.C['plan'], lw=0.8, ls='--'); ax[2].annotate('5 mm acceptance', (3.4, 5.5), fontsize=7.5, ha='right')
    fs.save(fig, FIG / 'identification')


def fig_friction_failure(adv, trace):
    sw = adv['settle_sweep']; Ts = np.array([float(k) for k in sw])
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.6))
    t, res_p, res_m = trace
    ax[0].plot(t, res_p, color=fs.C['est'], label='approach from +0.08 rad')
    ax[0].plot(t, res_m, color=fs.C['est2'], label='approach from −0.08 rad')
    ax[0].plot(t, 0.5 * (res_p + res_m), color=fs.C['ink'], label='mean of both')
    ax[0].axhline(0, color=fs.C['gt'], lw=0.8); ax[0].set_xlabel('time holding the pose (s)')
    ax[0].set_ylabel('measured − gravity torque, FL hip (N m)'); ax[0].set_title('Friction bias while settling'); ax[0].legend()
    ax[0].set_ylim(-0.25, 0.25)
    ax[1].loglog(Ts, [sw[k]['single_mass_rel_max'] * 100 for k in sw], 'o-', color=fs.C['bad'], label='one-sided, mass')
    ax[1].loglog(Ts, [sw[k]['twosided_mass_rel_max'] * 100 for k in sw], 'o-', color=fs.C['est'], label='two-sided, mass')
    ax[1].loglog(Ts, [sw[k]['single_com_max_m'] * 1000 for k in sw], 's--', color=fs.C['bad'], label='one-sided, COM (mm)')
    ax[1].loglog(Ts, [sw[k]['twosided_com_max_m'] * 1000 for k in sw], 's--', color=fs.C['est'], label='two-sided, COM (mm)')
    ax[1].axhline(5, color=fs.C['plan'], lw=0.8, ls=':')
    ax[1].set_xlabel('settle time per pose (s)'); ax[1].set_ylabel('max over legs: mass error (%) / COM error (mm)')
    ax[1].set_title('Calf identification vs settle time'); ax[1].legend(fontsize=7)
    fs.save(fig, FIG / 'friction_failure')


def friction_trace(r, poses, T=1.5):
    m, d = fixed_base_model(frictionloss=True)
    q, g = poses[0]
    out = []
    for s in (1, -1):
        t, e = [], []

        def log(m, d):
            t.append(d.time); e.append(d.actuator_force[0] - (D.gravity_regressor(r, d.qpos, g) @ D.params(*mass_params(r)))[0])
        hold(m, d, q, g, s, T, on_step=log)
        out.append(np.array(e))
    return np.array(t), out[0], out[1]


# ---- media ---------------------------------------------------------------------------------------------------------------

SBS_X = {'': 0.9, 'C_': 0.0, 'U_': -0.9}      # x of the MJCF-mesh, MJCF-collision and URDF copies (m)


def side_by_side_model():
    """One scene: Menagerie MJCF with its meshes (x = +0.9 m), the same MJCF showing only its collision geometry (x = 0), and
    the Gazebo URDF through MuJoCo's importer (x = -0.9 m), which has only collision primitives (its visuals are Collada meshes)."""
    w = mujoco.MjSpec.from_file(str(SCENE_XML)); w.delete(w.key('home'))
    w.body('base').pos = [SBS_X[''], 0, 0.445]
    g = mujoco.MjSpec.from_file(str(GO2_XML)); g.delete(g.key('home'))
    w.worldbody.add_frame(pos=[0, 0, 0]).attach_body(g.body('base'), 'C_', '')
    w.worldbody.add_frame(pos=[SBS_X['U_'], 0, 0]).attach_body(urdf_spec().body('base_link'), 'U_', '')
    m = w.compile()
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    hexrgb = lambda h, a=1.0: [int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)] + [a]  # noqa: E731
    for gi in range(m.ngeom):
        b = m.body(m.geom_bodyid[gi]).name
        if b.startswith('C_'):
            if m.geom_group[gi] == 3:
                m.geom_group[gi] = 0; m.geom_rgba[gi] = hexrgb(fs.C['gt']); m.geom_matid[gi] = -1
            else:
                m.geom_rgba[gi][3] = 0
        elif b.startswith('U_'):
            m.geom_rgba[gi] = hexrgb(fs.C['est']); m.geom_group[gi] = 0
    d = mujoco.MjData(m)
    return m, d


def sbs_pose(m, d, q_mjcf, q_urdf, z=None):
    """Set the joints of the three copies (MJCF order for the first two, URDF names for the third), clamp each to its own
    model's limits, put each base so its lowest foot touches the floor (or at height z), and run forward kinematics."""
    names = [f'{l}_{s}_joint' for l in LEGS for s in SEGS]
    umap = {'FL': 'lf', 'FR': 'rf', 'RL': 'lh', 'RR': 'rh'}; useg = {'hip': 'hip', 'thigh': 'upper_leg', 'calf': 'lower_leg'}
    bases = {}
    for pre, q in (('', q_mjcf), ('C_', q_mjcf), ('U_', q_urdf)):
        for k, n in enumerate(names):
            jn = pre + n if pre != 'U_' else f"U_{umap[n[:2]]}_{useg[n.split('_')[1]]}_joint"
            j = m.joint(jn)
            d.qpos[j.qposadr[0]] = np.clip(q[k], *m.jnt_range[j.id])
        fj = m.body(pre + 'base' if pre != 'U_' else 'U_base_link').jntadr[0]
        bases[pre] = m.jnt_qposadr[fj]
    for pre, x in SBS_X.items():
        a = bases[pre]; d.qpos[a:a + 7] = [x, 0, 0.5, 1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
    if z is not None:
        for a in bases.values():
            d.qpos[a + 2] = z
        mujoco.mj_forward(m, d)
        return
    feet = {pre: [m.geom(pre + leg).id for leg in LEGS] for pre in ('', 'C_')}
    feet['U_'] = [gi for gi in range(m.ngeom) if m.geom_type[gi] == mujoco.mjtGeom.mjGEOM_SPHERE
                  and m.body(m.geom_bodyid[gi]).name.startswith('U_')]
    for pre, a in bases.items():
        d.qpos[a + 2] -= min(d.geom_xpos[gi][2] - m.geom_size[gi][0] for gi in feet[pre])
    mujoco.mj_forward(m, d)


def render_side_by_side():
    m, d = side_by_side_model()
    sbs_pose(m, d, sim.Q_STAND, sim.Q_STAND)
    imgs = []
    for az, el in ((120, -18), (90, -4)):
        rec = sim.Recorder(m, None, width=1280, height=720, distance=2.3, azimuth=az, elevation=el, track='C_base')
        if not rec.ok:
            return None
        imgs.append(rec.render(d)); rec.still(d, MEDIA / f'side_by_side_az{az}.png'); rec.__exit__()
    fig, ax = plt.subplots(2, 1, figsize=(9.6, 10.8))
    for a, im, t in zip(ax, imgs, ('three-quarter view', 'side view')):
        a.imshow(im); a.set_axis_off(); a.set_title(t)
    fig.suptitle('Same joint angles (stand pose). Left to right: Gazebo URDF collision (blue), MJCF collision (grey), MJCF meshes',
                 fontsize=9.5)
    fs.save(fig, FIG / 'side_by_side')
    return True


def video_side_by_side():
    """16 s: 6 s camera orbit around the three copies standing, then 10 s with the bases held 0.55 m up and the same rear-thigh
    command to every copy, from 0.9 rad to 4.6 rad, down to -1.6 rad and back, each copy clamped to its own limits."""
    m, d = side_by_side_model()
    path = MEDIA / 'side_by_side.mp4'
    with sim.Recorder(m, path, width=960, height=540, fps=30, distance=2.1, azimuth=90, elevation=-12, track='C_base') as rec:
        if not rec.ok:
            return None
        n1, n2 = 6 * 30, 10 * 30
        for k in range(n1 + n2):
            d.time = k / 30
            if k < n1:
                rec.cam.azimuth = 90 + 360 * k / n1; q = sim.Q_STAND
            else:
                rec.cam.azimuth = 90
                s = (k - n1) / n2
                th = np.interp(s, [0, 0.3, 0.7, 1.0], [0.9, 4.6, -1.6, 0.9])
                q = sim.Q_STAND.copy(); q[[7, 10]] = th
            sbs_pose(m, d, q, q, z=None if k < n1 else 0.55)
            rec(m, d)
            if k == n1 + int(0.3 * n2):                      # rear thighs at the upper limit of each model
                rec.still(d, MEDIA / 'side_by_side.png')
            if k == n1 + int(0.7 * n2):                      # rear thighs at the lower limit
                rec.still(d, MEDIA / 'side_by_side_lower.png')
    return path


class GravityRec(sim.Recorder):
    """sim.Recorder that also draws the gravity vector (amber arrow, 0.35 m) from the base."""

    def render(self, d):
        if not self.ok:
            return None
        self.renderer.update_scene(d, camera=self.cam, scene_option=self.opt)
        scn = self.renderer.scene
        g = self.m.opt.gravity / np.linalg.norm(self.m.opt.gravity)
        p1 = d.xpos[1] + np.array([0, 0, 0.09]); p0 = p1 - 0.35 * g      # tip just above the base, pointing along g
        rgba = np.array([int(fs.C['plan'][i:i + 2], 16) / 255 for i in (1, 3, 5)] + [1], np.float32)
        if scn.ngeom < scn.maxgeom:
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
            mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, 0.012, p0, p1); scn.ngeom += 1
        return self.renderer.render()


def video_identification(r, poses, n=10, T=0.5):
    """The first n identification poses, both approach sides, 0.5 s each: 10 s. Real time."""
    m, d = fixed_base_model(frictionloss=True)
    path = MEDIA / 'identification.mp4'
    with GravityRec(m, path, width=960, height=540, fps=30, distance=1.6, azimuth=120, elevation=-10) as rec:
        if not rec.ok:
            return None
        t0 = [0.0]

        def step(m, d):
            tt = d.time; d.time = t0[0] + tt; rec(m, d); d.time = tt
        for i, (q, g) in enumerate(poses[:n]):
            for s in (1, -1):
                hold(m, d, q, g, s, T, on_step=step); t0[0] += T
                if i == 2 and s == 1:
                    rec.still(d, MEDIA / 'identification.png')
    return path


# ---- main ----------------------------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--no-media', action='store_true')
    ap.add_argument('--json', default=str(HERE / 'results.json')); a = ap.parse_args()
    t0 = time.time(); rng = np.random.default_rng(2)
    r = A.parse(); m = mujoco.MjModel.from_xml_path(str(GO2_XML)); pm = pin.buildModelFromMJCF(str(GO2_XML))
    res = {}
    fund, parsed, lumped = fundamental(r, m)
    res['fundamental'] = fund
    inter, T = intermediate(r, m, pm, parsed, lumped, rng)
    res['intermediate'] = inter
    adv, poses = advanced(r, lumped, rng)
    res['advanced'] = adv
    fig_differences(fund); fig_calf_diagram(r, m, parsed, lumped); fig_inertia_validity(inter); fig_gravity_torque(T)
    fig_identification(adv, r, lumped)
    tr = friction_trace(r, poses); fig_friction_failure(adv, tr)
    t, rp, rm = tr
    adv['friction_trace'] = dict(pose=0, joint='FL_hip', plateau_Nm=[float(rp[np.argmin(abs(t - 0.15))]), float(rm[np.argmin(abs(t - 0.15))])],
                                 mean_of_sides_max_after_0p15s_Nm=float(np.abs(0.5 * (rp + rm))[t >= 0.15].max()),
                                 one_side_at_1p5s_Nm=float(abs(rp[-1])))
    flat = {**fund, **inter, **adv}
    gl = can_render(m)
    if gl and not a.no_media:
        render_side_by_side()
        vids = [video_side_by_side(), video_identification(r, poses)]
        sizes = {p.name: p.stat().st_size for p in vids if p is not None and p.exists()}
        res['media_bytes'] = sizes; flat['media_mp4_under_6MB'] = float(len(sizes) == 2 and all(s < 6e6 for s in sizes.values()))
    res['checks'] = {k: dict(measured=flat[k], op=op, threshold=thr, passed=check(k, flat[k])) for k, (op, thr) in ACCEPT.items() if k in flat}
    res['skipped_checks'] = [k for k in ACCEPT if k not in flat]
    for k in res['skipped_checks']:
        warnings.warn(f'check {k} skipped')
    print('\nacceptance:')
    for k, c in res['checks'].items():
        print(f'  {"PASS" if c["passed"] else "MISS"}  {k:42s} {c["measured"]:.4g}  ({c["op"]} {c["threshold"]})')
    res['wall_time_s'] = round(time.time() - t0, 1); res['load_average_1min_at_end'] = round(os.getloadavg()[0], 1)
    pathlib.Path(a.json).write_text(json.dumps(res, indent=2, default=float) + '\n')
    print(f'wrote {a.json} in {res["wall_time_s"]} s')


if __name__ == '__main__':
    main()
