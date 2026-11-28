#!/usr/bin/env python3
"""Experiment for module 01 (robot anatomy), all three levels.
  fundamental:  anatomy.parse() of go2.xml against the compiled MuJoCo model and Pinocchio's MJCF import (counts, bodies,
                joints, actuators); kinematic tree diagram; front, side and top renders with the joint axes projected onto them,
                the camera model checked against MuJoCo's segmentation image; turntable video.
  intermediate: forward kinematics and body velocities of the parsed tree against MuJoCo; centre of mass and centroidal momentum
                against Pinocchio; transmission (u -> generalised force) and armature against MuJoCo; joint-range diagram;
                torque budget at the stand pose; joint-sweep video.
  advanced:     underactuation. B has six zero rows; in flight random joint torques leave the centroidal momentum rate at m_tot g;
                in stance it equals m_tot g plus the contact wrenches; integrator drift of momentum (Euler vs RK4); zero-momentum
                reorientation by a joint-space loop. Flight and reorientation videos.
Writes results.json, figures/ (PNG + SVG) and media/ (MP4 + PNG).
usage: python validate.py [--n 1000] [--no-media]"""
import argparse, json, os, pathlib, sys, time, warnings
from typing import Any
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'm00_math'))
sys.path.insert(0, str(HERE.parent / 'm04_kinematics'))
import sim  # noqa: E402  (first: sets MUJOCO_GL before mujoco is imported)
import figstyle as fs  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import pinocchio as pin  # noqa: E402
import anatomy as A  # noqa: E402
from common import G, GO2_XML, LEGS, quat_to_R  # noqa: E402
from rotations import R_to_rpy  # noqa: E402
from kinematics import Leg  # noqa: E402

FIG, MEDIA = HERE / 'figures', HERE / 'media'

# Acceptance thresholds, fixed before the first full run (MODULE_SPEC.md). (op, threshold)
ACCEPT = {
    # fundamental
    'count_mismatches_vs_mujoco_pinocchio': ('<=', 0), 'parse_bodies_vs_mujoco': ('<', 1e-12),
    'parse_joints_vs_mujoco': ('<', 1e-12), 'parse_actuators_vs_mujoco': ('<', 1e-12), 'total_mass_vs_mujoco_kg': ('<', 1e-12),
    'joint_limits_vs_pinocchio': ('<', 1e-12), 'armature_damping_vs_pinocchio': ('<', 1e-12),
    'camera_projection_bbox_px': ('<=', 1.5),
    # intermediate
    'fk_body_pose_vs_mujoco': ('<', 1e-12), 'fk_joint_anchor_axis_vs_mujoco': ('<', 1e-12),
    'body_velocity_vs_mujoco': ('<', 1e-12), 'com_vs_pinocchio_m': ('<', 1e-12),
    'centroidal_momentum_vs_pinocchio': ('<', 1e-12), 'actuator_force_vs_mujoco': ('<', 1e-12),
    'armature_in_mass_matrix': ('<', 1e-12), 'stand_torque_utilisation_max': ('<', 0.5),
    # advanced
    'B_base_rows_max_abs': ('<=', 0.0), 'qfrc_actuator_base_rows_max_abs': ('<=', 0.0), 'rank_B': ('range', (12, 12)),
    'flight_momentum_rate_vs_Mg_fd': ('<', 1e-6), 'flight_momentum_rate_vs_Mg_pinocchio': ('<', 1e-8),
    'stance_momentum_rate_vs_wrench': ('<', 1e-5), 'flight_drift_order_euler': ('range', (1.6, 2.4)),
    'flight_drift_rk4_ang_kgm2s': ('<', 1e-4), 'reorient_pitch_5_cycles_deg': ('>', 5.0),
    'reorient_max_ang_momentum': ('<', 1e-4), 'reorient_reverse_ratio': ('range', (-1.1, -0.9)),
    'reorient_half_speed_ratio': ('range', (0.9, 1.1)), 'media_mp4_under_6MB': ('>=', 1.0),
    # added after review, before the run that tests it: a loop of the same area over a different region must rotate the base
    # by a different amount; 0.1 is three times the 3% speed effect, which bounds what tracking error alone can cause
    'reorient_other_region_rel_diff': ('>', 0.1),
}
# Checks that need offscreen rendering (MUJOCO_GL=glfw and a live X display). Without one they are skipped with a warning and
# left out of results.json['checks'] (MODULE_SPEC.md); media_mp4_under_6MB is also left out with --no-media.
GL_CHECKS = ('camera_projection_bbox_px', 'media_mp4_under_6MB')


def can_render(m):
    """True if MuJoCo can open an offscreen GL context here (glfw needs a live X display)."""
    try:
        with mujoco.Renderer(m, 8, 8):
            return True
    except Exception as e:                                          # mujoco.FatalError when the display is missing
        warnings.warn(f'offscreen rendering unavailable ({type(e).__name__}: {e}): skipping {", ".join(GL_CHECKS)}, '
                      'views_annotated.png and the videos')
        return False


def check(key, val):
    op, thr = ACCEPT[key]
    return bool(thr[0] <= val <= thr[1]) if op == 'range' else bool({'<': val < thr, '<=': val <= thr, '>': val > thr,
                                                                       '>=': val >= thr}[op])


def mx(*arrs):
    return float(max(np.abs(np.asarray(a, float)).max() for a in arrs))


def to_pin(q, v=None, a=None) -> Any:                  # qp, (qp, vp) or (qp, vp, ap): one item per argument given
    """MuJoCo (q, v, a) to Pinocchio's free-flyer convention: quaternion x y z w; linear velocity in the base frame;
    a_lin = R^T p_ddot - w x v_local (derivative of the base-frame velocity vector)."""
    R = quat_to_R(q[3:7]); qp = np.r_[q[:3], q[4:7], q[3], q[7:]]
    if v is None:
        return qp
    vl = R.T @ v[:3]; vp = np.r_[vl, v[3:]]
    if a is None:
        return qp, vp
    return qp, vp, np.r_[R.T @ a[:3] - np.cross(v[3:6], vl), a[3:]]


def random_state(rng, robot):
    q = np.r_[rng.normal(size=3), rng.normal(size=4), rng.uniform(-np.pi, np.pi, robot.nv - 6)]
    q[3:7] /= np.linalg.norm(q[3:7])
    return q, rng.normal(size=robot.nv)


# ---- fundamental ---------------------------------------------------------------------------------------------------------

def fundamental(r, m, pm):
    res: dict[str, Any] = {}
    dm = A.dof(r)
    pin_bodies = sum(f.type == pin.FrameType.BODY for f in pm.frames)  # type: ignore[reportAttributeAccessIssue]  # pinocchio: API in a compiled extension, no stubs
    counts = dict(nq=(r.nq, m.nq, pm.nq), nv=(r.nv, m.nv, pm.nv), nu=(len(r.actuators), m.nu, None),     # Pinocchio has no actuators
                  bodies=(len(r.bodies), m.nbody - 1, pin_bodies), joints=(len(r.joints), m.njnt, pm.njoints - 1))
    res['counts'] = {k: dict(parsed=v[0], mujoco=int(v[1]), pinocchio=v[2] if v[2] is None else int(v[2])) for k, v in counts.items()}
    res['count_mismatches_vs_mujoco_pinocchio'] = sum(len({x for x in v if x is not None}) > 1 for v in counts.values())
    res['dof'] = dm
    eb = []
    for i, b in enumerate(r.bodies):
        k = i + 1; Ri = quat_to_R(m.body_iquat[k])
        eb += [b.pos - m.body_pos[k], b.R - quat_to_R(m.body_quat[k]), b.mass - m.body_mass[k], b.com - m.body_ipos[k],
               b.inertia - Ri @ np.diag(m.body_inertia[k]) @ Ri.T, b.parent - (m.body_parentid[k] - 1)]
    res['parse_bodies_vs_mujoco'] = mx(*eb)
    ej = []
    for k, j in enumerate(r.joints):
        typ = {int(mujoco.mjtJoint.mjJNT_FREE): 'free', int(mujoco.mjtJoint.mjJNT_HINGE): 'hinge'}[int(m.jnt_type[k])]
        ej += [float(typ != j.type), j.body - (m.jnt_bodyid[k] - 1), j.qadr - m.jnt_qposadr[k], j.vadr - m.jnt_dofadr[k],
               j.pos - m.jnt_pos[k], float((j.range is not None) != bool(m.jnt_limited[k])),
               j.damping - m.dof_damping[j.vadr], j.armature - m.dof_armature[j.vadr], j.frictionloss - m.dof_frictionloss[j.vadr]]
        if j.type == 'hinge':
            ej += [j.axis - m.jnt_axis[k], np.subtract(j.range, m.jnt_range[k])]
    res['parse_joints_vs_mujoco'] = mx(*ej)
    ea = []
    for k, a in enumerate(r.actuators):
        ea += [a.joint - m.actuator_trnid[k, 0], a.gear - m.actuator_gear[k, 0], np.subtract(a.ctrlrange, m.actuator_ctrlrange[k]),
               float(m.actuator_trntype[k] != mujoco.mjtTrn.mjTRN_JOINT)]
    res['parse_actuators_vs_mujoco'] = mx(*ea)
    res['total_mass_kg'] = r.mass; res['total_mass_vs_mujoco_kg'] = abs(r.mass - m.body_subtreemass[1])
    # Pinocchio's MJCF import: limits, armature and damping per hinge
    el, ed = [], []
    for j in r.joints:
        if j.type != 'hinge':
            continue
        pj = pm.joints[pm.getJointId(j.name)]
        el += [pm.lowerPositionLimit[pj.idx_q] - j.range[0], pm.upperPositionLimit[pj.idx_q] - j.range[1]]
        ed += [pm.armature[pj.idx_v] - j.armature, pm.damping[pj.idx_v] - j.damping]
    res['joint_limits_vs_pinocchio'] = mx(el); res['armature_damping_vs_pinocchio'] = mx(ed)
    res['pinocchio_effort_limits_finite'] = int(np.isfinite(pm.effortLimit).sum())
    res['tables'] = dict(
        bodies=[dict(name=b.name, parent=r.bodies[b.parent].name if b.parent >= 0 else 'world', mass_kg=b.mass,
                     pos_m=b.pos.round(5).tolist(), visual_geoms=b.geoms['visual'], collision_geoms=b.geoms['collision'])
                for b in r.bodies],
        joints=[dict(name=j.name, type=j.type, axis=j.axis.tolist() if j.type == 'hinge' else None,
                     range_rad=list(j.range) if j.range else None, damping=j.damping, armature=j.armature,
                     frictionloss=j.frictionloss, qadr=j.qadr, vadr=j.vadr) for j in r.joints],
        actuators=[dict(name=a.name, kind=a.kind, joint=r.joints[a.joint].name, gear=a.gear, ctrlrange=list(a.ctrlrange))
                   for a in r.actuators])
    return res


# ---- camera model for the annotated views ----------------------------------------------------------------------------------

def mono_camera(scn, eyes=(0, 1)):
    """MuJoCo renders a mono image from the average of its two eye cameras (they sit +-ipd/2 apart); return that average.
    eyes=(0,) returns eye camera 0 alone (the ablation in projection_check)."""
    c0 = scn.camera[0]; cs = [scn.camera[e] for e in eyes]
    return dict(pos=np.mean([np.array(c.pos, float) for c in cs], 0), fw=np.array(c0.forward, float), up=np.array(c0.up, float),
                center=float(np.mean([c.frustum_center for c in cs])), width=c0.frustum_width, top=c0.frustum_top,
                bottom=c0.frustum_bottom, near=c0.frustum_near, ortho=bool(c0.orthographic))


def project(cam, P, W, H):
    """World points (n, 3) to pixel coordinates (n, 2). Camera frame: right = forward x up. Perspective divides by depth and
    scales to the near plane; orthographic does not. A zero frustum width means 'from the aspect ratio'."""
    P = np.atleast_2d(P) - cam['pos']; rt = np.cross(cam['fw'], cam['up'])
    X, Y, Z = P @ rt, P @ cam['up'], P @ cam['fw']
    if not cam['ortho']:
        X, Y = X * cam['near'] / Z, Y * cam['near'] / Z
    hw = cam['width'] or (cam['top'] - cam['bottom']) / 2 * W / H
    return np.c_[(X - cam['center'] + hw) / (2 * hw) * W, (cam['top'] - Y) / (cam['top'] - cam['bottom']) * H]


VIEWS = dict(front=(180.0, 0.0), side=(90.0, 0.0), top=(90.0, -90.0))   # (azimuth, elevation) deg: look along -x, +y, -z
# MuJoCo camera: forward = (cos az cos el, sin az cos el, sin el), image right = forward x up. Front: +y on the right;
# side (from the robot's right): +x on the right.


def render_views(r, extent=0.78, W=960, H=720, lift=0.0):
    """Orthographic renders of the robot standing (Q_STAND) from the front, side and top, with segmentation masks.
    The floor and sky are replaced by white using the masks. Returns {view: (img, robot mask, camera, d)}."""
    m, d = sim.load()
    m.vis.global_.orthographic, m.vis.global_.fovy = 1, extent
    sim.place_on_ground(m, d); d.qpos[2] += lift; mujoco.mj_forward(m, d)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    out = {}
    with mujoco.Renderer(m, H, W) as rd:
        for name, (az, el) in VIEWS.items():
            cam = mujoco.MjvCamera(); cam.lookat[:] = d.subtree_com[1]; cam.distance = 2.0; cam.azimuth, cam.elevation = az, el
            rd.update_scene(d, cam); img = rd.render().copy(); C = mono_camera(rd.scene); C['eye0'] = mono_camera(rd.scene, (0,))
            rd.enable_segmentation_rendering(); rd.update_scene(d, cam); seg = rd.render().copy(); rd.disable_segmentation_rendering()
            mask = (seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM) & (seg[..., 0] >= 0) & (seg[..., 0] != floor)
            img[~mask] = 255
            out[name] = (img, mask, C, seg)
    return out, m, d


def projection_check(m, d, views, W=960, H=720):
    """Project every vertex of every visible mesh geom; compare the bounding box with the rendered silhouette's, per view.
    Also returns, per view, the min and max pixel shift if eye camera 0 alone were used instead of the two-eye average."""
    pts = []
    for g in range(m.ngeom):
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] <= 2:
            md = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[md]:m.mesh_vertadr[md] + m.mesh_vertnum[md]]
            pts.append(d.geom_xpos[g] + v @ d.geom_xmat[g].reshape(3, 3).T)
    pts = np.concatenate(pts); err, eye = {}, {}
    for name, (img, mask, C, seg) in views.items():
        uv = project(C, pts, W, H); ys, xs = np.nonzero(mask)
        # pixel i covers [i, i+1): silhouette edges are xs.min() and xs.max() + 1
        err[name] = mx([uv[:, 0].min() - xs.min(), uv[:, 0].max() - xs.max() - 1, uv[:, 1].min() - ys.min(), uv[:, 1].max() - ys.max() - 1])
        if 'eye0' in C:                         # ablation: eye camera 0 alone instead of the two-eye average
            s = np.linalg.norm(project(C['eye0'], pts, W, H) - uv, axis=1); eye[name] = [float(s.min()), float(s.max())]
    return err, len(pts), eye


# ---- intermediate --------------------------------------------------------------------------------------------------------

def parse_unnormalised():
    """anatomy.parse() with the quaternion normalisation in anatomy._R removed: the ablation behind the README's claim that
    the normalisation matters."""
    from unittest import mock
    with mock.patch.object(A, '_R', lambda s: quat_to_R(A._v(s))):
        return A.parse()


def intermediate(r, m, pm, n):
    d = mujoco.MjData(m); pd = pm.createData(); rng = np.random.default_rng(0); r_raw = parse_unnormalised()
    e = dict(pose=0.0, axis=0.0, vel=0.0, com=0.0, h=0.0, act=0.0, h_raw=0.0)
    hinge = [k for k, j in enumerate(r.joints) if j.type == 'hinge']
    for _ in range(n):
        q, v = random_state(rng, r)
        d.qpos[:], d.qvel[:] = q, v
        u = rng.uniform(-1.5, 1.5, m.nu) * m.actuator_ctrlrange[:, 1]; d.ctrl[:] = u      # a third beyond the range: tests clipping
        mujoco.mj_forward(m, d)
        R, p, an, ax = A.fk(r, q)
        e['pose'] = max(e['pose'], mx(p - d.xpos[1:], R - d.xmat[1:].reshape(-1, 3, 3)))
        e['axis'] = max(e['axis'], mx(an - d.xanchor, ax[hinge] - d.xaxis[hinge]))
        R, p, w, vb = A.velocities(r, q, v)
        for i in range(len(r.bodies)):
            ov = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_XBODY, i + 1, ov, 0)
            e['vel'] = max(e['vel'], mx(ov[:3] - w[i], ov[3:] - vb[i]))
        M, c, l, k = A.centroidal(r, q, v)
        qp, vp = to_pin(q, v); hg = pin.computeCentroidalMomentum(pm, pd, qp, vp)  # type: ignore[reportAttributeAccessIssue]  # pinocchio: API in a compiled extension, no stubs
        e['com'] = max(e['com'], mx(pd.com[0] - c)); e['h'] = max(e['h'], mx(hg.linear - l, hg.angular - k))
        _, _, l, k = A.centroidal(r_raw, q, v); e['h_raw'] = max(e['h_raw'], mx(hg.linear - l, hg.angular - k))
        e['act'] = max(e['act'], mx(A.actuator_force(r, u) - d.qfrc_actuator))
    res = dict(fk_body_pose_vs_mujoco=e['pose'], fk_joint_anchor_axis_vs_mujoco=e['axis'], body_velocity_vs_mujoco=e['vel'],
               com_vs_pinocchio_m=e['com'], centroidal_momentum_vs_pinocchio=e['h'], actuator_force_vs_mujoco=e['act'], samples=n,
               centroidal_momentum_vs_pinocchio_unnormalised_quat=e['h_raw'])
    # armature: the only difference between the mass matrices with and without it is diag(armature) on the hinge dofs
    m0 = mujoco.MjModel.from_xml_path(str(GO2_XML)); m0.dof_armature[:] = 0; d0 = mujoco.MjData(m0)
    arm = np.array([j.armature for j in r.joints if j.type == 'hinge']); worst = 0.0
    Ma, M0 = np.zeros((m.nv, m.nv)), np.zeros((m.nv, m.nv))
    for _ in range(50):
        q, _v = random_state(rng, r); d.qpos[:] = d0.qpos[:] = q
        mujoco.mj_forward(m, d); mujoco.mj_forward(m0, d0); mujoco.mj_fullM(m, d, Ma); mujoco.mj_fullM(m0, d0, M0)
        worst = max(worst, mx(Ma - M0 - np.diag(np.r_[np.zeros(6), arm])))
    res['armature_in_mass_matrix'] = worst
    q = np.r_[0, 0, 0.3, 1, 0, 0, 0, sim.Q_STAND]; d.qpos[:] = d0.qpos[:] = q
    mujoco.mj_forward(m, d); mujoco.mj_forward(m0, d0); mujoco.mj_fullM(m, d, Ma); mujoco.mj_fullM(m0, d0, M0)
    res['stand_M_diag_links'] = np.diag(M0)[6:].tolist(); res['stand_M_diag_with_armature'] = np.diag(Ma)[6:].tolist()
    return res


def torque_budget(r):
    """Stand under joint PD (sim.stand), read the actuator torques, compare with the limits. Then the largest vertical force
    each foot can hold at that pose: tau = J^T f with f = (0, 0, f_z), so f_z,max = min_i tau_max,i / |J_zi| (m04's Jacobian)."""
    m, d = sim.load(); sim.stand(m, d, settle=1.5); ix = sim.index(m)
    tau = d.actuator_force[ix.act].copy(); lim = m.actuator_ctrlrange[ix.act, 1]
    F = sim.foot_forces(m, d)
    fz_max = []
    for li, leg in enumerate(LEGS):
        J = Leg(m, leg).jacobian(d.qpos[ix.legs[leg]['qpos']])
        fz_max.append(float(np.min(lim[3 * li:3 * li + 3] / np.maximum(np.abs(J[2]), 1e-12))))
    M = r.mass
    return dict(stand_torque_Nm=tau.tolist(), torque_limit_Nm=lim.tolist(), stand_torque_utilisation=(np.abs(tau) / lim).tolist(),
                stand_torque_utilisation_max=float(np.max(np.abs(tau) / lim)), stand_foot_fz_N=F[:, 2].tolist(),
                fz_max_per_foot_N=fz_max, static_capacity_total_kg=float(sum(fz_max) / G),
                static_payload_kg=float(sum(fz_max) / G - M), stand_height_m=float(d.qpos[2]))


# ---- advanced ------------------------------------------------------------------------------------------------------------

def contact_wrenches(m, d):
    """Points, forces and torques (world frame) that the environment applies to the robot. A contact force acts on geom2 with
    the sign of the contact frame's normal (geom1 -> geom2) and on geom1 with the opposite sign. Robot-robot contacts are
    internal: they would enter twice with opposite signs, so they are skipped."""
    P, F, T = [], [], []; f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]; b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if (b1 == 0) == (b2 == 0):
            continue
        mujoco.mj_contactForce(m, d, i, f6); Rc = c.frame.reshape(3, 3).T; s = 1.0 if b2 != 0 else -1.0
        P.append(c.pos.copy()); F.append(s * Rc @ f6[:3]); T.append(s * Rc @ f6[3:])
    return P, F, T


def underactuation(r, m, pm, n):
    """B structure; in flight (no contacts with the world) the momentum rate is m_tot g whatever the joint torques; in stance it is
    m_tot g plus the contact wrenches. The constraint solver runs to tolerance 0 here (see README, failure analysis)."""
    B = A.selection(r); res: dict[str, Any] = dict(B_base_rows_max_abs=mx(B[:6]), rank_B=int(np.linalg.matrix_rank(B)))
    res['nullspace_BT_dim'] = int(r.nv - np.linalg.matrix_rank(B))
    d = mujoco.MjData(m); pd = pm.createData(); rng = np.random.default_rng(1)
    worst_act = worst_fd = worst_pin = 0.0; worst_fd_default = 0.0; base_acc, com_acc = [], []
    g = m.opt.gravity.copy(); eom = {True: np.zeros(2), False: np.zeros(2)}     # (base rows, joint rows)
    for tol, iters in ((1e-8, 100), (0.0, 1000)):
        m.opt.tolerance, m.opt.iterations = tol, iters
        rng = np.random.default_rng(1)
        for _ in range(n):
            q, v = random_state(rng, r); d.qpos[:], d.qvel[:] = q, v
            d.ctrl[:] = rng.uniform(-1, 1, m.nu) * m.actuator_ctrlrange[:, 1]; mujoco.mj_forward(m, d)
            M, c, l, k = A.centroidal(r, q, v); ld, kd = A.momentum_rate(r, q, v, d.qacc)
            err = mx(ld - M * g, kd)
            # residual of M qacc = qfrc_smooth + qfrc_constraint: how far the solver's qacc is from the equations of motion
            Ma = np.zeros(m.nv); mujoco.mj_mulM(m, d, Ma, d.qacc)
            rr = Ma - d.qfrc_smooth - d.qfrc_constraint
            eom[tol > 0] = np.maximum(eom[tol > 0], [mx(rr[:6]), mx(rr[6:])])
            if tol > 0:
                worst_fd_default = max(worst_fd_default, err); continue
            worst_fd = max(worst_fd, err); worst_act = max(worst_act, mx(d.qfrc_actuator[:6]))
            qp, vp, ap = to_pin(q, v, d.qacc.copy()); pin.computeCentroidalMomentumTimeVariation(pm, pd, qp, vp, ap)  # type: ignore[reportAttributeAccessIssue]  # pinocchio: API in a compiled extension, no stubs
            worst_pin = max(worst_pin, mx(pd.dhg.linear - M * g, pd.dhg.angular))
            base_acc.append(np.linalg.norm(d.qacc[3:6])); com_acc.append(np.linalg.norm(ld / M - g))
    m.opt.tolerance, m.opt.iterations = 1e-8, 100
    # the same random states with the joint torques set to zero: how much the torques move the base (and not the COM)
    rng = np.random.default_rng(1); dacc = []
    for _ in range(n):
        q, v = random_state(rng, r); d.qpos[:], d.qvel[:] = q, v
        u = rng.uniform(-1, 1, m.nu) * m.actuator_ctrlrange[:, 1]
        d.ctrl[:] = u; mujoco.mj_forward(m, d); a1 = d.qacc[:6].copy()
        d.ctrl[:] = 0; mujoco.mj_forward(m, d); dacc.append(np.linalg.norm(a1[3:6] - d.qacc[3:6]))
    res.update(qfrc_actuator_base_rows_max_abs=worst_act, flight_momentum_rate_vs_Mg_fd=worst_fd,
               flight_momentum_rate_vs_Mg_pinocchio=worst_pin, flight_momentum_rate_default_solver_tol=worst_fd_default,
               flight_eom_residual_default_solver_tol_base_rows=float(eom[True][0]),
               flight_eom_residual_default_solver_tol_joint_rows=float(eom[True][1]),
               flight_eom_residual_tol0_base_rows=float(eom[False][0]), flight_eom_residual_tol0_joint_rows=float(eom[False][1]),
               flight_torque_effect_on_base_ang_acc_median=float(np.median(dacc)),
               flight_com_acc_minus_g_max=float(np.max(com_acc)), samples=n)
    return res


def stance_balance():
    """Squat from the stand pose (min-jerk PD to a lower pose and back) on the floor. At every 10 ms compare the momentum rate
    of the robot (finite difference of centroidal() along the motion) with m_tot g + the contact wrenches from MuJoCo."""
    m, d = sim.load(); m.opt.tolerance, m.opt.iterations = 0.0, 1000
    sim.stand(m, d, settle=0.5); r = A.parse(); g = m.opt.gravity.copy()
    q0 = sim.Q_STAND.copy(); q1 = np.tile([0.0, 1.25, -2.5], 4)
    down, up = sim.min_jerk(q0, q1, 0.8), sim.min_jerk(q1, q0, 0.8); t0 = d.time
    ctrl = sim.joint_pd(m, lambda t: down(t - t0) if t - t0 < 0.9 else up(t - t0 - 0.9), kp=80, kd=2)
    rows = []

    def sample(m, d):
        if round((d.time - t0) / m.opt.timestep) % 5:
            return None
        mujoco.mj_forward(m, d)
        M, c, l, k = A.centroidal(r, d.qpos, d.qvel); ld, kd = A.momentum_rate(r, d.qpos, d.qvel, d.qacc)
        P, F, T = contact_wrenches(m, d); le, ke = A.external_rate(M, c, g, P, F, T); lg, kg = A.external_rate(M, c, g)
        return [d.time - t0, *ld, *kd, *le, *ke, *lg, *kg, c[2]]
    X = np.array(sim.run(m, d, ctrl, 1.8, on_step=sample))
    err = np.abs(X[:, 1:7] - X[:, 7:13]).max(); err_nocontact = np.abs(X[:, 1:7] - X[:, 13:19]).max(1)
    return dict(stance_momentum_rate_vs_wrench=float(err), stance_samples=len(X),
                stance_error_without_contacts_min_N=float(err_nocontact.min()),
                stance_fz_range_N=[float((X[:, 9] - X[:, 15]).min()), float((X[:, 9] - X[:, 15]).max())]), X


def flight(integrator, dt=0.002, T=0.9, rec=None):
    """Launch the robot (v = (1.2, 0, 4.5) m/s from 0.6 m height) and drive the 12 joints with sinusoidal torques (15 N m, 0.5-1.7 Hz)
    for T seconds; no contact with the floor in that time. Logs centroidal momentum and COM, base attitude."""
    m, d = sim.load(); m.opt.integrator, m.opt.timestep = integrator, dt; r = A.parse()
    ix = sim.index(m); d.qpos[:3] = (0, 0, 0.6); d.qpos[3:7] = (1, 0, 0, 0); d.qpos[ix.qpos] = sim.Q_STAND; d.qvel[0], d.qvel[2] = 1.2, 4.5
    mujoco.mj_forward(m, d); M, c0, l0, k0 = A.centroidal(r, d.qpos, d.qvel); g = m.opt.gravity.copy()
    freq = np.array([1.3, 0.7, 1.1, 0.9, 1.7, 0.5, 1.2, 0.8, 1.5, 1.0, 0.6, 1.4]); ph = np.arange(12.0)
    ctrl = lambda m, d: 15 * np.sin(2 * np.pi * freq * d.time + ph)  # noqa: E731
    floor_contacts = [0]

    def log(m, d):
        if rec is not None:
            rec(m, d)
        floor_contacts[0] += sum((m.geom_bodyid[d.contact[i].geom1] == 0) != (m.geom_bodyid[d.contact[i].geom2] == 0) for i in range(d.ncon))
        _, c, l, k = A.centroidal(r, d.qpos, d.qvel)
        return [d.time, *c, *(l - l0 - M * g * d.time), *(k - k0), *R_to_rpy(sim.base_pose(d)[2]), *d.qpos[:3]]
    X = np.array(sim.run(m, d, ctrl, T, on_step=log))
    c_ballistic = c0[None] + (l0 / M)[None] * X[:, :1] + 0.5 * g[None] * X[:, :1] ** 2
    return dict(X=X, c_ballistic=c_ballistic, floor_contacts=floor_contacts[0], M=M)


def drift(T=0.9):
    out = {}; runs = {}
    for name, integ in (('euler', mujoco.mjtIntegrator.mjINT_EULER), ('rk4', mujoco.mjtIntegrator.mjINT_RK4)):
        for dt in (0.002, 0.001):
            F = flight(integ, dt, T=T); X = F['X']; runs[(name, dt)] = F
            out[f'{name}_{dt * 1e3:g}ms_lin'] = float(np.abs(X[:, 4:7]).max()); out[f'{name}_{dt * 1e3:g}ms_ang'] = float(np.abs(X[:, 7:10]).max())
            out[f'{name}_{dt * 1e3:g}ms_floor_contacts'] = F['floor_contacts']
    out['flight_drift_order_euler'] = out['euler_2ms_ang'] / out['euler_1ms_ang']
    out['flight_drift_order_rk4'] = out['rk4_2ms_ang'] / out['rk4_1ms_ang']
    out['flight_drift_rk4_ang_kgm2s'] = out['rk4_2ms_ang']
    return out, runs


def loop(f=1.0, amp=0.45, sign=1.0, region=1.0):
    """Joint-space loop on every leg: thigh 0.9 + s r a sin(wt), knee -1.8 + r a (cos(wt) - 1): a circle of radius a through
    the stand pose, area pi a^2 in the (thigh, knee) plane; s = -1 runs the same circle backwards. r = -1 is the point
    reflection of the circle through the stand pose: same area, same direction, but over the region above it (knee bent less,
    knee in [-1.8, -0.9] rad). a = 0.45 keeps the knee inside [-2.7227, -0.83776] rad for both regions (-1.8 - 2a = -2.70)."""
    w = 2 * np.pi * f
    return lambda t: sim.Q_STAND + np.tile([0.0, sign * region * amp * np.sin(w * t), region * amp * (np.cos(w * t) - 1)], 4)


def reorient(f=1.0, sign=1.0, cycles=5, rec=None, kp=400.0, kd=6.0, region=1.0):
    """Zero gravity, robot at rest 1 m above the floor, joint PD (kp, kd) tracking loop(); RK4. Logs pitch, |k|, |l| and the
    FL thigh and knee angles every 10 ms."""
    m, d = sim.load(); m.opt.gravity[:] = 0; m.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4; r = A.parse()
    ix = sim.index(m); d.qpos[:3] = (0, 0, 1.0); d.qpos[3:7] = (1, 0, 0, 0); d.qpos[ix.qpos] = sim.Q_STAND; mujoco.mj_forward(m, d)
    qd = loop(f, sign=sign, region=region)

    def log(m, d):
        if rec is not None:
            rec(m, d)
        if round(d.time / m.opt.timestep) % 5:
            return None
        _, c, l, k = A.centroidal(r, d.qpos, d.qvel)
        return [d.time, *R_to_rpy(sim.base_pose(d)[2]), np.linalg.norm(k), np.linalg.norm(l), *d.qpos[ix.qpos[1:3]]]
    X = np.array(sim.run(m, d, sim.joint_pd(m, qd, kp, kd), cycles / f, on_step=log))
    return X


# ---- figures -------------------------------------------------------------------------------------------------------------

# Joint-role colours, used for nothing else. Kept apart from the semantic colours in fs.C (estimate blue, second estimate
# violet, plan amber, pass green, fail red), which mean something else in MODULE_SPEC.md. Vector components (x/y/z,
# roll/pitch/yaw) are told apart by line style in ink, so no colour carries two meanings.
JCOL = dict(hip='#159a9c', thigh='#c2378f', calf='#7b5b3a')                # teal, magenta, brown
XYZ_LS = ('-', '--', ':')                                                   # x / roll, y / pitch, z / yaw
ROLE = dict(hip='hip abduction (x)', thigh='hip flexion (y)', calf='knee (y)')


def fig_tree(r):
    fig, ax = fs.diagram(10.2, 5.6, (0, 10.2), (0, 5.6))
    fs.box(ax, (0.1, 2.5), 1.0, 0.6, 'world', fontsize=9)
    base = r.bodies[0]
    fs.box(ax, (2.1, 2.2), 1.5, 1.2, f'base\n{base.mass:.3f} kg\nIMU site', fontsize=8.5)
    fs.arrow(ax, (1.1, 2.8), (2.1, 2.8), 'free joint', color=fs.C['ink'], fontsize=8)
    ax.text(1.6, 2.6, '6 DoF, nq = 7', ha='center', va='top', fontsize=7.5, color=fs.C['muted'])
    ys = dict(FL=4.6, FR=3.35, RL=2.1, RR=0.85)
    for leg, y in ys.items():
        x = 4.6; prev = (3.6, 2.8)
        for part in ('hip', 'thigh', 'calf'):
            bi = r.body(f'{leg}_{part}'); b = r.bodies[bi]; j = r.joints[b.joints[0]]
            col = JCOL[part]
            fs.arrow(ax, prev, (x, y + 0.25), color=col, lw=1.1)
            ax.text((prev[0] + x) / 2, (prev[1] + y + 0.25) / 2 + 0.08, f'{"xyz"[int(np.argmax(np.abs(j.axis)))]}', color=col,
                    fontsize=8, ha='center', va='bottom', fontweight='semibold')
            fs.box(ax, (x, y), 1.45, 0.5, f'{leg}_{part}  {b.mass:.3f} kg', fontsize=7.5, color=col)
            lo, hi = np.degrees(j.range)
            ax.text(x + 0.72, y - 0.04, f'[{lo:.0f}°, {hi:.0f}°]', ha='center', va='top', fontsize=6.8, color=fs.C['muted'])
            prev = (x + 1.45, y + 0.25); x += 1.85
        fs.arrow(ax, prev, (x - 0.1, y + 0.25), color=fs.C['ink'], lw=0.8, style='-')
        ax.plot([x], [y + 0.25], 'o', color=fs.C['ink'], ms=6); ax.text(x + 0.12, y + 0.25, f'foot {leg}\nsphere r 22 mm',
                                                                         va='center', fontsize=7)
    for part, col in JCOL.items():
        ax.plot([], [], color=col, lw=2, label=f'{ROLE[part]} hinge')
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, 0.99), fontsize=7.5, handlelength=1.5)
    dm = A.dof(r)
    ax.text(0.1, 0.35, f'{dm["bodies"]} bodies, {dm["free_joints"]} free joint + {dm["hinges"]} hinges: nq = {dm["nq"]}, '
            f'nv = {dm["nv"]}, actuators nu = {dm["nu"]}, unactuated directions = {dm["unactuated"]}. Total mass {r.mass:.4f} kg.',
            fontsize=8, color=fs.C['ink'])
    ax.set_title('Kinematic tree of the Go2, generated from go2.xml by anatomy.parse()')
    return fs.save(fig, FIG / 'kinematic_tree')


def _annotate(ax, name, img, C, r, d, feet, W, H):
    ax.imshow(img); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis('off')
    R, p, an, axes = A.fk(r, d.qpos)
    L = 0.07
    for k, j in enumerate(r.joints):
        if j.type != 'hinge':
            continue
        role = j.name.split('_')[1]
        a, b = project(C, np.array([an[k] - L * axes[k], an[k] + L * axes[k]]), W, H)
        if np.hypot(*(b - a)) < 4:                                        # axis points at the camera: draw a ring
            ax.plot(*a, 'o', ms=9, mfc='none', mec=JCOL[role], mew=1.8)
        else:
            ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='-|>', color=JCOL[role], lw=1.8, mutation_scale=11))
        ax.plot(*project(C, an[k], W, H)[0], 'o', ms=3.5, color=JCOL[role])
    M, c, _, _ = A.centroidal(r, d.qpos)
    u = project(C, c, W, H)[0]
    ax.plot(*u, marker='o', ms=9, mfc='white', mec=fs.C['ink'], mew=1.4); ax.plot(*u, marker='+', ms=9, color=fs.C['ink'], mew=1.4)
    fs.label(ax, u, 'centre of mass', xytext={'front': (90, 25), 'side': (30, 45), 'top': (-15, -120)}[name], fontsize=8,
             ha='right' if name == 'top' else 'left')
    # base frame glyph
    o = p[0]; Rb = R[0]
    for i, nm in enumerate('xyz'):
        a, b = project(C, np.array([o, o + 0.1 * Rb[:, i]]), W, H)
        if np.hypot(*(b - a)) > 4:
            ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='-|>', color=fs.C['ink'], lw=1.0, mutation_scale=8))
            ax.text(*b, f' {nm}', fontsize=8, color=fs.C['ink'])
    # scale bar, 0.1 m (orthographic: one scale over the whole image)
    s = W / (2 * (C['width'] or (C['top'] - C['bottom']) / 2 * W / H)) * 0.1
    ax.plot([30, 30 + s], [H - 30, H - 30], color=fs.C['ink'], lw=2); ax.text(30 + s / 2, H - 40, '0.1 m', ha='center', fontsize=8)
    if name == 'side':
        hipF, hipR = an[r.joint('FR_thigh_joint')], an[r.joint('RR_thigh_joint')]
        a, b = project(C, np.array([hipR, hipF]), W, H); yb = min(a[1], b[1]) - 150
        ax.annotate('', xy=(b[0], yb), xytext=(a[0], yb), arrowprops=dict(arrowstyle='<->', color=fs.C['muted'], lw=0.8))
        ax.text((a[0] + b[0]) / 2, yb - 8, f'hip spacing {np.linalg.norm(hipF - hipR):.4f} m', ha='center', fontsize=8, color=fs.C['muted'])
        knee = an[r.joint('FR_calf_joint')]; foot = d.geom_xpos[feet[1]]
        for P0, P1, txt, off in ((hipF, knee, 'thigh 0.213 m', (75, 95)), (knee, foot, 'calf 0.213 m to foot centre', (35, 0))):
            mid = project(C, (P0 + P1) / 2, W, H)[0]; fs.label(ax, mid, txt, xytext=off, fontsize=8)
        fs.label(ax, project(C, d.site_xpos[0], W, H)[0], 'IMU site', xytext=(-10, -45), fontsize=8, ha='right')
    if name == 'top':
        for leg in LEGS:
            u = project(C, an[r.joint(f'{leg}_hip_joint')], W, H)[0]
            ax.text(u[0], u[1] + (40 if leg[1] == 'R' else -40), leg, ha='center', va='center', fontsize=9, fontweight='semibold')
        a, b = project(C, np.array([an[r.joint('FL_hip_joint')], an[r.joint('FR_hip_joint')]]), W, H)
        fs.label(ax, (a + b) / 2, f'hip axes {np.linalg.norm(an[r.joint("FL_hip_joint")] - an[r.joint("FR_hip_joint")]):.3f} m apart',
                 xytext=(60, 0), fontsize=8)
    if name == 'front':
        f = d.geom_xpos[feet]
        fs.label(ax, project(C, f[0], W, H)[0], f'foot track {abs(f[0, 1] - f[1, 1]):.3f} m', xytext=(40, -20), fontsize=8)
    ax.set_title({'front': 'front (looking along -x, +y to the right)', 'side': 'right side (looking along +y, +x to the right)',
                  'top': 'top (looking down)'}[name])


def fig_views(r, views, mv, d, W=960, H=720):
    feet = sim.index(mv).feet
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.4))
    for ax, (name, (img, mask, C, seg)) in zip(axs, views.items()):
        _annotate(ax, name, img, C, r, d, feet, W, H)
    handles = [plt.Line2D([], [], color=c, lw=2, label=ROLE[k] + ' axis') for k, c in JCOL.items()]
    axs[0].legend(handles=handles, loc='upper left', fontsize=7.5)
    return fs.save(fig, FIG / 'views_annotated')


def fig_joint_ranges(r):
    """Each joint's range drawn in its plane of motion: the link direction at the lower limit, upper limit and stand pose."""
    fig, axs = plt.subplots(1, 4, figsize=(11, 3.3))
    cases = [('FL_hip_joint', 'hip abduction, FL\n(front view, +y to the right)', sim.Q_STAND[0]),
             ('FL_thigh_joint', 'hip flexion, front legs\n(side view, +x to the right)', sim.Q_STAND[1]),
             ('RL_thigh_joint', 'hip flexion, rear legs\n(side view, +x to the right)', sim.Q_STAND[1]),
             ('FL_calf_joint', 'knee, all legs\n(thigh drawn vertical)', sim.Q_STAND[2])]
    for ax, (jn, title, q_st) in zip(axs, cases):
        j = r.joints[r.joint(jn)]; lo, hi = j.range; role = jn.split('_')[1]
        th = np.linspace(lo, hi, 100)
        # link direction (0, 0, -1) at zero angle. Hip: Rx(t) gives (0, sin t, -cos t), drawn with +y to the right.
        # Thigh and knee: Ry(t) gives (-sin t, 0, -cos t), drawn with +x to the right: a positive angle swings the foot back.
        if role == 'hip':
            dirs = lambda t: np.c_[np.sin(t), -np.cos(t)]  # noqa: E731
        else:
            dirs = lambda t: np.c_[-np.sin(t), -np.cos(t)]  # noqa: E731
        P = dirs(th)
        ax.fill(np.r_[0, P[:, 0], 0], np.r_[0, P[:, 1], 0], color=JCOL[role], alpha=0.15, lw=0)
        ax.plot(P[:, 0], P[:, 1], color=JCOL[role], lw=1.5)
        for t, txt, ls in ((lo, f'lower {np.degrees(lo):.0f}°', '-'), (hi, f'upper {np.degrees(hi):.0f}°', '-'),
                           (q_st, f'stand {np.degrees(q_st):.0f}°', '--')):
            e = dirs(np.array([t]))[0]
            ax.plot([0, e[0]], [0, e[1]], ls, color=fs.C['plan'] if ls == '--' else JCOL[role], lw=1.6 if ls == '--' else 1.0)
            ax.text(1.12 * e[0], 1.12 * e[1], txt, ha='center', va='center', fontsize=7.5)
        ax.plot([0, 0], [0, -1], ':', color=fs.C['gt'], lw=0.8); ax.plot(0, 0, 'o', color=fs.C['ink'], ms=4)
        ax.set_aspect('equal'); ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.45, 1.45); ax.axis('off')
        ax.set_title(title, fontsize=9)
    fig.suptitle('Joint ranges from go2.xml (dotted: zero angle, link straight down; dashed: the stand pose)', fontsize=10)
    return fs.save(fig, FIG / 'joint_ranges')


def fig_torque_budget(tb, inter):
    lab = [f'{l} {p}' for l in LEGS for p in ('hip', 'thigh', 'knee')]
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.6), gridspec_kw=dict(width_ratios=[1.6, 1]))
    ax = axs[0]; x = np.arange(12)
    cols = [JCOL[('hip', 'thigh', 'calf')[i % 3]] for i in range(12)]
    ax.bar(x, np.array(tb['stand_torque_utilisation']) * 100, color=cols)
    for i, (t, L) in enumerate(zip(tb['stand_torque_Nm'], tb['torque_limit_Nm'])):
        ax.text(i, abs(t) / L * 100 + 0.5, f'{abs(t):.1f}', ha='center', fontsize=6.5, color=fs.C['muted'])
    ax.set_xticks(x, lab, rotation=60, fontsize=7.5); ax.set(ylabel='|torque| / limit [%]',
                                                            title='Torque needed to stand (labels: N m) relative to the motor limit')
    ax2 = axs[1]
    lk = np.array(inter['stand_M_diag_links']); la = np.array(inter['stand_M_diag_with_armature'])
    ax2.bar(x - 0.2, lk, 0.4, color=fs.C['muted'], label='links only'); ax2.bar(x + 0.2, la, 0.4, color=fs.C['ink'], label='links + armature 0.01')
    ax2.set_yscale('log'); ax2.set_ylim(top=0.09); ax2.set_xticks(x, lab, rotation=60, fontsize=7.5)   # headroom for the legend
    ax2.set(ylabel=r'$M_{ii}$ [kg m$^2$]', title='Joint-space inertia at the stand pose'); ax2.legend(fontsize=7.5)
    return fs.save(fig, FIG / 'torque_budget_and_inertia')


def fig_B_and_M(r):
    m, d = sim.load(scene=False); d.qpos[:] = np.r_[0, 0, 0.3, 1, 0, 0, 0, sim.Q_STAND]; mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M); B = A.selection(r)
    fig, axs = plt.subplots(1, 2, figsize=(9.5, 4.2), gridspec_kw=dict(width_ratios=[0.8, 1.2]))
    ax = axs[0]; ax.imshow(B, cmap='Greys', vmin=0, vmax=1.5, aspect='equal'); ax.grid(False)
    ax.axhline(5.5, color=fs.C['muted'], lw=1.2); ax.set_yticks(range(18), ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'] + [f'{l}{p}' for l in LEGS for p in 'htk'], fontsize=7)
    ax.set_xticks(range(12), [f'{l}{p}' for l in LEGS for p in 'htk'], rotation=90, fontsize=7)
    ax.text(5.5, 2.5, 'six zero rows:\nno actuator acts\non the base', ha='center', va='center', color=fs.C['ink'], fontsize=8)
    ax.set_title(r'$B$ (18 x 12) in $\tau = B u$')
    ax = axs[1]; im = ax.imshow(np.log10(np.abs(M) + 1e-6), cmap='Greys', vmin=-6, vmax=1.5); ax.grid(False)
    ax.axhline(5.5, color=fs.C['muted'], lw=1.0); ax.axvline(5.5, color=fs.C['muted'], lw=1.0)
    ax.set_xticks([2.5, 11.5], ['base (6)', 'joints (12)']); ax.set_yticks([2.5, 11.5], ['base', 'joints'])
    for (i, j), t in {(2.5, 2.5): r'$M_{bb}$', (2.5, 11.5): r'$M_{bq}$', (11.5, 2.5): r'$M_{qb}$', (11.5, 11.5): r'$M_{qq}$'}.items():
        ax.text(j, i, t, ha='center', va='center', fontsize=12, color=fs.C['ink'], bbox=dict(fc=fs.C['paper'], ec='none', alpha=0.85, pad=1.5))
    fig.colorbar(im, ax=ax, label=r'$\log_{10}|M_{ij}|$'); ax.set_title('mass matrix at the stand pose (MuJoCo mj_fullM)')
    return fs.save(fig, FIG / 'selection_and_mass_matrix')


def fig_underactuation_diagram():
    fig, ax = fs.diagram(10, 4.0, (0, 10), (0, 4))
    fs.box(ax, (0.1, 2.4), 1.6, 0.8, 'motor torques\nu (12)', fontsize=8.5)
    fs.box(ax, (2.6, 2.4), 1.3, 0.8, r'$B$' + '\n(18 x 12)', fontsize=8.5)
    fs.box(ax, (0.1, 0.5), 1.6, 0.8, 'contact wrenches\n' + r'$\lambda$ (6 per foot)', fontsize=8.5)
    fs.box(ax, (2.6, 0.5), 1.3, 0.8, r'$J_\lambda^T$' + '\n' + r'(18 x 6$n_f$)', fontsize=8.5)
    fs.box(ax, (5.0, 2.65), 2.0, 0.55, 'joint rows (12)', fontsize=8.5)
    fs.box(ax, (5.0, 1.65), 2.0, 0.55, 'base rows (6)', fontsize=8.5, color=fs.C['muted'])
    fs.arrow(ax, (1.7, 2.8), (2.6, 2.8)); fs.arrow(ax, (1.7, 0.9), (2.6, 0.9))
    fs.arrow(ax, (3.9, 2.9), (5.0, 2.9), 'u (gear 1)', fontsize=8); fs.arrow(ax, (3.9, 2.6), (5.0, 1.95), '0', color=fs.C['muted'], fontsize=8)
    fs.arrow(ax, (3.9, 0.9), (5.0, 1.75), color=fs.C['ink']); fs.arrow(ax, (3.9, 1.1), (5.0, 2.7), color=fs.C['ink'])
    fs.box(ax, (7.7, 1.65), 2.2, 0.55, 'centroidal momentum', fontsize=8.5, color=fs.C['muted'])
    fs.arrow(ax, (7.0, 1.93), (7.7, 1.93))
    ax.text(8.8, 1.5, r'$\dot l = m_{\mathrm{tot}}\, g + \sum_j f_j$', ha='center', va='top', fontsize=9)
    ax.text(8.8, 1.12, r'$\dot k = \sum_j \left[(r_j - c) \times f_j + \mu_j\right]$', ha='center', va='top', fontsize=9)
    ax.text(8.8, 0.68, 'no u in these equations', ha='center', va='top', fontsize=8.5, color=fs.C['ink'])
    fs.box(ax, (7.7, 2.65), 2.2, 0.55, 'joint accelerations', fontsize=8.5)
    fs.arrow(ax, (7.0, 2.93), (7.7, 2.93))
    ax.text(5.0, 3.75, r'$M(q)\dot v + C(q, v) = B u + J_\lambda^T \lambda$,   $v = (v_{\mathcal{B}}, \omega_{\mathcal{B}}, \dot{\mathbf{q}}) \in \mathbb{R}^{18}$', fontsize=10)
    ax.text(5.0, 0.55, 'gravity enters through C; joint damping,\nfriction loss and limits are internal', fontsize=8, color=fs.C['muted'])
    ax.set_title('Why the floating base is underactuated: actuators reach only the joint rows')
    return fs.save(fig, FIG / 'underactuation_diagram')


def fig_flight(F):
    X = F['X']; t = X[:, 0]; cb = F['c_ballistic']
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.5))
    ax = axs[0]; ax.plot(t, cb[:, 2], color=fs.C['plan'], lw=3, alpha=0.5, label='ballistic parabola of the COM')
    ax.plot(t, X[:, 3], color=fs.C['ink'], label='COM, simulated'); ax.plot(t, X[:, 15], '--', color=fs.C['muted'], label='base origin, simulated')
    ax.set(xlabel='time [s]', ylabel='height [m]', title='Height in flight with flailing legs'); ax.legend(fontsize=7.5)
    ax = axs[1]
    for i, n in enumerate(('roll', 'pitch', 'yaw')):
        ax.plot(t, np.degrees(X[:, 10 + i]), XYZ_LS[i], color=fs.C['ink'], label=n)
    ax.set(xlabel='time [s]', ylabel='base attitude [deg]', title='The base rotates under joint torques'); ax.legend(fontsize=7.5)
    ax = axs[2]
    # linear axis in units of 1e-6: on a log axis this range (about 1e-7 to 5e-6) gets a single labelled decade
    ax.plot(t, 1e6 * np.linalg.norm(X[:, 4:7], axis=1), color=fs.C['ink'], label=r'$|l - l_0 - m_{\mathrm{tot}}\, g t|$ [kg m/s]')
    ax.plot(t, 1e6 * np.linalg.norm(X[:, 7:10], axis=1), '--', color=fs.C['muted'], label=r'$|k - k_0|$ [kg m$^2$/s]')
    ax.set(xlabel='time [s]', ylabel=r'[$10^{-6}$ kg m/s or kg m$^2$/s]', title='Momentum change not caused by gravity (RK4, 2 ms)')
    ax.set_ylim(bottom=0); ax.legend(fontsize=7.5)
    return fs.save(fig, FIG / 'flight_momentum')


def fig_stance(X):
    t = X[:, 0]
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.5))
    ax = axs[0]; ax.plot(t, X[:, 9] - X[:, 15], color=fs.C['gt'], lw=3, alpha=0.6, label=r'$\sum f_z$ from MuJoCo contacts')
    ax.plot(t, X[:, 3] - X[:, 15], '--', color=fs.C['est'], label=r'$\dot l_z - m_{\mathrm{tot}}\, g_z$ from the motion')
    ax.set(xlabel='time [s]', ylabel='[N]', title='Vertical: contacts carry weight + acceleration'); ax.legend(fontsize=7.5)
    ax = axs[1]
    for i, n in enumerate('xyz'):
        ax.plot(t, X[:, 10 + i], XYZ_LS[i], color=fs.C['gt'], lw=3, alpha=0.6)
        ax.plot(t, X[:, 4 + i], XYZ_LS[i], color=fs.C['est'], lw=1, label=rf'${n}$')
    ax.set(xlabel='time [s]', ylabel='[N m]', title=r'Contact moment about COM (grey), $\dot k$ (blue)')
    ax.legend(fontsize=7.5)
    ax = axs[2]
    ax.semilogy(t, np.abs(X[:, 1:7] - X[:, 7:13]).max(1), color=fs.C['good'], label='with contact wrenches')
    ax.semilogy(t, np.abs(X[:, 1:7] - X[:, 13:19]).max(1), color=fs.C['bad'], label='gravity only')
    ax.set(xlabel='time [s]', ylabel='max error [N or N m]', title='Momentum balance residual during a squat'); ax.legend(fontsize=7.5)
    return fs.save(fig, FIG / 'stance_momentum_balance')


def fig_drift(runs, D):
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
    for (name, dt), F in runs.items():
        X = F['X']; col = fs.C['bad'] if name == 'euler' else fs.C['good']; ls = '-' if dt == 0.002 else '--'
        axs[0].semilogy(X[:, 0], np.linalg.norm(X[:, 7:10], axis=1) + 1e-16, ls, color=col, label=f'{name}, {dt * 1e3:g} ms')
        axs[1].semilogy(X[:, 0], np.linalg.norm(X[:, 4:7], axis=1) + 1e-16, ls, color=col, label=f'{name}, {dt * 1e3:g} ms')
    axs[0].set(xlabel='time [s]', ylabel=r'$|k(t) - k(0)|$ [kg m$^2$/s]', title='Angular momentum drift in flight (should be 0)')
    axs[1].set(xlabel='time [s]', ylabel=r'$|l - l_0 - m_{\mathrm{tot}}\, g t|$ [kg m/s]', title='Linear momentum drift in flight (should be 0)')
    axs[0].legend(fontsize=7.5)
    return fs.save(fig, FIG / 'integrator_drift')


def fig_reorient(Xf, Xr, Xs, Xo):
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.8), gridspec_kw=dict(width_ratios=[1.4, 1, 1]))
    ax = axs[0]
    ax.plot(Xf[:, 0], np.degrees(Xf[:, 2]), color=fs.C['ink'], label='loop forward, 1 Hz')
    ax.plot(Xr[:, 0], np.degrees(Xr[:, 2]), color=fs.C['muted'], lw=2.0, label='loop reversed, 1 Hz')
    ax.plot(Xs[:, 0] / 2, np.degrees(Xs[:, 2]), '--', color=fs.C['ink'], label='forward, 0.5 Hz (time / 2)')
    ax.plot(Xo[:, 0], np.degrees(Xo[:, 2]), '-.', color=fs.C['muted'], lw=1.0, label='other region, same area, 1 Hz')
    ax.set(xlabel='time [s] (cycles at 1 Hz)', ylabel='base pitch [deg]', title='Pitch changes with zero angular momentum')
    ax.legend(fontsize=7.5)
    ax = axs[1]; ax.semilogy(Xf[:, 0], Xf[:, 4] + 1e-16, color=fs.C['ink'], label=r'$|k|$ [kg m$^2$/s]')
    ax.semilogy(Xf[:, 0], Xf[:, 5] + 1e-16, color=fs.C['muted'], label=r'$|l|$ [kg m/s]')
    ax.set(xlabel='time [s]', title='Momentum stays at zero'); ax.legend(fontsize=7.5)
    ax = axs[2]
    ax.plot(Xf[:, 6], Xf[:, 7], color=fs.C['ink'], lw=0.8, label='tracked')
    cmd = np.array([loop()(t)[1:3] for t in np.linspace(0, 1, 201)])     # the command the experiment used (amp 0.45)
    ax.plot(*cmd.T, '--', color=fs.C['plan'], label='commanded')
    ax.plot(Xo[:, 6], Xo[:, 7], '-.', color=fs.C['muted'], lw=0.8, label='other region, tracked')
    ax.plot(*np.array([loop(region=-1.0)(t)[1:3] for t in np.linspace(0, 1, 201)]).T, '--', color=fs.C['plan'])
    ax.set(xlabel='thigh [rad]', ylabel='knee [rad]', title='Joint-space loops (FL)'); ax.set_aspect('equal')
    ax.legend(fontsize=7, loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0)   # beside the loops, not over them
    return fs.save(fig, FIG / 'zero_momentum_reorientation')


def fig_reorient_failure(Xf, Xh, Lf, Lh):
    """The rotation depends on the loop the joints actually trace. Low PD gains shrink the traced loop more at 1 Hz than at
    0.5 Hz, so the rotation per cycle depends on speed."""
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6))
    ax = axs[0]; cmd = np.array([loop()(t)[1:3] for t in np.linspace(0, 1, 201)])
    ax.plot(*cmd.T, '--', color=fs.C['plan'], label='commanded')
    for X, lab, col, ls in ((Xf, 'kp 400, 1 Hz', fs.C['good'], '-'), (Xh, 'kp 400, 0.5 Hz', fs.C['good'], ':'),
                            (Lf, 'kp 60, 1 Hz', fs.C['bad'], '-'), (Lh, 'kp 60, 0.5 Hz', fs.C['bad'], ':')):
        n = len(X) // 5; ax.plot(X[-n:, 6], X[-n:, 7], ls, color=col, lw=1, label=lab)
    ax.set(xlabel='thigh [rad]', ylabel='knee [rad]', title='Traced loop, last cycle (FL)'); ax.set_aspect('equal'); ax.legend(fontsize=7)
    ax = axs[1]
    for X, Y, lab, col in ((Xf, Xh, 'kp 400', fs.C['good']), (Lf, Lh, 'kp 60', fs.C['bad'])):
        ax.plot(X[:, 0], np.degrees(X[:, 2]), color=col, label=f'{lab}, 1 Hz'); ax.plot(Y[:, 0] / 2, np.degrees(Y[:, 2]), ':', color=col, label=f'{lab}, 0.5 Hz (time / 2)')
    ax.set(xlabel='cycles', ylabel='base pitch [deg]', title='Same command, different speed'); ax.legend(fontsize=7)
    return fs.save(fig, FIG / 'reorientation_tracking_failure')


# ---- videos --------------------------------------------------------------------------------------------------------------

class Rec(sim.Recorder):
    """sim.Recorder plus: slow motion (`slow` sim seconds per video second, < 1 is slow motion) and extra marker spheres drawn
    into the scene from marks(d) -> [(pos, rgba, radius)]. Same encoder settings and limits as the parent."""

    def __init__(self, m, mp4, slow=1.0, marks=None, **kw):
        super().__init__(m, mp4, **kw); self.slow, self.marks = slow, marks

    def render(self, d):
        if not self.ok:
            return None
        assert self.renderer is not None                       # self.ok is renderer is not None
        self.renderer.update_scene(d, camera=self.cam, scene_option=self.opt)
        scn = self.renderer.scene
        for pos, rgba, rad in (self.marks(d) if self.marks else []):
            if scn.ngeom < scn.maxgeom:
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([rad, 0, 0]), np.asarray(pos, float),
                                    np.eye(3).ravel(), np.asarray(rgba, np.float32)); scn.ngeom += 1
        return self.renderer.render()

    def __call__(self, m, d):
        if not (self.ok and self.mp4) or d.time / self.slow + 1e-9 < self.n / self.fps or self.n >= sim.MAX_SECONDS * self.fps:
            return None
        if self.proc is None:
            import subprocess
            self.mp4.parent.mkdir(parents=True, exist_ok=True)
            self.proc = subprocess.Popen(
                ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{self.w}x{self.h}',
                 '-r', str(self.fps), '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '23', '-pix_fmt', 'yuv420p',
                 '-movflags', '+faststart', str(self.mp4)], stdin=subprocess.PIPE)
        frame = self.render(d); assert frame is not None and self.proc.stdin is not None   # self.ok checked above, stdin=PIPE
        self.proc.stdin.write(frame.tobytes()); self.n += 1
        return None


def show_hinge_axes(m):
    """Joint-axis arrows 1.8 x the model's mean body size long; the free joint (drawn as a cube) moved to a hidden group."""
    m.vis.scale.jointlength, m.vis.scale.jointwidth = 1.8, 0.08
    m.jnt_group[0] = 5


def video_turntable():
    """Two turns around the standing robot (kinematic, no dynamics): first the visual meshes with MuJoCo's joint-axis arrows,
    then the collision geometry with the inertia boxes and centres of mass. 30 fps, 6 s per turn."""
    m, d = sim.load(); sim.place_on_ground(m, d)
    show_hinge_axes(m)
    path = MEDIA / 'turntable.mp4'
    with Rec(m, path, width=960, height=540, fps=30, distance=1.25, azimuth=0, elevation=-15) as rec:
        if not rec.ok:
            return None
        n = 180
        for turn in range(2):
            o = rec.opt
            o.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 1; o.jointgroup[5] = 0
            o.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = o.flags[mujoco.mjtVisFlag.mjVIS_COM] = turn
            o.geomgroup[2], o.geomgroup[3] = 1 - turn, turn
            for k in range(n):
                rec.cam.azimuth = 360.0 * k / n + 90; d.time = (turn * n + k) / 30; rec(m, d)
                if turn == 0 and k == n // 8:
                    rec.still(d, MEDIA / 'turntable.png')
                if turn == 1 and k == n // 8:
                    rec.still(d, MEDIA / 'turntable_collision_inertia.png')
        rec.close()
    return path


def video_joint_sweep():
    """Base held 0.45 m above the floor; all four legs move one joint at a time from the stand pose to its lower limit, upper
    limit and back (min-jerk, 1.2 s per segment); joint-axis arrows shown. Kinematic playback, 30 fps."""
    m, d = sim.load(); ix = sim.index(m); r = A.parse()
    show_hinge_axes(m)
    path = MEDIA / 'joint_sweep.mp4'
    with Rec(m, path, width=960, height=540, fps=30, distance=1.35, azimuth=135, elevation=-18) as rec:
        if not rec.ok:
            return None
        rec.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 1; rec.opt.jointgroup[5] = 0
        t = 0.0; q = sim.Q_STAND.copy()
        for jj, part in enumerate(('hip', 'thigh', 'calf')):
            for leg_set in (('FL', 'FR', 'RL', 'RR'),):
                lo = np.array([r.joints[r.joint(f'{l}_{part}_joint')].range[0] for l in LEGS])
                hi = np.array([r.joints[r.joint(f'{l}_{part}_joint')].range[1] for l in LEGS])
                if part == 'thigh':                                    # full flexion range would hit the body: sweep 80% of it
                    lo, hi = q[jj] + 0.8 * (lo - q[jj]), q[jj] + 0.8 * (hi - q[jj])
                if part == 'hip':                                      # mirror the right legs: both sides abduct together
                    left = np.array([l[1] == 'L' for l in LEGS]); lo, hi = np.where(left, lo, -lo), np.where(left, hi, -hi)
                for a, b in ((np.full(4, q[jj]), lo), (lo, hi), (hi, np.full(4, q[jj]))):
                    tr = sim.min_jerk(a, b, 1.2)
                    for s in np.arange(0, 1.2, 1 / 30):
                        qq = q.copy(); qq[jj::3] = tr(s)
                        d.qpos[:] = 0; d.qpos[2] = 0.45; d.qpos[3] = 1; d.qpos[ix.qpos] = qq; d.time = t
                        mujoco.mj_forward(m, d); rec(m, d); t += 1 / 30
                    if part == 'thigh' and b is hi:
                        rec.still(d, MEDIA / 'joint_sweep.png')
        rec.close()
    return path


def video_flight():
    """The flight experiment (RK4, 2 ms) at 1/4 speed from a fixed side camera, with the COM (near-black sphere, the ink colour of the simulated COM in the figures) and the ballistic
    parabola it must follow (amber dots). The COM lies inside the base, so the robot is drawn semi-transparent
    (mjVIS_TRANSPARENT); the marker spheres are added after update_scene and stay opaque."""
    m, _ = sim.load(); path = MEDIA / 'flight.mp4'
    F0 = flight(mujoco.mjtIntegrator.mjINT_RK4); cb = F0['c_ballistic'][::25]
    r = A.parse()
    marks = lambda d: [(p, (0.79, 0.52, 0.06, 0.9), 0.012) for p in cb] + [(A.centroidal(r, d.qpos)[1], (0.122, 0.137, 0.157, 1), 0.03)]  # noqa: E731
    with Rec(m, path, slow=0.25, marks=marks, width=960, height=540, fps=30, distance=2.3, azimuth=90, elevation=-5) as rec:
        if not rec.ok:
            return None
        rec.cam.type = mujoco.mjtCamera.mjCAMERA_FREE; rec.cam.lookat[:] = (0.55, 0, 1.1); rec.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 1
        flight(mujoco.mjtIntegrator.mjINT_RK4, rec=rec)
        rec.close()
    # still: re-run to t = 0.45 s and capture
    with Rec(m, None, marks=marks, width=960, height=540, distance=2.3, azimuth=90, elevation=-5) as rec:
        rec.cam.type = mujoco.mjtCamera.mjCAMERA_FREE; rec.cam.lookat[:] = (0.55, 0, 1.1); rec.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 1
        snap = {}
        flight(mujoco.mjtIntegrator.mjINT_RK4, T=0.45, rec=lambda m, d: snap.update(d=d))
        rec.still(snap['d'], MEDIA / 'flight.png')
    return path


def video_reorient():
    """Zero gravity, 5 forward loops at 1 Hz, real time, side camera following the base: pitch turns by a few degrees per loop."""
    m, _ = sim.load(); path = MEDIA / 'reorientation.mp4'
    with Rec(m, path, width=960, height=540, fps=30, distance=1.6, azimuth=90, elevation=0) as rec:
        if not rec.ok:
            return None
        reorient(rec=rec); rec.close()
        snap = {}
        reorient(cycles=4, rec=lambda m, d: snap.update(d=d))
        rec.still(snap['d'], MEDIA / 'reorientation.png')
    return path


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=1000); ap.add_argument('--no-media', action='store_true')
    ap.add_argument('--json', default=str(HERE / 'results.json')); a = ap.parse_args()
    t0 = time.time(); r = A.parse(); m = mujoco.MjModel.from_xml_path(str(GO2_XML)); pm = pin.buildModelFromMJCF(str(GO2_XML))  # type: ignore[reportAttributeAccessIssue]  # pinocchio: API in a compiled extension, no stubs
    res = {}
    res['fundamental'] = fundamental(r, m, pm)
    gl = can_render(m); views = mv = dv = None                  # set by render_views below when gl
    if gl:
        views, mv, dv = render_views(r, lift=0.0)
        lifted, ml, dl = render_views(r, lift=0.4)                 # in the air: no floor occlusion for the bbox check
        perr, npts, eye = projection_check(ml, dl, lifted)
        res['fundamental'].update(camera_projection_bbox_px=max(perr.values()), camera_projection_per_view_px=perr,
                                  mesh_vertices=npts, camera_eye0_only_shift_px=eye)
    res['intermediate'] = intermediate(r, m, pm, a.n); res['intermediate'].update(torque_budget(r))
    adv = underactuation(r, m, pm, a.n); st, Xs = stance_balance(); adv.update(st)
    D, runs = drift(); adv.update(D)
    Xf, Xr, Xh, Xo = reorient(), reorient(sign=-1.0), reorient(f=0.5), reorient(region=-1.0)
    pf, pr, ph, po = (float(np.degrees(X[-1, 2] - X[0, 2])) for X in (Xf, Xr, Xh, Xo))
    adv.update(reorient_pitch_5_cycles_deg=abs(pf), reorient_pitch_signed_deg=pf, reorient_reverse_deg=pr, reorient_half_speed_deg=ph,
               reorient_reverse_ratio=pr / pf, reorient_half_speed_ratio=ph / pf,
               reorient_other_region_deg=po, reorient_other_region_ratio=po / pf, reorient_other_region_rel_diff=abs(po / pf - 1),
               reorient_other_region_knee_range_rad=[float(Xo[:, 7].min()), float(Xo[:, 7].max())],
               reorient_other_region_track_err_rad=float(np.abs(Xo[:, 6:8] - np.array([loop(region=-1.0)(t)[1:3] for t in Xo[:, 0]])).max()),
               reorient_max_ang_momentum=float(max(X[:, 4].max() for X in (Xf, Xr, Xh, Xo))),
               reorient_max_lin_momentum=float(max(X[:, 5].max() for X in (Xf, Xr, Xh, Xo))),
               reorient_track_err_rad=float(np.abs(Xf[:, 6:8] - np.array([loop()(t)[1:3] for t in Xf[:, 0]])).max()))
    cyc = np.minimum(Xf[:, 0].astype(int), 4); pd_ = np.degrees(Xf[:, 2])           # 1 Hz: cycle c is t in [c, c + 1)
    adv.update(reorient_pitch_swing_per_cycle_deg=[float(np.ptp(pd_[cyc == c])) for c in range(5)],
               reorient_pitch_net_per_cycle_deg=np.diff([pd_[cyc == c][0] for c in range(5)] + [pd_[-1]]).tolist())
    # failure case: the low-gain PD of the first run (kp 60, kd 2), rerun on the current loop (amplitude 0.45 rad)
    Lf, Lh = reorient(kp=60, kd=2), reorient(f=0.5, kp=60, kd=2)
    lf, lh = (float(np.degrees(X[-1, 2] - X[0, 2])) for X in (Lf, Lh))
    adv.update(reorient_lowgain_1Hz_deg=lf, reorient_lowgain_half_speed_deg=lh, reorient_lowgain_half_speed_ratio=lh / lf,
               reorient_lowgain_track_err_rad=float(np.abs(Lf[:, 6:8] - np.array([loop()(t)[1:3] for t in Lf[:, 0]])).max()),
               reorient_lowgain_half_speed_track_err_rad=float(np.abs(Lh[:, 6:8] - np.array([loop(0.5)(t)[1:3] for t in Lh[:, 0]])).max()),
               reorient_half_speed_track_err_rad=float(np.abs(Xh[:, 6:8] - np.array([loop(0.5)(t)[1:3] for t in Xh[:, 0]])).max()))
    F = runs[('rk4', 0.002)]
    adv.update(flight_com_vs_ballistic_m=float(np.abs(F['X'][:, 1:4] - F['c_ballistic']).max()),
               flight_base_pitch_range_deg=float(np.degrees(np.ptp(F['X'][:, 11]))),
               flight_base_vs_ballistic_m=float(np.abs(F['X'][:, 13:16] - F['c_ballistic']).max()))
    res['advanced'] = adv
    if gl:
        fig_views(r, views, mv, dv)
    fig_tree(r); fig_joint_ranges(r); fig_torque_budget(res['intermediate'], res['intermediate'])
    fig_B_and_M(r); fig_underactuation_diagram(); fig_flight(F); fig_stance(Xs); fig_drift(runs, D); fig_reorient(Xf, Xr, Xh, Xo); fig_reorient_failure(Xf, Xh, Lf, Lh)
    flat = {**res['fundamental'], **res['intermediate'], **adv}
    if gl and not a.no_media:
        vids = [video_turntable(), video_joint_sweep(), video_flight(), video_reorient()]
        sizes = {p.name: p.stat().st_size for p in vids if p is not None and p.exists()}
        res['media_bytes'] = sizes; flat['media_mp4_under_6MB'] = float(len(sizes) == 4 and all(s < 6e6 for s in sizes.values()))
    res['checks'] = {k: dict(measured=flat[k], op=op, threshold=thr, passed=check(k, flat[k])) for k, (op, thr) in ACCEPT.items() if k in flat}
    res['skipped_checks'] = [k for k in ACCEPT if k not in flat]
    for k in res['skipped_checks']:
        warnings.warn(f'check {k} skipped ({"no offscreen rendering" if not gl else "--no-media"})')
    print('\nacceptance:')
    for k, c in res['checks'].items():
        print(f'  {"PASS" if c["passed"] else "MISS"}  {k:42s} {c["measured"]:.4g}  ({c["op"]} {c["threshold"]})')
    res['wall_time_s'] = round(time.time() - t0, 1); res['load_average_1min_at_end'] = round(os.getloadavg()[0], 1)
    pathlib.Path(a.json).write_text(json.dumps(res, indent=2, default=float) + '\n')
    print(f'wrote {a.json} in {res["wall_time_s"]} s')


if __name__ == '__main__':
    main()
