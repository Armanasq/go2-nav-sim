#!/usr/bin/env python3
"""Module 04, second experiment: closed-form IK against the numerical IK, the worked standing example, the Jacobian determinant,
and statics tau = J^T f checked against a MuJoCo standing equilibrium.
usage: python analytic.py [--n 1000] [--json analytic.json]"""
import argparse, json, pathlib, sys, time
import numpy as np
import mujoco
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1])); sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from common import GO2_XML, LEGS, require_model  # noqa: E402
from kinematics import Leg  # noqa: E402

STAND = np.array([0.0, 0.9, -1.8])                                   # the MJCF 'home' keyframe, every leg


def ik_comparison(L, n, rng):
    targets, truth = [], []
    for _ in range(n):
        q = rng.uniform(L.lo, L.hi); truth.append(q); targets.append(L.fk(q))
    t0 = time.perf_counter(); cf = [L.ik_closed_form(t) for t in targets]; t_cf = (time.perf_counter() - t0) / n
    t0 = time.perf_counter(); num = [L.ik_multistart(t, STAND) for t in targets]; t_num = (time.perf_counter() - t0) / n
    rec, res, agree, n_multi = [], [], [], 0
    for t, q, S, (qn, _, _, ok) in zip(targets, truth, cf, num):
        rec.append(np.abs(S - q).max(1).min() if len(S) else np.inf)            # generating q among the closed-form solutions
        res.append(max(np.linalg.norm(L.fk(s) - t) for s in S) if len(S) else np.inf)
        n_multi += len(S) > 1
        if ok:                                                                  # polish the numerical answer, then compare
            qp = L.ik(t, qn, tol=1e-13, max_iter=100)[0]
            agree.append(np.abs(S - qp).max(1).min())
    return dict(closed_form_success=float(np.mean(np.array(rec) < 1e-9)), closed_form_max_q_error_rad=float(max(rec)),
                closed_form_max_fk_residual_m=float(max(res)), targets_with_two_solutions=n_multi / n,
                numerical_success=float(np.mean([r[3] for r in num])), max_q_diff_closed_vs_numerical_rad=float(max(agree)),
                agree_to_1e9_fraction=float(np.mean(np.array(agree) < 1e-9)),
                closed_form_us=t_cf * 1e6, numerical_us=t_num * 1e6)


def worked_example(L):
    """Every intermediate quantity of the closed form for one target, so the README numbers can be followed by hand."""
    def steps(target):
        x, y, z = target - L.hip; r2 = y * y + z * z - L.d ** 2; sz = -np.sqrt(r2)
        D = (x * x + sz * sz - L.l1 ** 2 - L.l2 ** 2) / (2 * L.l1 * L.l2); S = L.ik_closed_form(target)
        return dict(target_minus_hip=[x, y, z], s_z=sz, cos_q3_plus_gamma=D, solutions=S.tolist(),
                    fk_residual_m=[float(np.linalg.norm(L.fk(s) - target)) for s in S])
    lv = L.l1 ** 2
    return dict(standing=dict(q=STAND.tolist(), foot=L.fk(STAND).tolist(), **steps(L.fk(STAND))),
                reviewer_ideal_cos_q3=(0.225 ** 2 - 2 * lv) / (2 * lv),
                below_thigh_joint_0p225=steps(L.hip + [0, L.d, -0.225]),          # foot straight under the thigh joint
                below_hip_joint_0p225=steps(L.hip + [0, 0, -0.225]))             # foot straight under the abduction joint


def singularity(L, rng):
    qs = [rng.uniform(L.lo, L.hi) for _ in range(5000)]
    k = np.linspace(L.lo[2], L.hi[2], 100001)
    return dict(det_formula_vs_numpy_max=float(max(abs(L.det_jacobian(q) - np.linalg.det(L.jacobian(q))) for q in qs)),
                min_abs_sin_q3_plus_gamma=float(np.abs(np.sin(k + L.gamma)).min()), min_abs_sin_q3=float(np.abs(np.sin(k)).min()),
                det_at_standing=float(L.det_jacobian(STAND)), cond_at_standing=float(np.linalg.cond(L.jacobian(STAND))))


def contact_wrench_torque(L, q, Rb, pb, p, F, M):
    """Joint torques of a wrench (force F at world point p, moment M) acting on the calf: tau_i = z_i . ((p - o_i) x F + M)."""
    o, z, _ = L.frames(q); pl, Fl, Ml = Rb.T @ (p - pb), Rb.T @ F, Rb.T @ M
    return np.array([z[i] @ (np.cross(pl - o[i], Fl) + Ml) for i in range(3)])


def statics(m_scene):
    m = m_scene; d = mujoco.MjData(m); g = -m.opt.gravity[2]; mass = mujoco.mj_getTotalmass(m)
    legs = {leg: Leg(m, leg) for leg in LEGS}
    hand = {leg: (-L.jacobian(STAND).T @ [0, 0, mass * g / 4]).tolist() for leg, L in legs.items()}   # massless legs, 4 feet
    trot = {leg: (-L.jacobian(STAND).T @ [0, 0, mass * g / 2]).tolist() for leg, L in legs.items()}   # 2 feet
    # MuJoCo: hold the home pose with a joint PD on the torque motors until the robot is at rest. Frictionloss is zeroed so the
    # joint equation is tau_act + tau_damping + tau_contact = gravity (frictionloss could absorb up to 0.2 N m per joint at rest).
    # The soft contacts let the feet creep outward (hip joints still move at ~1e-3 rad/s after 6 s), so joint damping
    # (2 N m s/rad x 1e-3 rad/s) is kept as a term.
    m.dof_frictionloss[:] = 0
    mujoco.mj_resetDataKeyframe(m, d, 0); qdes = d.qpos[7:].copy()
    for _ in range(int(6 / m.opt.timestep)):
        d.ctrl[:] = 80 * (qdes - d.qpos[7:]) - 4 * d.qvel[6:]
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    Rb = d.xmat[1].reshape(3, 3); pb = d.xpos[1]; feet = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, leg): leg for leg in LEGS}
    tau_c = {leg: np.zeros(3) for leg in LEGS}; Fw = {leg: np.zeros(3) for leg in LEGS}; f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]; mujoco.mj_contactForce(m, d, i, f6); fr = c.frame.reshape(3, 3)
        leg = feet.get(c.geom2) or feet.get(c.geom1); assert leg, 'only feet should touch the floor'
        sgn = 1 if c.geom2 in feet else -1                                     # mj_contactForce: force of geom1 on geom2
        F, Mo = sgn * fr.T @ f6[:3], sgn * fr.T @ f6[3:]
        Fw[leg] += F; L = legs[leg]
        tau_c[leg] += contact_wrench_torque(L, d.qpos[7 + 3 * LEGS.index(leg):][:3], Rb, pb, c.pos, F, Mo)
    out = dict(total_mass_kg=mass, g=g, weight_per_foot_N=mass * g / 4, weight_per_foot_trot_N=mass * g / 2, hand_tau=hand, trot_tau=trot,
               limits=m.actuator_ctrlrange[:3, 1].tolist(), max_abs_qvel=float(np.abs(d.qvel).max()), base_height=float(pb[2]),
               base_rpy_deg=np.rad2deg([np.arctan2(Rb[2, 1], Rb[2, 2]), -np.arcsin(Rb[2, 0]), 0]).tolist(), legs={})
    # Lever rule for the front/rear split: front pair carries mg (x_com - x_rear) / (x_front - x_rear)
    xf, xr = (d.geom_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, l)][0] for l in ('FL', 'RL'))
    out['lever_rule_front_foot_N'] = float(mass * g * (d.subtree_com[1][0] - xr) / (xf - xr) / 2)
    out['com_x_minus_feet_mid_m'] = float(d.subtree_com[1][0] - (xf + xr) / 2)
    for k, leg in enumerate(LEGS):
        dof = slice(6 + 3 * k, 9 + 3 * k); grav = d.qfrc_bias[dof] - d.qfrc_passive[dof]
        act = d.actuator_force[3 * k:3 * k + 3]
        out['legs'][leg] = dict(contact_force_world_N=Fw[leg].tolist(), mujoco_actuator_tau=act.tolist(), JTf_tau=(-tau_c[leg]).tolist(),
                                leg_gravity_minus_damping_tau=grav.tolist(), predicted_tau=(grav - tau_c[leg]).tolist(),
                                max_abs_residual=float(np.abs(act - (grav - tau_c[leg])).max()))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--n', type=int, default=1000); ap.add_argument('--json'); a = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(require_model()); rng = np.random.default_rng(0)
    res = dict(ik={leg: ik_comparison(Leg(m, leg), a.n, rng) for leg in LEGS}, worked_FL=worked_example(Leg(m, 'FL')),
               singularity=singularity(Leg(m, 'FL'), rng), statics=statics(mujoco.MjModel.from_xml_path(str(GO2_XML.parent / 'scene.xml'))))
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(res, indent=2, default=float) + '\n')
    np.set_printoptions(precision=5, suppress=True)
    for leg, r in res['ik'].items():
        print(f"{leg} IK: closed form {100 * r['closed_form_success']:.1f}% (max |dq| {r['closed_form_max_q_error_rad']:.1e}, "
              f"{r['closed_form_us']:.0f} us) | numerical {100 * r['numerical_success']:.1f}% ({r['numerical_us']:.0f} us) | "
              f"closed vs polished numerical: {100 * r['agree_to_1e9_fraction']:.1f}% within 1e-9, max {r['max_q_diff_closed_vs_numerical_rad']:.1e} | two solutions {100 * r['targets_with_two_solutions']:.1f}%")
    print(json.dumps(dict(worked_FL=res['worked_FL'], singularity=res['singularity']), indent=1, default=float))
    s = res['statics']; print(f"mass {s['total_mass_kg']:.4f} kg, per foot {s['weight_per_foot_N']:.3f} N, trot {s['weight_per_foot_trot_N']:.3f} N; "
                              f"rest: max |qvel| {s['max_abs_qvel']:.1e}, base z {s['base_height']:.4f} m, rpy {np.round(s['base_rpy_deg'], 3)} deg")
    for leg in LEGS:
        print(f"{leg} hand {np.array(s['hand_tau'][leg])} trot {np.array(s['trot_tau'][leg])}")
    print(f"lever rule front foot {s['lever_rule_front_foot_N']:.3f} N, CoM ahead of feet midpoint {s['com_x_minus_feet_mid_m']:.4f} m")
    for leg, r in s['legs'].items():
        print(f"{leg} F {np.array(r['contact_force_world_N'])} N | MuJoCo tau {np.array(r['mujoco_actuator_tau'])} | -J^T f {np.array(r['JTf_tau'])} + "
              f"leg gravity - damping {np.array(r['leg_gravity_minus_damping_tau'])} | residual {r['max_abs_residual']:.1e}")


if __name__ == '__main__':
    main()
