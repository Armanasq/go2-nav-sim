"""Module 01: the Go2's anatomy read from its MJCF file, from first principles.

parse()          reads go2.xml with ElementTree and resolves MJCF default classes by hand: bodies (offsets, mass, inertia),
                 joints (type, axis, range, damping, armature, friction loss) and actuators (transmission, gear, torque range).
dof()            counts generalised coordinates and velocities: nq = 7 + 12, nv = 6 + 12.
selection()      the actuation matrix B (nv x nu) in  tau_generalised = B u;  its first six rows are zero.
fk()             world pose of every body, and world anchor and axis of every joint, by composing the parsed offsets.
velocities()     body angular velocity and origin velocity by recursive propagation down the tree.
centroidal()     total mass, centre of mass, linear and angular momentum about the centre of mass.
external_rate()  rate of change of centroidal momentum that gravity and a set of contact wrenches produce.
MuJoCo and Pinocchio are not called here; validate.py and the tests use them only to check these functions."""
import pathlib
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common import GO2_XML, axis_angle_R, quat_to_R  # noqa: E402

# (nq, nv) per joint type, MJCF reference "joint/type"
DOF = {'free': (7, 6), 'ball': (4, 3), 'slide': (1, 1), 'hinge': (1, 1)}
# MuJoCo's built-in values for the attributes read here, used when neither the element nor any default class sets them
BUILTIN = {'joint': dict(type='hinge', axis='0 0 1', pos='0 0 0', damping='0', armature='0', frictionloss='0'),
           'motor': dict(gear='1 0 0 0 0 0')}


def _v(s):
    return np.array(s.split(), float)


def _R(s):
    """Rotation from an MJCF quaternion string (w x y z). MuJoCo normalises quaternions on compile; so must we: the file's
    inertial quaternions are given to 6 digits and their norms differ from 1 by up to 4e-7."""
    q = _v(s)
    return quat_to_R(q / np.linalg.norm(q))


@dataclass
class Body:
    name: str
    parent: int              # index into Robot.bodies, -1 for the world
    pos: np.ndarray          # origin in the parent frame
    R: np.ndarray            # orientation in the parent frame
    mass: float
    com: np.ndarray          # centre of mass in the body frame
    inertia: np.ndarray      # 3x3 rotational inertia about the centre of mass, body frame
    joints: list = field(default_factory=list)   # indices into Robot.joints
    geoms: dict = field(default_factory=dict)    # {'visual': n, 'collision': n}


@dataclass
class Joint:
    name: str
    body: int
    type: str
    axis: np.ndarray         # unit vector, body frame
    pos: np.ndarray          # anchor in the body frame
    range: tuple             # (lo, hi) rad, or None when unlimited
    damping: float
    armature: float
    frictionloss: float
    qadr: int                # first index in qpos
    vadr: int                # first index in qvel


@dataclass
class Actuator:
    name: str
    kind: str                # element tag: motor, position, ...
    joint: int
    gear: float
    ctrlrange: tuple


@dataclass
class Robot:
    bodies: list
    joints: list
    actuators: list
    nq: int
    nv: int

    def body(self, name):
        return next(i for i, b in enumerate(self.bodies) if b.name == name)

    def joint(self, name):
        return next(i for i, j in enumerate(self.joints) if j.name == name)

    @property
    def mass(self):
        return sum(b.mass for b in self.bodies)


def _defaults(root):
    """{class name: {tag: attributes}}. A nested <default> inherits everything from its parent and overrides what it sets."""
    out = {}

    def walk(el, inherited):
        attrs = {t: dict(a) for t, a in inherited.items()}
        for c in el:
            if c.tag != 'default':
                attrs.setdefault(c.tag, {}).update(c.attrib)
        out[el.get('class', 'main')] = attrs
        for c in el:
            if c.tag == 'default':
                walk(c, attrs)

    top = root.find('default')
    walk(top if top is not None else ET.Element('default'), {})
    return out


def parse(path=GO2_XML):
    """Robot tables from an MJCF file. Handles what go2.xml uses: default classes and childclass, freejoint, hinge joints with
    range and autolimits, <inertial> with diaginertia or fullinertia, joint-transmission actuators."""
    root = ET.parse(path).getroot()
    comp = root.find('compiler')
    scale = 1.0 if comp is not None and comp.get('angle', 'degree') == 'radian' else np.pi / 180
    autolimits = comp is None or comp.get('autolimits', 'true') == 'true'
    cls = _defaults(root)

    def resolve(el, tag, active):
        a = dict(BUILTIN.get(tag, {}))
        a.update(cls.get('main', {}).get(tag, {}))
        a.update(cls[el.get('class', active)].get(tag, {}))
        a.update(el.attrib)
        return a

    bodies, joints, nq, nv = [], [], 0, 0

    def walk(el, parent, active):
        nonlocal nq, nv
        active = el.get('childclass', active)
        i = len(bodies)
        inl = el.find('inertial')
        Ri = _R(inl.get('quat', '1 0 0 0'))
        if inl.get('fullinertia'):
            xx, yy, zz, xy, xz, yz = _v(inl.get('fullinertia'))
            Ic = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        else:
            Ic = Ri @ np.diag(_v(inl.get('diaginertia'))) @ Ri.T
        b = Body(el.get('name'), parent, _v(el.get('pos', '0 0 0')), _R(el.get('quat', '1 0 0 0')),
                 float(inl.get('mass')), _v(inl.get('pos', '0 0 0')), Ic, geoms={'visual': 0, 'collision': 0})
        bodies.append(b)
        for c in el:
            if c.tag in ('joint', 'freejoint'):
                a = {'type': 'free', 'axis': '0 0 1', 'pos': '0 0 0', 'damping': '0', 'armature': '0', 'frictionloss': '0',
                     **c.attrib} if c.tag == 'freejoint' else resolve(c, 'joint', active)   # freejoint takes no defaults
                rng = a.get('range')
                limited = a.get('limited', 'auto')
                limited = (rng is not None) if limited == 'auto' and autolimits else limited == 'true'
                ax = _v(a['axis'])
                j = Joint(a.get('name', f'{b.name}_{a["type"]}'), i, a['type'], ax / np.linalg.norm(ax), _v(a['pos']),
                          tuple(scale * _v(rng)) if limited and a['type'] in ('hinge', 'ball') else (tuple(_v(rng)) if limited else None),
                          float(a['damping']), float(a['armature']), float(a['frictionloss']), nq, nv)
                nq += DOF[j.type][0]; nv += DOF[j.type][1]
                b.joints.append(len(joints)); joints.append(j)
            elif c.tag == 'geom':
                g = resolve(c, 'geom', active)
                b.geoms['visual' if g.get('contype') == '0' and g.get('conaffinity') == '0' else 'collision'] += 1
        for c in el.findall('body'):
            walk(c, i, active)

    for el in root.find('worldbody').findall('body'):
        walk(el, -1, 'main')

    acts = []
    for el in root.find('actuator'):
        a = resolve(el, el.tag, 'main')
        jn = [k for k, j in enumerate(joints) if j.name == a['joint']]
        assert len(jn) == 1, f'actuator {a.get("name")}: only joint transmissions are handled'
        g = _v(a['gear'])
        assert np.allclose(g[1:], 0), 'hinge transmission uses only the first gear component'
        cr = a.get('ctrlrange')
        acts.append(Actuator(a.get('name'), el.tag, jn[0], float(g[0]), tuple(_v(cr)) if cr else None))
    return Robot(bodies, joints, acts, nq, nv)


def dof(robot):
    """Counts that define the system: coordinates, velocities, actuated and unactuated velocity directions."""
    B = selection(robot)
    rank = int(np.linalg.matrix_rank(B))
    return dict(nq=robot.nq, nv=robot.nv, nu=len(robot.actuators), rank_B=rank, unactuated=robot.nv - rank,
                free_joints=sum(j.type == 'free' for j in robot.joints),
                hinges=sum(j.type == 'hinge' for j in robot.joints), bodies=len(robot.bodies))


def selection(robot):
    """B (nv x nu): column k puts actuator k's gear on its joint's velocity index. tau = B u."""
    B = np.zeros((robot.nv, len(robot.actuators)))
    for k, a in enumerate(robot.actuators):
        j = robot.joints[a.joint]
        assert j.type in ('hinge', 'slide')
        B[j.vadr, k] = a.gear
    return B


def actuator_force(robot, u):
    """Generalised force of the actuators: clip u to ctrlrange (MuJoCo clamps ctrl when ctrlrange is set), then B u."""
    u = np.array(u, float)
    for k, a in enumerate(robot.actuators):
        if a.ctrlrange is not None:
            u[k] = np.clip(u[k], *a.ctrlrange)
    return selection(robot) @ u


def fk(robot, qpos):
    """World pose of each body (R (n,3,3), p (n,3)) and world anchor and axis of each joint ((nj,3), (nj,3)).
    Body i:  R_i = R_parent R_off,  p_i = p_parent + R_parent pos_off;  then each hinge at anchor a (body frame) about unit
    axis s rotates the frame:  R <- R Rot(s, q),  p <- p + R a - R Rot(s, q) a.  A free joint sets the pose from qpos."""
    n = len(robot.bodies)
    R, p = np.zeros((n, 3, 3)), np.zeros((n, 3))
    anchor, axis = np.zeros((len(robot.joints), 3)), np.zeros((len(robot.joints), 3))
    for i, b in enumerate(robot.bodies):
        Rp, pp = (np.eye(3), np.zeros(3)) if b.parent < 0 else (R[b.parent], p[b.parent])
        Ri, pi = Rp @ b.R, pp + Rp @ b.pos
        for k in b.joints:
            j = robot.joints[k]
            if j.type == 'free':
                pi, Ri = np.array(qpos[j.qadr:j.qadr + 3], float), quat_to_R(qpos[j.qadr + 3:j.qadr + 7])
                anchor[k] = pi                                 # a free joint has no single axis; axis[k] stays 0
                continue
            assert j.type == 'hinge'
            anchor[k], axis[k] = pi + Ri @ j.pos, Ri @ j.axis
            Rj = axis_angle_R(j.axis, qpos[j.qadr])
            pi, Ri = pi + Ri @ j.pos - Ri @ Rj @ j.pos, Ri @ Rj
        R[i], p[i] = Ri, pi
    return R, p, anchor, axis


def velocities(robot, qpos, qvel):
    """World angular velocity w_i and world velocity v_i of each body origin.
    Child of a moving parent:  w_i = w_p + sum_j s_j dq_j,   v_i = v_p + w_p x (p_i - p_p) + sum_j (s_j dq_j) x (p_i - a_j)
    with s_j, a_j the world axis and anchor of the hinges in body i. Free joint (MuJoCo convention): v = qvel[0:3] in the world,
    w = R qvel[3:6] (body frame)."""
    R, p, anchor, axis = fk(robot, qpos)
    n = len(robot.bodies)
    w, v = np.zeros((n, 3)), np.zeros((n, 3))
    for i, b in enumerate(robot.bodies):
        wi, vi = (np.zeros(3), np.zeros(3)) if b.parent < 0 else (w[b.parent], v[b.parent] + np.cross(w[b.parent], p[i] - p[b.parent]))
        for k in b.joints:
            j = robot.joints[k]
            if j.type == 'free':
                vi, wi = np.array(qvel[j.vadr:j.vadr + 3], float), R[i] @ qvel[j.vadr + 3:j.vadr + 6]
                continue
            om = axis[k] * qvel[j.vadr]
            wi, vi = wi + om, vi + np.cross(om, p[i] - anchor[k])
        w[i], v[i] = wi, vi
    return R, p, w, v


def centroidal(robot, qpos, qvel=None):
    """Total mass M, centre of mass c, linear momentum l = sum m_i vc_i and angular momentum about c,
    k = sum [ m_i (c_i - c) x vc_i + R_i I_i R_i^T w_i ], all in the world frame."""
    nv = robot.nv
    R, p, w, v = velocities(robot, qpos, np.zeros(nv) if qvel is None else qvel)
    m = np.array([b.mass for b in robot.bodies])
    ci = p + np.einsum('nij,nj->ni', R, np.array([b.com for b in robot.bodies]))
    vc = v + np.cross(w, ci - p)
    M = m.sum(); c = m @ ci / M
    l = m @ vc
    k = sum(m[i] * np.cross(ci[i] - c, vc[i]) + R[i] @ robot.bodies[i].inertia @ R[i].T @ w[i] for i in range(len(m)))
    return M, c, l, k


def integrate(robot, qpos, qvel, h):
    """qpos after moving with constant qvel for time h: hinges add, the free joint's quaternion turns by the body rate."""
    q = np.array(qpos, float)
    for j in robot.joints:
        if j.type == 'free':
            q[j.qadr:j.qadr + 3] += h * np.asarray(qvel[j.vadr:j.vadr + 3])
            wq = np.asarray(qvel[j.vadr + 3:j.vadr + 6]) * h; th = np.linalg.norm(wq)
            dq = np.r_[np.cos(th / 2), np.sin(th / 2) * wq / th] if th > 0 else np.array([1.0, 0, 0, 0])
            a = q[j.qadr + 3:j.qadr + 7]                          # body-frame rate: q <- q * dq (Hamilton product)
            q[j.qadr + 3:j.qadr + 7] = [a[0] * dq[0] - a[1:] @ dq[1:], *(a[0] * dq[1:] + dq[0] * a[1:] + np.cross(a[1:], dq[1:]))]
        else:
            q[j.qadr] += h * qvel[j.vadr]
    return q


def momentum_rate(robot, qpos, qvel, qacc, eps=1e-6):
    """d/dt (l, k) along the motion (q, v, a) by a central difference of centroidal() in time: O(eps^2) accurate."""
    hp = centroidal(robot, integrate(robot, qpos, qvel, eps), np.asarray(qvel) + eps * np.asarray(qacc))
    hm = centroidal(robot, integrate(robot, qpos, qvel, -eps), np.asarray(qvel) - eps * np.asarray(qacc))
    return (hp[2] - hm[2]) / (2 * eps), (hp[3] - hm[3]) / (2 * eps)


def external_rate(M, c, g, points=(), forces=(), torques=()):
    """Momentum rate the outside world causes: l_dot = M g + sum f_i, k_dot = sum (p_i - c) x f_i + tau_i. Gravity acts at c,
    so it adds no moment about c. Joint torques, damping and limits are internal and do not appear."""
    ld, kd = M * np.asarray(g, float), np.zeros(3)
    for i, (pt, f) in enumerate(zip(points, forces)):
        ld = ld + f
        kd = kd + np.cross(np.asarray(pt) - c, f) + (torques[i] if len(torques) else 0)
    return ld, kd
