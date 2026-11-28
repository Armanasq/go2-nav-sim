"""Module 02: robot description. The Gazebo URDF of this repository against the MuJoCo Menagerie MJCF, from first principles.

parse_urdf()        reads a URDF with ElementTree: links (mass, centre of mass, inertia about it, collision primitives), joints
                    (origin, axis, limits, dynamics) and the <gazebo reference> names.
lump()              merges every fixed joint into its moving parent (mass sum, mass-weighted centre, parallel-axis theorem), so the
                    URDF has the same 13 bodies as the MJCF.
inertia_report()    validity of one inertia: positive definite, triangle inequality, pseudo-inertia (LMI) and a size check.
mirror_error()      left-right symmetry of two bodies under the reflection y -> -y.
gravity_regressor() Y(q, g) with tau_gravity = Y pi, pi = [m_i, m_i c_i] per body: linear in the mass parameters.
lstsq_qr()          least squares by QR, with the parameter covariance from the residual.
MuJoCo and Pinocchio are not called here (mjcf_collisions only reads geom sizes from a compiled model); validate.py and the
tests use them to check these functions."""
import pathlib
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'm01_anatomy'))
import anatomy as A  # noqa: E402  (module 01: MJCF parser and tree kinematics)

REPO = HERE.parents[1]
DESC = REPO / 'src/third_party/unitree-go2-ros2/robots/descriptions/go2_description'
GAZEBO_XACRO = REPO / 'src/go2_nav_sim/urdf/go2_nav.xacro'  # what src/go2_nav_sim/launch/sim.launch.py spawns: upstream
                                                 # robot.xacro plus sensors.xacro (5 fixed sensor links on base_link)
XACRO_ARGS = 'start_x:=-4.5 start_y:=-3.5'      # sim.launch.py START_X, START_Y; they only enter the ground-truth plugin
GAZEBO_URDF = HERE / 'go2_gazebo.urdf'           # xacro output of GAZEBO_XACRO, stored so the module runs without ROS
UNITREE_URDF = DESC / 'urdf/go2_description.urdf'  # also in the package, not used by the Gazebo launch
S_MIRROR = np.diag([1.0, -1.0, 1.0])             # reflection y -> -y (left <-> right)
LEG_OF = {'lf': 'FL', 'rf': 'FR', 'lh': 'RL', 'rh': 'RR'}
SEG_OF = {'hip': 'hip', 'upper_leg': 'thigh', 'lower_leg': 'calf', 'thigh': 'thigh', 'calf': 'calf'}


# ---- URDF ----------------------------------------------------------------------------------------------------------------

def xacro(src=GAZEBO_XACRO):
    """URDF text from xacro, with the ROS 2 Humble environment and this repository's install space; None without ROS."""
    setup = REPO / 'install/setup.bash'
    if not (shutil.which('bash') and pathlib.Path('/opt/ros/humble/setup.bash').exists() and setup.exists()):
        return None
    r = subprocess.run(['bash', '-c', f'source /opt/ros/humble/setup.bash && source {setup} && xacro {src} {XACRO_ARGS}'],
                       capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else None


def rpy_R(rpy):
    """URDF origin rpy: fixed-axis roll about x, then pitch about y, then yaw about z: R = Rz(y) Ry(p) Rx(r)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin') if el is not None else None
    xyz = np.array((o.get('xyz', '0 0 0') if o is not None else '0 0 0').split(), float)
    rpy = np.array((o.get('rpy', '0 0 0') if o is not None else '0 0 0').split(), float)
    return xyz, rpy_R(rpy)


@dataclass
class Link:
    name: str
    mass: float                 # kg; 0 when the link has no <inertial>
    com: np.ndarray             # centre of mass, link frame
    I: np.ndarray               # 3x3 inertia about the centre of mass, link frame
    collisions: list            # dicts: type (box|cylinder|sphere), size (MJCF convention: half sizes / radius, half-length)
    has_inertial: bool = True   #        pos, R in the link frame


@dataclass
class UJoint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    R: np.ndarray
    axis: np.ndarray
    lower: float = None
    upper: float = None
    effort: float = None
    velocity: float = None
    damping: float = 0.0
    friction: float = 0.0


def _collision(c):
    pos, R = _origin(c)
    g = c.find('geometry')[0]
    if g.tag == 'box':
        size = np.array(g.get('size').split(), float) / 2              # URDF full lengths -> MJCF half sizes
    elif g.tag == 'cylinder':
        size = np.array([float(g.get('radius')), float(g.get('length')) / 2])
    elif g.tag == 'sphere':
        size = np.array([float(g.get('radius'))])
    else:
        return None                                                     # meshes: not compared here
    return dict(type=g.tag, size=size, pos=pos, R=R)


def parse_urdf(path=GAZEBO_URDF):
    """(links {name: Link}, joints {name: UJoint}, gazebo reference names). Inertia given in the <inertial><origin> frame is
    rotated into the link frame: I_link = R I R^T."""
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for el in root.findall('link'):
        inl = el.find('inertial')
        if inl is None:
            links[el.get('name')] = Link(el.get('name'), 0.0, np.zeros(3), np.zeros((3, 3)), [], False)
        else:
            c, R = _origin(inl)
            t = inl.find('inertia').attrib
            Ii = np.array([[t['ixx'], t['ixy'], t['ixz']], [t['ixy'], t['iyy'], t['iyz']], [t['ixz'], t['iyz'], t['izz']]], float)
            links[el.get('name')] = Link(el.get('name'), float(inl.find('mass').get('value')), c, R @ Ii @ R.T, [])
        links[el.get('name')].collisions = [g for g in map(_collision, el.findall('collision')) if g is not None]
    for el in root.findall('joint'):
        xyz, R = _origin(el)
        ax = el.find('axis'); lim = el.find('limit'); dyn = el.find('dynamics')
        f = lambda e, k: None if e is None or e.get(k) is None else float(e.get(k))  # noqa: E731
        joints[el.get('name')] = UJoint(el.get('name'), el.get('type'), el.find('parent').get('link'),
                                        el.find('child').get('link'), xyz, R,
                                        np.array((ax.get('xyz') if ax is not None else '1 0 0').split(), float),
                                        f(lim, 'lower'), f(lim, 'upper'), f(lim, 'effort'), f(lim, 'velocity'),
                                        f(dyn, 'damping') or 0.0, f(dyn, 'friction') or 0.0)
    refs = [g.get('reference') for g in root.findall('gazebo') if g.get('reference')]
    return links, joints, refs


def combine(parts):
    """Lumped (m, c, I_c) of rigid parts [(m_k, c_k, I_k)] given in one frame:
    m = sum m_k,  c = sum m_k c_k / m,  I_c = sum [ I_k + m_k (|d_k|^2 1 - d_k d_k^T) ],  d_k = c_k - c   (parallel axis)."""
    m = sum(p[0] for p in parts)
    if m == 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    c = sum(p[0] * p[1] for p in parts) / m
    I = np.zeros((3, 3))
    for mk, ck, Ik in parts:
        dk = ck - c
        I += Ik + mk * (dk @ dk * np.eye(3) - np.outer(dk, dk))
    return m, c, I


@dataclass
class Body:
    name: str                   # canonical MJCF name (base, FL_hip, ...) when canonical() knows the link, else the link name
    link: str                   # the URDF link that owns the frame
    parent: int                 # index into the body list, -1 for the root
    joint: str                  # moving URDF joint into this body, None for the root
    pos: np.ndarray             # frame origin in the parent body frame at q = 0
    R: np.ndarray
    mass: float
    com: np.ndarray
    I: np.ndarray
    collisions: list
    members: list = field(default_factory=list)   # URDF links merged into this body


def canonical(link):
    """URDF link name -> Menagerie body name: lf_upper_leg_link -> FL_thigh (Gazebo xacro), FL_calf -> FL_calf (Unitree)."""
    m = re.fullmatch(r'(lf|rf|lh|rh)_(hip|upper_leg|lower_leg)_link', link)
    if m:
        return f'{LEG_OF[m[1]]}_{SEG_OF[m[2]]}'
    m = re.fullmatch(r'(FL|FR|RL|RR)_(hip|thigh|calf)', link)
    return link if m else None


def lump(links, joints):
    """Bodies of the URDF tree with every fixed joint merged into its parent: a fixed child at (p, R) in the parent adds the part
    (m, p + R c, R I R^T) and its collision primitives moved by the same transform. Depth-first from the root link."""
    children = {}
    for j in joints.values():
        children.setdefault(j.parent, []).append(j)
    root = next(n for n in links if all(j.child != n for j in joints.values()))
    bodies, parts = [], []

    def visit(link, body, p, R):
        L = links[link]
        parts[body].append((L.mass, p + R @ L.com, R @ L.I @ R.T))
        bodies[body].members.append(link)
        bodies[body].collisions += [dict(g, pos=p + R @ g['pos'], R=R @ g['R']) for g in L.collisions]
        for j in children.get(link, []):
            if j.type == 'fixed':
                visit(j.child, body, p + R @ j.xyz, R @ j.R)
            else:
                bodies.append(Body(canonical(j.child) or j.child, j.child, body, j.name, p + R @ j.xyz, R @ j.R,
                                   0.0, None, None, []))
                parts.append([])
                visit(j.child, len(bodies) - 1, np.zeros(3), np.eye(3))

    bodies.append(Body('base', root, -1, None, np.zeros(3), np.eye(3), 0.0, None, None, [])); parts.append([])
    visit(root, 0, np.zeros(3), np.eye(3))
    for b, P in zip(bodies, parts):
        b.mass, b.com, b.I = combine(P)
    return bodies


# ---- MJCF side (module 01's parser; collision geoms read from the compiled model) --------------------------------------------

def mjcf_collisions(m):
    """{body name: [collision primitive dicts]} from a compiled MjModel: geoms in group 3 (go2.xml class "collision")."""
    kinds = {2: 'sphere', 5: 'cylinder', 6: 'box'}
    out = {}
    for g in range(m.ngeom):
        if m.geom_group[g] != 3:
            continue
        name = m.body(m.geom_bodyid[g]).name
        n = {'sphere': 1, 'cylinder': 2, 'box': 3}[kinds[int(m.geom_type[g])]]
        out.setdefault(name, []).append(dict(type=kinds[int(m.geom_type[g])], size=m.geom_size[g][:n].copy(),
                                              pos=m.geom_pos[g].copy(), R=A.quat_to_R(m.geom_quat[g])))
    return out


def volume(g):
    s = g['size']
    return {'box': lambda: 8 * s.prod(), 'cylinder': lambda: np.pi * s[0] ** 2 * 2 * s[1],
            'sphere': lambda: 4 / 3 * np.pi * s[0] ** 3}[g['type']]()


def aabb(geoms):
    """(lo, hi) of the axis-aligned box around a list of primitives in their common frame. Box: half extents |R| h. Cylinder of
    radius r and half-length L along its local z = R e_z: L |e_z| + r sqrt(1 - e_z^2) per axis. Sphere: r."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in geoms:
        s, R = g['size'], g['R']
        if g['type'] == 'box':
            e = np.abs(R) @ s
        elif g['type'] == 'cylinder':
            ez = np.abs(R[:, 2]); e = s[1] * ez + s[0] * np.sqrt(np.clip(1 - ez ** 2, 0, None))
        else:
            e = np.full(3, s[0])
        lo, hi = np.minimum(lo, g['pos'] - e), np.maximum(hi, g['pos'] + e)
    return lo, hi


# ---- validity of an inertia --------------------------------------------------------------------------------------------------

def pseudo_inertia(m, c, Ic):
    """4x4 pseudo-inertia J = [[Sigma, h], [h^T, m]] about the body origin, h = m c, Sigma = tr(I_o)/2 1 - I_o with
    I_o = I_c + m (|c|^2 1 - c c^T). The mass distribution is physically realisable iff J is positive definite."""
    Io = Ic + m * (c @ c * np.eye(3) - np.outer(c, c))
    J = np.zeros((4, 4))
    J[:3, :3] = 0.5 * np.trace(Io) * np.eye(3) - Io
    J[:3, 3] = J[3, :3] = m * c
    J[3, 3] = m
    return J


def triangle_margin(Ic):
    """min_k (d_1 + d_2 + d_3 - 2 d_k) / tr I over the principal moments d: >= 0 iff d_i + d_j >= d_k for every k.
    0 for a thin rod, 1/3 for a sphere, negative for a tensor no mass distribution can have."""
    d = np.linalg.eigvalsh(0.5 * (Ic + Ic.T))
    return float((d.sum() - 2 * d.max()) / d.sum()) if d.sum() > 0 else float('nan')


def is_pd(M):
    try:
        np.linalg.cholesky(0.5 * (M + M.T))
        return True
    except np.linalg.LinAlgError:
        return False


def inertia_report(m, c, Ic, geoms=None):
    """Checks of one body: symmetric, positive definite, triangle inequality, pseudo-inertia positive definite, and the size
    check  r_g = sqrt(d_max / m) <= R_max  (radius of gyration about any axis through c cannot exceed the farthest point of the
    body from c); R_max is taken from the corners of the collision primitives' bounding box, so it is a plausibility test."""
    d = np.linalg.eigvalsh(0.5 * (Ic + Ic.T))
    rep = dict(mass=float(m), eig=d.tolist(), symmetric=bool(np.allclose(Ic, Ic.T, atol=1e-15)), pd=bool(d.min() > 0),
               triangle_margin=triangle_margin(Ic), pseudo_pd=bool(m > 0 and is_pd(pseudo_inertia(m, c, Ic))))
    rep['triangle'] = bool(rep['triangle_margin'] >= 0)
    if geoms and m > 0:
        lo, hi = aabb(geoms)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        rep['gyration_radius_m'] = float(np.sqrt(d.max() / m))
        rep['extent_from_com_m'] = float(np.linalg.norm(corners - c, axis=1).max())
        rep['gyration_over_extent'] = rep['gyration_radius_m'] / rep['extent_from_com_m']
    return rep


def mirror_error(ca, Ia, cb, Ib):
    """(|c_b - S c_a|, |I_b - S I_a S|_F / |I_a|_F): zero when body b is the mirror image of body a under S = diag(1, -1, 1)."""
    return float(np.linalg.norm(cb - S_MIRROR @ ca)), float(np.linalg.norm(Ib - S_MIRROR @ Ia @ S_MIRROR) / np.linalg.norm(Ia))


# ---- gravity regressor and identification ------------------------------------------------------------------------------------

def params(masses, coms):
    """pi = [m_0, m_0 c_0, m_1, m_1 c_1, ...] (4 per body, centre of mass in the body frame)."""
    return np.concatenate([[mi, *(mi * np.asarray(ci))] for mi, ci in zip(masses, coms)])


def gravity_regressor(robot, q, g):
    """Y (12 x 4 nb) with tau = Y pi the joint torque that holds the robot still at joint angles q under gravity g (world), base
    fixed at the identity pose. From V = -sum_i g . (m_i p_i + R_i h_i) and tau_k = dV/dq_k, with d p/dq_k = s_k x (p - a_k):
        Y[k, m_i]   = -g . ( s_k x (p_i - a_k) ),     Y[k, h_i,e] = -g . ( s_k x R_i e_e )
    for every body i at or below hinge k (s_k, a_k: world axis and anchor), zero otherwise."""
    qpos = np.r_[0, 0, 0, 1, 0, 0, 0, q]
    R, p, anchor, axis = A.fk(robot, qpos)
    nb = len(robot.bodies)
    hinges = [k for k, j in enumerate(robot.joints) if j.type == 'hinge']
    Y = np.zeros((len(hinges), 4 * nb))
    for r, k in enumerate(hinges):
        jb = robot.joints[k].body
        for i in range(nb):
            a = i
            while a >= 0 and a != jb:
                a = robot.bodies[a].parent
            if a != jb:
                continue                                   # body i is not moved by hinge k
            s = axis[k]
            Y[r, 4 * i] = -g @ np.cross(s, p[i] - anchor[k])
            Y[r, 4 * i + 1:4 * i + 4] = [-g @ np.cross(s, R[i][:, e]) for e in range(3)]
    return Y


def svd_rank(Y, rtol=1e-9):
    """Singular values of Y (column-scaled to unit norm, so units do not decide the rank) and the numerical rank."""
    n = np.linalg.norm(Y, axis=0)
    sv = np.linalg.svd(Y[:, n > 0] / n[n > 0], compute_uv=False)
    return sv, int((sv > rtol * sv[0]).sum()), int((n == 0).sum())


def lstsq_qr(Y, b):
    """x = argmin |Y x - b| via Y = Q R, x = R^-1 Q^T b. Returns x, the residual RMS and cov = s^2 (R^T R)^-1 with
    s^2 = |r|^2 / (n - p), the parameter covariance if the residuals were independent with equal variance."""
    Q, Rr = np.linalg.qr(Y)
    x = np.linalg.solve(Rr, Q.T @ b)
    r = b - Y @ x
    n, p = Y.shape
    s2 = r @ r / max(n - p, 1)
    Ri = np.linalg.inv(Rr)
    return x, float(np.sqrt(np.mean(r ** 2))), s2 * Ri @ Ri.T


def identify_body(Ys, taus, pi_known, cols):
    """Unknown parameters pi[cols] from stacked torques: (sum_n Y_n) pi = tau, known columns moved to the right-hand side:
    Y_u pi_u = tau - Y_k pi_k. Returns (estimate, residual RMS, covariance)."""
    Y = np.vstack(Ys); tau = np.concatenate(taus)
    known = np.setdiff1d(np.arange(Y.shape[1]), cols)
    return lstsq_qr(Y[:, cols], tau - Y[:, known] @ pi_known[known])
