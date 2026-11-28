"""URDF parsing and forward kinematics with numpy only (no ROS), used by run.py and the tests.
    u = Urdf(xml_text); T = u.fk({'lf_hip_joint': 0.3}, 'lf_foot_link')   # 4x4 pose of the link in the root frame"""
import xml.etree.ElementTree as ET

import numpy as np


def _vec(s, n=3):
    return np.array([float(v) for v in s.split()]) if s else np.zeros(n)


def rpy_matrix(r, p, y):
    """URDF fixed-axis roll, pitch, yaw: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, q):
    """Rodrigues rotation of angle q about the unit vector axis."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def transform(R=np.eye(3), t=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def quat_matrix(x, y, z, w):
    """Rotation matrix of a unit quaternion (x, y, z, w), the order tf2 uses."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotation_angle(R):
    """Angle of the rotation R in [0, pi]."""
    return float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


class Urdf:
    def __init__(self, text):
        self.root_el = ET.fromstring(text)
        self.links = [l.get('name') for l in self.root_el.findall('link')]
        self.joints = {}
        for j in self.root_el.findall('joint'):
            o, a, lim = j.find('origin'), j.find('axis'), j.find('limit')
            o = o.attrib if o is not None else {}
            self.joints[j.get('name')] = dict(
                type=j.get('type'), parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                xyz=_vec(o.get('xyz')), rpy=_vec(o.get('rpy')),
                axis=_vec(a.get('xyz')) if a is not None else np.array([1.0, 0, 0]),
                limit={k: float(v) for k, v in lim.attrib.items()} if lim is not None else None)
        self.parent_joint = {j['child']: n for n, j in self.joints.items()}
        self.inertial = {}                                  # link -> (mass, com in the link frame)
        for l in self.root_el.findall('link'):
            i = l.find('inertial')
            if i is not None and i.find('mass') is not None:
                o = i.find('origin')
                self.inertial[l.get('name')] = (float(i.find('mass').get('value')), _vec(o.get('xyz') if o is not None else None))
        roots = [l for l in self.links if l not in self.parent_joint]
        if len(roots) != 1:
            raise ValueError(f'expected one root link, found {roots}')
        self.root = roots[0]

    def actuated(self):
        return [n for n, j in self.joints.items() if j['type'] in ('revolute', 'continuous', 'prismatic')]

    def chain(self, link):
        """Joint names from the root down to link."""
        out = []
        while link != self.root:
            n = self.parent_joint[link]
            out.append(n)
            link = self.joints[n]['parent']
        return out[::-1]

    def fk(self, q, link, base=None):
        """Pose (4x4) of link in base (default: the root) for joint angles q (dict; missing joints are 0)."""
        T = np.eye(4)
        for n in self.chain(link):
            j = self.joints[n]
            T = T @ transform(rpy_matrix(*j['rpy']), j['xyz'])
            if j['type'] in ('revolute', 'continuous'):
                T = T @ transform(axis_angle(j['axis'], q.get(n, 0.0)))
            elif j['type'] == 'prismatic':
                T = T @ transform(t=j['axis'] / np.linalg.norm(j['axis']) * q.get(n, 0.0))
        return T if base is None else np.linalg.inv(self.fk(q, base)) @ T

    def holding_effort(self, q, joint, g=(0, 0, -9.8)):
        """Effort joint must apply to hold q against gravity, root frame fixed with g expressed in it:
        tau = -axis . sum_i (c_i - p) x (m_i g) over every link i below the joint (c_i its centre of mass, p the joint origin)."""
        T = self.fk(q, self.joints[joint]['child'])               # child frame = joint frame after the rotation
        p, axis = T[:3, 3], T[:3, :3] @ (self.joints[joint]['axis'] / np.linalg.norm(self.joints[joint]['axis']))
        tau = 0.0
        for link, (m, com) in self.inertial.items():
            if joint in self.chain(link):
                c = (self.fk(q, link) @ np.append(com, 1))[:3]
                tau += axis @ np.cross(c - p, m * np.asarray(g, float))
        return -tau
