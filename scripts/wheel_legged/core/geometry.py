"""Planar closed-chain encoder kinematics, including wheel support rotation.

Construction takes calibration data; runtime uses only four joint encoders.
"""

import numpy as np


def cross(a, b):
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def perpendicular(v):
    return np.stack((-v[..., 1], v[..., 0]), axis=-1)


def rotate(v, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.stack((c*v[..., 0]-s*v[..., 1], s*v[..., 0]+c*v[..., 1]), axis=-1)


def intersection(a, c, radius_a, radius_c, branch):
    delta = c-a
    distance = np.maximum(np.linalg.norm(delta, axis=-1), 1e-9)
    axis = delta/distance[..., None]
    along = (radius_a**2-radius_c**2+distance**2)/(2*distance)
    height_sq = radius_a**2-along**2
    b = a+along[..., None]*axis+branch*np.sqrt(np.maximum(height_sq, 0))[..., None]*perpendicular(axis)
    return b, (height_sq > 1e-9) & (distance > 1e-5)


def intersection_velocity(b, a, c, va, vc):
    u, v = b-a, b-c
    rhs_a, rhs_c = np.sum(u*va, axis=-1), np.sum(v*vc, axis=-1)
    determinant = cross(u, v)
    safe = np.where(np.abs(determinant) > 1e-8, determinant,
                    np.where(determinant < 0, -1e-8, 1e-8))
    velocity = np.stack(((rhs_a*v[..., 1]-u[..., 1]*rhs_c)/safe,
                          (u[..., 0]*rhs_c-rhs_a*v[..., 0])/safe), axis=-1)
    return velocity, np.abs(determinant) > 1e-6


class EncoderLegGeometry:
    def __init__(self, sides):
        if len(sides) != 2:
            raise ValueError('Expected left and right linkage calibration')
        self.sides = []
        for side in sides:
            result = {}
            for key in ('front', 'rear', 'arm_a', 'arm_c', 'arm_f', 'bc', 'bd', 'ef', 'ew'):
                value = np.array(side[key], dtype=float, copy=True)
                if value.shape != (2,) or not np.isfinite(value).all():
                    raise ValueError(f'Invalid linkage vector {key}')
                result[key] = value
            for key in ('ab', 'de', 'branch_b', 'branch_e', 'sign'):
                value = float(side[key])
                if not np.isfinite(value) or (value <= 0 if key in ('ab', 'de') else abs(value) != 1):
                    raise ValueError(f'Invalid linkage scalar {key}')
                result[key] = value
            self.sides.append(result)

    def calibration(self):
        return [{key: value.tolist() if isinstance(value, np.ndarray) else value
                 for key, value in side.items()} for side in self.sides]

    def calculate(self, joint_position, joint_velocity):
        joint_position, joint_velocity = np.asarray(joint_position), np.asarray(joint_velocity)
        if joint_position.shape != joint_velocity.shape or joint_position.shape[-1] != 4:
            raise ValueError('Expected four driven joint positions and velocities')
        output = []
        for side, g in enumerate(self.sides):
            q = g['sign']*joint_position[..., 2*side:2*side+2]
            dq = g['sign']*joint_velocity[..., 2*side:2*side+2]
            a = g['front']+rotate(g['arm_a'], q[..., 0])
            c = g['rear']+rotate(g['arm_c'], q[..., 1])
            f = g['rear']+rotate(g['arm_f'], q[..., 1])
            va = dq[..., :1]*perpendicular(a-g['front'])
            vc = dq[..., 1:]*perpendicular(c-g['rear'])
            vf = dq[..., 1:]*perpendicular(f-g['rear'])
            b, valid_b = intersection(a, c, g['ab'], np.linalg.norm(g['bc']), g['branch_b'])
            vb, valid_vb = intersection_velocity(b, a, c, va, vc)
            bc = c-b
            angle_b = np.arctan2(bc[..., 1], bc[..., 0])-np.arctan2(g['bc'][1], g['bc'][0])
            omega_b = cross(bc, vc-vb)/np.sum(bc**2, axis=-1)
            d = b+rotate(g['bd'], angle_b)
            vd = vb+omega_b[..., None]*perpendicular(d-b)
            e, valid_e = intersection(d, f, g['de'], np.linalg.norm(g['ef']), g['branch_e'])
            ve, valid_ve = intersection_velocity(e, d, f, vd, vf)
            ef = f-e
            angle_e = np.arctan2(ef[..., 1], ef[..., 0])-np.arctan2(g['ef'][1], g['ef'][0])
            omega_e = cross(ef, vf-ve)/np.sum(ef**2, axis=-1)
            wheel = e+rotate(g['ew'], angle_e)
            vw = ve+omega_e[..., None]*perpendicular(wheel-e)
            valid = valid_b & valid_e & valid_vb & valid_ve
            output.append((wheel, vw, omega_e, valid))
        return (np.stack([v[0] for v in output], axis=-2),
                np.stack([v[1] for v in output], axis=-2),
                np.stack([v[2] for v in output], axis=-1),
                np.stack([v[3] for v in output], axis=-1))

    def observation(self, native, joint_position, joint_velocity, exact_position=True):
        wheel, velocity, support_rate, valid = self.calculate(joint_position, joint_velocity)
        corrected = native.copy()
        corrected[:, 9:11] += np.where(valid, support_rate, 0)
        if exact_position:
            for columns, values in ((slice(11, 13), -wheel[:, :, 1]),
                                    (slice(13, 15), -velocity[:, :, 1]),
                                    (slice(15, 17), wheel[:, :, 0]),
                                    (slice(17, 19), velocity[:, :, 0])):
                corrected[:, columns] = np.where(valid, values, corrected[:, columns])
        return corrected, valid
