"""Quaternion wxyz and body x-forward/y-left/z-up calculations."""

import numpy as np
from .robot import IMU_BAKE_Q


def multiply(q, r):
    w, x, y, z = np.asarray(q).T
    rw, rx, ry, rz = r
    return np.column_stack((w*rw-x*rx-y*ry-z*rz, w*rx+x*rw+y*rz-z*ry,
                            w*ry-x*rz+y*rw+z*rx, w*rz+x*ry-y*rx+z*rw))


def aligned_control_attitude(raw_quaternion, extrinsic):
    site = multiply(raw_quaternion, extrinsic)
    return multiply(site, IMU_BAKE_Q*np.array([1, -1, -1, -1]))


def projected_up(q):
    """World +Z in body axes; upright is [0, 0, +1], not gravity -Z."""
    w, x, y, z = np.asarray(q).T
    return np.column_stack((2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))


def euler_rates(roll, pitch, gyro):
    sr, cr = np.sin(roll), np.cos(roll)
    cp = np.maximum(np.cos(pitch), 0.2)
    cross = sr*gyro[:, 1]+cr*gyro[:, 2]
    return np.column_stack((gyro[:, 0]+np.sin(pitch)/cp*cross,
                            cr*gyro[:, 1]-sr*gyro[:, 2], cross/cp))


def attitude(q, gyro):
    """ZYX angles/rates; retain the legacy near-pitch-limit clamp."""
    w, x, y, z = q.T
    pitch = np.arcsin(np.clip(2*(w*y-x*z), -1, 1))
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    rates = euler_rates(roll, pitch, gyro)
    return roll, pitch, yaw, rates[:, 0], rates[:, 2]
