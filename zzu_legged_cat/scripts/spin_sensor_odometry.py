"""Sensor-only planar odometry with directional wheel measurements."""

import numpy as np
from spin_anchor_env import SpinOdometry, heading_rotate
from spin_rl_env import attitude


def center_acceleration(q, gyro, acc, alpha):
    offset = np.array([0, -.020, 0])
    center = acc-np.cross(alpha, offset)-np.cross(gyro, np.cross(gyro, offset))
    roll, pitch, yaw, _, _ = attitude(q, gyro)
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    horizontal = np.column_stack((cp*center[:, 0]+sp*(sr*center[:, 1]+cr*center[:, 2]),
                                  cr*center[:, 1]-sr*center[:, 2]))
    return heading_rotate(horizontal, yaw)


def wheel_kinematics(q, gyro, native, lateral):
    roll, pitch, yaw, _, _ = attitude(q, gyro)
    cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    gx, gy, gz = gyro.T
    lengths, swings = native[:, 11:13], native[:, 15:17]
    length_rates, swing_rates = native[:, 13:15], native[:, 17:19]
    rotation = (cp[:, None]*(-gy[:, None]*lengths-gz[:, None]*lateral)
                + (sp*sr)[:, None]*(gz[:, None]*swings+gx[:, None]*lengths)
                + (sp*cr)[:, None]*(gx[:, None]*lateral-gy[:, None]*swings))
    forward = (-.043*(native[:, 9:11]-gy[:, None])-cp[:, None]*swing_rates
               + (sp*cr)[:, None]*length_rates-rotation)
    sideways = -(cr[:, None]*(gz[:, None]*swings+gx[:, None]*lengths)
                 + sr[:, None]*(gy[:, None]*swings+length_rates))
    return forward, sideways, yaw


class DirectionalSpinOdometry(SpinOdometry):
    """Wheel rolling speed constrains forward motion, not lateral sliding.

    The lateral pseudo-measurement is optional for controlled comparisons.
    With it disabled, rotating forward measurements and the IMU carry the
    planar estimate; constant unobserved sideways motion is not identifiable
    while the heading remains fixed.
    """

    def __init__(self, count, lateral, forward_tau=.015, sideways_tau=np.inf):
        super().__init__(count, lateral)
        self.tau = np.array([forward_tau, sideways_tau])
        self.forward = np.zeros((count, 2))
        self.sideways = np.zeros((count, 2))
        self.weights = np.ones((count, 2))

    def step(self, q, gyro, acc, native, dt):
        alpha = np.where(self.ready[:, None], (gyro-self.previous_gyro)/dt, 0)
        self.previous_gyro[:] = gyro
        acceleration = center_acceleration(q, gyro, acc, alpha)
        forward, sideways, yaw = wheel_kinematics(q, gyro, native, self.lateral)
        self.forward[:], self.sideways[:] = forward, sideways
        self.weights[:] = 1/(1+(native[:, 20:22]/.12)**2)
        mean_forward = np.sum(self.weights*forward, axis=1)/self.weights.sum(axis=1)
        measured = np.column_stack((mean_forward, sideways.mean(axis=1)))
        old = self.velocity.copy()
        predicted = heading_rotate(self.velocity+acceleration*dt, -yaw)
        trust = np.maximum(self.weights.mean(axis=1), .3)
        gain = dt/(self.tau[None]/trust[:, None]+dt)
        corrected = predicted+gain*(measured-predicted)
        self.velocity[:] = heading_rotate(corrected, yaw)
        initial = measured.copy()
        if np.isinf(self.tau[1]):
            initial[:, 1] = 0
        self.velocity[~self.ready] = heading_rotate(initial, yaw)[~self.ready]
        self.velocity[:] = np.clip(self.velocity, -5, 5)
        self.position += .5*(old+self.velocity)*dt
        self.ready[:] = True


class InnovationSpinOdometry(SpinOdometry):
    """Per-wheel innovation gating against a continuous inertial reference.

    Agreement permits slow inertial re-anchoring. A wheel pair can be rejected
    together; their mutual agreement alone is not evidence of ground speed.
    Confidence expresses consistency, not an identified friction coefficient.
    """

    def __init__(self, count, lateral, directional_reference=False):
        super().__init__(count, lateral)
        self.directional_reference = directional_reference
        self.inertial_velocity = np.zeros((count, 2))
        self.wheel_confidence = np.zeros((count, 2))
        self.sideways_confidence = np.zeros(count)
        self.innovation = np.zeros((count, 2))

    def reset(self, ids):
        super().reset(ids)
        self.inertial_velocity[ids] = 0
        self.wheel_confidence[ids] = self.sideways_confidence[ids] = 0
        self.innovation[ids] = 0

    def initialize_velocity(self, velocity, gyro, ids=None):
        ids = slice(None) if ids is None else ids
        self.velocity[ids] = self.inertial_velocity[ids] = velocity
        self.previous_gyro[ids] = gyro
        self.ready[ids] = True

    def step(self, q, gyro, acc, native, dt):
        alpha = np.where(self.ready[:, None], (gyro-self.previous_gyro)/dt, 0)
        self.previous_gyro[:] = gyro
        acceleration = center_acceleration(q, gyro, acc, alpha)
        forward, sideways, yaw = wheel_kinematics(q, gyro, native, self.lateral)
        old = self.velocity.copy()
        predicted = self.velocity+acceleration*dt
        self.inertial_velocity += acceleration*dt
        missing = ~self.ready
        initial = heading_rotate(np.column_stack((forward.mean(axis=1), sideways.mean(axis=1))), yaw)
        predicted[missing] = self.inertial_velocity[missing] = initial[missing]
        reference = heading_rotate(self.inertial_velocity, -yaw)
        self.innovation[:] = forward-reference[:, :1]
        self.wheel_confidence[:] = np.exp(-.5*np.minimum(np.abs(self.innovation)/.060, 12)**2)
        side_measurement = sideways.mean(axis=1)
        self.sideways_confidence[:] = np.exp(-.5*np.minimum(
            np.abs(side_measurement-reference[:, 1])/.030, 12)**2)
        mean_forward = np.sum(self.wheel_confidence*forward, axis=1)/np.maximum(
            self.wheel_confidence.sum(axis=1), 1e-12)
        measured = np.column_stack((mean_forward, side_measurement))
        confidence = np.column_stack((self.wheel_confidence.max(axis=1), self.sideways_confidence))
        heading_prediction = heading_rotate(predicted, -yaw)
        # Earlier wheel corrections must not survive indefinitely once their
        # measurement is rejected. Return to the independent inertial track.
        heading_prediction += dt/(.020+dt)*(1-confidence)*(reference-heading_prediction)
        corrected = heading_prediction+dt/(.015+dt)*confidence*(measured-heading_prediction)
        self.velocity[:] = np.clip(heading_rotate(corrected, yaw), -5, 5)
        if self.directional_reference:
            # Each reliable direction remains useful when another wheel or the
            # lateral prior is rejected. Requiring all three loses observability
            # during single-wheel slip and lets small IMU errors lock out good data.
            trusted = confidence > .7
            reference += dt/(.5+dt)*trusted*(heading_rotate(self.velocity, -yaw)-reference)
            self.inertial_velocity[:] = heading_rotate(reference, yaw)
        else:
            trusted = (self.wheel_confidence.min(axis=1) > .7) & (self.sideways_confidence > .7)
            self.inertial_velocity[trusted] += dt/(.5+dt)*(
                self.velocity-self.inertial_velocity)[trusted]
        self.position += .5*(old+self.velocity)*dt
        self.ready[:] = True
