"""Encoder-only virtual-work mapping through both closed-chain legs.

Body coordinates are x forward, y left, z up. Leg inputs are forces on
the body, not forces on the axle. Wheel inputs are physical motor torques.
The support-rotation Jacobian includes wheel motor reaction on the linkage.
This is a kinematic force allocation, not a contact/friction dynamics model.
"""

import numpy as np
from .robot import TORQUE_LIMIT_NM

ACTION_NAMES = ('left_body_fx', 'left_body_fz', 'right_body_fx', 'right_body_fz',
                'forward_wheel_torque', 'yaw_differential_wheel_torque')
ACTION_SCALE = np.array([30., 40., 30., 40., 1., 1.])
ACTUATOR_ORDER = ('left_front', 'left_rear', 'left_wheel',
                  'right_front', 'right_rear', 'right_wheel')
TORQUE_LIMIT = TORQUE_LIMIT_NM


class WrenchTorqueMapper:
    def __init__(self, geometry):
        self.geometry = geometry

    def jacobians(self, positions):
        positions = np.asarray(positions)
        q = np.broadcast_to(positions[..., None, :], (*positions.shape[:-1], 2, 4))
        dq = np.broadcast_to([[1., 0., 1., 0.], [0., 1., 0., 1.]], q.shape)
        wheel, velocity, spin, valid = self.geometry.calculate(q, dq)
        jacobian = velocity.swapaxes(-3, -2).swapaxes(-2, -1)
        return wheel[..., 0, :, :], jacobian, spin.swapaxes(-2, -1), valid.all(axis=-2)

    def map(self, positions, commands):
        """Return joint Nm, point-force body wrench, validity.

        tau_leg = -J_axle.T F_body + J_support.T tau_wheel follows from
        omega_wheel = qdot_wheel + J_support qdot_leg. No inverse Jacobian
        is needed, including near a folded-leg singularity.
        """
        commands = np.asarray(commands)
        if commands.shape != (*np.asarray(positions).shape[:-1], 6) or not np.isfinite(commands).all():
            raise ValueError('Expected six finite virtual force/torque commands')
        wheel, jacobian, support, valid = self.jacobians(positions)
        force = commands[..., :4].reshape(*commands.shape[:-1], 2, 2)
        wheel_tau = np.stack((-commands[..., 4]-commands[..., 5],
                              -commands[..., 4]+commands[..., 5]), axis=-1)
        wheel_tau = np.clip(wheel_tau, -10, 10)
        legs = (-np.einsum('...scj,...sc->...sj', jacobian, force)
                + support*wheel_tau[..., :, None])
        legs = np.where(valid[..., :, None], legs, 0)
        torque = np.concatenate((legs[..., 0, :], wheel_tau[..., :1],
                                 legs[..., 1, :], wheel_tau[..., 1:]), axis=-1)
        torque = np.clip(torque, -TORQUE_LIMIT, TORQUE_LIMIT)
        return torque, (wheel, force), valid

    @staticmethod
    def point_force_wrench(wheel, force, lateral):
        """Quasistatic force and moment about the base origin; no ground truth."""
        arm = np.stack((wheel[..., 0], np.broadcast_to(lateral, wheel.shape[:-1]),
                        wheel[..., 1]), axis=-1)
        body_force = np.stack((force[..., 0], np.zeros_like(force[..., 0]), force[..., 1]), axis=-1)
        return np.concatenate((body_force.sum(axis=-2), np.cross(arm, body_force).sum(axis=-2)), axis=-1)

    def equivalent_body_wrench(self, positions, torque, lateral):
        """Recover virtual forces for diagnostics; invalid/singular cases are flagged."""
        wheel, jac, support, valid = self.jacobians(positions)
        legs = np.asarray(torque)[..., [0, 1, 3, 4]].reshape(*jac.shape[:-2], 2)
        wheel_tau = np.asarray(torque)[..., [2, 5]]
        effective = legs-support*wheel_tau[..., :, None]
        jt = jac.swapaxes(-1, -2)
        condition = np.linalg.cond(jt)
        valid &= np.isfinite(condition) & (condition < 1e4)
        force = -np.einsum('...sij,...sj->...si', np.linalg.pinv(jt, rcond=1e-4), effective)
        return self.point_force_wrench(wheel, force, lateral), force, valid, condition
