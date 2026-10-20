"""Engine-independent PD execution, with no policy or native controller mix."""

import numpy as np

from .core.actuation import JointPDGains, JointTargetPD
from .interfaces import PhysicsBackend


class JointServoRuntime:
    """Call once per servo tick, holding physical targets across policy ticks."""

    def __init__(self, backend: PhysicsBackend, gains: JointPDGains):
        self.backend, self.gains = backend, gains
        self.servo = JointTargetPD()
        self.previous_torque = np.zeros((backend.num_envs, 6))

    def reset(self, ids=None):
        self.backend.reset(ids)
        if ids is None:
            self.previous_torque[:] = 0
        else:
            self.previous_torque[ids] = 0

    def tick(self, leg_position_target, wheel_velocity_target):
        leg = np.asarray(leg_position_target)
        wheel = np.asarray(wheel_velocity_target)
        if leg.shape != (self.backend.num_envs, 4) or wheel.shape != (self.backend.num_envs, 2):
            raise ValueError('Expected (N,4) leg radians and (N,2) wheel rad/s')
        sensors = self.backend.read_sensors()
        torque, _ = self.servo.control(sensors.joint_position, sensors.joint_velocity,
            leg, wheel, self.gains, self.previous_torque, self.backend.dt)
        self.backend.apply_joint_torque(torque)
        self.backend.step()
        self.previous_torque[:] = torque
        return torque.copy()
