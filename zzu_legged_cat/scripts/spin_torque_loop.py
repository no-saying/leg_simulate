"""Six independent motor torque PI loops and an optional first-order plant.

Feedback is measured motor torque in joint Nm, never ground contact force.
The plant parameters below are simulation assumptions, not identified motors.
"""

import numpy as np


class TorquePI:
    def __init__(self, count, limits, kp=.4, ki=60.):
        self.limits = np.asarray(limits, dtype=float)
        self.kp, self.ki = kp, ki
        self.integral = np.zeros((count, len(limits)))

    def reset(self, ids):
        self.integral[ids] = 0

    def step(self, target, measured, dt):
        target = np.clip(target, -self.limits, self.limits)
        error = target-measured
        proposed = self.integral+self.ki*error*dt
        raw = target+self.kp*error+proposed
        # Conditional integration allows unwinding while blocking saturation windup.
        unwind = ((raw > self.limits) & (error < 0)) | ((raw < -self.limits) & (error > 0))
        accept = (np.abs(raw) <= self.limits) | unwind
        self.integral[:] = np.clip(np.where(accept, proposed, self.integral), -self.limits, self.limits)
        return np.clip(target+self.kp*error+self.integral, -self.limits, self.limits)


class SimulatedTorqueMotors:
    def __init__(self, count, limits, time_constant=.008):
        self.limits = np.asarray(limits, dtype=float)
        self.time_constant = time_constant
        self.torque = np.zeros((count, len(limits)))
        self.gain = np.ones_like(self.torque)
        self.bias = np.zeros_like(self.torque)

    def reset(self, ids, measured, gain):
        self.torque[ids] = measured
        self.gain[ids] = gain
        self.bias[ids] = 0

    def step(self, command, dt):
        equilibrium = np.clip(self.gain*command+self.bias, -self.limits, self.limits)
        self.torque += (1-np.exp(-dt/self.time_constant))*(equilibrium-self.torque)
        return self.torque.copy()
