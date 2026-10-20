"""Six actuator target tracking in physical joint torque units."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from .robot import LEG_CHANNELS, WHEEL_CHANNELS, TORQUE_LIMIT_NM, TORQUE_RATE_NM_S


@dataclass(frozen=True)
class JointPDGains:
    leg_kp: float
    leg_kd: float
    wheel_kv: float

    def __post_init__(self):
        values = np.array([self.leg_kp, self.leg_kd, self.wheel_kv])
        if not np.isfinite(values).all() or np.any(values <= 0):
            raise ValueError('Expected positive finite physical PD gains')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text())['gains'])


class JointTargetPD:
    """Leg position PD and wheel velocity P; no torque-error integral loop."""

    @staticmethod
    def requested_torque(position, velocity, position_target, wheel_speed_target, gains):
        position, velocity = np.asarray(position), np.asarray(velocity)
        torque = np.zeros_like(velocity, dtype=float)
        kp = np.asarray(gains.leg_kp)
        kd = np.asarray(gains.leg_kd)
        kv = np.asarray(gains.wheel_kv)
        torque[..., LEG_CHANNELS] = (kp[..., None]*(position_target-position[..., LEG_CHANNELS])
                                     - kd[..., None]*velocity[..., LEG_CHANNELS])
        torque[..., WHEEL_CHANNELS] = kv[..., None]*(wheel_speed_target-velocity[..., WHEEL_CHANNELS])
        return torque

    def control(self, position, velocity, position_target, wheel_speed_target, gains,
                previous_torque, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Expected positive finite controller timestep')
        requested = self.requested_torque(position, velocity, position_target, wheel_speed_target, gains)
        if not np.isfinite(requested).all():
            raise FloatingPointError('Nonfinite PD torque request')
        torque = limit_torque(requested, previous_torque, dt)
        return torque, requested


def limit_torque(requested, previous_torque, dt):
    """Limit physical Nm, independently of engine control gain conventions."""
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('Expected positive finite controller timestep')
    previous_torque = np.asarray(previous_torque)
    if not np.isfinite(previous_torque).all():
        raise FloatingPointError('Nonfinite previous torque')
    limited = np.clip(requested, -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM)
    return previous_torque+np.clip(limited-previous_torque,
                                  -TORQUE_RATE_NM_S*dt, TORQUE_RATE_NM_S*dt)
