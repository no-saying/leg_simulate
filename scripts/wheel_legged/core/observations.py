"""Compact observations, independent of simulator storage and truth state."""

import numpy as np

FRAME_SIZE = 25
OBS_SCALE = np.array([5]*3+[1]*3+[1.2, 3.5, .2]+[1]*4+[10]*4+[30]*2+[1]*6,
                     dtype=np.float32)


def compact_frame(gyro, up, commands, leg_angles, joint_rates, previous_action):
    """Preserve the legacy compact contract, including raw leg joint angles.

    Commands: forward m/s, yaw rad/s, requested leg length m.
    Joint rates: LF, LR, RF, RR, LW, RW (not motor output order).
    Action semantics are supplied by the caller's policy contract.
    """
    groups = (gyro, up, commands, leg_angles, joint_rates, previous_action)
    arrays = [np.asarray(value) for value in groups]
    n = arrays[0].shape[0]
    if any(value.shape != (n, size) for value, size in zip(arrays, (3, 3, 3, 4, 6, 6))):
        raise ValueError('Expected batched gyro/up/commands/angles/rates/actions')
    frame = np.column_stack(arrays)
    if not np.isfinite(frame).all():
        raise FloatingPointError('Nonfinite sensor observation')
    return np.clip(frame/OBS_SCALE, -20, 20).astype(np.float32)


class ObservationHistory:
    """Explicit total frame count; reset only the selected environments."""

    def __init__(self, num_envs, frames=5, frame_size=FRAME_SIZE):
        if num_envs < 1 or frames < 1 or frame_size < 1:
            raise ValueError('History dimensions must be positive')
        self.values = np.zeros((num_envs, frames, frame_size), np.float32)

    def reset(self, ids, frame):
        self.values[ids] = np.asarray(frame, np.float32)[:, None, :]

    def append(self, frame):
        self.values[:, :-1] = self.values[:, 1:]
        self.values[:, -1] = frame

    def observation(self):
        return self.values.reshape(len(self.values), -1).copy()
