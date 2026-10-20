"""CPU reference contracts: radians, seconds, meters, physical joint Nm.

No qpos/ctrl addresses, engine model objects, terrain truth or contacts enter
SensorState. TruthState is an explicit, separate training/evaluation channel.
All arrays include a leading environment dimension; readers return snapshots.
"""

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class SensorState:
    joint_position: np.ndarray  # (N, 6), canonical ACTUATOR_NAMES order
    joint_velocity: np.ndarray  # (N, 6), canonical ACTUATOR_NAMES order
    body_quaternion: np.ndarray  # (N, 4), world-from-control, wxyz
    body_angular_velocity: np.ndarray  # (N, 3), control x-forward/y-left/z-up
    specific_force: np.ndarray  # (N, 3), calibrated IMU axes; includes support


@dataclass(frozen=True)
class TruthState:
    world_position: np.ndarray  # (N, 3), base origin
    world_linear_velocity: np.ndarray  # (N, 3), base origin, world axes
    joint_torque: np.ndarray  # (N, 6), measured physical motor torque


class PhysicsBackend(Protocol):
    num_envs: int
    dt: float

    def read_sensors(self) -> SensorState: ...

    def read_truth(self) -> TruthState: ...

    def apply_joint_torque(self, torque_nm: np.ndarray) -> None: ...

    def step(self) -> None:
        """Advance one dt; reads afterwards describe the new state."""
        ...

    def reset(self, ids: np.ndarray | None = None) -> None:
        """Reset selected instances, including actuator and solver state."""
        ...

    def close(self) -> None: ...
