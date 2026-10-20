"""Exercise the physical-unit interface; this is not a balance evaluation."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from wheel_legged.backends.mujoco import MujocoBackend
from wheel_legged.core.actuation import JointPDGains
from wheel_legged.core.frames import projected_up
from wheel_legged.core.observations import compact_frame, ObservationHistory
from wheel_legged.core.robot import LEG_CHANNELS, WHEEL_CHANNELS
from wheel_legged.runtime import JointServoRuntime


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--model', type=Path,
        default=root/'model/v3/COD-2026RoboMaster-Balance-v3.xml')
    parser.add_argument('--gains', type=Path, default=root/'config/rl_joint_pd.json')
    parser.add_argument('--envs', type=int, default=2)
    parser.add_argument('--ticks', type=int, default=100)
    args = parser.parse_args()
    if args.ticks < 1:
        parser.error('--ticks must be positive')
    backend = MujocoBackend.from_xml(args.model, num_envs=args.envs)
    try:
        runtime = JointServoRuntime(backend, JointPDGains.load(args.gains))
        initial = backend.read_sensors()
        leg_target = initial.joint_position[:, LEG_CHANNELS].copy()
        wheel_target = np.zeros((args.envs, 2))
        previous_action = np.zeros((args.envs, 6))
        commands = np.tile([0, 0, .117], (args.envs, 1))
        history = ObservationHistory(args.envs, frames=5)

        def frame():
            sensors = backend.read_sensors()
            rates = sensors.joint_velocity[:, np.r_[LEG_CHANNELS, WHEEL_CHANNELS]]
            return compact_frame(sensors.body_angular_velocity,
                projected_up(sensors.body_quaternion), commands,
                sensors.joint_position[:, LEG_CHANNELS], rates, previous_action)

        history.reset(np.arange(args.envs), frame())
        peak = 0.
        for tick in range(args.ticks):
            torque = runtime.tick(leg_target, wheel_target)
            peak = max(peak, float(np.max(np.abs(torque))))
            if (tick+1) % 20 == 0:
                history.append(frame())
        truth = backend.read_truth()
        finite = bool(np.isfinite(history.observation()).all()
                      and np.isfinite(truth.world_position).all())
        print(json.dumps(dict(backend='mujoco', ticks=args.ticks, dt=backend.dt,
            simulated_seconds=args.ticks*backend.dt, sensor_history_shape=list(history.observation().shape),
            peak_joint_torque_nm=peak, finite=finite,
            native_controller_used=False, policy_or_training_used=False,
            scope='backend smoke test, not whole-robot balance acceptance')))
        if not finite:
            raise RuntimeError('Nonfinite backend rollout')
    finally:
        backend.close()


if __name__ == '__main__':
    main()
