"""Fixed-origin reward, sensor odometry and V5 actor migration regressions."""

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from eval_spin_rl import summary
from spin_anchor_env import AnchorSpinEnv, SpinOdometry, anchor_reward_terms
from stable_baselines3 import PPO
from train_spin_rl import warm_start_actor

def test_stationary_outside_remains_costly_and_inward_motion_is_preferred():
    radius = np.array([0, 0.15, 0.3, 0.6, 0.9])
    delta = np.column_stack((radius, np.zeros(5)))
    terms = anchor_reward_terms(delta, np.zeros((5, 2)), np.zeros(5))
    cost = sum(terms.values())
    assert (np.diff(cost) < 0).all()
    assert cost[3] == pytest.approx(-28)
    delta = np.tile([0.6, 0], (3, 1))
    motion = np.array([[-0.2, 0], [0, 0], [0.2, 0]])
    cost = sum(anchor_reward_terms(delta, motion, np.zeros(3)).values())
    assert cost[0] > cost[1] > cost[2]
    assert cost[0] < 0
    late = anchor_reward_terms(delta, np.zeros((3, 2)), np.full(3, 5))
    early = anchor_reward_terms(delta, np.zeros((3, 2)), np.zeros(3))
    assert (sum(late.values()) < sum(early.values())).all()


def test_spin_odometry_subtracts_leg_rotation_and_imu_centripetal_acceleration():
    odom = SpinOdometry(1)
    native = np.zeros((1, 28))
    native[:, 11:13] = 0.117
    omega, dt = 2.76, 0.02
    native[:, 9:11] = omega*np.array([-0.1361375, 0.1361375])/0.043
    gyro = np.array([[0, 0, omega]])
    acc = np.array([[0, 0.020*omega**2, 9.81]])
    for i in range(1500):
        angle = i*dt*omega
        q = np.array([[np.cos(angle/2), 0, 0, np.sin(angle/2)]])
        odom.step(q, gyro, acc, native, dt)
    np.testing.assert_allclose(odom.position, 0, atol=1e-12)
    odom.position[:] = [0.3, 0]
    np.testing.assert_allclose(odom.features(np.array([np.pi/2]))[:, :2], [[0, -1]], atol=1e-12)


@pytest.fixture
def env():
    environment = AnchorSpinEnv(1, 1, cases=[(3, 0.7, 0, 1)], terrain_cases=[{
        "location": "ramp", "handover": "lqr", "offset": 0,
        "takeover_time": 0, "fade": 0.3, "dropout": False}], disturbances=False)
    environment.seed(1000)
    environment.reset()
    try:
        yield environment
    finally:
        environment.close()


def test_new_observation_has_direction_but_no_simulator_position_or_contacts(env):
    frame = env._frame_obs().copy()
    assert frame.shape == (1, 56)
    env.qpos[:, :2] += 0.5
    env.qvel[:, :2] += 1
    env.start_xy[:] = 100
    env.contact_values[:] = 100
    env.standard_requested[:] = -100
    np.testing.assert_array_equal(env._frame_obs(), frame)
    env.odometry.position[:] = [0.3, 0]
    assert np.linalg.norm(env._frame_obs()[0, -4:-2]) == pytest.approx(1)


def test_actor_warm_start_preserves_v5_actions_and_resets_critic(env, migration_checkpoint):
    torch.set_num_threads(1)
    old, source = migration_checkpoint('MlpPolicy', 52*8)
    new = PPO("MlpPolicy", env, n_steps=8, batch_size=8,
              policy_kwargs={"net_arch": [256, 256]}, device="cpu")
    critic_before = new.policy.value_net.weight.detach().clone()
    warm_start_actor(new, source)
    frame = np.random.default_rng(1).normal(size=(4, 8, 56)).astype(np.float32)
    old_obs = frame[:, :, :52].reshape(4, -1)
    np.testing.assert_allclose(new.predict(frame.reshape(4, -1), deterministic=True)[0],
                               old.predict(old_obs, deterministic=True)[0], atol=1e-6)
    torch.testing.assert_close(new.policy.value_net.weight, critic_before)


def test_return_requires_reentry_dwell_and_persistent_escape_is_failure(env):
    args = [np.zeros(1) for _ in range(10)]
    args[7] = args[8] = np.zeros((1, 5))
    env.qpos[:, :2] = env.start_xy+[0.4, 0]
    env._reward(*args)
    assert env.return_attempts[0] == 1
    assert env.excursion_active[0]
    env.qpos[:, :2] = env.start_xy+[0.1, 0]
    for _ in range(498):
        env._reward(*args)
    assert env.return_successes[0] == 0
    env._reward(*args)
    env._reward(*args)
    assert env.return_successes[0] == 1
    assert not env.excursion_active[0]
    env.excursion_age[:] = 6.1
    assert env._failure_reasons(np.array([True]), np.zeros(1), np.zeros(1), np.array([0.4]))[0] & 8
    env.excursion_age[:] = 0
    assert env._failure_reasons(np.array([True]), np.zeros(1), np.zeros(1), np.array([1.01]))[0] & 8


def test_terminating_trial_reports_anchor_metrics_and_summary(env):
    env.episode_steps = 3
    for _ in range(3):
        obs, reward, done, infos = env.step(np.zeros((1, 5), np.float32))
        assert obs.shape == (1, 448)
        assert np.isfinite(obs).all() and np.isfinite(reward).all()
    assert done[0]
    metrics = {**infos[0]["spin_metrics"], "return": infos[0]["episode"]["r"]}
    stats = summary([metrics])
    assert "bounded_spin_passed" in stats
    assert "mean_final_drift_m" in stats["by_leg_band"]["middle"]
    assert "odometry_peak_error_m" in metrics
