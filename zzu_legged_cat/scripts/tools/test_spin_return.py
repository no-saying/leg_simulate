"""Return curriculum rewards, episode transitions and deployment boundary."""

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spin_asymmetric_policy import AsymmetricSpinPolicy, warm_start_history_residual
from spin_return_env import (RETURN_STAGES, ReturnTrainingSpinEnv, return_gate_passes,
                             return_tracking_cost, return_velocity_reference)
from stable_baselines3 import PPO


def test_reference_is_bounded_inward_and_brakes_at_origin():
    points = np.array([[0, 0], [.001, 0], [.3, .4], [30, 40]])
    reference = return_velocity_reference(points)
    assert np.isfinite(reference).all()
    np.testing.assert_array_equal(reference[0], [0, 0])
    assert np.linalg.norm(reference[1]) < .002
    assert (np.linalg.norm(reference, axis=1) <= .25).all()
    assert (np.sum(points[1:]*reference[1:], axis=1) < 0).all()
    rotation = np.array([[0, -1], [1, 0]])
    np.testing.assert_allclose(return_velocity_reference(points@rotation.T), reference@rotation.T)


def test_tracking_prefers_return_to_stopping_or_outward_motion_and_slows_near_target():
    displacement = np.tile([.4, 0], (4, 1))
    velocities = np.array([[-.20, 0], [0, 0], [.20, 0], [-1, 0]])
    costs = return_tracking_cost(displacement, velocities, np.full(4, .20))
    assert costs[0] > costs[1] > costs[2] > costs[3]
    at_origin = return_tracking_cost(np.zeros((2, 2)), velocities[:2], .20)
    assert at_origin[1] > at_origin[0]


def test_gate_requires_real_return_opportunities_rotation_and_stability():
    stats = dict(cases=30, completed=24, return_attempts=30, return_successes=24,
                 mean_yaw_angle_ratio=.9, mean_roll_rms_deg=1.)
    assert return_gate_passes(stats)
    for changes in (dict(return_attempts=0, return_successes=0), dict(return_successes=0),
                    dict(mean_yaw_angle_ratio=.2), dict(completed=10), dict(mean_roll_rms_deg=4)):
        assert not return_gate_passes({**stats, **changes})


@pytest.fixture
def env():
    result = ReturnTrainingSpinEnv(2, 1, cases=[(3, .7, 0, 1)]*2,
        terrain_cases=[dict(location='lower_flat', handover='lqr', offset=0,
                            takeover_time=0, fade=.3, dropout=False)]*2,
        disturbances=False, gate_trials=True, linkage_map='exact')
    result.seed(1000)
    result.reset()
    try:
        yield result
    finally:
        result.close()


def test_known_target_offset_matches_sensor_odometry_without_robot_teleport(env):
    np.testing.assert_allclose(env.qpos[:, :2]-env.start_xy, -env.target_offset, atol=1e-12)
    np.testing.assert_allclose(env.odometry.position, -env.target_offset, atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(env.target_offset, axis=1), .35)
    frame = env._frame_obs().copy()
    env.qpos[:, :2] += 100
    env.qvel[:, :2] += 100
    env.start_xy[:] -= 100
    np.testing.assert_array_equal(env._frame_obs(), frame)


def test_deadline_and_reference_change_only_on_new_episodes(env):
    elapsed = np.full(2, 5.)
    env.unscaled_yaw[:] = 2.
    first, _ = env._spin_reference(elapsed)
    assert np.max(np.abs(first)) < 1.3
    env.set_return_stage(2)
    assert (env.stage_per_episode == 0).all()
    env.excursion_age[:] = 7
    reasons = env._failure_reasons(np.ones(2, bool), np.zeros(2), np.zeros(2), np.full(2, .4))
    assert not (reasons & 8).any()
    env._reset_ids(np.array([0], dtype=np.int32))
    assert env.stage_per_episode.tolist() == [2, 0]
    env.excursion_age[:] = 7
    reasons = env._failure_reasons(np.ones(2, bool), np.zeros(2), np.zeros(2), np.full(2, .4))
    assert (reasons & 8).tolist() == [8, 0]
    env.excursion_age[:] = 16
    assert (env._failure_reasons(np.ones(2, bool), np.zeros(2), np.zeros(2), np.full(2, .4)) & 8).all()


def test_history_checkpoint_transfer_can_reset_value_without_changing_actions(env, tmp_path):
    torch.set_num_threads(1)
    kwargs = dict(n_steps=4, batch_size=4, policy_kwargs=dict(history_residual=True), device='cpu')
    source = PPO(AsymmetricSpinPolicy, env, seed=1, **kwargs)
    target = PPO(AsymmetricSpinPolicy, env, seed=2, **kwargs)
    with torch.no_grad():
        source.policy.mlp_extractor.policy_net.history_projection.weight.fill_(.01)
    path = tmp_path/'source.zip'
    source.save(path)
    value = target.policy.value_net.weight.detach().clone()
    warm_start_history_residual(target, path, reset_critic=True)
    obs = env._observation()
    np.testing.assert_allclose(target.predict(obs, deterministic=True)[0],
                               source.predict(obs, deterministic=True)[0], atol=1e-6)
    torch.testing.assert_close(target.policy.value_net.weight, value)
    env.episode_steps = 2
    for _ in range(2):
        obs, reward, done, infos = env.step(np.zeros((2, 5), np.float32))
        assert obs.shape == (2, 471) and np.isfinite(obs).all() and np.isfinite(reward).all()
    assert done.all()
    assert 'return_velocity' in infos[0]['spin_metrics']['reward_totals']
