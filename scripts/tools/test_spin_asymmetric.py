"""IMU frame consistency, privileged isolation and a real PPO update/save/load."""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jump_rl_env import XML, body_quat_from_control_euler
from spin_asymmetric_env import AsymmetricSpinEnv, PRIVILEGED_SIZE
from spin_asymmetric_policy import ACTOR_SIZE, AsymmetricSpinPolicy, SensorActor, warm_start_sensor_actor
from spin_imu import aligned_control_attitude, imu_attitude_extrinsic
from spin_rl_env import SpinRLBatchEnv
from stable_baselines3 import PPO


@pytest.mark.parametrize('angles', [(0, 0, 0), (.18, -.12, .7), (-.2, .15, -2)])
def test_all_imu_channels_share_one_coordinate_frame(angles):
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[3:7] = body_quat_from_control_euler(angles)
    mujoco.mj_forward(model, data)
    batch = SpinRLBatchEnv.__new__(SpinRLBatchEnv)
    batch.sensordata = data.sensordata[None].copy()
    batch.sensordata[:, 4:7] = [.2, -.7, 1.3]
    batch.sensordata[:, 7:10] = [.6, 9.7, -.4]
    _, gyro, acc = batch._imu()
    q = aligned_control_attitude(batch.sensordata[:, :4], imu_attitude_extrinsic(model))
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, q[0])
    site = data.site_xmat[model.site('imu').id].reshape(3, 3)
    for raw, transformed in ((batch.sensordata[0, 4:7], gyro[0]),
                              (batch.sensordata[0, 7:10], acc[0])):
        np.testing.assert_allclose(matrix.reshape(3, 3)@transformed, site@raw, atol=1e-7)


@pytest.fixture
def env():
    result = AsymmetricSpinEnv(1, 1, cases=[(3, .7, 0, 1)], terrain_cases=[dict(
        location='ramp', handover='lqr', offset=0, takeover_time=0, fade=.3, dropout=False)],
        disturbances=False, episode_seconds=.12)
    result.seed(1000)
    try:
        yield result
    finally:
        result.close()


def test_true_state_changes_critic_observations_only(env):
    obs = env.reset()
    assert obs.shape == (1, ACTOR_SIZE+PRIVILEGED_SIZE)
    frame = env._frame_obs().copy()
    env.qpos[:, :2] += [.4, -.3]
    env.qvel[:, :2] += [.5, .2]
    env.start_xy[:] -= .2
    env.contact_values[:] = 100
    np.testing.assert_array_equal(env._frame_obs(), frame)
    new = env._observation()
    np.testing.assert_array_equal(new[:, :ACTOR_SIZE], obs[:, :ACTOR_SIZE])
    assert np.max(np.abs(new[:, ACTOR_SIZE:]-obs[:, ACTOR_SIZE:])) > .1


def test_privileged_values_cannot_affect_actor_or_actor_gradients(env, tmp_path):
    torch.set_num_threads(1)
    model = PPO(AsymmetricSpinPolicy, env, n_steps=8, batch_size=8, n_epochs=1,
                policy_kwargs=dict(net_arch=[32, 32]), seed=10, device='cpu')
    obs = env.reset()
    changed = obs.copy()
    changed[:, ACTOR_SIZE:] += np.arange(PRIVILEGED_SIZE)+1
    np.testing.assert_array_equal(model.predict(obs, deterministic=True)[0],
                                  model.predict(changed, deterministic=True)[0])
    inputs = torch.tensor(obs, requires_grad=True)
    distribution = model.policy.get_distribution(inputs)
    distribution.distribution.mean.sum().backward()
    assert torch.count_nonzero(inputs.grad[:, ACTOR_SIZE:]) == 0
    assert torch.count_nonzero(inputs.grad[:, :ACTOR_SIZE]) > 0
    with torch.no_grad():
        value = model.policy.predict_values(torch.tensor(obs))
        changed_value = model.policy.predict_values(torch.tensor(changed))
    assert not torch.allclose(value, changed_value)
    before = model.policy.action_net.weight.detach().clone()
    model.learn(16)
    assert not torch.equal(model.policy.action_net.weight, before)
    assert all(torch.isfinite(p).all() for p in model.policy.parameters())
    assert np.isfinite(list(model.logger.name_to_value.values())).all()
    model.save(tmp_path/'asymmetric')
    loaded = PPO.load(tmp_path/'asymmetric', device='cpu')
    expected = model.predict(obs, deterministic=True)[0]
    np.testing.assert_array_equal(loaded.predict(obs, deterministic=True)[0], expected)
    np.testing.assert_allclose(loaded.policy.predict_actor(obs[:, :ACTOR_SIZE]), expected, atol=1e-6)
    exported = torch.jit.script(SensorActor(loaded.policy).eval())
    exported.save(str(tmp_path/'actor.pt'))
    standalone = torch.jit.load(str(tmp_path/'actor.pt'))
    with torch.no_grad():
        np.testing.assert_allclose(standalone(torch.tensor(obs[:, :ACTOR_SIZE])).numpy(), expected, atol=1e-6)


def test_v6_actor_migration_preserves_actions_and_fresh_critic(env):
    source = Path(__file__).resolve().parents[2]/'logs/rl_training/spin_anchor_v6_resume131k_to1m.zip'
    old = PPO.load(source, device='cpu')
    model = PPO(AsymmetricSpinPolicy, env, n_steps=8, batch_size=8,
                policy_kwargs=dict(net_arch=[256, 256]), device='cpu')
    critic = model.policy.value_net.weight.detach().clone()
    warm_start_sensor_actor(model, source)
    obs = np.random.default_rng(1).normal(size=(3, ACTOR_SIZE+PRIVILEGED_SIZE)).astype(np.float32)
    np.testing.assert_allclose(model.predict(obs, deterministic=True)[0],
                               old.predict(obs[:, :ACTOR_SIZE], deterministic=True)[0], atol=1e-6)
    torch.testing.assert_close(model.policy.value_net.weight, critic)


def test_disturbance_is_a_finite_force_pulse_observable_by_imu(env):
    env.reset()
    env.push_time[:] = 0
    env.push_velocity[:] = [.15, .08]
    velocity = env.qvel.copy()
    inertial = env.odometry.inertial_velocity.copy()
    env._before_policy_step()
    np.testing.assert_array_equal(env.qvel, velocity)
    assert np.linalg.norm(env.external_force[:, env.base_body, :2]) > 1
    assert env.push_applied.all()
    env.step(np.zeros((1, 5), np.float32))
    assert np.linalg.norm(env.odometry.inertial_velocity-inertial) > .01
    assert np.linalg.norm(env.odometry.inertial_velocity-env.qvel[:, :2]) < .025
    for _ in range(2):
        env.step(np.zeros((1, 5), np.float32))
    np.testing.assert_allclose(env.external_force[:, env.base_body, :2], 0, atol=1e-12)
    np.testing.assert_array_equal(env.push_remaining, 0)


def test_rollout_friction_randomizes_wheels_and_terrain_without_actor_input(env):
    env.friction_range = (.3, .3)
    env.reset()
    np.testing.assert_allclose(env.floor_friction[0, env.friction_geoms, 0], .3)
    assert env.episode_friction[0] == pytest.approx(.3)
    frame = env._frame_obs().copy()
    env.floor_friction[0, env.friction_geoms, 0] = 1.2
    np.testing.assert_array_equal(env._frame_obs(), frame)
    env.friction_range = (1.2, 1.2)
    env.reset()
    np.testing.assert_allclose(env.floor_friction[0, env.friction_geoms, 0], 1.2)
    assert env.episode_friction[0] == pytest.approx(1.2)
