"""Linkage virtual work, actuator ownership, sensor isolation and PPO export."""

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spin_asymmetric_env import PRIVILEGED_SIZE
from spin_asymmetric_policy import AsymmetricSpinPolicy, SensorActor
from spin_wrench_control import ACTION_SCALE, TORQUE_LIMIT
from spin_wrench_env import (ACTOR_SIZE, FRAME_SIZE, OBS_SCALE, WrenchSpinEnv,
                             ReturnWrenchSpinEnv, pitch_balance_reward)
from stable_baselines3 import PPO


@pytest.fixture
def env():
    result = WrenchSpinEnv(3, 3, cases=[(3, .7, .117-length, 1)
                           for length in (.047, .117, .210)],
        terrain_cases=[dict(location='ramp', handover='lqr', offset=0,
                            takeover_time=0, fade=.02, dropout=False)]*3,
        disturbances=False, episode_seconds=.12)
    result.seed(1000)
    result.reset()
    try:
        yield result
    finally:
        result.close()


def test_encoder_jacobian_matches_finite_difference_and_virtual_work(env):
    q = env.qpos[:, env.qa[:4]].copy()
    wheel, jac, support, valid = env.mapper.jacobians(q)
    assert valid.all()
    eps = 1e-6
    for side in range(2):
        for joint in range(2):
            offset = np.zeros_like(q)
            offset[:, side*2+joint] = eps
            plus = env.mapper.geometry.calculate(q+offset, np.zeros_like(q))[0]
            minus = env.mapper.geometry.calculate(q-offset, np.zeros_like(q))[0]
            np.testing.assert_allclose((plus[:, side]-minus[:, side])/(2*eps),
                                       jac[:, side, :, joint], atol=1e-7)
    commands = np.tile([2., 8., -3., 9., .1, -.05], (3, 1))
    ctrl, tau, _, _ = env.mapper.map(q, commands)
    assert (np.abs(tau) < TORQUE_LIMIT).all()
    dq = np.random.default_rng(11).normal(size=(3, 4))
    dw = np.random.default_rng(12).normal(size=(3, 2))
    _, velocity, spin, _ = env.mapper.geometry.calculate(q, dq)
    # The wheel rotor rotates with its moving support, not just its encoder.
    motor_power = (tau[:, [0, 1, 3, 4]]*dq).sum(axis=1)+(tau[:, [2, 5]]*dw).sum(axis=1)
    virtual_power = -(commands[:, :4].reshape(3, 2, 2)*velocity).sum(axis=(1, 2))
    virtual_power += (tau[:, [2, 5]]*(dw+spin)).sum(axis=1)
    np.testing.assert_allclose(motor_power, virtual_power, atol=1e-10)
    np.testing.assert_allclose(ctrl*env.mapper.gain, tau, atol=1e-12)
    assert not np.allclose(jac[0], jac[-1])
    np.testing.assert_allclose(np.einsum('nsj,nsj->ns', support, dq.reshape(3, 2, 2)), spin)


def test_full_takeover_excludes_all_six_native_outputs_even_nonfinite(env, monkeypatch):
    action = np.tile([.1, .3, -.1, .3, .1, .2], (3, 1))
    env.direct_action[:] = action
    env.authority[:] = 1
    env.rl_owner[:] = True
    env.output_rate_limit[:] = 1e12
    expected = env.mapper.map(env.qpos[:, env.qa[:4]], action*ACTION_SCALE)[0]
    monkeypatch.setattr(env.control, 'step', lambda mask, dt: pytest.fail('Native ran during RL ownership'))
    actual = env._actuator_control(np.ones(3, bool))
    np.testing.assert_allclose(actual, expected, atol=1e-15)
    assert np.isfinite(actual).all()
    np.testing.assert_array_equal(env.native_leak_peak, 0)
    env.direct_action[:] = 0
    np.testing.assert_array_equal(env._actuator_control(np.ones(3, bool)), 0)


def test_binary_ownership_has_no_blended_middle_and_sensor_only_frame(env, monkeypatch):
    env.direct_action[:] = [.1, .3, -.1, .3, .1, .2]
    direct = env.mapper.map(env.qpos[:, env.qa[:4]], env.direct_action*ACTION_SCALE)[0]
    native = np.full((3, 6), .03)
    seen = []
    def step(mask, dt):
        seen.append(mask.copy())
        output = native.copy()
        output[~mask] = np.nan
        return output
    monkeypatch.setattr(env.control, 'step', step)
    env.takeover_time[:] = 1
    env._prepare_control(np.array([0., .5, 1.5]), env.direct_action.copy(), np.ones(3, bool))
    np.testing.assert_array_equal(env.authority, [0, 0, 1])
    env.output_rate_limit[:] = 1e12
    result = env._actuator_control(np.ones(3, bool))
    np.testing.assert_allclose(result, (1-env.authority[:, None])*native+env.authority[:, None]*direct)
    np.testing.assert_array_equal(seen, [[True, True, False]])
    frame = env._frame_obs().copy()
    assert frame.shape == (3, FRAME_SIZE) and len(OBS_SCALE) == FRAME_SIZE
    env.qpos[:, :2] += [.5, -.4]
    env.qvel[:, :2] += [.3, .2]
    env.contact_values[:] = 100
    env.native_torque[:] = 900
    np.testing.assert_array_equal(env._frame_obs(), frame)


def test_slew_limit_and_rl_sensor_updates_do_not_need_native_controller(env, monkeypatch):
    env.rl_owner[:] = True
    env.direct_action[:] = [.5, .5, .5, .5, 1, 1]
    env.native_previous_output[:] = 0
    monkeypatch.setattr(env.control, 'step', lambda *args: pytest.fail('Native ran during RL ownership'))
    previous = env.native_previous_output.copy()
    result = env._actuator_control(np.ones(3, bool))*env.mapper.gain
    assert (np.abs(result-previous) <= env.output_rate_limit*env.dt+1e-12).all()
    env.qvel[:, env.va[4:6]] += 2
    frame = env._frame_obs()
    native = env._native_observation()
    assert np.isfinite(frame).all()
    wheel, velocity, support, valid = env.geometry.calculate(env.qpos[:, env.qa[:4]], env.qvel[:, env.va[:4]])
    np.testing.assert_allclose(native[:, 9:11], env.qvel[:, env.va[4:6]]+support, atol=1e-5)
    env._after_physics_step()
    assert np.isfinite(env.odometry.position).all()


def test_dropout_switches_whole_motor_owner_and_resets_native_state(env, monkeypatch):
    resets = []
    original = env.control.reset
    def reset(mask):
        resets.append(mask.copy())
        original(mask)
    monkeypatch.setattr(env.control, 'reset', reset)
    env.takeover_time[:] = 0
    env.dropout_time[:] = 1
    env.dropout_length[:] = .5
    mask = np.ones(3, bool)
    for time, owner in ((.5, True), (1.1, False), (1.6, True)):
        env._prepare_control(np.full(3, time), np.zeros((3, 6)), mask)
        assert (env.rl_owner == owner).all()
        assert (env.authority == int(owner)).all()
    assert len(resets) == 1
    np.testing.assert_array_equal(env.owner_switches, 3)


def test_return_curriculum_supports_exclusive_six_output_control():
    env = ReturnWrenchSpinEnv(1, 1, cases=[(0, 0, 0, 1)],
        terrain_cases=[dict(location='lower_flat', handover='pid', takeover_time=0,
                            fade=.3, dropout=False)], disturbances=False, gate_trials=True,
        episode_seconds=.04)
    try:
        obs = env.reset()
        assert obs.shape == (1, ACTOR_SIZE+PRIVILEGED_SIZE)
        for _ in range(2):
            obs, reward, done, infos = env.step(np.zeros((1, 6), np.float32))
        assert done[0] and np.isfinite(reward).all()
        metrics = infos[0]['spin_metrics']
        assert metrics['full_rl_control_seconds'] == pytest.approx(.04)
        assert metrics['native_update_seconds'] == 0
        assert metrics['native_torque_leak_peak_ctrl'] == 0
        assert 'return_velocity' in metrics['reward_totals']
    finally:
        env.close()


def test_mirrored_linkage_and_body_wrench_directions(env):
    q = np.tile([.2, -.2, -.2, .2], (3, 1))
    commands = np.tile([0., 10., 0., 10., 0., 0.], (3, 1))
    _, tau, (wheel, force), _ = env.mapper.map(q, commands)
    np.testing.assert_allclose(tau[:, :2], -tau[:, 3:5], atol=.04)
    wrench = env.mapper.point_force_wrench(wheel, force, env.axle_lateral)
    np.testing.assert_allclose(wrench[:, :3], np.tile([0, 0, 20.], (3, 1)))
    one_side = force.copy()
    one_side[:, 1] = 0
    left = env.mapper.point_force_wrench(wheel, one_side, env.axle_lateral)
    np.testing.assert_allclose(left[:, 3], env.axle_lateral[0]*10)
    commands[:] = [0, 0, 0, 0, 0, .1]
    _, tau, _, _ = env.mapper.map(q, commands)
    np.testing.assert_allclose(tau[:, [2, 5]], np.tile([-.1, .1], (3, 1)))


def test_six_action_ppo_update_and_deployment_have_no_privileged_dependency(env, tmp_path):
    torch.set_num_threads(1)
    model = PPO(AsymmetricSpinPolicy, env, n_steps=4, batch_size=12, n_epochs=1,
                policy_kwargs=dict(net_arch=[32, 32], actor_size=ACTOR_SIZE), device='cpu', seed=12)
    obs = env._observation()
    assert obs.shape == (3, ACTOR_SIZE+PRIVILEGED_SIZE)
    changed = obs.copy()
    changed[:, ACTOR_SIZE:] += 5
    expected = model.predict(obs, deterministic=True)[0]
    np.testing.assert_array_equal(model.predict(changed, deterministic=True)[0], expected)
    before = model.policy.action_net.weight.detach().clone()
    model.learn(24)
    assert not torch.equal(before, model.policy.action_net.weight)
    assert all(torch.isfinite(p).all() for p in model.policy.parameters())
    model.save(tmp_path/'wrench')
    loaded = PPO.load(tmp_path/'wrench', device='cpu')
    assert loaded.policy.actor_size == ACTOR_SIZE
    expected = loaded.predict(obs, deterministic=True)[0]
    np.testing.assert_allclose(loaded.policy.predict_actor(obs[:, :ACTOR_SIZE]), expected, atol=1e-6)
    actor = torch.jit.script(SensorActor(loaded.policy).eval())
    actor.save(str(tmp_path/'wrench.actor.pt'))
    with torch.no_grad():
        np.testing.assert_allclose(torch.jit.load(str(tmp_path/'wrench.actor.pt'))(
            torch.tensor(obs[:, :ACTOR_SIZE])).numpy(), expected, atol=1e-6)
    assert expected.shape == (3, 6)


def test_pitch_reward_has_broad_symmetric_gradient_and_damping():
    angles = np.deg2rad([0, 5, 10, 20, 40])
    zero = np.zeros(5)
    action = np.zeros((5, 6))
    reward, terms = pitch_balance_reward(angles, zero, zero, zero, action, action)
    assert (np.diff(reward) < 0).all()
    assert terms['pitch_upright'][0] == 6
    mirrored, _ = pitch_balance_reward(-angles, zero, zero, zero, action, action)
    np.testing.assert_array_equal(reward, mirrored)
    moving, _ = pitch_balance_reward(angles, np.ones(5)*2, zero, zero, action, action)
    assert (moving < reward).all()
    worst, _ = pitch_balance_reward(angles, zero+100, zero+100, zero+100,
                                    action+1, action+2)
    assert (worst > 0).all()


def test_pitch_task_is_flat_immediate_exclusive_and_has_no_old_reward(monkeypatch):
    pitch = WrenchSpinEnv(2, 2, reward_profile='pitch', episode_seconds=.04)
    try:
        obs = pitch.reset()
        assert obs.shape == (2, ACTOR_SIZE+PRIVILEGED_SIZE)
        assert pitch.location == ['lower_flat']*2
        np.testing.assert_array_equal(pitch.slope_deg, 0)
        assert (np.abs(pitch.ext) <= .010).all()
        assert not pitch.disturbances
        monkeypatch.setattr(pitch.control, 'step', lambda *args: pytest.fail('Native ran during pitch RL'))
        for _ in range(2):
            _, reward, done, infos = pitch.step(np.tile([0, .3, 0, .3, 0, 0], (2, 1)))
        assert done.all() and np.isfinite(reward).all()
        metrics = infos[0]['spin_metrics']
        assert metrics['native_update_seconds'] == 0
        assert metrics['full_rl_control_seconds'] == pytest.approx(.04)
        assert metrics['native_torque_leak_peak_ctrl'] == 0
        assert metrics['rl_pitch_rms_deg'] >= 0
        assert set(metrics['reward_totals']) == {
            'survival', 'terminal', 'pitch_upright', 'pitch_rate', 'roll_guard',
            'height_guard', 'effort', 'smooth'}
        # An early failure costs ten, independently of the episode horizon.
        assert pitch.failure_cost_per_second == 0
        assert pitch.training_reward_scale == 1
    finally:
        pitch.close()
