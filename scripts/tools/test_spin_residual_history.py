"""Residual actor migration, motor torque PI tracking and linkage diagnostics."""

import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spin_asymmetric_env import AsymmetricSpinEnv
from spin_asymmetric_policy import ACTOR_SIZE, AsymmetricSpinPolicy, SensorActor, warm_start_history_residual
from spin_torque_loop import TorquePI, SimulatedTorqueMotors
from spin_wrench_control import TORQUE_LIMIT, WrenchTorqueMapper
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[2]


def make_env(**kwargs):
    result = AsymmetricSpinEnv(1, 1, cases=[(3, .7, 0, 1)],
        terrain_cases=[dict(location='ramp', handover='lqr', offset=0,
                           takeover_time=0, fade=.02, dropout=False)],
        disturbances=False, episode_seconds=.12, **kwargs)
    result.seed(1000)
    return result


def test_history_migration_preserves_actor_critic_and_sensor_boundary(tmp_path):
    torch.set_num_threads(1)
    env = make_env()
    try:
        old = PPO.load(ROOT/'logs/rl_training/spin_asymmetric_v7_friction_1m_20261017_655360.zip', device='cpu')
        model = PPO(AsymmetricSpinPolicy, env, n_steps=8, batch_size=8, n_epochs=2,
                    policy_kwargs=dict(net_arch=[256, 256], history_residual=True), device='cpu')
        warm_start_history_residual(model, ROOT/'logs/rl_training/spin_asymmetric_v7_friction_1m_20261017_655360.zip')
        obs = env.reset()
        random_obs = np.random.default_rng(12).normal(size=(5, env.observation_space.shape[0])).astype(np.float32)
        for inputs in (obs, random_obs):
            np.testing.assert_allclose(model.predict(inputs, deterministic=True)[0],
                                       old.predict(inputs, deterministic=True)[0], atol=1e-6)
            with torch.no_grad():
                torch.testing.assert_close(model.policy.predict_values(torch.tensor(inputs)),
                                           old.policy.predict_values(torch.tensor(inputs)))
        changed = obs.copy()
        changed[:, ACTOR_SIZE:] += 20
        np.testing.assert_array_equal(model.predict(obs, deterministic=True)[0],
                                      model.predict(changed, deterministic=True)[0])
        before = model.policy.mlp_extractor.policy_net.history_encoder[0].weight.detach().clone()
        model.learn(32)
        assert not torch.equal(before, model.policy.mlp_extractor.policy_net.history_encoder[0].weight)
        model.save(tmp_path/'history')
        loaded = PPO.load(tmp_path/'history', device='cpu')
        assert loaded.policy.history_residual
        expected = model.predict(obs, deterministic=True)[0]
        np.testing.assert_array_equal(loaded.predict(obs, deterministic=True)[0], expected)
        actor = torch.jit.script(SensorActor(loaded.policy).eval())
        with torch.no_grad():
            np.testing.assert_allclose(actor(torch.tensor(obs[:, :ACTOR_SIZE])).numpy(), expected, atol=1e-6)
    finally:
        env.close()


def test_six_torque_pi_loops_reject_motor_gain_and_bias_errors():
    target = np.array([[12., -15., 2., -18., 14., -2.]])
    gain = np.array([[.8, 1.2, .9, 1.1, .85, 1.15]])
    bank = SimulatedTorqueMotors(1, TORQUE_LIMIT)
    bank.reset([0], np.zeros((1, 6)), gain)
    pi = TorquePI(1, TORQUE_LIMIT)
    measured = np.zeros((1, 6))
    for tick in range(2000):
        if tick == 800:
            bank.bias[:] = [.7, -.8, .2, -.6, .5, -.2]
        measured = bank.step(pi.step(target, measured, .001), .001)
    np.testing.assert_allclose(measured, target, atol=1e-5)
    assert np.max(np.abs(target-(gain*target+bank.bias))) > 1
    for _ in range(1000):
        measured = bank.step(pi.step(np.tile(TORQUE_LIMIT*2, (1, 1)), measured, .001), .001)
    assert (np.abs(pi.integral) <= TORQUE_LIMIT).all()
    for _ in range(1000):
        measured = bank.step(pi.step(np.zeros((1, 6)), measured, .001), .001)
    np.testing.assert_allclose(measured, 0, atol=1e-5)


def test_native_residual_wheel_audit_and_body_wrench_roundtrip():
    env = make_env()
    try:
        env.reset()
        env.step(np.array([[0, 0, 0, .5, -.5]], np.float32))
        audit = env.control.wheel_mix_audit()[0]
        filtered = env.control.spin_actions()[0]
        np.testing.assert_allclose(audit[2:4], filtered[3:5]*[.20, .15], atol=1e-7)
        np.testing.assert_allclose(audit[4], audit[0]+audit[2], atol=1e-7)
        np.testing.assert_allclose(audit[6:8], env.ctrl[0, [2, 5]]*10, atol=1e-6)
        mapper = WrenchTorqueMapper(env.model)
        positions = env.qpos[:, env.qa[:4]]
        desired = np.array([[2., 8., -3., 9., .1, -.05]])
        _, torque, (wheel, force), valid = mapper.map(positions, desired)
        wrench, reconstructed, recovered, condition = mapper.equivalent_body_wrench(
            positions, torque, env.axle_lateral)
        assert valid.all() and recovered.all() and np.isfinite(condition).all()
        np.testing.assert_allclose(reconstructed, force, atol=1e-9)
        np.testing.assert_allclose(wrench, mapper.point_force_wrench(wheel, force, env.axle_lateral), atol=1e-9)
    finally:
        env.close()


def test_optional_residual_pi_path_resets_and_remains_finite():
    env = make_env(residual_torque_loop='pi')
    try:
        obs = env.reset()
        for _ in range(3):
            obs, reward, _, _ = env.step(np.zeros((1, 5), np.float32))
            assert np.isfinite(obs).all() and np.isfinite(reward).all()
            assert (np.abs(env.ctrl*env.motor_ctrl_gain) <= TORQUE_LIMIT+1e-6).all()
        env.residual_pi.integral[:] = 5
        env.reset()
        np.testing.assert_array_equal(env.residual_pi.integral, 0)
    finally:
        env.close()


def test_exact_remap_preserves_virtual_force_and_wheel_torque():
    env = make_env(linkage_map='exact')
    try:
        env.reset()
        env._fill_frames()
        native = np.array([[.003, -.002, .03, -.004, .005, -.02]], np.float32)
        env.control.step = lambda mask, dt: native
        source = env.control.leg_jacobians().swapaxes(-1, -2)
        force = np.linalg.solve(source, (native[:, [0, 1, 3, 4]]*50).reshape(1, 2, 2, 1))[..., 0]
        _, exact, _, valid = env.force_mapper.jacobians(env.qpos[:, env.qa[:4]])
        assert valid.all()
        actual = env._actuator_control(np.ones(1, bool))
        expected = np.einsum('nscj,nsc->nsj', exact, force)
        np.testing.assert_allclose(actual[:, [0, 1, 3, 4]]*50, expected.reshape(1, 4), atol=1e-7)
        np.testing.assert_array_equal(actual[:, [2, 5]], native[:, [2, 5]])
    finally:
        env.close()
