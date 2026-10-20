"""Self-contained checkpoint fixtures for policy migration regressions."""

import gymnasium as gym
import numpy as np
import pytest
from stable_baselines3 import PPO


class PolicySpaceEnv(gym.Env):
    def __init__(self, observation_size):
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (observation_size,), np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (5,), np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return np.zeros(self.observation_space.shape, np.float32), {}

    def step(self, action):
        return np.zeros(self.observation_space.shape, np.float32), 0.0, False, False, {}


@pytest.fixture
def migration_checkpoint(tmp_path):
    environments = []

    def create(policy, observation_size):
        environment = PolicySpaceEnv(observation_size)
        environments.append(environment)
        model = PPO(policy, environment, n_steps=8, batch_size=8,
                    policy_kwargs=dict(net_arch=[256, 256]), seed=10, device='cpu')
        path = tmp_path / f'source_{len(environments)}.zip'
        model.save(path)
        return model, path

    yield create
    for environment in environments:
        environment.close()
