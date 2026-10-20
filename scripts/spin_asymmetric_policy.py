"""PPO policy with a sensor-only actor and a privileged training critic."""

import numpy as np
import torch
from torch import nn
from stable_baselines3.common.policies import ActorCriticPolicy
from wheel_legged.policies.networks import (
    HistoryActor, HistoryResidualActor, SensorActor as PortableSensorActor,
)

ACTOR_SIZE = 56*8


class AsymmetricExtractor(nn.Module):
    def __init__(self, total_size, net_arch, activation_fn, actor_size=ACTOR_SIZE, history_shape=None,
                 history_residual=False, history_latent_size=64, history_hidden_size=256):
        super().__init__()
        self.actor_size = actor_size
        architecture = net_arch if isinstance(net_arch, dict) else dict(pi=net_arch, vf=net_arch)

        def network(size, layers):
            modules = []
            for width in layers:
                modules.extend((nn.Linear(size, width), activation_fn()))
                size = width
            return nn.Sequential(*modules), size

        if history_residual:
            self.policy_net = HistoryResidualActor(actor_size, architecture['pi'], activation_fn)
            self.latent_dim_pi = self.policy_net.output_size
        elif history_shape is None:
            self.policy_net, self.latent_dim_pi = network(actor_size, architecture['pi'])
        else:
            self.policy_net = HistoryActor(history_shape, architecture['pi'], activation_fn,
                                           history_latent_size, history_hidden_size)
            self.latent_dim_pi = self.policy_net.output_size
        self.value_net, self.latent_dim_vf = network(total_size, architecture['vf'])

    def forward_actor(self, features):
        return self.policy_net(features[..., :self.actor_size])

    def forward_critic(self, features):
        return self.value_net(features)

    def forward(self, features):
        return self.forward_actor(features), self.forward_critic(features)


class SensorActor(PortableSensorActor):
    """Standalone deterministic deployment network, with no privileged inputs."""

    def __init__(self, policy):
        super().__init__(policy.mlp_extractor.policy_net, policy.action_net)


class AsymmetricSpinPolicy(ActorCriticPolicy):
    def __init__(self, *args, actor_size=ACTOR_SIZE, history_shape=None, history_residual=False,
                 history_latent_size=64, history_hidden_size=256, **kwargs):
        self.actor_size = actor_size
        self.history_residual = history_residual
        self.history_latent_size = history_latent_size
        self.history_hidden_size = history_hidden_size
        if history_latent_size < 1 or history_hidden_size < 1:
            raise ValueError('History network widths must be positive')
        if history_residual and (actor_size != ACTOR_SIZE or history_shape is not None):
            raise ValueError('History residual migration requires the existing 448-sensor contract')
        self.history_shape = tuple(history_shape) if history_shape is not None else None
        if self.history_shape is not None and (len(self.history_shape) != 2
                or self.history_shape[0] < 2 or self.history_shape[1] < 1
                or np.prod(self.history_shape) != actor_size):
            raise ValueError('History shape must match the sensor actor input size')
        super().__init__(*args, **kwargs)
        if history_residual:
            # SB3 orthogonal initialization runs during super().__init__.
            # Zero this new path afterwards to preserve migrated actor outputs.
            nn.init.zeros_(self.mlp_extractor.policy_net.history_projection.weight)
            nn.init.zeros_(self.mlp_extractor.policy_net.history_projection.bias)

    def _get_constructor_parameters(self):
        return {**super()._get_constructor_parameters(), 'actor_size': self.actor_size,
                'history_shape': self.history_shape, 'history_residual': self.history_residual,
                'history_latent_size': self.history_latent_size,
                'history_hidden_size': self.history_hidden_size}

    def _build_mlp_extractor(self):
        if self.features_dim <= self.actor_size or not self.share_features_extractor:
            raise ValueError('Expected sensor prefix plus privileged suffix and a flat shared extractor')
        self.mlp_extractor = AsymmetricExtractor(
            self.features_dim, self.net_arch, self.activation_fn, self.actor_size,
            self.history_shape, self.history_residual, self.history_latent_size,
            self.history_hidden_size).to(self.device)

    def predict_actor(self, sensors, deterministic=True):
        """Deployment needs no critic observations or simulator state."""
        sensors = np.asarray(sensors, dtype=np.float32)
        if sensors.shape[-1] != self.actor_size:
            raise ValueError(f'Expected {self.actor_size} sensor observations')
        unbatched = sensors.ndim == 1
        inputs = torch.as_tensor(np.atleast_2d(sensors), device=self.device)
        self.set_training_mode(False)
        with torch.no_grad():
            latent = self.mlp_extractor.policy_net(inputs)
            actions = self._get_action_dist_from_latent(latent).get_actions(deterministic)
        result = np.clip(actions.cpu().numpy(), self.action_space.low, self.action_space.high)
        return result[0] if unbatched else result


def warm_start_sensor_actor(model, source):
    """Migrate V5/V6 actor weights; leave critic and optimizer fresh."""
    from stable_baselines3 import PPO
    old = PPO.load(source, device='cpu')
    frame = old.policy.mlp_extractor.policy_net[0].in_features//8
    if frame not in (52, 56):
        raise ValueError('Expected a V5 or V6 sensor actor')
    target = model.policy.state_dict()
    for name, value in old.policy.state_dict().items():
        if name == 'mlp_extractor.policy_net.0.weight' and frame == 52:
            expanded = torch.zeros_like(target[name])
            for i in range(8):
                expanded[:, i*56:i*56+52] = value[:, i*52:(i+1)*52]
            target[name] = expanded
        elif name.startswith(('mlp_extractor.policy_net.', 'action_net.')) or name == 'log_std':
            target[name] = value
    model.policy.load_state_dict(target)


def warm_start_history_residual(model, source, reset_critic=False):
    """Preserve the actor; optionally reset value weights for a changed reward."""
    from stable_baselines3 import PPO
    old = PPO.load(source, device='cpu')
    if (old.observation_space != model.observation_space
            or old.action_space != model.action_space or not model.policy.history_residual):
        raise ValueError('History migration requires identical residual task inputs and outputs')
    target = model.policy.state_dict()
    prefix = 'mlp_extractor.policy_net.'
    for name, value in old.policy.state_dict().items():
        if reset_critic and name.startswith(('mlp_extractor.value_net.', 'value_net.')):
            continue
        renamed = (prefix+'base_net.'+name[len(prefix):]
                   if name.startswith(prefix) and not old.policy.history_residual else name)
        if renamed not in target or target[renamed].shape != value.shape:
            raise ValueError(f'Incompatible source parameter: {name}')
        target[renamed] = value
    model.policy.load_state_dict(target)
