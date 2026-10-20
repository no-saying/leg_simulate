"""Portable inference networks; training adapters provide optimization/losses."""

import torch
from torch import nn


class HistoryActor(nn.Module):
    """Compress a chronological sensor window, then act on current sensors."""

    def __init__(self, history_shape, layers, activation_fn, latent_size=64, hidden_size=256):
        super().__init__()
        history, frame = history_shape
        self.history_size = (history-1)*frame
        self.history_encoder = nn.Sequential(nn.Linear(self.history_size, hidden_size), activation_fn(),
                                             nn.Linear(hidden_size, latent_size), activation_fn())
        modules, size = [], frame+latent_size
        for width in layers:
            modules.extend((nn.Linear(size, width), activation_fn()))
            size = width
        self.control_net = nn.Sequential(*modules)
        self.output_size = size

    def forward(self, sensors):
        history = self.history_encoder(sensors[..., :self.history_size])
        return self.control_net(torch.cat((sensors[..., self.history_size:], history), dim=-1))


class HistoryResidualActor(nn.Module):
    """Legacy 56 x 8 history branch; retained for old checkpoint compatibility."""

    def __init__(self, actor_size, layers, activation_fn):
        super().__init__()
        self.history_size = actor_size-56
        self.history_encoder = nn.Sequential(nn.Linear(self.history_size, 128), activation_fn(),
                                             nn.Linear(128, 32), activation_fn())
        modules, size = [], actor_size
        for width in layers:
            modules.extend((nn.Linear(size, width), activation_fn()))
            size = width
        self.base_net = nn.Sequential(*modules)
        self.history_projection = nn.Linear(32, size)
        self.output_size = size

    def forward(self, sensors):
        compressed = self.history_encoder(sensors[..., :self.history_size])
        return self.base_net(sensors)+self.history_projection(compressed)


class SensorActor(nn.Module):
    """Standalone deterministic sensor-to-action network with no critic."""

    def __init__(self, features: nn.Module, action_head: nn.Module):
        super().__init__()
        self.network = nn.Sequential(features, action_head)

    def forward(self, sensors):
        return torch.clamp(self.network(sensors), -1, 1)
