"""Compare end-to-end rollout throughput, including episode warmup/reset."""

import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile
import time

import numpy as np
import torch
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jump_rl_env import JumpRLEnv
from jump_rl_batch_env import JumpRLBatchEnv
from rl_batch_control import ROOT, build_bridges


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--envs", type=int, default=8)
    p.add_argument("--threads", type=int, default=4)
    p.add_argument("--steps", type=int, default=512)
    p.add_argument(
        "--backends",
        nargs="+",
        choices=["serial", "subproc", "mjbatch"],
        default=["serial", "subproc", "mjbatch"],
    )
    args = p.parse_args()
    torch.set_num_threads(1)
    bridge, _ = build_bridges()
    actions = (
        np.random.default_rng(2).uniform(-0.3, 0.3, (args.steps, args.envs, 5)).astype(np.float32)
    )
    rows = []
    with tempfile.TemporaryDirectory() as directory:
        for backend in args.backends:
            if backend == "serial":
                factories = []
                for i in range(args.envs):
                    path = Path(directory) / f"serial_{i}.so"
                    shutil.copyfile(bridge, path)
                    factories.append(lambda path=path: Monitor(JumpRLEnv(bridge_path=path)))
                env = DummyVecEnv(factories)
            elif backend == "subproc":
                env = make_vec_env(
                    JumpRLEnv,
                    n_envs=args.envs,
                    env_kwargs={"bridge_path": str(bridge)},
                    vec_env_cls=SubprocVecEnv,
                )
            else:
                env = JumpRLBatchEnv(args.envs, args.threads)
            try:
                env.seed(42)
                start = time.perf_counter()
                obs = env.reset()
                reset_s = time.perf_counter() - start
                assert np.isfinite(obs).all()
                start = time.perf_counter()
                episodes = 0
                for action in actions:
                    obs, reward, done, _ = env.step(action)
                    assert np.isfinite(obs).all() and np.isfinite(reward).all()
                    episodes += int(done.sum())
                elapsed = time.perf_counter() - start
                row = dict(
                    backend=backend,
                    num_envs=args.envs,
                    threads=args.threads,
                    vector_steps=args.steps,
                    transitions=args.envs * args.steps,
                    reset_seconds=reset_s,
                    rollout_seconds=elapsed,
                    transitions_per_second=args.envs * args.steps / elapsed,
                    episodes=episodes,
                )
                rows.append(row)
                print("BENCHMARK", json.dumps(row), flush=True)
            finally:
                env.close()
    path = ROOT / "logs/rl_training" / f"benchmark_{args.envs}_threads{args.threads}.json"
    path.write_text(json.dumps(rows, indent=2) + "\n")


if __name__ == "__main__":
    main()
