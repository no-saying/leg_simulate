#!/usr/bin/env python3
"""跳跃 PPO 训练脚本。

用法: python3 scripts/train_jump_rl.py [总步数]
      OUT=scripts/jump_ppo_full 指定输出前缀(默认 scripts/jump_ppo)
      RESUME=scripts/jump_ppo_200k.zip 续训
      RL_CURRICULUM=balance 阶段一(只学平衡)
      GAMMA=0.999 / N_STEPS=4096 覆盖折扣/批长(dt=0.001 时必需调大)
      RL_BACKEND=mjbatch N_ENVS=16 MJ_THREADS=8 启用批量物理环境
      RL_BACKEND=subproc N_ENVS=8 使用独立进程环境
输出: <OUT>_100k.zip ... <OUT>.zip (策略权重)
"""

import os
import sys
import time
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from jump_rl_env import JumpRLEnv


def main():
    total = int(sys.argv[1]) if len(sys.argv) > 1 else 300_000
    out = os.environ.get(
        "OUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "jump_ppo")
    )
    gamma = float(os.environ.get("GAMMA", "0.99"))
    n_steps = int(os.environ.get("N_STEPS", "2048"))
    backend = os.environ.get("RL_BACKEND", "subproc")
    n_envs = int(os.environ.get("N_ENVS", "16" if backend == "mjbatch" else "1"))
    batch_size = int(os.environ.get("BATCH_SIZE", "256"))
    if total < 1 or n_envs < 1 or n_steps < 1 or batch_size < 2 or n_steps * n_envs < 2:
        raise ValueError("Invalid total steps, N_ENVS, N_STEPS or BATCH_SIZE")
    torch.set_num_threads(int(os.environ.get("TORCH_THREADS", "1")))
    if backend == "mjbatch":
        from jump_rl_batch_env import JumpRLBatchEnv

        env = JumpRLBatchEnv(n_envs, int(os.environ.get("MJ_THREADS", "8")))
        env.seed(0)
    elif backend == "subproc":
        from rl_batch_control import build_bridges

        bridge, _ = build_bridges()
        env = make_vec_env(
            JumpRLEnv,
            n_envs=n_envs,
            seed=0,
            env_kwargs={"bridge_path": str(bridge)},
            vec_env_cls=SubprocVecEnv if n_envs > 1 else None,
        )
    else:
        raise ValueError("RL_BACKEND must be mjbatch or subproc")
    try:
        resume = os.environ.get("RESUME")
        if resume:
            model = PPO.load(resume, env=env, device=os.environ.get("DEVICE", "cpu"))
            print(f"从 {resume} 续训 {total} 步")
        else:
            model = PPO(
                "MlpPolicy",
                env,
                verbose=1,
                n_steps=n_steps,
                batch_size=batch_size,
                n_epochs=int(os.environ.get("N_EPOCHS", "10")),
                gamma=gamma,
                learning_rate=3e-4,
                ent_coef=0.01,
                clip_range=0.2,
                policy_kwargs=dict(net_arch=[256, 256]),
                seed=0,
                device=os.environ.get("DEVICE", "cpu"),
            )
        print(f"环境后端: {backend}, 数量: {n_envs}, rollout 样本数: {model.n_steps * n_envs}")
        t0 = time.time()
        ckpt = 100_000
        initial_steps = model.num_timesteps
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        while model.num_timesteps - initial_steps < total:
            batch = min(ckpt, total - (model.num_timesteps - initial_steps))
            model.learn(total_timesteps=batch, reset_num_timesteps=False, progress_bar=False)
            steps_done = model.num_timesteps - initial_steps
            model.save(f"{out}_{model.num_timesteps // 1000}k")
            print(f"checkpoint saved: {model.num_timesteps} actual steps ({steps_done}/{total})")
        model.save(out)
        print(f"\n训练完成 {steps_done} 步, 耗时 {time.time() - t0:.0f}s -> {out}.zip")
    finally:
        env.close()


if __name__ == "__main__":
    main()
