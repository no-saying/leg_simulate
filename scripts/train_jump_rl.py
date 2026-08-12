#!/usr/bin/env python3
"""跳跃 PPO 训练脚本。

用法: python3 scripts/train_jump_rl.py [总步数]
      OUT=scripts/jump_ppo_full 指定输出前缀(默认 scripts/jump_ppo)
      RESUME=scripts/jump_ppo_200k.zip 续训
      RL_CURRICULUM=balance 阶段一(只学平衡)
      GAMMA=0.999 / N_STEPS=4096 覆盖折扣/批长(dt=0.001 时必需调大)
      N_ENVS=8 多进程并行环境数(机器 22 核, 建议 8~12, 线性加速)
输出: <OUT>_100k.zip ... <OUT>.zip (策略权重)
"""
import os
import sys
import time

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from jump_rl_env import JumpRLEnv


def main():
    total = int(sys.argv[1]) if len(sys.argv) > 1 else 300_000
    out = os.environ.get("OUT", os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "jump_ppo"))
    gamma = float(os.environ.get("GAMMA", "0.99"))
    n_steps = int(os.environ.get("N_STEPS", "2048"))
    n_envs = int(os.environ.get("N_ENVS", "1"))
    if n_steps % n_envs != 0:
        n_steps = (n_steps // n_envs) * n_envs

    env = make_vec_env(JumpRLEnv, n_envs=n_envs, seed=0)
    print(f"并行环境数: {n_envs} (每 rollout 步收集 {n_envs} 条轨迹)")
    resume = os.environ.get("RESUME")
    if resume:
        model = PPO.load(resume, env=env)
        print(f"从 {resume} 续训 {total} 步")
    else:
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            n_steps=n_steps,
            batch_size=256,
            n_epochs=10,
            gamma=gamma,
            learning_rate=3e-4,
            ent_coef=0.01,
            clip_range=0.2,
            policy_kwargs=dict(net_arch=[256, 256]),
            seed=0,
        )

    t0 = time.time()
    # 每 100k 步保存一次 checkpoint, 防止训练后期退化时找不到好模型
    ckpt = 100_000
    steps_done = 0
    while steps_done < total:
        batch = min(ckpt, total - steps_done)
        model.learn(total_timesteps=batch, reset_num_timesteps=False, progress_bar=False)
        steps_done += batch
        model.save(f"{out}_{steps_done//1000}k")
        print(f"checkpoint saved: {out}_{steps_done//1000}k ({steps_done}/{total})")
    model.save(out)
    print(f"\n训练完成 {total} 步, 耗时 {time.time()-t0:.0f}s -> {out}.zip")


if __name__ == "__main__":
    main()
