#!/usr/bin/env python3
"""评估/回放训练好的跳跃策略, 打印最高高度/姿态。"""
import os
import sys

import numpy as np
from stable_baselines3 import PPO

from jump_rl_env import JumpRLEnv


def main():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jump_ppo")
    if not os.path.exists(path + ".zip"):
        print("模型不存在, 先训练: python3 scripts/train_jump_rl.py")
        return
    model = PPO.load(path)
    env = JumpRLEnv()

    for ep in range(3):
        obs, _ = env.reset()
        done = False
        max_h = 0.0
        peak_pitch = peak_roll = 0.0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            done = term or trunc
            max_h = max(max_h, info["height"])
            peak_pitch = max(peak_pitch, abs(info["pitch_deg"]))
            peak_roll = max(peak_roll, abs(info["roll_deg"]))
        print(f"episode {ep}: 最高弹起 {max_h:.3f}m | "
              f"峰值|pitch|={peak_pitch:.1f}° | 峰值|roll|={peak_roll:.1f}° | "
              f"终止={'翻车' if term else '超时'}")


if __name__ == "__main__":
    main()
