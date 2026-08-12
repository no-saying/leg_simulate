#!/usr/bin/env python3
"""C 侧 RL 前向推理 vs Python PPO.predict 一致性验证。

流程:
  1. 加载 scripts/jump_ppo_300k.zip, 生成若干固定随机观测。
  2. 用策略网络原始均值得到参考输出 -> /tmp/rl_ref_py.txt。
  3. 观测写入 /tmp/rl_obs_verify.txt。
  4. 编译运行 scripts/rl_jump_c_test.c(只依赖 rl_jump_net.c/h),
     得到 C 输出 /tmp/rl_out_c.txt。
  5. 逐样本对比最大绝对误差, 打印结论。

用法: python3 scripts/verify_rl_jump_c.py [模型路径]
"""
import os
import subprocess
import sys
import types
import tempfile

import numpy as np
import torch

if 'cv2' not in sys.modules:
    _f = types.ModuleType('cv2')
    _o = types.ModuleType('cv2.ocl')
    _o.setUseOpenCL = lambda *a, **k: None
    _f.ocl = _o
    sys.modules['cv2'] = _f
    sys.modules['cv2.ocl'] = _o
from stable_baselines3 import PPO


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL = os.environ.get("JUMP_MODEL",
                       os.path.join(ROOT, "scripts", "jump_ppo_300k.zip"))
CTEST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rl_jump_c_test.c")
NET_DIR = os.path.join(ROOT, "control", "controllers", "rl_jump")


def main():
    model = PPO.load(MODEL)
    rng = np.random.default_rng(20260811)
    n_in = model.observation_space.shape[0]
    n_out = model.action_space.shape[0]
    print(f"模型: 观测 {n_in} 维, 动作 {n_out} 维")
    # 固定样本: 全零 + 随机 + 典型姿态(前仰/后仰/侧倾/抬腿)
    def fixed_obs(vals):
        o = np.zeros(n_in, dtype=np.float32)
        for i, v in enumerate(vals):
            if i < n_in:
                o[i] = v
        return o

    obs_list = [
        np.zeros(n_in, dtype=np.float32),
        rng.normal(0.0, 0.5, n_in).astype(np.float32),
        rng.normal(0.0, 1.0, n_in).astype(np.float32),
        fixed_obs([0.10, 0.0, 0.2, 0.0, 0.23, 0.0, 0.0,
                   0.1, -0.1, 0.1, -0.1, 1.0, -1.0, 0.085, 0.3, 0.2,
                   5.7, 0.0, 0.0, 1.2, 0.3, 0.2, -0.1, 2.0, -2.0,
                   0.4, -0.3, 0.1, -0.1, 0.2, 0.1, -0.1, -0.2]),
        fixed_obs([-0.08, 0.05, -0.15, 0.1, 0.20, 0.2, 0.2,
                   0.2, 0.2, -0.2, -0.2, -1.5, 1.5, 0.12, 0.1, 0.4,
                   -4.6, 2.9, 0.035, 2.4, -0.2, 0.1, 0.3, -3.0, 3.0,
                   0.2, 0.5, -0.3, 0.2, -0.4, 0.1, -0.2, 0.3]),
    ]
    for _ in range(5):
        obs_list.append(rng.normal(0.0, 2.0, n_in).astype(np.float32))

    with open("/tmp/rl_obs_verify.txt", "w") as f:
        for obs in obs_list:
            f.write(" ".join(f"{v:.8e}" for v in obs) + "\n")

    # 参考输出用网络原始均值(不裁剪): PPO.predict 会对动作做 [-1,1] 裁剪,
    # 而训练环境在低通前同样裁剪, 两侧裁剪逻辑一致, 这里只校验网络本身。
    obs_t = torch.as_tensor(np.stack(obs_list))
    dist = model.policy.get_distribution(obs_t)
    refs = dist.distribution.mean.detach().numpy()
    with open("/tmp/rl_ref_py.txt", "w") as f:
        for a in refs:
            f.write(" ".join(f"{v:.8e}" for v in a) + "\n")

    bin_path = os.path.join(tempfile.gettempdir(), "rl_jump_c_test")
    subprocess.run(
        ["gcc", "-O2", "-I", NET_DIR, CTEST,
         os.path.join(NET_DIR, "rl_jump_net.c"), "-lm", "-o", bin_path],
        check=True)
    with open("/tmp/rl_out_c.txt", "w") as f:
        subprocess.run([bin_path, "/tmp/rl_obs_verify.txt", "/tmp/rl_out_c.txt"],
                       check=True)

    c_out = np.loadtxt("/tmp/rl_out_c.txt")
    py_out = np.loadtxt("/tmp/rl_ref_py.txt")
    diff = np.abs(c_out - py_out)
    print(f"样本数={len(obs_list)} 维度={c_out.shape[1]}")
    print(f"最大绝对误差 = {diff.max():.3e}")
    print(f"平均绝对误差 = {diff.mean():.3e}")
    for i, d in enumerate(diff):
        print(f"  sample {i}: py={py_out[i]} c={c_out[i]} diff={d}")
    tol = 1e-4
    if diff.max() < tol:
        print(f"PASS: C 前向与 PPO.predict 一致(误差 < {tol})")
    else:
        print(f"FAIL: 误差超过 {tol}, 检查权重导出/前向实现")
        sys.exit(1)


if __name__ == "__main__":
    main()
