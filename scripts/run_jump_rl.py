#!/usr/bin/env python3
"""实时回放 RL 跳跃策略(MuJoCo 渲染窗口), 循环执行直到关窗。"""
import os
import sys
import types

import numpy as np
import mujoco
import mujoco.viewer
# 绕过 opencv/numpy2 冲突: MlpPolicy 不用 cv2, 只是 SB3 导入链碰它
if 'cv2' not in sys.modules:
    _fake_cv2 = types.ModuleType('cv2')
    _fake_ocl = types.ModuleType('cv2.ocl')
    _fake_ocl.setUseOpenCL = lambda *a, **k: None
    _fake_cv2.ocl = _fake_ocl
    sys.modules['cv2'] = _fake_cv2
    sys.modules['cv2.ocl'] = _fake_ocl
from stable_baselines3 import PPO

from jump_rl_env import JumpRLEnv, XML


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    model_path = os.environ.get("JUMP_MODEL", os.path.join(here, "jump_ppo.zip"))
    if not os.path.exists(model_path):
        print("模型不存在:", model_path)
        return
    model = PPO.load(model_path)
    env = JumpRLEnv()
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)

    def reset_pose():
        d.qpos[:] = env.data.qpos[:]
        d.qvel[:] = env.data.qvel[:]
        d.ctrl[:] = env.data.ctrl[:]
        mujoco.mj_forward(m, d)

    obs, _ = env.reset()
    reset_pose()
    print("RL 跳跃回放启动(循环执行), 关窗退出")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        ep = 0
        while viewer.is_running():
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            d.qpos[:] = env.data.qpos[:]
            d.qvel[:] = env.data.qvel[:]
            d.ctrl[:] = env.data.ctrl[:]
            mujoco.mj_forward(m, d)
            viewer.sync()
            if term or trunc:
                ep += 1
                print(f"回合{ep} 结束: 最高{info['height']:.3f}m "
                      f"pitch={info['pitch_deg']:.0f}° 恢复={info['reached_min']}")
                obs, _ = env.reset()
                reset_pose()


if __name__ == "__main__":
    main()
