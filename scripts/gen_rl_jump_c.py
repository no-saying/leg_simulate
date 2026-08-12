#!/usr/bin/env python3
"""把训练好的 PPO 跳跃策略导出为 C 头文件(权重数组)。

输出: control/controllers/rl_jump/rl_jump_net.h
      (C 推理见 control/controllers/rl_jump/rl_jump_net.c)
  - 网络: MlpPolicy, 隐层 [256,256], tanh 激活, 输出层无激活
  - 输入/输出维数从策略网络自动读取(与 JumpRLEnv obs/action 一致)
"""
import os
import sys
import types
import numpy as np

# 绕过 opencv/numpy2 冲突
if 'cv2' not in sys.modules:
    _f = types.ModuleType('cv2')
    _o = types.ModuleType('cv2.ocl')
    _o.setUseOpenCL = lambda *a, **k: None
    _f.ocl = _o
    sys.modules['cv2'] = _f
    sys.modules['cv2.ocl'] = _o

from stable_baselines3 import PPO

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL = os.environ.get("JUMP_MODEL", os.path.join(ROOT, "scripts", "jump_ppo_300k.zip"))
OUT_DIR = os.path.join(ROOT, "control", "controllers", "rl_jump")


def fmt_mat(name, w, indent=4):
    """权重矩阵 -> C 数组(按行展平)。"""
    rows = []
    for r in w:
        rows.append("        {" + ",".join(f"{v:.8e}f" for v in r) + "},")
    return "\n".join(rows)


def fmt_vec(name, b, indent=4):
    return "    " + ",".join(f"{v:.8e}f" for v in b) + ","


def main():
    model = PPO.load(MODEL)
    policy = model.policy
    # SB3 MlpPolicy 网络分两段:
    #   mlp_extractor.policy_net = Sequential(Linear(20,256), Tanh, Linear(256,256), Tanh)
    #   action_net               = Linear(256, 2)  (输出层, 无激活)
    # 两段都要导出, 否则 C 侧推理缺输出层。
    net = list(policy.mlp_extractor.policy_net) + [policy.action_net]
    layers = []
    for mod in net:
        if hasattr(mod, "weight"):
            layers.append(("linear", mod.weight.detach().numpy(),
                           mod.bias.detach().numpy()))
        elif mod.__class__.__name__ == "Tanh":
            layers.append(("tanh", None, None))
    if layers[-1][0] != "linear":
        raise SystemExit("FATAL: 最后一层不是 Linear, 请检查网络结构")

    os.makedirs(OUT_DIR, exist_ok=True)
    n_in = layers[0][1].shape[1]
    n_out = layers[-1][1].shape[0]
    with open(os.path.join(OUT_DIR, "rl_jump_net.h"), "w") as f:
        f.write("/* rl_jump_net.h - 由 scripts/gen_rl_jump_c.py 生成, 勿手改 */\n")
        f.write("#ifndef RL_JUMP_NET_H\n#define RL_JUMP_NET_H\n\n")
        f.write(f"#define RL_JUMP_N_IN {n_in}\n#define RL_JUMP_N_OUT {n_out}\n\n")
        # 隐层宽度取第一个 Linear 的输出维, 供 C 前向分配中间层缓冲
        for kind, w, b in layers:
            if kind == "linear":
                f.write(f"#define RL_JUMP_HIDDEN {w.shape[0]}\n")
                break
        f.write("\n")
        lid = 0
        for kind, w, b in layers:
            if kind == "tanh":
                f.write(f"/* layer {lid}: tanh */\n")
                continue
            f.write(f"/* layer {lid}: Linear {w.shape[1]}->{w.shape[0]} */\n")
            f.write(f"static const float RL_W{lid}[{w.shape[0]}][{w.shape[1]}] = {{\n")
            f.write(fmt_mat(f"W{lid}", w) + "\n};\n")
            f.write(f"static const float RL_B{lid}[{w.shape[0]}] = {{\n")
            f.write(fmt_vec(f"B{lid}", b) + "\n};\n")
            lid += 1
        f.write(f"#define RL_JUMP_N_LAYERS {lid}\n")
        f.write("\n#endif /* RL_JUMP_NET_H */\n")
    print(f"导出 {len(layers)} 层(Linear {sum(1 for k,_,_ in layers if k=='linear')}), "
          f"输入 {n_in} 维 -> 输出 {n_out} 维 "
          f"-> {OUT_DIR}/rl_jump_net.h")


if __name__ == "__main__":
    main()
