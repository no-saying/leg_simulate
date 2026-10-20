/*
 * rl_jump_net.c - RL 跳跃策略 C 前向推理
 * ================================================================
 * 只负责"权重矩阵 -> 前向计算", 不包含任何控制逻辑:
 *
 *   输入 20 维(与 scripts/jump_rl_env.py::_get_obs 完全一致)
 *     h1 = tanh(W0 * obs + B0)         20 -> 256
 *     h2 = tanh(W1 * h1  + B1)         256 -> 256
 *     out = W2 * h2 + B2               256 -> 2 (输出层无激活)
 *
 * 权重来自 scripts/gen_rl_jump_c.py 导出的 rl_jump_net.h(勿手改)。
 * 矩阵存储为 [out][in], 与 PyTorch weight 直接对应。
 */
#include "rl_jump_net.h"

#include <math.h>

void rl_jump_net_forward(const float in[RL_JUMP_N_IN], float out[RL_JUMP_N_OUT])
{
    float h1[RL_JUMP_HIDDEN];
    float h2[RL_JUMP_HIDDEN];

    /* 第 0 层: 20 -> 256 + tanh */
    for (int i = 0; i < RL_JUMP_HIDDEN; i++) {
        float acc = RL_B0[i];
        for (int j = 0; j < RL_JUMP_N_IN; j++)
            acc += RL_W0[i][j] * in[j];
        h1[i] = tanhf(acc);
    }

    /* 第 1 层: 256 -> 256 + tanh */
    for (int i = 0; i < RL_JUMP_HIDDEN; i++) {
        float acc = RL_B1[i];
        for (int j = 0; j < RL_JUMP_HIDDEN; j++)
            acc += RL_W1[i][j] * h1[j];
        h2[i] = tanhf(acc);
    }

    /* 输出层: 256 -> 2, 无激活(与 PPO.predict deterministic 一致) */
    for (int i = 0; i < RL_JUMP_N_OUT; i++) {
        float acc = RL_B2[i];
        for (int j = 0; j < RL_JUMP_HIDDEN; j++)
            acc += RL_W2[i][j] * h2[j];
        out[i] = acc;
    }
}
