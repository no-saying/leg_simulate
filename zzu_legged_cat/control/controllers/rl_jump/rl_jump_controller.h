/*
 * rl_jump_controller.h - RL 跳跃控制器接口(直接力矩控制版)
 * ================================================================
 * 架构定位:
 *   - 训练好的 PPO 策略直接输出全部 6 个执行器的力矩
 *     [lf, lr, 左轮, rf, rr, 右轮], 跳跃期间完全接管, 不再叠加三环。
 *   - 这样三环在跳跃期间不会约束腿/轮(旧叠加方案会限制跳高)。
 *   - 空闲(IDLE)输出 0, 原控制器全权; SPACE 触发后 ACTIVE 接管;
 *     落地恢复/翻车/超时后 FADEOUT 交叉渐变交还三环。
 *
 * 输出 RlJumpCmd:
 *   - active: 1=RL 在管(需要主循环做混合), 0=纯三环
 *   - ctrl[6]: RL 直接力矩(与 s->ctrl 同单位, 范围 ±10)
 *   - mix:     0=纯 RL, 1=纯三环(FADEOUT 期间 0->1 渐变)
 *
 * 观测/动作定义与 scripts/jump_rl_env.py 严格一致(34 维观测 / 5 维动作)。
 */
#ifndef RL_JUMP_CONTROLLER_H
#define RL_JUMP_CONTROLLER_H

#include "sim_state.h"
#include "../controllers.h"

/* RL 跳跃状态机阶段 */
typedef enum {
    RL_JUMP_IDLE = 0,     /* 空闲: RL 不介入, 输出 0 */
    RL_JUMP_PREP,         /* 准备: 收腿到最短, 轮子三环稳住 */
    RL_JUMP_ACTIVE,       /* 跳跃中: RL 直接控制全部执行器 */
    RL_JUMP_FADEOUT,      /* 退出: 与三环交叉渐变, 防切换冲击 */
} RlJumpState;

/* RL 每步输出(直接力矩) */
typedef struct {
    int   active;         /* 1=RL 接管执行器(需要混合), 0=纯三环 */
    float ctrl[6];        /* RL 直接力矩 ctrl: [lf,lr,lw,rf,rr,rw], ±10 */
    float mix[6];         /* 逐通道混合权重: 0=纯 RL, 1=纯三环 */
} RlJumpCmd;

/* 初始化(幂等) */
void rl_jump_controller_init(void);

/* 请求触发一次跳跃(置标志, 下个控制步进入 ACTIVE) */
void rl_jump_controller_trigger(void);

/* 每控制步更新: 构建观测 -> 前向 -> 平滑 -> 输出直接力矩 */
void rl_jump_controller_update(SensorState *s, double dt, RlJumpCmd *out);

/* 当前是否在跳跃/退出中(供 supervisor 跳出自救判定) */
int rl_jump_controller_active(void);

/* 当前状态机阶段(日志/调试) */
int rl_jump_controller_state(void);

/* 纯前向推理(供一致性测试/移植用, 实现在 rl_jump_net.c) */
void rl_jump_net_forward(const float *in, float *out);

#endif /* RL_JUMP_CONTROLLER_H */
