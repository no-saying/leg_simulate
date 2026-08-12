/*
 * controllers.h - 控制器模块总接口
 * ================================================================
 * 三个可切换/叠加的控制器通过统一接口接入:
 *   - pid/    : 现有三环(俯仰PD+速度规划+偏航+roll稳定), 由 balance 核心提供
 *   - lqr/    : 基于离线辨识分段模型的分段 LQR, 输出 u = -K*(x-x_trim)
 *   - energy/ : 能量整形, 大角度摆起/回收(基于机械能量差)
 *   - supervisor/ : 模式切换(正常/小角度LQR/大角度能量整形)与权重混合
 *
 * 架构约定:
 *   1. 控制器之间解耦: 每个控制器只依赖 sensors 输入 + 自己的状态,
 *      不直接调用其它控制器的内部。
 *   2. supervisor 是唯一做"切换"的地方, 其余模块无分支逻辑。
 *   3. 输出统一为 ControlCmd: 共模轮扭矩/差速轮扭矩/腿摆/腿长/腿长差。
 */
#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "sim_state.h"

/* 统一控制输出: 对应辨识模型的 5 个输入通道 */
typedef struct {
    float wheel_tau;      /* 共模轮扭矩 Nm */
    float yaw_tau;        /* 差速轮扭矩 Nm */
    float leg_swing;      /* 腿摆参考(相对名义角) rad */
    float leg_len;        /* 腿长参考(相对名义) rad */
    float leg_len_diff;   /* 腿长差参考 rad */
    int   active;         /* 本控制器是否参与输出 */
} ControlCmd;

/* 控制器模式枚举(由 supervisor 输出) */
typedef enum {
    CTRL_MODE_PID = 0,        /* 三环 PID(默认/兜底) */
    CTRL_MODE_LQR,            /* 分段 LQR(小角度稳定区) */
    CTRL_MODE_ENERGY,         /* 能量整形(大角度摆起) */
    CTRL_MODE_MIXED,          /* PID + LQR 叠加 */
} CtrlMode;

#endif /* CONTROLLERS_H */
