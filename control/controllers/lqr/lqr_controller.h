/*
 * lqr_controller.h - 分段 LQR 控制器
 * ================================================================
 * 基于离线辨识拆分的两个子系统分段模型(按腿长分段):
 *   - pitch 子系统: 7 状态 x 3 输入, 输出 wheel_tau/leg_swing/leg_len
 *   - roll 子系统:  5 状态 x 2 输入, 输出 yaw_tau/leg_len_diff
 * 输出 u = -K * (x - x_trim), 叠加到三环 PID 输出上。
 *
 * 拆分原因: 整体 12 状态模型的耦合通道(wheel_diff_v)辨识 R²≈0.5,
 * 叠加会破坏三环 yaw/roll 稳定(实测 yaw 自转失控); 拆开后 pitch 与
 * roll 各自独立, pitch 子系统 R² 0.92~1.0, 可安全叠加。
 */
#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "sim_state.h"
#include "../controllers.h"

/* 初始化(幂等) */
void lqr_controller_init(void);

/* 每控制步更新: 根据当前 leg_len 选段, 计算 LQR 输出 */
void lqr_controller_update(const SensorState *s, double dt, ControlCmd *out);

/* 当前生效的腿长段索引, 供日志/调试 */
int lqr_controller_segment(void);

/* 当前 pitch/roll 子系统的 LQR 输出(调试用) */
float lqr_controller_pitch_out(void);
float lqr_controller_roll_out(void);

#endif /* LQR_CONTROLLER_H */
