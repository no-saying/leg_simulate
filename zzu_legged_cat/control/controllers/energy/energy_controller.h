/*
 * energy_controller.h - 能量整形控制器
 * ================================================================
 * 大角度(pitch 超出三环/LQR 稳定区)时, 基于机械能量差驱动轮扭矩,
 * 把车体"摆回"竖直平衡点附近, 再交给 LQR/PID 稳定。
 *
 * 能量差: dE = E - E0
 *   E  = 0.5*J*pitch_rate^2 + m*g*h*(cos(pitch)-cos(pitch_ref))
 * 控制律: tau = Kp_e * dE * sign(pitch_rate*cos(pitch))
 *   - 车体下摆(能量不足)时注入能量, 上摆(能量过高)时耗散。
 */
#ifndef ENERGY_CONTROLLER_H
#define ENERGY_CONTROLLER_H

#include "sim_state.h"
#include "../controllers.h"

/* 初始化(幂等) */
void energy_controller_init(void);

/* 每控制步更新: 根据能量差计算轮扭矩输出(只填 wheel_tau) */
void energy_controller_update(const SensorState *s, double dt, ControlCmd *out);

/* 当前能量差(调试/日志), J */
float energy_controller_delta(void);

#endif /* ENERGY_CONTROLLER_H */
