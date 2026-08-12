#ifndef RUNTIME_ROBOT_CONTROL_H
#define RUNTIME_ROBOT_CONTROL_H

#include "sim_state.h"

void robot_control_step(SensorState *s, double dt);

/* 回合级重置: 清空低层控制器(LegJoint PID 等)和运行时状态,
 * 供训练环境每回合预热前调用, 避免控制器状态跨回合残留。 */
void robot_control_reset(void);

/* RL 接管期间置 1: supervisor 跳过翻倒自救(部署与训练一致), 其余照常 */
void robot_control_set_rl_active(int on);

#endif
