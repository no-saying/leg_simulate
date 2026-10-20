#ifndef RUNTIME_ROBOT_CONTROL_H
#define RUNTIME_ROBOT_CONTROL_H

#include "sim_state.h"

void robot_control_step(SensorState *s, double dt);

/* 回合级重置: 清空低层控制器(LegJoint PID 等)和运行时状态,
 * 供训练环境每回合预热前调用, 避免控制器状态跨回合残留。 */
void robot_control_reset(void);

/* RL 接管期间置 1: supervisor 跳过翻倒自救(部署与训练一致), 其余照常 */
void robot_control_set_rl_active(int on);

/* Spin residual: [dL_diff, dL_common, swing, wheel_common, wheel_yaw],
 * normalized to [-1,1]. Positive dL means a physically longer leg. */
void robot_control_set_spin_residual(const float action[5], int enabled);
void robot_control_get_spin_residual(float action[5]);
/* Native common/yaw, RL common/yaw, common before/after caps, wheel Nm. */
void robot_control_get_spin_mix_audit(float state[8]);
void robot_control_get_spin_reference_audit(float state[12]);

/* Applied left/right extension, automatic height increase and lost differential (m). */
void robot_control_get_leg_allocation(float state[4]);

#endif
