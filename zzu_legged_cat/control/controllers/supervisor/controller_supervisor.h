/*
 * controller_supervisor.h - 控制器模式切换与混合
 * ================================================================
 * 唯一负责"何时用哪个控制器"的模块:
 *   - 大角度(|pitch| >= ENERGY_ENTER_DEG): 能量整形把车摆回平衡区
 *   - 小角度且模型通过验收: LQR 接管，腿长只选择线性化参数段
 *   - 其它情况(启动/扰动/腿长越界): 纯 PID 三环兜底
 *
 * 切换带滞回(enter/exit 阈值不同), 避免边界抖动。
 * 输出直接给 robot_control.c 的混控层。
 */
#ifndef CONTROLLER_SUPERVISOR_H
#define CONTROLLER_SUPERVISOR_H

#include "sim_state.h"
#include "../controllers.h"

/* 初始化(幂等) */
void controller_supervisor_init(void);

/* 每控制步: 根据姿态/腿长选模式, 输出最终控制命令 */
void controller_supervisor_update(const SensorState *s, double dt, ControlCmd *out);

/* RL 接管期间置 1: supervisor 跳过"翻倒检测"自救(部署/训练一致),
 * 三环/LQR 轮子仍正常工作, 只有自救状态机不介入。 */
void controller_supervisor_set_rl_busy(int on);
/* Permission only: posture gates and the ordinary transition ramp still apply. */
void controller_supervisor_set_lqr_enabled(int enabled);

/* 当前模式(调试/日志) */
CtrlMode controller_supervisor_mode(void);

/* 当前自救阶段(调试/日志): 0=正常, 1=检测, 2=摆腿, 3=撑地推回, 4=回正确认 */
int controller_supervisor_phase(void);
float controller_supervisor_lqr_mix(void);

#endif /* CONTROLLER_SUPERVISOR_H */
