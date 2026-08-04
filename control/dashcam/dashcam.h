/*
 * dashcam.h - 行车记录仪日志模块
 * ================================================================
 *
 * 从 controller.c 抽出，职责只有两件事:
 *   1. 在 logs/ 下生成 dashcam_<时间戳>.csv 和同名 _events.log。
 *   2. 按固定间隔把 SensorState 快照、按键事件写入文件。
 *
 * 本模块不依赖 MuJoCo/GLFW，只依赖 sim_state.h 的数据契约，
 * 与平衡控制核心一样可以独立移植。
 */
#ifndef DASHCAM_H
#define DASHCAM_H

#include "sim_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 打开新的日志文件对(CSV + events)。DASHCAM_HZ 环境变量可覆盖采样间隔。 */
void dashcam_open(void);

/* 关闭并刷新日志文件。 */
void dashcam_close(void);

/* 写一条按键事件: "%.3f %s %s %.3f" = sim_t key action wall_t。 */
void dashcam_log_event(double sim_t, const char *key, const char *action, double wall_t);

/* 写一帧状态快照(按 DASHCAM_HZ 节流)。
 * keys 是当前按住的按键串(如 "WE")，manual_active/hold_pos 来自平衡核心，
 * 由调用方传入，避免日志模块反向依赖控制 API。 */
void dashcam_log_row(const SensorState *s, double wall_t, const char *keys,
                     int manual_active, double hold_pos);

#ifdef __cplusplus
}
#endif

#endif
