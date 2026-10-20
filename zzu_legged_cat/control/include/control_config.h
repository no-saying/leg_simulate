#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/*
 * control_config.h - 全工程控制参数总表(汇总头)
 * ================================================================
 *
 * 原 1300 行单一配置头按功能拆成 6 个子头，本文件只做统一包含，
 * 所有实现文件保持 #include "control_config.h" 不变。
 *
 * 子头文件:
 *   - config_mech.h      : 机械常数 / 跳跃 / 侧翻自救
 *   - config_balance.h   : 平衡路径开关 / 俯仰 / 偏航 / roll 稳定
 *   - config_safety.h    : 近原点停车 / 远场刹车 / 轮速保护
 *   - config_odom.h      : 里程计积分 / 中立角 / 速度规划
 *   - config_manual.h    : 手动驾驶 / 离线辨识 / 旧原点控制
 *   - config_lowlevel.h  : 低层执行器 / GUI调试 / 历史测试
 *
 * 单位约定:
 *   - 角度: rad，日志显示时才转 deg。
 *   - 角速度: rad/s。
 *   - 速度: m/s。
 *   - 加速度: m/s^2。
 *   - 扭矩: Nm，最终写入 MuJoCo actuator 前会在 runtime/robot_control.c 缩放。
 *   - 时间: s。
 */

#include "config_mech.h"
#include "config_balance.h"
#include "config_safety.h"
#include "config_odom.h"
#include "config_manual.h"
#include "config_lowlevel.h"
#include "config_controllers.h"

#endif
