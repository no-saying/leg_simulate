/* config_lowlevel.h - 低层执行器 / GUI调试 / 历史测试 参数
 * 由 control_config.h 统一包含。 */
#ifndef CONFIG_LOWLEVEL_H
#define CONFIG_LOWLEVEL_H

/* ==========================================================================
 * runtime/robot_control.c 低层输出参数
 * ========================================================================== */

#ifndef LEG_ANGLE_KP
/* 四腿关节角度伺服 P 增益。实测 15.0 会让腿伺服变硬、
 * 与位置保持耦合导致车漂移 ±2m 且腿像泄力, 恢复原 4.0。 */
#if MODEL_V3
#define LEG_ANGLE_KP 50.0f  /* v3 闭链腿: 高刚度才不被轮驱反作用力推离 */
#else
#define LEG_ANGLE_KP 4.0f
#endif
#endif

#ifndef LEG_ANGLE_KI
/* 四腿关节角度伺服 I 增益。用于消除静差，过大会慢性振荡。 */
#if MODEL_V3
#define LEG_ANGLE_KI 0.05f  /* v3: 积分减小, 防积分饱和甩腿 */
#else
#define LEG_ANGLE_KI 0.90f
#endif
#endif

#ifndef LEG_ANGLE_KD
/* 四腿关节角度伺服 D 增益。用于抑制腿关节速度，过大会放大速度噪声。 */
#if MODEL_V3
#define LEG_ANGLE_KD 0.60f  /* v3 轻机构: 需要速度阻尼压住 200Hz 采样极限环 */
#else
#define LEG_ANGLE_KD 0.045f
#endif
#endif

#ifndef LEG_ANGLE_MAX
/* 腿关节 PID 输出限幅，不是腿角参考限幅。 */
#if MODEL_V3
#define LEG_ANGLE_MAX 10.0f
#else
#define LEG_ANGLE_MAX 8.0f
#endif
#endif

#ifndef LEG_ANGLE_DEADBAND
/* 腿关节角误差死区，单位 rad；必须明显小于手动腿长指令幅度，
 * 否则 E/Q 的腿长目标偏移会被死区吞掉(实测 0.012 死区 = 0.012 指令时不响应)。 */
#define LEG_ANGLE_DEADBAND 0.003f
#endif

#ifndef LEG_ANGLE_OUT_LPF
/* 腿关节 PID 输出低通系数/时间项；当前 0 表示不额外滤波。 */
#define LEG_ANGLE_OUT_LPF 0.0f
#endif

#ifndef LEG_ANGLE_IMPROVE
/* PID 微分模式。默认对测量微分，减少设定值阶跃造成的微分冲击。 */
#define LEG_ANGLE_IMPROVE PID_Derivative_On_Measurement
#endif

#ifndef WHEEL_MIX_BALANCE_WEIGHT
/* balance.c 输出的俯仰共模扭矩最终缩放权重。 */
#if MODEL_V3
#define WHEEL_MIX_BALANCE_WEIGHT 1.0f   /* v3 轻车: 俯仰扭矩全量直出 */
#else
#define WHEEL_MIX_BALANCE_WEIGHT 0.60f
#endif
#endif

#ifndef WHEEL_MIX_YAW_WEIGHT
/* balance.c 输出的偏航差动扭矩最终缩放权重。 */
#define WHEEL_MIX_YAW_WEIGHT 0.40f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_ENABLE
/* 输出层最终轮速保护开关；只修正俯仰共模扭矩，不动偏航差动项。 */
#if MODEL_V3
#define WHEEL_OUTPUT_SPEED_CAP_ENABLE 0
#else
#define WHEEL_OUTPUT_SPEED_CAP_ENABLE 1
#endif
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_START
/* 输出层轮速保护启动阈值，单位 m/s。 */
#define WHEEL_OUTPUT_SPEED_CAP_START 3.0f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_KP
/* 输出层轮速保护刹车增益，单位 Nm/(m/s)。 */
#define WHEEL_OUTPUT_SPEED_CAP_KP 4.0f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_MIN_TAU
/* 输出层轮速保护最小反向扭矩，单位 Nm。 */
#define WHEEL_OUTPUT_SPEED_CAP_MIN_TAU 1.5f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_MAX_TAU
/* 输出层轮速保护最大反向扭矩，单位 Nm。 */
#define WHEEL_OUTPUT_SPEED_CAP_MAX_TAU 12.0f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_HOLD_KP
/* 输出层轮速持续超限时，保持时间到额外刹车扭矩的增益。 */
#define WHEEL_OUTPUT_SPEED_CAP_HOLD_KP 2.0f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_HOLD_MAX
/* 输出层轮速超限 hold 时间最大值，单位 s。 */
#define WHEEL_OUTPUT_SPEED_CAP_HOLD_MAX 4.0f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_DECAY
/* 输出层轮速恢复后 hold 时间衰减速度，单位 s/s。 */
#define WHEEL_OUTPUT_SPEED_CAP_DECAY 3.0f
#endif

#ifndef WHEEL_OUTPUT_SPEED_CAP_PITCH_GATE
/* 只有 |pitch| 小于该值时才累积输出层轮速保护保持时间，单位 rad。 */
#define WHEEL_OUTPUT_SPEED_CAP_PITCH_GATE 0.14f
#endif


/* ==========================================================================
 * controller.c 调试/开环测试参数
 * ========================================================================== */

#ifndef OPEN_LOOP_WHEEL_TEST
/* 开环轮输出测试开关。开启后绕过 balance 轮输出，只写下面两个控制量。 */
#define OPEN_LOOP_WHEEL_TEST 0
#endif

#ifndef OPEN_LOOP_WHEEL_CTRL_L
/* 开环左轮执行器控制量，只有 OPEN_LOOP_WHEEL_TEST=1 时有效。 */
#define OPEN_LOOP_WHEEL_CTRL_L 0.0
#endif

#ifndef OPEN_LOOP_WHEEL_CTRL_R
/* 开环右轮执行器控制量，只有 OPEN_LOOP_WHEEL_TEST=1 时有效。 */
#define OPEN_LOOP_WHEEL_CTRL_R 0.0
#endif


/* ==========================================================================
 * 历史无界面测试参数
 * ========================================================================== */

#ifndef RUN_HEADLESS_TEST_DT
/* 历史无界面测试文件使用的打印时间步长，单位 s；当前主入口不使用该文件。 */
#define RUN_HEADLESS_TEST_DT 0.001
#endif

#ifndef LEG_V3_SERVO_KP
/* v3 直接腿位置伺服: ctrl = KP*(ref-pos) - KD*vel, 1kHz。
 * 实测硬锁(80)让伺服饱和打满、机体高频振动触发 impact 门控全关。
 * v3 用软保持(自由腿即能平衡), 只防塌陷到极限。 */
#define LEG_V3_SERVO_KP 80.0f
#define LEG_V3_SERVO_KD 4.0f
#endif

#endif
