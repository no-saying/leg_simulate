/* config_safety.h - 近原点停车 / 远场刹车 / 轮速保护 参数
 * 由 control_config.h 统一包含。 */
#ifndef CONFIG_SAFETY_H
#define CONFIG_SAFETY_H

/* ==========================================================================
 * 近原点停车、高速空转、轮速保护
 * ========================================================================== */

#ifndef SIMPLE_PARK_VEL_DAMP
/* 近原点停车速度阻尼，单位 Nm/(m/s)。**置 0 停用**。
 * 它和 SIMPLE_VEL_DAMP 是同一个 −K·v 形式的共模轮扭矩项: 两个都是 4
 * 时总增益 8, 叠加 12ms 轮速滤波的相位滞后后这个"速度->俯仰"回路不
 * 再稳定——实测关掉小陀螺/松手的瞬间车会以 3m/s 冲出去(13s 前进
 * 9.6m, 用户反馈"取消陀螺会高速漂移刹不住车")。
 * 只留 SIMPLE_VEL_DAMP=4、park 置 0 后同一工况只前进 0.15m。
 * 静止慢漂两种设置实测都是 0.000m/s, park 没有额外贡献。 */
#define SIMPLE_PARK_VEL_DAMP 0.0f
#endif

#ifndef SIMPLE_SPIN_DAMP
/* 历史高速空转阻尼项。默认 0，不参与当前主路径。 */
#define SIMPLE_SPIN_DAMP 0.0f
#endif

#ifndef SIMPLE_WHEEL_GOV_ENABLE
/* 轮速监管器开关。只削弱“继续加速同方向”的扭矩，保留能刹车的扭矩。 */
#if MODEL_V3
#define SIMPLE_WHEEL_GOV_ENABLE 0   /* v3 调参期: 关掉轮速限制器 */
#else
#define SIMPLE_WHEEL_GOV_ENABLE 1
#endif
#endif

#ifndef SIMPLE_WHEEL_GOV_START
/* 轮速监管器介入阈值，单位 m/s。 */
#define SIMPLE_WHEEL_GOV_START 1.20f
#endif

#ifndef SIMPLE_WHEEL_GOV_KP
/* 轮速超过阈值后的允许扭矩下降斜率，单位 Nm/(m/s)。 */
#define SIMPLE_WHEEL_GOV_KP 6.0f
#endif

#ifndef SIMPLE_WHEEL_GOV_TAU
/* 轮速监管器基准允许扭矩，单位 Nm。 */
#define SIMPLE_WHEEL_GOV_TAU 46.0f
#endif

#ifndef SIMPLE_WHEEL_GOV_MIN_TAU
/* 轮速监管器最低仍允许的同向驱动扭矩，单位 Nm。 */
#define SIMPLE_WHEEL_GOV_MIN_TAU 0.0f
#endif

#ifndef SIMPLE_WHEEL_BRAKE_START
/* 主动轮速刹车阈值，单位 m/s；当前 MAX=0，所以默认不额外刹车。 */
#define SIMPLE_WHEEL_BRAKE_START 2.0f
#endif

#ifndef SIMPLE_WHEEL_BRAKE_KP
/* 主动轮速刹车增益，单位 Nm/(m/s)。 */
#define SIMPLE_WHEEL_BRAKE_KP 4.0f
#endif

#ifndef SIMPLE_WHEEL_BRAKE_MAX
/* 主动轮速刹车最大扭矩，单位 Nm；0 表示关闭该项。 */
#define SIMPLE_WHEEL_BRAKE_MAX 0.0f
#endif

#ifndef SIMPLE_WHEEL_BRAKE_PITCH_RAD
/* 只有 |pitch| 小于该值时才允许主动轮速刹车，避免大角度救车时反向抢扭矩。 */
#define SIMPLE_WHEEL_BRAKE_PITCH_RAD 0.20f
#endif


/* ==========================================================================
 * 远离原点/高速远场刹车
 * ========================================================================== */

#ifndef SIMPLE_FAR_BRAKE_POS
/* 远场刹车位置阈值，单位 m；|x| 超过该值才考虑远场刹车。 */
#define SIMPLE_FAR_BRAKE_POS 0.50f
#endif

#ifndef SIMPLE_FAR_BRAKE_V
/* 远场刹车速度阈值，单位 m/s。 */
#define SIMPLE_FAR_BRAKE_V 0.60f
#endif

#ifndef SIMPLE_FAR_BRAKE_SPIN_V
/* 即使不远离原点，机体速度超过该值也可触发高速刹车逻辑。 */
#define SIMPLE_FAR_BRAKE_SPIN_V 2.50f
#endif

#ifndef SIMPLE_FAR_BRAKE_KP
/* 远场刹车增益，单位 Nm/(m/s)。 */
#define SIMPLE_FAR_BRAKE_KP 4.0f
#endif

#ifndef SIMPLE_FAR_BRAKE_MAX
/* 远场刹车最大扭矩，单位 Nm；当前为 0，保留入口但不启用。 */
#define SIMPLE_FAR_BRAKE_MAX 0.0f
#endif

#ifndef SIMPLE_FAR_BRAKE_PITCH_RAD
/* 远场刹车允许的最大 |pitch|，单位 rad。 */
#define SIMPLE_FAR_BRAKE_PITCH_RAD 0.55f
#endif


/* ==========================================================================
 * 截获前倾/轮速截获历史实验项
 * ========================================================================== */

#ifndef SIMPLE_CAPTURE_LEAN_ENABLE
/* 高速时直接把目标俯仰角推向速度截获方向。默认关闭，避免放大打滑误判。 */
#define SIMPLE_CAPTURE_LEAN_ENABLE 0
#endif

#ifndef SIMPLE_CAPTURE_LEAN_V
/* 截获前倾速度触发阈值，单位 m/s。 */
#define SIMPLE_CAPTURE_LEAN_V 2.50f
#endif

#ifndef SIMPLE_CAPTURE_LEAN_STEP
/* 截获前倾每次给目标俯仰角的偏移步长，单位 rad。 */
#define SIMPLE_CAPTURE_LEAN_STEP 0.090f
#endif

#ifndef SIMPLE_CAPTURE_LEAN_MAX
/* 截获前倾允许的最大目标俯仰角，单位 rad。 */
#define SIMPLE_CAPTURE_LEAN_MAX 0.520f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_ENABLE
/* 轮速截获状态机开关。默认关闭；开启后会锁定高速方向并主动刹车。 */
#define SIMPLE_WHEEL_CAPTURE_ENABLE 0
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_ENTER_V
/* 进入轮速截获的速度阈值，单位 m/s。 */
#define SIMPLE_WHEEL_CAPTURE_ENTER_V 2.40f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_EXIT_V
/* 退出轮速截获的速度阈值，单位 m/s。 */
#define SIMPLE_WHEEL_CAPTURE_EXIT_V 0.45f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_OFFSET_MIN
/* 轮速截获给目标俯仰角的最小偏置，单位 rad。 */
#define SIMPLE_WHEEL_CAPTURE_OFFSET_MIN 0.075f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_OFFSET_KV
/* 轮速截获偏置随速度增加的增益，单位 rad/(m/s)。 */
#define SIMPLE_WHEEL_CAPTURE_OFFSET_KV 0.030f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_OFFSET_MAX
/* 轮速截获目标俯仰角偏置最大值，单位 rad。 */
#define SIMPLE_WHEEL_CAPTURE_OFFSET_MAX 0.180f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_NEUTRAL_MAX
/* 轮速截获目标俯仰角绝对限幅，单位 rad。 */
#define SIMPLE_WHEEL_CAPTURE_NEUTRAL_MAX 0.620f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_PITCH_RAD
/* 允许轮速截获的最大 |pitch|，单位 rad。 */
#define SIMPLE_WHEEL_CAPTURE_PITCH_RAD 0.780f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_TAU_MIN
/* 轮速截获反向刹车最小扭矩，单位 Nm。 */
#define SIMPLE_WHEEL_CAPTURE_TAU_MIN 3.0f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_TAU_KV
/* 轮速截获刹车扭矩随速度增加的增益，单位 Nm/(m/s)。 */
#define SIMPLE_WHEEL_CAPTURE_TAU_KV 2.0f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_TAU_MAX
/* 轮速截获反向刹车最大扭矩，单位 Nm。 */
#define SIMPLE_WHEEL_CAPTURE_TAU_MAX 16.0f
#endif

#ifndef SIMPLE_WHEEL_CAPTURE_RATE
/* 轮速截获模式下允许的扭矩变化率，单位 Nm/s。 */
#define SIMPLE_WHEEL_CAPTURE_RATE 900.0f
#endif


/* ==========================================================================
 * 打滑检测、打滑刹车、控制用轮速预处理
 * ========================================================================== */

#ifndef SIMPLE_SLIP_BRAKE_ENABLE
/* 打滑时主动反向刹车。默认关闭，当前只做保守里程计和轮速监管。 */
#define SIMPLE_SLIP_BRAKE_ENABLE 0
#endif

#ifndef SIMPLE_SLIP_BRAKE_START
/* 打滑刹车速度阈值，单位 m/s。 */
#define SIMPLE_SLIP_BRAKE_START 3.0f
#endif

#ifndef SIMPLE_SLIP_BRAKE_KP
/* 打滑刹车增益，单位 Nm/(m/s)。 */
#define SIMPLE_SLIP_BRAKE_KP 3.0f
#endif

#ifndef SIMPLE_SLIP_BRAKE_MAX
/* 打滑刹车最大扭矩，单位 Nm。 */
#define SIMPLE_SLIP_BRAKE_MAX 12.0f
#endif

#ifndef SIMPLE_SLIP_BRAKE_PITCH_RAD
/* 只有 |pitch| 小于该值时才允许打滑刹车，单位 rad。 */
#define SIMPLE_SLIP_BRAKE_PITCH_RAD 0.42f
#endif

#ifndef SIMPLE_WHEEL_CTRL_V_MAX
/* 控制用轮速输入硬限幅，单位 m/s。防止编码器尖峰直接进入控制。 */
#define SIMPLE_WHEEL_CTRL_V_MAX 30.0f
#endif

#ifndef SIMPLE_WHEEL_CTRL_DV_MAX
/* 控制用轮速每 1 ms 最大变化量，单位 m/s per 1ms；dt 变大时按比例放大。 */
#define SIMPLE_WHEEL_CTRL_DV_MAX 0.50f
#endif

#ifndef SIMPLE_SLIP_ODOM_V
/* 打滑时仍允许积分的一点保守里程计速度，单位 m/s。 */
#define SIMPLE_SLIP_ODOM_V 0.08f
#endif

#ifndef SIMPLE_SLIP_RESIDUAL
/* 打滑残差阈值，单位 m/s^2；|wheel_acc - imu_acc| 超过它认为轮速不可信。 */
#define SIMPLE_SLIP_RESIDUAL 18.0f
#endif

#ifndef SIMPLE_SLIP_LOW_V
/* 低速打滑判据的"机体速度"阈值，单位 m/s。
 * 旧判据用 |imu_acc|>0.1m/s^2 当证据: IMU 加速度的偏置/腿摆噪声
 * 轻松超过 0.1, 静止时 |轮速|<0.15 又恒成立, 于是打滑标志一上电就
 * 锁死。打滑标志会直接把腿摆指令清零, 表现为"腿摆摆一下又回去了 /
 * 按 S 不走"(用户实测)。改成用融合速度 simple_imu_v 做证据:
 * 只有"机体确实在动(>0.2m/s)而轮子不转"才是真正的抱死滑动。 */
#define SIMPLE_SLIP_LOW_V 0.20f
#endif

/* Speed evidence in m/s, after leg and yaw compensation. Timers use seconds. */
#ifndef SIMPLE_SLIP_V_ENTER
#define SIMPLE_SLIP_V_ENTER 0.22f
#endif
#ifndef SIMPLE_SLIP_A_ENTER
#define SIMPLE_SLIP_A_ENTER 1.5f
#endif
#ifndef SIMPLE_SLIP_YAW_V_ENTER
#define SIMPLE_SLIP_YAW_V_ENTER 0.22f
#endif
#ifndef SIMPLE_SLIP_ENTER_T
#define SIMPLE_SLIP_ENTER_T 0.050f
#endif
#ifndef SIMPLE_SLIP_WHEEL_V_ENTER
#define SIMPLE_SLIP_WHEEL_V_ENTER 0.32f
#endif
#ifndef SIMPLE_SLIP_FAST_V
#define SIMPLE_SLIP_FAST_V 0.70f
#endif
#ifndef SIMPLE_SLIP_FAST_T
#define SIMPLE_SLIP_FAST_T 0.012f
#endif
#ifndef SIMPLE_SLIP_EXIT_T
#define SIMPLE_SLIP_EXIT_T 0.150f
#endif
#ifndef SIMPLE_SLIP_IMU_HORIZON
#define SIMPLE_SLIP_IMU_HORIZON 0.60f
#endif

/* Each wheel is compared against IMU/base motion after yaw/leg compensation. */
#define WHEEL_OBSTACLE_V_ENTER 0.30f
#define WHEEL_OBSTACLE_V_EXIT  0.16f
#define WHEEL_OBSTACLE_V_MIN   0.10f
#define WHEEL_OBSTACLE_A_ENTER 6.0f
#define WHEEL_OBSTACLE_ENTER_T 0.020f
#define WHEEL_OBSTACLE_EXIT_T  0.150f

#endif
