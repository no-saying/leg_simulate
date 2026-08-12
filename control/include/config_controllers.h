/*
 * config_controllers.h - 控制器切换/LQR/能量整形参数
 * ================================================================
 * 统一存放 controllers/ 目录下的可调参数:
 *   - supervisor : 模式切换阈值与滞回
 *   - lqr        : 输出限幅与启动延迟
 *   - energy     : 能量整形模型常数与增益
 *
 * 由 control_config.h 统一包含, 各实现文件只读这些宏。
 */
#ifndef CONFIG_CONTROLLERS_H
#define CONFIG_CONTROLLERS_H

/* ==========================================================================
 * supervisor: 模式切换
 * ==========================================================================
 * 能量整形负责大 pitch 摆动(轮子仍着地, 轮扭矩可摆回);
 * 大 roll(>25°) 视为侧翻, 轮子离地, 交给 H 键侧翻自救(腿推地)。
 */
#ifndef CTRL_ENERGY_ENTER_DEG
#define CTRL_ENERGY_ENTER_DEG 15.0f  /* |pitch| >= 15° 才判定翻倒(避免跳跃/行驶误触发) */
#endif
#ifndef CTRL_ENERGY_EXIT_DEG
#define CTRL_ENERGY_EXIT_DEG 8.0f    /* |pitch| < 8° 且角速度小才退出 */
#endif
#ifndef CTRL_ENERGY_EXIT_RATE
#define CTRL_ENERGY_EXIT_RATE 2.5f   /* |pitch_rate| < 2.5 rad/s 才退出 */
#endif
#ifndef CTRL_LQR_MAX_DEG
#define CTRL_LQR_MAX_DEG 3.0f        /* |pitch| < 3° 才允许 LQR 叠加 */
#endif
#ifndef CTRL_LQR_LEG_LEN_MIN
#define CTRL_LQR_LEG_LEN_MIN -0.30f  /* LQR 分段覆盖的腿长下限 */
#endif
#ifndef CTRL_LQR_LEG_LEN_MAX
#define CTRL_LQR_LEG_LEN_MAX 0.06f   /* LQR 分段覆盖的腿长上限 */
#endif
#ifndef CTRL_LQR_MIX_GAIN
#define CTRL_LQR_MIX_GAIN 0.3f       /* LQR 扰动叠加增益(0=关闭)。
                                      * 满权重会与腿摆/三环冲突(pitch ±2.6°),
                                      * 0.3 保留 LQR 增强同时稳态 pitch 稳定。 */
#endif
#ifndef CTRL_LQR_STARTUP_DELAY
#define CTRL_LQR_STARTUP_DELAY 2.0f  /* 启动后 LQR 延迟介入, 先让 PID 稳定 */
#endif
#ifndef CTRL_DEBUG
#define CTRL_DEBUG 1                 /* 1=打印模式切换日志, 0=关闭 */
#endif

/* ==========================================================================
 * supervisor 状态机: 前翻/后翻自救
 * ==========================================================================
 * 状态: IDLE(正常) -> PREP(摆腿) -> PUSH(撑地推回) -> SETTLE(回正确认)
 *       -> IDLE; 任一步超时进 ABORT 撤力交 PID 兜底。
 */
#ifndef CTRL_SWING_FWD
#define CTRL_SWING_FWD 0.23f    /* 前翻撑地腿摆(名义) */
#endif
#ifndef CTRL_SWING_BACK
#define CTRL_SWING_BACK -1.0f   /* 后翻撑地腿摆(极限) */
#endif
#ifndef CTRL_SWING_RATE
#define CTRL_SWING_RATE 2.0f    /* 腿摆变化率限幅 rad/s */
#endif
#ifndef CTRL_PUSH_LEN
#define CTRL_PUSH_LEN 0.22f     /* 撑地推回时收腿量(正=收腿/降低重心) */
#endif
#ifndef CTRL_LEN_RATE
#define CTRL_LEN_RATE 1.5f      /* 收腿/放腿变化率限幅 rad/s */
#endif
#ifndef CTRL_PUSH_TAU
#define CTRL_PUSH_TAU 15.0f     /* 撑地推回轮扭矩上限 Nm(慢推防过头) */
#endif
#ifndef CTRL_PUSH_DECAY_START
#define CTRL_PUSH_DECAY_START 0.20f  /* 扭矩衰减起始 |pitch|(rad, ≈11.5°) */
#endif
#ifndef CTRL_PUSH_DECAY_END
#define CTRL_PUSH_DECAY_END 0.35f    /* 扭矩衰减结束 |pitch|(rad, ≈20°), 之上满扭矩 */
#endif
#ifndef CTRL_OVERSHOOT_RAD
#define CTRL_OVERSHOOT_RAD 0.08f     /* 越过竖直反向超过该角度(rad, ≈4.6°)进 SETTLE */
#endif
#ifndef CTRL_TAU_RATE
#define CTRL_TAU_RATE 1200.0f   /* 自救轮扭矩变化率 Nm/s */
#endif
#ifndef CTRL_PREP_TIMEOUT
#define CTRL_PREP_TIMEOUT 1.5f  /* 摆腿超时 s */
#endif
#ifndef CTRL_PUSH_TIMEOUT
#define CTRL_PUSH_TIMEOUT 4.0f  /* 推回超时 s */
#endif
#ifndef CTRL_SETTLE_ENTER_DEG
#define CTRL_SETTLE_ENTER_DEG 25.0f  /* 进入回正确认的 |pitch| */
#endif
#ifndef CTRL_SETTLE_EXIT_DEG
#define CTRL_SETTLE_EXIT_DEG 10.0f   /* 回正完成判定 |pitch| */
#endif
#ifndef CTRL_SETTLE_RATE
#define CTRL_SETTLE_RATE 2.5f        /* 回正完成判定角速度 rad/s */
#endif
#ifndef CTRL_SETTLE_HOLD
#define CTRL_SETTLE_HOLD 0.5f        /* 回正保持时间 s */
#endif
#ifndef CTRL_SETTLE_TIMEOUT
#define CTRL_SETTLE_TIMEOUT 2.0f     /* 回正确认超时 s */
#endif
#ifndef CTRL_SETTLE_BACK_DEG
#define CTRL_SETTLE_BACK_DEG 30.0f   /* SETTLE 中又翻回该角度 -> 回 PUSH */
#endif
#ifndef CTRL_SETTLE_BRAKE
#define CTRL_SETTLE_BRAKE 18.0f       /* 回正阶段角速度阻尼(Nm/(rad/s)) */
#endif

/* ==========================================================================
 * lqr: 输出限幅(防止启动暂态/分段切换时扰动过大)
 * ========================================================================== */
#ifndef LQR_WHEEL_TAU_MAX
#define LQR_WHEEL_TAU_MAX 8.0f
#endif
#ifndef LQR_YAW_TAU_MAX
#define LQR_YAW_TAU_MAX 4.0f
#endif
#ifndef LQR_LEG_SWING_MAX
#define LQR_LEG_SWING_MAX 0.02f
#endif
#ifndef LQR_LEG_LEN_MAX
#define LQR_LEG_LEN_MAX 0.03f
#endif
#ifndef LQR_LEG_DIFF_MAX
#define LQR_LEG_DIFF_MAX 0.02f
#endif

/* ==========================================================================
 * energy: 能量整形模型常数与增益
 * ========================================================================== */
#ifndef ENERGY_MASS
#define ENERGY_MASS 5.0f
#endif
#ifndef ENERGY_G
#define ENERGY_G 9.81f
#endif
#ifndef ENERGY_COM_H
#define ENERGY_COM_H 0.14f
#endif
#ifndef ENERGY_PITCH_INERTIA
#define ENERGY_PITCH_INERTIA 0.035f
#endif
#ifndef ENERGY_KP
#define ENERGY_KP 12.0f
#endif
#ifndef ENERGY_TAU_MAX
#define ENERGY_TAU_MAX 80.0f
#endif
#ifndef ENERGY_RATE_MAX
#define ENERGY_RATE_MAX 1500.0f
#endif

#endif /* CONFIG_CONTROLLERS_H */
