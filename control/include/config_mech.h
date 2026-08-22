/* config_mech.h - 机械常数 / 跳跃 / 侧翻自救 参数
 * 由 control_config.h 统一包含。 */
#ifndef CONFIG_MECH_H
#define CONFIG_MECH_H

/* ==========================================================================
 * 机械/标定常数
 * ========================================================================== */

#ifndef MODEL_V3
/* v3 = 串联腿闭链轮腿模型(新): 每侧 2 驱动关节(claf0+大腿),
 * 虚拟坐标单位为米(m), 与旧四腿模型的 rad 语义不同。 */
#define MODEL_V3 1
#endif

#ifndef BALANCE_LEG_SW_NOM
/* 腿摆名义值(单位 m, v3 为轮轴前向偏移)。等效静态质心标定点；
 * 调它会同时改变默认站姿和 pitch 平衡偏置。 */
#define BALANCE_LEG_SW_NOM (-0.01596f)
#endif

/* 是否启用历史纯惯性里程计路径。当前主路径不用它，保留用于旧实验复现。 */
#define USE_IMU_ONLY_ODOM 0

/* 轮半径，必须和 MJCF 轮几何半径一致(v3 轮径 86mm)；改错会让轮速/里程计整体比例错误。 */
#define WHEEL_RADIUS 0.043f

/* 轮毂减速比(电机:轮)。必须和 MJCF 轮执行器 gear 一致(当前 10)。
 * 轮上扭矩 = 减速比 * 电机扭矩；电机转速 = 减速比 * 轮转速。
 * 电机转子惯量/阻尼按 N^2 反射到轮轴，已写入 MJCF 轮关节 armature/damping。 */
#ifndef WHEEL_GEAR_RATIO
#define WHEEL_GEAR_RATIO 10.0f
#endif

/* 轮编码器前向符号；由平面轨迹标定得到。改错会让回原点方向反掉。 */
#define WHEEL_ODOM_SIGN (-1.0f)

/* 历史失控跑偏判据阈值，单位 m/s；主路径基本不用，保留给旧观察器分支。 */
#define WHEEL_V_RUNAWAY 2.40f

/* 里程计速度死区，单位 m/s；低于该值认为静止，减小近原点积分漂移。
 * 0.003: 只滤纯数值噪声, 0.02~0.05m/s 的慢漂必须积分,
 * 否则位置环永远看不到漂移(实测 bx 漂 0.7m 而 es 不动)。 */
#define WHEEL_V_DEADZONE 0.003f

/* FIR 低频轮速死区，单位 m/s；用于抗高频抖动的历史分支。 */
#define WHEEL_FIR_DEADZONE 0.08f

/*
 * 腿部运动学线性化系数，单位 m/rad。
 * 用来估计腿摆/腿长变化对轮轴 x 方向速度的贡献，然后从轮编码器速度里扣掉。
 * v3: sw 本身定义为轮轴 x 偏移(m), 腿长变化只动 z 不产生 x 速度。
 */
#define LEG_AXLE_DX_DSW 1.0f
#define LEG_AXLE_DX_DEL 0.0f
#define LEG_AXLE_DX_DER 0.0f

/* 名义轮轴高度，单位 m；pitch_rate * LEG_AXLE_Z_NOM 是俯仰转动引起的等效轮速项。 */
#define LEG_AXLE_Z_NOM  (-0.08656f)

/* 手动腿长控制(E=伸长/升高, Q=缩短/降低)。
 * 符号约定和辨识模型一致: leg_extend 正 -> 实测 leg_len 向 0 靠近(约 -0.06)且车体
 * 降低(z 变小)；leg_extend 负 -> leg_len 更负(约 -0.13)且车体升高。
 * 最大伸出量必须落在 leg_t 每侧 +/-0.016 rad 夹紧范围内。 */
#ifndef MANUAL_LEG_EXTEND_ENABLE
#define MANUAL_LEG_EXTEND_ENABLE 1
#endif
#ifndef MANUAL_LEG_EXTEND_MAX
#define MANUAL_LEG_EXTEND_MAX 0.040f
#endif

#ifndef MANUAL_LEG_EXTEND_RATE
/* 手动腿长目标变化率，单位 rad/s。按住 E/Q 时持续调节，松开保持当前值。
 * v3 单位 m/s: 0.04 m/s -> 全程(±0.04)约 1s。 */
#define MANUAL_LEG_EXTEND_RATE 0.04f
#endif


/* ==========================================================================
 * 空格跳跃(SPACE): 下蹲蓄力 -> 蹬伸起跳 -> 空中收腿 -> 落地缓冲
 * ========================================================================== */
#ifndef MANUAL_JUMP_ENABLE
#define MANUAL_JUMP_ENABLE 1
#endif
#ifndef JUMP_CROUCH_EXT
#define JUMP_CROUCH_EXT  0.15f   /* 下蹲: 腿缩短、车体压低蓄力 */
#endif
#ifndef JUMP_CROUCH_T
#define JUMP_CROUCH_T    0.20f
#endif
#ifndef JUMP_CROUCH_STEP
#define JUMP_CROUCH_STEP 0.0016f /* ~1.6 rad/s 快速下蹲 */
#endif
#ifndef JUMP_LAUNCH_EXT
#define JUMP_LAUNCH_EXT -0.18f   /* 蹬伸: 腿快速伸长把车体弹起 */
#endif
#ifndef JUMP_LAUNCH_T
#define JUMP_LAUNCH_T    0.18f
#endif
#ifndef JUMP_LAUNCH_STEP
#define JUMP_LAUNCH_STEP 0.0200f /* ~20 rad/s 蹬伸 */
#endif
#ifndef JUMP_AIR_EXT
#define JUMP_AIR_EXT    -0.18f   /* 空中保持全伸，姿态更稳 */
#endif
#ifndef JUMP_AIR_T
#define JUMP_AIR_T       0.55f
#endif
#ifndef JUMP_AIR_STEP
#define JUMP_AIR_STEP    0.0020f
#endif
#ifndef JUMP_LAND_EXT
#define JUMP_LAND_EXT    0.06f   /* 落地缓冲: 腿伸长软化触地 */
#endif
#ifndef JUMP_LAND_T
#define JUMP_LAND_T      0.80f
#endif
#ifndef JUMP_LAND_STEP
#define JUMP_LAND_STEP   0.0008f
#endif

#ifndef JUMP_ROLL_KP
/* 跳跃准备/蹬伸阶段的 roll 补偿增益: 车体向一侧倾斜时，加强另一侧腿的
 * 扭矩让起跳左右对称。单位 ctrl/rad。 */
#define JUMP_ROLL_KP 6.0f
#endif

#ifndef JUMP_PITCH_KP
/* 跳跃准备阶段 pitch 补偿增益: 车头低(pitch<0)就前腿少下蹲/多蹬、
 * 后腿多下蹲/少蹬，把姿态摆平再起跳。温和幅度防翻车。 */
#define JUMP_PITCH_KP 4.0f
#endif

#ifndef JUMP_DIR_KP
/* 起跳方向控制: 蹬伸阶段按 IMU 加速度方向修正腿差动，
 * 把起跳加速度拉回竖直(侧向/前后加速度 -> 左右/前后腿差动)。 */
#define JUMP_DIR_KP 0.35f
#endif

#ifndef JUMP_AIR_PITCH_KP
/* 空中姿态: 前后腿差动(swing)按 pitch 修正, 单位 swing/rad。 */
#define JUMP_AIR_PITCH_KP 0.60f
#endif
#ifndef JUMP_AIR_ROLL_KP
/* 空中姿态: 左右腿差动(ext)按 roll 修正, 单位 ext/rad。 */
#define JUMP_AIR_ROLL_KP 0.50f
#endif


/* ==========================================================================
 * 侧翻自救(SELF-RIGHT): 翻车后按 IMU 判定着地侧，打满对应腿把车体推回正
 * ========================================================================== */
#ifndef SELF_RIGHT_ENABLE
#define SELF_RIGHT_ENABLE 1   /* H键手动触发: 腿摆角前后摇摆翻身 */
#endif
#ifndef SELF_RIGHT_ROLL_ON
#define SELF_RIGHT_ROLL_ON  1.15f   /* rad, |roll| 超过进入自救(~66°) */
#endif
#ifndef SELF_RIGHT_PITCH_ON
#define SELF_RIGHT_PITCH_ON 1.05f   /* rad, |pitch| 超过进入自救(~60°) */
#endif
#ifndef SELF_RIGHT_ROLL_OFF
#define SELF_RIGHT_ROLL_OFF 0.80f   /* rad, 回正退出(~46°) */
#endif
#ifndef SELF_RIGHT_PITCH_OFF
#define SELF_RIGHT_PITCH_OFF 0.70f
#endif
#ifndef SELF_RIGHT_TORQUE
#define SELF_RIGHT_TORQUE 4.0f      /* 自救腿扭矩(ctrl, gainprm=50 -> 200Nm) */
#endif
#ifndef SELF_RIGHT_SLEW
#define SELF_RIGHT_SLEW 0.015f      /* 自救扭矩变化率/步(1kHz): 约0.3s推满 */
#endif
#ifndef SELF_RIGHT_SWING
#define SELF_RIGHT_SWING 0.80f      /* 保留: 腿摆角幅度(不再用于自救主逻辑) */
#endif
#ifndef SELF_RIGHT_RETRACT_EXT
#define SELF_RIGHT_RETRACT_EXT 0.25f  /* 自救第一步: 收腿到最短(腿长上限) */
#endif
#ifndef SELF_RIGHT_RETRACT_T
#define SELF_RIGHT_RETRACT_T 0.8f   /* 收腿保持时间(s) */
#endif
#ifndef SELF_RIGHT_RESTORE_EXT
#define SELF_RIGHT_RESTORE_EXT 0.0f /* 自救第二步: 恢复名义腿长 */
#endif
#ifndef SELF_RIGHT_RESTORE_T
#define SELF_RIGHT_RESTORE_T 1.0f   /* 恢复腿长保持时间(s) */
#endif

#ifndef JUMP_TORQUE
/* 跳跃蹬伸阶段腿执行器打满的扭矩幅度(ctrl 单位, gainprm=50 -> Nm)。
 * 4.0=200Nm 斜抛前翻, 2.0=100Nm 弹不起, 3.0=150Nm 折中。 */
#define JUMP_TORQUE 3.0f
#endif

#ifndef JUMP_AIR_PITCH_KP
/* 空中/落地姿态控制: 按 IMU pitch 的 PD，输出前/后腿差动扭矩。 */
#define JUMP_AIR_PITCH_KP 15.0f
#define JUMP_AIR_PITCH_KD 2.0f
#endif
#ifndef JUMP_AIR_ROLL_KP
/* 空中/落地姿态控制: 按 IMU roll 的 PD，输出左/右腿差动扭矩。 */
#define JUMP_AIR_ROLL_KP 15.0f
#define JUMP_AIR_ROLL_KD 2.0f
#endif
#ifndef JUMP_AIR_CTRL_MAX
#define JUMP_AIR_CTRL_MAX 6.0f
#endif

#ifndef JUMP_TRIG_PITCH_MAX
/* 起跳姿态门槛(rad): |pitch| 超过该值拒绝触发，防止从歪斜姿态起跳翻车。 */
#define JUMP_TRIG_PITCH_MAX 0.12f
#endif
#ifndef JUMP_TRIG_ROLL_MAX
/* 起跳姿态门槛(rad): |roll| 超过该值拒绝触发。 */
#define JUMP_TRIG_ROLL_MAX 0.08f
#endif
#ifndef JUMP_TRIG_V_MAX
/* 跳跃触发最大速度(m/s): 车还在明显移动时不允许跳(实测 GUI 里
 * 车运动时按空格, 蹬伸把车甩成侧翻)。 */
#define JUMP_TRIG_V_MAX 0.25f
#endif
#ifndef JUMP_TRIG_RATE_MAX
/* 跳跃触发最大俯仰角速度(rad/s)。 */
#define JUMP_TRIG_RATE_MAX 1.0f
#endif

#ifndef JUMP_LAND_KP
/* 软着陆虚拟弹簧刚度(低=柔，触地让位)。 */
#define JUMP_LAND_KP 1.5f
#endif
#ifndef JUMP_LAND_KD
/* 软着陆虚拟阻尼(高=吸能，吸收触地冲击)。 */
#define JUMP_LAND_KD 0.12f
#endif
#ifndef JUMP_LAND_CTRL_MAX
#define JUMP_LAND_CTRL_MAX 4.0f    /* 软着陆扭矩上限(ctrl) */
#endif
#ifndef JUMP_LAND_LOAD
#define JUMP_LAND_LOAD 18.0f       /* 触地判定: 四腿总负载(Nm)超过即软腿 */
#endif

#endif
