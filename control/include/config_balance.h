/* config_balance.h - 平衡主路径开关 / 俯仰 / 偏航 / roll 稳定 参数
 * 由 control_config.h 统一包含。 */
#ifndef CONFIG_BALANCE_H
#define CONFIG_BALANCE_H

/* ==========================================================================
 * balance.c 主控制路径开关
 * ========================================================================== */

#ifndef USE_SIMPLE_THREE_LOOP
/* 当前主路径总开关: 俯仰 PD + 原点速度规划 + 偏航差动。关掉会进入旧观察器分支。 */
#define USE_SIMPLE_THREE_LOOP 1
#endif

#ifndef SIMPLE_PURE_BALANCE
/* 三环路径里的简化稳定版本。当前默认开启；关闭后会进入更长的历史恢复逻辑。 */
#define SIMPLE_PURE_BALANCE 1
#endif

#ifndef SIMPLE_ORIGIN_ENABLE
/* 是否让里程计误差参与目标俯仰角规划。关闭后只做平衡和速度阻尼，不主动回原点。 */
#if MODEL_V3
#define SIMPLE_ORIGIN_ENABLE 1   /* v3: 位置环驱动腿摆(腿要有动作) */
#else
#define SIMPLE_ORIGIN_ENABLE 1
#endif
#endif

#ifndef SIMPLE_HOLD_AT_POINT
/* 无手动指令时远场(>0.18m)行为:
 *   1 = 保持当前点(速度归零，不追原点)——符合“没发指令就停在那”的需求；
 *   0 = 旧行为，远场回原点(会造成来回振荡和持续漂移)。 */
#define SIMPLE_HOLD_AT_POINT 1
#endif


/* ==========================================================================
 * 俯仰高频平衡环
 * ========================================================================== */

#ifndef SIMPLE_PITCH_KP
/* 俯仰角度 P 增益。加入 10:1 减速后轮惯量反射(armature 0.052)是原来的
 * 25 倍，原 240 会救不住倾角、车直接漂走；实测 800 + 扭矩上限 24 可稳定。
 * v3 新模型(轮轴同向、安装矩阵补偿)俯仰符号与旧模型相反:
 * 扫参实测 tau = -KP*pitch - KD*pitch_rate 才能稳定站立。 */
#if MODEL_V3
#define SIMPLE_PITCH_KP (-40.0f)
#else
#define SIMPLE_PITCH_KP 240.0f
#endif
#endif

#ifndef SIMPLE_PITCH_KD
/* 俯仰角速度 D 增益。调大: 阻尼更强，压掉目标俯仰低周摇摆引起的
 * pitch 过冲(实测 tp=2.4° 时 pitch 冲到 3.1°)；过大: 放大陀螺噪声。 */
#if MODEL_V3
#define SIMPLE_PITCH_KD (-12.0f)
#else
#define SIMPLE_PITCH_KD 85.0f
#endif
#endif

#ifndef SIMPLE_PITCH_KI_NM
/* 俯仰静差积分增益，单位 Nm/(rad*s)。
 * 小增益(5)把静止时的残倾角(+2~3°)缓慢拉回 0，消除“后仰前栽”
 * 慢爬——残倾角是长时漂移→翻车的根源。大增益(30)会 windup 反而
 * 激起振荡，所以取小值；|pitch|>0.15 时清零避免救车时打架。 */
#define SIMPLE_PITCH_KI_NM 0.0f
#endif

#ifndef SIMPLE_PITCH_RATE_TAU
/* 俯仰角速度一阶低通时间常数。0=不过滤；实测 8~20ms 滤波都会
 * 拖垮启动暂态(0.5s 内翻车)，保持不过滤，打滑抖振靠轮速监管处理。 */
#define SIMPLE_PITCH_RATE_TAU 0.0f
#endif

#ifndef SIMPLE_GAIN_SCHED_ENABLE
/* 是否启用近/远段俯仰增益调度。默认关，避免切换尖峰污染当前稳定路径。 */
#define SIMPLE_GAIN_SCHED_ENABLE 0
#endif

#ifndef SIMPLE_GAIN_NEAR_RAD
/* 增益调度近端误差阈值，单位 rad；小于该误差使用 NEAR 增益。 */
#define SIMPLE_GAIN_NEAR_RAD 0.035f
#endif

#ifndef SIMPLE_GAIN_FAR_RAD
/* 增益调度远端误差阈值，单位 rad；大于该误差使用标准远端增益。 */
#define SIMPLE_GAIN_FAR_RAD 0.140f
#endif

#ifndef SIMPLE_PITCH_KP_NEAR
/* 近原点小误差 Kp；默认等于 SIMPLE_PITCH_KP，便于只覆盖近端参数。 */
#define SIMPLE_PITCH_KP_NEAR SIMPLE_PITCH_KP
#endif

#ifndef SIMPLE_PITCH_KD_NEAR
/* 近原点小误差 Kd；默认等于 SIMPLE_PITCH_KD。 */
#define SIMPLE_PITCH_KD_NEAR SIMPLE_PITCH_KD
#endif

#ifndef SIMPLE_VEL_DAMP
/* 机体前向速度阻尼，单位 Nm/(m/s)，直接作用在共模轮扭矩上。
 * 速度刹车统一走速度环 -> 目标俯仰角链路，俯仰环用满扭矩去执行，
 * 这里的直连阻尼只保留很小值防高频抖振，避免与抬身扭矩反向抵消
 * (漂移 1m/s 时 5Nm 前驱刹车 vs 俯仰误差 -5Nm 抬身, 净扭矩≈0)。
 * 抬身瞬间该阻尼按俯仰误差自动退让, 见 balance_step_pitch_yaw.inc。 */
#if MODEL_V3
#define SIMPLE_VEL_DAMP 2.0f   /* v3: 轮速符号已修正, 恢复速度阻尼 */
#else
#define SIMPLE_VEL_DAMP 1.5f
#endif
#endif

#ifndef SIMPLE_NEUTRAL_VEL
/* 无原点规划时速度到目标俯仰角的微小偏置系数。 */
#define SIMPLE_NEUTRAL_VEL -0.100f
#endif

#ifndef SIMPLE_BASE_NEUTRAL
/* 目标俯仰角基准偏置，单位 rad；机械质心偏差优先调 BALANCE_LEG_SW_NOM。 */
#if MODEL_V3
#define SIMPLE_BASE_NEUTRAL 0.0f
#else
#define SIMPLE_BASE_NEUTRAL 0.0f
#endif
#endif


/* ==========================================================================
 * 俯仰输出限幅/斜率
 * ========================================================================== */

#ifndef SIMPLE_TAU_START
/* 启动早期共模扭矩限幅，单位 Nm；重轮下 72 会让轮子打滑级联，降到 24。 */
#if MODEL_V3
#define SIMPLE_TAU_START 8.0f    /* v3: 基础平衡定型 */
#else
#define SIMPLE_TAU_START 72.0f
#endif
#endif

#ifndef SIMPLE_TAU_RUN
/* 正常运行共模扭矩限幅，单位 Nm。 */
#if MODEL_V3
#define SIMPLE_TAU_RUN 8.0f
#else
#define SIMPLE_TAU_RUN 72.0f
#endif
#endif

#ifndef SIMPLE_RATE_START
/* 启动早期扭矩变化率限制，单位 Nm/s；越小越柔，越大响应越猛。 */
#if MODEL_V3
#define SIMPLE_RATE_START 50000.0f   /* v3 轻车: 等效无速率限制 */
#else
#define SIMPLE_RATE_START 420.0f
#endif
#endif

#ifndef SIMPLE_RATE_RUN
/* 正常运行扭矩变化率限制，单位 Nm/s。 */
#if MODEL_V3
#define SIMPLE_RATE_RUN 50000.0f
#else
#define SIMPLE_RATE_RUN 420.0f
#endif
#endif

#ifndef SIMPLE_BIG_RECOVERY
/* 大角度恢复实验开关。默认关；开启会在大俯仰角下临时改变 Kp/Kd/限幅。 */
#define SIMPLE_BIG_RECOVERY 0
#endif


/* ==========================================================================
 * 偏航差动控制
 * ========================================================================== */

#ifndef SIMPLE_YAW_SIGN
/* 偏航输出符号修正。若偏航越控越偏，首先检查这个符号。 */
#if MODEL_V3
#define SIMPLE_YAW_SIGN -1.0f   /* v3 标定: 差速左大->yaw 负方向 */
#else
#define SIMPLE_YAW_SIGN 1.0f
#endif
#endif

#ifndef SIMPLE_YAW_KP
/* 偏航位置环 P 增益，生成偏航角速度参考。 */
#define SIMPLE_YAW_KP 1.20f
#endif

#ifndef SIMPLE_YAW_KD
/* 偏航角速度阻尼增益，作用到差动轮扭矩。 */
#define SIMPLE_YAW_KD 0.80f
#endif

#ifndef SIMPLE_YAW_KI
/* 偏航位置积分增益，用于慢慢消除长期偏航误差。 */
#define SIMPLE_YAW_KI 0.08f
#endif

#ifndef SIMPLE_YAW_MAX
/* 偏航差动扭矩最大值，单位 Nm；最终还会被俯仰扭矩余量门控。
 * v3 新模型差速/陀螺符号未标定前先关闭, 防止差速正反馈把车甩翻。 */
#if MODEL_V3
#define SIMPLE_YAW_MAX 0.0f   /* 先验证纯 pitch */
#else
#define SIMPLE_YAW_MAX 1.2f
#endif
#endif

#ifndef SIMPLE_YAW_I_MAX
/* 偏航积分限幅，单位 rad*s；防止长时间饱和后反向过冲。 */
#define SIMPLE_YAW_I_MAX 1.0f
#endif

#ifndef SIMPLE_YAW_POS_KP
/* 偏航位置到偏航角速度参考的 P 增益别名，默认沿用 SIMPLE_YAW_KP。 */
#define SIMPLE_YAW_POS_KP SIMPLE_YAW_KP
#endif

#ifndef SIMPLE_YAW_RATE_KP
/* 偏航角速度误差到差动扭矩的增益别名，默认沿用 SIMPLE_YAW_KD。 */
#define SIMPLE_YAW_RATE_KP SIMPLE_YAW_KD
#endif

#ifndef SIMPLE_YAW_RATE_MAX
/* 偏航角速度参考限幅，单位 rad/s。 */
#define SIMPLE_YAW_RATE_MAX 1.6f
#endif

#ifndef SIMPLE_YAW_BAL_TAU_MIN
/* 俯仰共模扭矩低于该值时不给偏航，避免平衡初期抢力矩。 */
#define SIMPLE_YAW_BAL_TAU_MIN 8.0f
#endif

#ifndef SIMPLE_YAW_BAL_TAU_FULL
/* 俯仰共模扭矩达到该值后偏航门控放到满权重。 */
#define SIMPLE_YAW_BAL_TAU_FULL 24.0f
#endif


/* ==========================================================================
 * roll 轴稳定: 左右差动腿伸长找平车体
 * ==========================================================================
 * 实测符号(头less SIDE_EXT_TEST): leg_side_extend>0(左腿伸/右腿收) ->
 * 左高右低 -> roll 正(右侧下沉)。所以右倾(roll>0)时收左腿、伸右腿,
 * 指令 side_ref = -KP*roll - KD*roll_rate, 即“哪边沉就抬哪边”。
 * 差动腿伸长只用于接地时的慢速找平, 空中/冲击/跳跃时退出。
 */
#ifndef ROLL_STAB_KP
#if MODEL_V3
#define ROLL_STAB_KP 0.0f      /* 分开验证: 先只开 yaw */
#else
#define ROLL_STAB_KP 8.0f
#endif
#endif
#ifndef ROLL_STAB_KD
#define ROLL_STAB_KD 0.02f      /* v3: m / (rad/s) */
#endif
#ifndef ROLL_STAB_KI
#define ROLL_STAB_KI 0.01f      /* v3: 积分增益 */
#endif
#ifndef ROLL_STAB_I_MAX
#define ROLL_STAB_I_MAX 0.02f   /* v3: 积分项限幅, m */
#endif
#ifndef ROLL_STAB_MAX
#define ROLL_STAB_MAX 0.040f    /* v3: 差动腿伸长指令总限幅, m(腿行程上限) */
#endif
#ifndef ROLL_STAB_RATE
#define ROLL_STAB_RATE 0.0005f  /* v3: 差动指令变化率限幅, m / 1ms 控制步 */
#endif
#ifndef ROLL_STAB_STARTUP
#if MODEL_V3
#define ROLL_STAB_STARTUP 0.6f
#else
#define ROLL_STAB_STARTUP 1.5f
#endif
#endif
#ifndef ROLL_STAB_DISABLE_ABS
#define ROLL_STAB_DISABLE_ABS 0.30f /* |roll| 超该值退出(接近翻车由自救接管) */
#endif

#endif
