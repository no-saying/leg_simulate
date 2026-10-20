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
#define SIMPLE_PITCH_KP (-40.0f)  /* v3: 跑通版强度, 扭矩由上限限制 */
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

#ifndef SIMPLE_LONG_PITCH_KD
/* 长腿自动站立时降低角速度反馈，避免陀螺噪声形成持续轮扭矩。 */
#if MODEL_V3
#define SIMPLE_LONG_PITCH_KD (-9.5f)
#else
#define SIMPLE_LONG_PITCH_KD SIMPLE_PITCH_KD
#endif
#endif

#ifndef SIMPLE_LONG_PITCH_KD_START
#define SIMPLE_LONG_PITCH_KD_START 0.095f
#endif

#ifndef SIMPLE_LONG_PITCH_KD_FULL
#define SIMPLE_LONG_PITCH_KD_FULL 0.135f
#endif

#ifndef SIMPLE_PITCH_KI_NM
/* 俯仰静差积分增益，单位 Nm/(rad*s)。
 * 小增益(5)把静止时的残倾角(+2~3°)缓慢拉回 0，消除“后仰前栽”
 * 慢爬——残倾角是长时漂移→翻车的根源。大增益(30)会 windup 反而
 * 激起振荡，所以取小值；|pitch|>0.15 时清零避免救车时打架。 */
#define SIMPLE_PITCH_KI_NM 2.0f   /* 部署加固: 兜质心/装配造成的常值偏置 */
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
/* v3 轮速符号已修正。2.0 时静止慢漂仍有 ~0.09m/s(20s 漂 2.2m);
 * 提到 4.0 后 12~20s 稳态漂移 0.017m/s, 车基本停在原地。
 * 手动档位(|manual_v|>=0.05)会按设计关掉这项阻尼, 不影响 W/S 加速。 */
#define SIMPLE_VEL_DAMP 4.0f
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
/* 启动早期共模扭矩限幅，单位 Nm(自动模式)。
 * 启动/静止需要强扭矩救住瞬态, 保持 8Nm;
 * 手动行驶/转向的扭矩由 SIMPLE_MANUAL_PITCH_TAU_MAX(1.2Nm)单独
 * 限制, 避免轮子烧胎空转反推车体滑动(漂移根因)。 */
#if MODEL_V3
#define SIMPLE_TAU_START 8.0f
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
 * v3 手动驾驶时左右轮微小差速会积累成 90°+ 漂移(用户实测), yaw 环
 * 必须开启。初始 1.2Nm(旧模型值), 若方向反了调 SIMPLE_YAW_SIGN。 */
#if MODEL_V3
#define SIMPLE_YAW_MAX 1.2f
#else
#define SIMPLE_YAW_MAX 1.2f
#endif
#endif

#ifndef SIMPLE_YAW_RATE_TAU
/* 偏航角速度测量一阶低通时间常数，单位 s(0=不过滤)。
 * v3 的 1kHz 偏航环用即时角速度反馈，8ms 滞后会增加长腿晃动。 */
#define SIMPLE_YAW_RATE_TAU 0.0f
#endif

#ifndef SIMPLE_YAW_OUT_SLEW
/* 偏航差动扭矩输出斜率限制，单位 Nm/s。
 * 偏航环 20ms 一次更新, 力矩可以瞬跳 ±上限; 轮子极小惯量会被砸到
 * ±20rad/s, 手感是"一卡一卡"。限住变化率后转向才连续。 */
#define SIMPLE_YAW_OUT_SLEW 0.0f   /* 0=不限斜率(实测限斜率反而更抖) */
#endif

#ifndef SIMPLE_YAW_I_MAX
/* 偏航积分限幅，单位 rad*s；防止长时间饱和后反向过冲。 */
#define SIMPLE_YAW_I_MAX 1.0f
#endif

#ifndef SIMPLE_YAW_POS_KP
/* 偏航位置到偏航角速度参考的 P 增益，单位 (rad/s)/rad。
 * 旧值 1.2(沿用 SIMPLE_YAW_KP) 太小: 模型左右腿机构不是严格镜像(轮
 * 几何 x 偏移左右差 ~3mm), 直线行驶时腿摆推力带一个恒定的偏航扰动,
 * 位置环要产生足够差速扭矩就需要很大的角度误差 -> 实测按 W 直行 12s
 * 车头偏 -35°、横漂 5m(用户反馈"前进和转向耦合")。
 * 提到 3.0 后同样工况车头只偏 -3.8°、横漂 0.6m。
 * 手动原地转向用 SIMPLE_MANUAL_YAW_RATE_KP 单独调, 不受此影响。 */
#define SIMPLE_YAW_POS_KP 3.00f
#endif

#ifndef SIMPLE_LONG_YAW_BLEND_START
#define SIMPLE_LONG_YAW_BLEND_START 0.095f
#endif
#ifndef SIMPLE_LONG_YAW_BLEND_FULL
#define SIMPLE_LONG_YAW_BLEND_FULL 0.135f
#endif
#ifndef SIMPLE_LONG_YAW_POS_KP
/* 长腿直行时的航向保持增益。 */
#define SIMPLE_LONG_YAW_POS_KP 5.00f
#endif
#ifndef SIMPLE_LONG_YAW_MAX
/* 长腿直行与静止的偏航纠偏扭矩上限；主动转向仍用手动上限。 */
#define SIMPLE_LONG_YAW_MAX 2.00f
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
 * v3: side>0 收左腿、伸右腿；CAD 左轮在 -Y。
 * side_ref=-KP*roll-KD*roll_dot 收高侧、伸低侧。
 * 单侧支撑扰动允许冲击期找平，并额外收腿；空中/跳跃时退出。
 */
#ifndef ROLL_STAB_KP
/* roll 找平 P 增益，单位 m/rad(差动腿长 / 侧倾角)。
 * 几何标定: 轮距 0.252m, 差动腿长 d 对机体的找平角约 2d/轮距 = 7.9d,
 * 所以"完全补偿 1rad 侧倾"需要 d = 0.126m, 即单位增益 KP≈0.126。
 * 旧值 8.0 是这个的 63 倍: |roll|>0.005rad(0.29°) 就吃满 ±0.04m 限幅,
 * 整个环变成继电器式bang-bang -> 极限环。实测静止时 roll 峰峰 11.9°、
 * 差动腿长 ±0.025m 来回抽(用户看到"车在来回摆动、腿不是定长"),
 * 加 0.3Nm 侧倾力矩后 roll 峰峰 8.4°。
 * 0.30(约 2.4 倍单位增益) + KD=0.009: 静止 roll 峰峰 0.30°、
 * 差动腿长峰峰 0.0019m; 0.3Nm 力矩下残余 roll 0.19° 无振荡。 */
#if MODEL_V3
#define ROLL_STAB_KP 0.30f
#else
#define ROLL_STAB_KP 8.0f
#endif
#endif
#ifndef ROLL_STAB_KD
#define ROLL_STAB_KD 0.009f     /* v3: m / (rad/s)，超前时间约 30ms */
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

#define ROLL_OBSTACLE_ENTER       0.025f  /* rad */
#define ROLL_OBSTACLE_LARGE       0.050f  /* rad */
#define ROLL_OBSTACLE_RATE        0.18f   /* rad/s, Euler roll derivative */
#define ROLL_OBSTACLE_HOLD        0.30f   /* s */
#define ROLL_OBSTACLE_RETRACT_MAX 0.020f  /* m, common retraction */
#define ROLL_OBSTACLE_RETRACT_RATE 0.12f  /* m/s */
#define ROLL_OBSTACLE_RESTORE_RATE 0.020f /* m/s */
#ifndef WHEEL_OBSTACLE_RETRACT
#define WHEEL_OBSTACLE_RETRACT    0.012f /* m, per affected leg */
#endif
#define WHEEL_OBSTACLE_HOLD       0.20f  /* s */

#endif
