/* config_odom.h - 里程计积分 / 中立角 / 速度规划 参数
 * 由 control_config.h 统一包含。 */
#ifndef CONFIG_ODOM_H
#define CONFIG_ODOM_H

/* ==========================================================================
 * 里程计积分、近原点中立角、速度规划
 * ========================================================================== */

#ifndef SIMPLE_ODOM_V_MAX
/* 里程计积分速度限幅，单位 m/s。 */
#define SIMPLE_ODOM_V_MAX 2.50f
#endif

#ifndef SIMPLE_POS_CLAMP
/* 位置状态硬限幅，单位 m；保护长期仿真不被一次错误积分冲飞。 */
#define SIMPLE_POS_CLAMP 50.0f
#endif

#ifndef SIMPLE_ORIGIN_DEADBAND
/* 原点位置死区，单位 m；小于该范围时不再把位置误差送入回原点控制。 */
#define SIMPLE_ORIGIN_DEADBAND 0.025f
#endif

#ifndef SIMPLE_ORIGIN_SHORT_WINDOW_ENABLE
/* 短窗口原点控制开关。开启后只在姿态、轮速、惯性测量都安静时相信里程计。 */
#define SIMPLE_ORIGIN_SHORT_WINDOW_ENABLE 1
#endif

#ifndef SIMPLE_ORIGIN_DIRECT_TAU_ENABLE
/* 近场直接轮扭矩模式总开关。开启后低速贴零时保持当前俯仰角，用直接刹停/
 * 爬行扭矩回原点，避免目标俯仰阶跃的仰身冲量；关闭则退回纯俯仰角规划。 */
#define SIMPLE_ORIGIN_DIRECT_TAU_ENABLE 0
#endif

#ifndef SIMPLE_ORIGIN_TRUST_PITCH_RAD
/* 可信原点窗口允许的最大俯仰角，单位 rad；越小越不依赖大姿态下的轮里程计。 */
#define SIMPLE_ORIGIN_TRUST_PITCH_RAD 0.12f
#endif

#ifndef SIMPLE_ORIGIN_TRUST_RATE_RADPS
/* 可信原点窗口允许的最大俯仰角速度，单位 rad/s；用于排除正在剧烈救车的阶段。 */
#define SIMPLE_ORIGIN_TRUST_RATE_RADPS 1.50f
#endif

#ifndef SIMPLE_ORIGIN_TRUST_BODY_V_MAX
/* 可信原点窗口允许的最大补偿后机体速度，单位 m/s；高速运动时不长期累计里程计。 */
#define SIMPLE_ORIGIN_TRUST_BODY_V_MAX 0.80f
#endif

#ifndef SIMPLE_ORIGIN_TRUST_ACC_MAX
/* 可信原点窗口允许的最大前向惯性加速度，单位 m/s^2；加速度尖峰时冻结位置积分。 */
#define SIMPLE_ORIGIN_TRUST_ACC_MAX 4.0f
#endif

#ifndef SIMPLE_ORIGIN_TRUST_WHEEL_MIN
/* 可信原点窗口要求的最小轮速可信度，0..1；打滑、离地、冲击会把它压低。 */
#define SIMPLE_ORIGIN_TRUST_WHEEL_MIN 0.85f
#endif

#ifndef SIMPLE_ORIGIN_RELEASE_POS
/* 回到原点后释放位置环的位置窗口，单位 m；进入后开始把位置记忆向 0 泄放。
 * 腿摆方案下位置误差是纠偏依据, 不能车一慢溜就抹掉: 只有真正贴住保持点
 * 且速度几乎为零时才释放。 */
#define SIMPLE_ORIGIN_RELEASE_POS 0.050f
#endif

#ifndef SIMPLE_ORIGIN_RELEASE_V
/* 回到原点后释放位置环的速度窗口，单位 m/s；0.025 以下才算真静止,
 * 慢速漂移(0.03~0.1m/s)时保留位置误差给腿摆纠偏。 */
#define SIMPLE_ORIGIN_RELEASE_V 0.025f
#endif

#ifndef SIMPLE_ORIGIN_FAST_POS_KP
/* 短窗口内位置误差到目标速度的快速增益，单位 1/s。
 * 位置基准是 hold_pos(进入/松开手动时吸附的当前点)，不是启动原点，
 * 满足“无指令时维持当前点”。重轮下位置回环会激起 pitch 振荡，
 * 轻轮(armature 0.002)下正常，负责消除松手后的慢爬。
 * 0.20 更温和: 起步漂移被吸收进 hold_pos 后不再来回走位，
 * 避免位置环用 ±2.9° 俯仰来回纠正造成 pitch 低周摇摆。 */
#define SIMPLE_ORIGIN_FAST_POS_KP 0.20f
#endif

#ifndef SIMPLE_ORIGIN_FAST_V_MAX
/* 短窗口内回原点目标速度限幅，单位 m/s；比常规规划更积极，但仍保持温和。 */
#define SIMPLE_ORIGIN_FAST_V_MAX 0.25f
#endif

#ifndef SIMPLE_ORIGIN_FAST_PITCH_KP
/* 短窗口内速度误差到目标俯仰角的快速增益，单位 rad/(m/s)。
 * 0.045rad 限幅 / 0.10 增益 ≈ 0.45m/s 才饱和: 0.1m/s 级漂移只给
 * 1° 级倾角，避免小幅漂移就把 pitch 甩到 ±2.9°。 */
#define SIMPLE_ORIGIN_FAST_PITCH_KP 0.10f
#endif

#ifndef SIMPLE_ORIGIN_FAST_PITCH_MAX
/* 短窗口内速度规划给目标俯仰角的限幅，单位 rad(约 2.6°)。
 * 目标俯仰限幅先收紧, pitch 才可能只做 1° 级低频摆动。 */
#define SIMPLE_ORIGIN_FAST_PITCH_MAX 0.045f
#endif

#ifndef SIMPLE_ORIGIN_DIRECT_PITCH_KP
/* 短窗口内位置误差直接补到目标俯仰角的增益，单位 rad/m。
 * 0.3m 位置误差 × 0.15 = 0.045rad，被限幅压到 0.015rad(0.86°)，
 * 只负责慢慢把车推回保持点，不参与大倾角机动。 */
#define SIMPLE_ORIGIN_DIRECT_PITCH_KP 0.15f
#endif

#ifndef SIMPLE_ORIGIN_DIRECT_PITCH_MAX
/* 位置直接补偿的最大俯仰角，单位 rad(约 1.2°)。 */
#define SIMPLE_ORIGIN_DIRECT_PITCH_MAX 0.020f
#endif

#ifndef SIMPLE_ORIGIN_DIRECT_SPEED_GATE
/* 直接位置补偿随速度衰减的窗口，单位 m/s；速度高于该值时不再叠加直接补偿，
 * 避免高速回程时位置项继续往运动方向加速。 */
#define SIMPLE_ORIGIN_DIRECT_SPEED_GATE 0.30f
#endif

#ifndef SIMPLE_ORIGIN_APPROACH_RAMP_POS
/* 回原点减速斜坡距离，单位 m；位置误差小于该值后，目标速度按剩余距离线性收窄，
 * 让机器人提前减速而不是以高速冲过零点。 */
#define SIMPLE_ORIGIN_APPROACH_RAMP_POS 0.35f
#endif

#ifndef SIMPLE_ORIGIN_APPROACH_V_MIN_FRAC
/* 减速斜坡末端目标速度下限比例；防止斜坡把目标速度压到 0 后失去回程趋势。 */
#define SIMPLE_ORIGIN_APPROACH_V_MIN_FRAC 0.15f
#endif

#ifndef SIMPLE_ORIGIN_APPROACH_HOMEBOUND_KP
/* 斜坡内回程方向刹车增益，单位 rad/(m/s)；比常规短窗口增益更硬，用于抵消
 * 目标俯仰角的响应延迟，把接近零点速度压住。 */
#define SIMPLE_ORIGIN_APPROACH_HOMEBOUND_KP 0.45f
#endif

#ifndef SIMPLE_ORIGIN_APPROACH_AWAY_KP
/* 斜坡内远离方向纠偏增益，单位 rad/(m/s)。 */
#define SIMPLE_ORIGIN_APPROACH_AWAY_KP 0.40f
#endif

#ifndef SIMPLE_ORIGIN_APPROACH_PITCH_MAX
/* 斜坡内回程方向目标俯仰角限幅，单位 rad(约 2.6°)。 */
#define SIMPLE_ORIGIN_APPROACH_PITCH_MAX 0.045f
#endif

#ifndef SIMPLE_ORIGIN_STOP_POS
/* 近场捕获窗口，单位 m；进入后优先把速度刹向 0，再靠直接位置补偿爬回原点。 */
#define SIMPLE_ORIGIN_STOP_POS 0.16f
#endif

#ifndef SIMPLE_ORIGIN_STOP_V
/* 近场捕获触发速度，单位 m/s；窗口内速度高于该值时执行刹停。 */
#define SIMPLE_ORIGIN_STOP_V 0.08f
#endif

#ifndef SIMPLE_ORIGIN_STOP_PITCH_KP
/* 近场捕获刹车增益，单位 rad/(m/s)。 */
#define SIMPLE_ORIGIN_STOP_PITCH_KP 0.45f
#endif

#ifndef SIMPLE_ORIGIN_STOP_PITCH_MAX
/* 近场捕获刹车目标俯仰角限幅，单位 rad(约 2.6°)。 */
#define SIMPLE_ORIGIN_STOP_PITCH_MAX 0.045f
#endif

#ifndef SIMPLE_ORIGIN_CREEP_POS_KP
/* 近场爬行直接轮扭矩增益，单位 Nm/m。不经过目标俯仰角，避免仰身冲量。 */
#define SIMPLE_ORIGIN_CREEP_POS_KP 14.0f
#endif

#ifndef SIMPLE_ORIGIN_CREEP_TAU_KP
#define SIMPLE_ORIGIN_CREEP_TAU_KP SIMPLE_ORIGIN_CREEP_POS_KP
#endif

#ifndef SIMPLE_ORIGIN_CREEP_TAU_MAX
/* 近场直接轮扭矩爬行最大扭矩，单位 Nm。 */
#define SIMPLE_ORIGIN_CREEP_TAU_MAX 2.0f
#endif

#ifndef SIMPLE_ORIGIN_CREEP_PITCH_KP
/* 大俯仰兜底爬行: 速度误差到目标俯仰角增益，单位 rad/(m/s)。 */
#define SIMPLE_ORIGIN_CREEP_PITCH_KP 0.35f
#endif

#ifndef SIMPLE_ORIGIN_CREEP_PITCH_MAX
/* 大俯仰兜底爬行目标俯仰角限幅，单位 rad(约 2.6°)。 */
#define SIMPLE_ORIGIN_CREEP_PITCH_MAX 0.045f
#endif

#ifndef SIMPLE_ORIGIN_STOP_TAU_KP
/* 近场直接轮扭矩刹停增益，单位 Nm/(m/s)；反对当前速度，不经过目标俯仰角，
 * 避免仰身冲量把车推向反方向。 */
#define SIMPLE_ORIGIN_STOP_TAU_KP 10.0f
#endif

#ifndef SIMPLE_ORIGIN_STOP_TAU_MAX
/* 近场直接轮扭矩刹停最大扭矩，单位 Nm。 */
#define SIMPLE_ORIGIN_STOP_TAU_MAX 4.5f
#endif

#ifndef SIMPLE_POS_DECAY_WHEN_HOME
/* 原点释放后位置记忆向 0 衰减的速率，单位 1/s；越大越快忘掉长期里程计漂移。 */
#define SIMPLE_POS_DECAY_WHEN_HOME 0.20f
#endif

#ifndef SIMPLE_ORIGIN_MEMORY_MAX
/* 短窗口原点控制保留的位置记忆上限，单位 m；超过后只当作“需要回正”的有界误差。 */
#define SIMPLE_ORIGIN_MEMORY_MAX 5.0f
#endif

#ifndef SIMPLE_ODOM_LATERAL_DECAY
/* 显示用横向里程计向 0 衰减的速率，单位 1/s；只影响日志/界面，不进入控制。 */
#define SIMPLE_ODOM_LATERAL_DECAY 1.50f
#endif

#ifndef SIMPLE_ODOM_DISPLAY_Y_MAX
/* 显示用横向里程计限幅，单位 m；避免偏航投影把小误差显示成长距离横漂。 */
#define SIMPLE_ODOM_DISPLAY_Y_MAX 0.25f
#endif

#ifndef SIMPLE_PARK_PITCH_RAD
/* 近原点停车阻尼允许的 pitch 误差窗口，单位 rad。 */
#define SIMPLE_PARK_PITCH_RAD 0.105f
#endif

#ifndef SIMPLE_POS_PITCH_RAD
/* 历史直接位置到 pitch 控制窗口，单位 rad。 */
#define SIMPLE_POS_PITCH_RAD 0.052f
#endif

#ifndef SIMPLE_CAPTURE_DAMP
/* 截获/近原点额外速度阻尼系数；当前为 0。 */
#define SIMPLE_CAPTURE_DAMP 0.0f
#endif

#ifndef SIMPLE_POS_NEUTRAL_KP
/* 历史位置误差到目标俯仰角的 P 增益。 */
#define SIMPLE_POS_NEUTRAL_KP 0.060f
#endif

#ifndef SIMPLE_VEL_NEUTRAL_KD
/* 历史速度误差到目标俯仰角的 D 增益。 */
#define SIMPLE_VEL_NEUTRAL_KD 0.028f
#endif

#ifndef SIMPLE_NEUTRAL_MAX
/* 非速度规划路径下目标俯仰角偏置最大值，单位 rad(约 1.4°)。 */
#define SIMPLE_NEUTRAL_MAX 0.025f
#endif

#ifndef SIMPLE_SPEED_PLAN_ENABLE
/* 推荐原点控制路径开关: 位置误差 -> 目标速度 -> 目标俯仰角。 */
#define SIMPLE_SPEED_PLAN_ENABLE 1
#endif

#ifndef SIMPLE_SPEED_PLAN_POS_KP
/* 位置误差到目标速度的 P 增益，单位 1/s。 */
#define SIMPLE_SPEED_PLAN_POS_KP 0.05f
#endif

#ifndef SIMPLE_SPEED_PLAN_V_MAX
/* 原点速度规划目标速度限幅，单位 m/s。 */
#define SIMPLE_SPEED_PLAN_V_MAX 0.18f
#endif

#ifndef SIMPLE_SPEED_PLAN_PITCH_KP
/* 速度误差到目标俯仰角的 P 增益，单位 rad/(m/s)。 */
#define SIMPLE_SPEED_PLAN_PITCH_KP 0.05f
#endif

#ifndef SIMPLE_SPEED_PLAN_PITCH_MAX
/* 速度规划给目标俯仰角的默认限幅，单位 rad(约 2.6°)。 */
#define SIMPLE_SPEED_PLAN_PITCH_MAX 0.045f
#endif

#ifndef SIMPLE_SPEED_PLAN_HOMEBOUND_GAIN
/* 正在回原点时放大速度误差到目标俯仰角的倍率。 */
#define SIMPLE_SPEED_PLAN_HOMEBOUND_GAIN 1.2f
#endif

#ifndef SIMPLE_SPEED_PLAN_HOMEBOUND_MAX
/* 回原点方向目标俯仰角限幅，单位 rad(约 1.7°)。 */
#define SIMPLE_SPEED_PLAN_HOMEBOUND_MAX 0.030f
#endif

#ifndef SIMPLE_SPEED_PLAN_AWAY_GAIN
/* 正在远离原点时放大纠偏的倍率。 */
#define SIMPLE_SPEED_PLAN_AWAY_GAIN 1.2f
#endif

#ifndef SIMPLE_SPEED_PLAN_AWAY_MAX
/* 远离原点方向目标俯仰角限幅，单位 rad(约 1.7°)。 */
#define SIMPLE_SPEED_PLAN_AWAY_MAX 0.030f
#endif

#ifndef SIMPLE_SPEED_PLAN_START_TIME
/* 原点速度规划启动延时，单位 s；让落地和平衡先稳定。 */
#define SIMPLE_SPEED_PLAN_START_TIME 0.70f
#endif

#endif
