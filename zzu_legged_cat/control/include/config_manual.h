/* config_manual.h - 手动驾驶 / 离线辨识 / 旧原点控制 参数
 * 由 control_config.h 统一包含。 */
#ifndef CONFIG_MANUAL_H
#define CONFIG_MANUAL_H

/* ==========================================================================
 * 手动驾驶 (GUI WASD)
 * ========================================================================== */

#ifndef SIMPLE_MANUAL_V_MAX
/* W/S 手动驾驶目标速度，单位 m/s。用户反馈偏慢，加大倾角提速。 */
#define SIMPLE_MANUAL_V_MAX 1.20f
#endif

#ifndef SIMPLE_MANUAL_ACCEL
/* 手动驾驶速度指令变化率上限，单位 m/s^2；斜坡化避免起步/刹停瞬间猛冲。 */
#define SIMPLE_MANUAL_ACCEL 0.30f
#endif

#ifndef SIMPLE_MANUAL_DECEL
#define SIMPLE_MANUAL_DECEL 0.80f
#endif

#ifndef SIMPLE_MANUAL_YAW_ACCEL
/* 手动转向指令变化率上限，单位 rad/s^2。
 * 2.5 起步太猛 -> yaw 耦合进 pitch 的扰动大, 压不住翻车。0.8 平缓。 */
#define SIMPLE_MANUAL_YAW_ACCEL 0.8f
#endif

#ifndef SIMPLE_MANUAL_YAW_RATE
/* A/D 手动转向目标偏航角速度，单位 rad/s。
 * 0.8 时 yaw 耦合进 pitch 的扰动太大, 手动 pitch 环+摩擦极限
 * 压不住 -> 转向 2s 内翻车。0.4 时扰动减半, 轮子咬得住。 */
#define SIMPLE_MANUAL_YAW_RATE 1.20f
#endif

#ifndef SIMPLE_MANUAL_SPIN_YAW_RATE
/* 小陀螺低腿长目标偏航角速度；长腿在控制环中降到普通转向目标。
 * 这里是指令值，实际角速度仍受牵引力和偏航环限幅约束。 */
#define SIMPLE_MANUAL_SPIN_YAW_RATE 2.40f
#endif

#ifndef SIMPLE_MANUAL_SPIN_RATE_SCALE
/* 小陀螺全腿长分段目标统一提速 15%，普通 A/D 速度不变。 */
#define SIMPLE_MANUAL_SPIN_RATE_SCALE 1.15f
#endif

#ifndef SIMPLE_MANUAL_YAW_RATE_MAX
/* 手动模式偏航角速度参考限幅，单位 rad/s。
 * auto 的 SIMPLE_YAW_RATE_MAX=1.6 会把小陀螺 3.0rad/s 的目标压掉一半,
 * 手动模式单独放开到 3.5。 */
#define SIMPLE_MANUAL_YAW_RATE_MAX 3.5f
#endif

#ifndef SIMPLE_MANUAL_YAW_MAX
/* 手动转向差动扭矩上限，单位 Nm。俯仰保持的共模扭矩会占掉大部分轮上
 * 力矩余量，2.5 推不动原地转(实测只有 ~3°/s)；12 只到 72°/s，
 * 24 到 149°/s, 32 让 3.0rad/s 的小陀螺目标基本达到。
 * v3 轮腿车偏航惯量只有 ~0.013 kg·m², 0.8Nm 差动(yaw_mix 0.32)
 * 已经产生 ~200 rad/s² 的偏航加速度, 30ms 内冲到 3.7rad/s 过冲,
 * 车身被轮子反作用力掀翻(实测 pitch 顶到饱和、车走大圆漂移)。
 * 0.6Nm 差动(每轮 0.12Nm)足够 0.8rad/s 原地转, 且不扰动 pitch。 */
#define SIMPLE_MANUAL_YAW_MAX 1.20f
#endif

#ifndef SIMPLE_MANUAL_SPIN_YAW_MAX
/* 小陀螺独立差动上限，普通 A/D 仍使用 SIMPLE_MANUAL_YAW_MAX。 */
#define SIMPLE_MANUAL_SPIN_YAW_MAX 1.40f
#endif

#ifndef SIMPLE_MANUAL_YAW_RATE_KP
/* 手动转向偏航角速度增益，单位 Nm/(rad/s)。
 * 7 时稳态 149°/s 稳定不漂; 12 到 159°/s 但自转被扰动带偏(漂 2.7m)。
 * v3 轮腿车 9/3 都太高 -> 差速扭矩过大, 偏航率瞬间过冲到 3.7rad/s,
 * 0.4 时另外发现"转一下就停": 腿伺服对偏航的反作用有静摩擦, 0.4 在
 * 速率误差 0.4rad/s 时只给 0.16Nm, 车转到 ~36° 就完全停住不再转。
 * 偏航环提到每步 1kHz 执行(去掉 20ms 零阶保持的相位滞后)后重标:
 *   KP=1.2 -> 偏航率 0.38rad/s, 抖动 std 0.19, 离起点 0.12m
 *   KP=2.4 -> 0.47rad/s, std 0.18, 离起点 0.12m   ← 取这个
 *   KP=4.0/8.0 -> 0.49rad/s 但 std 升到 0.23(开始激发 8Hz 偏航谐振)
 * 上限仍是 0.6Nm。 */
#define SIMPLE_MANUAL_YAW_RATE_KP 2.40f
#endif

#ifndef SIMPLE_MANUAL_PITCH_TAU_MAX
/* 手动驾驶时俯仰环共模扭矩上限，单位 Nm。
 * 注意: 共模扭矩对两个轮子是"每个轮子都给这么多"(c2=c5=pitch_mix),
 * 所以这里的数值必须和单轮摩擦极限比较: μ·(m·g/2)·r ≈ 1.0·11.9·0.0435
 * ≈ 0.52Nm/轮。给到 4Nm 并不会真的输出 4Nm——超过摩擦极限后轮子
 * 打滑, 实际牵引力封顶在 μ·N。
 * 旧值 1.2Nm 的问题不是"太大"而是"太接近工作点": 手动俯仰 P 增益
 * 是 -25, 1.2Nm 对应仅 2.7° 误差就饱和, 于是正常巡航时俯仰环一直
 * 顶在限幅上(毫无余量), 松手刹车/扰动的瞬态一超 3° 就救不回来
 * (实测松手后 20° 后仰 -> 翻车)。放大到 4Nm 让俯仰环有真实的工作
 * 区间(约 9°), 物理上仍由地面摩擦封顶。 */
#define SIMPLE_MANUAL_PITCH_TAU_MAX 4.00f
#endif

#ifndef SIMPLE_LONG_TURN_PITCH_TAU_MAX
/* 长腿停转保留正常停车的恢复余量，不能在切换时降到 1 Nm。 */
#define SIMPLE_LONG_TURN_PITCH_TAU_MAX 4.00f
#endif

#ifndef SIMPLE_TURN_PITCH_TAU_MAX
/* 中长腿纯转向时留给俯仰平衡的共模扭矩；保留非零值维持姿态。 */
#define SIMPLE_TURN_PITCH_TAU_MAX 0.25f
#endif

#ifndef SIMPLE_SHORT_TURN_PITCH_TAU_MAX
/* 短腿的轮子需要稍多的俯仰平衡余量。 */
#define SIMPLE_SHORT_TURN_PITCH_TAU_MAX 0.50f
#endif

#ifndef SIMPLE_TURN_PITCH_RECOVERY_START
#define SIMPLE_TURN_PITCH_RECOVERY_START 0.040f
#endif

#ifndef SIMPLE_TURN_PITCH_RECOVERY_FULL
#define SIMPLE_TURN_PITCH_RECOVERY_FULL 0.080f
#endif

#ifndef SIMPLE_MANUAL_SWING_SPEED_START
/* 手动 W/S 摆腿随速度衰减的起始速度，单位 m/s。
 * 摆腿是速度执行器: 固定摆角产生恒定加速度, 没有轮速阻尼会一直
 * 加速到 4m/s+ 安全刹。速度超过该值后摆腿按比例回中, 软限速在
 * ~1.5-2m/s 巡航, 不需要轮速阻尼(轮速阻尼会超摩擦烧胎)。 */
#define SIMPLE_MANUAL_SWING_SPEED_START 1.0f
#endif

#ifndef SIMPLE_MANUAL_SWING_SPEED_RANGE
/* 摆腿衰减速度窗口，单位 m/s: 从 SPEED_START 到 SPEED_START+RANGE
 * 摆腿从满幅线性减到 0。 */
#define SIMPLE_MANUAL_SWING_SPEED_RANGE 1.0f
#endif

#ifndef SIMPLE_MANUAL_PITCH_KP
/* 手动驾驶俯仰环增益(自动模式仍用 -40/-12 救启动瞬态)。
 * 轮上摩擦极限 ~0.5Nm/轮: -40 在 2° 误差时就要求 1.4Nm/轮 -> 烧胎
 * 空转、车体滑动漂移。手动用 -12/-4: 2° 时 0.42Nm/轮内, 轮子咬地
 * 纯滚动, pitch 仍能收在 1° 左右。 */
#define SIMPLE_MANUAL_PITCH_KP (-25.0f)
#endif

#ifndef SIMPLE_MANUAL_PITCH_KD
#define SIMPLE_MANUAL_PITCH_KD (-6.0f)
#endif

#ifndef SIMPLE_MANUAL_SWING_MAX
/* 手动腿摆最大偏置，单位 m(相对 SIMPLE_MANUAL_SWING_TRIM)。
 * 不再复用自动位置环的 ±0.03: 手动速度环需要参考有余量, 参考被限幅顶住
 * 时腿伺服会把车"钉死"(实测 S 反向时摆角停在限幅上, 车完全静止不动,
 * 连速度环都推不动)。0.05 对应摆角总范围 ±0.05m、关节约 ±27°, 仍在
 * 行程内; 实际巡航只用得到 0.014~0.028。 */
#define SIMPLE_MANUAL_SWING_MAX 0.050f
#endif

#ifndef SIMPLE_MANUAL_SWING_DRIVE
/* 手动 W/S 按住时的固定腿摆角，单位 m。
 * 用户要求的驱动方式: 腿摆到一个固定角度, 质心相对轮轴后移/前移,
 * 俯仰环输出轮扭矩让车前进/后退, pitch 目标恒为 0。
 * 0.015 太大: 闭链腿屈曲 + pitch 瞬态超摩擦 -> 按 S 翻车(GUI 实测)。
 * 0.006 小幅摆动: 屈曲小、pitch 扰动小, 轮子咬得住, 缓慢前进/后退。 */
#define SIMPLE_MANUAL_SWING_DRIVE 0.012f
#endif

#ifndef SIMPLE_MANUAL_SWING_V_KP
/* 手动 W/S 腿摆速度环增益，单位 m/(m/s)。
 * 腿摆角就是"质心相对轮轴的前后偏移"，本质是加速度执行器: 固定摆角
 * 会一直加速, 没有速度反馈就无从收敛。用速度误差闭环后腿摆停在
 * "维持目标车速"的角度上并保持, 不会随速度升高被软限速拉回中位
 * (旧固定摆角+软限速的方案表现为"腿摆一下又回去了")。
 * 符号: 车慢于目标(v_err>0) -> 摆角减小 -> 质心前移 -> 向前加速。
 * 0.030: 1.2m/s 误差 -> 36mm 摆角, 被 ±SIMPLE_MANUAL_SWING_MAX 限幅;
 * 实测 W 稳态 1.18m/s(指令 1.2), S 反向 0.6m/s, 正反向可用。 */
#define SIMPLE_MANUAL_SWING_V_KP 0.030f
#endif

#ifndef SIMPLE_LONG_MANUAL_BRAKE_START
#define SIMPLE_LONG_MANUAL_BRAKE_START 0.135f
#endif
#ifndef SIMPLE_LONG_MANUAL_BRAKE_KP
/* 长腿摆腿驱动时的同向超速轮上制动，单位 Nm/(m/s)。 */
#define SIMPLE_LONG_MANUAL_BRAKE_KP 3.0f
#endif
#ifndef SIMPLE_LONG_MANUAL_BRAKE_MAX
#define SIMPLE_LONG_MANUAL_BRAKE_MAX 2.0f
#endif
#ifndef SIMPLE_LONG_POST_BRAKE_KP
#define SIMPLE_LONG_POST_BRAKE_KP 3.0f
#endif
#ifndef SIMPLE_LONG_POST_BRAKE_MAX
#define SIMPLE_LONG_POST_BRAKE_MAX 2.0f
#endif
#ifndef SIMPLE_LONG_POST_TIMEOUT
#define SIMPLE_LONG_POST_TIMEOUT 6.0f
#endif

#ifndef SIMPLE_MANUAL_SWING_V_ERR_MAX
/* 手动速度误差限幅，单位 m/s；防止摆腿/打滑瞬态把腿摆一次打满。 */
#define SIMPLE_MANUAL_SWING_V_ERR_MAX 1.20f
#endif

#ifndef SIMPLE_MANUAL_SWING_RECENTER_RATE
/* 进手动后"摆位吸附偏移"回到平衡摆角的速度，单位 m/s。
 * 取和摆腿变化率限幅同量级(0.01m/s), 腿重新对中时不会超过伺服跟得上
 * 的速度; 24mm 偏移约 2.5s 归零。 */
#define SIMPLE_MANUAL_SWING_RECENTER_RATE 0.010f
#endif

#ifndef SIMPLE_MANUAL_SWING_RATE
/* 手动模式腿摆偏置的变化率限幅，单位 m / 1ms 控制步(即 m/s ÷ 1000)。
 * 0.00001(0.01m/s) 时摆角从 +0.05 走到 -0.05 要 10s，位置环根本追不上
 * 转向起步那一下的前后扰动 -> 原地转时会滑出 0.7~1m。
 * 0.00003(0.03m/s) 是响应与"摆腿不抽搐"的折中: 满幅 1.7s。 */
#define SIMPLE_MANUAL_SWING_RATE 0.00003f
#endif

#ifndef SIMPLE_MANUAL_SWING_TRIM
/* 手动硬腿伺服与自动软腿伺服的平衡摆位相差约 7.5mm。 */
#define SIMPLE_MANUAL_SWING_TRIM (-0.0075f)
#endif

#ifndef SIMPLE_MANUAL_LEG_LEN_REF
/* 手动摆角配平标定时的腿长, 单位 m(与 SIMPLE_MANUAL_SWING_TRIM 配套)。 */
#define SIMPLE_MANUAL_LEG_LEN_REF 0.110f
#endif

#ifndef SIMPLE_MANUAL_SWING_TRIM_LK
/* 已由共用配平表按腿长处理。 */
#define SIMPLE_MANUAL_SWING_TRIM_LK 0.0f
#endif

#ifndef SIMPLE_MANUAL_SWING_TRIM_MIN
/* 配平摆角下限，单位 m。 */
#define SIMPLE_MANUAL_SWING_TRIM_MIN (-0.0075f)
#endif

#ifndef SIMPLE_MANUAL_SWING_TRIM_MAX
/* 配平摆角上限，单位 m。 */
#define SIMPLE_MANUAL_SWING_TRIM_MAX (-0.0075f)
#endif

#ifndef SIMPLE_MANUAL_HOLD_POS_KP
/* 手动"不前后走"时(纯转向/零指令)把车钉在原地的位置环 P 增益，
 * 单位 (m/s)/m。输出的是目标速度, 再由腿摆速度环实现。
 * 标定(按住 A 转 15s, 量机体离起点的最大距离):
 *   0.6 -> 0.57m(绕半径 0.3m 的圆跑), 3.0 -> 0.29m, 5.0 -> 0.18m(且转完
 *   回到起点 0.10m), 8.0/12 -> 反而变差(|pitch| 冲到 5~6°, 摆腿执行器
 *   跟不上位置环 -> 振荡)。取 5.0。 */
#define SIMPLE_MANUAL_HOLD_POS_KP 5.00f
#endif

#ifndef SIMPLE_MANUAL_HOLD_V_MAX
/* 手动"原地保持"目标速度限幅，单位 m/s；限制回正速度, 避免大偏差
 * 时用高速冲回进入点(过冲后反而更难停)。 */
#define SIMPLE_MANUAL_HOLD_V_MAX 0.45f
#endif

#ifndef SIMPLE_MANUAL_TURN_VEL_DAMP
/* 纯转向的前向轮速阻尼，单位 Nm/(m/s)。中腿小陀螺实测 2.0
 * 比 0 和 4.0 更能限制旋转期间的最大位移。 */
#define SIMPLE_MANUAL_TURN_VEL_DAMP 2.0f
#endif

#ifndef SIMPLE_MANUAL_YAW_LEG_KP
/* 叉腿转向增益，单位 m/(rad/s)：偏航角速度误差 -> 左右腿摆差动。
 * 差动把轮轴线在水平面内"拧"一个角度, 由闭链腿几何直接产生偏航力矩,
 * 不像轮子差速那样必须先把轮胎推滑起来, 也不带前后推力。 */
#define SIMPLE_MANUAL_YAW_LEG_KP 0.030f
#endif

#ifndef SIMPLE_MANUAL_YAW_LEG_MAX
/* 叉腿最大差动，单位 m(左右腿摆参考互差 2×该值)。
 * 用户反馈"差动腿摆摆的太多了": 0.045 时左右腿摆互差 90mm, 视觉上腿
 * 叉得很开。实际转向力矩主要仍来自轮子差速(实测只留叉腿时偏航率
 * 均值只有 -0.03rad/s), 所以把幅度收到 0.018 只留一点几何偏置。 */
#define SIMPLE_MANUAL_YAW_LEG_MAX 0.018f
#endif

#ifndef SIMPLE_MANUAL_YAW_LEG_RATE
/* 叉腿差动变化率限幅，单位 m/s。 */
#define SIMPLE_MANUAL_YAW_LEG_RATE 0.060f
#endif

#ifndef SIMPLE_MANUAL_YAW_LEG_SIGN
/* 叉腿转向方向: +1 时 A(manual_yaw>0) 给正差动。方向不对改成 -1。 */
#define SIMPLE_MANUAL_YAW_LEG_SIGN 1.0f
#endif

#ifndef SIMPLE_MANUAL_YAW_LEG_SCALE_MIN
/* 叉腿幅值随腿长缩放的系数下限(腿短时不要缩到 0)。 */
#define SIMPLE_MANUAL_YAW_LEG_SCALE_MIN 0.8f
#endif

#ifndef SIMPLE_MANUAL_YAW_LEG_SCALE_MAX
/* 叉腿幅值随腿长缩放的系数上限(腿长时限幅, 防止关节行程过大)。 */
#define SIMPLE_MANUAL_YAW_LEG_SCALE_MAX 1.6f
#endif

#ifndef SIMPLE_TURN_LEG_MIN
/* 下限覆盖最短可用腿长约 0.071m，留机械读数余量。 */
#define SIMPLE_TURN_LEG_MIN 0.066f
#endif
#ifndef SIMPLE_TURN_LEG_MAX
/* 普通 A/D 转向覆盖机械最大可用腿长，保留约 8mm 读数余量。 */
#define SIMPLE_TURN_LEG_MAX 0.165f
#endif
#ifndef SIMPLE_SPIN_LEG_REDUCE_START
#define SIMPLE_SPIN_LEG_REDUCE_START 0.132f
#endif
#ifndef SIMPLE_SPIN_LEG_REDUCE_FULL
/* 长腿高速小陀螺横漂明显，在这段腿长内降到普通转向速度。 */
#define SIMPLE_SPIN_LEG_REDUCE_FULL 0.145f
#endif


#ifndef SIMPLE_MANUAL_SWING_STOP_KP
/* 松手停车腿摆刹车增益，单位 m/(m/s)。
 * 前进(v>0) -> 腿向后摆(sw<0) -> 质心前移 -> 俯仰环轮子反转刹停,
 * pitch 保持 0, 不用倾角刹停。比速度环增益高一点, 停车更快。 */
#define SIMPLE_MANUAL_SWING_STOP_KP 0.020f
#endif

#ifndef SIMPLE_MANUAL_SWING_STOP_VMAX
/* 松手刹车用的速度反馈限幅，单位 m/s。
 * 腿摆/轮子几何耦合会让轮速瞬间读出 3m/s+ 的假速度, 直接乘增益会
 * 把腿摆打满反方向翻车; 限幅后刹车腿摆最大也只有 ~0.032m。 */
#define SIMPLE_MANUAL_SWING_STOP_VMAX 0.60f
#endif

#ifndef TURN_DRIFT_POS_KP
/* A/D 原地差速转向的"原地保持"位置增益，单位 Nm/m。
 * v3 闭链腿几何在 yaw 时会把轮轴反作用耦合进前向位移(实测转 90°
 * 漂 4m+), 轮子差速扭矩本身刹不住这种滑动。转向期间用里程计+
 * IMU 融合速度测真实前滑, 直接给共模轮扭矩把车钉在进入点原地转。 */
#define TURN_DRIFT_POS_KP 0.40f
#endif

#ifndef TURN_DRIFT_V_KP
/* A/D 原地转向"原地保持"速度增益，单位 Nm/(m/s)。 */
#define TURN_DRIFT_V_KP 1.20f
#endif

#ifndef TURN_DRIFT_TAU_MAX
/* A/D 原地转向刹车共模扭矩上限，单位 Nm。
 * 必须小于轮上摩擦极限(实测 ~0.5Nm/轮 开始打滑), 否则刹滑扭矩
 * 自己把轮子打滑放大漂移。先置 0 关闭(实测共模刹滑会把 yaw 方向
 * 带反), 等 pitch 增益收住不打滑后再评估是否需要。 */
#define TURN_DRIFT_TAU_MAX 0.0f
#endif

#ifndef SIMPLE_MANUAL_YAW_SPEED_GATE
/* 手动转向随前进速度衰减的窗口，单位 m/s；速度越高差动扭矩越柔，
 * 防止高速行驶中打方向把俯仰/横滚掀翻。 */
#define SIMPLE_MANUAL_YAW_SPEED_GATE 0.80f
#endif

#ifndef SIMPLE_MANUAL_TRACK_KP_SCALE
/* 手动模式跟踪增益缩放: 虚拟目标持续移动，比静态回原点用更柔的跟踪增益，
 * 避免追目标时过冲成高速漂移。
 * 1.0: 手动速度由腿摆承担, 位置环只在启动/松手时拉回, 不需要柔化,
 * 0.65 会拉不回启动振荡(实测启动后退 4m)。 */
#define SIMPLE_MANUAL_TRACK_KP_SCALE 1.0f
#endif

#ifndef SIMPLE_MANUAL_PITCH_KP
/* 手动行驶反馈式中立角增益，单位 rad/(m/s)：速度误差 -> 目标俯仰。
 * 车体保持小倾角巡航，速度由反馈自然收敛。与收紧后的 0.07rad 限幅
 * 匹配(0.07/0.12 = 0.58m/s 饱和)，速度误差小时只用 1° 级倾角。 */
#define SIMPLE_MANUAL_PITCH_KP 0.12f
#endif

#ifndef SIMPLE_MANUAL_PITCH_MAX
/* 手动行驶目标俯仰上限，单位 rad(约 4.0°)。用户要求倾角不用太大:
 * 前仰前进/后仰后退靠速度反馈收敛，不再用 8.6° 大倾角猛冲。 */
#define SIMPLE_MANUAL_PITCH_MAX 0.070f
#endif

#ifndef SIMPLE_MANUAL_PITCH_KP_MAX
/* 手动行驶目标俯仰上限(速度环直接输出, 未走超速/回程分支), 单位 rad。
 * 速度控制由腿摆承担, 这个上限只留 ~1.1° 坡度补偿, 避免俯仰环
 * 抬身扭矩与速度方向打架。 */
#define SIMPLE_MANUAL_PITCH_KP_MAX 0.020f
#endif

#ifndef SIMPLE_MANUAL_AWAY_GAIN
/* 手动行驶超速时刹速增益倍率。 */
#define SIMPLE_MANUAL_AWAY_GAIN 2.5f
#endif

#ifndef SIMPLE_MANUAL_AWAY_MAX
/* 手动模式超速刹速目标俯仰上限，单位 rad(约 1.7°)。
 * 与 auto 一致(0.03): manual_active=1 时 sensor 的位置环会用这个
 * 限幅, 0.06 会让启动刹车过猛、轮子瞬间反转把车甩退(实测 -2.5m/s)。
 * 手动速度由腿摆承担, tp 只需小角度。 */
#define SIMPLE_MANUAL_AWAY_MAX 0.030f
#endif

#ifndef SIMPLE_MANUAL_HOMEBOUND_MAX
/* 手动模式回程方向目标俯仰上限，单位 rad(约 1.7°)。 */
#define SIMPLE_MANUAL_HOMEBOUND_MAX 0.030f
#endif

#ifndef SIMPLE_MANUAL_SPEED_BRAKE_V
/* 手动模式速度刹车阈值，单位 m/s；超过后叠加反向俯仰限速，防高速侧翻。 */
#define SIMPLE_MANUAL_SPEED_BRAKE_V 4.00f
#endif

#ifndef SIMPLE_MANUAL_SPEED_BRAKE_KP
/* 手动模式速度刹车增益，单位 rad/(m/s)。 */
#define SIMPLE_MANUAL_SPEED_BRAKE_KP 0.10f
#endif

#ifndef SIMPLE_MANUAL_SPEED_BRAKE_MAX
/* 手动模式速度刹车目标俯仰叠加上限，单位 rad(约 1.7°)。 */
#define SIMPLE_MANUAL_SPEED_BRAKE_MAX 0.030f
#endif

#ifndef SIMPLE_MANUAL_SPEED_CAP_ENABLE
/* 手动模式输出层轮速限速开关。开启后手动行驶轮速超过阈值，
 * 直接把共模轮扭矩覆写成刹车，不经过俯仰环，物理上必然刹得住。 */
#define SIMPLE_MANUAL_SPEED_CAP_ENABLE 0
#endif

#ifndef SIMPLE_MANUAL_SPEED_CAP_START
/* 手动模式输出层限速阈值，单位 m/s。 */
#define SIMPLE_MANUAL_SPEED_CAP_START 0.60f
#endif

#ifndef SIMPLE_MANUAL_SPEED_CAP_KP
/* 手动模式输出层限速刹车增益，单位 Nm/(m/s)。 */
#define SIMPLE_MANUAL_SPEED_CAP_KP 10.0f
#endif

#ifndef SIMPLE_MANUAL_SPEED_CAP_MAX
/* 手动模式输出层限速刹车最大扭矩，单位 Nm。 */
#define SIMPLE_MANUAL_SPEED_CAP_MAX 20.0f
#endif

#ifndef SIMPLE_MANUAL_SPEED_CAP_PITCH
/* 手动输出层限速刹车只在 |pitch| 小于该值(rad)时介入；
 * 大俯仰救车阶段不能覆写俯仰环，否则会前栽。 */
#define SIMPLE_MANUAL_SPEED_CAP_PITCH 0.150f
#endif

#ifndef SIMPLE_MANUAL_STOP_KP
/* 松手停车刹车增益，单位 rad/(m/s)；目标速度 0，纯速度刹停不追位置。
 * 限幅收紧后靠更高的比例增益尽快进入 4° 刹车倾角。 */
#define SIMPLE_MANUAL_STOP_KP 8.0f
#endif

#ifndef SIMPLE_MANUAL_STOP_MAX
/* 松手停车刹车目标俯仰上限，单位 rad(约 4.0°)。 */
#define SIMPLE_MANUAL_STOP_MAX 0.070f
#endif

#ifndef SIMPLE_MANUAL_STOP_V
/* 松手停车完成判速，单位 m/s；低于该值后恢复自动保持。
 * 0.06 太严, 转向后轮速噪声读 0.1-0.3 一直卡在刹车状态, 摆腿
 * 反复修正反而扰动 pitch。0.15 让低速快速交还自动位置保持。 */
#define SIMPLE_MANUAL_STOP_V 0.15f
#endif

#ifndef SIMPLE_MANUAL_PITCH_GAIN
/* 手动 W/S 速度误差 -> 目标俯仰角增益，单位 rad/(m/s)。
 * v3 腿摆弱, 前进/后退必须靠目标 pitch 偏移让平衡环输出轮扭矩:
 * 按 W(manual_v>0) 车慢于目标 -> 目标 pitch 前倾 -> 轮扭矩向前推。
 * 0.04 rad/(m/s): 1.2m/s 误差 -> 4.8° 前倾, 与松手刹车限幅同量级。 */
#define SIMPLE_MANUAL_PITCH_GAIN 0.12f
#endif

#ifndef SIMPLE_MANUAL_PITCH_MAX
/* 手动行驶目标俯仰角上限，单位 rad(约 8°)；防止加速指令把车压成趴地。 */
#define SIMPLE_MANUAL_PITCH_MAX 0.140f
#endif

#ifndef SIMPLE_MANUAL_PITCH_GATE
/* 手动行驶前倾角门控，单位 rad；实测俯仰超过该值时自动撤掉驱动指令，
 * 让车先回正。限制的是姿态而不是速度。
 * 注意: 从这里到 PITCH_GATE_HARD 之间是"按比例收敛驱动", 不是硬清零。
 * 旧实现到 8.6° 直接清零, 正常行驶的瞬态就会把腿摆指令打回中位,
 * 用户看到的现象就是"腿摆摆一下又回去了"。 */
#define SIMPLE_MANUAL_PITCH_GATE 0.120f   /* v3: 超过6.9°开始按比例收敛 */
#endif

#ifndef SIMPLE_MANUAL_PITCH_GATE_HARD
/* 手动驾驶姿态硬保护阈值，单位 rad(约17°)；超过才彻底撤掉手动指令。
 * 轮上摩擦极限约 μ·m·g/2·r≈0.5Nm/轮, 手动共模扭矩上限就设在那里,
 * 超过 ~12° 基本救不回来, 所以硬门控留在这里做最后保护。 */
#define SIMPLE_MANUAL_PITCH_GATE_HARD 0.300f
#endif

#ifndef SIMPLE_MANUAL_NEUTRAL_SLEW
/* 手动行驶目标俯仰变化率上限，单位 rad/s。
 * 0.1 会给速度环加太多相位滞后，W 持续驱动时过冲成 1m/s+ 脉冲循环；
 * 0.3 实测消除“前进→后退”振荡(10s W 零后退)，只剩启动/松手瞬态。 */
#define SIMPLE_MANUAL_NEUTRAL_SLEW 0.300f
#endif

#ifndef SIMPLE_MANUAL_WS_KP
/* 手动行驶直接轮速反馈增益，单位 Nm/(m/s)。
 * 手动 W/S 不走“倾角指令”链路(重轮下会过冲成 1m/s+ 脉冲循环)，
 * 而是把轮速误差直接叠加到扭矩: tau -= K*(body_v - manual_v)。
 * 俯仰环只负责站直，速度由该轮速环收敛，W/S 对称且天然限速。 */
#define SIMPLE_MANUAL_WS_KP 3.5f
#endif

#ifndef SIMPLE_MANUAL_SPEED_KI
/* 手动 W/S 速度误差积分增益，单位 rad/(m)。
 * 纯比例速度环巡航有静差(实测 0.35 目标跑到 0.5m/s)，
 * 加慢积分把稳态速度拉到目标。 */
#define SIMPLE_MANUAL_SPEED_KI 0.060f
#endif

#ifndef SIMPLE_NEUTRAL_TAU
/* 目标俯仰角参考一阶低通时间常数，单位 s；越小目标俯仰响应越快，
 * 但太小会激发俯仰环的低周摇摆。0.18s 在刹车响应与平滑之间折中。 */
#define SIMPLE_NEUTRAL_TAU 0.180f
#endif


/* ==========================================================================
 * 离线辨识注入
 * ========================================================================== */

#ifndef SIMPLE_IDENT_NEUTRAL_ENABLE
/* 目标俯仰角方波注入开关。正常运行必须为 0，只在辨识速度响应时打开。 */
#define SIMPLE_IDENT_NEUTRAL_ENABLE 0
#endif

#ifndef SIMPLE_IDENT_NEUTRAL_START
/* 方波注入开始时间，单位 s。 */
#define SIMPLE_IDENT_NEUTRAL_START 2.0f
#endif

#ifndef SIMPLE_IDENT_NEUTRAL_AMP
/* 方波目标俯仰角幅值，单位 rad。 */
#define SIMPLE_IDENT_NEUTRAL_AMP 0.020f
#endif

#ifndef SIMPLE_IDENT_NEUTRAL_PERIOD
/* 方波周期，单位 s。 */
#define SIMPLE_IDENT_NEUTRAL_PERIOD 2.0f
#endif


/* ==========================================================================
 * 旧版直接原点控制和腿摆辅助
 * ========================================================================== */

#ifndef SIMPLE_ORIGIN_PITCH_KP
/* 旧版位置误差直接映射目标俯仰角的 P 增益；速度规划关闭时使用。 */
#define SIMPLE_ORIGIN_PITCH_KP 0.040f
#endif

#ifndef SIMPLE_ORIGIN_SIGN
/* 旧版原点控制符号。若关闭速度规划后回原点方向反掉，先查它。 */
#define SIMPLE_ORIGIN_SIGN -1.0f
#endif

#ifndef SIMPLE_ORIGIN_PITCH_KD
/* 旧版速度阻尼到目标俯仰角的 D 增益。 */
#define SIMPLE_ORIGIN_PITCH_KD 0.000f
#endif

#ifndef SIMPLE_ORIGIN_PITCH_KI
/* 旧版位置积分到目标俯仰角的 I 增益。 */
#define SIMPLE_ORIGIN_PITCH_KI 0.003f
#endif

#ifndef SIMPLE_ORIGIN_PITCH_MAX
/* 旧版目标俯仰角限幅，单位 rad。 */
#define SIMPLE_ORIGIN_PITCH_MAX 0.040f
#endif

#ifndef SIMPLE_ORIGIN_ACTIVE_PITCH_RAD
/* 只有 |pitch| 小于该值时允许原点控制介入，单位 rad。 */
#define SIMPLE_ORIGIN_ACTIVE_PITCH_RAD 0.280f
#endif

#ifndef SIMPLE_ORIGIN_LEG_KP
/* 腿摆位置环 P 增益，单位 rad/m: 位置误差 -> 腿摆偏置。
 * 腿摆移动重心, 轮子配合加减速, 车身 pitch 保持小角度,
 * 这是轮腿机器人原地保持/加减速的正确执行机构。
 * 0.08: 里程计 es 有 ±0.05 噪声, 0.2 会让腿摆跟着噪声推车,
 * 车滑动而轮子不转(编码器读不到)形成漂移循环。 */
#define SIMPLE_ORIGIN_LEG_KP 0.08f
#endif

#ifndef SIMPLE_ORIGIN_LEG_KD
/* 腿摆速度环增益，单位 rad/(m/s): 速度误差 -> 腿摆偏置。
 * 0.15 + ±2.0m/s 误差窗口: 0.2m/s 漂移摆 0.03rad(1.7°),
 * 2m/s 才摆满 0.18rad, 幅度随误差渐变, 不再一打就是满幅。 */
#define SIMPLE_ORIGIN_LEG_KD 0.02f  /* v3: 0.03m/1.5m/s, 摆腿温和 */
#endif

#ifndef SIMPLE_ORIGIN_LEG_KI
/* 腿摆位置积分增益，单位 rad/(m*s): 消除慢速静差漂移。 */
#define SIMPLE_ORIGIN_LEG_KI 0.40f
#endif

#ifndef SIMPLE_ORIGIN_LEG_MAX
/* 腿摆偏置限幅。v3 的 sw 单位是米(轮轴前向偏移), 0.18m 经反解
 * (lf += 9.545×sw) 会变成 1.72rad≈98° 的关节摆动, 用户实测"摆动
 * 角度太大"直接翻车。改为 ±0.03m ≈ 关节 ±0.29rad(±16°)。 */
#define SIMPLE_ORIGIN_LEG_MAX 0.03f
#endif

#ifndef SIMPLE_AUTO_SWING_ENABLE
/* 静止平衡是否启用自动位置环摆腿。
 * 0 = 静止时不摆腿, 只靠轮子俯仰环平衡(腿锁在名义摆角)。
 *     旧版 auto 位置环把 leg_ref 打到 -0.18, 而 v3 的 sw 单位是米,
 *     0.18m 摆幅对应关节 1.7rad≈98°, 腿大幅抽动反而翻车(用户实测)。
 * 1 = 启用旧版 auto 位置环摆腿(加减速时腿摆移动重心), 需要配合
 *     米级小限幅(如 ±0.03m)使用, 否则摆幅过大。
 */
#define SIMPLE_AUTO_SWING_ENABLE 0
#endif

#endif
