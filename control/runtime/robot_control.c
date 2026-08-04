#include "robot_control.h"

#include <math.h>
#include <stdlib.h>
#include "balance.h"
#include "leg_t.h"
#include "pid.h"
#include "control_config.h"
#include "wheel_t.h"

/* ==========================================================================
 * 低层执行器桥接层
 * ==========================================================================
 *
 * robot_control.c 是 balance.c 和 MuJoCo 执行器之间的桥:
 *   - balance.c 输出虚拟控制量: 共模轮扭矩、偏航差动扭矩、虚拟腿摆/腿长。
 *   - 本文件负责把虚拟量变成 6 个执行器控制量。
 *
 * 低层 PID、轮扭矩混控、输出层轮速保护等可调宏统一定义在:
 *   control/include/control_config.h
 */

typedef struct {
    WheelMotor left_wheel;       /* 左轮电机环对象，保留给后续轮环实验。 */
    WheelMotor right_wheel;      /* 右轮电机环对象，保留给后续轮环实验。 */
    LegJoint lf;                 /* 左前腿角度伺服。 */
    LegJoint lr;                 /* 左后腿角度伺服。 */
    LegJoint rf;                 /* 右前腿角度伺服。 */
    LegJoint rr;                 /* 右后腿角度伺服。 */
    int ready;                   /* 布尔值，所有 PID 对象是否已初始化。 */
} LowLevelControllers;

typedef struct {
    float pitch_tau;             /* Nm，balance.c 最新输出的俯仰共模扭矩。 */
    float yaw_tau;               /* Nm，balance.c 最新输出的偏航差动扭矩。 */
    float wheel_speed_cap_hold;  /* s，输出层轮速超限持续时间。 */
    LowLevelControllers low;     /* 腿和保留轮环的有状态 PID 对象。 */
} RobotControlRuntime;

typedef struct {
    float left;                  /* rad，左侧虚拟腿伸长指令。 */
    float right;                 /* rad，右侧虚拟腿伸长指令。 */
} SidePair;

static RobotControlRuntime g_control;

static float clampf_runtime(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void init_low_level_controllers(LowLevelControllers *c, double dt)
{
    if (c->ready) return;

    /*
     * 当前顶层控制器直接写扭矩指令，但仍初始化 WheelMotor 对象。
     * 这样能记录预期的硬件环结构，也方便后续轮电机环实验留在本文件内。
     */
    /* 轮速环: P+积分+微分。齿轮后轮惯量 0.052，纯 P 会欠阻尼振荡，
     * 需要 Kd 才能收敛到目标轮速(实测 P-only 超调 10 倍)。 */
    PID_Init_Config_s spd = {0.26f, 0.03f, 0.10f, 5, -5, 0.01f, 0, 0, 0.3f, 0.1f, 0, 0};
    PID_Init_Config_s cur = {0.50f, 0.01f, 0, 5, -5, 0, 0, 0, 0.3f, 0.1f, 0, 0};
    WheelMotor_Init_s w = {
        .control_settings = {
            SPEED_LOOP, SPEED_LOOP,
            MOTOR_DIR_NORMAL, FEEDBACK_DIR_NORMAL,
            MOTOR_FEED, MOTOR_FEED
        },
        .speed_PID_cfg = spd,
        .current_PID_cfg = cur,
        .speed_div = 2,
        .current_div = 1
    };
    WheelMotorInit(&c->left_wheel, &w, (float)dt);
    /*
     * 右轮速度环反馈必须用原始编码器速度(不取反)。
     * 右轮关节轴向与左轮相反，纯转向时两轮目标原始速度相同(左前右后)；
     * 若 ref 和反馈都取反，会变成正反馈，轮速发散(实测右轮冲到 ±50rad/s)。
     */
    WheelMotorInit(&c->right_wheel, &w, (float)dt);

    PID_Init_Config_s angle = {
        LEG_ANGLE_KP, LEG_ANGLE_KI, LEG_ANGLE_KD,
        LEG_ANGLE_MAX, -LEG_ANGLE_MAX, LEG_ANGLE_DEADBAND,
        LEG_ANGLE_IMPROVE,
        1.2f, 0.25f, 0.08f, LEG_ANGLE_OUT_LPF, 0.0f
    };
    LegJoint_Init_s leg = {
        .control_settings = {
            LEG_ANGLE_LOOP, LEG_ANGLE_LOOP,
            LEG_DIR_NORMAL, LEG_DIR_NORMAL,
            LEG_FEED_MOTOR, LEG_FEED_MOTOR, LEG_FEED_MOTOR
        },
        .angle_PID_cfg = angle,
        .angle_div = 5
    };
    LegJointInit(&c->lf, &leg, (float)dt);
    LegJointInit(&c->lr, &leg, (float)dt);
    LegJointInit(&c->rf, &leg, (float)dt);
    LegJointInit(&c->rr, &leg, (float)dt);
    c->ready = 1;
}

static BalanceSensorFrame make_balance_frame(const SensorState *s)
{
    BalanceSensorFrame frame = {
        .pitch = (float)s->pitch,
        .pitch_rate = (float)s->gyro[1],
        .roll = (float)s->roll,
        .roll_rate = (float)s->gyro[0],
        .left_wheel_vel = (float)s->lw_vel,
        .right_wheel_vel = (float)s->rw_vel,
        .acc_mag = (float)s->acc_mag,
        .acc_x = (float)s->acc[0],
        .acc_z = (float)s->acc[2],
        .yaw = (float)s->yaw,
        .yaw_rate = (float)s->yaw_rate,
        .left_wheel_pos = (float)s->lw_pos,
        .right_wheel_pos = (float)s->rw_pos,
        .lf_pos = (float)s->lf_pos,
        .lr_pos = (float)s->lr_pos,
        .rf_pos = (float)s->rf_pos,
        .rr_pos = (float)s->rr_pos,
        .lf_tau = (float)s->lf_torque,
        .lr_tau = (float)s->lr_torque,
        .rf_tau = (float)s->rf_torque,
        .rr_tau = (float)s->rr_torque,
        .left_wheel_tau = (float)s->lw_torque,
        .right_wheel_tau = (float)s->rw_torque,
        .lf_vel = (float)s->lf_vel,
        .lr_vel = (float)s->lr_vel,
        .rf_vel = (float)s->rf_vel,
        .rr_vel = (float)s->rr_vel
    };
    return frame;
}

static void copy_balance_debug(SensorState *s, const BalanceOutput *out)
{
    s->est_v = out->speed_ref;
    s->est_x = out->position;
    s->target_pitch = balance_get_neutral();
    s->wheel_trust = balance_get_wheel_trust();
    s->pos_trust = balance_get_pos_trust();
    s->airborne = balance_get_airborne();
    s->impact = balance_get_impact();
    s->slip = balance_get_slip();
    s->raw_airborne = balance_get_raw_airborne();
    s->raw_impact = balance_get_raw_impact();
    s->raw_slip = balance_get_raw_slip();
    s->raw_zupt = balance_get_raw_zupt();
    s->wheel_v_filt = balance_get_wheel_v_filt();
    s->wheel_fir = balance_get_wheel_fir();
    s->wheel_thrash = balance_get_wheel_thrash();
    s->imu_thrash = balance_get_imu_thrash();
    s->wheel_acc = balance_get_wheel_acc();
    s->imu_for_slip = balance_get_imu_for_slip();
    s->wheel_imu_res = balance_get_wheel_imu_res();
    s->leg_kin_v = balance_get_leg_kin_v();
    s->pitch_kin_v = balance_get_pitch_kin_v();
    s->wheel_body_v = balance_get_wheel_body_v();
    s->support = balance_get_support();
    s->leg_load = balance_get_leg_load();
    s->leg_swing = balance_get_leg_swing();
    s->leg_extend = balance_get_leg_extend();
    s->leg_len = balance_get_leg_len();
    s->leg_extend_manual = balance_get_leg_extend_manual();
    s->leg_side_extend = balance_get_leg_side_extend();
    s->obs_phase = balance_get_obs_phase();
    s->return_score = balance_get_return_score();
    s->capture_score = balance_get_capture_score();
    s->odom_forward = balance_get_odom_forward();
    s->contact_mode = balance_get_contact_mode();
    s->side_balance = balance_get_side_balance();
    s->ekf_s = balance_get_ekf_s();
    s->odom_x = balance_get_odom_x();
    s->odom_y = balance_get_odom_y();
    s->yaw_error = balance_get_yaw_error();
}

static SidePair clamp_leg_extension(double common, double differential)
{
    SidePair ext = {
        .left = (float)(common + differential),
        .right = (float)(common - differential)
    };
    ext.left = clampf_runtime(ext.left, -0.260f, 0.260f);
    ext.right = clampf_runtime(ext.right, -0.260f, 0.260f);
    return ext;
}

static void update_leg_servos(LowLevelControllers *low, SensorState *s, float swing_ref)
{
    SidePair ext = clamp_leg_extension(s->leg_extend, s->leg_side_extend);

    int jump_phase = balance_get_jump_phase();
    if (jump_phase == 1 || jump_phase == 2) {
        /* 跳跃下蹲/蹬伸: 腿执行器直接打扭矩(±5 -> ±250Nm/腿)，
         * 用爆发力把整个车弹离地面，而不是位置伺服平滑顶起。 */
        float jump_ext = (jump_phase == 1) ? JUMP_CROUCH_EXT : JUMP_LAUNCH_EXT;
        float lf_r = swing_ref - jump_ext;
        float lr_r = 0.4f - swing_ref - jump_ext;
        float rf_r = -0.1f - swing_ref + jump_ext;
        float rr_r = -0.4f + swing_ref + jump_ext;
        float lf_e = lf_r - (float)s->lf_pos;
        float lr_e = lr_r - (float)s->lr_pos;
        float rf_e = rf_r - (float)s->rf_pos;
        float rr_e = rr_r - (float)s->rr_pos;
        float c_lf = (fabsf(lf_e) > 0.02f) ? ((lf_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float c_lr = (fabsf(lr_e) > 0.02f) ? ((lr_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float c_rf = (fabsf(rf_e) > 0.02f) ? ((rf_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float c_rr = (fabsf(rr_e) > 0.02f) ? ((rr_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        /* 下蹲准备阶段用满增益摆平车体，蹬伸阶段用半增益保持对称。 */
        float roll_kp = (jump_phase == 1) ? JUMP_ROLL_KP : (0.5f * JUMP_ROLL_KP);
        float roll_corr = roll_kp * (float)s->roll;
        /* 到位死区: 关节已到目标附近就撤力，避免反向振荡。 */
        /* 注: lf=左前 lr=左后 rf=右前 rr=右后；pitch 补偿分前/后，roll 分左/右。 */
        s->ctrl[0] = clampf_runtime(c_lf - roll_corr, -10.0f, 10.0f);
        s->ctrl[1] = clampf_runtime(c_lr - roll_corr, -10.0f, 10.0f);
        s->ctrl[3] = clampf_runtime(c_rf + roll_corr, -10.0f, 10.0f);
        s->ctrl[4] = clampf_runtime(c_rr + roll_corr, -10.0f, 10.0f);
        return;
    }

    if (jump_phase == 3 || jump_phase == 4) {
        /* 落地软着陆: 腿负载大(触地)时转软弹簧-阻尼，腿让位吸能；
         * 空中(负载小)仍用正常伺服保持姿态。 */
        float load = fabsf((float)s->lf_torque) + fabsf((float)s->lr_torque)
                   + fabsf((float)s->rf_torque) + fabsf((float)s->rr_torque);
        float lf_ref = swing_ref - ext.left;
        float lr_ref = 0.4f - swing_ref - ext.left;
        float rf_ref = -0.1f - swing_ref + ext.right;
        float rr_ref = -0.4f + swing_ref + ext.right;
        if (load > JUMP_LAND_LOAD) {
            s->ctrl[0] = clampf_runtime(
                JUMP_LAND_KP * (lf_ref - (float)s->lf_pos) - JUMP_LAND_KD * (float)s->lf_vel,
                -JUMP_LAND_CTRL_MAX, JUMP_LAND_CTRL_MAX);
            s->ctrl[1] = clampf_runtime(
                JUMP_LAND_KP * (lr_ref - (float)s->lr_pos) - JUMP_LAND_KD * (float)s->lr_vel,
                -JUMP_LAND_CTRL_MAX, JUMP_LAND_CTRL_MAX);
            s->ctrl[3] = clampf_runtime(
                JUMP_LAND_KP * (rf_ref - (float)s->rf_pos) - JUMP_LAND_KD * (float)s->rf_vel,
                -JUMP_LAND_CTRL_MAX, JUMP_LAND_CTRL_MAX);
            s->ctrl[4] = clampf_runtime(
                JUMP_LAND_KP * (rr_ref - (float)s->rr_pos) - JUMP_LAND_KD * (float)s->rr_vel,
                -JUMP_LAND_CTRL_MAX, JUMP_LAND_CTRL_MAX);
            return;
        }
    }

    LegJointSetRef(&low->lf, swing_ref - ext.left);
    LegJointSetRef(&low->lr, 0.4f - swing_ref - ext.left);
    LegJointSetRef(&low->rf, -0.1f - swing_ref + ext.right);
    LegJointSetRef(&low->rr, -0.4f + swing_ref + ext.right);

    s->ctrl[0] = LegJointUpdate(&low->lf, (float)s->lf_pos, (float)s->lf_vel, 0);
    s->ctrl[1] = LegJointUpdate(&low->lr, (float)s->lr_pos, (float)s->lr_vel, 0);
    s->ctrl[3] = LegJointUpdate(&low->rf, (float)s->rf_pos, (float)s->rf_vel, 0);
    s->ctrl[4] = LegJointUpdate(&low->rr, (float)s->rr_pos, (float)s->rr_vel, 0);
}

static float apply_output_speed_cap(SensorState *s, double dt, float pitch_mix)
{
#if WHEEL_OUTPUT_SPEED_CAP_ENABLE
    /*
     * 最终输出保护:
     *   - 在执行器边界监控共模轮速。
     *   - 只改写俯仰共模扭矩。
     *   - 保持偏航差动扭矩独立，便于推理和调参。
     */
    float wheel_v_m = (float)(((s->lw_vel - s->rw_vel) * 0.5) * 0.058);
    float abs_wheel_v = fabsf(wheel_v_m);
    int cap_hold_allowed = fabsf((float)s->pitch) < WHEEL_OUTPUT_SPEED_CAP_PITCH_GATE;

    if (abs_wheel_v > WHEEL_OUTPUT_SPEED_CAP_START && cap_hold_allowed) {
        g_control.wheel_speed_cap_hold += (float)dt;
        g_control.wheel_speed_cap_hold = clampf_runtime(g_control.wheel_speed_cap_hold,
                                                0.0f, WHEEL_OUTPUT_SPEED_CAP_HOLD_MAX);
    } else {
        g_control.wheel_speed_cap_hold -= WHEEL_OUTPUT_SPEED_CAP_DECAY * (float)dt;
        g_control.wheel_speed_cap_hold = clampf_runtime(g_control.wheel_speed_cap_hold,
                                                0.0f, WHEEL_OUTPUT_SPEED_CAP_HOLD_MAX);
    }

    if (abs_wheel_v > WHEEL_OUTPUT_SPEED_CAP_START) {
        float spin_dir = (wheel_v_m > 0.0f) ? 1.0f : -1.0f;
        float brake_tau = WHEEL_OUTPUT_SPEED_CAP_MIN_TAU
                         + WHEEL_OUTPUT_SPEED_CAP_KP * (abs_wheel_v - WHEEL_OUTPUT_SPEED_CAP_START)
                         + WHEEL_OUTPUT_SPEED_CAP_HOLD_KP * g_control.wheel_speed_cap_hold;
        brake_tau = clampf_runtime(brake_tau, 0.0f, WHEEL_OUTPUT_SPEED_CAP_MAX_TAU);

        float desired_common = -spin_dir * brake_tau;
        if (pitch_mix * spin_dir > desired_common * spin_dir)
            pitch_mix = desired_common;
    }
#else
    (void)s;
    (void)dt;
#endif
    return pitch_mix;
}

static float apply_manual_speed_cap(SensorState *s, float pitch_mix)
{
#if SIMPLE_MANUAL_SPEED_CAP_ENABLE
    /*
     * 手动驾驶输出层限速: 轮速超过阈值时，直接把共模轮扭矩覆写成刹车，
     * 不经过俯仰环。这样无论俯仰环想怎么驱动，轮子物理上都会被刹住，
     * 手动行驶既不会越跑越快，松手后也能真正停下来。
     */
    float wheel_v_m = (float)(((s->lw_vel - s->rw_vel) * 0.5) * 0.058);
    float abs_v = fabsf(wheel_v_m);
    /* 手动中/松手停车刹车阶段生效，自动回程模式绝不介入。 */
    if (balance_get_manual_active() || balance_get_post_manual()) {
        if (abs_v > SIMPLE_MANUAL_SPEED_CAP_START) {
            float brake = SIMPLE_MANUAL_SPEED_CAP_KP *
                          (abs_v - SIMPLE_MANUAL_SPEED_CAP_START);
            brake = clampf_runtime(brake, 0.0f, SIMPLE_MANUAL_SPEED_CAP_MAX);
            pitch_mix = (wheel_v_m > 0.0f) ? -brake : brake;
        }
    }
#else
    (void)s;
#endif
    return pitch_mix;
}

static void mix_wheel_outputs(SensorState *s, double dt)
{
    float pitch_mix = WHEEL_MIX_BALANCE_WEIGHT * g_control.pitch_tau;
    float yaw_mix = WHEEL_MIX_YAW_WEIGHT * g_control.yaw_tau;

    pitch_mix = apply_output_speed_cap(s, dt, pitch_mix);
    pitch_mix = apply_manual_speed_cap(s, pitch_mix);

    /* A/D 纯转向: 差速轮速环 + 俯仰 PD。
     * 先用环境变量定位正确轮速方向:
     *   TURN_SPIN_SIGN=+1/-1 翻转目标轮速符号
     *   TURN_PC_MAX=0        关闭俯仰校正(诊断用)
     * 找到正确符号后固化参数。 */
    static int turn_branch = -1;
    if (turn_branch < 0) {
        const char *e = getenv("TURN_BRANCH");
        turn_branch = e ? atoi(e) : 0;
    }
    if (turn_branch &&
        balance_get_manual_active() &&
        fabsf(balance_get_manual_v()) < 0.05f &&
        fabsf(balance_get_manual_yaw()) > 0.10f) {
        static float spin_sign = 0.0f;          /* 待 A/B 确认 */
        static int spin_sign_init = 0;
        static float pc_max = 0.0f;
        static int pc_max_init = 0;
        if (!spin_sign_init) {
            const char *e = getenv("TURN_SPIN_SIGN");
            spin_sign = e ? (float)atof(e) : 1.0f;
            spin_sign_init = 1;
        }
        if (!pc_max_init) {
            const char *e = getenv("TURN_PC_MAX");
            pc_max = e ? (float)atof(e) : 10.0f;
            pc_max_init = 1;
        }
        float pc = 240.0f * (float)s->pitch + 70.0f * (float)s->gyro[1];
        pc = pc > pc_max ? pc_max : (pc < -pc_max ? -pc_max : pc);
        /* 目标轮速 = 偏航角速度 × 半轮距 / 轮半径(符号待确认) */
        float spin_radps = spin_sign * balance_get_manual_yaw() * 0.18f / WHEEL_RADIUS;
        WheelMotorSetRef(&g_control.low.left_wheel, spin_radps);
        WheelMotorSetRef(&g_control.low.right_wheel, spin_radps);
        float l_tau = WheelMotorUpdate(&g_control.low.left_wheel,
                                       (float)s->lw_pos, (float)s->lw_vel,
                                       (float)s->lw_torque);
        float r_tau = WheelMotorUpdate(&g_control.low.right_wheel,
                                       (float)s->rw_pos, (float)s->rw_vel,
                                       (float)s->rw_torque);
        l_tau = l_tau > 1.5f ? 1.5f : (l_tau < -1.5f ? -1.5f : l_tau);
        r_tau = r_tau > 1.5f ? 1.5f : (r_tau < -1.5f ? -1.5f : r_tau);
        float lw_s = (float)s->lw_vel;
        float rw_s = (float)s->rw_vel;
        if (lw_s > 6.0f && l_tau > 0.0f) l_tau = 0.0f;
        if (lw_s < -6.0f && l_tau < 0.0f) l_tau = 0.0f;
        if (rw_s > 6.0f && r_tau > 0.0f) r_tau = 0.0f;
        if (rw_s < -6.0f && r_tau < 0.0f) r_tau = 0.0f;
        /* 右轮轴镜像: 俯仰校正 pc 取反才是物理共模。 */
        s->ctrl[2] = (pc + l_tau) / 10.0;
        s->ctrl[5] = (-pc + r_tau) / 10.0;
        return;
    }

    s->ctrl[2] = (pitch_mix - yaw_mix) / 10.0;
    s->ctrl[5] = -(pitch_mix + yaw_mix) / 10.0;
}

void robot_control_step(SensorState *s, double dt)
{
    init_low_level_controllers(&g_control.low, dt);

    BalanceSensorFrame frame = make_balance_frame(s);
    BalanceOutput out;
    balance_update_frame(&frame, dt, &out);
    g_control.pitch_tau = out.wheel_tau;
    g_control.yaw_tau = out.yaw_tau;

    copy_balance_debug(s, &out);
    update_leg_servos(&g_control.low, s, out.leg_swing);
    mix_wheel_outputs(s, dt);
}
