#include "robot_control.h"

#include <math.h>
#include <stdlib.h>
#include "balance.h"
#include "controllers/controllers.h"
#include "controllers/rl_jump/rl_jump_controller.h"
#include "controllers/supervisor/controller_supervisor.h"
#include "leg_kin_v3.h"
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
static int g_rl_active = 0;          /* RL 接管标志(供 supervisor 跳自救) */

void robot_control_set_rl_active(int on)
{
    g_rl_active = on ? 1 : 0;
}

void robot_control_reset(void)
{
    /* 清空低层控制器与运行时状态: ready=0 后下一帧会重新初始化
     * LegJoint PID/轮电机, wheel_speed_cap_hold 等也归零。 */
    memset(&g_control, 0, sizeof(g_control));
}

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

static SidePair clamp_leg_extension(const SensorState *s,
                                    double common, double differential)
{
    /* RL 跳跃期允许更大的腿长范围(训练动作幅度 ±0.30 超出常规 ±0.26),
     * 空闲/正常行驶仍按常规限位保护。 */
#if MODEL_V3
    float limit = (s->rl_state != 0) ? 0.06f : 0.04f;   /* v3: 单位 m */
#else
    float limit = (s->rl_state != 0) ? 0.35f : 0.26f;
#endif
    SidePair ext = {
        .left = (float)(common + differential),
        .right = (float)(common - differential)
    };
    ext.left = clampf_runtime(ext.left, -limit, limit);
    ext.right = clampf_runtime(ext.right, -limit, limit);
    return ext;
}

static void update_leg_servos(LowLevelControllers *low, SensorState *s, float swing_ref)
{
    SidePair ext = clamp_leg_extension(s, s->leg_extend, s->leg_side_extend);
#if MODEL_V3
    /* v3: 直接锁虚拟腿长 L(保持车高), 腿摆 sw 自由。
     * 直接锁关节角会与平衡冲突(轻闭链机构), 锁 L 只提供竖直支撑力。
     * 控制量仅用于调试打印, 实际输出在下方 v3 分支计算。 */
    float lf_r = 0.0f, lr_r = 0.0f, rf_r = 0.0f, rr_r = 0.0f;
    (void)ext;
#else
    float lf_r = swing_ref - ext.left;
    float lr_r = 0.4f - swing_ref - ext.left;
    float rf_r = -0.1f - swing_ref + ext.right;
    float rr_r = -0.4f + swing_ref + ext.right;
#endif

    /* 临时调试: RL_LEG_DBG=2 时打印腿伺服参考/误差 */
    static int leg_dbg2 = -1;
    static int leg_dbg2_cnt = 0;
    if (leg_dbg2 < 0) {
        const char *e = getenv("RL_LEG_DBG");
        leg_dbg2 = e ? atoi(e) : 0;
    }
    if (leg_dbg2 >= 2) {
        leg_dbg2_cnt++;
        if (leg_dbg2_cnt >= 500) {
            leg_dbg2_cnt = 0;
            printf("[LegRef] t=%.2f sw=%+.3f exL=%+.3f exR=%+.3f | "
                   "ref[lf,lr,rf,rr]=[%+.3f,%+.3f,%+.3f,%+.3f] pos=[%+.3f,%+.3f,%+.3f,%+.3f]\n",
                   s->sim_time, swing_ref, ext.left, ext.right,
                   lf_r, lr_r, rf_r, rr_r,
                   (float)s->lf_pos, (float)s->lr_pos,
                   (float)s->rf_pos, (float)s->rr_pos);
        }
    }

    int jump_phase = balance_get_jump_phase();
    if (jump_phase == 2) {
        /* 蹬伸: 腿执行器直接打扭矩(±JUMP_TORQUE -> Nm)，
         * 用爆发力把整个车弹离地面。姿态差动叠到扭矩上:
         * roll 修正(左右腿对称)、pitch 修正(前后腿差动)、
         * IMU 加速度方向修正(防前冲)。 */
        float jump_ext = JUMP_LAUNCH_EXT;
        float j_lf_r = swing_ref - jump_ext;
        float j_lr_r = 0.4f - swing_ref - jump_ext;
        float j_rf_r = -0.1f - swing_ref + jump_ext;
        float j_rr_r = -0.4f + swing_ref + jump_ext;
        float lf_e = j_lf_r - (float)s->lf_pos;
        float lr_e = j_lr_r - (float)s->lr_pos;
        float rf_e = j_rf_r - (float)s->rf_pos;
        float rr_e = j_rr_r - (float)s->rr_pos;
        float c_lf = (fabsf(lf_e) > 0.02f) ? ((lf_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float c_lr = (fabsf(lr_e) > 0.02f) ? ((lr_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float c_rf = (fabsf(rf_e) > 0.02f) ? ((rf_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float c_rr = (fabsf(rr_e) > 0.02f) ? ((rr_e > 0.0f) ?  JUMP_TORQUE : -JUMP_TORQUE) : 0.0f;
        float roll_corr = JUMP_ROLL_KP * (float)s->roll;
        float pitch_corr = JUMP_PITCH_KP * (float)s->pitch;
        float dir_corr = JUMP_DIR_KP * (float)s->acc[0];
        /* 注: lf=左前 lr=左后 rf=右前 rr=右后；pitch 分前/后，roll 分左/右。 */
        s->ctrl[0] = clampf_runtime(c_lf - roll_corr - pitch_corr - dir_corr, -10.0f, 10.0f);
        s->ctrl[1] = clampf_runtime(c_lr - roll_corr + pitch_corr + dir_corr, -10.0f, 10.0f);
        s->ctrl[3] = clampf_runtime(c_rf + roll_corr - pitch_corr - dir_corr, -10.0f, 10.0f);
        s->ctrl[4] = clampf_runtime(c_rr + roll_corr + pitch_corr + dir_corr, -10.0f, 10.0f);
        return;
    }

    if (jump_phase == 3 || jump_phase == 4) {
        /* 落地软着陆: 腿负载大(触地)时转软弹簧-阻尼，腿让位吸能；
         * 空中(负载小)仍用正常伺服保持姿态。 */
        if (jump_phase == 3) {
            /* 空中姿态控制: 前后腿差动(swing)按 pitch 修正把车摆平,
             * 左右腿差动(ext)按 roll 修正防侧翻。
             * 符号: pitch 正(后仰) -> swing 减小让车头抬起回正,
             * roll 正(右侧沉) -> 右腿伸/左腿收。 */
            swing_ref += JUMP_AIR_PITCH_KP * (float)s->pitch;
            ext.left  -= JUMP_AIR_ROLL_KP * (float)s->roll;
            ext.right += JUMP_AIR_ROLL_KP * (float)s->roll;
        }
        float load = fabsf((float)s->lf_torque) + fabsf((float)s->lr_torque)
                   + fabsf((float)s->rf_torque) + fabsf((float)s->rr_torque);
#if MODEL_V3
        float sw_dev = swing_ref - BALANCE_LEG_SW_NOM;
        float lf_ref, lr_ref, rf_ref, rr_ref;
        leg_kin_v3_inverse(sw_dev, ext.left, ext.right,
                           &lf_ref, &lr_ref, &rf_ref, &rr_ref);
#else
        float lf_ref = swing_ref - ext.left;
        float lr_ref = 0.4f - swing_ref - ext.left;
        float rf_ref = -0.1f - swing_ref + ext.right;
        float rr_ref = -0.4f + swing_ref + ext.right;
#endif
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

#if MODEL_V3
    /* v3: 锁虚拟腿长 L(保持车高), 腿摆 sw 自由。
     * 直接锁关节角会与平衡冲突(轻闭链机构, 实测 KPL>=1 都翻车);
     * 锁 L 只提供竖直支撑力(车重 23.5N), 不影响 pitch 平衡。
     *   L  = L_NOM + J_L0*lf + J_L1*lr (左), 右腿镜像
     *   tauL = -KP_L*(L-L_NOM) - KD_L*Ldot   (虚拟竖直力)
     *   关节 ctrl = J_L*tauL / LEG_GAIN(50) */
    float leg_sw_dbg, leg_l, leg_r;
    leg_kin_v3_forward((float)s->lf_pos, (float)s->lr_pos,
                       (float)s->rf_pos, (float)s->rr_pos,
                       &leg_sw_dbg, &leg_l, &leg_r);
    float sw_dot_dbg, l_dot, r_dot;
    leg_kin_v3_velocities((float)s->lf_vel, (float)s->lr_vel,
                          (float)s->rf_vel, (float)s->rr_vel,
                          &sw_dot_dbg, &l_dot, &r_dot);
    /* 腿长目标用中位(重心在轮轴正上方), 而非名义装配位。
     * 腿长伺服保持满刚度支撑(弱化会让腿塌陷, 实测 L 掉到 0.09)。 */
    float tau_l = -(LEG_V3_SERVO_KP * (leg_l - LEG_V3_L_MID)
                    + LEG_V3_SERVO_KD * l_dot);
    float tau_r = -(LEG_V3_SERVO_KP * (leg_r - LEG_V3_L_MID)
                    + LEG_V3_SERVO_KD * r_dot);
    /* 腿摆 sw 软锁: 目标 = balance 输出的 swing_ref(含速度/位置环的
     * 腿摆指令, 加减速时腿前后摆辅助; 由 leg_kin_v3_forward 保证单位一致)。
     * 死区内自由(平衡依赖腿摆柔性), 死区外拉回防过度漂移。 */
    float sw_l = LEG_V3_SW_NOM + LEG_V3_J_S0 * (float)s->lf_pos
                                + LEG_V3_J_S1 * (float)s->lr_pos;
    float sw_r = LEG_V3_SW_NOM - LEG_V3_J_S0 * (float)s->rf_pos
                                - LEG_V3_J_S1 * (float)s->rr_pos;
    /* 腿摆参考直接取 balance 输出的 swing_ref(含手动 W/S 的腿摆指令)。
     * A/D 纯转向时 swing_ref=SW_NOM -> 腿锁中位, 只差速转;
     * W/S 时 swing_ref 随速度误差前后摆 -> 平衡环轮子驱动前进, pitch 恒 0。 */
    float sw_ref_l = swing_ref;
    float sw_ref_r = swing_ref;
    float e_sw_l = sw_l - sw_ref_l;
    float e_sw_r = sw_r - sw_ref_r;
    /* 自动静止用软腿(大死区+弱刚度): 腿在死区内自由, 停在自然平衡点
     * (sw≈+0.009), 静止稳定不漂移。手动驾驶用硬腿(小死区+强刚度+
     * 左右各自阻尼): 腿锁位不晃动, 差模(剪刀)摆动也被阻尼。 */
    /* 前进/后退(W/S)和松手刹车时腿锁位(防摆腿乱动);
     * A/D 纯转向时腿恢复软模式: 闭链腿会吸收 yaw-pitch 耦合,
     * 硬腿会把耦合全传给车身 -> pitch 扰动 -> 翻车(实测)。 */
    int manual_leg_stiff = (balance_get_manual_active() &&
                            fabsf(balance_get_manual_v()) >= 0.05f) ||
                           balance_get_post_manual();
    float sw_kp = manual_leg_stiff ? LEG_V3_SW_KP : 300.0f;
    float sw_kd = manual_leg_stiff ? LEG_V3_SW_KD : 10.0f;
    float sw_dead = manual_leg_stiff ? LEG_V3_SW_DEAD : 0.02f;
    float sw_l_dot, sw_r_dot;
    if (manual_leg_stiff) {
        /* 左右腿各自的速度阻尼(不是平均速度): 差模(剪刀)摆动也被阻尼,
         * 否则转向时轮子反作用力会把左右腿剪开, 高频晃动(用户实测)。 */
        sw_l_dot = LEG_V3_J_S0 * (float)s->lf_vel
                 + LEG_V3_J_S1 * (float)s->lr_vel;
        sw_r_dot = -LEG_V3_J_S0 * (float)s->rf_vel
                  - LEG_V3_J_S1 * (float)s->rr_vel;
    } else {
        sw_l_dot = sw_dot_dbg;
        sw_r_dot = sw_dot_dbg;
    }
    float e_sw_l_dead = e_sw_l;
    float e_sw_r_dead = e_sw_r;
    if (fabsf(e_sw_l_dead) > sw_dead)
        e_sw_l_dead -= (e_sw_l_dead > 0) ? sw_dead : -sw_dead;
    else
        e_sw_l_dead = 0.0f;
    if (fabsf(e_sw_r_dead) > sw_dead)
        e_sw_r_dead -= (e_sw_r_dead > 0) ? sw_dead : -sw_dead;
    else
        e_sw_r_dead = 0.0f;
    float tau_sw_l = -(sw_kp * e_sw_l_dead + sw_kd * sw_l_dot);
    float tau_sw_r = -(sw_kp * e_sw_r_dead + sw_kd * sw_r_dot);
    float tau_lf, tau_lr, tau_rf, tau_rr;
    leg_kin_v3_forces(tau_l, tau_sw_l, tau_r, tau_sw_r,
                      &tau_lf, &tau_lr, &tau_rf, &tau_rr);
    s->ctrl[0] = clampf_runtime(tau_lf / LEG_V3_LEG_GAIN, -10.0f, 10.0f);
    s->ctrl[1] = clampf_runtime(tau_lr / LEG_V3_LEG_GAIN, -10.0f, 10.0f);
    s->ctrl[3] = clampf_runtime(tau_rf / LEG_V3_LEG_GAIN, -10.0f, 10.0f);
    s->ctrl[4] = clampf_runtime(tau_rr / LEG_V3_LEG_GAIN, -10.0f, 10.0f);
#else
    LegJointSetRef(&low->lf, lf_r);
    LegJointSetRef(&low->lr, lr_r);
    LegJointSetRef(&low->rf, rf_r);
    LegJointSetRef(&low->rr, rr_r);

    s->ctrl[0] = LegJointUpdate(&low->lf, (float)s->lf_pos, (float)s->lf_vel, 0);
    s->ctrl[1] = LegJointUpdate(&low->lr, (float)s->lr_pos, (float)s->lr_vel, 0);
    s->ctrl[3] = LegJointUpdate(&low->rf, (float)s->rf_pos, (float)s->rf_vel, 0);
    s->ctrl[4] = LegJointUpdate(&low->rr, (float)s->rr_pos, (float)s->rr_vel, 0);
#endif
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
#if MODEL_V3
    float wheel_v_m = (float)(-((s->lw_vel + s->rw_vel) * 0.5) * WHEEL_RADIUS);
#else
    float wheel_v_m = (float)(((s->lw_vel - s->rw_vel) * 0.5) * 0.058);
#endif
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
#if MODEL_V3
    float wheel_v_m = (float)(-((s->lw_vel + s->rw_vel) * 0.5) * WHEEL_RADIUS);
#else
    float wheel_v_m = (float)(((s->lw_vel - s->rw_vel) * 0.5) * 0.058);
#endif
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
#if MODEL_V3
        s->ctrl[5] = (pc + r_tau) / 10.0;
#else
        s->ctrl[5] = (-pc + r_tau) / 10.0;
#endif
        return;
    }

    s->ctrl[2] = (pitch_mix - yaw_mix) / 10.0;
#if MODEL_V3
    s->ctrl[5] = (pitch_mix + yaw_mix) / 10.0;
#else
    s->ctrl[5] = -(pitch_mix + yaw_mix) / 10.0;
#endif
}

void robot_control_step(SensorState *s, double dt)
{
    init_low_level_controllers(&g_control.low, dt);

    BalanceSensorFrame frame = make_balance_frame(s);
    BalanceOutput out;
    balance_update_frame(&frame, dt, &out);
    copy_balance_debug(s, &out);

    /* 控制器模式切换: 以 PID 三环输出为基线, supervisor 决定是否叠加
     * LQR 扰动或由能量整形完全接管(见 controllers/supervisor/)。 */
    ControlCmd cmd = {0};
    cmd.wheel_tau = out.wheel_tau;
    cmd.yaw_tau = out.yaw_tau;
    cmd.leg_swing = out.leg_swing;
    cmd.active = 1;
    controller_supervisor_set_rl_busy(g_rl_active);
    controller_supervisor_update(s, dt, &cmd);

    /* RL 跳跃控制器: 空闲 active=0; 触发后 active=1, 直接输出全部 6 个
     * 执行器力矩(跳跃期间完全接管, 不再叠加三环, 避免三环限制跳高)。
     * 必须在腿伺服/轮混控之前调用(设置 s->rl_state 供限位放宽判断)。 */
    RlJumpCmd rj;
    rl_jump_controller_update(s, dt, &rj);

    g_control.pitch_tau = cmd.wheel_tau;
    g_control.yaw_tau = cmd.yaw_tau;

    /* LQR/能量整形可能给出腿摆/腿长扰动: 通过 leg_extend/leg_side_extend
     * 通道传给腿伺服, 与 PID 的 leg_swing 参考叠加。 */
    if (cmd.leg_len != 0.0f || cmd.leg_len_diff != 0.0f) {
        s->leg_extend += cmd.leg_len;
        s->leg_side_extend += cmd.leg_len_diff;
    }
    update_leg_servos(&g_control.low, s, cmd.leg_swing);
    mix_wheel_outputs(s, dt);

    /* 临时调试: RL_LEG_DBG=1 时每 0.5s 打印一次腿关节位置与虚拟坐标,
     * 用于测量部署时三环作用下的腿位包络(校准训练随机化范围)。 */
    static int leg_dbg = -1;
    static int leg_dbg_cnt = 0;
    if (leg_dbg < 0) {
        const char *e = getenv("RL_LEG_DBG");
        leg_dbg = e ? atoi(e) : 0;
    }
    if (leg_dbg) {
        leg_dbg_cnt++;
        if (leg_dbg_cnt >= 500) {
            leg_dbg_cnt = 0;
            float lf = (float)s->lf_pos, lr = (float)s->lr_pos;
            float rf = (float)s->rf_pos, rr = (float)s->rr_pos;
#if MODEL_V3
            float sw, L, R;
            leg_kin_v3_forward(lf, lr, rf, rr, &sw, &L, &R);
#else
            float sw = 0.25f * ((lf - lr + 0.4f) + (0.3f - rf + rr));
            float L = 0.5f * (0.4f - lf - lr);
            float R = 0.5f * (rf + rr + 0.5f);
#endif
            printf("[LegDbg] t=%.2f lf=%+.3f lr=%+.3f rf=%+.3f rr=%+.3f "
                   "sw=%+.3f L=%+.3f R=%+.3f\n",
                   s->sim_time, lf, lr, rf, rr, sw, L, R);
        }
    }

    /* RL 直接力矩接管: ctrl = mix*base + (1-mix)*rl, 逐通道混合。
     * ACTIVE 时 mix=0 纯 RL; PREP 时腿=强PD/轮=三环;
     * FADEOUT 时 mix 0->1 渐变平滑交还三环。 */
    if (rj.active) {
        for (int i = 0; i < 6; i++)
            s->ctrl[i] = rj.mix[i] * s->ctrl[i]
                       + (1.0f - rj.mix[i]) * rj.ctrl[i];
    }
}
