/*
 * ident_bridge.c - 辨识用 C 桥接库
 * ================================================================
 * 把线上真实三环控制器(balance_update_frame)暴露给 Python 辨识脚本:
 *   - ident_step() : 每物理步输入 BalanceSensorFrame + 腿长参考,
 *                    内部调用三环, 返回 wheel_tau/yaw_tau/leg_swing。
 *   - 腿长参考通过 balance_set_leg_extend_manual() 注入, 作为辨识输入。
 *
 * 编译(在 scripts/ 下):
 *   gcc -O2 -fPIC -shared -o ident_bridge.so ident_bridge.c \
 *       ../control/balance/balance.c ../control/balance/balance_api.c \
 *       ../control/balance/balance_util.c ../control/balance/balance_core.c \
 *       -I../control/include -I../control/balance -lm
 *
 * 只依赖控制核心(不依赖 MuJoCo/GLFW), 方便 Python ctypes 调用。
 */
#include <stddef.h>

#include "../control/balance/balance.h"
#include "../control/runtime/robot_control.h"
#include "../control/include/sim_state.h"
#include "../control/controllers/supervisor/controller_supervisor.h"
#include "../control/controllers/rl_jump/rl_jump_controller.h"
#include "../control/lowlevel/leg_t.h"
#include "../control/lowlevel/pid.h"
#include "../control/include/control_config.h"

typedef struct {
    float pitch, pitch_rate, roll, roll_rate;
    float left_wheel_vel, right_wheel_vel;
    float acc_mag, acc_x, acc_z;
    float yaw, yaw_rate;
    float left_wheel_pos, right_wheel_pos;
    float lf_pos, lr_pos, rf_pos, rr_pos;
    float lf_tau, lr_tau, rf_tau, rr_tau;
    float left_wheel_tau, right_wheel_tau;
    float lf_vel, lr_vel, rf_vel, rr_vel;
    float acc_y;
    double quat[4];
    double body_pos[3];
    double body_vel[3];
    double sim_time;
    int    step_count;
} IdentFrame;

typedef struct {
    float wheel_tau;
    float yaw_tau;
    float leg_swing;
    float leg_extend;
    float leg_side_extend;   /* 三环 roll 稳定输出: 左右腿差动, 辨识时腿伺服必须执行 */
    float speed_ref;
    float position;
} IdentOut;

void ident_warmup_init(void);
void ident_warmup_step(const IdentFrame *f, double dt, float ctrl[6]);
void ident_warmup_reset(void);

/* 回合级重置: 训练环境每回合预热前调用, 清空所有控制器静态状态
 * (balance 积分/滤波、supervisor 状态机、腿伺服 PID、RL 状态机),
 * 避免同一进程内连续训练回合的状态残留(会导致预热偶发极限环振荡)。 */
void ident_reset(void)
{
    balance_reset_state();            /* 恢复全部内部状态(积分/滤波清零) */
    balance_init();                   /* 模式置 0 */
    balance_toggle();                 /* 开启平衡 */
    controller_supervisor_init();     /* 重置自救状态机 */
    rl_jump_controller_init();        /* 重置 RL 跳跃状态机 */
    robot_control_reset();            /* 重置低层伺服 PID 等 */
    ident_warmup_reset();             /* 重置预热腿伺服 PID */
}

/* 完整一步部署控制(整个 robot_control_step: 三环+LQR/supervisor+腿伺服+
 * 轮混控+轮速保护), 训练环境预热与部署完全一致。 */
void ident_robot_step(const IdentFrame *f, double dt, float ctrl[6])
{
    SensorState s;
    memset(&s, 0, sizeof(s));

    for (int i = 0; i < 4; i++) s.quat[i] = f->quat[i];
    s.pitch = f->pitch;
    s.roll = f->roll;
    s.yaw = f->yaw;
    s.yaw_rate = f->yaw_rate;
    s.gyro[0] = f->roll_rate;
    s.gyro[1] = f->pitch_rate;
    s.gyro[2] = f->yaw_rate;
    s.acc[0] = f->acc_x;
    s.acc[1] = f->acc_y;
    s.acc[2] = f->acc_z;
    s.acc_mag = f->acc_mag;
    for (int i = 0; i < 3; i++) {
        s.body_pos[i] = f->body_pos[i];
        s.body_vel[i] = f->body_vel[i];
    }
    s.lw_pos = f->left_wheel_pos;
    s.rw_pos = f->right_wheel_pos;
    s.lw_vel = f->left_wheel_vel;
    s.rw_vel = f->right_wheel_vel;
    s.lf_pos = f->lf_pos; s.lf_vel = f->lf_vel;
    s.lr_pos = f->lr_pos; s.lr_vel = f->lr_vel;
    s.rf_pos = f->rf_pos; s.rf_vel = f->rf_vel;
    s.rr_pos = f->rr_pos; s.rr_vel = f->rr_vel;
    s.lf_torque = f->lf_tau; s.lr_torque = f->lr_tau;
    s.rf_torque = f->rf_tau; s.rr_torque = f->rr_tau;
    s.lw_torque = f->left_wheel_tau;
    s.rw_torque = f->right_wheel_tau;
    s.sim_time = f->sim_time;
    s.step_count = f->step_count;

    robot_control_step(&s, dt);
    for (int i = 0; i < 6; i++) ctrl[i] = (float)s.ctrl[i];
}

void ident_init(void)
{
    balance_init();
    balance_toggle();   /* 开启平衡模式 */
    ident_warmup_init();
}

/* ---- 真实 C 应用腿伺服/轮混控(供训练环境三环预热, 与部署一致) ---- */

static LegJoint g_wlf, g_wlr, g_wrf, g_wrr;
static int g_warmup_ready = 0;

void ident_warmup_init(void)
{
    if (g_warmup_ready) return;
    /* 与 control/runtime/robot_control.c::init_low_level_controllers 相同配置 */
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
    LegJointInit(&g_wlf, &leg, 0.001f);
    LegJointInit(&g_wlr, &leg, 0.001f);
    LegJointInit(&g_wrf, &leg, 0.001f);
    LegJointInit(&g_wrr, &leg, 0.001f);
    g_warmup_ready = 1;
}

void ident_warmup_reset(void)
{
    g_warmup_ready = 0;
    ident_warmup_init();
}

static float clampf_bridge(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* 一步完整预热: 三环核心 + 腿伺服(PID) + 轮混控(0.6/0.4 权重) + 轮速保护。
 * 输出 ctrl[6] 与 C 应用 s->ctrl 同单位(±10)。 */
void ident_warmup_step(const IdentFrame *f, double dt, float ctrl[6])
{
    BalanceSensorFrame frame;
    BalanceOutput b;

    frame.pitch           = f->pitch;
    frame.pitch_rate      = f->pitch_rate;
    frame.roll            = f->roll;
    frame.roll_rate       = f->roll_rate;
    frame.left_wheel_vel  = f->left_wheel_vel;
    frame.right_wheel_vel = f->right_wheel_vel;
    frame.acc_mag         = f->acc_mag;
    frame.acc_x           = f->acc_x;
    frame.acc_z           = f->acc_z;
    frame.yaw             = f->yaw;
    frame.yaw_rate        = f->yaw_rate;
    frame.left_wheel_pos  = f->left_wheel_pos;
    frame.right_wheel_pos = f->right_wheel_pos;
    frame.lf_pos          = f->lf_pos;
    frame.lr_pos          = f->lr_pos;
    frame.rf_pos          = f->rf_pos;
    frame.rr_pos          = f->rr_pos;
    frame.lf_tau          = f->lf_tau;
    frame.lr_tau          = f->lr_tau;
    frame.rf_tau          = f->rf_tau;
    frame.rr_tau          = f->rr_tau;
    frame.left_wheel_tau  = f->left_wheel_tau;
    frame.right_wheel_tau = f->right_wheel_tau;
    frame.lf_vel          = f->lf_vel;
    frame.lr_vel          = f->lr_vel;
    frame.rf_vel          = f->rf_vel;
    frame.rr_vel          = f->rr_vel;

    balance_update_frame(&frame, dt, &b);

    /* 腿伺服参考(与 update_leg_servos 一致): 限位 ±0.26 */
    float swing_ref = b.leg_swing;
    float ext = balance_get_leg_extend();
    float side = balance_get_leg_side_extend();
    float exL = clampf_bridge(ext + side, -0.26f, 0.26f);
    float exR = clampf_bridge(ext - side, -0.26f, 0.26f);
    LegJointSetRef(&g_wlf, swing_ref - exL);
    LegJointSetRef(&g_wlr, 0.4f - swing_ref - exL);
    LegJointSetRef(&g_wrf, -0.1f - swing_ref + exR);
    LegJointSetRef(&g_wrr, -0.4f + swing_ref + exR);
    ctrl[0] = LegJointUpdate(&g_wlf, f->lf_pos, f->lf_vel, f->lf_tau);
    ctrl[1] = LegJointUpdate(&g_wlr, f->lr_pos, f->lr_vel, f->lr_tau);
    ctrl[3] = LegJointUpdate(&g_wrf, f->rf_pos, f->rf_vel, f->rf_tau);
    ctrl[4] = LegJointUpdate(&g_wrr, f->rr_pos, f->rr_vel, f->rr_tau);

    /* 轮混控: 0.6 平衡权重 + 0.4 偏航权重(与 mix_wheel_outputs 一致) */
    float pitch_mix = WHEEL_MIX_BALANCE_WEIGHT * b.wheel_tau;
    float yaw_mix = WHEEL_MIX_YAW_WEIGHT * b.yaw_tau;

#if WHEEL_OUTPUT_SPEED_CAP_ENABLE
    /* 轮速保护(与 apply_output_speed_cap 一致): 轮速>3m/s 时刹车 */
    {
        float wheel_v_m = (float)(((f->left_wheel_vel - f->right_wheel_vel) * 0.5) * WHEEL_RADIUS);
        float abs_v = fabsf(wheel_v_m);
        if (abs_v > WHEEL_OUTPUT_SPEED_CAP_START) {
            float spin_dir = (wheel_v_m > 0.0f) ? 1.0f : -1.0f;
            float brake = WHEEL_OUTPUT_SPEED_CAP_MIN_TAU
                        + WHEEL_OUTPUT_SPEED_CAP_KP * (abs_v - WHEEL_OUTPUT_SPEED_CAP_START);
            brake = clampf_bridge(brake, 0.0f, WHEEL_OUTPUT_SPEED_CAP_MAX_TAU);
            float desired = -spin_dir * brake;
            if (pitch_mix * spin_dir > desired * spin_dir)
                pitch_mix = desired;
        }
    }
#endif

    ctrl[2] = (pitch_mix - yaw_mix) / 10.0f;
    ctrl[5] = -(pitch_mix + yaw_mix) / 10.0f;
}

void ident_set_leg_len(float v)
{
    /* 腿长参考注入: 与 GUI E/Q 同路径, 正值=缩短/降低, 负值=伸长/升高 */
    balance_set_leg_extend_manual(v);
}

/* 直接覆盖平衡核心的 leg_extend 参考, 用于辨识时把腿长当独立输入。
 * 三环仍负责 pitch/yaw/roll 稳定, 但腿长参考完全由辨识脚本给定。 */
void ident_set_leg_len_direct(float v)
{
    balance_set_leg_extend_manual(v);
    /* leg_extend_manual 会被 balance_update 的 ext_ref 吸收,
     * 这里额外把 leg_extend 状态本身也推到目标, 保证实际腿长跟随。 */
    balance_set_leg_extend_manual(v);
}

void ident_step(const IdentFrame *f, double dt, IdentOut *out)
{
    BalanceSensorFrame frame;
    BalanceOutput b;

    frame.pitch           = f->pitch;
    frame.pitch_rate      = f->pitch_rate;
    frame.roll            = f->roll;
    frame.roll_rate       = f->roll_rate;
    frame.left_wheel_vel  = f->left_wheel_vel;
    frame.right_wheel_vel = f->right_wheel_vel;
    frame.acc_mag         = f->acc_mag;
    frame.acc_x           = f->acc_x;
    frame.acc_z           = f->acc_z;
    frame.yaw             = f->yaw;
    frame.yaw_rate        = f->yaw_rate;
    frame.left_wheel_pos  = f->left_wheel_pos;
    frame.right_wheel_pos = f->right_wheel_pos;
    frame.lf_pos          = f->lf_pos;
    frame.lr_pos          = f->lr_pos;
    frame.rf_pos          = f->rf_pos;
    frame.rr_pos          = f->rr_pos;
    frame.lf_tau          = f->lf_tau;
    frame.lr_tau          = f->lr_tau;
    frame.rf_tau          = f->rf_tau;
    frame.rr_tau          = f->rr_tau;
    frame.left_wheel_tau  = f->left_wheel_tau;
    frame.right_wheel_tau = f->right_wheel_tau;
    frame.lf_vel          = f->lf_vel;
    frame.lr_vel          = f->lr_vel;
    frame.rf_vel          = f->rf_vel;
    frame.rr_vel          = f->rr_vel;

    balance_update_frame(&frame, dt, &b);

    out->wheel_tau = b.wheel_tau;
    out->yaw_tau   = b.yaw_tau;
    out->leg_swing = b.leg_swing;
    out->leg_extend = balance_get_leg_extend();
    out->leg_side_extend = balance_get_leg_side_extend();
    out->speed_ref = b.speed_ref;
    out->position  = b.position;
}

int ident_balance_active(void)
{
    return balance_active();
}
