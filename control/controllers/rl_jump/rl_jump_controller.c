/*
 * rl_jump_controller.c - RL 跳跃控制器实现(直接力矩控制版)
 * ================================================================
 * 与训练环境 scripts/jump_rl_env.py 保持同一套约定:
 *   - 观测 34 维(索引定义见 build_obs, 与 _get_obs 一致, 含离地标志)。
 *   - 动作 5 维 [-1,1] = [腿长力, 腿摆力, 左右差, 左轮, 右轮]:
 *       腿: 虚拟力矩经运动学 Jᵀ 映射到 4 个主关节(不独立驱动关节,
 *           不破坏连杆约束); 轮: 直接力矩 ×10。
 *   - 分阶段: 地面接触=跳跃爆发(腿长发力); 空中=动腿摆+轮子调落地姿态。
 *   - 动作低通 0.7/0.3(训练环境 _step 里的 _act_filt)。
 *
 * 状态机:
 *   IDLE   : active=0, 三环全权。
 *   ACTIVE : active=1, mix=0, RL 直接写全部 6 个执行器力矩, 固定爆发窗口
 *            (训练窗口一致), 只管"蹲->跳->落地", 不做长时间平衡。
 *   FADEOUT: active=1, mix 0->1 渐变(交叉淡化), 渐变完回 IDLE。
 *
 * 退出判定:
 *   - 本轮必须离地超过 RL_JUMP_MIN_AIR_H 才算"跳过";
 *   - 跳过且落地站稳保持 RL_JUMP_RECOVER_HOLD 秒 -> 提前退出交还三环;
 *   - 翻车或爆发窗口超时(RL_JUMP_TIMEOUT) -> 快速退出, 三环/supervisor 接管。
 */
#include "rl_jump_controller.h"

#include <math.h>
#include <stdio.h>

#include "control_config.h"
#include "rl_jump_net.h"
#include "../../balance/balance.h"

/* 训练环境常量(scripts/jump_rl_env.py) */
#define RL_JUMP_NOM_HEIGHT   0.085f   /* 观测高度基准, obs[18] = z - 0.085 */
#define RL_JUMP_TAU_L        150.0f   /* 腿长虚拟力矩量程 Nm(降低起跳力度) */
#define RL_JUMP_TAU_SW       60.0f    /* 腿摆虚拟力矩量程 Nm */
#define RL_JUMP_TAU_DIFF     60.0f    /* 左右腿长差虚拟力矩量程 Nm */
#define RL_JUMP_LEG_GAIN     50.0f    /* 腿执行器 gainprm(力矩->ctrl) */
#define RL_JUMP_WHEEL_CTRL   10.0f    /* 轮动作 -> ctrl(±10 -> ±100Nm) */
#define RL_JUMP_LP_ALPHA     0.5f     /* 动作低通: filt = 0.5*filt + 0.5*raw */

/* PREP 收腿到最短(用户指定: 触发后先收到最短腿再执行跳跃) */
#define RL_JUMP_PREP_EXT     0.25f    /* leg_extend_manual 极限收腿 */
#define RL_JUMP_PREP_TIMEOUT 3.0f     /* 收腿最长时长 s(实测 1.5s 收不到位) */
#define RL_JUMP_PREP_L_DONE  0.05f    /* 腿收到 L>=0.05(部署物理极限≈0.057) */

/* 退出判定阈值 */
#define RL_JUMP_TIMEOUT        3.0f    /* RL 全程窗口时长 s(与训练一致) */
#define RL_JUMP_RECOVER_HOLD   0.4f    /* 落地站稳需保持的时间 s */
#define RL_JUMP_RECOVER_H      0.03f   /* 高度增量低于此视为落地 */
#define RL_JUMP_RECOVER_PITCH  15.0f   /* 落地站稳 pitch 门限 deg */
#define RL_JUMP_RECOVER_ROLL   15.0f   /* 落地站稳 roll 门限 deg */
#define RL_JUMP_FALL_DEG       55.0f   /* 超过视为翻车, 快速退出交给自救 */
#define RL_JUMP_MIN_AIR_H      0.02f   /* 本轮必须离地超过该高度才算跳过 */

#define RL_JUMP_MIX_RATE       2.0f    /* FADEOUT 混合权重变化速度 1/s(过渡更平缓) */
#define RL_JUMP_ABORT_MIX_RATE 20.0f   /* 翻车时快速交还三环 1/s */

/* 电流观测归一化量程(与训练环境 OBS_CUR_SCALE 一致) */
#define RL_JUMP_CUR_LEG  250.0f
#define RL_JUMP_CUR_WHEEL 100.0f

/* ---- 控制器运行状态 ---- */
static int    g_ready = 0;
static int    g_trigger = 0;           /* 触发请求标志(按键回调里置位) */
static int    g_state = RL_JUMP_IDLE;
static float  g_t = 0.0f;              /* 当前状态累计时间 s */
static float  g_prep_t = 0.0f;         /* PREP 收腿累计时间 s */
static float  g_hold_t = 0.0f;         /* 落地恢复保持时间 s */
static float  g_filt[RL_JUMP_N_OUT] = {0};  /* 低通后的 RL 动作 */
static float  g_act[RL_JUMP_N_OUT] = {0};   /* 实际输出的 RL 动作 [-1,1] */
static float  g_obs_h = 0.0f;          /* 观测高度增量(日志) */
static float  g_max_h = 0.0f;          /* 本轮 ACTIVE 期间的最大高度增量 */
static float  g_mix = 1.0f;            /* 交叉渐变权重(0=纯RL, 1=纯三环) */
static int    g_prev_air = 0;          /* 上一帧离地标志(落地检测) */
static int    g_landing = 0;           /* 落地瞬间标志 */
static float  g_landing_t = 0.0f;      /* 落地后累计时间 */

static void rl_jump_controller_init_inner(void)
{
    g_ready = 1;
    g_trigger = 0;
    g_state = RL_JUMP_IDLE;
    g_t = 0.0f;
    g_prep_t = 0.0f;
    g_hold_t = 0.0f;
    for (int i = 0; i < RL_JUMP_N_OUT; i++) g_filt[i] = g_act[i] = 0.0f;
    g_obs_h = 0.0f;
    g_max_h = 0.0f;
    g_mix = 1.0f;
    g_prev_air = 0;
    g_landing = 0;
    g_landing_t = 0.0f;
}

void rl_jump_controller_init(void)
{
    rl_jump_controller_init_inner();
}

void rl_jump_controller_trigger(void)
{
    g_trigger = 1;
}

int rl_jump_controller_active(void)
{
    return (g_state != RL_JUMP_IDLE);
}

int rl_jump_controller_state(void)
{
    return g_state;
}

/*
 * 构建 34 维观测, 索引与 scripts/jump_rl_env.py::_get_obs 完全一致:
 *   0,1   : pitch, roll(rad)
 *   2,3   : 世界系角速度 pitch_rate, roll_rate(已核实 gyro == qvel[3:6])
 *   4..6  : 腿虚拟坐标: 腿摆 sw, 左腿长 L, 右腿长 R
 *           (4 个主关节角经线性运动学合成, 与训练环境/平衡控制器同一套映射)
 *   7..10 : 四腿关节速度 lf, lr, rf, rr
 *   11,12 : 左右轮速
 *   13    : 机体世界 z
 *   14,15 : |水平速度 vx|, |vy|
 *   16,17 : pitch, roll(deg)
 *   18    : z - 0.085(相对名义高度增量)
 *   19    : 时间戳(自本次 RL 接管起的相对时间 s)
 *   20..22: 世界线速度 vx, vy, vz
 *   23,24 : 左右轮位置
 *   25,26 : 机体世界位置 x, y
 *   27..32: 电流(电机侧力矩, 按量程归一化): lf, lr, lw, rf, rr, rw
 *   33    : 离地标志(0=接触地面, 1=空中), 分阶段控制的阶段信号
 */
static void build_obs(const SensorState *s, float obs[RL_JUMP_N_IN])
{
    float lf = (float)s->lf_pos, lr = (float)s->lr_pos;
    float rf = (float)s->rf_pos, rr = (float)s->rr_pos;
    /* 关节电机 -> 腿长/腿摆: 与平衡控制器 _leg_targets 同一套线性映射 */
    float sw = 0.25f * ((lf - lr + 0.4f) + (0.3f - rf + rr));
    float leg_l = 0.5f * (0.4f - lf - lr);
    float leg_r = 0.5f * (rf + rr + 0.5f);

    obs[0]  = (float)s->pitch;
    obs[1]  = (float)s->roll;
    obs[2]  = (float)s->gyro[1];
    obs[3]  = (float)s->gyro[0];
    obs[4]  = sw;
    obs[5]  = leg_l;
    obs[6]  = leg_r;
    obs[7]  = (float)s->lf_vel;
    obs[8]  = (float)s->lr_vel;
    obs[9]  = (float)s->rf_vel;
    obs[10] = (float)s->rr_vel;
    obs[11] = (float)s->lw_vel;
    obs[12] = (float)s->rw_vel;
    obs[13] = (float)s->body_pos[2];
    obs[14] = fabsf((float)s->body_vel[0]);
    obs[15] = fabsf((float)s->body_vel[1]);
    obs[16] = (float)(s->pitch * 57.29578);
    obs[17] = (float)(s->roll * 57.29578);
    obs[18] = (float)(s->body_pos[2] - RL_JUMP_NOM_HEIGHT);
    obs[19] = g_t;                       /* 时间戳: 自本次接管起 */
    obs[20] = (float)s->body_vel[0];
    obs[21] = (float)s->body_vel[1];
    obs[22] = (float)s->body_vel[2];
    obs[23] = (float)s->lw_pos;
    obs[24] = (float)s->rw_pos;
    obs[25] = (float)s->body_pos[0];
    obs[26] = (float)s->body_pos[1];
    /* 电流: 腿=s->lf_torque 已是电机侧; 轮=电机侧(s->lw_torque/减速比) */
    obs[27] = (float)s->lf_torque / RL_JUMP_CUR_LEG;
    obs[28] = (float)s->lr_torque / RL_JUMP_CUR_LEG;
    obs[29] = (float)(s->lw_torque / WHEEL_GEAR_RATIO) / RL_JUMP_CUR_WHEEL;
    obs[30] = (float)s->rf_torque / RL_JUMP_CUR_LEG;
    obs[31] = (float)s->rr_torque / RL_JUMP_CUR_LEG;
    obs[32] = (float)(s->rw_torque / WHEEL_GEAR_RATIO) / RL_JUMP_CUR_WHEEL;
#if RL_JUMP_N_IN > 33
    obs[33] = ((s->body_pos[2] - RL_JUMP_NOM_HEIGHT) > 0.02) ? 1.0f : 0.0f;
#endif
}

/* RL 虚拟坐标动作 -> ctrl: 腿用运动学 Jᵀ 映射到 4 个主关节,
 * 轮子直接力矩。与训练环境 _step_ctrl 完全一致。 */
static void apply_action(RlJumpCmd *out, const float act[RL_JUMP_N_OUT])
{
    float tau_l = act[0] * RL_JUMP_TAU_L;
    float tau_sw = act[1] * RL_JUMP_TAU_SW;
    float tau_diff = act[2] * RL_JUMP_TAU_DIFF;
    float tau_ll = tau_l + 0.5f * tau_diff;
    float tau_lr = tau_l - 0.5f * tau_diff;
    out->ctrl[0] = (tau_sw - tau_ll) / RL_JUMP_LEG_GAIN;   /* lf */
    out->ctrl[1] = (-tau_sw - tau_ll) / RL_JUMP_LEG_GAIN;  /* lr */
    out->ctrl[3] = (-tau_sw + tau_lr) / RL_JUMP_LEG_GAIN;  /* rf */
    out->ctrl[4] = (tau_sw + tau_lr) / RL_JUMP_LEG_GAIN;   /* rr */
    out->ctrl[2] = act[3] * RL_JUMP_WHEEL_CTRL;            /* 左轮 */
    out->ctrl[5] = act[4] * RL_JUMP_WHEEL_CTRL;            /* 右轮 */
}

static void set_debug_fields(SensorState *s)
{
    s->rl_state = g_state;
    s->rl_act0 = g_act[0];
    s->rl_act1 = g_act[1];
    /* 日志沿用旧列名: ext_delta=左轮动作, swing_delta=右轮动作 */
    s->rl_ext_delta = g_act[2];
    s->rl_swing_delta = g_act[5];
    s->rl_obs_h = g_obs_h;
}

void rl_jump_controller_update(SensorState *s, double dt, RlJumpCmd *out)
{
    if (!g_ready) rl_jump_controller_init_inner();

    float dtf = (float)dt;
    g_t += dtf;
    out->active = 0;
    for (int i = 0; i < 6; i++) out->mix[i] = 1.0f;
    for (int i = 0; i < 6; i++) out->ctrl[i] = 0.0f;

    switch (g_state) {
    case RL_JUMP_IDLE:
        for (int i = 0; i < RL_JUMP_N_OUT; i++) g_filt[i] = g_act[i] = 0.0f;
        if (g_trigger) {
            g_trigger = 0;
            g_state = RL_JUMP_PREP;
            g_t = 0.0f;
            g_prep_t = 0.0f;
            balance_set_leg_extend_manual(RL_JUMP_PREP_EXT);
            printf("[RLJump] t=%.3f prep(retract legs)\n", s->sim_time);
        }
        break;

    case RL_JUMP_PREP: {
        /* 收腿到最短: 通过平衡器 leg_extend_manual 极限收腿,
         * 轮子/俯仰仍由三环(base)稳住; 腿到位或超时后交给 RL。
         * 部署与训练一致: RL 统一从"最短腿"状态起跳。 */
        g_prep_t += dtf;
        float L = 0.5f * (0.4f - (float)s->lf_pos - (float)s->lr_pos);
        if (L >= RL_JUMP_PREP_L_DONE || g_prep_t >= RL_JUMP_PREP_TIMEOUT) {
            g_state = RL_JUMP_ACTIVE;
            g_t = 0.0f;
            g_prep_t = 0.0f;
            g_hold_t = 0.0f;
            g_obs_h = 0.0f;
            g_max_h = 0.0f;
            g_mix = 0.0f;
            balance_set_leg_extend_manual(0.0f);
            printf("[RLJump] t=%.3f start\n", s->sim_time);
        }
        break;
    }

    case RL_JUMP_ACTIVE: {
        float obs[RL_JUMP_N_IN];
        float raw[RL_JUMP_N_OUT];

        build_obs(s, obs);
        g_obs_h = obs[19];
        if (g_obs_h > g_max_h) g_max_h = g_obs_h;
        rl_jump_net_forward(obs, raw);
        if (g_t < 0.02f) {
            printf("[RLJump-obs]");
            for (int oi = 0; oi < RL_JUMP_N_IN; oi++)
                printf(" %+.4f", obs[oi]);
            printf("\n");
        }

        /* 与训练环境一致: 先裁剪 [-1,1], 再低通 0.7/0.3 */
        for (int i = 0; i < RL_JUMP_N_OUT; i++) {
            float a = raw[i];
            a = (a > 1.0f) ? 1.0f : ((a < -1.0f) ? -1.0f : a);
            g_filt[i] = (1.0f - RL_JUMP_LP_ALPHA) * g_filt[i]
                      + RL_JUMP_LP_ALPHA * a;
        }
        for (int i = 0; i < RL_JUMP_N_OUT; i++) g_act[i] = g_filt[i];
        /* 地面: 左右腿差力矩修正 roll(与训练环境一致)——
         * 哪边沉(roll>0 右侧沉), 右侧腿少发力/左侧多发力 -> τ_diff<0。
         * obs[17]=roll_deg, obs[3]=roll_rate。 */
        if (obs[33] < 0.5f) {
            float aux_diff = -1.5f * obs[17] - 0.3f * obs[3];
            /* 落地检测: 空中->地面, 落地后 0.3s 内 τ_diff 修正增强 */
            if (g_prev_air) {
                g_landing = 1;
                g_landing_t = 0.0f;
            }
            if (g_landing) {
                g_landing_t += dtf;
                if (g_landing_t > 0.3f) g_landing = 0;
                aux_diff = -4.0f * obs[17] - 0.3f * obs[3];
            }
            g_act[2] += aux_diff / RL_JUMP_TAU_DIFF;
            if (g_act[2] > 1.0f) g_act[2] = 1.0f;
            else if (g_act[2] < -1.0f) g_act[2] = -1.0f;
        }
        g_prev_air = (obs[33] > 0.5f) ? 1 : 0;
        out->active = 1;
        for (int i = 0; i < 6; i++) out->mix[i] = 0.0f;
        /* 分阶段混合(与训练环境 _step_ctrl 一致):
         *   地面接触: 轮子交三环稳住(防止起跳侧翻), RL 只管腿的虚拟坐标;
         *   空中    : 轮子交给 RL 调落地姿态。
         * obs[33] = 离地标志。 */
        if (obs[33] < 0.5f) {
            out->mix[2] = 1.0f;   /* 左轮 -> 三环 */
            out->mix[5] = 1.0f;   /* 右轮 -> 三环 */
        }
        apply_action(out, g_act);
        /* 空中: 轮子对姿态几乎无作用(角动量守恒), 固定为 0,
         * 姿态轨迹由起跳瞬间规划决定(与训练环境一致)。 */
        if (obs[33] > 0.5f) {
            out->ctrl[2] = 0.0f;
            out->ctrl[5] = 0.0f;
        }

        /* 退出判定 */
        float pitch_deg = fabsf((float)(s->pitch * 57.29578));
        float roll_deg = fabsf((float)(s->roll * 57.29578));
        if (pitch_deg > RL_JUMP_FALL_DEG || roll_deg > RL_JUMP_FALL_DEG) {
            g_state = RL_JUMP_FADEOUT;
            g_t = 0.0f;
            printf("[RLJump] t=%.3f abort: fall pitch=%.0f roll=%.0f\n",
                   s->sim_time, pitch_deg, roll_deg);
            break;
        }
        /* 只有"真的离过地"才进入落地恢复判定(防未跳即退出) */
        int recovered = (g_max_h > RL_JUMP_MIN_AIR_H &&
                         g_obs_h < RL_JUMP_RECOVER_H &&
                         pitch_deg < RL_JUMP_RECOVER_PITCH &&
                         roll_deg < RL_JUMP_RECOVER_ROLL &&
                         fabsf((float)s->gyro[1]) < 2.0f);
        if (recovered) {
            g_hold_t += dtf;
            if (g_hold_t >= RL_JUMP_RECOVER_HOLD) {
                g_state = RL_JUMP_FADEOUT;
                g_t = 0.0f;
                printf("[RLJump] t=%.3f recovered h=%.3f\n", s->sim_time, g_obs_h);
            }
        } else {
            g_hold_t = 0.0f;
        }
        if (g_t >= RL_JUMP_TIMEOUT) {
            g_state = RL_JUMP_FADEOUT;
            g_t = 0.0f;
            printf("[RLJump] t=%.3f timeout\n", s->sim_time);
        }
        break;
    }

    case RL_JUMP_FADEOUT: {
        /* 交叉渐变: RL 力矩保持, mix 0->1, 主循环 ctrl = mix*base + (1-mix)*rl。
         * 翻车时快速交还, 让 supervisor 自救尽快接管。 */
        float pitch_deg = fabsf((float)(s->pitch * 57.29578));
        float roll_deg = fabsf((float)(s->roll * 57.29578));
        float rate = (pitch_deg > RL_JUMP_FALL_DEG || roll_deg > RL_JUMP_FALL_DEG)
                     ? RL_JUMP_ABORT_MIX_RATE : RL_JUMP_MIX_RATE;
        g_mix += rate * dtf;
        if (g_mix > 1.0f) g_mix = 1.0f;
        for (int i = 0; i < 6; i++) out->mix[i] = g_mix;
        out->active = 1;
        apply_action(out, g_act);
        if (g_mix >= 1.0f) {
            g_state = RL_JUMP_IDLE;
            g_t = 0.0f;
            for (int i = 0; i < RL_JUMP_N_OUT; i++) g_filt[i] = g_act[i] = 0.0f;
            g_mix = 1.0f;
        }
        break;
    }
    }

    set_debug_fields(s);
}
