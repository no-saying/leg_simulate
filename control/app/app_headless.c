/*
 * app_headless.c - 应用层: 无界面测试模式
 * ================================================================
 * 从原 controller.c 拆出，用于自动化测试控制逻辑:
 *   - MANUAL_TEST: 分段手动驾驶脚本 f/b/l/r/u/d@截止时间
 *   - FALL_START : 初始横滚 90° 测试侧翻自救
 *   - ROLL_TILT_START / ROLL_TORQUE_AT / ROLL_PUSH_AT: roll 稳定验证钩子
 *   - SIDE_EXT_TEST: 差动腿伸长注入
 *   - JUMP_AT / SELF_RIGHT_AT: 定时触发跳跃/自救
 */
#include "app_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "balance.h"
#include "control_config.h"
#include "controllers/rl_jump/rl_jump_controller.h"
#include "dashcam.h"
#include "telemetry.h"

int run_headless(Simulation *sim, double duration)
{
    double last_log_time = 0.0;
    const double log_interval = 0.05;
    /* 无界面手动驾驶测试钩子: MANUAL_TEST=f/b/l/r 对应 W/S/A/D。
     * 支持分段脚本: 用逗号分隔，每段 "键@截止时间"，
     * 例如 f@4,b@8 = 前进到 4s、再后退到 8s。 */
    const char *mt = getenv("MANUAL_TEST");
    float manual_v = 0.0f, manual_yaw = 0.0f;
    int manual_test = 0;
    typedef struct { float v, yaw, ext; double until; } ManualSeg;
    ManualSeg mseg[8];
    int mseg_n = 0;
    if (mt) {
        char copy[128];
        snprintf(copy, sizeof(copy), "%s", mt);
        char *tok = strtok(copy, ",");
        while (tok && mseg_n < 8) {
            char kbuf[32];
            double until = -1.0;
            char *at = strchr(tok, '@');
            if (at) {
                *at = '\0';
                until = atof(at + 1);
            }
            snprintf(kbuf, sizeof(kbuf), "%s", tok);
            ManualSeg seg = {0, 0, 0, until};
            if (strstr(kbuf, "f")) seg.v += SIMPLE_MANUAL_V_MAX;
            if (strstr(kbuf, "b")) seg.v -= SIMPLE_MANUAL_V_MAX;
            if (strstr(kbuf, "l")) seg.yaw += SIMPLE_MANUAL_YAW_RATE;
            if (strstr(kbuf, "r")) seg.yaw -= SIMPLE_MANUAL_YAW_RATE;
            if (strstr(kbuf, "s")) seg.yaw += SIMPLE_MANUAL_SPIN_YAW_RATE;  /* s=小陀螺 */
            if (strstr(kbuf, "z")) seg.yaw -= SIMPLE_MANUAL_SPIN_YAW_RATE;  /* z=反向小陀螺 */
            if (strstr(kbuf, "u")) seg.ext -= MANUAL_LEG_EXTEND_MAX;  /* u=伸长/升高 */
            if (strstr(kbuf, "d")) seg.ext += MANUAL_LEG_EXTEND_MAX;  /* d=缩短/降低 */
            mseg[mseg_n++] = seg;
            tok = strtok(NULL, ",");
        }
        manual_test = 1;
        printf("[Manual] headless test segments=%d\n", mseg_n);
    }

    printf("COD-2026 headless | dt=%.4f nsensordata=%d duration=%.2f\n",
           sim->model->opt.timestep, (int)sim->model->nsensordata, duration);
    dashcam_open();

    /* 测试钩子: FALL_START=1 让车一开始横滚 90°(右侧着地)，验证侧翻自救。 */
    const char *fall_env = getenv("FALL_START");
    if (fall_env && atoi(fall_env) == 1) {
        sim->data->qpos[3] = 0.70710678f;
        sim->data->qpos[4] = 0.70710678f;
        sim->data->qpos[5] = 0.0f;
        sim->data->qpos[6] = 0.0f;
        sim->state.step_count = 0;
        mj_forward(sim->model, sim->data);
        printf("[Fall] started rolled 90deg right for self-right test\n");
    }
    /* PITCH_TILT_START=deg: 初始给一个 pitch 倾斜(度), 复现前翻(+)/后翻(-)。 */
    const char *pitch_tilt_env = getenv("PITCH_TILT_START");
    if (pitch_tilt_env) {
        float deg = (float)atof(pitch_tilt_env);
        float half = (float)(deg * 0.5 * 3.14159265358979 / 180.0);
        sim->data->qpos[3] = cosf(half);
        sim->data->qpos[4] = 0.0f;
        sim->data->qpos[5] = sinf(half);
        sim->data->qpos[6] = 0.0f;
        mj_forward(sim->model, sim->data);
        printf("[PitchTilt] started pitch=%.1f deg\n", deg);
    }
    /* ROLL_TILT_START=deg: 初始给一个 roll 倾斜(度), 验证 roll 稳定把车找平。 */
    const char *tilt_env = getenv("ROLL_TILT_START");
    if (tilt_env) {
        float deg = (float)atof(tilt_env);
        float half = (float)(deg * 0.5 * 3.14159265358979 / 180.0);
        sim->data->qpos[3] = cosf(half);
        sim->data->qpos[4] = sinf(half);
        sim->data->qpos[5] = 0.0f;
        sim->data->qpos[6] = 0.0f;
        mj_forward(sim->model, sim->data);
        printf("[RollTilt] started roll=%.1f deg for roll-stab test\n", deg);
    }

    float headless_leg_target = 0.0f;   /* headless 腿长目标：段结束后保持 */
    /* JUMP_AT 支持逗号分隔的多个触发时刻，例如 JUMP_AT=6,10,14 */
    const char *jump_at_env = getenv("JUMP_AT");
    double jump_at_t[8];
    int jump_at_n = 0;
    if (jump_at_env) {
        char copy[64];
        snprintf(copy, sizeof(copy), "%s", jump_at_env);
        char *tok = strtok(copy, ",");
        while (tok && jump_at_n < 8) {
            jump_at_t[jump_at_n++] = atof(tok);
            tok = strtok(NULL, ",");
        }
    }
    int jump_done[8] = {0};
    /* RL_JUMP_AT=t1,t2: 定时触发 RL 跳跃(逗号分隔多个时刻) */
    const char *rlj_env = getenv("RL_JUMP_AT");
    double rlj_at_t[8];
    int rlj_at_n = 0;
    if (rlj_env) {
        char copy[64];
        snprintf(copy, sizeof(copy), "%s", rlj_env);
        char *tok = strtok(copy, ",");
        while (tok && rlj_at_n < 8) {
            rlj_at_t[rlj_at_n++] = atof(tok);
            tok = strtok(NULL, ",");
        }
        printf("[RLJump] headless triggers at t=");
        for (int i = 0; i < rlj_at_n; i++) printf("%.2f ", rlj_at_t[i]);
        printf("\n");
    }
    int rlj_done[8] = {0};
    /* RL_JUMP_PERIOD=s: 每隔 s 秒周期性触发 RL 跳跃(持续测试用) */
    const char *rlj_period_env = getenv("RL_JUMP_PERIOD");
    double rlj_period = rlj_period_env ? atof(rlj_period_env) : 0.0;
    double rlj_next = (rlj_period > 0.0) ? rlj_period : 1e18;
    if (rlj_period > 0.0)
        printf("[RLJump] periodic trigger every %.2fs\n", rlj_period);
    const char *sr_env = getenv("SELF_RIGHT_AT");
    double sr_at = sr_env ? atof(sr_env) : -1.0;
    int sr_done = 0;
    /* SIDE_EXT_TEST=±value: 试验钩子，恒定注入左右差动腿伸长，观察 roll 响应。 */
    const char *side_test_env = getenv("SIDE_EXT_TEST");
    float side_test = side_test_env ? (float)atof(side_test_env) : 0.0f;
    if (side_test != 0.0f)
        printf("[SideExt] test hook side_extend=%.3f\n", side_test);
    /* ROLL_PUSH_AT=t,y_frc: t 时刻给 base 施加 y 向力(单位 N), 测试 roll 稳定。
     * 例: ROLL_PUSH_AT=3,40 -> 3s 时向右推 40N 持续。 */
    const char *push_env = getenv("ROLL_PUSH_AT");
    double push_t = -1.0; float push_f = 0.0f;
    if (push_env) {
        sscanf(push_env, "%lf,%f", &push_t, &push_f);
        printf("[RollPush] test hook t=%.1f force=%.1fN\n", push_t, push_f);
    }
    /* ROLL_TORQUE_AT=t,tau: t 时刻给 base 施加恒定 roll 力矩(绕 x, Nm),
     * 制造持续的 roll 偏置, 验证“哪边沉抬哪边”的差动腿策略。 */
    const char *rtq_env = getenv("ROLL_TORQUE_AT");
    double rtq_t = -1.0; float rtq_tau = 0.0f;
    if (rtq_env) {
        sscanf(rtq_env, "%lf,%f", &rtq_t, &rtq_tau);
        printf("[RollTorque] test hook t=%.1f tau=%.2fNm\n", rtq_t, rtq_tau);
    }
    while (sim->state.sim_time < duration) {
        maybe_auto_enable_balance(sim);
        if (side_test != 0.0f)
            balance_set_leg_side_extend(side_test);
        if (push_t >= 0.0 && sim->state.sim_time >= push_t) {
            int bid = mj_name2id(sim->model, mjOBJ_BODY, "base");
            if (bid >= 0) sim->data->xfrc_applied[6*bid + 1] = push_f;
        }
        if (rtq_t >= 0.0 && sim->state.sim_time >= rtq_t) {
            int bid = mj_name2id(sim->model, mjOBJ_BODY, "base");
            if (bid >= 0) sim->data->xfrc_applied[6*bid + 3] = rtq_tau;
        }
        if (sr_at >= 0.0 && !sr_done && sim->state.sim_time >= sr_at) {
            balance_trigger_self_right();
            sr_done = 1;
            printf("[SelfRight] triggered at t=%.2f\n", sim->state.sim_time);
        }
        for (int ji = 0; ji < jump_at_n; ji++) {
            if (!jump_done[ji] && sim->state.sim_time >= jump_at_t[ji]) {
                balance_trigger_jump();
                jump_done[ji] = 1;
                printf("[Jump] triggered at t=%.2f\n", sim->state.sim_time);
            }
        }
        for (int ji = 0; ji < rlj_at_n; ji++) {
            if (!rlj_done[ji] && sim->state.sim_time >= rlj_at_t[ji]) {
                rl_jump_controller_trigger();
                rlj_done[ji] = 1;
                printf("[RLJump] triggered at t=%.2f\n", sim->state.sim_time);
            }
        }
        if (rlj_period > 0.0 && sim->state.sim_time >= rlj_next) {
            rl_jump_controller_trigger();
            printf("[RLJump] periodic trigger at t=%.2f\n", sim->state.sim_time);
            rlj_next += rlj_period;
        }
        if (manual_test) {
            int active = 0;
            int matched = 0;
            float ext_target = 0.0f;
            for (int i = 0; i < mseg_n; i++) {
                if (sim->state.sim_time < mseg[i].until || mseg[i].until < 0.0) {
                    manual_v = mseg[i].v;
                    manual_yaw = mseg[i].yaw;
                    ext_target = mseg[i].ext;
                    matched = 1;
                    /* 只有带 f/b/l/r 的段才进入手动驾驶模式；纯 u/d 腿长段
                     * 保持普通待机，和 GUI 里只按 E/Q 的行为一致。 */
                    active = (mseg[i].v != 0.0f || mseg[i].yaw != 0.0f);
                    break;
                }
            }
            balance_set_manual(active ? manual_v : 0.0f,
                               active ? manual_yaw : 0.0f, active);
            if (matched) headless_leg_target = ext_target;   /* 段内设定，段外保持 */
            balance_set_leg_extend_manual(headless_leg_target);
        }
        if (!simulation_step(sim, 0))
            return 1;
        g_dash_sim_t = sim->state.sim_time;
        dashcam_log_row(&sim->state, sim->state.sim_time,
                        manual_keys_state(), balance_get_manual_active(),
                        balance_get_hold_pos());

        if (sim->state.sim_time - last_log_time >= log_interval) {
            last_log_time = sim->state.sim_time;
            telemetry_print_frame(&sim->state, 0.0);
        }
    }
    dashcam_close();
    return 0;
}
