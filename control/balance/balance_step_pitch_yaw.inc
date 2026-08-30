/* balance_step_pitch_yaw.inc - 简化三环路径: 俯仰PD/偏航环/输出
 * 由 balance_core.c 的 balance_update() include 展开。 */

    float p_err_pb = pitch - g_balance.neutral;
    float kp_pb = SIMPLE_PITCH_KP;
    float kd_pb = SIMPLE_PITCH_KD;
#if SIMPLE_GAIN_SCHED_ENABLE
    {
        float e_abs_pb = fabsf(p_err_pb);
        float span_pb = SIMPLE_GAIN_FAR_RAD - SIMPLE_GAIN_NEAR_RAD;
        if (span_pb < 1e-4f) span_pb = 1e-4f;
        float blend_pb = (e_abs_pb - SIMPLE_GAIN_NEAR_RAD) / span_pb;
        blend_pb = clampf_local(blend_pb, 0.0f, 1.0f);
        kp_pb = SIMPLE_PITCH_KP_NEAR + blend_pb * (SIMPLE_PITCH_KP - SIMPLE_PITCH_KP_NEAR);
        kd_pb = SIMPLE_PITCH_KD_NEAR + blend_pb * (SIMPLE_PITCH_KD - SIMPLE_PITCH_KD_NEAR);
    }
#endif
    float tau_max_pb = (g_balance.run_time < 1.20f) ? SIMPLE_TAU_START : SIMPLE_TAU_RUN;
#if SIMPLE_BIG_RECOVERY
    float abs_p_pb = fabsf(pitch);
    if (g_balance.run_time > 1.20f && abs_p_pb > 0.14f) {
        kp_pb = 210.0f;
        kd_pb = 50.0f;
        tau_max_pb = 56.0f;
    }
    if (g_balance.run_time > 1.20f && abs_p_pb > 0.26f) {
        kp_pb = 285.0f;
        kd_pb = 62.0f;
        tau_max_pb = 76.0f;
    }
#endif
    float park_scale_pb = 1.0f - (fabsf(pitch) - SIMPLE_PARK_PITCH_RAD) / SIMPLE_PARK_PITCH_RAD;
    park_scale_pb = clampf_local(park_scale_pb, 0.0f, 1.0f);
    if (g_balance.run_time < 1.60f) park_scale_pb = 0.0f;
    float park_v_pb = clampf_local(g_balance.body_v_filt, -0.55f, 0.55f);
    float capture_scale_pb = (fabsf(g_balance.pos) < 0.25f && abs_pitch_pb < 0.10f && g_balance.run_time > 2.0f) ? 1.0f : 0.0f;
    float spin_v_pb = 0.0f;
    if (abs_pitch_pb < 0.35f && fabsf(g_balance.body_v_filt) > 1.20f) {
        spin_v_pb = g_balance.body_v_filt - ((g_balance.body_v_filt > 0.0f) ? 1.20f : -1.20f);
        spin_v_pb = clampf_local(spin_v_pb, -0.80f, 0.80f);
    }
    /* 俯仰平衡环保持最高频率，直接输出共模轮扭矩；位置控制只通过 g_balance.neutral 影响它。
     * 速度阻尼: 直接刹共模轮扭矩，不依赖目标俯仰角，所以 tp 收紧后仍能
     * 刹住起步漂移；手动驾驶有速度指令时放松，避免拖慢 W/S 加速。 */
    /* 速度阻尼按俯仰误差退让: 车体接近目标姿态(p_err≈0)时全力刹轮,
     * 俯仰环正在抬身(p_err 大)时让位, 避免 10Nm 抬身扭矩被
     * 2m/s 时的 10Nm 直刹完全抵消、车永远抬不起身(实测前冲翻车根因)。 */
    float vel_damp_scale_pb = 1.0f - clampf_local(fabsf(p_err_pb) / 0.06f,
                                                  0.0f, 1.0f);
    float vel_damp_pb = SIMPLE_VEL_DAMP * vel_damp_scale_pb;
    if (g_balance.manual_active && fabsf(g_balance.manual_v) >= 0.05f)
        /* 手动时保留 60% 速度阻尼: 腿摆管加减速, 轮子仍要能刹住
         * 启动/超速动量(0.3 太弱, 启动振荡 -2.4m/s 刹不住)。 */
        vel_damp_pb *= 0.60f;
    if (g_balance.manual_active && fabsf(g_balance.manual_yaw) >= 0.05f &&
        fabsf(g_balance.manual_v) < 0.05f)
        /* A/D 纯转向: body_v 是差速伪影(轮子差速被误读成前进),
         * 速度阻尼会输出共模刹停扭矩, 与转向竞争造成前进漂移
         * (实测 pitchMix -8Nm, bx 漂移 2m+)。转向时完全关闭速度阻尼。 */
        vel_damp_pb = 0.0f;
    float tau_cmd_pb = kp_pb * p_err_pb + kd_pb * g_balance.pitch_rate_filt
                     - vel_damp_pb * v_damp_pb
                     - park_scale_pb * SIMPLE_PARK_VEL_DAMP * park_v_pb
                     - capture_scale_pb * SIMPLE_CAPTURE_DAMP * park_v_pb
                     - SIMPLE_SPIN_DAMP * spin_v_pb;
    /* 空中弱化轮驱: 起跳后轮子离地, 平衡扭矩只会把轮子空转,
     * 落地瞬间带着高轮速产生前冲(实测跳跃落地 vh 冲到 +3.3)。
     * 空中把轮扭矩压到 20%, 落地后平衡恢复。 */
    if (airborne_pb)
        tau_cmd_pb *= 0.20f;
    /* 俯仰静差积分: 缓慢消除重轮下的残倾角(比例饱和导致轮子爬行/
     * 低频 rocking)。大倾角救车时清零避免积分与救车方向打架。 */
    if (abs_pitch_pb > 0.15f) {
        g_balance.pitch_i = 0.0f;
    } else {
        g_balance.pitch_i += pitch * dt_f;
        g_balance.pitch_i = clampf_local(g_balance.pitch_i, -0.30f, 0.30f);
    }
    tau_cmd_pb -= SIMPLE_PITCH_KI_NM * g_balance.pitch_i;
    if (g_balance.origin_creep > 0.5f) {
        /* 近场直接轮扭矩: 刹停项反对当前速度，爬行项在低速时把位置误差
         * 转成小幅轮扭矩，速度升高时自动收窄，避免把车越推越快。 */
        float creep_gate_pb = 1.0f - fabsf(g_balance.body_v_filt) / 0.10f;
        creep_gate_pb = clampf_local(creep_gate_pb, 0.0f, 1.0f);
        /* 用相对保持点的误差(不是绝对里程计): 启动救车后里程计与
         * 真实位置失同步，绝对 pos 会把车往错误方向推。 */
        float creep_tau_pb = -SIMPLE_ORIGIN_CREEP_TAU_KP *
                             pos_ctrl_pb * creep_gate_pb;
        creep_tau_pb = clampf_local(creep_tau_pb,
                                    -SIMPLE_ORIGIN_CREEP_TAU_MAX,
                                     SIMPLE_ORIGIN_CREEP_TAU_MAX);
        float brake_tau_pb = -SIMPLE_ORIGIN_STOP_TAU_KP *
                             g_balance.body_v_filt;
        brake_tau_pb = clampf_local(brake_tau_pb,
                                    -SIMPLE_ORIGIN_STOP_TAU_MAX,
                                     SIMPLE_ORIGIN_STOP_TAU_MAX);
        tau_cmd_pb += creep_tau_pb + brake_tau_pb;
    }
    int far_away_pb = (!g_balance.manual_active &&
                       fabsf(g_balance.pos) > SIMPLE_FAR_BRAKE_POS &&
                       (g_balance.pos * g_balance.body_v_filt) > 0.0f);
    int fast_spin_pb = (!g_balance.manual_active &&
                        fabsf(g_balance.body_v_filt) > SIMPLE_FAR_BRAKE_SPIN_V);
    if ((far_away_pb || fast_spin_pb) &&
        fabsf(g_balance.body_v_filt) > SIMPLE_FAR_BRAKE_V &&
        abs_pitch_pb < SIMPLE_FAR_BRAKE_PITCH_RAD &&
        !airborne_pb && !impact_pb) {
        float brake_pb = SIMPLE_FAR_BRAKE_KP * (fabsf(g_balance.body_v_filt) - SIMPLE_FAR_BRAKE_V);
        brake_pb = clampf_local(brake_pb, 0.0f, SIMPLE_FAR_BRAKE_MAX);
        tau_cmd_pb -= ((g_balance.body_v_filt > 0.0f) ? 1.0f : -1.0f) * brake_pb;
    }
#if SIMPLE_WHEEL_CAPTURE_ENABLE
    if (g_balance.wheel_capture_dir != 0.0f) {
        float cap_tau_pb = SIMPLE_WHEEL_CAPTURE_TAU_MIN +
            SIMPLE_WHEEL_CAPTURE_TAU_KV *
            (fabsf(g_balance.body_v_filt) - SIMPLE_WHEEL_CAPTURE_ENTER_V);
        cap_tau_pb = clampf_local(cap_tau_pb,
                                  SIMPLE_WHEEL_CAPTURE_TAU_MIN,
                                  SIMPLE_WHEEL_CAPTURE_TAU_MAX);
        if (tau_cmd_pb * g_balance.wheel_capture_dir > -cap_tau_pb) {
            tau_cmd_pb = -g_balance.wheel_capture_dir * cap_tau_pb;
        }
    }
#endif
#if SIMPLE_SLIP_BRAKE_ENABLE
    if (!airborne_pb && !impact_pb &&
        fabsf(g_balance.body_v_filt) > SIMPLE_SLIP_BRAKE_START &&
        abs_pitch_pb < SIMPLE_SLIP_BRAKE_PITCH_RAD) {
        float spin_dir_pb = (g_balance.body_v_filt > 0.0f) ? 1.0f : -1.0f;
        float power_tau_pb = tau_cmd_pb * spin_dir_pb;
        if (power_tau_pb > 0.0f) {
            float brake_tau_pb = SIMPLE_SLIP_BRAKE_KP *
                                 (fabsf(g_balance.body_v_filt) - SIMPLE_SLIP_BRAKE_START);
            brake_tau_pb = clampf_local(brake_tau_pb, 0.0f, SIMPLE_SLIP_BRAKE_MAX);
            tau_cmd_pb = -spin_dir_pb * brake_tau_pb;
        }
    }
#endif
#if SIMPLE_WHEEL_GOV_ENABLE
    if (abs_wheel_pb > SIMPLE_WHEEL_GOV_START) {
        float spin_dir_pb = (g_balance.wheel_v_filt > 0.0f) ? 1.0f : -1.0f;
        float spin_excess_pb = abs_wheel_pb - SIMPLE_WHEEL_GOV_START;
        float power_tau_pb = tau_cmd_pb * spin_dir_pb;
        if (power_tau_pb > 0.0f) {
            float allowed_tau_pb = SIMPLE_WHEEL_GOV_TAU - SIMPLE_WHEEL_GOV_KP * spin_excess_pb;
            if (allowed_tau_pb < SIMPLE_WHEEL_GOV_MIN_TAU) allowed_tau_pb = SIMPLE_WHEEL_GOV_MIN_TAU;
            if (allowed_tau_pb < power_tau_pb) tau_cmd_pb = spin_dir_pb * allowed_tau_pb;
        }
        if (abs_wheel_pb > SIMPLE_WHEEL_BRAKE_START && abs_pitch_pb < SIMPLE_WHEEL_BRAKE_PITCH_RAD) {
            float brake_tau_pb = SIMPLE_WHEEL_BRAKE_KP * (abs_wheel_pb - SIMPLE_WHEEL_BRAKE_START);
            brake_tau_pb = clampf_local(brake_tau_pb, 0.0f, SIMPLE_WHEEL_BRAKE_MAX);
            tau_cmd_pb -= spin_dir_pb * brake_tau_pb;
        }
    }
#endif
    tau_cmd_pb = clampf_local(tau_cmd_pb, -tau_max_pb, tau_max_pb);
    float tau_rate_pb = (g_balance.run_time < 1.20f) ? SIMPLE_RATE_START : SIMPLE_RATE_RUN;
#if SIMPLE_BIG_RECOVERY
    if (g_balance.run_time > 1.20f && abs_p_pb > 0.14f) tau_rate_pb = 700.0f;
    if (g_balance.run_time > 1.20f && abs_p_pb > 0.26f) tau_rate_pb = 950.0f;
#endif
#if SIMPLE_WHEEL_CAPTURE_ENABLE
    if (g_balance.wheel_capture_dir != 0.0f && tau_rate_pb < SIMPLE_WHEEL_CAPTURE_RATE) {
        tau_rate_pb = SIMPLE_WHEEL_CAPTURE_RATE;
    }
#endif
    float max_delta_pb = tau_rate_pb * dt_f;
    g_balance.tau_cmd += clampf_local(tau_cmd_pb - g_balance.tau_cmd, -max_delta_pb, max_delta_pb);
    *tau = g_balance.fallen ? 0.0f : g_balance.tau_cmd;

    float yaw_gate_pb = 0.0f;
    if (SIMPLE_YAW_MAX > 0.0f) {
        float gate_span_pb = SIMPLE_YAW_BAL_TAU_FULL - SIMPLE_YAW_BAL_TAU_MIN;
        if (gate_span_pb < 1.0f) gate_span_pb = 1.0f;
        /* 门控方向: 俯仰扭矩小(姿态平静)时偏航全权可用；
         * 只有大俯仰救车(扭矩顶到上限)时才把偏航让位给俯仰环。
         * 原来的公式在 |tau|<MIN 时把偏航清零，导致松手后 pitch 一
         * 平静 yaw 就没人管、车继续转。 */
        yaw_gate_pb = (SIMPLE_YAW_BAL_TAU_FULL - fabsf(g_balance.tau_cmd)) / gate_span_pb;
        yaw_gate_pb = clampf_local(yaw_gate_pb, 0.0f, 1.0f);
        if (airborne_pb || impact_pb) yaw_gate_pb = 0.0f;
    }
    if (g_balance.manual_active) yaw_gate_pb = 1.0f;
    float yaw_lim_pb = SIMPLE_YAW_MAX * yaw_gate_pb;
    if (g_balance.manual_active) {
        if (fabsf(g_balance.manual_yaw) < 0.05f) {
            /* 手动前进/后退(无 A/D 转向): yaw 位置纠偏用小限幅,
             * 只压住轮子差速造成的慢漂, 不抢前进共模扭矩。
             * 实测 32Nm 纠偏会与驱动竞争 -> 差速过冲振荡。 */
            yaw_lim_pb = SIMPLE_YAW_MAX;
        } else {
        /* 手动转向扭矩随前进速度衰减: 高速行驶中打方向更柔，防掀翻。 */
        float yaw_spd_scale = 1.0f - fabsf(g_balance.body_v_filt) /
                                      SIMPLE_MANUAL_YAW_SPEED_GATE;
        yaw_spd_scale = clampf_local(yaw_spd_scale, 0.25f, 1.0f);
        /* A/D 纯转向(无前后指令)时不做速度衰减: 自转时 body_v 会被差速
         * 误读成前进速度，衰减会掐掉转向扭矩导致共模前驱占主导。 */
        if (fabsf(g_balance.manual_v) < 0.05f)
            yaw_spd_scale = 1.0f;
        yaw_lim_pb = SIMPLE_MANUAL_YAW_MAX * yaw_gate_pb * yaw_spd_scale;
        }
    }

    g_balance.yaw_elapsed += dt_f;
    const float yaw_period_pb = 1.0f / (float)YAW_LOOP_HZ;
    if (g_balance.yaw_elapsed >= yaw_period_pb) {
        float yaw_dt_pb = g_balance.yaw_elapsed;
        g_balance.yaw_elapsed = 0.0f;
        if (yaw_gate_pb > 0.10f && !g_balance.manual_active) {
            g_balance.yaw_i += yaw_err_pb * yaw_dt_pb;
            g_balance.yaw_i = clampf_local(g_balance.yaw_i, -SIMPLE_YAW_I_MAX, SIMPLE_YAW_I_MAX);
        } else {
            g_balance.yaw_i *= 0.985f;
        }

        float yaw_rate_ref_pb;
        if (g_balance.manual_active && fabsf(g_balance.manual_yaw) >= 0.05f) {
            /* A/D 直接给偏航角速度(已斜坡化)，yaw 位置环让位。
             * v3 标定(2026-08-24): r/D(manual_yaw<0) 应右转(yaw 负),
             * 差速 -yaw 直接驱动, 不再取反(旧模型符号不同)。 */
            yaw_rate_ref_pb = -g_balance.manual_yaw;
        } else {
            /* 手动 W/S 前进(无 A/D 转向)或静止: 保持位置纠偏,
             * 防止轮子微小差速积累成 90°+ yaw 漂移(用户实测)。
             * A/D 转向时 manual_yaw 非零走上一分支。 */
            if (g_balance.manual_active)
                yaw_rate_ref_pb = SIMPLE_YAW_POS_KP * yaw_err_pb;
            else
                yaw_rate_ref_pb = SIMPLE_YAW_POS_KP * yaw_err_pb +
                                  SIMPLE_YAW_KI * g_balance.yaw_i;
        }
        /* 手动模式(含 T 键小陀螺 3.0rad/s)放开角速度参考限幅,
         * auto 仍用 1.6rad/s 的保守上限。
         * 手动前进(无转向)时用保守上限, 纠偏不过冲。 */
        float yaw_ref_max_pb = (g_balance.manual_active &&
                                fabsf(g_balance.manual_yaw) >= 0.05f)
                             ? SIMPLE_MANUAL_YAW_RATE_MAX
                             : SIMPLE_YAW_RATE_MAX;
        yaw_rate_ref_pb = clampf_local(yaw_rate_ref_pb,
                                       -yaw_ref_max_pb, yaw_ref_max_pb);
        /* yaw_rate 一阶低通: 差速陀螺高频振荡会让 yaw_held 在 ±限幅
         * 之间来回跳, 轮子交替反向 -> 车不转向只抖动(实测 yawMix
         * ±1.6Nm 振荡)。8ms 时间常数压掉高频, 保留转向响应。 */
        float yaw_rate_alpha = yaw_dt_pb / (0.008f + yaw_dt_pb);
        yaw_rate_alpha = clampf_local(yaw_rate_alpha, 0.0f, 1.0f);
        g_balance.yaw_rate_filt += yaw_rate_alpha * (yaw_rate - g_balance.yaw_rate_filt);
        /* 手动 A/D 转向: yaw_rate 误差环, 实际转速收敛到 manual_yaw。
         * v3 标定(2026-08-24): manual_yaw>0(左转A) 应 yaw 正,
         * 误差 = yaw_rate - manual_yaw, 用 +SIGN 方向修正。 */
        float yaw_rate_err_pb = (g_balance.manual_active &&
                                 fabsf(g_balance.manual_yaw) >= 0.05f)
                              ? g_balance.yaw_rate_filt - g_balance.manual_yaw
                              : g_balance.yaw_rate_filt + yaw_rate_ref_pb;
        float yaw_rate_kp_pb = g_balance.manual_active
                             ? SIMPLE_MANUAL_YAW_RATE_KP
                             : SIMPLE_YAW_RATE_KP;
        float yaw_ref_pb = SIMPLE_YAW_SIGN * yaw_rate_kp_pb * yaw_rate_err_pb;
        g_balance.yaw_held = clampf_local(yaw_ref_pb, -yaw_lim_pb, yaw_lim_pb);
    }
    {
        g_balance.yaw_held = clampf_local(g_balance.yaw_held, -yaw_lim_pb, yaw_lim_pb);
    }
    *yaw = g_balance.fallen ? 0.0f : g_balance.yaw_held;
    return;
