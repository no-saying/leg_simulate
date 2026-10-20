/* balance_step_pitch_yaw.inc - 简化三环路径: 俯仰PD/偏航环/输出
 * 由 balance_core.c 的 balance_update() include 展开。 */

    float p_err_pb = pitch - g_balance.neutral;
    float kp_pb = SIMPLE_PITCH_KP;
    float kd_pb = SIMPLE_PITCH_KD;
    int manual_pitch_pb = (g_balance.manual_active || g_balance.post_manual) &&
                          g_balance.run_time > 1.0f;
    /* 手动驾驶用更软的俯仰增益: 自动/启动保留强增益(-40)救瞬态,
     * 手动时轮上摩擦只支持 ~0.5Nm/轮, 强增益的扭矩需求会烧胎空转
     * 反推车体滑动(漂移根因)。软增益让轮子咬地纯滚动。 */
    if (manual_pitch_pb) {
        /* 手动(含 W/S/A/D)统一用软增益: 强增益在摆腿换向/转向时
         * 扭矩需求超摩擦, 烧胎反推车体漂移。软增益让轮子咬地。 */
        kp_pb = SIMPLE_MANUAL_PITCH_KP;
        kd_pb = SIMPLE_MANUAL_PITCH_KD;
    }
    int short_leg_pb = (g_balance.leg_len_dbg < SHORT_LEG_MAX);
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
    if (!manual_pitch_pb) {
        float long_span_pb = SIMPLE_LONG_PITCH_KD_FULL - SIMPLE_LONG_PITCH_KD_START;
        if (long_span_pb < 1e-4f) long_span_pb = 1e-4f;
        float long_blend_pb = clampf_local(
            (g_balance.leg_len_dbg - SIMPLE_LONG_PITCH_KD_START) /
            long_span_pb,
            0.0f, 1.0f);
        kd_pb += long_blend_pb * (SIMPLE_LONG_PITCH_KD - SIMPLE_PITCH_KD);
    }
    if (short_leg_pb) {
        kp_pb *= SHORT_LEG_BALANCE_GAIN_SCALE;
        kd_pb *= SHORT_LEG_BALANCE_GAIN_SCALE;
    }
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
    /* 手动(含转向/行驶): park 停车阻尼必须让位。
     * 它和手动腿摆位置环作用在同一个共模轮扭矩上, 实测按住 A 时两者
     * 对打, 偏航率在 ±0.5rad/s 之间抖(手感"一卡一卡"), pitch 也被
     * 带着晃 2~3°; 关掉后离起点 0.06m、|pitch| 0.2°。
     * 自动模式的停车保持仍然保留。 */
    if (g_balance.manual_active || g_balance.post_manual)
        park_scale_pb = 0.0f;
    float park_v_pb = clampf_local(control_v_pb, -0.55f, 0.55f);
    float capture_scale_pb = (fabsf(g_balance.pos) < 0.25f && abs_pitch_pb < 0.10f && g_balance.run_time > 2.0f) ? 1.0f : 0.0f;
    float spin_v_pb = 0.0f;
    if (abs_pitch_pb < 0.35f && fabsf(control_v_pb) > 1.20f) {
        spin_v_pb = control_v_pb - ((control_v_pb > 0.0f) ? 1.20f : -1.20f);
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
    if (short_leg_pb)
        vel_damp_pb = SHORT_LEG_VEL_BRAKE * vel_damp_scale_pb;
    /* 手动 W/S: 摆腿是速度执行器 -> 关掉轮速阻尼(否则与摆腿打架/烧胎)。
     * 判据必须是"腿摆速度环真的在驱动", 而不是"腿长落在某个窗口内":
     * 实测(2026-09-26)腿摆在 0.072~0.157m 全程都有权限(GUI/headless
     * 全腿长扫 W/S/A/D/Z 都能正常驱动), 原来的 0.095/0.140 窗口是
     * 误判 —— 窗口外腿摆被强制清零、只留轮速阻尼, 而轮速阻尼
     * -K·v 在这个位形上是正反馈, 于是"按 W 反向疯狂加速、压根没刹车"。
     * 现在窗口与 origin 步的 leg_win_ok 一致, 窗口内=摆腿驱动=阻尼让位。 */
    extern float balance_leg_drive_min(void);
    extern float balance_leg_drive_max(void);
    float swauth_min_pb = balance_leg_drive_min();
    float swauth_max_pb = balance_leg_drive_max();
    int swing_has_authority = (g_balance.leg_len_dbg >= swauth_min_pb &&
                               g_balance.leg_len_dbg <= swauth_max_pb);
    if (g_balance.manual_active && fabsf(g_balance.manual_v) >= 0.05f &&
        swing_has_authority)
        vel_damp_pb = 0.0f;
    if ((g_balance.manual_active && fabsf(g_balance.manual_yaw) >= 0.05f &&
         fabsf(g_balance.manual_v) < 0.05f) ||
        (g_balance.post_manual && g_balance.post_spin))
        /* A/D 纯转向: 保留速度阻尼而不是关掉。
         * 原注释认为 body_v 是"差速伪影", 但实测转向时 body_v 与里程计
         * 位移一致(都是真实滑动, 可达 1.2m/s), 而且 roll 稳定修好后
         * 转向不再需要靠"抽腿"来掩盖漂移。转向是前后扰动最强的工况
         * (yaw 反作用经闭链腿传到前后), 关掉阻尼后车会滑出 2m+。
         * 数值取 SIMPLE_MANUAL_TURN_VEL_DAMP。 */
        vel_damp_pb = SIMPLE_MANUAL_TURN_VEL_DAMP;
    /* v3 正共模轮扭矩使机体向正 x 运动；速度阻尼采用 -K*v。 */
    float vel_brake_tau_pb = -vel_damp_pb * v_damp_pb;
    if (g_balance.manual_active &&
        g_balance.leg_len_dbg >= SIMPLE_LONG_MANUAL_BRAKE_START &&
        fabsf(g_balance.manual_v) >= 0.05f &&
        control_v_pb * g_balance.manual_v > 0.0f) {
        float overspeed_pb = fabsf(control_v_pb) - fabsf(g_balance.manual_v);
        if (overspeed_pb > 0.0f) {
            float brake_pb = clampf_local(SIMPLE_LONG_MANUAL_BRAKE_KP * overspeed_pb,
                                          0.0f, SIMPLE_LONG_MANUAL_BRAKE_MAX);
            vel_brake_tau_pb -= copysignf(brake_pb, control_v_pb);
        }
    }
    if (g_balance.post_manual &&
        g_balance.leg_len_dbg >= SIMPLE_LONG_MANUAL_BRAKE_START) {
        float brake_pb = clampf_local(SIMPLE_LONG_POST_BRAKE_KP *
                                      fabsf(control_v_pb),
                                      0.0f, SIMPLE_LONG_POST_BRAKE_MAX);
        vel_brake_tau_pb -= copysignf(brake_pb, control_v_pb);
    }
    float tau_cmd_pb = kp_pb * p_err_pb + kd_pb * g_balance.pitch_rate_filt
                     + vel_brake_tau_pb
                     - park_scale_pb * SIMPLE_PARK_VEL_DAMP * park_v_pb
                     - capture_scale_pb * SIMPLE_CAPTURE_DAMP * park_v_pb
                     - SIMPLE_SPIN_DAMP * spin_v_pb;
    {
        /* 临时调试(env TDBG=1): 打印俯仰环各分项, 用于定位"短腿按 W 没刹车"。
         * 只读, 不影响控制。 */
        static int tdbg = -1;
        static int tdbg_cnt = 0;
        if (tdbg < 0) tdbg = getenv("TDBG") ? atoi(getenv("TDBG")) : 0;
        if (tdbg && (++tdbg_cnt % 20) == 0) {
            printf("[TDBG] t=%.2f man=%d post=%d L=%.4f neutral=%+.4f "
                   "pitch=%+.4f p_err=%+.4f kp=%+.1f kd=%+.1f prate=%+.3f "
                   "vdamp=%+.3f swauth=%d | terms p=%.3f d=%.3f vd=%.3f "
                   "park=%.3f cap=%.3f spin=%.3f | m_v=%+.2f b_v=%+.3f "
                   "tau_raw=%.3f\n",
                   g_balance.run_time, g_balance.manual_active,
                   g_balance.post_manual, g_balance.leg_len_dbg,
                   g_balance.neutral, pitch, p_err_pb, kp_pb, kd_pb,
                   g_balance.pitch_rate_filt, vel_damp_pb, swing_has_authority,
                   kp_pb * p_err_pb, kd_pb * g_balance.pitch_rate_filt,
                   vel_brake_tau_pb,
                   -park_scale_pb * SIMPLE_PARK_VEL_DAMP * park_v_pb,
                   -capture_scale_pb * SIMPLE_CAPTURE_DAMP * park_v_pb,
                   -SIMPLE_SPIN_DAMP * spin_v_pb,
                   g_balance.manual_v, control_v_pb, tau_cmd_pb);
        }
    }
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
        float creep_gate_pb = 1.0f - fabsf(control_v_pb) / 0.10f;
        creep_gate_pb = clampf_local(creep_gate_pb, 0.0f, 1.0f);
        /* 用相对保持点的误差(不是绝对里程计): 启动救车后里程计与
         * 真实位置失同步，绝对 pos 会把车往错误方向推。 */
        float creep_tau_pb = -SIMPLE_ORIGIN_CREEP_TAU_KP *
                             pos_ctrl_pb * creep_gate_pb;
        creep_tau_pb = clampf_local(creep_tau_pb,
                                    -SIMPLE_ORIGIN_CREEP_TAU_MAX,
                                     SIMPLE_ORIGIN_CREEP_TAU_MAX);
        float brake_tau_pb = -SIMPLE_ORIGIN_STOP_TAU_KP *
                             control_v_pb;
        brake_tau_pb = clampf_local(brake_tau_pb,
                                    -SIMPLE_ORIGIN_STOP_TAU_MAX,
                                     SIMPLE_ORIGIN_STOP_TAU_MAX);
        tau_cmd_pb += creep_tau_pb + brake_tau_pb;
    }
    int far_away_pb = (!g_balance.manual_active &&
                       fabsf(g_balance.pos) > SIMPLE_FAR_BRAKE_POS &&
                       (g_balance.pos * control_v_pb) > 0.0f);
    int fast_spin_pb = (!g_balance.manual_active &&
                        fabsf(control_v_pb) > SIMPLE_FAR_BRAKE_SPIN_V);
    if ((far_away_pb || fast_spin_pb) &&
        fabsf(control_v_pb) > SIMPLE_FAR_BRAKE_V &&
        abs_pitch_pb < SIMPLE_FAR_BRAKE_PITCH_RAD &&
        !airborne_pb && !impact_pb) {
        float brake_pb = SIMPLE_FAR_BRAKE_KP * (fabsf(control_v_pb) - SIMPLE_FAR_BRAKE_V);
        brake_pb = clampf_local(brake_pb, 0.0f, SIMPLE_FAR_BRAKE_MAX);
        tau_cmd_pb -= ((control_v_pb > 0.0f) ? 1.0f : -1.0f) * brake_pb;
    }
    /* A/D 纯转向的"原地保持": v3 闭链腿几何在 yaw 时会把轮轴反作用
     * 耦合进前向位移, 实测转 90° 漂 4m+。轮子差速扭矩刹不住这种
     * 滑动, 这里用里程计+IMU 融合速度测真实前滑, 直接给共模轮扭矩
     * 把车钉在进入点原地转(限幅小于摩擦极限, 不会自己打滑)。 */
    if (g_balance.manual_active &&
        fabsf(g_balance.manual_yaw) >= 0.05f &&
        fabsf(g_balance.manual_v) < 0.05f) {
        float drift_pos_pb = g_balance.pos - g_balance.hold_pos;
        float drift_v_pb = g_balance.simple_imu_v;
        float drift_tau_pb = -TURN_DRIFT_POS_KP * drift_pos_pb
                             - TURN_DRIFT_V_KP * drift_v_pb;
        drift_tau_pb = clampf_local(drift_tau_pb,
                                    -TURN_DRIFT_TAU_MAX,
                                     TURN_DRIFT_TAU_MAX);
        tau_cmd_pb += drift_tau_pb;
    }
#if SIMPLE_WHEEL_CAPTURE_ENABLE
    if (g_balance.wheel_capture_dir != 0.0f) {
        float cap_tau_pb = SIMPLE_WHEEL_CAPTURE_TAU_MIN +
            SIMPLE_WHEEL_CAPTURE_TAU_KV *
            (fabsf(control_v_pb) - SIMPLE_WHEEL_CAPTURE_ENTER_V);
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
        fabsf(control_v_pb) > SIMPLE_SLIP_BRAKE_START &&
        abs_pitch_pb < SIMPLE_SLIP_BRAKE_PITCH_RAD) {
        float spin_dir_pb = (control_v_pb > 0.0f) ? 1.0f : -1.0f;
        float power_tau_pb = tau_cmd_pb * spin_dir_pb;
        if (power_tau_pb > 0.0f) {
            float brake_tau_pb = SIMPLE_SLIP_BRAKE_KP *
                                 (fabsf(control_v_pb) - SIMPLE_SLIP_BRAKE_START);
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
    /* 手动驾驶共模扭矩单独压小: 自动/启动保留 8Nm 救住瞬态, 手动时
     * 轮上摩擦极限 ~0.5Nm/轮, 大扭矩只会烧胎空转反推车体滑动
     * (实测 -66rad/s 空转、车体前滑 2.5m/s -> 漂移)。手动把共模
     * 压到摩擦范围内, 车真正滚动。 */
    if ((g_balance.manual_active || g_balance.post_manual) &&
        g_balance.run_time > 1.0f) {
        float manual_tau_max_pb = SIMPLE_MANUAL_PITCH_TAU_MAX;
        if (g_balance.manual_active &&
            fabsf(g_balance.manual_yaw) >= 0.05f &&
            fabsf(g_balance.manual_v) < 0.05f) {
            float short_turn_blend_pb = clampf_local(
                (0.118f - g_balance.leg_len_dbg) / (0.118f - SHORT_LEG_MAX),
                0.0f, 1.0f);
            float turn_tau_max_pb = SIMPLE_TURN_PITCH_TAU_MAX +
                short_turn_blend_pb * (SIMPLE_SHORT_TURN_PITCH_TAU_MAX -
                                       SIMPLE_TURN_PITCH_TAU_MAX);
            float recovery_pb = clampf_local(
                (fabsf(pitch) - SIMPLE_TURN_PITCH_RECOVERY_START) /
                (SIMPLE_TURN_PITCH_RECOVERY_FULL - SIMPLE_TURN_PITCH_RECOVERY_START),
                0.0f, 1.0f);
            manual_tau_max_pb = turn_tau_max_pb + recovery_pb *
                (SIMPLE_MANUAL_PITCH_TAU_MAX - turn_tau_max_pb);
        } else if (g_balance.post_manual && g_balance.post_spin) {
            float long_turn_blend_pb = clampf_local(
                (g_balance.leg_len_dbg - 0.140f) / 0.005f, 0.0f, 1.0f);
            /* Stopping must retain the normal pitch recovery authority. */
            manual_tau_max_pb += long_turn_blend_pb * fmaxf(0.0f,
                SIMPLE_LONG_TURN_PITCH_TAU_MAX - SIMPLE_MANUAL_PITCH_TAU_MAX);
        }
        tau_cmd_pb = clampf_local(tau_cmd_pb,
                                  -manual_tau_max_pb, manual_tau_max_pb);
    }
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

    /* 超出已验证的机械腿长范围时拒绝转向。 */
    /* 调试: TURNLEG_MIN/MAX 覆盖转向/陀螺腿长窗口(默认 SIMPLE_TURN_LEG_*)。 */
    static float turnleg_min_cfg = -1.0f, turnleg_max_cfg = -1.0f;
    if (turnleg_min_cfg < 0.0f) {
        const char *e = getenv("TURNLEG_MIN");
        turnleg_min_cfg = e ? (float)atof(e) : SIMPLE_TURN_LEG_MIN;
    }
    if (turnleg_max_cfg < 0.0f) {
        const char *e = getenv("TURNLEG_MAX");
        turnleg_max_cfg = e ? (float)atof(e) : SIMPLE_TURN_LEG_MAX;
    }
    int leg_win_turn = (g_balance.leg_len_dbg >= turnleg_min_cfg &&
                        g_balance.leg_len_dbg <= turnleg_max_cfg);
#if MODEL_V3
    if (g_balance.turn_full_range) {
        leg_win_turn = g_balance.leg_len_dbg >= LEG_V3_L_MID-CTRL_LQR_FULL_EXT_MAX-0.008f &&
                       g_balance.leg_len_dbg <= LEG_V3_L_MID-CTRL_LQR_FULL_EXT_MIN+0.008f;
    }
#endif
    if (!leg_win_turn) {
        if (fabsf(g_balance.manual_yaw) >= 0.05f ||
            fabsf(g_balance.manual_yaw_target) >= 0.05f) {
            static int warned = 0;
            if (!warned) {
                warned = 1;
                printf("[TurnBlock] leg_len=%.3fm 超出转向窗口[%.3f,%.3f], "
                       "已拒绝转向/陀螺\n", (double)g_balance.leg_len_dbg,
                       (double)turnleg_min_cfg, (double)turnleg_max_cfg);
            }
        }
        g_balance.manual_yaw = 0.0f;
        g_balance.manual_yaw_target = 0.0f;
    } else {
        /* 回到窗口内允许再次提示 */
        { }
    }

    float long_yaw_blend_pb = clampf_local(
        (g_balance.leg_len_dbg - SIMPLE_LONG_YAW_BLEND_START) /
        (SIMPLE_LONG_YAW_BLEND_FULL - SIMPLE_LONG_YAW_BLEND_START), 0.0f, 1.0f);
    float yaw_hold_max_pb = SIMPLE_YAW_MAX + long_yaw_blend_pb *
                            (SIMPLE_LONG_YAW_MAX - SIMPLE_YAW_MAX);
    float yaw_pos_kp_pb = SIMPLE_YAW_POS_KP + long_yaw_blend_pb *
                          (SIMPLE_LONG_YAW_POS_KP - SIMPLE_YAW_POS_KP);
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
    float yaw_lim_pb = yaw_hold_max_pb * yaw_gate_pb;
    if (g_balance.manual_active) {
        if (fabsf(g_balance.manual_yaw) < 0.05f) {
            /* 手动前进/后退(无 A/D 转向): yaw 位置纠偏用小限幅,
             * 只压住轮子差速造成的慢漂, 不抢前进共模扭矩。
             * 实测 32Nm 纠偏会与驱动竞争 -> 差速过冲振荡。 */
            yaw_lim_pb = yaw_hold_max_pb;
        } else {
        /* 手动转向扭矩随前进速度衰减: 高速行驶中打方向更柔，防掀翻。 */
        float yaw_spd_scale = 1.0f - fabsf(control_v_pb) /
                                      SIMPLE_MANUAL_YAW_SPEED_GATE;
        yaw_spd_scale = clampf_local(yaw_spd_scale, 0.25f, 1.0f);
        /* A/D 纯转向(无前后指令)时不做速度衰减: 自转时 body_v 会被差速
         * 误读成前进速度，衰减会掐掉转向扭矩导致共模前驱占主导。 */
        if (fabsf(g_balance.manual_v) < 0.05f)
            yaw_spd_scale = 1.0f;
        float yaw_max_pb = g_balance.manual_spin_active
                         ? SIMPLE_MANUAL_SPIN_YAW_MAX : SIMPLE_MANUAL_YAW_MAX;
        yaw_lim_pb = yaw_max_pb * yaw_gate_pb * yaw_spd_scale;
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
                yaw_rate_ref_pb = yaw_pos_kp_pb * yaw_err_pb;
            else
                yaw_rate_ref_pb = yaw_pos_kp_pb * yaw_err_pb +
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
        /* 偏航角速度滤波可配置；v3 默认零延迟以保留阻尼相位。 */
        float yaw_rate_alpha = yaw_dt_pb / (SIMPLE_YAW_RATE_TAU + yaw_dt_pb);
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
        if (g_balance.manual_active && fabsf(g_balance.manual_yaw) >= 0.05f)
            yaw_ref_pb += WHEEL_YAW_DRAG_GAIN * g_balance.manual_yaw / WHEEL_MIX_YAW_WEIGHT;
        g_balance.yaw_held = clampf_local(yaw_ref_pb, -yaw_lim_pb, yaw_lim_pb);
    }
    {
        g_balance.yaw_held = clampf_local(g_balance.yaw_held, -yaw_lim_pb, yaw_lim_pb);
        /* 偏航力矩输出斜率限制。
         * 偏航环 20ms 一次更新, 输出可以在 +MAX 与 -MAX 之间瞬跳; 轮子
         * 惯量极小(0.4kg/半径 0.043), 力矩跳变会把轮速砸到 ±20rad/s,
         * 手感就是"一卡一卡"(实测转向时轮速 50ms 内从 -14 跳到 +22rad/s)。
         * 限住力矩变化率后轮速轨迹平滑, 转向变成连续转动。 */
        if (SIMPLE_YAW_OUT_SLEW > 0.0f) {
            float dyaw_slew_pb = SIMPLE_YAW_OUT_SLEW * dt_f;
            g_balance.yaw_out += clampf_local(g_balance.yaw_held - g_balance.yaw_out,
                                              -dyaw_slew_pb, dyaw_slew_pb);
        } else {
            g_balance.yaw_out = g_balance.yaw_held;   /* 0=不限斜率 */
        }
    }
    *yaw = g_balance.fallen ? 0.0f : g_balance.yaw_out;

    /* ---- 原地转向: 双腿前后叉开(左右腿摆差动) ----
     * 左腿摆参考 = swing_ref + diff, 右腿 = swing_ref - diff。差动把轮轴线
     * 在水平面内"拧"一个角度, 直接产生偏航力矩(闭链腿的几何耦合), 不需要
     * 轮胎先滑起来, 也不会像轮子差速那样带出前后推力 —— 用户要求的
     * "腿叉开转"。轮子差速环(yaw_held)保留做精细修正。
     * 符号: 正 diff = 左轮相对右轮前移 = 绕 +z 偏航(正向), 对应 A 的正
     * 偏航指令; 若实测反向, 改 SIMPLE_MANUAL_YAW_LEG_SIGN。 */
    {
        float diff_ref_pb = 0.0f;
        if (g_balance.manual_active && fabsf(g_balance.manual_yaw) >= 0.05f &&
            !airborne_pb && !impact_pb && g_balance.run_time > 1.0f) {
            float yaw_rate_err_leg = g_balance.yaw_rate_filt - g_balance.manual_yaw;
            /* 差动幅值按腿长缩放: 叉腿的几何效果取决于"腿张角"= d/L,
             * 腿变长时同样的 d 张角变小、对偏航的作用变弱。按当前腿长
             * 相对标定腿长缩放, 保持同一张角(用户要求"差动腿摆角度应该
             * 和腿长有关")。 */
            float leg_scale_pb = g_balance.leg_len_dbg / SIMPLE_MANUAL_LEG_LEN_REF;
            leg_scale_pb = clampf_local(leg_scale_pb,
                                        SIMPLE_MANUAL_YAW_LEG_SCALE_MIN,
                                        SIMPLE_MANUAL_YAW_LEG_SCALE_MAX);
            float leg_max_pb = SIMPLE_MANUAL_YAW_LEG_MAX * leg_scale_pb;
            diff_ref_pb = -SIMPLE_MANUAL_YAW_LEG_SIGN *
                          SIMPLE_MANUAL_YAW_LEG_KP * yaw_rate_err_leg;
            diff_ref_pb = clampf_local(diff_ref_pb,
                                       -leg_max_pb, leg_max_pb);
        }
        float dstep_pb = SIMPLE_MANUAL_YAW_LEG_RATE * dt_f;
        g_balance.leg_swing_diff += clampf_local(
            diff_ref_pb - g_balance.leg_swing_diff, -dstep_pb, dstep_pb);
    }
    return;
