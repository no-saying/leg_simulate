/* balance_step_legacy1.inc - 旧速度环路径: 100Hz 速度环/滤波/打滑检测
 * 仅 USE_SIMPLE_THREE_LOOP=0 时编译, 由 balance_update() include 展开。 */

    g_balance.speed_elapsed += dt_f;
    const float speed_period = 1.0f / (float)SPEED_LOOP_HZ;
    if (g_balance.speed_elapsed >= speed_period) {
        float elapsed = g_balance.speed_elapsed;
        g_balance.speed_elapsed = 0.0f;

        float left_load = 0.5f * (fabsf(lf_tau) + fabsf(lr_tau));
        float right_load = 0.5f * (fabsf(rf_tau) + fabsf(rr_tau));
        float load_hi = left_load > right_load ? left_load : right_load;
        float load_lo = left_load > right_load ? right_load : left_load;
        float side_balance = (load_hi > 1.5f) ? load_lo / (load_hi + 1e-3f) : 1.0f;
        side_balance = clampf_local(side_balance, 0.0f, 1.0f);
        float side_alpha = elapsed / (0.080f + elapsed);
        g_balance.side_support_balance += side_alpha * (side_balance - g_balance.side_support_balance);

        float leg_load = 0.25f * (fabsf(lf_tau) + fabsf(lr_tau) + fabsf(rf_tau) + fabsf(rr_tau));
        float imu_support = (acc_mag - 2.5f) / 5.5f;
        imu_support = clampf_local(imu_support, 0.0f, 1.0f);
        float load_support = (leg_load - 0.8f) / 8.0f;
        load_support = clampf_local(load_support, 0.0f, 1.0f);
        float support_conf = 0.55f * imu_support + 0.45f * load_support;
        support_conf = clampf_local(support_conf, 0.0f, 1.0f);

        int airborne = (support_conf < 0.16f && acc_mag < 5.0f);
        int impact = (acc_mag > 150.0f || fabsf(g_balance.imu_acc_filt) > 75.0f);
        int raw_slip = (!airborne && !impact && fabsf(g_balance.wheel_v_filt) > 1.80f &&
                        fabsf(g_balance.imu_acc_filt) < 2.0f);
        int raw_zupt = (!airborne && !impact && support_conf > 0.55f &&
                        fabsf(g_balance.wheel_v_filt) < 0.055f && fabsf(g_balance.imu_acc_filt) < 0.85f &&
                        fabsf(pitch_rate) < 0.45f);

        if (raw_slip) {
            if (g_balance.slip_score < 8) g_balance.slip_score++;
        } else if (g_balance.slip_score > 0) {
            g_balance.slip_score -= 2;
            if (g_balance.slip_score < 0) g_balance.slip_score = 0;
        }
        if (raw_zupt) {
            if (g_balance.zupt_score < 8) g_balance.zupt_score++;
        } else if (g_balance.zupt_score > 0) {
            g_balance.zupt_score -= 2;
            if (g_balance.zupt_score < 0) g_balance.zupt_score = 0;
        }

        int slip_like = (g_balance.slip_score >= 5);
        int zupt = (g_balance.zupt_score >= 4);
        float wheel_trust = 1.0f;
        if (airborne) wheel_trust = 0.0f;
        else if (impact) wheel_trust = 0.20f;
        else if (slip_like) wheel_trust = 0.15f;
        else if (fabsf(g_balance.wheel_v_filt) > 1.40f) wheel_trust = 0.45f;
        g_balance.wheel_trust_dbg = wheel_trust;
        g_balance.pos_trust_dbg = (!airborne && !impact) ? wheel_trust : 0.0f;
        g_balance.airborne_dbg = airborne ? 1.0f : 0.0f;
        g_balance.impact_dbg = impact ? 1.0f : 0.0f;
        g_balance.slip_dbg = slip_like ? 1.0f : 0.0f;
        g_balance.raw_airborne_dbg = airborne ? 1.0f : 0.0f;
        g_balance.raw_impact_dbg = impact ? 1.0f : 0.0f;
        g_balance.raw_slip_dbg = raw_slip ? 1.0f : 0.0f;
        g_balance.raw_zupt_dbg = raw_zupt ? 1.0f : 0.0f;
        g_balance.support_dbg = support_conf;
        g_balance.leg_load_dbg = leg_load;
        g_balance.contact_mode_dbg = airborne ? (float)CONTACT_AIRBORNE :
                             (impact ? (float)CONTACT_IMPACT :
                              (zupt ? (float)CONTACT_ZUPT :
                               (slip_like ? (float)CONTACT_SLIP : (float)CONTACT_STICK)));

        float v_obs = g_balance.wheel_v_filt;
        if (fabsf(v_obs) < WHEEL_V_DEADZONE) v_obs = 0.0f;
        if (airborne) v_obs = 0.0f;
        if (slip_like && fabsf(v_obs) > 0.80f) {
            v_obs = (v_obs > 0.0f) ? 0.80f : -0.80f;
        }
        g_balance.vel = v_obs;
        g_balance.vel_ff += (elapsed / (0.020f + elapsed)) * (v_obs - g_balance.vel_ff);

        if (!airborne && !impact && wheel_trust > 0.10f) {
            g_balance.pos += v_obs * elapsed * wheel_trust;
        }
        if (zupt) {
            g_balance.vel = 0.0f;
            g_balance.vel_ff *= 0.40f;
            g_balance.pos *= 0.995f;
        }
        g_balance.pos = clampf_local(g_balance.pos, -1.50f, 1.50f);
        g_balance.ekf_s = g_balance.pos;
        g_balance.soft_s = g_balance.pos;

        float cy = cosf(yaw_ang), sy = sinf(yaw_ang);
        g_balance.odom_x = g_balance.pos * cy;
        g_balance.odom_y = g_balance.pos * sy;
        g_balance.odom_forward_dbg = g_balance.pos;

        int startup_hold = (g_balance.run_time < 1.20f || fabsf(pitch) > 0.16f);
        float pos_err = 0.0f;
        if (g_balance.pos > 0.055f) pos_err = g_balance.pos - 0.055f;
        if (g_balance.pos < -0.055f) pos_err = g_balance.pos + 0.055f;

        if (!startup_hold && !airborne && !impact && fabsf(pos_err) > 0.010f && wheel_trust > 0.35f) {
            g_balance.pos_i += pos_err * elapsed;
        } else {
            g_balance.pos_i *= 0.965f;
        }
        g_balance.pos_i = clampf_local(g_balance.pos_i, -0.22f, 0.22f);

        float v_ref = 0.0f;
        if (!startup_hold && !airborne && !impact) {
            v_ref = -0.42f * pos_err - 0.10f * g_balance.pos_i;
            v_ref = clampf_local(v_ref, -0.22f, 0.22f);
        }
        float v_err = g_balance.vel_ff - v_ref;
        if (fabsf(v_err) < WHEEL_V_DEADZONE) v_err = 0.0f;

        if (startup_hold || airborne || impact) {
            g_balance.neutral *= 0.82f;
            g_balance.leg_swing *= 0.82f;
            g_balance.pos_i *= 0.90f;
        } else {
            float neutral_ref = -0.115f * v_err - 0.020f * pos_err - 0.010f * g_balance.pos_i;
            if (fabsf(g_balance.pos) > 0.50f) {
                neutral_ref = -0.170f * g_balance.vel_ff - 0.018f * ((g_balance.pos > 0.0f) ? 1.0f : -1.0f);
            }
            neutral_ref = clampf_local(neutral_ref, -0.030f, 0.030f);
            float an = elapsed / (0.055f + elapsed);
            g_balance.neutral += an * (neutral_ref - g_balance.neutral);

            float leg_ref = -0.020f * v_err - 0.020f * pos_err - 0.004f * g_balance.pos_i;
            leg_ref = clampf_local(leg_ref, -0.024f, 0.024f);
            float max_step = 0.0025f * (elapsed / speed_period);
            max_step = clampf_local(max_step, 0.0004f, 0.0040f);
            float dleg = leg_ref - g_balance.leg_swing;
            dleg = clampf_local(dleg, -max_step, max_step);
            g_balance.leg_swing += dleg;
        }

        float ext_ref = ext_damp_ref;
#if MANUAL_LEG_EXTEND_ENABLE
        ext_ref += g_balance.leg_extend_manual;
#endif
        if (impact && ext_ref < 0.004f) ext_ref = 0.004f;
        if (!airborne && !impact && g_balance.side_support_balance < 0.72f && support_conf > 0.35f) {
            if (ext_ref < 0.0015f) ext_ref = 0.0015f;
        }
        ext_ref = clampf_local(ext_ref, -0.260f, 0.260f);
        float ext_step = 0.00025f * (elapsed / speed_period);
        ext_step = clampf_local(ext_step, 0.0001f, 0.0007f);
        g_balance.leg_extend += clampf_local(ext_ref - g_balance.leg_extend, -ext_step, ext_step);

        float side_ref = 0.0f;
        if (!airborne && !impact && support_conf > 0.35f && g_balance.side_support_balance < 0.72f) {
            side_ref = 0.004f * (right_load - left_load) / (left_load + right_load + 1e-3f);
        }
        side_ref = clampf_local(side_ref, -0.004f, 0.004f);
        float side_step = 0.00022f * (elapsed / speed_period);
        side_step = clampf_local(side_step, 0.0001f, 0.0006f);
        g_balance.leg_side_extend += clampf_local(side_ref - g_balance.leg_side_extend, -side_step, side_step);

        g_balance.obs_phase_dbg = startup_hold ? 0.0f : (fabsf(g_balance.pos) > 0.50f ? 5.0f : 7.0f);
        g_balance.return_score_dbg = fabsf(pos_err) / 0.20f;
        if (g_balance.return_score_dbg > 2.0f) g_balance.return_score_dbg = 2.0f;
        g_balance.capture_score_dbg = fabsf(g_balance.vel_ff) / 0.30f;
        if (g_balance.capture_score_dbg > 2.0f) g_balance.capture_score_dbg = 2.0f;
        *spd = g_balance.vel;
        *ex = g_balance.pos;
    }

    g_balance.yaw_elapsed += dt_f;
    const float yaw_period = 1.0f / (float)YAW_LOOP_HZ;
    if (g_balance.yaw_elapsed >= yaw_period) {
        g_balance.yaw_elapsed = 0.0f;
        float yaw_cmd = 3.2f * yaw_ang + 2.0f * yaw_rate;
        yaw_cmd = clampf_local(yaw_cmd, -18.0f, 18.0f);
        g_balance.yaw_held = yaw_cmd;
    }
    *yaw = g_balance.yaw_held;

    float dsw = clampf_local(g_balance.leg_swing, -0.026f, 0.026f);
    *sw = balance_sw_nom() + dsw;

    float p_err = pitch - g_balance.neutral;
    float v_damp = clampf_local(g_balance.wheel_v_filt, -1.20f, 1.20f);
    float tau_cmd = SIMPLE_PITCH_KP * p_err + SIMPLE_PITCH_KD * pitch_rate - SIMPLE_VEL_DAMP * v_damp;
    float tau_max = (g_balance.run_time < 1.20f) ? SIMPLE_TAU_START : SIMPLE_TAU_RUN;
    if (fabsf(pitch) > 0.16f) tau_max *= 0.65f;
    if (g_balance.impact_dbg > 0.5f || g_balance.airborne_dbg > 0.5f) tau_max = 50.0f;
    tau_cmd = clampf_local(tau_cmd, -tau_max, tau_max);
    float tau_rate = (g_balance.run_time < 1.20f) ? SIMPLE_RATE_START : SIMPLE_RATE_RUN;
    if (g_balance.impact_dbg > 0.5f || g_balance.airborne_dbg > 0.5f) tau_rate *= 0.70f;
    float max_delta = tau_rate * dt_f;
    g_balance.tau_cmd += clampf_local(tau_cmd - g_balance.tau_cmd, -max_delta, max_delta);
#if SELF_RIGHT_ENABLE
    if (g_balance.self_right_active) *tau = 0.0f;   /* 自救期间轮扭矩清零 */
    else
#endif
#if MANUAL_JUMP_ENABLE
    if (g_balance.jump_active && g_balance.jump_t > JUMP_CROUCH_T + JUMP_LAUNCH_T) {
        /* 空中+触地瞬间限轮扭矩(防打滑后漂)；落地缓冲期放开，
         * 让平衡环用轮子主动救姿态(轮子先着地后才能做更多挽救)。 */
        float wheel_cap = (g_balance.jump_t < JUMP_CROUCH_T + JUMP_LAUNCH_T + JUMP_AIR_T)
                        ? 6.0f : 25.0f;
        *tau = clampf_local(*tau, -wheel_cap, wheel_cap);
    } else
#endif
    *tau = g_balance.tau_cmd;
    return;
    }

    // ---- 速度环，100 Hz ----
    g_balance.speed_elapsed += dt_f;
    const float speed_period = 1.0f / (float)SPEED_LOOP_HZ;
    if (g_balance.speed_elapsed >= speed_period) {
        float elapsed = g_balance.speed_elapsed;
        g_balance.speed_elapsed = 0.0f;
        g_balance.run_time += elapsed;

        // 编码器 + 惯性测量的扩展卡尔曼里程计。
        // 状态量: [前向位移, 前向速度, 加速度偏置]。
        float cp = cosf(pitch), sp = sinf(pitch);
        // 机体系 x 方向比力，已扣除重力。
        // 保持原有极性: 闭环增益是按这个极性辨识出来的。
        // 世界系相关量在抖动下看起来可能反向，不能随手改符号。
        float imu_acc_raw = cp * acc_x + sp * acc_z - 9.81f * sp;
        float wheel_v_raw_encoder = WHEEL_ODOM_SIGN * wheel_v;
        float left_swing_dot = 0.5f * (lf_vel_j - lr_vel_j);
        float right_swing_dot = 0.5f * (rr_vel_j - rf_vel_j);
        float swing_dot = 0.5f * (left_swing_dot + right_swing_dot);
        float left_ext_dot = -0.5f * (lf_vel_j + lr_vel_j);
        float right_ext_dot = 0.5f * (rf_vel_j + rr_vel_j);
        float leg_kin_v = (LEG_AXLE_DX_DSW * swing_dot +
                           LEG_AXLE_DX_DEL * left_ext_dot +
                           LEG_AXLE_DX_DER * right_ext_dot);
        float pitch_kin_v = LEG_AXLE_Z_NOM * pitch_rate;
        float wheel_v_body = wheel_v_raw_encoder - leg_kin_v - pitch_kin_v;
        float wheel_v_raw = wheel_v_raw_encoder;
        g_balance.leg_kin_v_dbg = leg_kin_v;
        g_balance.pitch_kin_v_dbg = pitch_kin_v;
        g_balance.wheel_body_v_dbg = wheel_v_body;
        float drive_tau = 0.5f * (lw_tau - rw_tau);
        float drive_acc_raw = drive_tau * 0.022f;
        float left_swing = 0.5f * (lf_pos - lr_pos + 0.4f);
        float right_swing = 0.5f * (rr_pos - rf_pos + 0.3f);
        float swing = 0.5f * (left_swing + right_swing);
        float left_ext = 0.5f * (0.4f - lf_pos - lr_pos);
        float right_ext = 0.5f * (rf_pos + rr_pos + 0.5f);
        float leg_rel_x = (LEG_AXLE_DX_DSW * swing +
                           LEG_AXLE_DX_DEL * left_ext +
                           LEG_AXLE_DX_DER * right_ext);
        float pitch_rel_x = LEG_AXLE_Z_NOM * pitch;
        float wheel_s_comp_dbg = WHEEL_ODOM_SIGN * ((lw_pos - rw_pos) * 0.5f) * WHEEL_RADIUS
                               - leg_rel_x - pitch_rel_x;
        (void)wheel_s_comp_dbg;
        float wheel_s_raw = WHEEL_ODOM_SIGN * ((lw_pos - rw_pos) * 0.5f) * WHEEL_RADIUS;
        float wheel_ds = 0.0f;
        if (!g_balance.wheel_s_ready) {
            g_balance.wheel_s_prev = wheel_s_raw;
            g_balance.wheel_s_ready = 1;
        } else {
            wheel_ds = wheel_s_raw - g_balance.wheel_s_prev;
            g_balance.wheel_s_prev = wheel_s_raw;
        }
        if (imu_acc_raw >  20.0f) imu_acc_raw =  20.0f;
        if (imu_acc_raw < -20.0f) imu_acc_raw = -20.0f;
        if (wheel_v_raw >  4.5f) wheel_v_raw =  4.5f;
        if (wheel_v_raw < -4.5f) wheel_v_raw = -4.5f;
        if (drive_acc_raw >  6.0f) drive_acc_raw =  6.0f;
        if (drive_acc_raw < -6.0f) drive_acc_raw = -6.0f;
        if (wheel_ds >  0.06f) wheel_ds =  0.06f;
        if (wheel_ds < -0.06f) wheel_ds = -0.06f;

        // 双路径滤波，主要为了降低延迟:
        //   - 轻量一阶递归滤波，大约 8-12 ms，用于控制速度和惯性积分。
        //   - 匹配有限脉冲响应滤波，只用于抖动残差和里程计可信度。
        // 如果把 50 ms 有限脉冲响应滤波放进控制路径，会叠加信任冻结和慢恢复，
        // 体感上形成约 0.7-1.0 s 的滞后。
        float wheel_v_prev = g_balance.wheel_v_filt;
        const float iir_tau = 0.008f;  // 约 20 Hz，控制用相位滞后较小
        float iir_a = elapsed / (iir_tau + elapsed);
        if (iir_a > 1.0f) iir_a = 1.0f;
        if (!g_balance.filter_ready) {
            g_balance.wheel_v_filt = wheel_v_raw;
            g_balance.imu_acc_filt = imu_acc_raw;
            g_balance.drive_acc_filt = drive_acc_raw;
            g_balance.filter_ready = 1;
        } else {
            g_balance.wheel_v_filt += iir_a * (wheel_v_raw - g_balance.wheel_v_filt);
            g_balance.imu_acc_filt += iir_a * (imu_acc_raw - g_balance.imu_acc_filt);
            g_balance.drive_acc_filt += iir_a * (drive_acc_raw - g_balance.drive_acc_filt);
        }

        float wheel_fir = fir_push(g_balance.fir_wheel, wheel_v_raw);
        g_balance.wheel_fir = wheel_fir;
        float imu_fir = fir_push(g_balance.fir_imu, imu_acc_raw);
        (void)fir_push(g_balance.fir_drive, drive_acc_raw);
        if (g_balance.fir_fill < FIR_N) g_balance.fir_fill++;
        if (g_balance.fir_fill < FIR_N) {
            g_balance.wheel_thrash = 0.0f;
            g_balance.imu_thrash = 0.0f;
        } else {
            // 相位对齐的抖动残差: |raw(t-M) - FIR(t)|。
            float wheel_raw_d = fir_delayed_raw(g_balance.fir_wheel);
            float imu_raw_d = fir_delayed_raw(g_balance.fir_imu);
            float thr_w = fabsf(wheel_raw_d - wheel_fir);
            float thr_i = fabsf(imu_raw_d - imu_fir);
            g_balance.wheel_thrash = 0.75f * g_balance.wheel_thrash + 0.25f * thr_w;
            g_balance.imu_thrash = 0.75f * g_balance.imu_thrash + 0.25f * thr_i;
        }
        g_balance.fir_idx++;
        if (g_balance.fir_idx >= FIR_N) g_balance.fir_idx = 0;

        float wheel_acc = (g_balance.wheel_v_filt - wheel_v_prev) / (elapsed > 1e-4f ? elapsed : 0.01f);
        float left_load = 0.5f * (fabsf(lf_tau) + fabsf(lr_tau));
        float right_load = 0.5f * (fabsf(rf_tau) + fabsf(rr_tau));
        float load_hi = left_load > right_load ? left_load : right_load;
        float load_lo = left_load > right_load ? right_load : left_load;
        float side_balance = 1.0f;
        if (load_hi > 1.5f) {
            side_balance = load_lo / (load_hi + 1e-3f);
        }
        if (side_balance > 1.0f) side_balance = 1.0f;
        if (side_balance < 0.0f) side_balance = 0.0f;
        float side_alpha = elapsed / (0.080f + elapsed);
        g_balance.side_support_balance += side_alpha * (side_balance - g_balance.side_support_balance);

        float leg_load = 0.25f * (fabsf(lf_tau) + fabsf(lr_tau) +
                                  fabsf(rf_tau) + fabsf(rr_tau));
        float sw_est = balance_sw_nom() + g_balance.leg_swing;
        float lf_ref = sw_est;
        float lr_ref = 0.4f - sw_est;
        float rf_ref = -0.1f - sw_est;
        float rr_ref = -0.4f + sw_est;
        float leg_angle_err = 0.25f * (fabsf(lf_pos - lf_ref) + fabsf(lr_pos - lr_ref) +
                                       fabsf(rf_pos - rf_ref) + fabsf(rr_pos - rr_ref));
        float leg_load_conf = (leg_load - 1.0f) / 10.0f;
        if (leg_load_conf > 1.0f) leg_load_conf = 1.0f;
        if (leg_load_conf < 0.0f) leg_load_conf = 0.0f;
        float leg_angle_conf = 1.0f - (leg_angle_err - 0.08f) / 0.28f;
        if (leg_angle_conf > 1.0f) leg_angle_conf = 1.0f;
        if (leg_angle_conf < 0.0f) leg_angle_conf = 0.0f;
        float imu_support_conf = (acc_mag - 2.5f) / 5.5f;
        if (imu_support_conf > 1.0f) imu_support_conf = 1.0f;
        if (imu_support_conf < 0.0f) imu_support_conf = 0.0f;
        float support_conf = leg_angle_conf * (0.45f * imu_support_conf + 0.55f * leg_load_conf);
        if (imu_support_conf > 0.85f && leg_angle_conf > 0.75f && support_conf < 0.65f) {
            support_conf = 0.65f;
        }

        int raw_airborne = (support_conf < 0.18f && acc_mag < 5.0f);
#if !USE_IMU_ONLY_ODOM
        raw_airborne = raw_airborne || (acc_mag < 2.0f && fabsf(g_balance.wheel_v_filt) > 0.30f);
#endif
        int raw_impact = (acc_mag > 170.0f || fabsf(imu_acc_raw) > 95.0f ||
                          (leg_load > 65.0f && fabsf(imu_acc_raw) > 40.0f));
        float imu_for_slip = g_balance.imu_acc_filt - g_balance.acc_bias;
        float wheel_imu_res = fabsf(wheel_acc - imu_for_slip);
#if USE_IMU_ONLY_ODOM
        int raw_slip = 0;
        int raw_zupt = (support_conf > 0.62f &&
                        fabsf(g_balance.imu_acc_filt) < 1.8f &&
                        fabsf(imu_acc_raw) < 2.5f &&
                        fabsf(pitch_rate) < 0.75f &&
                        fabsf(drive_tau) < 10.0f);
#else
        // 打滑判据不直接拿残差和 g_balance.vel 比，避免自锁。
        // 使用匹配 FIR 高频残差，以及运动学轮速和惯性测量的一致性。
        // 高频残差含义: 轮子剧烈抖动时 |raw-FIR| 会明显变大。
        int thrash_spin = (g_balance.wheel_thrash > 0.90f &&
                           fabsf(g_balance.wheel_v_filt) > 0.80f &&
                           support_conf > 0.25f);
        int free_spin = (support_conf > 0.28f &&
                         fabsf(g_balance.wheel_v_filt) > 1.80f &&
                         fabsf(imu_for_slip) < 2.2f &&
                         (fabsf(wheel_acc) > 8.0f || g_balance.wheel_thrash > 0.70f));
        int kinematic_slip = (support_conf > 0.35f &&
                              fabsf(g_balance.wheel_v_filt) > 1.00f &&
                              wheel_imu_res > 14.0f);
        int runaway = (fabsf(g_balance.wheel_v_filt) > WHEEL_V_RUNAWAY);
        int raw_slip = free_spin || kinematic_slip || runaway || thrash_spin;
        if (fabsf(g_balance.wheel_v_filt) > 2.80f || g_balance.wheel_thrash > 1.60f) {
            raw_slip = 1;
        }
        int raw_zupt = (support_conf > 0.58f &&
                        fabsf(g_balance.wheel_fir) < 0.18f &&
                        fabsf(wheel_acc) < 4.5f &&
                        fabsf(imu_acc_raw) < 3.2f &&
                        fabsf(g_balance.vel) < 0.35f &&
                        fabsf(pitch_rate) < 0.90f &&
                        fabsf(drive_tau) < 16.0f &&
                        g_balance.wheel_thrash < 0.75f);
        // 不能仅因为轮子抖动就做零速更新，否则会抹掉真实滑行。
        int raw_slip_zupt = 0;
        raw_zupt = raw_zupt || raw_slip_zupt;
#endif

        g_balance.raw_airborne_dbg = raw_airborne ? 1.0f : 0.0f;
        g_balance.raw_impact_dbg = raw_impact ? 1.0f : 0.0f;
        g_balance.raw_slip_dbg = raw_slip ? 1.0f : 0.0f;
        g_balance.raw_zupt_dbg = raw_zupt ? 1.0f : 0.0f;
        g_balance.wheel_acc_dbg = wheel_acc;
        g_balance.imu_for_slip_dbg = imu_for_slip;
        g_balance.wheel_imu_res_dbg = wheel_imu_res;
