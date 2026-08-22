/* balance_step_sensor.inc - 简化三环路径: 传感器预处理/滤波/接触状态
 * 由 balance_core.c 的 balance_update() include 展开。 */
    {
    g_balance.run_time += dt_f;

    if (!g_balance.yaw_ready) {
        /* 偏航零点取启动瞬间值，之后只控制相对偏航，避免依赖世界朝向。 */
        g_balance.yaw_raw_prev = yaw_ang;
        g_balance.yaw_zero = yaw_ang;
        g_balance.yaw_cont = 0.0f;
        g_balance.yaw_err_dbg = 0.0f;
        g_balance.yaw_ready = 1;
    } else {
        g_balance.yaw_cont += wrap_pi_local(yaw_ang - g_balance.yaw_raw_prev);
        g_balance.yaw_raw_prev = yaw_ang;
    }
    float yaw_err_pb = wrap_pi_local(yaw_ang - g_balance.yaw_zero);
    g_balance.yaw_err_dbg = yaw_err_pb;

    if (!g_balance.pitch_rate_ready) {
        g_balance.pitch_rate_filt = pitch_rate;
        g_balance.pitch_rate_ready = 1;
    } else {
        float pr_alpha = dt_f / (SIMPLE_PITCH_RATE_TAU + dt_f);
        pr_alpha = clampf_local(pr_alpha, 0.0f, 1.0f);
        g_balance.pitch_rate_filt += pr_alpha * (pitch_rate - g_balance.pitch_rate_filt);
    }

    /* 简单三环路径沿用控制器/日志的前向约定:
     * 正轮速对应当前模型里的机体正 x 方向。 */
    float wheel_v_raw = wheel_v;
    wheel_v_raw = clampf_local(wheel_v_raw, -SIMPLE_WHEEL_CTRL_V_MAX, SIMPLE_WHEEL_CTRL_V_MAX);
    if (!g_balance.simple_wheel_v_ready) {
        g_balance.simple_wheel_v_limited = wheel_v_raw;
        g_balance.simple_wheel_v_ready = 1;
    } else {
        float dv_max_pb = SIMPLE_WHEEL_CTRL_DV_MAX;
        if (dt_f > 0.0015f) dv_max_pb *= dt_f / 0.001f;
        g_balance.simple_wheel_v_limited += clampf_local(wheel_v_raw - g_balance.simple_wheel_v_limited,
                                                 -dv_max_pb, dv_max_pb);
    }
    wheel_v_raw = g_balance.simple_wheel_v_limited;
    float cp = cosf(pitch), sp = sinf(pitch);
    /* 将惯性加速度投影到机体前向，并扣除俯仰引入的重力分量。 */
    float imu_acc_raw = cp * acc_x + sp * acc_z - 9.81f * sp;
    if (imu_acc_raw >  20.0f) imu_acc_raw =  20.0f;
    if (imu_acc_raw < -20.0f) imu_acc_raw = -20.0f;

    /* 由腿关节速度估计轮轴相对机体的运动，用于补偿轮速中的腿摆/腿长变化分量。
     * v3: 用闭链腿线性运动学映射(sw/L 单位 m)。 */
#if MODEL_V3
    float swing_dot, l_dot, r_dot;
    leg_kin_v3_velocities(lf_vel_j, lr_vel_j, rf_vel_j, rr_vel_j,
                          &swing_dot, &l_dot, &r_dot);
#else
    float left_swing_dot = 0.5f * (lf_vel_j - lr_vel_j);
    float right_swing_dot = 0.5f * (rr_vel_j - rf_vel_j);
    float swing_dot = 0.5f * (left_swing_dot + right_swing_dot);
    float left_ext_dot = -0.5f * (lf_vel_j + lr_vel_j);
    float right_ext_dot = 0.5f * (rf_vel_j + rr_vel_j);
#endif
    float leg_kin_v = (LEG_AXLE_DX_DSW * swing_dot +
                       LEG_AXLE_DX_DEL * l_dot +
                       LEG_AXLE_DX_DER * r_dot);
    float pitch_kin_v = LEG_AXLE_Z_NOM * pitch_rate;
    float wheel_v_body = wheel_v_raw - leg_kin_v - pitch_kin_v;
    g_balance.leg_kin_v_dbg = leg_kin_v;
    g_balance.pitch_kin_v_dbg = pitch_kin_v;
    g_balance.wheel_body_v_dbg = wheel_v_body;

    float wheel_v_prev = g_balance.wheel_v_filt;
    float a_fast = dt_f / (0.012f + dt_f);
    if (a_fast > 1.0f) a_fast = 1.0f;
    if (!g_balance.filter_ready) {
        g_balance.wheel_v_filt = wheel_v_raw;
        g_balance.body_v_filt = wheel_v_body;
        g_balance.imu_acc_filt = imu_acc_raw;
        g_balance.drive_acc_filt = 0.0f;
        g_balance.filter_ready = 1;
    } else {
        g_balance.wheel_v_filt += a_fast * (wheel_v_raw - g_balance.wheel_v_filt);
        g_balance.body_v_filt += a_fast * (wheel_v_body - g_balance.body_v_filt);
        g_balance.imu_acc_filt += a_fast * (imu_acc_raw - g_balance.imu_acc_filt);
    }
    g_balance.wheel_acc_dbg = (g_balance.wheel_v_filt - wheel_v_prev) / (dt_f > 1e-5f ? dt_f : 0.001f);
    g_balance.imu_for_slip_dbg = g_balance.imu_acc_filt;
    g_balance.wheel_imu_res_dbg = fabsf(g_balance.wheel_acc_dbg - g_balance.imu_for_slip_dbg);

    /* 腿负载既用于离地判断，也给腿长一个轻微阻尼，减小弹跳。 */
    float leg_load_common = 0.25f * (fabsf(lf_tau) + fabsf(lr_tau) + fabsf(rf_tau) + fabsf(rr_tau));
    float ext_rate_common = 0.5f * (l_dot + r_dot);
    /* 实测虚拟腿长，与辨识状态定义一致，供行车记录仪/未来模型分段选参使用。 */
#if MODEL_V3
    float sw_dbg, L_dbg, R_dbg;
    leg_kin_v3_forward(lf_pos, lr_pos, rf_pos, rr_pos,
                       &sw_dbg, &L_dbg, &R_dbg);
    g_balance.leg_len_dbg = 0.5f * (L_dbg + R_dbg);
#else
    float left_len_dbg = 0.5f * (0.4f - lf_pos - lr_pos);
    float right_len_dbg = 0.5f * (rf_pos + rr_pos + 0.5f);
    g_balance.leg_len_dbg = 0.5f * (left_len_dbg + right_len_dbg);
#endif
    float ext_damp_ref = -0.0022f * ext_rate_common;
    if (leg_load_common > 22.0f) ext_damp_ref += 0.0020f;
    ext_damp_ref = clampf_local(ext_damp_ref, -0.006f, 0.006f);

#if SIMPLE_PURE_BALANCE
    float v_damp_pb = clampf_local(g_balance.body_v_filt, -0.90f, 0.90f);
    float abs_pitch_pb = fabsf(pitch);
    float abs_wheel_pb = fabsf(g_balance.wheel_v_filt);
    float abs_body_pb = fabsf(g_balance.body_v_filt);
    /* 接触状态判断: 离地/冲击/打滑都只用现实传感器，不用仿真接触标志。 */
    int airborne_pb = (acc_mag < 4.0f && leg_load_common < 1.2f);
    int impact_pb = (acc_mag > 150.0f || fabsf(g_balance.imu_acc_filt) > 70.0f);
    int slip_pb = (!airborne_pb && !impact_pb && abs_wheel_pb > 0.80f &&
                   g_balance.wheel_imu_res_dbg > SIMPLE_SLIP_RESIDUAL &&
                   abs_pitch_pb < 0.35f);
    /* 低速打滑: 轮子几乎不转(|wf|<0.15)但 IMU 前向加速度非零
     * (|imu_acc|>0.1), 说明车在滑而轮不滚(慢漂根因)。带滞回计数
     * 防误判: 连续 4 步才算打滑, 连续 40 步不满足才解除。 */
    int slip_low_cond_pb = (!airborne_pb && !impact_pb &&
                            abs_wheel_pb < 0.15f &&
                            fabsf(g_balance.imu_acc_filt) > 0.10f &&
                            abs_pitch_pb < 0.35f);
    if (slip_low_cond_pb)
        g_balance.slip_score++;
    else
        g_balance.slip_score--;
    g_balance.slip_score = (g_balance.slip_score > 4) ? 4 :
                           (g_balance.slip_score < -40) ? -40 : g_balance.slip_score;
    int slip_low_pb = (g_balance.slip_score > 0);
    int zupt_pb = (!airborne_pb && !impact_pb && g_balance.run_time > 1.20f &&
                   abs_pitch_pb < 0.060f && fabsf(pitch_rate) < 1.2f &&
                   abs_body_pb < 0.008f && fabsf(g_balance.imu_acc_filt) < 1.5f);
#if SIMPLE_WHEEL_CAPTURE_ENABLE
    int capture_allowed_pb = (!airborne_pb && !impact_pb &&
                              abs_pitch_pb < SIMPLE_WHEEL_CAPTURE_PITCH_RAD &&
                              g_balance.run_time > 0.70f);
    if (capture_allowed_pb &&
        (abs_body_pb > SIMPLE_WHEEL_CAPTURE_ENTER_V ||
         (g_balance.wheel_capture_dir != 0.0f && abs_body_pb > SIMPLE_WHEEL_CAPTURE_EXIT_V))) {
        float new_dir_pb = (g_balance.body_v_filt > 0.0f) ? 1.0f : -1.0f;
        if (g_balance.wheel_capture_dir == 0.0f ||
            fabsf(g_balance.body_v_filt) > SIMPLE_WHEEL_CAPTURE_ENTER_V) {
            g_balance.wheel_capture_dir = new_dir_pb;
        }
        if (g_balance.wheel_capture_dir * g_balance.body_v_filt <= 0.0f &&
            abs_body_pb < SIMPLE_WHEEL_CAPTURE_EXIT_V * 1.4f) {
            g_balance.wheel_capture_dir = 0.0f;
            g_balance.wheel_capture_t = 0.0f;
        } else {
            g_balance.wheel_capture_t += dt_f;
        }
    } else {
        g_balance.wheel_capture_dir = 0.0f;
        g_balance.wheel_capture_t = 0.0f;
    }
#else
    g_balance.wheel_capture_dir = 0.0f;
    g_balance.wheel_capture_t = 0.0f;
#endif
    /* 轮里程计可信度: 离地不用轮速，冲击/打滑只保守使用，高速轻微降权。 */
    float wheel_trust_pb = 1.0f;
    if (airborne_pb) wheel_trust_pb = 0.0f;
    else if (impact_pb) wheel_trust_pb = 0.15f;
    else if (slip_pb) wheel_trust_pb = 0.15f;
    else if (slip_low_pb) wheel_trust_pb = 0.30f;
    else if (abs_wheel_pb > 3.20f) wheel_trust_pb = 0.70f;
    int stable_odom_pb = (g_balance.run_time > 0.50f && abs_pitch_pb < 0.30f &&
                          fabsf(pitch_rate) < 5.0f && !airborne_pb && !impact_pb);
    /* 控制目标是“停车保持点”: 回程机制负责把它维持住，而不是回启动原点。 */
    float pos_ctrl_pb = g_balance.pos - g_balance.hold_pos;
    SimpleOriginWindow origin_pb = make_simple_origin_window(pos_ctrl_pb,
                                                             g_balance.body_v_filt,
                                                             pitch,
                                                             g_balance.pitch_rate_filt,
                                                             g_balance.imu_acc_filt,
                                                             wheel_trust_pb,
                                                             stable_odom_pb,
                                                             slip_pb,
                                                             zupt_pb);
    {
        float trust_alpha_pb = dt_f / (0.050f + dt_f);
        trust_alpha_pb = clampf_local(trust_alpha_pb, 0.0f, 1.0f);
    g_balance.origin_trust += trust_alpha_pb * (origin_pb.trust_ref - g_balance.origin_trust);
    }
    float neutral_pb = SIMPLE_BASE_NEUTRAL + SIMPLE_NEUTRAL_VEL * v_damp_pb;
    int origin_time_ok_pb;
#if SIMPLE_SPEED_PLAN_ENABLE
    origin_time_ok_pb = (g_balance.run_time > SIMPLE_SPEED_PLAN_START_TIME);
#else
    origin_time_ok_pb = (g_balance.run_time > 1.60f);
#endif
    if (SIMPLE_ORIGIN_ENABLE && origin_time_ok_pb && origin_pb.return_ok &&
        abs_pitch_pb < SIMPLE_ORIGIN_ACTIVE_PITCH_RAD) {
        float pos_err_pb = origin_pb.pos_err;
#if SIMPLE_SPEED_PLAN_ENABLE
        /* 原点控制链路: 位置误差 -> 目标速度 -> 目标俯仰角。
         * 已通过小信号辨识确认当前模型中正目标俯仰角会增加前向速度，
         * 所以速度误差到俯仰角的符号为负。
         */
        float pos_kp_plan_pb = SIMPLE_SPEED_PLAN_POS_KP;
        float v_max_plan_pb = SIMPLE_SPEED_PLAN_V_MAX;
        float pitch_kp_plan_pb = SIMPLE_SPEED_PLAN_PITCH_KP;
        float pitch_max_plan_pb = SIMPLE_SPEED_PLAN_PITCH_MAX;
#if SIMPLE_ORIGIN_SHORT_WINDOW_ENABLE
        /* 短窗口内使用更积极的回零参数；窗口外不再积分，只使用有界位置记忆回正。 */
        pos_kp_plan_pb = SIMPLE_ORIGIN_FAST_POS_KP;
        v_max_plan_pb = SIMPLE_ORIGIN_FAST_V_MAX;
        pitch_kp_plan_pb = SIMPLE_ORIGIN_FAST_PITCH_KP;
        pitch_max_plan_pb = SIMPLE_ORIGIN_FAST_PITCH_MAX;
#endif
        if (g_balance.manual_active) {
            /* 手动跟踪更柔和，避免追移动目标时过冲成高速漂移。 */
            pos_kp_plan_pb *= SIMPLE_MANUAL_TRACK_KP_SCALE;
            v_max_plan_pb *= SIMPLE_MANUAL_TRACK_KP_SCALE;
        }
        float v_ref_pb = -pos_kp_plan_pb * pos_err_pb;
        v_ref_pb = clampf_local(v_ref_pb,
                                -v_max_plan_pb,
                                 v_max_plan_pb);
        float v_err_pb = clampf_local(g_balance.body_v_filt - v_ref_pb, -3.0f, 3.0f);
        float pos_abs_pb = fabsf(pos_err_pb);
        int moving_home_pb = (pos_err_pb * g_balance.body_v_filt < -0.02f);
        int moving_away_pb = (pos_err_pb * g_balance.body_v_filt > 0.02f);
        /* 减速斜坡: 越接近原点，目标速度越小，形成可刹停的速度剖面，
         * 避免以高速冲过零点后再反向救车。 */
        float ramp_scale_pb = 1.0f;
        if (pos_abs_pb < SIMPLE_ORIGIN_APPROACH_RAMP_POS) {
            ramp_scale_pb = SIMPLE_ORIGIN_APPROACH_V_MIN_FRAC +
                (1.0f - SIMPLE_ORIGIN_APPROACH_V_MIN_FRAC) *
                (pos_abs_pb / SIMPLE_ORIGIN_APPROACH_RAMP_POS);
            v_ref_pb *= ramp_scale_pb;
        }
        /* 近场捕获: 已贴近原点还有速度时，双向都往目标速度 0 刹。
         * 增益和限幅都保持温和，配合更快的 neutral 滤波，避免刹停后俯仰过冲反弹。 */
        int near_stop_pb = (pos_abs_pb < SIMPLE_ORIGIN_STOP_POS &&
                            fabsf(g_balance.body_v_filt) > SIMPLE_ORIGIN_STOP_V);
        int creep_pb = (pos_abs_pb < SIMPLE_ORIGIN_STOP_POS &&
                        fabsf(g_balance.body_v_filt) <= SIMPLE_ORIGIN_STOP_V);
        /* 近场直接轮扭矩模式: 贴近原点、速度不大且俯仰不大时，不再走“目标俯仰角”
         * 链路。目标俯仰一旦变化，身体仰身过程本身会先把轮子反向推 0.3~0.5 m。
         * 这里保持当前俯仰角（p_err 归零），由 tau 段的直接刹停 + 爬行扭矩处理。
         * 高速或大俯仰时退回俯仰式刹停/爬行兜底。 */
        int near_direct_ok_pb = (pos_abs_pb < SIMPLE_ORIGIN_STOP_POS &&
                                 fabsf(g_balance.body_v_filt) < 0.45f &&
                                 abs_pitch_pb < 0.100f &&
                                 fabsf(g_balance.pitch_rate_filt) < 1.5f);
#if !SIMPLE_ORIGIN_DIRECT_TAU_ENABLE
        near_direct_ok_pb = 0;
#endif
        if (near_direct_ok_pb) {
            g_balance.origin_creep = 1.0f;
            neutral_pb = pitch;
        } else {
            g_balance.origin_creep = 0.0f;
            if (near_stop_pb) {
                v_ref_pb = 0.0f;
                v_err_pb = g_balance.body_v_filt;
                pitch_kp_plan_pb = SIMPLE_ORIGIN_STOP_PITCH_KP;
                pitch_max_plan_pb = SIMPLE_ORIGIN_STOP_PITCH_MAX;
            } else if (creep_pb) {
                /* 大俯仰兜底爬行: 与直接扭矩模式效果相同，但走俯仰角链路。 */
                v_ref_pb = -SIMPLE_ORIGIN_FAST_POS_KP * pos_err_pb * ramp_scale_pb;
                v_ref_pb = clampf_local(v_ref_pb,
                                        -SIMPLE_ORIGIN_FAST_V_MAX,
                                         SIMPLE_ORIGIN_FAST_V_MAX);
                v_err_pb = clampf_local(g_balance.body_v_filt - v_ref_pb,
                                        -1.0f, 1.0f);
                pitch_kp_plan_pb = SIMPLE_ORIGIN_CREEP_PITCH_KP;
                pitch_max_plan_pb = SIMPLE_ORIGIN_CREEP_PITCH_MAX;
            } else if (moving_home_pb) {
                pitch_kp_plan_pb *= SIMPLE_SPEED_PLAN_HOMEBOUND_GAIN;
                pitch_max_plan_pb = g_balance.manual_active
                                  ? SIMPLE_MANUAL_HOMEBOUND_MAX
                                  : SIMPLE_SPEED_PLAN_HOMEBOUND_MAX;
                if (pos_abs_pb < SIMPLE_ORIGIN_APPROACH_RAMP_POS) {
                    /* 斜坡内回程刹车更硬，抵消目标俯仰角响应延迟。 */
                    pitch_kp_plan_pb = SIMPLE_ORIGIN_APPROACH_HOMEBOUND_KP;
                    pitch_max_plan_pb = SIMPLE_ORIGIN_APPROACH_PITCH_MAX;
                    if (g_balance.manual_active)
                        pitch_max_plan_pb = SIMPLE_MANUAL_HOMEBOUND_MAX;
                }
            } else if (moving_away_pb) {
                pitch_kp_plan_pb *= g_balance.manual_active
                                  ? SIMPLE_MANUAL_AWAY_GAIN
                                  : SIMPLE_SPEED_PLAN_AWAY_GAIN;
                pitch_max_plan_pb = g_balance.manual_active
                                  ? SIMPLE_MANUAL_AWAY_MAX
                                  : SIMPLE_SPEED_PLAN_AWAY_MAX;
                if (pos_abs_pb < SIMPLE_ORIGIN_APPROACH_RAMP_POS) {
                    pitch_kp_plan_pb = SIMPLE_ORIGIN_APPROACH_AWAY_KP;
                    pitch_max_plan_pb = SIMPLE_ORIGIN_APPROACH_PITCH_MAX;
                    if (g_balance.manual_active)
                        pitch_max_plan_pb = SIMPLE_MANUAL_AWAY_MAX;
                }
            }
            float plan_neutral_pb = -pitch_kp_plan_pb * v_err_pb;
            {
                /* 直接位置补偿只在低速时叠加: 高速回程时把它压掉，
                 * 避免位置项继续往运动方向加速，放大接近零点时的动量。 */
                float direct_gate_pb = 1.0f - fabsf(g_balance.body_v_filt) /
                                            SIMPLE_ORIGIN_DIRECT_SPEED_GATE;
                direct_gate_pb = clampf_local(direct_gate_pb, 0.0f, 1.0f);
                /* 符号实测: +倾角指令(neutral>0)会让车向后跑(与 W 前进
                 * 相反，位置环增益符号需取反)。正反馈会让保持修正放大
                 * 成跑飞(实测 t=120s 起 tp +9° 后车加速后退到 -180m)。 */
                float pos_neutral_pb = +SIMPLE_ORIGIN_DIRECT_PITCH_KP *
                                       pos_err_pb * direct_gate_pb;
                pos_neutral_pb = clampf_local(pos_neutral_pb,
                                              -SIMPLE_ORIGIN_DIRECT_PITCH_MAX,
                                               SIMPLE_ORIGIN_DIRECT_PITCH_MAX);
                plan_neutral_pb += pos_neutral_pb;
            }
            plan_neutral_pb = clampf_local(plan_neutral_pb,
                                           -pitch_max_plan_pb,
                                            pitch_max_plan_pb);
            neutral_pb = SIMPLE_BASE_NEUTRAL + plan_neutral_pb;
            /* 静止死区(auto 保持): 速度小且位置误差小时，轮速反馈只是
             * 原地 rocking 的噪声(启动救车/开远后里程计会与真实位置
             * 失同步)，再发倾角指令会维持 pitch 慢周期振荡。静止时只
             * 保持姿态，并把保持点缓慢吸附到当前里程计——车停哪认哪，
             * 位置环只刹速度、不追一个失真的原点。 */
            if (fabsf(g_balance.body_v_filt) < 0.003f &&
                fabsf(pos_ctrl_pb) < 2.50f) {
                neutral_pb = SIMPLE_BASE_NEUTRAL;
                g_balance.hold_pos += 0.01f * dt_f *
                                      (g_balance.pos - g_balance.hold_pos);
            }
        }
#else
        g_balance.pos_i += pos_err_pb * dt_f;
        g_balance.pos_i = clampf_local(g_balance.pos_i, -0.30f, 0.30f);
        neutral_pb += SIMPLE_ORIGIN_SIGN * (SIMPLE_ORIGIN_PITCH_KP * pos_err_pb
                                           + SIMPLE_ORIGIN_PITCH_KD * v_damp_pb
                                           + SIMPLE_ORIGIN_PITCH_KI * g_balance.pos_i);
        neutral_pb = clampf_local(neutral_pb, -SIMPLE_ORIGIN_PITCH_MAX, SIMPLE_ORIGIN_PITCH_MAX);
#endif
    } else {
    g_balance.pos_i *= 0.995f;
    g_balance.origin_creep = 0.0f;
    }


#endif
