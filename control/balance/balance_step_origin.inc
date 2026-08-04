/* balance_step_origin.inc - 简化三环路径: 原点控制/手动驾驶/腿长/roll稳定
 * 由 balance_core.c 的 balance_update() include 展开。 */
    /* 空闲保持: 车低速持续 2s 后，保持点一次性吸附到当前位置。
     * 旧行为 hold_pos 只在手动进出时吸附，空闲启动后永远停在原点，
     * 位置环会一直“回程”，表现为 60-90s 后开始持续后溜。
     * 吸附后 pos_err≈0，位置环退化为纯速度刹车，车停在当前点。 */
    if (!g_balance.manual_active && !g_balance.post_manual) {
        if (fabsf(g_balance.body_v_filt) < 0.03f) {
            g_balance.idle_calm_t += dt_f;
            if (g_balance.idle_calm_t > 2.0f) {
                g_balance.hold_pos = g_balance.pos;
                g_balance.idle_calm_t = 0.0f;
            }
        } else {
            g_balance.idle_calm_t = 0.0f;
        }
    }
    if (g_balance.manual_active) {
        /* 指令斜坡化: 速度/转向指令按固定加速度爬升，避免起步瞬间猛冲。 */
        float dv_m = SIMPLE_MANUAL_ACCEL * dt_f;
        float d_m = g_balance.manual_v_target - g_balance.manual_v;
        d_m = clampf_local(d_m, -dv_m, dv_m);
        g_balance.manual_v += d_m;
        float dy_m = SIMPLE_MANUAL_YAW_ACCEL * dt_f;
        float dym = g_balance.manual_yaw_target - g_balance.manual_yaw;
        dym = clampf_local(dym, -dy_m, dy_m);
        g_balance.manual_yaw += dym;

        /* 手动限速刹车: 速度超过上限时叠加反向俯仰，防止追目标漂移成高速
         * 后急转/颠簸把车掀翻。 */
        float v_abs_m = fabsf(g_balance.body_v_filt);
        if (v_abs_m > SIMPLE_MANUAL_SPEED_BRAKE_V) {
            float brake_m = SIMPLE_MANUAL_SPEED_BRAKE_KP *
                            (v_abs_m - SIMPLE_MANUAL_SPEED_BRAKE_V);
            brake_m = clampf_local(brake_m, 0.0f, SIMPLE_MANUAL_SPEED_BRAKE_MAX);
            if (g_balance.body_v_filt > 0.0f) {
                neutral_pb -= brake_m;
            } else {
                neutral_pb += brake_m;
            }
        }

        /* 前倾角门控: 实测俯仰超过阈值时撤掉驱动/转向指令，先回正姿态。
         * 按用户要求，不限制速度，只限制前倾角。 */
        if (fabsf(pitch) > SIMPLE_MANUAL_PITCH_GATE) {
            g_balance.manual_v = 0.0f;
            g_balance.manual_v_target = 0.0f;
            g_balance.manual_yaw = 0.0f;
            g_balance.manual_yaw_target = 0.0f;
        }
        /* 手动行驶: 原点速度环同款增益(第一版方案)。速度误差 -> 目标俯仰，
         * 超速档 2.5x/0.18rad，足够强的刹车倾角才能压住“后仰前栽”耦合。
         * A/D 纯转向(无前后指令)时用 auto 级别的速度/位置保持(相对 hold_pos
         * 维持当前点)；手动弱限幅(0.09rad/1.5x)追不上漂移，实测 zero 指令
         * 下 3s 跑到 2.7m/s。 */
        if (fabsf(g_balance.manual_v) >= 0.05f) {
            float v_err_m = clampf_local(g_balance.body_v_filt -
                                         g_balance.manual_v, -3.0f, 3.0f);
            float kp_m = SIMPLE_ORIGIN_FAST_PITCH_KP;
            float max_m = SIMPLE_ORIGIN_FAST_PITCH_MAX;
            if (fabsf(v_err_m * g_balance.manual_v) > 0.02f) {
                kp_m *= SIMPLE_SPEED_PLAN_AWAY_GAIN;
                max_m = SIMPLE_SPEED_PLAN_AWAY_MAX;
            }
            neutral_pb = SIMPLE_BASE_NEUTRAL - kp_m * v_err_m;
            neutral_pb = clampf_local(neutral_pb, -max_m, max_m);
        } else {
            /* 零指令保持: 速度刹向 0 + 位置保持在进入点(不追回启动原点)。 */
            float pos_err_m = g_balance.pos - g_balance.hold_pos;
            /* 静止死区: 位置/速度都足够小时不再给倾角指令，靠俯仰环自带
             * 速度阻尼自然稳住，避免保持计划反复过冲造成 pitch 慢周期晃动。 */
            int rest_m = (fabsf(pos_err_m) < 0.45f &&
                          fabsf(g_balance.body_v_filt) < 0.02f);
            if (rest_m) {
                /* 静止时也保留软速度刹车: pitch=0 未必是零速平衡点，
                 * 完全清零会让 0.02~0.05m/s 的慢溜自由漂(实测漂移根因)。 */
                neutral_pb = SIMPLE_BASE_NEUTRAL - 0.10f * g_balance.body_v_filt;
            } else {
                float v_ref_m = -SIMPLE_ORIGIN_FAST_POS_KP * pos_err_m;
                v_ref_m = clampf_local(v_ref_m,
                                       -SIMPLE_ORIGIN_FAST_V_MAX,
                                        SIMPLE_ORIGIN_FAST_V_MAX);
                float v_err_m = clampf_local(g_balance.body_v_filt - v_ref_m,
                                             -3.0f, 3.0f);
                float kp_m = SIMPLE_ORIGIN_FAST_PITCH_KP;
                float max_m = SIMPLE_SPEED_PLAN_AWAY_MAX;
                if (pos_err_m * g_balance.body_v_filt > 0.02f) {
                    kp_m *= SIMPLE_SPEED_PLAN_AWAY_GAIN;
                } else if (pos_err_m * g_balance.body_v_filt < -0.02f) {
                    kp_m *= SIMPLE_SPEED_PLAN_HOMEBOUND_GAIN;
                }
                neutral_pb = SIMPLE_BASE_NEUTRAL - kp_m * v_err_m;
                float direct_gate_m = 1.0f -
                                      fabsf(g_balance.body_v_filt) /
                                      SIMPLE_ORIGIN_DIRECT_SPEED_GATE;
                direct_gate_m = clampf_local(direct_gate_m, 0.0f, 1.0f);
                float pos_n_m = -SIMPLE_ORIGIN_DIRECT_PITCH_KP *
                                pos_err_m * direct_gate_m;
                pos_n_m = clampf_local(pos_n_m,
                                       -SIMPLE_ORIGIN_DIRECT_PITCH_MAX,
                                        SIMPLE_ORIGIN_DIRECT_PITCH_MAX);
                neutral_pb += pos_n_m;
                neutral_pb = clampf_local(neutral_pb,
                                          -(max_m + SIMPLE_ORIGIN_DIRECT_PITCH_MAX),
                                           max_m + SIMPLE_ORIGIN_DIRECT_PITCH_MAX);
            }
        }
        /* 目标俯仰变化率限幅: 只在 W/S 速度指令时启用(按键切换/方向反转
         * 时 tp 慢慢建立，避免瞬间打满 ±10° 造成 pitch 甩动)。
         * 零指令保持直接走 SIMPLE_NEUTRAL_TAU 滤波，刹车响应不能太慢。 */
        if (fabsf(g_balance.manual_v) >= 0.05f) {
            float slew_m = SIMPLE_MANUAL_NEUTRAL_SLEW * dt_f;
            float d_m = neutral_pb - g_balance.manual_neutral;
            d_m = clampf_local(d_m, -slew_m, slew_m);
            g_balance.manual_neutral += d_m;
            neutral_pb = g_balance.manual_neutral;
        }
        /* 滚转安全网: 车身明显倾斜时自动撤手动指令，防止边缘状态下翻车。 */
        if (fabsf(roll) > 0.25f) {
            g_balance.manual_v = 0.0f;
            g_balance.manual_v_target = 0.0f;
            g_balance.manual_yaw = 0.0f;
            g_balance.manual_yaw_target = 0.0f;
        }
        g_balance.origin_creep = 0.0f;
        g_balance.post_manual = 0;
        g_balance.post_manual_t = 0.0f;
    } else if (g_balance.post_manual) {
        /* 松手停车刹车: 行驶中里程计失真，不能追位置；目标速度 0，
         * bang-bang 强刹(小比例倾角会形成“后仰前栽”爬行平衡刹不停)。
         * 速度低于阈值持续 0.3s 或超时 2s 后退出，交给自动保持。 */
        g_balance.post_manual_t += dt_f;
        float v_err_stop = clampf_local(g_balance.body_v_filt, -2.0f, 2.0f);
        neutral_pb = SIMPLE_BASE_NEUTRAL -
                     SIMPLE_MANUAL_STOP_KP * v_err_stop;
        neutral_pb = clampf_local(neutral_pb,
                                  -SIMPLE_MANUAL_STOP_MAX,
                                   SIMPLE_MANUAL_STOP_MAX);
        g_balance.origin_creep = 0.0f;
        if (fabsf(g_balance.body_v_filt) < SIMPLE_MANUAL_STOP_V &&
            g_balance.post_manual_t > 0.30f) {
            g_balance.post_manual = 0;
            g_balance.post_manual_rest = 0;
    g_balance.runaway_t = 0.0f;
    g_balance.pitch_i = 0.0f;
            g_balance.hold_pos = g_balance.pos;
            g_balance.yaw_zero = g_balance.yaw_raw_prev;
        } else if (g_balance.post_manual_t > 2.0f) {
            g_balance.post_manual = 0;
            g_balance.post_manual_rest = 0;
    g_balance.runaway_t = 0.0f;
    g_balance.pitch_i = 0.0f;
            g_balance.hold_pos = g_balance.pos;
            g_balance.yaw_zero = g_balance.yaw_raw_prev;
        }
    }
#if SIMPLE_IDENT_NEUTRAL_ENABLE
    if (g_balance.run_time > SIMPLE_IDENT_NEUTRAL_START) {
        float ident_t_pb = g_balance.run_time - SIMPLE_IDENT_NEUTRAL_START;
        float ident_phase_pb = fmodf(ident_t_pb, SIMPLE_IDENT_NEUTRAL_PERIOD);
        float ident_half_pb = 0.5f * SIMPLE_IDENT_NEUTRAL_PERIOD;
        neutral_pb = (ident_phase_pb < ident_half_pb)
                   ? SIMPLE_IDENT_NEUTRAL_AMP
                   : -SIMPLE_IDENT_NEUTRAL_AMP;
    }
#endif
#if SIMPLE_CAPTURE_LEAN_ENABLE
    if (stable_odom_pb &&
        fabsf(g_balance.body_v_filt) > SIMPLE_CAPTURE_LEAN_V &&
        abs_pitch_pb < SIMPLE_CAPTURE_LEAN_MAX &&
        !airborne_pb && !impact_pb) {
        float capture_neutral_pb = pitch +
            ((g_balance.body_v_filt > 0.0f) ? SIMPLE_CAPTURE_LEAN_STEP : -SIMPLE_CAPTURE_LEAN_STEP);
        capture_neutral_pb = clampf_local(capture_neutral_pb,
                                          -SIMPLE_CAPTURE_LEAN_MAX,
                                           SIMPLE_CAPTURE_LEAN_MAX);
        neutral_pb = capture_neutral_pb;
    }
#endif
#if SIMPLE_WHEEL_CAPTURE_ENABLE
    if (g_balance.wheel_capture_dir != 0.0f) {
        float cap_offset_pb = SIMPLE_WHEEL_CAPTURE_OFFSET_MIN +
            SIMPLE_WHEEL_CAPTURE_OFFSET_KV *
            (fabsf(g_balance.body_v_filt) - SIMPLE_WHEEL_CAPTURE_ENTER_V);
        cap_offset_pb = clampf_local(cap_offset_pb,
                                     SIMPLE_WHEEL_CAPTURE_OFFSET_MIN,
                                     SIMPLE_WHEEL_CAPTURE_OFFSET_MAX);
        neutral_pb = g_balance.wheel_capture_dir * cap_offset_pb;
        neutral_pb = clampf_local(neutral_pb,
                                  -SIMPLE_WHEEL_CAPTURE_NEUTRAL_MAX,
                                   SIMPLE_WHEEL_CAPTURE_NEUTRAL_MAX);
    }
#endif
    /* 近场直接扭矩模式里目标俯仰只慢速跟随当前角度（稳定但不主动仰身），
     * 避免目标俯仰阶跃造成的反向冲量；远场仍用快滤波保证回程响应。 */
    float neutral_tau_pb = (g_balance.wheel_capture_dir != 0.0f) ? 0.030f
                         : (g_balance.origin_creep > 0.5f ? 0.350f
                                                          : SIMPLE_NEUTRAL_TAU);
    g_balance.neutral += (dt_f / (neutral_tau_pb + dt_f)) * (neutral_pb - g_balance.neutral);
#if MANUAL_JUMP_ENABLE
    /* 蹬伸阶段(轮子还着地)给轮子一个小的前驱，抵消起跳时车体后仰，
     * 让空中姿态更水平、落地更稳。 */
    if (g_balance.jump_active && g_balance.jump_t >= JUMP_CROUCH_T &&
        g_balance.jump_t < JUMP_CROUCH_T + JUMP_LAUNCH_T) {
        g_balance.neutral -= 0.012f;
    }
#endif
    float leg_ref_pb = 0.0f;
    if (SIMPLE_ORIGIN_ENABLE && g_balance.run_time > 2.0f && origin_pb.return_ok && abs_pitch_pb < 0.17f) {
        float pos_err_pb = origin_pb.pos_err;
        float leg_k_pb = SIMPLE_ORIGIN_LEG_KP;
        if (fabsf(g_balance.pos) >= 0.25f) leg_k_pb *= 2.8f;
        leg_ref_pb = SIMPLE_ORIGIN_SIGN * (leg_k_pb * pos_err_pb
                   + SIMPLE_ORIGIN_LEG_KD * clampf_local(g_balance.body_v_filt, -0.5f, 0.5f));
        leg_ref_pb = clampf_local(leg_ref_pb, -SIMPLE_ORIGIN_LEG_MAX, SIMPLE_ORIGIN_LEG_MAX);
    }
    float leg_step_pb = 0.00055f;
    g_balance.leg_swing += clampf_local(leg_ref_pb - g_balance.leg_swing, -leg_step_pb, leg_step_pb);
    float ext_step_pb = 0.00045f;
    float ext_ref_pb = ext_damp_ref;
#if MANUAL_LEG_EXTEND_ENABLE
    ext_ref_pb += g_balance.leg_extend_manual;
#endif
#if SELF_RIGHT_ENABLE
    if (g_balance.self_right_active) {
        /* 自救: 先收腿到最短(降低重心/稳定车体)，再恢复名义腿长。 */
        float sr_seq = SELF_RIGHT_RETRACT_T + SELF_RIGHT_RESTORE_T;
        float ph = fmodf(g_balance.self_right_t, sr_seq);
        ext_ref_pb = (ph < SELF_RIGHT_RETRACT_T)
                   ? SELF_RIGHT_RETRACT_EXT : SELF_RIGHT_RESTORE_EXT;
        ext_step_pb = 0.0020f;   /* 自救收/放腿稍快: 0.25rad 约 0.13s */
    }
#endif
#if MANUAL_JUMP_ENABLE
    if (g_balance.jump_active) {
        g_balance.jump_t += dt_f;
        float jt = g_balance.jump_t;
        if (jt < JUMP_CROUCH_T) {
            ext_ref_pb = JUMP_CROUCH_EXT;
            ext_step_pb = JUMP_CROUCH_STEP;
        } else if (jt < JUMP_CROUCH_T + JUMP_LAUNCH_T) {
            ext_ref_pb = JUMP_LAUNCH_EXT;
            ext_step_pb = JUMP_LAUNCH_STEP;
        } else if (jt < JUMP_CROUCH_T + JUMP_LAUNCH_T + JUMP_AIR_T) {
            ext_ref_pb = JUMP_AIR_EXT;
            ext_step_pb = JUMP_AIR_STEP;
        } else if (jt < JUMP_CROUCH_T + JUMP_LAUNCH_T + JUMP_AIR_T + JUMP_LAND_T) {
            ext_ref_pb = JUMP_LAND_EXT;
            ext_step_pb = JUMP_LAND_STEP;
        } else {
            g_balance.jump_active = 0;
            g_balance.jump_t = 0.0f;
        }
    }
#endif
    ext_ref_pb = clampf_local(ext_ref_pb, -0.260f, 0.260f);
    g_balance.leg_extend += clampf_local(ext_ref_pb - g_balance.leg_extend, -ext_step_pb, ext_step_pb);

    /* ---- roll 轴稳定: 左右差动腿伸长找平 ----
     * 策略(一侧沉就抬哪侧, 同时另一侧对称收回, 保持重心高度不变):
     *   roll>0(右侧下沉) -> 收左腿(-) + 伸右腿(+) = leg_side_extend<0
     *   roll<0(左侧下沉) -> 伸左腿(+) + 收右腿(-) = leg_side_extend>0
     * 即 side_ref = -KP*roll - KD*roll_rate。差动方式比只抬/只收一侧
     * 效率高一倍, 且不改变左右平均腿长, 不扰动共模高度/俯仰。
     * 空中/冲击/跳跃/自救/大幅侧倾时退出, 避免与姿态控制和翻车判定打架。
     */
    {
        float rr_alpha = dt_f / (0.020f + dt_f);
        rr_alpha = clampf_local(rr_alpha, 0.0f, 1.0f);
        g_balance.roll_rate_filt += rr_alpha * (roll_rate - g_balance.roll_rate_filt);

        int roll_stab_ok =
            !g_balance.jump_active &&
            !g_balance.self_right_active &&
            !airborne_pb && !impact_pb &&
            g_balance.run_time > ROLL_STAB_STARTUP &&
            fabsf(roll) < ROLL_STAB_DISABLE_ABS;

        float side_ref_pb = 0.0f;
        if (roll_stab_ok) {
            float roll_err = roll;                    /* 目标 roll=0 */
            float roll_d = g_balance.roll_rate_filt;
            float p_pb = -ROLL_STAB_KP * roll_err - ROLL_STAB_KD * roll_d;
            g_balance.roll_i += -ROLL_STAB_KI * roll_err * dt_f;
            g_balance.roll_i = clampf_local(g_balance.roll_i,
                                            -ROLL_STAB_I_MAX, ROLL_STAB_I_MAX);
            side_ref_pb = p_pb + g_balance.roll_i;
        } else {
            g_balance.roll_i *= 0.95f;   /* 退出时泄放积分 */
        }
        side_ref_pb = clampf_local(side_ref_pb, -ROLL_STAB_MAX, ROLL_STAB_MAX);
        float side_step_pb = ROLL_STAB_RATE;
        if (dt_f > 0.0015f) side_step_pb *= dt_f / 0.001f;
        g_balance.leg_side_extend += clampf_local(
            side_ref_pb - g_balance.leg_side_extend, -side_step_pb, side_step_pb);
    }

    if (origin_pb.track_ok) {
        /* 只有短窗口可信时积分位置；窗口外不继续积累长期漂移。 */
        float wheel_v_odom_pb = g_balance.body_v_filt;
        if (fabsf(wheel_v_odom_pb) < WHEEL_V_DEADZONE) wheel_v_odom_pb = 0.0f;
        wheel_v_odom_pb = clampf_local(wheel_v_odom_pb, -SIMPLE_ODOM_V_MAX, SIMPLE_ODOM_V_MAX);
        float v_int_pb = wheel_trust_pb * wheel_v_odom_pb;
        g_balance.pos += v_int_pb * dt_f;
    } else {
        g_balance.simple_imu_v *= 0.985f;
    }
    {
        /* 位置记忆限幅改在“相对虚拟目标”上做，手动开出去多远都允许。 */
        float pos_rel_pb = g_balance.pos - g_balance.hold_pos;
        pos_rel_pb = clampf_local(pos_rel_pb,
                                  -SIMPLE_ORIGIN_MEMORY_MAX,
                                   SIMPLE_ORIGIN_MEMORY_MAX);
        g_balance.pos = g_balance.hold_pos + pos_rel_pb;
    }
    if (origin_pb.release_ok) {
        /* 原点附近且速度很小时释放位置记忆，避免长期依赖轮里程计的残余偏置。 */
        float pos_rel_pb = g_balance.pos - g_balance.hold_pos;
        pos_rel_pb = decay_to_zero_local(pos_rel_pb, SIMPLE_POS_DECAY_WHEN_HOME, dt_f);
        g_balance.pos = g_balance.hold_pos + pos_rel_pb;
        g_balance.pos_i = decay_to_zero_local(g_balance.pos_i, SIMPLE_POS_DECAY_WHEN_HOME, dt_f);
    }
    g_balance.ekf_s = g_balance.pos;
    g_balance.odom_x = g_balance.pos * cosf(yaw_err_pb);
    {
        /* 横向里程计只用于日志/界面。它来自前向位移的偏航投影，不参与控制闭环。 */
        float odom_y_target_pb = g_balance.pos * sinf(yaw_err_pb);
        float odom_y_alpha_pb = dt_f / (0.080f + dt_f);
        odom_y_alpha_pb = clampf_local(odom_y_alpha_pb, 0.0f, 1.0f);
        g_balance.odom_y += odom_y_alpha_pb * (odom_y_target_pb - g_balance.odom_y);
        if (!origin_pb.track_ok || origin_pb.release_ok) {
            g_balance.odom_y = decay_to_zero_local(g_balance.odom_y,
                                                   SIMPLE_ODOM_LATERAL_DECAY,
                                                   dt_f);
        }
        g_balance.odom_y = clampf_local(g_balance.odom_y,
                                        -SIMPLE_ODOM_DISPLAY_Y_MAX,
                                         SIMPLE_ODOM_DISPLAY_Y_MAX);
    }
    g_balance.vel = g_balance.body_v_filt;
    g_balance.vel_ff = g_balance.body_v_filt;
    g_balance.wheel_trust_dbg = wheel_trust_pb;
    g_balance.pos_trust_dbg = g_balance.origin_trust;
    g_balance.airborne_dbg = airborne_pb ? 1.0f : 0.0f;
    g_balance.impact_dbg = impact_pb ? 1.0f : 0.0f;
    g_balance.slip_dbg = slip_pb ? 1.0f : 0.0f;
    g_balance.raw_airborne_dbg = g_balance.airborne_dbg;
    g_balance.raw_impact_dbg = g_balance.impact_dbg;
    g_balance.raw_slip_dbg = g_balance.slip_dbg;
    g_balance.raw_zupt_dbg = zupt_pb ? 1.0f : 0.0f;
    g_balance.support_dbg = (!airborne_pb && !impact_pb) ? 1.0f : 0.0f;
    g_balance.leg_load_dbg = leg_load_common;
    g_balance.contact_mode_dbg = (float)CONTACT_STICK;
    g_balance.obs_phase_dbg = 0.0f;
    g_balance.return_score_dbg = 0.0f;
    g_balance.capture_score_dbg = 0.0f;
    g_balance.odom_forward_dbg = g_balance.pos;
    *spd = g_balance.vel;
    *ex = g_balance.pos;

    /* 偏航在俯仰扭矩更新之后计算，这样可以用真实共模平衡力矩做门控，
     * 不需要额外发明“足够稳定”的标志位。

     */
    *sw = balance_sw_nom();
