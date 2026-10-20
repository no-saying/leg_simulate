/* balance_step_legacy4.inc - 旧速度环路径: 偏航环/俯仰PD/输出
 * 仅 USE_SIMPLE_THREE_LOOP=0 时编译。 */
            if (capture) g_balance.capture_score_dbg += 0.5f;
            if (fabsf(g_balance.vel) > 0.10f) g_balance.capture_score_dbg += 0.4f;
            if (thrash_hard) g_balance.capture_score_dbg += 0.3f;
            if (g_balance.capture_score_dbg > 2.0f) g_balance.capture_score_dbg = 2.0f;
        }
        if (startup_hold) g_balance.obs_phase_dbg = 0.0f;
        else if (launch_recover) g_balance.obs_phase_dbg = 1.0f;
        else if (airborne) g_balance.obs_phase_dbg = 2.0f;
        else if (impact) g_balance.obs_phase_dbg = 3.0f;
        else if (slip_like || thrash_hard) g_balance.obs_phase_dbg = 4.0f;
        else if (brake_zone) g_balance.obs_phase_dbg = 5.0f;
        else if (far_field || outbound || homebound) g_balance.obs_phase_dbg = 6.0f;
        else if (settle || near_band) g_balance.obs_phase_dbg = 7.0f;
        else g_balance.obs_phase_dbg = 8.0f;

        if (settle && origin_soft && !brake_zone) {
            g_balance.away_s *= (1.0f - 2.2f * elapsed);
            g_balance.soft_s *= (1.0f - 1.5f * elapsed);
            g_balance.ekf_s  *= (1.0f - 1.0f * elapsed);
            g_balance.pos_i  *= (1.0f - 3.0f * elapsed);
            if (g_balance.away_s < 0.05f) {
                g_balance.away_s = 0.0f;
                g_balance.brake_dir = 0.0f;
                g_balance.brake_latched = 0;
                g_balance.brake_v_sign = 0.0f;
            }
        } else if (!brake_zone && homebound && wheel_odom_ok && !thrash_hard) {
            g_balance.away_s *= (1.0f - 0.45f * elapsed);
        }

        float pos_err = 0.0f;
        if (hold_pos > 0.09f) pos_err = hold_pos - 0.09f;
        if (hold_pos < -0.09f) pos_err = hold_pos + 0.09f;

        static float s_brake_v = 0.0f;
        {
            float v_src = wheel_odom_v;
            if (!wheel_odom_ok && g_balance.fir_fill >= FIR_N && !slip_like &&
                g_balance.wheel_thrash < 0.90f && fabsf(g_balance.wheel_v_filt) < 1.80f) {
                v_src = g_balance.wheel_fir;
                if (fabsf(v_src) < WHEEL_FIR_DEADZONE) v_src = 0.0f;
            }
            if (v_src >  1.20f) v_src =  1.20f;
            if (v_src < -1.20f) v_src = -1.20f;
            float a_br = elapsed / (0.070f + elapsed);
            if (a_br > 1.0f) a_br = 1.0f;
            s_brake_v += a_br * (v_src - s_brake_v);
            if (fabsf(s_brake_v) < WHEEL_V_DEADZONE) s_brake_v *= 0.80f;
            if (s_brake_v >  1.40f) s_brake_v =  1.40f;
            if (s_brake_v < -1.40f) s_brake_v = -1.40f;
        }

        if (brake_zone && wheel_odom_ok && fabsf(s_brake_v) < 0.10f &&
            abs_away < 0.45f && abs_hold < 0.45f) {
            g_balance.away_s *= (1.0f - 1.0f * elapsed);
            if (g_balance.away_s < 0.38f) {
                g_balance.brake_latched = 0;
                g_balance.brake_v_sign = 0.0f;
            }
        }

        if (origin_soft && !settle && !near_band && !brake_zone && far_field) {
            g_balance.pos_i += pos_err * elapsed;
        } else {
            g_balance.pos_i *= (g_balance.origin_trust > 0.35f) ? 0.970f : 0.900f;
        }
        if (g_balance.pos_i >  0.35f) g_balance.pos_i =  0.35f;
        if (g_balance.pos_i < -0.35f) g_balance.pos_i = -0.35f;

        float pos_w = 0.0f;
        float i_w = 0.0f;
        if (origin_soft && !settle && !brake_zone && far_field) {
            pos_w = 1.0f;
            i_w = 0.60f;
        }

        float v_ref = 0.0f;
        if (brake_zone || launch_recover || startup_hold) {
            v_ref = 0.0f;
        } else if (pos_w > 0.05f || (far_field && !settle)) {
            v_ref = -0.38f * pos_err - 0.07f * i_w * g_balance.pos_i;
            float vmax = thrash_hard ? 0.10f : 0.18f;
            if (v_ref >  vmax) v_ref =  vmax;
            if (v_ref < -vmax) v_ref = -vmax;
        }

        float a_damp = acc_unbiased;
        if (impact && fabsf(a_damp) > 2.0f) a_damp = 0.0f;
        if (a_damp >  3.0f) a_damp =  3.0f;
        if (a_damp < -3.0f) a_damp = -3.0f;

        float pitch_abs = fabsf(pitch);
        float lean_scale = 1.0f;
        if (pitch_abs > 0.10f) lean_scale = 0.50f;
        if (pitch_abs > 0.16f) lean_scale = 0.20f;
        if (pitch_abs > 0.22f) lean_scale = 0.0f;
        if (g_balance.side_support_balance < 0.55f) lean_scale *= 0.50f;

        int recover_coast = (launch_recover && g_balance.run_time > 13.0f &&
                             !airborne && !impact && support_conf > 0.45f &&
                             g_balance.side_support_balance > 0.58f &&
                             fabsf(g_balance.vel) > 0.05f && fabsf(g_balance.vel) < 1.10f);

        if (startup_hold || (launch_recover && !recover_coast)) {
            // 单独恢复阶段: 只做纯平衡。
            g_balance.neutral = 0.0f;
            g_balance.vel *= 0.96f;
            g_balance.vel_ff *= 0.96f;
            g_balance.leg_swing *= 0.90f;
        } else if (recover_coast) {
            g_balance.neutral = -0.030f * g_balance.vel - 0.004f * a_damp;
            if (g_balance.neutral >  0.011f) g_balance.neutral =  0.011f;
            if (g_balance.neutral < -0.011f) g_balance.neutral = -0.011f;
        } else if (brake_zone) {
            // 超过 0.5 m: 只使用轮速刹车，沿用 v72 风格。
            float v_br = s_brake_v;
            float cand = 0.0f;
            if (fabsf(v_br) > WHEEL_V_DEADZONE) {
                cand = (v_br > 0.0f) ? 1.0f : -1.0f;
            }
            if (g_balance.brake_v_sign == 0.0f && cand != 0.0f) {
                g_balance.brake_v_sign = cand;
                g_balance.brake_v_hold = 0.0f;
            } else if (cand != 0.0f && cand * g_balance.brake_v_sign < 0.0f) {
                g_balance.brake_v_hold += elapsed;
                if (g_balance.brake_v_hold > 0.30f) {
                    g_balance.brake_v_sign = cand;
                    g_balance.brake_v_hold = 0.0f;
                }
            } else {
                g_balance.brake_v_hold = 0.0f;
                if (cand != 0.0f) g_balance.brake_v_sign = cand;
            }

            if (wheel_odom_ok && !thrash_hard && fabsf(v_br) > WHEEL_V_DEADZONE) {
                g_balance.neutral = -0.18f * v_br - 0.012f * a_damp;
            } else if (g_balance.brake_v_sign != 0.0f && !thrash_hard &&
                       fabsf(v_br) > WHEEL_V_DEADZONE) {
                float mag = fabsf(v_br);
                if (mag < 0.10f) mag = 0.10f;
                if (mag > 0.85f) mag = 0.85f;
                g_balance.neutral = -0.14f * g_balance.brake_v_sign * mag;
#if SIMPLE_HOLD_AT_POINT
            } else {
                /* 保持当前点: 只靠上面两个速度归零分支停车，不再追原点。 */
                g_balance.neutral = 0.0f;
            }
#else
            } else if (g_balance.brake_dir != 0.0f && (fabsf(g_balance.soft_s) > 0.12f || odom_assist) &&
                       abs_away > 0.50f) {
                float ret = 0.012f;
                if (odom_assist) ret = 0.018f;
                if (abs_away > 1.20f) ret = 0.022f;
                g_balance.neutral = -ret * g_balance.brake_dir;
            } else {
                g_balance.neutral = 0.0f;
            }
#endif
            if (g_balance.neutral >  0.028f) g_balance.neutral =  0.028f;
            if (g_balance.neutral < -0.028f) g_balance.neutral = -0.028f;
        } else if (settle && near_band) {
            if (vel_control_ok && !thrash_hard &&
                fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float kv = vel_control_imu ? 0.09f : 0.14f;
                g_balance.neutral = -kv * vel_for_control - 0.012f * a_damp;
            } else {
                g_balance.neutral = 0.0f;
                g_balance.vel *= 0.95f;
                g_balance.vel_ff *= 0.95f;
            }
        } else if (far_field || outbound || capture) {
#if SIMPLE_HOLD_AT_POINT
            /* 保持当前点: 不追原点，只靠下方速度归零项停车。 */
            g_balance.neutral = 0.0f;
#else
            float dir = 0.0f;
            if (fabsf(hold_pos) > 0.08f) {
                dir = (hold_pos > 0.0f) ? 1.0f : -1.0f;
                g_balance.brake_dir = dir;
            } else if (g_balance.brake_dir != 0.0f) {
                dir = g_balance.brake_dir;
            }
            float ret = 0.010f;
            if (abs_hold > 0.25f) ret = 0.014f;
            if (abs_hold > 0.35f) ret = 0.018f;
            if (coast_path_assist && abs_hold > 0.18f) ret += 0.004f;
            g_balance.neutral = (dir != 0.0f) ? (-ret * dir) : 0.0f;
#endif
            if (vel_control_ok && !thrash_hard &&
                fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float v_err = vel_for_control - v_ref;
                if (fabsf(v_err) < WHEEL_V_DEADZONE) v_err = 0.0f;
                float kv = vel_control_imu ? 0.06f : 0.10f;
                g_balance.neutral += -kv * v_err - 0.008f * pos_err;
                if (i_w > 0.0f) g_balance.neutral += -0.003f * i_w * g_balance.pos_i;
            }
        } else if (homebound) {
            if (vel_control_ok && !thrash_soft &&
                fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float kv = vel_control_imu ? 0.08f : 0.12f;
                g_balance.neutral = -kv * vel_for_control - 0.012f * a_damp
                            - 0.006f * pos_err;
            } else {
                g_balance.neutral = 0.0f;
            }
        } else if (origin_soft) {
            float v_err = vel_for_control - v_ref;
            if (fabsf(v_err) < WHEEL_V_DEADZONE) v_err = 0.0f;
            g_balance.neutral = (0.024f * v_ref - 0.12f * v_err
                         - 0.012f * pos_err
                         - 0.003f * i_w * g_balance.pos_i
                         - 0.014f * a_damp);
        } else if (supported_coast) {
            /* 轮里程计正在打滑，但惯性速度仍能给出可用刹车方向。
             * 这里保持很弱: 它是阻尼，不是位置保持。 */
            g_balance.neutral = -0.032f * g_balance.vel - 0.006f * a_damp;
            if (g_balance.neutral >  0.014f) g_balance.neutral =  0.014f;
            if (g_balance.neutral < -0.014f) g_balance.neutral = -0.014f;
        } else {
            if (vel_control_ok && !thrash_hard &&
                fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float kv = vel_control_imu ? 0.055f : 0.08f;
                g_balance.neutral = -kv * vel_for_control - 0.012f * a_damp;
            } else {
                g_balance.neutral = 0.0f;
            }
        }

        g_balance.neutral *= lean_scale;
        if (g_balance.neutral >  0.034f) g_balance.neutral =  0.034f;
        if (g_balance.neutral < -0.034f) g_balance.neutral = -0.034f;
        g_balance.drive_tau_bias = 0.0f;

        float leg_ref = 0.0f;
        if (startup_hold || (launch_recover && !recover_coast) || airborne ||
            support_conf < 0.26f ||
            g_balance.side_support_balance < 0.50f ||
            pitch_abs > 0.18f) {
            leg_ref = -0.80f * g_balance.leg_swing;
        } else if (recover_coast) {
            leg_ref = -0.010f * g_balance.vel;
        } else if (brake_zone) {
            if (vel_control_ok && !thrash_hard &&
                fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float kvl = vel_control_imu ? 0.016f : 0.026f;
                leg_ref = -kvl * vel_for_control;
            } else if (g_balance.brake_v_sign != 0.0f && !thrash_hard &&
                       fabsf(s_brake_v) > WHEEL_V_DEADZONE) {
                float mag = fabsf(s_brake_v);
                if (mag < 0.10f) mag = 0.10f;
                if (mag > 0.80f) mag = 0.80f;
                leg_ref = -0.020f * g_balance.brake_v_sign * mag;
            } else if (g_balance.brake_dir != 0.0f && (fabsf(g_balance.soft_s) > 0.12f || odom_assist) &&
                       abs_away > 0.50f) {
                float lret = 0.012f;
                if (odom_assist) lret = 0.018f;
                if (abs_away > 1.20f) lret = 0.022f;
                leg_ref = -lret * g_balance.brake_dir;
            } else {
                leg_ref = -0.75f * g_balance.leg_swing;
            }
        } else if (settle && near_band) {
            if (vel_control_ok && fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float kvl = vel_control_imu ? 0.016f : 0.026f;
                leg_ref = -kvl * vel_for_control;
            } else {
                leg_ref = -0.65f * g_balance.leg_swing;
            }
        } else if (far_field || outbound || capture) {
            float dir = 0.0f;
            if (fabsf(hold_pos) > 0.08f) {
                dir = (hold_pos > 0.0f) ? 1.0f : -1.0f;
            } else {
                dir = g_balance.brake_dir;
            }
            float lret = 0.010f;
            if (abs_hold > 0.25f) lret = 0.014f;
            if (abs_hold > 0.35f) lret = 0.018f;
            if (coast_path_assist && abs_hold > 0.18f) lret += 0.003f;
            leg_ref = (dir != 0.0f) ? (-lret * dir) : 0.0f;
            if (vel_control_ok && fabsf(vel_for_control) > WHEEL_V_DEADZONE) {
                float kvl = vel_control_imu ? 0.007f : 0.012f;
                leg_ref += -kvl * vel_for_control;
            }
        } else if (origin_soft && fabsf(pos_err) > 0.03f) {
            leg_ref = -0.026f * pos_err
                      - 0.014f * v_ref
                      - 0.012f * vel_for_control
                      - 0.003f * i_w * g_balance.pos_i;
        } else if (supported_coast) {
            leg_ref = -0.014f * g_balance.vel;
        } else if (vel_control_ok && fabsf(vel_for_control) > 0.10f && !thrash_hard) {
            float kvl = vel_control_imu ? 0.011f : 0.018f;
            leg_ref = -kvl * vel_for_control;
        } else {
            leg_ref = -0.55f * g_balance.leg_swing;
        }
        if (leg_ref >  0.024f) leg_ref =  0.024f;
        if (leg_ref < -0.024f) leg_ref = -0.024f;
        float max_step = 0.00220f * (elapsed / speed_period);
        if (max_step > 0.0045f) max_step = 0.0045f;
        float leg_delta = leg_ref - g_balance.leg_swing;
        if (leg_delta >  max_step) leg_delta =  max_step;
        if (leg_delta < -max_step) leg_delta = -max_step;
        g_balance.leg_swing += leg_delta;

        // 双电机腿虚拟坐标:
        // 摆角是差动电机角，伸长量是共模电机角。
        // 正伸长量对应两侧车轮向下运动。
        float ext_ref = 0.0f;
        // 冲击或弹跳尖峰时轻微软腿，减少单轮跳动。
        if (impact || acc_mag > 95.0f) {
            ext_ref = 0.006f;
        } else if (g_balance.side_support_balance < 0.70f && support_conf > 0.35f) {
            ext_ref = 0.0025f;
        }
        if (ext_ref >  0.012f) ext_ref =  0.012f;
        if (ext_ref < -0.012f) ext_ref = -0.012f;

        float side_ref = 0.0f;
        // 单侧卸载时用差动伸长量找平机体。
        if (!airborne && !impact && support_conf > 0.30f &&
            g_balance.side_support_balance < 0.78f) {
            float left_load_side = 0.5f * (fabsf(lf_tau) + fabsf(lr_tau));
            float right_load_side = 0.5f * (fabsf(rf_tau) + fabsf(rr_tau));
            // 轻载侧稍微伸长，让两侧轮子重新接地。
            side_ref = 0.007f * (right_load_side - left_load_side) /
                       (left_load_side + right_load_side + 1e-3f);
        }
        if (side_ref >  0.006f) side_ref =  0.006f;
        if (side_ref < -0.006f) side_ref = -0.006f;
        if (airborne || impact) {
            side_ref = 0.0f;
        }

        float ext_step = 0.00035f * (elapsed / speed_period);
        if (ext_step > 0.0010f) ext_step = 0.0010f;
        float ext_delta = ext_ref - g_balance.leg_extend;
        if (ext_delta >  ext_step) ext_delta =  ext_step;
        if (ext_delta < -ext_step) ext_delta = -ext_step;
        g_balance.leg_extend += ext_delta;

        float side_step = 0.00030f * (elapsed / speed_period);
        if (side_step > 0.0009f) side_step = 0.0009f;
        float side_delta = side_ref - g_balance.leg_side_extend;
        if (side_delta >  side_step) side_delta =  side_step;
        if (side_delta < -side_step) side_delta = -side_step;
        g_balance.leg_side_extend += side_delta;
        *spd = g_balance.vel;
        *ex = g_balance.pos;
    }

    // ---- 偏航环，50 Hz ----
    g_balance.yaw_elapsed += dt_f;
    const float yaw_period = 1.0f / (float)YAW_LOOP_HZ;
    if (g_balance.yaw_elapsed >= yaw_period) {
        g_balance.yaw_elapsed = 0.0f;
        float yaw_cmd = 3.6f * yaw_ang + 2.2f * yaw_rate;
        float yaw_lim = 26.0f;
        float yaw_scale = 1.0f;
        if (g_balance.run_time > 13.0f) {
            if (g_balance.wheel_thrash > 1.15f || fabsf(g_balance.wheel_v_filt) > 3.0f) {
                yaw_scale *= 0.55f;
                yaw_lim = 14.0f;
            }
            if (g_balance.side_support_balance < 0.58f) {
                yaw_scale *= 0.65f;
                yaw_lim = (yaw_lim < 11.0f) ? yaw_lim : 11.0f;
            }
            if (g_balance.airborne_dbg > 0.5f || g_balance.impact_dbg > 0.5f || g_balance.support_dbg < 0.40f) {
                yaw_scale *= 0.35f;
                yaw_lim = (yaw_lim < 7.0f) ? yaw_lim : 7.0f;
            }
        }
        yaw_cmd *= yaw_scale;
        if (yaw_cmd >  yaw_lim) yaw_cmd =  yaw_lim;
        if (yaw_cmd < -yaw_lim) yaw_cmd = -yaw_lim;
        g_balance.yaw_held = yaw_cmd;
    }
    *yaw = g_balance.yaw_held;

    // ---- 腿摆: 里程计位置/速度 -> sw ----
    float dsw = g_balance.leg_swing;
    float dsw_limit = (g_balance.side_support_balance < 0.62f) ? 0.014f : 0.030f;
    if (dsw >  dsw_limit) dsw =  dsw_limit;
    if (dsw < -dsw_limit) dsw = -dsw_limit;
    *sw = balance_sw_nom() + dsw;

    // ---- 俯仰 PD ----
    float p_err = pitch - g_balance.neutral;
    float Kp = 325.0f;
    float Kd = 58.0f;
    *tau = Kp * p_err + Kd * pitch_rate + g_balance.drive_tau_bias;

    // 牵引保护: 轮子抖动/空转时缩小共模驱动。
    // 角速度阻尼保持原样，用来维持平衡。
    {
        float abs_wh = fabsf(g_balance.wheel_v_filt);
        if (abs_wh > 1.80f) {
            float damp = Kd * pitch_rate;
            float drive = *tau - damp;
            float scale = 1.80f / (abs_wh + 1e-3f);
            if (scale < 0.25f) scale = 0.25f;
            if (scale > 1.0f) scale = 1.0f;
            // FIR 抖动残差较大时进一步削弱驱动。
            if (g_balance.wheel_thrash > 0.70f) {
                scale *= 0.70f;
                if (scale < 0.20f) scale = 0.20f;
            }
            *tau = damp + scale * drive;
        }
    }

    if (*tau >  200.0f) *tau =  200.0f;
    if (*tau < -200.0f) *tau = -200.0f;

    {
        float tau_max = 145.0f;
        float tau_rate = 4200.0f;
        if (g_balance.run_time < 0.85f) {
            tau_max = 68.0f;
            tau_rate = 950.0f;
        }
        if (g_balance.wheel_thrash > 0.70f || fabsf(g_balance.wheel_v_filt) > 2.0f) {
            tau_max = (tau_max < 96.0f) ? tau_max : 96.0f;
            tau_rate = (tau_rate < 2100.0f) ? tau_rate : 2100.0f;
        }
        if (g_balance.impact_dbg > 0.5f || g_balance.airborne_dbg > 0.5f || g_balance.support_dbg < 0.45f) {
            tau_max = (tau_max < 54.0f) ? tau_max : 54.0f;
            tau_rate = (tau_rate < 750.0f) ? tau_rate : 750.0f;
        }
        if (*tau >  tau_max) *tau =  tau_max;
        if (*tau < -tau_max) *tau = -tau_max;
        float max_delta = tau_rate * dt_f;
        float delta = *tau - g_balance.tau_cmd;
        if (delta >  max_delta) delta =  max_delta;
        if (delta < -max_delta) delta = -max_delta;
        g_balance.tau_cmd += delta;
        *tau = g_balance.tau_cmd;
    }
}
