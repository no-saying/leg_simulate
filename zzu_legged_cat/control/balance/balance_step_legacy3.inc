/* balance_step_legacy3.inc - 旧速度环路径: 位置/保持/腿摆/差动找平
 * 仅 USE_SIMPLE_THREE_LOOP=0 时编译。 */
        if (g_balance.ekf_s >  1.2f) g_balance.ekf_s =  1.2f;
        if (g_balance.ekf_s < -1.2f) g_balance.ekf_s = -1.2f;

        // 抓地锁存下的路径修正: 轮子平稳时，编码器弧长可信。
        // 将扩展卡尔曼路径轻微拉向轮积分路径，避免错误符号的饱和路径把机器人推过原点。
        if (grip_lock && !startup_hold && g_balance.wheel_thrash < 0.40f) {
            float wheel_ds_path = wheel_ds;
            if (wheel_ds_path >  0.015f) wheel_ds_path =  0.015f;
            if (wheel_ds_path < -0.015f) wheel_ds_path = -0.015f;
            // 只在抖动平稳时，把路径和轮弧长混合。
            g_balance.ekf_s += 0.35f * wheel_ds_path;
            if (fabsf(g_balance.wheel_v_filt) < 0.18f && fabsf(g_balance.vel) < 0.18f &&
                fabsf(g_balance.imu_acc_filt - g_balance.acc_bias) < 1.0f) {
                g_balance.vel *= 0.75f;
            }
        }
        // 低延迟速度跟踪，优先用轮速并带死区。
        // 最终控制速度会在保持段只由轮里程计重新生成；
        // 这里的预跟踪主要供软诊断和扩展卡尔曼辅助使用。
        {
            float v_obs = 0.0f;
#if !USE_IMU_ONLY_ODOM
            if (!airborne && !impact &&
                g_balance.wheel_thrash < 0.55f && fabsf(g_balance.wheel_v_filt) < 1.40f) {
                v_obs = wheel_v_limited;
            } else if (!airborne && !impact && g_balance.fir_fill >= FIR_N &&
                       fabsf(g_balance.wheel_fir) < 1.40f && g_balance.wheel_thrash < 1.20f) {
                v_obs = g_balance.wheel_fir;
            }
#endif
            if (fabsf(v_obs) < WHEEL_V_DEADZONE) v_obs = 0.0f;
            float a_ff = elapsed / (0.010f + elapsed);
            if (a_ff > 1.0f) a_ff = 1.0f;
            g_balance.vel_ff += a_ff * (v_obs - g_balance.vel_ff);
        }

        // 软路径 + 保持逻辑，v104 基线，不使用额外里程计辅助。
        // ------------------------------------------------------------
        // 硬规则:
        //  1. 不用抖动惯性项，也不从抖动残差里发明方向。
        //  2. 启动和俯仰恢复阶段只做纯平衡。
        //  3. |x| > 0.5 时优先刹车。
        //  4. 平面里程计 ox/oy 只积分平稳或轻度有限脉冲响应滤波的轮位移。
        g_balance.soft_s_prev = g_balance.soft_s;

        int long_slip_guard = (g_balance.run_time > 13.0f && g_balance.slip_score >= 5);
        int pitch_recover;
        if (long_slip_guard) {
            pitch_recover = (fabsf(pitch) > 0.12f);
        } else {
            pitch_recover = (fabsf(pitch) > 0.20f ||
                             (fabsf(pitch) > 0.16f && support_conf < 0.35f));
        }
        int launch_recover = (g_balance.run_time < 1.00f) || pitch_recover;

        float wheel_odom_v = 0.0f;
        int wheel_odom_ok = 0;
        int wheel_odom_calm = 0;
#if !USE_IMU_ONLY_ODOM
        {
            float v_w = 0.0f;
            int calm = (!airborne && !impact &&
                        g_balance.wheel_thrash < 0.55f &&
                        fabsf(g_balance.wheel_v_filt) < 1.40f &&
                        wheel_trust > 0.10f);
            int fir_ok = (!airborne && !impact &&
                          g_balance.fir_fill >= FIR_N &&
                          g_balance.wheel_thrash < 1.20f &&
                          fabsf(g_balance.wheel_fir) < 1.40f);
            if (calm) {
                v_w = g_balance.wheel_v_filt;
                if (fir_ok) v_w = 0.80f * v_w + 0.20f * g_balance.wheel_fir;
                wheel_odom_ok = 1;
                wheel_odom_calm = 1;
            } else if (fir_ok && fabsf(g_balance.wheel_fir) > WHEEL_FIR_DEADZONE) {
                /* 只使用轻度有限脉冲响应滤波，但仍然是真实轮速低频分量。 */
                v_w = g_balance.wheel_fir;
                wheel_odom_ok = 1;
            }
            if (wheel_odom_ok) {
                if (fabsf(v_w) < (wheel_odom_calm ? WHEEL_V_DEADZONE : WHEEL_FIR_DEADZONE))
                    v_w = 0.0f;
                if (v_w >  1.50f) v_w =  1.50f;
                if (v_w < -1.50f) v_w = -1.50f;
            }
            wheel_odom_v = v_w;

        }
#endif

        int origin_odom_ok = (wheel_odom_ok && wheel_odom_calm &&
                              !slip_like && !airborne && !impact &&
                              support_conf > 0.50f &&
                              g_balance.side_support_balance > 0.62f &&
                              wheel_trust > 0.45f && pos_trust > 0.50f &&
                              g_balance.wheel_thrash < 0.42f &&
                              fabsf(g_balance.wheel_v_filt) < 1.15f);
        if (origin_odom_ok) {
            g_balance.origin_trust += 2.5f * elapsed;
            if (g_balance.origin_trust > 1.0f) g_balance.origin_trust = 1.0f;
        } else {
            g_balance.origin_trust -= 5.0f * elapsed;
            if (g_balance.origin_trust < 0.0f) g_balance.origin_trust = 0.0f;
        }

        /* 只从平稳轮速更新粘滞运动方向。 */
        if (wheel_odom_ok && wheel_odom_calm &&
            fabsf(wheel_odom_v) > WHEEL_V_DEADZONE) {
            float cand = (wheel_odom_v > 0.0f) ? 1.0f : -1.0f;
            if (g_balance.motion_dir == 0.0f) {
                g_balance.motion_dir = cand;
                g_balance.motion_hold = 0.0f;
            } else if (cand * g_balance.motion_dir < 0.0f) {
                g_balance.motion_hold += elapsed;
                if (g_balance.motion_hold > 0.50f) {
                    g_balance.motion_dir = cand;
                    g_balance.motion_hold = 0.0f;
                }
            } else {
                g_balance.motion_hold = 0.0f;
                g_balance.motion_dir = cand;
            }
            float a_hv = elapsed / (0.06f + elapsed);
            if (a_hv > 1.0f) a_hv = 1.0f;
            g_balance.hold_v += a_hv * (wheel_odom_v - g_balance.hold_v);
            g_balance.hold_v_age = 0.0f;
        } else {
            g_balance.hold_v_age += elapsed;
            if (g_balance.hold_v_age > 0.35f) {
                g_balance.hold_v *= (1.0f - 0.50f * elapsed);
                if (fabsf(g_balance.hold_v) < 0.03f) g_balance.hold_v = 0.0f;
            }
        }

        float vel_for_control = wheel_odom_v;
        int vel_control_ok = wheel_odom_ok;
        int vel_control_imu = 0;
        if (!vel_control_ok && !launch_recover && !startup_hold &&
            !airborne && !impact && support_conf > 0.45f &&
            g_balance.side_support_balance > 0.55f && fabsf(g_balance.vel) > 0.10f &&
            fabsf(g_balance.vel) < 1.10f && g_balance.wheel_thrash < 4.50f) {
            vel_for_control = g_balance.vel;
            if (vel_for_control >  0.75f) vel_for_control =  0.75f;
            if (vel_for_control < -0.75f) vel_for_control = -0.75f;
            vel_control_ok = 1;
            vel_control_imu = 1;
        }
        {
            float a_ff = elapsed / (0.012f + elapsed);
            if (a_ff > 1.0f) a_ff = 1.0f;
            float v_track = wheel_odom_ok ? wheel_odom_v : 0.0f;
            g_balance.vel_ff += a_ff * (v_track - g_balance.vel_ff);
            if (!wheel_odom_ok) g_balance.vel_ff *= 0.92f;
        }

        if (launch_recover || startup_hold || airborne) {
            g_balance.soft_s *= 0.88f;
            g_balance.ekf_s  *= 0.90f;
            g_balance.pos_i  *= 0.85f;
            g_balance.away_s *= 0.92f;
            g_balance.coast_t = 0.0f;
            g_balance.hold_v *= 0.70f;
            if (g_balance.run_time < 13.0f || airborne) g_balance.coast_s *= 0.90f;
        } else if (wheel_odom_ok) {
            float ds = wheel_odom_v * elapsed;
            if (wheel_odom_calm) {
                if (ds >  0.030f) ds =  0.030f;
                if (ds < -0.030f) ds = -0.030f;
            } else {
                if (ds >  0.012f) ds =  0.012f;
                if (ds < -0.012f) ds = -0.012f;
            }
            if (impact) ds *= 0.35f;
            g_balance.soft_s += ds;
            g_balance.soft_s *= 0.99997f;
            g_balance.coast_t *= 0.90f;
        } else {
            /* 抖动时冻结带符号的控制路径，沿用 v72 思路。 */
            g_balance.soft_s *= 0.9998f;
            if (g_balance.wheel_thrash > 0.80f || fabsf(g_balance.wheel_v_filt) > 2.00f) {
                g_balance.coast_t += elapsed;
                if (g_balance.coast_t > 0.35f) g_balance.away_s += 0.50f * elapsed;
            } else {
                g_balance.coast_t *= 0.88f;
            }
        }
        if (g_balance.soft_s >  6.0f) g_balance.soft_s =  6.0f;
        if (g_balance.soft_s < -6.0f) g_balance.soft_s = -6.0f;

        if (g_balance.run_time > 13.0f && !airborne && !impact &&
            support_conf > 0.45f && g_balance.side_support_balance > 0.58f &&
            g_balance.origin_trust < 0.35f && fabsf(g_balance.vel) > 0.08f && fabsf(g_balance.vel) < 1.20f) {
            float ds_c = g_balance.vel * elapsed;
            if (ds_c >  0.010f) ds_c =  0.010f;
            if (ds_c < -0.010f) ds_c = -0.010f;
            g_balance.coast_s += ds_c;
            g_balance.coast_s *= 0.9995f;
        } else if (g_balance.origin_trust > 0.55f || startup_hold || airborne || impact) {
            g_balance.coast_s *= 0.94f;
        } else {
            g_balance.coast_s *= 0.995f;
        }
        if (g_balance.coast_s >  0.80f) g_balance.coast_s =  0.80f;
        if (g_balance.coast_s < -0.80f) g_balance.coast_s = -0.80f;

        /* 平面里程计 ox/oy 只用于地图和日志，不作为控制位置。
         * 只要不是离地/冲击，就按衰减后的轮速低频积分。
         * 这样即使 soft_s 冻结，二维地图仍能继续显示。 */
#if !USE_IMU_ONLY_ODOM
        if (!launch_recover && !startup_hold && !airborne) {
            float v_map = 0.0f;
            float w_map = 0.0f;
            if (wheel_odom_ok) {
                v_map = wheel_odom_v;
                w_map = wheel_odom_calm ? 1.0f : 0.55f;
            } else if (g_balance.fir_fill >= FIR_N && !impact && !slip_like &&
                       g_balance.wheel_thrash < 0.90f && fabsf(g_balance.wheel_v_filt) < 1.80f) {
                v_map = g_balance.wheel_fir;
                if (fabsf(v_map) < WHEEL_FIR_DEADZONE) v_map = 0.0f;
                if (v_map >  1.20f) v_map =  1.20f;
                if (v_map < -1.20f) v_map = -1.20f;
                w_map = (g_balance.wheel_thrash > 0.60f) ? 0.30f : 0.60f;
            }
            if (w_map > 0.0f && fabsf(v_map) >= WHEEL_V_DEADZONE) {
                float ds_m = w_map * v_map * elapsed;
                if (ds_m >  0.025f) ds_m =  0.025f;
                if (ds_m < -0.025f) ds_m = -0.025f;
                if (impact) ds_m *= 0.25f;
                float cy_s = cosf(yaw_ang), sy_s = sinf(yaw_ang);
                g_balance.odom_x += ds_m * cy_s;
                g_balance.odom_y += ds_m * sy_s;
            }
        }
#endif
        if (g_balance.odom_x >  8.0f) g_balance.odom_x =  8.0f;
        if (g_balance.odom_x < -8.0f) g_balance.odom_x = -8.0f;
        if (g_balance.odom_y >  8.0f) g_balance.odom_y =  8.0f;
        if (g_balance.odom_y < -8.0f) g_balance.odom_y = -8.0f;

        {
            float cand = fabsf(g_balance.soft_s);
            if (cand > g_balance.away_s) g_balance.away_s = cand;
            if (!launch_recover && !startup_hold && !airborne &&
                wheel_odom_ok && wheel_odom_calm &&
                fabsf(wheel_odom_v) > WHEEL_V_DEADZONE) {
                g_balance.away_s += 0.90f * fabsf(wheel_odom_v) * elapsed;
            }
            if (g_balance.away_s > 6.0f) g_balance.away_s = 6.0f;
        }

        /* brake_dir 只来自真实软路径，不从抖动里推断。 */
        if (fabsf(g_balance.soft_s) > 0.10f) {
            g_balance.brake_dir = (g_balance.soft_s > 0.0f) ? 1.0f : -1.0f;
        }

        /* pos 优先使用软路径。只有持续可信的轮里程计才允许闭合原点环。
         * 噪声/打滑轮里程计仍记录到 ox/oy，但不能产生持续的位置指令。 */
        float odom_forward = g_balance.odom_x * cosf(yaw_ang) + g_balance.odom_y * sinf(yaw_ang);
        g_balance.odom_forward_dbg = odom_forward;
        int odom_assist = 0;
        if (grip_lock && wheel_odom_calm && wheel_trust > 0.20f) {
            g_balance.pos = 0.25f * g_balance.ekf_s + 0.75f * g_balance.soft_s;
        } else {
            g_balance.pos = g_balance.soft_s;
        }
        if (g_balance.origin_trust > 0.75f && fabsf(odom_forward) > 0.18f) {
            odom_assist = 1;
        }
        if (odom_assist) {
            float od = odom_forward;
            if (od >  0.90f) od =  0.90f;
            if (od < -0.90f) od = -0.90f;
            float blend = 0.25f * g_balance.origin_trust;
            g_balance.pos = (1.0f - blend) * g_balance.pos + blend * od;
            if (fabsf(od) > g_balance.away_s) g_balance.away_s = fabsf(od);
            if (fabsf(g_balance.soft_s) < 0.10f) {
                g_balance.brake_dir = (od > 0.0f) ? 1.0f : -1.0f;
            }
        }

        int imu_path_assist = (!odom_assist && g_balance.run_time > 13.0f &&
                               !startup_hold && !airborne && !impact &&
                               support_conf > 0.48f &&
                               g_balance.side_support_balance > 0.58f &&
                               fabsf(g_balance.ekf_s) > 0.12f &&
                               fabsf(g_balance.ekf_s) < 1.15f &&
                               g_balance.origin_trust < 0.35f);
        if (imu_path_assist) {
            float is = g_balance.ekf_s;
            if (is >  0.45f) is =  0.45f;
            if (is < -0.45f) is = -0.45f;
            g_balance.pos = 0.82f * g_balance.pos + 0.18f * is;
            if (fabsf(is) > g_balance.away_s) g_balance.away_s = fabsf(is);
            if (fabsf(g_balance.soft_s) < 0.10f) {
                g_balance.brake_dir = (is > 0.0f) ? 1.0f : -1.0f;
            }
        }

        int coast_path_assist = (!odom_assist && g_balance.run_time > 13.0f &&
                                 !startup_hold && !airborne && !impact &&
                                 support_conf > 0.45f &&
                                 g_balance.side_support_balance > 0.58f &&
                                 g_balance.origin_trust < 0.35f &&
                                 fabsf(g_balance.coast_s) > 0.12f);
        if (coast_path_assist) {
            float cs = g_balance.coast_s;
            if (cs >  0.55f) cs =  0.55f;
            if (cs < -0.55f) cs = -0.55f;
            g_balance.pos = 0.55f * g_balance.pos + 0.45f * cs;
            if (fabsf(cs) > g_balance.away_s) g_balance.away_s = fabsf(cs);
            if (fabsf(g_balance.soft_s) < 0.10f) {
                g_balance.brake_dir = (cs > 0.0f) ? 1.0f : -1.0f;
            }
        }

        int thrash_hard = (g_balance.wheel_thrash > 0.95f || fabsf(g_balance.wheel_v_filt) > 2.80f);
        int thrash_soft = (g_balance.wheel_thrash > 0.55f || fabsf(g_balance.wheel_v_filt) > 1.80f);
        int supported_coast = (!startup_hold && !launch_recover &&
                               !airborne && !impact &&
                               support_conf > 0.50f &&
                               g_balance.side_support_balance > 0.58f &&
                               fabsf(pitch) < 0.16f &&
                               fabsf(g_balance.vel) > 0.12f && fabsf(g_balance.vel) < 0.95f);
        int origin_ok = (!startup_hold && !launch_recover &&
                         !airborne && !impact &&
                         support_conf > 0.28f &&
                         g_balance.side_support_balance > 0.45f);
        int origin_soft = origin_ok && g_balance.origin_trust > 0.55f && !thrash_hard;

        float abs_pos_raw = fabsf(g_balance.pos);
        float abs_prev = fabsf(g_balance.soft_s_prev);
        float path_grow = (abs_pos_raw - abs_prev) / (elapsed > 1e-4f ? elapsed : 0.01f);
        float abs_away = g_balance.away_s;
        if (abs_pos_raw > abs_away) abs_away = abs_pos_raw;

        float hold_pos = g_balance.pos;
        float abs_hold = fabsf(hold_pos);

        if (!launch_recover && !startup_hold &&
            g_balance.origin_trust > 0.55f &&
            (abs_away > 0.50f || abs_hold > 0.50f)) {
            g_balance.brake_latched = 1;
        } else if (g_balance.origin_trust < 0.20f ||
                   (abs_away < 0.38f && abs_hold < 0.38f &&
                    fabsf(wheel_odom_v) < 0.15f)) {
            g_balance.brake_latched = 0;
            g_balance.brake_v_sign = 0.0f;
            g_balance.brake_v_hold = 0.0f;
        }
        int brake_zone = (!launch_recover && !startup_hold && g_balance.origin_trust > 0.35f) &&
                         (g_balance.brake_latched || abs_away > 0.50f || abs_hold > 0.50f);

        int far_field = (!brake_zone && !launch_recover && !startup_hold &&
                         abs_hold > 0.18f);
        int near_band = (!brake_zone && !launch_recover && !startup_hold &&
                         abs_hold < 0.14f);
        int outbound  = (!brake_zone && !launch_recover && abs_hold > 0.16f &&
                         (path_grow > 0.02f || abs_hold > 0.28f));
        int homebound = (!brake_zone && !launch_recover && abs_hold > 0.10f &&
                         path_grow < -0.04f);

        if (brake_zone) {
            if (g_balance.capture_t < 1.0f) g_balance.capture_t = 1.0f;
        } else if (g_balance.origin_trust > 0.35f && (outbound || far_field)) {
            if (g_balance.capture_t < 1.4f) g_balance.capture_t = 1.4f;
        } else if (g_balance.origin_trust > 0.35f && (homebound || near_band)) {
            if (g_balance.capture_t < 0.8f) g_balance.capture_t = 0.8f;
        }
        if (g_balance.capture_t > 0.0f) {
            g_balance.capture_t -= elapsed;
            if (g_balance.capture_t < 0.0f) g_balance.capture_t = 0.0f;
        }
        int capture = (g_balance.origin_trust > 0.25f && g_balance.capture_t > 0.0f);

        int settle_raw = (!startup_hold && !launch_recover &&
                          !airborne && !impact && !brake_zone &&
                          support_conf > 0.28f &&
                          abs_hold < 0.13f &&
                          !thrash_hard &&
                          fabsf(wheel_odom_v) < 0.12f);
        if (settle_raw) {
            g_balance.settle_s += elapsed;
            if (g_balance.settle_s > 2.5f) g_balance.settle_s = 2.5f;
        } else {
            g_balance.settle_s -= 1.2f * elapsed;
            if (g_balance.settle_s < 0.0f) g_balance.settle_s = 0.0f;
        }
        int settle = (g_balance.settle_s > 0.35f);

        /* 仅供观察器日志使用的阶段标签，不会改变控制分支。 */
        g_balance.return_score_dbg = 0.0f;
        if (origin_ok) {
            g_balance.return_score_dbg += abs_hold / 0.50f;
            if (fabsf(vel_for_control) > WHEEL_V_DEADZONE) g_balance.return_score_dbg += 0.5f;
            if (homebound) g_balance.return_score_dbg += 0.7f;
            if (g_balance.return_score_dbg > 2.0f) g_balance.return_score_dbg = 2.0f;
        }
        g_balance.capture_score_dbg = 0.0f;
        if (origin_ok) {
            if (brake_zone) g_balance.capture_score_dbg += 0.8f;
