/* balance_step_legacy2.inc - 旧速度环路径: 接触滞回/扩展卡尔曼/里程计信任
 * 仅 USE_SIMPLE_THREE_LOOP=0 时编译。 */

        if (raw_airborne) {
            if (g_balance.airborne_score < 8) g_balance.airborne_score++;
        } else if (g_balance.airborne_score > 0) {
            g_balance.airborne_score -= 2;
            if (g_balance.airborne_score < 0) g_balance.airborne_score = 0;
        }
        if (raw_impact) {
            g_balance.impact_score = 2;
        } else if (g_balance.impact_score > 0) {
            g_balance.impact_score--;
        }
        if (raw_slip) {
            if (g_balance.slip_score < 10) g_balance.slip_score += 1;
        } else if (g_balance.slip_score > 0) {
            // 机体有支撑时更快退出打滑状态；旧的粘滞打滑会让机器人长期卡在滑行模式。
            g_balance.slip_score -= (support_conf > 0.45f) ? 2 : 1;
            if (g_balance.slip_score < 0) g_balance.slip_score = 0;
        }
        if (raw_zupt) {
            if (g_balance.zupt_score < 8) g_balance.zupt_score++;
        } else if (g_balance.zupt_score > 0) {
            g_balance.zupt_score -= 2;
            if (g_balance.zupt_score < 0) g_balance.zupt_score = 0;
        }

        int airborne = (g_balance.airborne_score >= 5);
        int impact = (g_balance.impact_score > 0);
        int slip_like = (g_balance.slip_score >= 5);
        int zupt = (g_balance.zupt_score >= 4 && !airborne && !impact);
        int contact_mode = CONTACT_STICK;
        if (airborne) {
            contact_mode = CONTACT_AIRBORNE;
        } else if (impact) {
            contact_mode = CONTACT_IMPACT;
        } else if (zupt) {
            contact_mode = CONTACT_ZUPT;
            slip_like = 0;
        } else if (slip_like) {
            contact_mode = CONTACT_SLIP;
        }

        // 抓地恢复: 如果轮速平稳且支撑良好，即使打滑分数还没退完，
        // 机体也不应被当成自由空转。强制回到粘着状态并重新信任轮速，
        // 这能抑制长时间滑行漂移。
        // 平稳门限故意放宽一些；抖动经常偏高，仍允许轮通道逐步回到闭环。
        int calm_wheels = (fabsf(g_balance.wheel_v_filt) < 2.20f &&
                           fabsf(wheel_acc) < 16.0f &&
                           g_balance.wheel_thrash < 0.85f &&
                           support_conf > 0.30f &&
                           !airborne && !impact);
        if (calm_wheels) {
            if (g_balance.grip_score < 12) g_balance.grip_score += 3;
        } else if (g_balance.grip_score > 0) {
            g_balance.grip_score -= 1;
        }
        int grip_lock = (g_balance.grip_score >= 3);
        if (grip_lock) {
            slip_like = 0;
            if (contact_mode == CONTACT_SLIP) {
                contact_mode = CONTACT_STICK;
            }
        }

        float imu_acc_x = g_balance.imu_acc_filt;
        if (impact) {
            imu_acc_x = g_balance.acc_bias;
        }
        if (imu_acc_x >  8.0f) imu_acc_x =  8.0f;
        if (imu_acc_x < -8.0f) imu_acc_x = -8.0f;

#if !USE_IMU_ONLY_ODOM
        // 只在支撑平稳时估计加速度偏置；抖动/打滑时冻结。
        if (!airborne && !impact && !slip_like &&
            g_balance.wheel_thrash < 0.40f &&
            support_conf > 0.55f &&
            fabsf(pitch_rate) < 0.80f && fabsf(drive_tau) < 18.0f &&
            fabsf(g_balance.wheel_v_filt) < 0.45f && fabsf(g_balance.vel) < 0.35f) {
            float bias_alpha = elapsed / (1.20f + elapsed);
            g_balance.acc_bias += bias_alpha * (imu_acc_x - g_balance.acc_bias);
        }
        if (g_balance.acc_bias >  2.5f) g_balance.acc_bias =  2.5f;
        if (g_balance.acc_bias < -2.5f) g_balance.acc_bias = -2.5f;
#endif
#if USE_IMU_ONLY_ODOM
        if (!airborne && !impact && support_conf > 0.55f) {
            float bias_alpha = elapsed / (0.65f + elapsed);
            if (fabsf(pitch_rate) > 1.2f || fabsf(drive_tau) > 18.0f) {
                bias_alpha *= 0.20f;
            }
            g_balance.acc_bias += bias_alpha * (imu_acc_x - g_balance.acc_bias);
        }
        if (g_balance.acc_bias >  5.0f) g_balance.acc_bias =  5.0f;
        if (g_balance.acc_bias < -5.0f) g_balance.acc_bias = -5.0f;
#endif

        float wheel_v_limited = g_balance.wheel_v_filt;
        if (wheel_v_limited >  2.5f) wheel_v_limited =  2.5f;
        if (wheel_v_limited < -2.5f) wheel_v_limited = -2.5f;

        float wheel_trust = 1.0f;
#if USE_IMU_ONLY_ODOM
        wheel_trust = 0.0f;
#else
        if (airborne) {
            wheel_trust = 0.0f;
        } else if (zupt) {
            wheel_trust = 0.0f;
        } else if (impact) {
            wheel_trust = 0.12f;
        } else if (fabsf(g_balance.wheel_v_filt) > WHEEL_V_RUNAWAY) {
            wheel_trust = 0.0f;
        } else if (g_balance.wheel_thrash > 1.20f) {
            // 有限脉冲响应高频残差显示轮子在抖动，此时忽略编码器。
            wheel_trust = 0.0f;
        } else if (slip_like || g_balance.wheel_thrash > 0.70f) {
            wheel_trust = (fabsf(g_balance.wheel_v_filt) < 0.70f && g_balance.wheel_thrash < 0.90f) ? 0.15f : 0.03f;
        } else if (fabsf(g_balance.wheel_v_filt) < 0.90f && g_balance.wheel_thrash < 0.40f) {
            wheel_trust = 1.0f;
        } else if (fabsf(g_balance.wheel_v_filt) < 1.40f && g_balance.wheel_thrash < 0.55f) {
            wheel_trust = 0.50f;
        } else {
            wheel_trust = 0.10f;
        }
        if (wheel_trust > 0.0f) {
            float imu_for_gate = g_balance.imu_acc_filt - g_balance.acc_bias;
            if (fabsf(wheel_acc - imu_for_gate) > 18.0f && fabsf(g_balance.wheel_v_filt) > 1.4f) {
                wheel_trust *= 0.30f;
            }
        }
        if (support_conf < 0.35f) {
            wheel_trust *= support_conf / 0.35f;
        }
        // 只有抖动确实平稳时，抓地锁存才提高轮速可信度。
        if (grip_lock && g_balance.wheel_thrash < 0.40f && fabsf(g_balance.wheel_v_filt) < 1.20f) {
            if (wheel_trust < 0.80f) wheel_trust = 0.80f;
        } else if (grip_lock && g_balance.wheel_thrash >= 0.40f) {
            // 抓地分数还在，但抖动又升高了，需要降低可信度。
            wheel_trust *= 0.25f;
        }
#endif
        float pos_trust = 1.0f;
        if (airborne) {
            pos_trust = 0.0f;
        } else if (zupt) {
            pos_trust = 0.0f;
        } else if (impact) {
            pos_trust = 0.08f;
        } else if (g_balance.wheel_thrash > 1.00f || fabsf(g_balance.wheel_v_filt) > 3.00f) {
            // 严重抖动: 冻结路径，防止错误符号把位置越推越远。
            pos_trust = 0.0f;
        } else if (slip_like || g_balance.wheel_thrash > 0.60f) {
            pos_trust = 0.20f;
        } else if (wheel_trust < 0.5f) {
            pos_trust = 0.55f;
        }
        // 只有抖动平稳时才给满路径可信度。
        if (grip_lock && g_balance.wheel_thrash < 0.40f) {
            pos_trust = 1.0f;
        } else if (grip_lock) {
            pos_trust = 0.35f;
        }

        // 打滑时仍保留惯性传播；只有测量融合需要降低轮通道可信度。
        float propagate_trust = 1.0f;
        if (airborne) {
            propagate_trust = 0.20f;
        } else if (impact) {
            propagate_trust = 0.55f;
        } else if (zupt) {
            propagate_trust = 0.0f;
        }
        g_balance.wheel_trust_dbg = wheel_trust;
        g_balance.pos_trust_dbg = pos_trust;
        g_balance.airborne_dbg = airborne ? 1.0f : 0.0f;
        g_balance.impact_dbg = impact ? 1.0f : 0.0f;
        g_balance.slip_dbg = slip_like ? 1.0f : 0.0f;
        g_balance.support_dbg = support_conf;
        g_balance.leg_load_dbg = leg_load;
        g_balance.contact_mode_dbg = (float)contact_mode;

        float acc_unbiased = imu_acc_x - g_balance.acc_bias;
        if (acc_unbiased >  5.0f) acc_unbiased =  5.0f;
        if (acc_unbiased < -5.0f) acc_unbiased = -5.0f;

        // 启动阶段先围绕 0 做纯平衡，防止抖动路径把机器人带跑。
        int startup_hold = (g_balance.run_time < 0.60f);
        // 只有编码器读数非常异常时才认为是严重空转。
        int free_spin_now = ((fabsf(g_balance.wheel_v_filt) > 3.50f || g_balance.wheel_thrash > 1.00f) &&
                            !(grip_lock && g_balance.wheel_thrash < 0.40f));

        if (airborne) {
            // 保留速度，只拒绝明显离谱的加速度样本。
            if (fabsf(acc_unbiased) > 4.0f) {
                /* 跳过积分。 */
            } else {
                g_balance.vel += 0.40f * acc_unbiased * elapsed;
            }
            g_balance.vel *= 0.998f;
        } else if (impact) {
            // 拒绝尖峰，但不要清空速度估计，避免额外滞后。
            if (fabsf(acc_unbiased) < 2.5f) {
                g_balance.vel += 0.30f * acc_unbiased * elapsed;
            }
            g_balance.vel *= 0.997f;
        } else if (startup_hold) {
            g_balance.vel += 0.35f * acc_unbiased * elapsed;
            g_balance.vel *= 0.96f;
            g_balance.ekf_s *= 0.88f;
            g_balance.odom_x *= 0.88f;
            g_balance.odom_y *= 0.88f;
            g_balance.pos_i *= 0.92f;
        } else {
            float acc_use = acc_unbiased;
            // 类似 v41 的处理: 位置路径在抖动时冻结，但速度继续保留。
            if (fabsf(acc_use) < 0.25f &&
                (free_spin_now || g_balance.wheel_thrash > 1.00f)) {
                acc_use *= 0.40f;
            }
            g_balance.vel += acc_use * elapsed;
            float path_scale = propagate_trust;
            if (free_spin_now || g_balance.wheel_thrash > 1.00f) {
                // 严重抖动时冻结路径，防止错误符号持续累积。
                path_scale = 0.0f;
            } else if (slip_like || g_balance.wheel_thrash > 0.60f) {
                path_scale *= 0.35f;
            }
            g_balance.ekf_s += path_scale * (g_balance.vel * elapsed + 0.5f * acc_use * elapsed * elapsed);
            // 严重抖动时只做轻微泄放，抑制偏置游走，不直接杀掉滑行状态。
            if (free_spin_now || g_balance.wheel_thrash > 1.00f) {
                if (g_balance.run_time > 13.0f && support_conf > 0.45f &&
                    g_balance.side_support_balance > 0.58f && fabsf(acc_use) < 4.0f) {
                    g_balance.vel *= 0.9985f;
                } else {
                    g_balance.vel *= 0.994f;
                }
            }
#if USE_IMU_ONLY_ODOM
            g_balance.vel *= 0.985f;
            g_balance.ekf_s *= 0.9992f;
#endif
        }
        if (g_balance.vel >  2.2f) g_balance.vel =  2.2f;
        if (g_balance.vel < -2.2f) g_balance.vel = -2.2f;

        float F[3][3] = {
            {1.0f, elapsed, -0.5f * elapsed * elapsed},
            {0.0f, 1.0f,   -elapsed},
            {0.0f, 0.0f,    1.0f}
        };
        float FP[3][3] = {{0}};
        float Pn[3][3] = {{0}};
        for (int r=0; r<3; r++)
            for (int c=0; c<3; c++)
                for (int k=0; k<3; k++)
                    FP[r][c] += F[r][k] * g_balance.P[k][c];
        for (int r=0; r<3; r++)
            for (int c=0; c<3; c++)
                for (int k=0; k<3; k++)
                    Pn[r][c] += FP[r][k] * F[c][k];
        Pn[0][0] += 0.0005f;
        Pn[1][1] += 0.0400f;
        Pn[2][2] += 0.00001f;
        for (int r=0; r<3; r++)
            for (int c=0; c<3; c++)
                g_balance.P[r][c] = Pn[r][c];

        if (zupt) {
            g_balance.soft_s *= 0.92f;
            g_balance.away_s *= 0.96f;
            g_balance.odom_x *= 0.995f;
            g_balance.odom_y *= 0.995f;
            float R = 0.004f;
            float z = 0.0f;
            float innovation = z - g_balance.vel;
            float S = g_balance.P[1][1] + R;
            if (S < 1e-6f) S = 1e-6f;
            float K[3] = { g_balance.P[0][1] / S, g_balance.P[1][1] / S, g_balance.P[2][1] / S };
            g_balance.ekf_s    += K[0] * innovation;
            g_balance.vel      += K[1] * innovation;
            g_balance.acc_bias += K[2] * innovation;
            g_balance.vel *= 0.55f;

            float Pold[3][3];
            for (int r=0; r<3; r++)
                for (int c=0; c<3; c++)
                    Pold[r][c] = g_balance.P[r][c];
            for (int r=0; r<3; r++)
                for (int c=0; c<3; c++)
                    g_balance.P[r][c] = Pold[r][c] - K[r] * Pold[1][c];
        } else if (!USE_IMU_ONLY_ODOM && wheel_trust > 0.0f &&
                   g_balance.wheel_thrash < 0.70f && fabsf(g_balance.wheel_v_filt) < 1.80f) {
            // 只有抖动适中时才融合轮速。强抖动测量会翻转速度符号，
            // 让阻尼项变成激励项。
            float R = 0.25f / wheel_trust;
            float z = wheel_v_limited;
            if (grip_lock && g_balance.wheel_thrash < 0.45f) {
                z = wheel_v_limited;
                R = 0.04f;
            } else if (slip_like) {
                if (fabsf(wheel_v_limited) < 0.90f) {
                    z = wheel_v_limited;
                    R *= 20.0f;
                } else {
                    z = g_balance.vel;
                    R *= 80.0f;
                }
            }
            if (impact) {
                z = 0.90f * g_balance.vel + 0.10f * wheel_v_limited;
                R *= 6.0f;
            }
            if (fabsf(g_balance.wheel_v_filt) < 0.08f && fabsf(imu_acc_x) < 0.8f &&
                acc_mag > 7.0f && acc_mag < 13.0f && fabsf(pitch_rate) < 0.5f) {
                z = 0.0f;
                R = 0.02f;
            }

            float innovation = z - g_balance.vel;
            if (slip_like || g_balance.wheel_thrash > 0.45f) {
                float lim = 0.10f;
                if (innovation >  lim) innovation =  lim;
                if (innovation < -lim) innovation = -lim;
            }
            if (fabsf(innovation) > 2.0f) R *= 3.0f;
            float S = g_balance.P[1][1] + R;
            if (S < 1e-6f) S = 1e-6f;
            float K[3] = { g_balance.P[0][1] / S, g_balance.P[1][1] / S, g_balance.P[2][1] / S };
            g_balance.ekf_s    += pos_trust * K[0] * innovation;
            g_balance.vel      += K[1] * innovation;
            g_balance.acc_bias += K[2] * innovation;
            if (g_balance.vel >  2.2f) g_balance.vel =  2.2f;
            if (g_balance.vel < -2.2f) g_balance.vel = -2.2f;
            if (g_balance.acc_bias >  3.0f) g_balance.acc_bias =  3.0f;
            if (g_balance.acc_bias < -3.0f) g_balance.acc_bias = -3.0f;
            float Pold[3][3];
            for (int r=0; r<3; r++)
                for (int c=0; c<3; c++)
                    Pold[r][c] = g_balance.P[r][c];
            for (int r=0; r<3; r++)
                for (int c=0; c<3; c++)
                    g_balance.P[r][c] = Pold[r][c] - K[r] * Pold[1][c];
        }
        if (g_balance.P[0][0] < 0.0001f) g_balance.P[0][0] = 0.0001f;
        if (g_balance.P[1][1] < 0.0001f) g_balance.P[1][1] = 0.0001f;
        if (g_balance.P[2][2] < 0.0001f) g_balance.P[2][2] = 0.0001f;
        if (g_balance.P[0][0] > 8.0f) g_balance.P[0][0] = 8.0f;
        if (g_balance.P[1][1] > 6.0f) g_balance.P[1][1] = 6.0f;
        if (g_balance.P[2][2] > 1.0f) g_balance.P[2][2] = 1.0f;
        g_balance.P[1][0] = g_balance.P[0][1] = 0.5f * (g_balance.P[0][1] + g_balance.P[1][0]);
        g_balance.P[2][0] = g_balance.P[0][2] = 0.5f * (g_balance.P[0][2] + g_balance.P[2][0]);
        g_balance.P[2][1] = g_balance.P[1][2] = 0.5f * (g_balance.P[1][2] + g_balance.P[2][1]);

        float cy = cosf(yaw_ang), sy = sinf(yaw_ang);
        float ds_fused = g_balance.vel * elapsed;
#if !USE_IMU_ONLY_ODOM
        float wheel_ds_limited = wheel_ds;
        if (wheel_ds_limited >  0.025f) wheel_ds_limited =  0.025f;
        if (wheel_ds_limited < -0.025f) wheel_ds_limited = -0.025f;
        if ((contact_mode == CONTACT_STICK || grip_lock) && g_balance.wheel_thrash < 0.45f) {
            ds_fused = 0.30f * ds_fused + 0.70f * wheel_ds_limited;
        } else if (contact_mode == CONTACT_SLIP || g_balance.wheel_thrash > 0.60f) {
            if (g_balance.wheel_thrash > 1.00f || fabsf(g_balance.wheel_v_filt) > 3.00f) {
                ds_fused = 0.0f;  // 严重抖动时冻结路径
            } else {
                ds_fused = 0.85f * g_balance.vel * elapsed;
            }
        } else if (contact_mode == CONTACT_ZUPT) {
            ds_fused = 0.0f;
            g_balance.vel *= 0.80f;
        } else {
            ds_fused = 0.0f;
        }
#else
        if (airborne || impact) {
            ds_fused = 0.0f;
        } else if (zupt) {
            ds_fused = 0.0f;
            g_balance.vel *= 0.80f;
        }
#endif
        if (ds_fused >  0.04f) ds_fused =  0.04f;
        if (ds_fused < -0.04f) ds_fused = -0.04f;
        g_balance.odom_x += pos_trust * ds_fused * cy;
        g_balance.odom_y += pos_trust * ds_fused * sy;
        if (airborne) {
            g_balance.ekf_s *= 0.999f;
            g_balance.odom_x *= 0.999f;
            g_balance.odom_y *= 0.999f;
        }
        if (impact) {
            g_balance.ekf_s *= 0.9995f;
            g_balance.odom_x *= 0.9995f;
            g_balance.odom_y *= 0.9995f;
        }
        if (g_balance.odom_x >  4.0f) g_balance.odom_x =  4.0f;
        if (g_balance.odom_x < -4.0f) g_balance.odom_x = -4.0f;
        if (g_balance.odom_y >  4.0f) g_balance.odom_y =  4.0f;
        if (g_balance.odom_y < -4.0f) g_balance.odom_y = -4.0f;
