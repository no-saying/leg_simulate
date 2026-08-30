/* balance_step_origin.inc - 简化三环路径: 原点控制/手动驾驶/腿长/roll稳定
 * 由 balance_core.c 的 balance_update() include 展开。 */
    /* 空闲保持: 车低速持续 2s 后，保持点一次性吸附到当前位置。
     * 旧行为 hold_pos 只在手动进出时吸附，空闲启动后永远停在原点，
     * 位置环会一直“回程”，表现为 60-90s 后开始持续后溜。
     * 吸附后 pos_err≈0，位置环退化为纯速度刹车，车停在当前点。 */
    if (!g_balance.manual_active && !g_balance.post_manual) {
        /* 只有真正静止(速度 < 5mm/s)才重新锚定保持点;
         * 0.01~0.02m/s 的慢漂不算静止, 不能每 2s 就“合理化”掉位置误差,
         * 否则腿摆位置环永远看不到漂移。 */
        if (fabsf(g_balance.body_v_filt) < 0.005f) {
            g_balance.idle_calm_t += dt_f;
            if (g_balance.idle_calm_t > 2.0f) {
                g_balance.hold_pos = g_balance.pos;
                g_balance.idle_calm_t = 0.0f;
            }
        } else {
            g_balance.idle_calm_t = 0.0f;
        }
    }
    /* 手动腿摆目标(单位 m): 在下方 leg_ref 段落统一消费。
     * W/S 时 = K*(目标速度-实际速度) -> 腿摆前移驱动前进, pitch 恒 0;
     * A/D 纯转向时 = 0 -> 腿锁中位, 只做轮子差速;
     * 松手刹车时 = -K*v -> 反向摆腿刹停。 */
    float manual_swing_target = 0.0f;
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
            /* 手动速度控制: 腿摆角度驱动, 目标 pitch 恒为 0。
             * 腿摆前移(sw>0) -> 质心相对轮轴后移 -> 俯仰环输出轮扭矩
             * 推车前进, pitch 保持 0。按住期间腿摆固定 ±SWING_DRIVE,
             * 不用速度反馈(启动/摆腿时轮速被腿几何耦合污染, 会把
             * 腿摆打反翻车); 速度超过 SWING_SPEED_START 后摆腿按比例
             * 回中软限速, 巡航 ~1.5-2m/s, 不依赖轮速阻尼。 */
            neutral_pb = SIMPLE_BASE_NEUTRAL;
            /* 软限速用 IMU 融合速度: 轮速会被摆腿动作几何耦合污染
             * (读 +2m/s 假速度), 把 W/S 摆腿指令压成 0 导致不走。
             * IMU 积分速度反映真实车速。 */
            float v_swing_pb = clampf_local(g_balance.simple_imu_v,
                                            -2.0f, 2.0f);
            float swing_spd_scale = 1.0f -
                (fabsf(v_swing_pb) - SIMPLE_MANUAL_SWING_SPEED_START) /
                SIMPLE_MANUAL_SWING_SPEED_RANGE;
            swing_spd_scale = clampf_local(swing_spd_scale, 0.0f, 1.0f);
            manual_swing_target = (g_balance.manual_v > 0.0f)
                                  ?  SIMPLE_MANUAL_SWING_DRIVE * swing_spd_scale
                                  : -SIMPLE_MANUAL_SWING_DRIVE * swing_spd_scale;
            manual_swing_target = clampf_local(manual_swing_target,
                                               -SIMPLE_MANUAL_SWING_MAX,
                                                SIMPLE_MANUAL_SWING_MAX);
        } else {
            /* A/D 纯转向(无前后指令): 差速会让 body_v 误读成前进/后退,
             * 位置/速度环若给 neutral 偏移会推车前进(实测 tp=-2.7°
             * 边转边前冲)。转向时 neutral 固定 0, 腿摆锁中位, 只做差速。 */
            if (fabsf(g_balance.manual_yaw) >= 0.05f) {
                neutral_pb = SIMPLE_BASE_NEUTRAL;
                manual_swing_target = 0.0f;
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
        }
        /* 滚转安全网: 车身明显倾斜时自动撤手动指令，防止边缘状态下翻车。 */
        if (fabsf(roll) > 0.25f) {
            g_balance.manual_v = 0.0f;
            g_balance.manual_v_target = 0.0f;
            g_balance.manual_yaw = 0.0f;
            g_balance.manual_yaw_target = 0.0f;
            manual_swing_target = 0.0f;
        }
        g_balance.origin_creep = 0.0f;
        g_balance.post_manual = 0;
        g_balance.post_manual_t = 0.0f;
    } else if (g_balance.post_manual) {
        /* 松手停车刹车: 反向摆腿刹停, pitch 目标保持 0。
         * 前进(v>0) -> 腿向后摆(sw<0) -> 质心前移 -> 俯仰环轮子反转
         * 刹停。不用倾角刹停, 车体姿态始终竖直, 不会"后仰前栽"。 */
        g_balance.post_manual_t += dt_f;
        /* 用 IMU 融合速度(不是轮速): 摆腿动作本身会被轮速误读成
         * 前进速度, 形成"刹车->假速度->更猛刹车"正反馈(实测转向后
         * 松手刹车翻车)。IMU 积分速度不受腿几何耦合污染。 */
        float v_err_stop = clampf_local(g_balance.simple_imu_v,
                                        -SIMPLE_MANUAL_SWING_STOP_VMAX,
                                         SIMPLE_MANUAL_SWING_STOP_VMAX);
        neutral_pb = SIMPLE_BASE_NEUTRAL;
        manual_swing_target = -SIMPLE_MANUAL_SWING_STOP_KP * v_err_stop;
        manual_swing_target = clampf_local(manual_swing_target,
                                           -SIMPLE_MANUAL_SWING_MAX,
                                            SIMPLE_MANUAL_SWING_MAX);
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
#if MODEL_V3
    /* v3: 静止平衡固定中性角(0), 不做位置/速度自适应 —— 里程计/位置环
     * 符号未标定时 neutral 漂移会把 pitch 环推成恒向猛冲 -> 翻滚。
     * 但手动驾驶(W/S)必须允许目标 pitch 偏移: 按 W 前倾 -> 平衡环输出
     * 轮扭矩推车前进; 按 S 后仰 -> 后退。手动分支在上面已根据速度误差
     * 计算 neutral_pb, 这里只在非手动时固定为 0。 */
    if (!g_balance.manual_active && !g_balance.post_manual)
        neutral_pb = SIMPLE_BASE_NEUTRAL;
#else
    /* 近场直接扭矩模式里目标俯仰只慢速跟随当前角度（稳定但不主动仰身），
     * 避免目标俯仰阶跃造成的反向冲量；远场仍用快滤波保证回程响应。 */
#endif
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
    if ((g_balance.manual_active || g_balance.post_manual) &&
        g_balance.run_time > 1.0f) {
        /* 手动加减速: 腿摆由速度误差驱动(摆腿移动重心 -> 轮子配合),
         * 车身 pitch 保持小角度。auto 位置环的腿摆在手动时关闭,
         * 否则会把前进的车往回拉(实测 sw 打满 -0.15 翻车)。
         * 启动 1s 内不介入: 平衡建立瞬态轮子会空转冲击(vh 冲到 +3),
         * 此时速度环/腿摆介入会把车推成正反馈振荡。 */
        /* 符号: sw>0 让车向前, sw<0 让车后退(实测)。
         * 车慢于目标(manual_v-v>0) -> 向前摆腿(sw>0);
         * 车快于目标(manual_v-v<0) -> 向后摆腿刹车(sw<0)。
         * auto 用 SIGN=-1, 手动速度环方向相反, 取 -SIGN。 */
        /* 手动 W/S: 腿摆角度就是执行器, pitch 保持 0(上方 neutral 已固定)。
         * A/D 纯转向 / 零指令: manual_swing_target=0, 腿锁中位。
         * 松手刹车: manual_swing_target = -K*v, 反向摆腿刹停。 */
        leg_ref_pb = manual_swing_target;
        leg_ref_pb = clampf_local(leg_ref_pb, -SIMPLE_MANUAL_SWING_MAX,
                                  SIMPLE_MANUAL_SWING_MAX);
        g_balance.leg_pos_i = 0.0f;   /* 手动时清掉 auto 位置积分 */
    }
#if SIMPLE_AUTO_SWING_ENABLE
    else if (SIMPLE_ORIGIN_ENABLE && g_balance.run_time > 1.5f &&
               abs_pitch_pb < 0.35f) {
        /* 腿摆位置环不依赖短窗口 return_ok: 车一漂移就要摆腿纠正,
         * 否则漂移过程里里程计窗口失效会让腿摆失联、只剩慢速 tp。 */
        float pos_err_pb = g_balance.pos - g_balance.hold_pos;
        /* 腿摆位置积分: 消除慢速静差漂移(坡/重心偏置造成的恒定力)。 */
        /* 速度门控: 车明显在动(|v|>0.25)时泄放积分, 防止位置积分
         * windup —— 车先走到前方(积分正)再后退时, 积分若不清会
         * 让腿摆推错方向, 把后退放大成失控(实测 GUI 里 sw 打满
         * -0.15 车仍以 -3m/s 后退)。 */
        if (fabsf(g_balance.body_v_filt) < 0.25f)
            g_balance.leg_pos_i += pos_err_pb * dt_f;
        else
            g_balance.leg_pos_i *= 0.99f;
        g_balance.leg_pos_i = clampf_local(g_balance.leg_pos_i, -1.0f, 1.0f);
        float leg_k_pb = SIMPLE_ORIGIN_LEG_KP;
        if (fabsf(g_balance.pos) >= 0.25f) leg_k_pb *= 2.8f;
        /* 打滑时轮速失真(轮不转), 速度反馈切到 IMU 融合速度,
         * 让腿摆速度环感知真实滑动并刹住。 */
        float leg_v_pb = slip_low_pb ? g_balance.simple_imu_v
                                     : g_balance.body_v_filt;
        leg_ref_pb = SIMPLE_ORIGIN_SIGN * (leg_k_pb * pos_err_pb
                   + SIMPLE_ORIGIN_LEG_KD * clampf_local(leg_v_pb, -2.0f, 2.0f)
                   + SIMPLE_ORIGIN_LEG_KI * g_balance.leg_pos_i);
        leg_ref_pb = clampf_local(leg_ref_pb, -SIMPLE_ORIGIN_LEG_MAX, SIMPLE_ORIGIN_LEG_MAX);
        /* 静止死区: 位置误差 <2cm 且速度 <2cm/s 时腿摆回 0,
         * 防止静止时 leg_ref 残差让腿慢慢顶动车体(实测静止 143s
         * 后 sw 从 0.04 涨到 0.10 顶动触发翻车)。 */
        if (fabsf(pos_err_pb) < 0.04f &&
            fabsf(g_balance.body_v_filt) < 0.03f)
            leg_ref_pb = 0.0f;
    }
#endif /* SIMPLE_AUTO_SWING_ENABLE */
    if (slip_low_pb) {
        /* 打滑补偿: 轮不转车在滑时, 腿摆推力只会加剧滑动(推力被
         * 地面滑动消耗, 车滑得更远), 且 es 位置不可信(轮不转
         * 积分不更新), 位置环越追越滑。打滑期间腿摆完全归零、
         * 位置积分清零, 车停住轮子重新咬合后由滞回解除再恢复。 */
        leg_ref_pb = 0.0f;
        g_balance.leg_pos_i = 0.0f;
    }
    /* 摆腿变化率限幅。v3 的 sw 单位是米(轮轴前向偏移), 0.006/步
     * (1ms 步长) = 6 m/s 的摆腿速度, 满幅 0.03m 只需 5ms, 视觉上
     * 就是"一抽一抽"(用户实测)。改为 0.0003/步 = 0.3 m/s, 满幅
     * 约 100ms 平滑到位, 既能跟上加减速又不抽搐。 */
    /* 摆腿变化率限幅: 手动时 0.06 m/s。 */
    float leg_step_pb = 0.0003f;
    if (g_balance.manual_active || g_balance.post_manual)
        leg_step_pb = 0.00006f;
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

    /* ---- IMU 辅助里程计 ----
     * 互补滤波: IMU 前向加速度积分出速度, 与轮速按速度区间加权融合。
     *   - 低速(0~0.2m/s): 轮编码器量化噪声大, 偏信 IMU 积分;
     *   - 高速: 轮速可信, IMU 只做残差校正(防积分漂移)。
     * 融合速度再积分成位置, 解决“极低速慢漂时里程计冻结”的问题。
     */
    {
        float imu_v_next = g_balance.simple_imu_v +
                           g_balance.imu_acc_filt * dt_f;
        float v_abs_pb = fabsf(g_balance.body_v_filt);
        /* 低速(<0.2m/s)时轮编码器不可信(滑动时轮不转), 让 IMU 加速度
         * 积分主导; 高速轮速主导并校正 IMU 防漂。 */
        float wheel_w_pb = 0.10f + 0.90f *
                           clampf_local(v_abs_pb / 0.20f, 0.0f, 1.0f);
        /* 打滑补偿: 检测到低速打滑时完全放弃轮速(轮在滑不滚),
         * 里程计速度 100% 由 IMU 加速度积分提供。 */
        if (slip_low_pb)
            wheel_w_pb = 0.0f;
        g_balance.simple_imu_v = wheel_w_pb * g_balance.body_v_filt +
                                 (1.0f - wheel_w_pb) * imu_v_next;
        g_balance.simple_imu_v = clampf_local(g_balance.simple_imu_v,
                                              -SIMPLE_ODOM_V_MAX,
                                               SIMPLE_ODOM_V_MAX);
        /* 静止衰减: 轮速≈0 且 IMU 加速度≈0 时把 IMU 积分速度泄放,
         * 防止 IMU 偏置在长时间静止时积分出假速度。 */
        if (fabsf(g_balance.body_v_filt) < 0.005f &&
            fabsf(g_balance.imu_acc_filt) < 0.15f)
            g_balance.simple_imu_v *= 0.995f;
        /* 静止检测(zupt)成立时直接清零 IMU 积分速度: 即使 IMU 加速度
         * 滤波有小偏置(|a|>0.15 时上面 0.995 泄放不触发), 长时间积分
         * 仍会积累假速度(实测静止 120s 后里程计漂到 -400m, GUI trail
         * 点一直动而车体不动, 用户反馈"模型钉住但点在动")。 */
        if (zupt_pb)
            g_balance.simple_imu_v = 0.0f;
        /* 离地/冲击时不积分位置; 其余时间用融合速度积分。 */
        if (!airborne_pb && !impact_pb) {
            float v_int_pb = g_balance.simple_imu_v;
            if (fabsf(v_int_pb) < WHEEL_V_DEADZONE) v_int_pb = 0.0f;
            g_balance.pos += v_int_pb * dt_f;
        }
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
        /* 只衰减旧位置积分(历史遗留项), 位置记忆本身保留:
         * 腿摆位置环需要真实位置误差来纠偏, 一旦衰减掉,
         * 慢漂就被“合理化”成静止。 */
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
    /* 腿摆输出: 名义摆角 + 腿摆偏置。
     * 腿摆是轮腿机器人的核心执行机构 —— 摆腿移动重心, 轮子配合
     * 加减速, 车身 pitch 保持小角度。此前这里只输出名义角,
     * g_balance.leg_swing 算完就被丢弃, 等于摆腿从未被启用。 */
    *sw = balance_sw_nom() + g_balance.leg_swing;
