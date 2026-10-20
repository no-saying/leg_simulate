/* balance_step_origin.inc - 简化三环路径: 原点控制/手动驾驶/腿长/roll稳定
 * 由 balance_core.c 的 balance_update() include 展开。 */
    /* 短腿配平表的陡段不能跟随腿长瞬态立即前摆。 */
    float leg_trim_target = leg_trim_dev(g_balance.leg_len_dbg);
    if (!g_balance.leg_trim_ready) {
        g_balance.leg_trim_ref = leg_trim_target;
        g_balance.leg_trim_ready = 1;
    } else {
        float trim_step = 0.010f * dt_f;
        g_balance.leg_trim_ref += clampf_local(leg_trim_target - g_balance.leg_trim_ref,
                                               -trim_step, trim_step);
    }
    /* 低腿长时逐渐退出腿摆控制，在深屈位形完全锁到几何中位。
     * 接管在失稳工作点之前完成，避免腿长变化时摆腿参考阶跃。 */
    float short_blend_pb = clampf_local(
        (SHORT_LEG_SW_FREEZE_RELEASE - g_balance.leg_len_dbg) /
        (SHORT_LEG_SW_FREEZE_RELEASE - SHORT_LEG_SW_FREEZE_FULL), 0.0f, 1.0f);
    if (g_balance.jump_active || g_balance.self_right_active)
        short_blend_pb = 0.0f;
    float short_freeze_pb = short_blend_pb;
    g_balance.short_sw_blend = short_freeze_pb;
    /* 空闲保持: 车低速持续 2s 后，保持点一次性吸附到当前位置。
     * 旧行为 hold_pos 只在手动进出时吸附，空闲启动后永远停在原点，
     * 位置环会一直“回程”，表现为 60-90s 后开始持续后溜。
     * 吸附后 pos_err≈0，位置环退化为纯速度刹车，车停在当前点。 */
    if (!g_balance.manual_active && !g_balance.post_manual) {
        /* 只有真正静止(速度 < 5mm/s)才重新锚定保持点;
         * 0.01~0.02m/s 的慢漂不算静止, 不能每 2s 就“合理化”掉位置误差,
         * 否则腿摆位置环永远看不到漂移。 */
        if (fabsf(control_v_pb) < 0.005f) {
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
    /* 姿态门控缩放(1=全权, 0=撤指令): 手动/松手刹车都用它收敛摆腿驱动。 */
    float pitch_gate_scale_m = 1.0f;
    /* 进手动瞬间把摆腿参考吸附到当前实际摆位: 自动静止时腿在自然
     * 平衡位(sw≈+0.009), 名义参考是 -0.016。若直接切硬模式, 伺服
     * 会把腿猛拽 0.025m -> pitch 扰动 -> 按 S/W 翻车(GUI 实测)。
     * 吸附后参考从当前位置开始, 手动摆腿命令再缓慢叠加。 */
    if (g_balance.manual_active && !g_balance.manual_swing_seeded) {
        float sw_trim_now = balance_sw_nom() + g_balance.leg_trim_ref;
        g_balance.manual_swing_offset = sw_dbg - sw_trim_now;
        g_balance.leg_swing = g_balance.manual_swing_offset; /* 直接吸附到当前摆位 */
        /* 进手动时把"保持点"锚在当前里程计位置: 纯转向/松手时位置环
         * 钉在这个点, 而不是钉在很久以前的自动保持点。 */
        g_balance.hold_pos = g_balance.pos;
        g_balance.manual_swing_seeded = 1;
    }
    if (!g_balance.manual_active)
        g_balance.manual_swing_seeded = 0;
    if (!g_balance.spin_rate_external && fabsf(g_balance.manual_yaw_target) >
        0.5f * (SIMPLE_MANUAL_YAW_RATE + SIMPLE_MANUAL_SPIN_YAW_RATE)) {
        float span_pb = SIMPLE_SPIN_LEG_REDUCE_FULL - SIMPLE_SPIN_LEG_REDUCE_START;
        if (span_pb < 1e-4f) span_pb = 1e-4f;
        float blend_pb = clampf_local(
            (g_balance.leg_len_dbg - SIMPLE_SPIN_LEG_REDUCE_START) / span_pb,
            0.0f, 1.0f);
        float spin_rate_pb = SIMPLE_MANUAL_SPIN_RATE_SCALE *
            (SIMPLE_MANUAL_SPIN_YAW_RATE + blend_pb *
             (SIMPLE_MANUAL_YAW_RATE - SIMPLE_MANUAL_SPIN_YAW_RATE));
        g_balance.manual_yaw_target = copysignf(spin_rate_pb,
                                                 g_balance.manual_yaw_target);
    }
    /* 吸附只是"防止进手动瞬间伺服猛拽腿"的过渡, 不能一直保留:
     * 静止时软腿停在实际平衡位(sw≈+0.008), 比伺服名义摆角
     * BALANCE_LEG_SW_NOM 靠后 ~24mm, 也就是质心在轮轴后方。
     * 自动状态靠 SIMPLE_VEL_DAMP 把这个静差压成 ~0.2m/s 慢漂,
     * 而手动转向/停车要关掉速度阻尼(否则差速伪影会把车推走),
     * 静差就完全暴露: 一按 A/D(无前后指令, 腿锁在当前位)车就自己
     * 往后加速(~1.3m/s²), 表现为"转弯和前进/后退耦合"(用户反馈)。
     * 这里让吸附偏移在 ~2.5s 内平滑衰减到 0, 腿重新对到平衡摆角,
     * 静差消失, 车才能真正原地停住/原地转。 */
    if (g_balance.manual_active) {
        float recenter_step = SIMPLE_MANUAL_SWING_RECENTER_RATE * dt_f;
        /* 手动摆角配平点随腿长变化: 腿越长, 平衡所需摆角越靠后(负方向)。
         * 实测(按住转向看离起点漂移, 越小越好):
         *   腿长 0.110m -> 配平 0.016 (漂 0.06m)
         *   腿长 0.131m -> 配平 0.005 (漂 0.05m); 仍用 0.016 会漂 0.21m
         *   腿长 0.149m -> 配平 0.005 (漂 0.10m); 仍用 0.016 会漂 0.86m
         * 用"随腿长线性收缩 + 高低位限幅"拟合, 见 config_manual.h。 */
        float trim_pb = SIMPLE_MANUAL_SWING_TRIM
                      + SIMPLE_MANUAL_SWING_TRIM_LK *
                        (g_balance.leg_len_dbg - SIMPLE_MANUAL_LEG_LEN_REF);
        trim_pb = clampf_local(trim_pb, SIMPLE_MANUAL_SWING_TRIM_MIN,
                                        SIMPLE_MANUAL_SWING_TRIM_MAX);
        float trim_err_m = g_balance.manual_swing_offset - trim_pb;
        if (trim_err_m > recenter_step)
            g_balance.manual_swing_offset -= recenter_step;
        else if (trim_err_m < -recenter_step)
            g_balance.manual_swing_offset += recenter_step;
        else
            g_balance.manual_swing_offset = trim_pb;
    }
    if (g_balance.manual_active) {
        /* ---- 腿摆没有权限的腿长: 拒绝 W/S 驱动 ----
         * 与转向同样的处理: 窗口外(实测 <0.095 或 >0.140m)腿摆推不动车,
         * 按下 W 只会让摆腿指令打满、其横向分力把车反向推走(用户实测
         * "腿到最短一按 W 就反向疯狂加速, 压根没有刹车")。
         * 这里直接不接速度指令, 只保留俯仰平衡 + 轮上刹车(轮速阻尼已在
         * 窗口外恢复), 车停在原地。 */
        /* 【注意】这里只关"腿摆驱动", 不能连速度反馈一起关!
         * 之前写成清 manual_v -> 速度环目标变 0、原点环又因 manual_active
         * 让位 -> 车上没有任何速度反馈在跑: 按 W 后 v 从 0.46 一路涨到
         * 4.1m/s 撞翻(用户 GUI 实测 dashcam_20260926_123744, sw 只跳了一下
         * 0.022 就被拉回 0, 而 v 单调加速 —— "压根没有刹车")。
         * 现在的处理: 腿摆不参与(leg_ref=0, 由下面的降级块负责),
         * 速度指令照常给, 由"窗口外保留的轮速阻尼"负责刹车/限速。 */
        /* 指令斜坡化: 速度/转向指令按固定加速度爬升，避免起步瞬间猛冲。 */
        float slew_m = (g_balance.manual_v * g_balance.manual_v_target <= 0.0f ||
                        fabsf(g_balance.manual_v_target) < fabsf(g_balance.manual_v))
                     ? SIMPLE_MANUAL_DECEL : SIMPLE_MANUAL_ACCEL;
        float dv_m = slew_m * dt_f;
        float d_m = g_balance.manual_v_target - g_balance.manual_v;
        d_m = clampf_local(d_m, -dv_m, dv_m);
        g_balance.manual_v += d_m;
        float dy_m = SIMPLE_MANUAL_YAW_ACCEL * dt_f;
        float dym = g_balance.manual_yaw_target - g_balance.manual_yaw;
        dym = clampf_local(dym, -dy_m, dy_m);
        g_balance.manual_yaw += dym;

        /* 手动限速刹车: 速度超过上限时叠加反向俯仰，防止追目标漂移成高速
         * 后急转/颠簸把车掀翻。 */
        float v_abs_m = fabsf(control_v_pb);
        if (v_abs_m > SIMPLE_MANUAL_SPEED_BRAKE_V) {
            float brake_m = SIMPLE_MANUAL_SPEED_BRAKE_KP *
                            (v_abs_m - SIMPLE_MANUAL_SPEED_BRAKE_V);
            brake_m = clampf_local(brake_m, 0.0f, SIMPLE_MANUAL_SPEED_BRAKE_MAX);
            if (control_v_pb > 0.0f) {
                neutral_pb -= brake_m;
            } else {
                neutral_pb += brake_m;
            }
        }

        /* 前倾角门控: 姿态偏离时先按比例收敛驱动/转向指令，只有超过
         * 硬阈值才彻底撤掉。比例段是为了让大扰动下驱动平滑退让，
         * 避免"指令还在但腿摆被硬清零"造成的来回抽动。 */
        pitch_gate_scale_m = 1.0f;
        {
            float ap_m = fabsf(pitch);
            float span_m = SIMPLE_MANUAL_PITCH_GATE_HARD - SIMPLE_MANUAL_PITCH_GATE;
            if (span_m < 0.01f) span_m = 0.01f;
            pitch_gate_scale_m = (SIMPLE_MANUAL_PITCH_GATE_HARD - ap_m) / span_m;
            pitch_gate_scale_m = clampf_local(pitch_gate_scale_m, 0.0f, 1.0f);
        }
        if (fabsf(pitch) > SIMPLE_MANUAL_PITCH_GATE_HARD) {
            g_balance.manual_v = 0.0f;
            g_balance.manual_v_target = 0.0f;
            g_balance.manual_yaw = 0.0f;
            g_balance.manual_yaw_target = 0.0f;
            pitch_gate_scale_m = 0.0f;
        }
        /* 手动前后运动统一走"腿摆速度环", 目标速度三种来源:
         *   W/S 按住      : v_ref = 已斜坡化的手动速度指令;
         *   A/D 纯转向    : v_ref = 位置环"钉在转向进入点"(差速会让
         *                    body_v 误读, 但位置误差是真实的);
         *   零指令(松手) : 同上, 钉在当前保持点。
         * 目标俯仰恒为 0(pitch 环只负责站直), 前进/后退完全由腿摆
         * 产生的质心偏移完成 —— 这就是用户要的"W/S 摆腿走路"。
         *
         * 物理关系(模型实测 + 几何标定一致): sw 是"轮轴相对基座的前向
         * 偏移", sw 减小 = 轮轴相对车体后移 = 质心相对轮轴前移 ->
         * 俯仰环输出向前轮扭矩 -> 车前进。所以正的速度误差要负摆角。
         * 速度反馈用 body_v_filt: 已扣除腿运动学(swing_dot)与俯仰运动
         * 引入的轮速分量, 不会被摆腿动作自身污染。 */
        float v_ref_m = 0.0f;
        if (g_balance.manual_stop_pending) {
            /* Establish the parking point after braking, so a moving turn
             * does not drive back toward the point where W/S was released. */
            g_balance.hold_pos = g_balance.pos;
            if (fabsf(g_balance.manual_v) < 0.05f && fabsf(control_v_pb) < 0.08f)
                g_balance.manual_stop_calm_t += dt_f;
            else
                g_balance.manual_stop_calm_t = 0.0f;
            if (g_balance.manual_stop_calm_t > 0.30f)
                g_balance.manual_stop_pending = 0;
        }
        if (fabsf(g_balance.manual_v) >= 0.05f) {
            v_ref_m = g_balance.manual_v;
        } else if (!g_balance.manual_stop_pending) {
            float pos_err_m = g_balance.pos - g_balance.hold_pos;
            v_ref_m = -SIMPLE_MANUAL_HOLD_POS_KP * pos_err_m;
            v_ref_m = clampf_local(v_ref_m,
                                   -SIMPLE_MANUAL_HOLD_V_MAX,
                                    SIMPLE_MANUAL_HOLD_V_MAX);
        }
        neutral_pb = SIMPLE_BASE_NEUTRAL;
        float v_err_m = clampf_local(v_ref_m - control_v_pb,
                                     -SIMPLE_MANUAL_SWING_V_ERR_MAX,
                                      SIMPLE_MANUAL_SWING_V_ERR_MAX);
        manual_swing_target = -SIMPLE_MANUAL_SWING_V_KP * v_err_m;
        manual_swing_target = clampf_local(manual_swing_target,
                                           -SIMPLE_MANUAL_SWING_MAX,
                                            SIMPLE_MANUAL_SWING_MAX);
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
        /* 松手刹车: 目标速度 0, 与 W/S 同一个速度环(只是符号取反)。
         * 前进(v>0) -> 腿摆增大 -> 质心后移 -> 俯仰环输出向后轮扭矩刹停。 */
        manual_swing_target = (g_balance.post_spin &&
                               g_balance.leg_len_dbg >= 0.140f) ? 0.0f :
                              SIMPLE_MANUAL_SWING_STOP_KP * v_err_stop;
        manual_swing_target = clampf_local(manual_swing_target,
                                           -SIMPLE_MANUAL_SWING_MAX,
                                            SIMPLE_MANUAL_SWING_MAX);
        g_balance.origin_creep = 0.0f;
        if (fabsf(control_v_pb) < SIMPLE_MANUAL_STOP_V &&
            g_balance.post_manual_t > 0.30f) {
            g_balance.post_manual = 0;
            g_balance.post_manual_rest = 0;
    g_balance.runaway_t = 0.0f;
    g_balance.pitch_i = 0.0f;
            g_balance.hold_pos = g_balance.pos;
            g_balance.yaw_zero = g_balance.yaw_raw_prev;
        } else if (g_balance.post_manual_t >
                   (g_balance.leg_len_dbg >= SIMPLE_LONG_MANUAL_BRAKE_START
                    ? SIMPLE_LONG_POST_TIMEOUT : 2.0f)) {
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
        fabsf(control_v_pb) > SIMPLE_CAPTURE_LEAN_V &&
        abs_pitch_pb < SIMPLE_CAPTURE_LEAN_MAX &&
        !airborne_pb && !impact_pb) {
        float capture_neutral_pb = pitch +
            ((control_v_pb > 0.0f) ? SIMPLE_CAPTURE_LEAN_STEP : -SIMPLE_CAPTURE_LEAN_STEP);
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
            (fabsf(control_v_pb) - SIMPLE_WHEEL_CAPTURE_ENTER_V);
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
    if (short_blend_pb > 0.0f && g_balance.run_time > 1.2f &&
        !airborne_pb && !impact_pb) {
        float short_v_ref_pb = 0.0f;
        if (g_balance.manual_active && fabsf(g_balance.manual_v) >= 0.05f) {
            short_v_ref_pb = g_balance.manual_v;
        } else if (!g_balance.post_manual) {
            short_v_ref_pb = -SHORT_LEG_TILT_POS_V_KP *
                             (g_balance.pos - g_balance.hold_pos);
            short_v_ref_pb = clampf_local(short_v_ref_pb, -0.30f, 0.30f);
        }
        float tilt_pb = SHORT_LEG_TILT_SIGN * SHORT_LEG_TILT_V_KP *
                        (control_v_pb - short_v_ref_pb);
        tilt_pb = clampf_local(tilt_pb, -SHORT_LEG_TILT_MAX, SHORT_LEG_TILT_MAX);
        neutral_pb += short_blend_pb * (tilt_pb - neutral_pb);
    }
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
        /* 手动 W/S: 腿摆速度环输出(sw 减小 = 质心前移 = 向前),
         * pitch 保持 0(上方 neutral 已固定)。
         * A/D 纯转向 / 零指令: manual_swing_target=0, 腿锁中位。
         * 松手刹车: manual_swing_target = +K*v, 反向摆腿刹停。 */
        /* 参考 = 进手动时的实际摆位偏移 + 手动摆腿命令:
         * 腿从当前位置缓慢摆到命令位置, 不猛拽。 */
        /* 姿态门控按比例收敛摆腿驱动(不是硬清零): 姿态偏离时驱动
         * 让位给俯仰环, 回正后自动恢复按住键的摆腿指令。 */
        manual_swing_target *= pitch_gate_scale_m;
        leg_ref_pb = g_balance.manual_swing_offset + manual_swing_target;
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
        if (fabsf(control_v_pb) < 0.25f)
            g_balance.leg_pos_i += pos_err_pb * dt_f;
        else
            g_balance.leg_pos_i *= 0.99f;
        g_balance.leg_pos_i = clampf_local(g_balance.leg_pos_i, -1.0f, 1.0f);
        float leg_k_pb = SIMPLE_ORIGIN_LEG_KP;
        if (fabsf(g_balance.pos) >= 0.25f) leg_k_pb *= 2.8f;
        /* 打滑时轮速失真(轮不转), 速度反馈切到 IMU 融合速度,
         * 让腿摆速度环感知真实滑动并刹住。 */
        float leg_v_pb = slip_low_pb ? g_balance.simple_imu_v
                                     : control_v_pb;
        leg_ref_pb = SIMPLE_ORIGIN_SIGN * (leg_k_pb * pos_err_pb
                   + SIMPLE_ORIGIN_LEG_KD * clampf_local(leg_v_pb, -2.0f, 2.0f)
                   + SIMPLE_ORIGIN_LEG_KI * g_balance.leg_pos_i);
        leg_ref_pb = clampf_local(leg_ref_pb, -SIMPLE_ORIGIN_LEG_MAX, SIMPLE_ORIGIN_LEG_MAX);
        /* 静止死区: 位置误差 <2cm 且速度 <2cm/s 时腿摆回 0,
         * 防止静止时 leg_ref 残差让腿慢慢顶动车体(实测静止 143s
         * 后 sw 从 0.04 涨到 0.10 顶动触发翻车)。 */
        if (fabsf(pos_err_pb) < 0.04f &&
            fabsf(control_v_pb) < 0.03f)
            leg_ref_pb = 0.0f;
    }
#endif /* SIMPLE_AUTO_SWING_ENABLE */
    if (linear_slip_pb) {
        /* Keep the support/trim pose; only soften requested translation.
         * Returning the swing to zero abruptly also moves the center of mass. */
        g_balance.leg_pos_i = 0.0f;
        if (g_balance.manual_active && fabsf(g_balance.manual_v) >= 0.05f)
            leg_ref_pb = g_balance.manual_swing_offset +
                         0.35f * (leg_ref_pb - g_balance.manual_swing_offset);
        else if (!g_balance.manual_active && !g_balance.post_manual)
            leg_ref_pb = g_balance.leg_swing;
    }
    leg_ref_pb *= 1.0f - short_freeze_pb;
    {
        /* 临时调试(env SWDBG=1): 打印腿摆参考链路的内部量。 */
        static int swdbg = -1;
        static int swdbg_cnt = 0;
        if (swdbg < 0) swdbg = getenv("SWDBG") ? atoi(getenv("SWDBG")) : 0;
        if (swdbg) {
            swdbg_cnt++;
            if (swdbg_cnt >= 100) {
                swdbg_cnt = 0;
                printf("[SW] t=%.2f man=%d post=%d off=%+.4f tgt=%+.4f "
                       "legref=%+.4f sw=%+.4f | simu_v=%+.3f body_v=%+.3f "
                       "wheel_v=%+.3f slip_low=%d wheel_vmax=%.2f\n",
                       g_balance.run_time, g_balance.manual_active,
                       g_balance.post_manual, g_balance.manual_swing_offset,
                       manual_swing_target, leg_ref_pb, g_balance.leg_swing,
                       g_balance.simple_imu_v, control_v_pb,
                       wheel_v_raw, slip_low_pb, SIMPLE_SLIP_LOW_V);
            }
        }
    }
    /* ---- 腿长范围保护 ----
     * 实测: 腿长 <0.085 或 >0.140m 时, 摆角参考改 4cm 车的行为完全不变
     * => 腿摆在该位形下【没有权限】(闭链几何锁死)。此时若仍用腿摆做前后
     * 控制, 车会被腿长伺服的横向分力推着漂(实测短腿 +2.87m/s、长腿 -1.28m/s)。
     * 降级为"轮扭矩 + 允许倾角"(类似 Segway): 腿摆只保留配平,
     * 前后由目标倾角驱动轮扭矩, 允许 ±3° 倾角换掉"腿摆无权限"这件事。 */
    {
        /* 只在超出实际机械行程时关闭摆腿动态控制。中间的旧窗口
         * [0.085,0.140] 会把正常短腿/长腿误判成无权限，随后切入错误
         * 的倾角降级环，扰动时容易形成持续单向运动。 */
        static float legwin_min_cfg, legwin_max_cfg;
        static int legwin_ready = 0;
        if (!legwin_ready) {
            const char *e = getenv("LEGWIN_MIN");
            legwin_min_cfg = e ? (float)atof(e) : -1.0f;
            e = getenv("LEGWIN_MAX");
            legwin_max_cfg = e ? (float)atof(e) : -1.0f;
            legwin_ready = 1;
        }
        float legwin_min = legwin_min_cfg >= 0.0f ? legwin_min_cfg : balance_leg_drive_min();
        float legwin_max = legwin_max_cfg >= 0.0f ? legwin_max_cfg : balance_leg_drive_max();
        int leg_win_ok = (g_balance.leg_len_dbg >= legwin_min &&
                          g_balance.leg_len_dbg <= legwin_max);
        if (!leg_win_ok) {
            leg_ref_pb = 0.0f;                 /* 腿摆只留配平, 不参与动态 */
            /* 超出机械范围时只撤掉腿摆动态参考，保持当前俯仰目标。
             * 不再用未经验证的速度误差->倾角降级环改写 neutral；该环
             * 在腿长异常时会把小扰动放大成持续前后运动。 */
        }
    }

    /* 摆腿变化率限幅。v3 的 sw 单位是米(轮轴前向偏移), 0.006/步
     * (1ms 步长) = 6 m/s 的摆腿速度, 满幅 0.03m 只需 5ms, 视觉上
     * 就是"一抽一抽"(用户实测)。改为 0.0003/步 = 0.3 m/s, 满幅
     * 约 100ms 平滑到位, 既能跟上加减速又不抽搐。 */
    /* 摆腿变化率限幅: 手动时 0.01 m/s(满幅 0.015m 约 1.5s)。
     * 0.06 m/s 时 0.25s 摆到位, pitch 瞬态超轮上摩擦极限 -> 按 S
     * 从静止翻车(GUI 实测)。放慢后 pitch 率低, 软增益+1.2Nm 压得住。 */
    float leg_step_pb = 0.0003f;
    if (g_balance.manual_active || g_balance.post_manual)
        leg_step_pb = SIMPLE_MANUAL_SWING_RATE;
    g_balance.leg_swing += clampf_local(leg_ref_pb - g_balance.leg_swing, -leg_step_pb, leg_step_pb);
    /* 腿长是竖直支撑通道，不能用摆腿级别的快速参考变化。
     * 0.00045 m/步在 1 kHz 下等于 0.45 m/s，深屈/伸长时会让
     * 腿长阻尼力瞬间变成几十 Nm，并通过闭链雅可比激发俯仰。
     * 限到 0.00012 m/步(0.12 m/s)，全行程约 0.33 s，给姿态环
     * 足够时间吸收腿长动作。 */
    float ext_step_pb = 0.00012f;
    /* 短腿段单独放慢腿长动作: 深屈位形下腿长伺服的横向泄漏与动作速度相关 */
    if (g_balance.leg_len_dbg < SHORT_LEG_MAX)
        ext_step_pb *= SHORT_LEG_EXT_STEP_SCALE;
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

    /* v3 ext>0 收腿，CAD 左轮在 -Y。roll<0 时左侧被顶高，side>0 收左腿。
     * 单侧支撑扰动时增加共模收腿，避免低侧被对称补偿继续伸长。 */
    {
        float roll_dot_pb = attitude_roll_rate(roll, pitch, roll_rate, gyro_y, yaw_rate);
        float rr_alpha = dt_f / (0.020f + dt_f);
        rr_alpha = clampf_local(rr_alpha, 0.0f, 1.0f);
        g_balance.roll_rate_filt += rr_alpha * (roll_dot_pb - g_balance.roll_rate_filt);

        int roll_support_ok = !g_balance.jump_active && !g_balance.self_right_active &&
            !airborne_pb && g_balance.run_time > ROLL_STAB_STARTUP &&
            fabsf(roll) < ROLL_STAB_DISABLE_ABS;
#if MODEL_V3
        if (g_balance.jump_active || g_balance.self_right_active) {
            g_balance.obstacle_roll_hold = 0.0f;
            g_balance.obstacle_retract = 0.0f;
            for (int i = 0; i < 2; ++i) {
                g_balance.obstacle_wheel_hold[i] = 0.0f;
                g_balance.obstacle_leg_retract[i] = 0.0f;
            }
        }
        if (roll_support_ok &&
            ((fabsf(roll) > ROLL_OBSTACLE_ENTER && fabsf(g_balance.roll_rate_filt) > ROLL_OBSTACLE_RATE) ||
             fabsf(roll) > ROLL_OBSTACLE_LARGE ||
             ((impact_pb || slip_pb) && fabsf(roll) > ROLL_OBSTACLE_ENTER)))
            g_balance.obstacle_roll_hold = ROLL_OBSTACLE_HOLD;
        else
            g_balance.obstacle_roll_hold = fmaxf(0.0f, g_balance.obstacle_roll_hold - dt_f);
#endif

        int roll_stab_ok =
            !g_balance.jump_active &&
            !g_balance.self_right_active &&
            !airborne_pb && (!impact_pb || g_balance.obstacle_roll_hold > 0.0f) &&
            g_balance.run_time > ROLL_STAB_STARTUP &&
            fabsf(roll) < ROLL_STAB_DISABLE_ABS;

        float side_ref_pb = 0.0f;
        if (roll_stab_ok) {
            float roll_err = roll;                    /* 目标 roll=0 */
            float roll_d = g_balance.roll_rate_filt;
            float p_pb = -ROLL_STAB_KP * roll_err - ROLL_STAB_KD * roll_d;
            float di_pb = -ROLL_STAB_KI * roll_err * dt_f;
            int at_limit_pb = fabsf(p_pb + g_balance.roll_i) >= ROLL_STAB_MAX &&
                              di_pb * (p_pb + g_balance.roll_i) > 0.0f;
            if (g_balance.obstacle_roll_hold <= 0.0f && !at_limit_pb &&
                di_pb * g_balance.roll_allocation_error <= 0.0f)
                g_balance.roll_i += di_pb;
            g_balance.roll_i = clampf_local(g_balance.roll_i,
                                            -ROLL_STAB_I_MAX, ROLL_STAB_I_MAX);
            side_ref_pb = p_pb + g_balance.roll_i;
        } else {
            g_balance.roll_i *= 0.95f;   /* 退出时泄放积分 */
        }
        side_ref_pb = clampf_local(side_ref_pb, -ROLL_STAB_MAX, ROLL_STAB_MAX);
        float side_step_pb = ROLL_STAB_RATE * dt_f / 0.001f;
        g_balance.leg_side_extend += clampf_local(
            side_ref_pb - g_balance.leg_side_extend, -side_step_pb, side_step_pb);
#if MODEL_V3
        float roll_retract_pb = 0.0f;
        if (roll_support_ok && g_balance.obstacle_roll_hold > 0.0f)
            roll_retract_pb = fminf(fabsf(g_balance.leg_side_extend), ROLL_OBSTACLE_RETRACT_MAX);
        for (int i = 0; i < 2; ++i) {
            if (roll_support_ok && g_balance.slip_observer.wheel[i].active)
                g_balance.obstacle_wheel_hold[i] = WHEEL_OBSTACLE_HOLD;
            else
                g_balance.obstacle_wheel_hold[i] = fmaxf(0.0f,
                    g_balance.obstacle_wheel_hold[i] - dt_f);
            float retract_ref_pb = roll_retract_pb;
            if (roll_support_ok && g_balance.obstacle_wheel_hold[i] > 0.0f)
                retract_ref_pb = fmaxf(retract_ref_pb, WHEEL_OBSTACLE_RETRACT);
            float base_pb = g_balance.leg_extend + (i ? -1.0f : 1.0f) * g_balance.leg_side_extend;
            retract_ref_pb = fminf(retract_ref_pb,
                fmaxf(0.0f, MANUAL_LEG_EXTEND_SHORT_MAX - base_pb));
            float retract_rate_pb = retract_ref_pb > g_balance.obstacle_leg_retract[i] ?
                ROLL_OBSTACLE_RETRACT_RATE : ROLL_OBSTACLE_RESTORE_RATE;
            g_balance.obstacle_leg_retract[i] += clampf_local(
                retract_ref_pb - g_balance.obstacle_leg_retract[i],
                -retract_rate_pb * dt_f, retract_rate_pb * dt_f);
        }
        g_balance.obstacle_retract = 0.5f * (g_balance.obstacle_leg_retract[0] +
                                           g_balance.obstacle_leg_retract[1]);
#endif
    }

    /* ---- IMU 辅助里程计 ----
     * 互补滤波: IMU 前向加速度积分出速度, 与轮速按速度区间加权融合。
     *   - 低速(0~0.2m/s): 轮编码器量化噪声大, 偏信 IMU 积分;
     *   - 高速: 轮速可信, IMU 只做残差校正(防积分漂移)。
     * 融合速度再积分成位置, 解决“极低速慢漂时里程计冻结”的问题。
     */
    {
        /* ---- 前向速度: 卡尔曼融合(轮速 + IMU 加速度) ----
         * 旧实现是"按 |v| 手工分段的互补滤波", 低速靠 IMU 积分(会漂)、
         * 高速靠轮速(打滑时失真), 且没有偏置估计——实测关小陀螺瞬间
         * simple_imu_v 被污染到 -0.8m/s(车实际在前进), 导致刹车方向反了、
         * 车加速到 2.8m/s。改成 KF: 显式估计加速度计偏置, 量测噪声按
         * 打滑/离地自适应, 离地时纯惯性递推。
         * 参数与 scripts/tools/identify_lqr_v3.py 的 KF 严格一致。 */
        float trust_pb = 1.0f;
        float v_meas_pb = g_balance.body_v_filt;
        if (linear_slip_pb) {
            trust_pb = 0.0f;           /* Skip corrupt encoders and bias learning. */
        } else if (impact_pb) {
            trust_pb = 0.30f;          /* 冲击/不确定 */
        }
        if (zupt_pb) {                 /* 零速更新: 拉向 0 并学偏置 */
            v_meas_pb = 0.0f;
            trust_pb = 1.0f;
        }
        g_balance.simple_imu_v = vel_kf_step(g_balance.imu_acc_filt, v_meas_pb,
                                             trust_pb, airborne_pb, dt_f);
        if (linear_slip_pb) {
            g_balance.simple_imu_v = control_v_pb;
            vel_kf_set_velocity(control_v_pb);
        }
        {
            float imu_v_next = g_balance.simple_imu_v + g_balance.imu_acc_filt * dt_f;
            (void)imu_v_next;
        }
        if (0) {
                }  /* 旧互补滤波分支已由 KF 取代, 保留结构便于回退 */
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
    g_balance.vel = control_v_pb;
    g_balance.vel_ff = control_v_pb;
    g_balance.wheel_trust_dbg = wheel_trust_pb;
    g_balance.pos_trust_dbg = g_balance.origin_trust;
    g_balance.airborne_dbg = airborne_pb ? 1.0f : 0.0f;
    g_balance.impact_dbg = impact_pb ? 1.0f : 0.0f;
    g_balance.slip_dbg = slip_pb ? 1.0f : 0.0f;
    g_balance.raw_airborne_dbg = g_balance.airborne_dbg;
    g_balance.raw_impact_dbg = g_balance.impact_dbg;
    g_balance.raw_slip_dbg = g_balance.slip_observer.raw ? 1.0f : 0.0f;
    g_balance.raw_zupt_dbg = zupt_pb ? 1.0f : 0.0f;
    g_balance.support_dbg = (!airborne_pb && !impact_pb) ? 1.0f : 0.0f;
    g_balance.leg_load_dbg = leg_load_common;
    g_balance.contact_mode_dbg = airborne_pb ? CONTACT_AIRBORNE :
        impact_pb ? CONTACT_IMPACT : slip_pb ? CONTACT_SLIP : CONTACT_STICK;
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
    float sw_normal_pb = balance_sw_nom() + g_balance.leg_trim_ref
                       + g_balance.leg_swing;
    *sw = sw_normal_pb + g_balance.short_sw_blend *
          (SHORT_LEG_SW_CENTER - sw_normal_pb);
