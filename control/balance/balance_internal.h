/*
 * balance_internal.h - 平衡控制核心的内部共享头文件
 * ================================================================
 * 只允许 balance/ 目录下的 .c 文件包含本头文件。
 *
 * 拆分背景:
 *   原 balance_core.c 超过 3000 行，按“工具 / 状态 / 各控制环”拆成多个
 *   文件后，共享的运行状态 BalanceRuntime 和少量工具函数声明集中在这里，
 *   避免每个文件重复定义。
 *
 * 状态访问约定:
 *   - g_balance 是全局唯一的平衡控制状态对象，由 balance_core.c 定义，
 *     其余文件通过 extern 访问。
 *   - 工具函数(滤波/限幅/窗口)由 balance_util.c 实现。
 */
#ifndef BALANCE_INTERNAL_H
#define BALANCE_INTERNAL_H

#include "balance.h"
#include "slip_observer.h"
#include <stdint.h>

/* 接触状态枚举: 供调试日志和接触模式判断共用。 */
enum {
    CONTACT_AIRBORNE = 0,   /* 离地 */
    CONTACT_IMPACT   = 1,   /* 冲击 */
    CONTACT_SLIP     = 2,   /* 打滑 */
    CONTACT_STICK    = 3,   /* 正常粘着 */
    CONTACT_ZUPT     = 4,   /* 零速更新 */
};

/* 匹配 FIR 滤波器长度; M = (N-1)/2 即群延迟采样数。 */
enum { FIR_N = 11, FIR_M = 5 };

/* 控制环分频: 速度/里程计环 100 Hz、偏航环 50 Hz。
 * 试过把偏航环提到 1kHz: 抖动反而变大(std 0.24 -> 0.35), 说明这个
 * "一卡一卡"不是采样率造成的, 而是环路在激发偏航轴的机构谐振,
 * 提采样率只会让环路带宽更高、更容易激发。保持 50Hz。 */
enum {
    SPEED_LOOP_HZ = 100,
    YAW_LOOP_HZ   = 1000,
};

/*
 * BalanceRuntime - 平衡控制器的完整运行状态
 * ----------------------------------------------------------------
 * 把原先散落的 g_xxx 状态收进一个控制器对象。当前工程只有一个平衡
 * 控制实例，所以对象保持文件级全局；但状态边界已经明确，后续要继续
 * 拆滤波、里程计、偏航子模块时可以直接传指针。
 */
typedef struct {
    int bal_mode;                 /* 平衡开关，0=输出归零并清状态，1=运行控制。 */

    float pos;                    /* m，前向轴里程计位置。 */
    float ekf_s;                  /* m，旧扩展卡尔曼/路径位移读数。 */
    float odom_x;                 /* m，按偏航投影后的显示用里程计 x。 */
    float odom_y;                 /* m，按偏航投影后的显示用里程计 y。 */
    float vel;                    /* m/s，当前机体前向速度估计。 */
    float vel_ff;                 /* m/s，低延迟速度副本，用于阻尼/前馈。 */

    float soft_s;                 /* m，低延迟速度积分得到的软路径。 */
    float soft_s_prev;            /* m，上一帧软路径，用于判断路径增长。 */
    float capture_t;              /* s，越过原点后的纯刹车锁存计时。 */
    float settle_s;               /* 评分，正在原点附近稳定的证据。 */
    float away_s;                 /* m，持续远离原点的记忆量。 */
    float coast_t;                /* s，长时间打滑/滑行状态计时。 */
    float coast_s;                /* m，惯性积分得到的滑行位移记忆。 */
    int brake_latched;            /* 远场刹车滞回锁存。 */
    float brake_v_sign;           /* 符号，刹车区内记住的速度方向。 */
    float brake_v_hold;           /* s，反向速度证据计时。 */
    float motion_dir;             /* 符号，由里程计得到的机体运动方向记忆。 */
    float motion_hold;            /* s，反向运动证据计时。 */
    float hold_v;                 /* m/s，死算用的平稳速度记忆。 */
    float hold_v_age;             /* s，距离上次更新平稳速度的时间。 */
    float brake_dir;              /* 符号，已经确认的回原点/刹车方向。 */
    float origin_trust;           /* 0..1，允许闭合位置环的置信度。 */

    float wheel_v_filt;           /* m/s，原始共模轮速快速低通值。 */
    float body_v_filt;            /* m/s，扣除腿/俯仰运动后的机体速度。 */
    float simple_imu_v;           /* m/s，历史纯惯性速度积分器。 */
    float simple_wheel_v_limited; /* m/s，控制用斜率限制轮速。 */
    int simple_wheel_v_ready;     /* 布尔值，轮速斜率限制器是否已有首帧。 */
    float imu_acc_filt;           /* m/s^2，低通后的机体前向惯性加速度。 */
    float drive_acc_filt;         /* m/s^2，低通后的驱动加速度估计。 */
    int filter_ready;             /* 布尔值，常用快速滤波器是否初始化。 */
    float wheel_s_prev;           /* m，上一帧轮路径样本，用于差分检查。 */
    int wheel_s_ready;            /* 布尔值，上一帧轮路径样本是否有效。 */
    float acc_bias;               /* m/s^2，扩展卡尔曼估计的前向加速度偏置。 */
    float P[3][3];                /* 扩展卡尔曼协方差矩阵。 */

    float neutral;                /* rad，速度/原点环给出的目标俯仰角。 */
    float origin_creep;           /* 布尔浮点，近场直接轮扭矩爬行模式标志。 */
    float manual_v;               /* m/s，GUI 手动驾驶当前（斜坡化）前向速度。 */
    float manual_v_target;        /* m/s，GUI 手动驾驶目标前向速度。 */
    float manual_yaw;             /* rad/s，GUI 手动驾驶当前（斜坡化）偏航角速度。 */
    float manual_yaw_target;      /* rad/s，GUI 手动驾驶目标偏航角速度。 */
    int manual_active;            /* 布尔值，GUI 手动驾驶模式标志。 */
    int manual_stop_pending;
    float manual_stop_calm_t;
    int manual_spin_active;       /* 布尔值，当前指令是原地小陀螺。 */
    int spin_rate_external;       /* Caller supplies an already ramped physical rad/s reference. */
    int turn_full_range;          /* Explicit residual-policy full-travel turn mode. */
    float hold_pos;               /* m，停车保持点: 手动指令移动它，空闲时跟随车身。 */
    int post_manual;              /* 布尔值，松手后停车刹车模式标志。 */
    int post_spin;                /* 布尔值，松开纯转向后保持腿摆配平。 */
    int post_manual_rest;         /* 布尔值，松手刹车滞回: 已停稳不再刹。 */
    float runaway_t;              /* s，无指令跑飞持续计时(安全切断)。 */
    float pitch_i;                /* rad*s，俯仰静差积分(拉回残倾角)。 */
    float post_manual_t;          /* s，停车刹车模式累计时间。 */
    float manual_neutral;         /* rad，手动目标俯仰(带变化率限幅)。 */
    float manual_v_i;             /* m*s，手动速度误差积分，消除巡航静差。 */
    int odom_rest_lock;           /* 布尔值，静止锁定里程计(防 rocking 积分)。 */
    int fall_score;               /* 采样数，侧翻判据滞回计数。 */
    int fallen;                   /* 布尔值，已判定摔倒，轮扭矩停机。 */
    float pos_i;                  /* m*s，慢速里程计位置积分。 */
    float leg_swing;              /* rad，辅助回原点的腿摆偏置。 */
    float leg_swing_diff;         /* m，左右腿摆差动(叉腿): 正=左腿前摆/右腿后摆，用于原地转向。 */
    int manual_swing_seeded;      /* 布尔值，进手动时摆腿参考是否已吸附到当前摆位。 */
    float manual_swing_offset;    /* m，进手动时当前摆位相对名义的偏移(种子)。 */
    float leg_pos_i;              /* m*s，腿摆位置误差积分(消除慢速静差漂移)。 */
    float leg_extend;             /* v3: m，正值收腿，负值伸腿。 */
    float leg_extend_manual;      /* rad，GUI E/Q 手动腿长目标，正=缩短/降低。 */
    float leg_side_extend;        /* v3: m，正值收左腿、伸右腿。 */
    float obstacle_roll_hold;     /* s，单侧支撑扰动保持时间。 */
    float obstacle_retract;       /* m，额外共模收腿，恢复时缓慢释放。 */
    float obstacle_wheel_hold[2];
    float obstacle_leg_retract[2]; /* m, additional retraction on each side. */
    float roll_rate_filt;         /* rad/s，roll 角速度低通(用于 roll 稳定 D 项)。 */
    float roll_i;                 /* rad，roll 稳定积分项(消除单侧负载静差)。 */
    float roll_allocation_error;  /* Requested minus achievable differential, m. */
    float leg_len_dbg;            /* rad，实测虚拟腿长(与辨识状态同定义)。 */
    float leg_trim_ref;           /* m，限制变化率后的腿摆配平参考。 */
    int leg_trim_ready;
    float short_sw_blend;         /* 0..1，短腿腿摆冻结/倾角接管权重。 */
    float idle_calm_t;            /* s，空闲低速累计时间，用于吸附保持点。 */
    int jump_active;              /* 跳跃时序进行中 */
    float jump_t;                 /* s，跳跃时序时间 */
    float last_pitch;             /* rad，最近一次姿态，用于起跳门槛 */
    float last_roll;              /* rad */
    int self_right_active;        /* 侧翻自救进行中(用腿推回正) */
    float self_right_t;           /* s，自救脉冲计时 */
    float drive_tau_bias;         /* Nm，历史平移动作直接扭矩偏置。 */

    float obs_phase_dbg;          /* 阶段编号，观察器/恢复逻辑日志读数。 */
    float return_score_dbg;       /* 评分，正在回原点的证据。 */
    float capture_score_dbg;      /* 评分，截获/刹车状态证据。 */
    float odom_forward_dbg;       /* m，日志用前向里程计投影。 */
    float wheel_trust_dbg;        /* 0..1，编码器速度可信度。 */
    float pos_trust_dbg;          /* 0..1，里程计积分可信度。 */
    float airborne_dbg;           /* 浮点布尔值，滞回后的离地状态。 */
    float impact_dbg;             /* 浮点布尔值，滞回后的冲击状态。 */
    float slip_dbg;               /* 浮点布尔值，滞回后的打滑状态。 */
    float raw_airborne_dbg;       /* 浮点布尔值，瞬时离地判据。 */
    float raw_impact_dbg;         /* 浮点布尔值，瞬时冲击判据。 */
    float raw_slip_dbg;           /* 浮点布尔值，瞬时打滑判据。 */
    float raw_zupt_dbg;           /* 浮点布尔值，瞬时零速更新判据。 */
    float support_dbg;            /* 0..1，惯性/腿负载综合支撑置信度。 */
    float leg_load_dbg;           /* 执行器单位，四腿平均绝对负载。 */
    float contact_mode_dbg;       /* CONTACT_* 枚举值，转成浮点供日志读取。 */
    float side_support_balance;   /* 0..1，左右支撑负载均衡程度。 */
    float wheel_acc_dbg;          /* m/s^2，滤波后的轮加速度。 */
    float imu_for_slip_dbg;       /* m/s^2，用于打滑残差的惯性加速度。 */
    float wheel_imu_res_dbg;      /* m/s^2，|轮加速度 - 惯性加速度| 打滑残差。 */
    float leg_kin_v_dbg;          /* m/s，腿运动造成的轮速分量。 */
    float pitch_kin_v_dbg;        /* m/s，俯仰转动造成的轮速分量。 */
    float wheel_body_v_dbg;       /* m/s，补偿后的机体前向轮速。 */

    int airborne_score;           /* 采样数，离地检测滞回计数。 */
    int impact_score;             /* 采样数，冲击检测滞回计数。 */
    int slip_score;               /* 采样数，打滑检测滞回计数。 */
    SlipObserver slip_observer;
    float slip_blend, slip_yaw_blend, slip_inertial_v;
    float slip_gyro_yaw_prev, slip_gyro_roll_prev;
    int slip_gyro_ready;
    int zupt_score;               /* 采样数，零速更新滞回计数。 */
    int grip_score;               /* 采样数，稳定抓地滞回计数。 */

    float speed_elapsed;          /* s，100 Hz 速度/里程计环累计时间。 */
    float yaw_elapsed;            /* s，50 Hz 偏航环累计时间。 */
    float yaw_held;                /* Nm，偏航差动扭矩目标(50Hz 更新)。 */
    float yaw_out;                 /* Nm，经输出斜率限制后实际下发的偏航扭矩。 */
    float yaw_raw_prev;           /* rad，上一帧原始偏航角，用于解包络。 */
    float yaw_zero;               /* rad，平衡启动时捕获的偏航参考。 */
    float yaw_cont;               /* rad，连续解包络后的偏航位移。 */
    float yaw_err_dbg;            /* rad，导出到日志的偏航误差。 */
    float yaw_i;                  /* rad*s，带限幅的偏航积分项。 */
    float yaw_rate_filt;          /* rad/s，偏航角速度一阶低通(防差速振荡) */
    int yaw_ready;                /* 布尔值，偏航参考是否已初始化。 */

    float run_time;               /* s，平衡模式开启后的运行时间。 */
    float tau_cmd;                /* Nm，斜率限制后的共模轮平衡扭矩。 */
    float pitch_rate_filt;        /* rad/s，可选低通后的俯仰角速度。 */
    float wheel_capture_dir;      /* 符号，启用轮速截获时的方向。 */
    float wheel_capture_t;        /* s，轮速截获模式累计时间。 */
    int pitch_rate_ready;         /* 布尔值，俯仰角速度滤波器是否初始化。 */

    float wheel_thrash;           /* m/s，轮速高频残差指标。 */
    float wheel_fir;              /* m/s，匹配 FIR 低频轮速。 */
    float imu_thrash;             /* m/s^2，惯性加速度高频残差指标。 */
    float fir_wheel[FIR_N];       /* 轮速 FIR 输入环形缓冲区。 */
    float fir_imu[FIR_N];         /* 惯性加速度 FIR 输入环形缓冲区。 */
    float fir_drive[FIR_N];       /* 驱动加速度 FIR 输入环形缓冲区。 */
    int fir_idx;                  /* 匹配 FIR 共用的最新样本下标。 */
    int fir_fill;                 /* FIR 预热完成前已有的有效样本数。 */
} BalanceRuntime;

/* 原点窗口判定结果: 当前采样是否允许做位置积分/回正/释放。 */
typedef struct {
    int track_ok;                 /* 布尔值，当前采样是否允许短窗口积分前向位置。 */
    int return_ok;                /* 布尔值，当前采样是否允许闭合原点回正控制。 */
    int release_ok;               /* 布尔值，已经回到很小误差，可释放位置记忆。 */
    float pos_err;                /* m，扣除死区后的前向位置误差。 */
    float trust_ref;              /* 0..1，给调试显示用的目标可信度。 */
} SimpleOriginWindow;

/* 全局平衡状态对象: 由 balance_core.c 定义，其余控制环文件共享。 */
extern BalanceRuntime g_balance;

/* 腿摆名义角(rad): 等效静态质心标定点，由 balance_core.c 定义。 */
extern float balance_sw_nom(void);

/* 当前实际腿长对应的静态摆角配平偏差(m)。 */
extern float leg_trim_dev(float leg_len);

/* 手动前后驱动的腿摆权限窗口(m): 窗口内腿摆速度环负责 W/S/A/D 的前后
 * 运动, 轮上速度阻尼让位; 窗口外视为腿长读数异常, 关闭前后驱动。 */
extern float balance_leg_drive_min(void);
extern float balance_leg_drive_max(void);

/*
 * BalanceCtx - balance_update 各段落之间共享的输入/中间量
 * ----------------------------------------------------------------
 * balance_core.c 的 balance_update() 拆成多个段落函数后，段落之间需要
 * 传递的“本次控制步”数据统一放在这个结构体里:
 *   - 传感器原始输入(由入口拆包)
 *   - 段落计算出的中间量(供后续段落使用)
 *   - 输出指针(由入口传入)
 */
typedef struct {
    /* ---- 传感器输入(入口拆包) ---- */
    float pitch, pitch_rate, roll, roll_rate;   /* 姿态(rad)与角速度(rad/s) */
    float lw_vel, rw_vel;                       /* 左右轮编码器角速度(rad/s) */
    float acc_mag, acc_x, acc_z;                /* 惯性测量 */
    float yaw_ang, yaw_rate;                    /* 偏航角与角速度 */
    float lw_pos, rw_pos;                       /* 左右轮编码器位置 */
    float lf_pos, lr_pos, rf_pos, rr_pos;       /* 四腿关节位置 */
    float lf_tau, lr_tau, rf_tau, rr_tau;       /* 四腿关节力矩 */
    float lw_tau, rw_tau;                       /* 左右轮力矩 */
    float lf_vel_j, lr_vel_j, rf_vel_j, rr_vel_j; /* 四腿关节速度 */
    float dt_f;                                 /* 控制步长(s) */

    /* ---- 入口计算出的公共量 ---- */
    float wheel_v;                /* m/s，共模前进速度(左轮-右轮)*半径/2 */

    /* ---- 简化三环路径: 传感器段落输出(供后续段落使用) ---- */
    float wheel_v_raw;            /* m/s，斜率限制后的轮速 */
    float imu_acc_raw;            /* m/s^2，投影到机体前向的惯性加速度 */
    float leg_kin_v;              /* m/s，腿运动造成的轮速分量 */
    float pitch_kin_v;            /* m/s，俯仰转动造成的轮速分量 */
    float wheel_v_body;           /* m/s，扣除腿/俯仰后的机体轮速 */
    float v_damp_pb;              /* 限幅后的机体速度 */
    float abs_pitch_pb, abs_wheel_pb, abs_body_pb;
    int airborne_pb, impact_pb, slip_pb, zupt_pb;
    float wheel_trust_pb;         /* 轮里程计可信度 0..1 */
    int stable_odom_pb;
    float pos_ctrl_pb;            /* 相对保持点的位置误差 */
    float leg_load_common;
    float ext_damp_ref;
    float yaw_err_pb;             /* 连续偏航误差(rad) */
    float pos_err_pb;             /* 原点位置误差(rad) */

    /* ---- 简化三环路径: 原点段落输出(供俯仰/偏航段使用) ---- */
    SimpleOriginWindow origin_pb;
    float neutral_pb;             /* 目标俯仰角 */
    float tau_cmd_pb;             /* 俯仰扭矩(斜坡限幅前) */
    float yaw_lim_pb;             /* 偏航扭矩限幅 */

    /* ---- 输出指针 ---- */
    float *tau, *yaw, *sw, *spd, *ex;
} BalanceCtx;

/* balance_update 段落函数: 每个文件一个功能段落，全部由 balance_core.c 调用。
 * 段落接口统一接收本次控制步的全部上下文(BalanceCtx)，段落内部只读
 * 传感器输入、读写 g_balance 状态、通过 c->输出指针写回控制量。 */
void balance_step_sensor(BalanceCtx *c);           /* 传感器预处理/滤波/接触状态/里程计信任 */
void balance_step_origin(BalanceCtx *c);           /* 原点窗口/速度规划/手动驾驶/目标俯仰 */
void balance_step_pitch_yaw(BalanceCtx *c);        /* 俯仰PD/偏航环/腿摆/roll稳定 */
void balance_step_legacy(BalanceCtx *c);           /* 旧速度环路径(仅备用配置编译) */
void balance_update_with_imu(const double *s, double dt, float *tau, float *yaw,
                             float *sw, float *spd, float *ex, float acc_y, float gyro_y);

/* balance_update 入口内部的小函数，供段落复用 */
void balance_legacy_yaw_pitch(BalanceCtx *c);      /* 旧路径偏航/腿摆/俯仰PD输出 */

/* ---- balance_util.c 提供的工具函数(保留原静态函数名, 调用点不用改) ---- */
float clampf_local(float x, float lo, float hi);
float decay_to_zero_local(float x, float rate, float dt);
float deadband_position_error(float pos, float deadband);
float wrap_pi_local(float x);
SimpleOriginWindow make_simple_origin_window(float pos, float body_v, float pitch,
                                             float pitch_rate, float imu_acc,
                                             float wheel_trust, int stable_odom,
                                             int slip, int zupt);
void reset_odom_filter(void);
float fir_push(float *buf, float x);
float fir_delayed_raw(const float *buf);

#endif /* BALANCE_INTERNAL_H */
