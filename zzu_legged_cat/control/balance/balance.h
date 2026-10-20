/*
 * balance.h - 平衡控制器: 俯仰平衡 + 速度环 + 偏航环
 */
#ifndef BALANCE_H
#define BALANCE_H

#include "control_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void balance_init(void);
void balance_reset_state(void);   /* 回合级重置: 恢复全部内部状态到初始值 */
void balance_toggle(void);     // B键
void balance_tune_neutral(void);       // N键: 整定中立角
void balance_tune_off(void);           // 关闭整定
void balance_adj_neutral(float delta); // M/,键: 手动微调
int  balance_tuning(void);             // 是否正在整定
int  balance_need_reset(void);         // 返回1=需悬空再落下
int  balance_active(void);
int  balance_get_manual_active(void);  // 1=GUI 手动驾驶模式激活
int  balance_get_post_manual(void);    // 1=松手后停车刹车模式
float balance_get_manual_v(void);      // 当前手动速度指令(斜坡化后)，m/s
float balance_get_manual_v_target(void); // 手动速度目标(未经过斜坡/安全门控)，m/s
float balance_get_manual_yaw(void);    // 当前手动转向指令(斜坡化后)，rad/s
int  balance_get_manual_spin(void);    /* Spin intent, independent of height rate reduction. */
float balance_get_hold_pos(void);      // 当前停车保持点位置，m
float balance_get_neutral(void);      // 获取当前中立角
int  balance_get_fallen(void);        // 1=已判定侧翻，轮扭矩已停机
float balance_get_wheel_trust(void);  // 调试: 编码器测量可信度
float balance_get_pos_trust(void);    // 调试: 里程计积分可信度
float balance_get_airborne(void);     // 调试: 滞回后的离地标志
float balance_get_impact(void);       // 调试: 最近冲击标志
float balance_get_slip(void);         // 调试: 滞回后的打滑标志
float balance_get_raw_airborne(void); // 调试: 瞬时离地判据
float balance_get_raw_impact(void);   // 调试: 瞬时冲击判据
float balance_get_raw_slip(void);     // 调试: 瞬时打滑判据
float balance_get_raw_zupt(void);     // 调试: 瞬时零速更新判据
float balance_get_support(void);      // 调试: 腿/惯性支撑置信度
float balance_get_leg_load(void);     // 调试: 平均绝对腿执行器负载
float balance_get_leg_swing(void);    // 调试: 腿摆指令偏置
float balance_get_leg_swing_diff(void); // 叉腿转向: 左右腿摆差动指令(m)
float balance_get_leg_extend(void);   // 调试: 共模腿伸长指令
float balance_get_obstacle_retract(void);
float balance_get_obstacle_leg_retract(int right);
float balance_get_wheel_mismatch(int right);
int balance_get_wheel_obstacle(int right);
void balance_set_roll_allocation_error(float error);
void balance_set_leg_extend_manual(float v); // GUI E/Q: 手动腿长目标
float balance_get_leg_extend_manual(void);   // 调试: 当前手动腿长目标
float balance_get_leg_len(void);      // 调试: 实测虚拟腿长(与辨识状态同定义)
void balance_trigger_jump(void);      // GUI SPACE: 触发跳跃时序
int  balance_get_jump_phase(void);    // 0=无,1=下蹲,2=蹬伸,3=空中,4=落地
int  balance_get_self_right_active(void); // 1=侧翻自救中
int  balance_get_self_right_push(void);   // 1=自救脉冲推腿中(0=松开)
void balance_trigger_self_right(void);    // GUI H: 手动触发侧翻自救
float balance_get_leg_side_extend(void); // 调试: 左右差动腿伸长指令
void balance_set_leg_side_extend(float v); // 调试/试验: 直接注入左右差动腿伸长
float balance_get_obs_phase(void);    // 调试: 仅观察器阶段，不切控制
float balance_get_return_score(void); // 调试: 回原点证据评分
float balance_get_capture_score(void);// 调试: 截获/刹车证据评分
float balance_get_odom_forward(void); // 调试: 投影到当前偏航方向的里程计
float balance_get_contact_mode(void); // 调试: 0=离地,1=冲击,2=打滑,3=粘着,4=零速
float balance_get_side_balance(void); // 调试: 左右支撑均衡度，0=单侧,1=均衡
float balance_get_ekf_s(void);        // 调试: 扩展卡尔曼路径位移
float balance_get_odom_x(void);       // 调试: 偏航投影里程计 x
float balance_get_odom_y(void);       // 调试: 偏航投影里程计 y
float balance_get_wheel_v_filt(void); // 调试: 低延迟滤波轮速
float balance_get_wheel_fir(void);    // 调试: 匹配有限脉冲响应滤波轮速
float balance_get_wheel_thrash(void); // 调试: 轮速高频残差
float balance_get_imu_thrash(void);   // 调试: 惯性高频残差
float balance_get_wheel_acc(void);    // 调试: 滤波轮加速度
float balance_get_imu_for_slip(void); // 调试: 扣除偏置后的打滑用惯性加速度
float balance_get_wheel_imu_res(void);// 调试: |轮加速度-惯性加速度| 打滑残差
float balance_get_leg_kin_v(void);    // 调试: 腿运动带来的轮速修正量
float balance_get_pitch_kin_v(void);  // 调试: 俯仰运动带来的轮速修正量
float balance_get_wheel_body_v(void); // 调试: 补偿后的机体前向轮速
float balance_get_slip_inertial_v(void);
float balance_get_slip_velocity_error(void);
float balance_get_slip_yaw_error(void);
float balance_get_slip_blend(void);
float balance_get_slip_yaw_blend(void);
float balance_get_yaw_error(void);    // 调试: 偏航控制器使用的连续偏航误差

/*
 * BalanceSensorFrame 是 balance.c 的唯一推荐输入接口。
 *
 * 这里刻意只放真实机器人能直接获得的量:
 *   - 惯性测量姿态/角速度/加速度
 *   - 左右轮编码器位置和速度
 *   - 四个腿关节编码器位置/速度
 *   - 六个执行器反馈力矩
 *
 * 禁止把 MuJoCo 世界坐标、真实机体速度等“仿真答案”放进这个结构体。
 */
typedef struct {
    float pitch;
    float pitch_rate;
    float roll;
    float roll_rate;
    float left_wheel_vel;
    float right_wheel_vel;
    float acc_mag;
    float acc_x;
    float acc_z;
    float yaw;
    float yaw_rate;
    float left_wheel_pos;
    float right_wheel_pos;
    float lf_pos, lr_pos, rf_pos, rr_pos;
    float lf_tau, lr_tau, rf_tau, rr_tau;
    float left_wheel_tau;
    float right_wheel_tau;
    float lf_vel, lr_vel, rf_vel, rr_vel;
    float acc_y, gyro_y;
} BalanceSensorFrame;

typedef struct {
    float wheel_tau;   /* 共模轮扭矩，正负号由 controller 的轮轴混控处理。 */
    float yaw_tau;     /* 偏航差动扭矩。 */
    float leg_swing;   /* 四腿虚拟摆角参考。 */
    float speed_ref;   /* 调试/日志: 当前前向速度估计。 */
    float position;    /* 调试/日志: 当前里程计路径位移估计。 */
} BalanceOutput;

// 更新频率由 balance.c 内部分频:
// - 俯仰平衡环: 每个物理步，模型步长为 0.001 秒时等效 1 kHz，大于 500 Hz
// - 速度/里程计环: 100 Hz
// - 偏航环: 50 Hz
void balance_update_frame(const BalanceSensorFrame *frame, double dt, BalanceOutput *out);

// 兼容旧实验代码的数组接口。新代码请使用 balance_update_frame()。
void balance_update(const double *s, double dt,
                    float *wheel_tau_out, float *yaw_tau_out,
                    float *leg_swing_out, float *speed_ref_out,
                    float *extend_out);

/*
 * GUI 手动驾驶接口:
 *   speed_ref   : W/S 目标前向速度，单位 m/s（0=不前后移动）。
 *   yaw_rate_ref: A/D 目标偏航角速度，单位 rad/s（0=不转向）。
 *   active      : 1=进入手动驾驶，原点回程/里程计位置记忆暂停；
 *                 0=退回自动回原点控制。
 * 手动驾驶仍走平衡核心的速度环和偏航环，不会绕过俯仰稳定。
 */
void balance_set_manual(float speed_ref, float yaw_rate_ref, int active);
void balance_set_spin_rate_reference(float yaw_rate_ref, int active);
void balance_set_turn_full_range(int enabled);

#ifdef __cplusplus
}
#endif
#endif
