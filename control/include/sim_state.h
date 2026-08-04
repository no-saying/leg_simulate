#ifndef SIM_STATE_H
#define SIM_STATE_H

/*
 * SensorState 是仿真外壳里的“单帧快照”。
 *
 * 分层约定:
 *   - 姿态、IMU、轮编码器、腿编码器、执行器力矩可以进入控制器。
 *   - body_pos 是 MuJoCo 世界坐标，只用于日志、相机和屏幕轨迹显示。
 *   - balance_* 调试量由 balance.c 产生，回填到这里供日志/GUI 读取。
 */
typedef struct {
    double quat[4];
    double pitch;
    double roll;
    double yaw;
    double yaw_rate;
    double gyro[3];
    double acc[3];
    double acc_mag;

    double body_pos[3];

    double lf_pos, lf_vel;
    double lr_pos, lr_vel;
    double rf_pos, rf_vel;
    double rr_pos, rr_vel;

    double lw_pos, rw_pos;
    double lw_vel, rw_vel;

    double lf_torque, lr_torque;
    double rf_torque, rr_torque;
    double lw_torque, rw_torque;

    double ctrl[6];

    double est_x, est_v, target_pitch;
    double wheel_trust, pos_trust;
    double airborne, impact, slip, support, leg_load, leg_swing, contact_mode;
    double raw_airborne, raw_impact, raw_slip, raw_zupt;
    double wheel_v_filt, wheel_fir, wheel_thrash, imu_thrash;
    double wheel_acc, imu_for_slip, wheel_imu_res;
    double leg_kin_v, pitch_kin_v, wheel_body_v;
    double leg_extend, leg_side_extend;
    double leg_len, leg_extend_manual;
    double obs_phase, return_score, capture_score, odom_forward;
    double side_balance;
    double ekf_s, odom_x, odom_y;
    double yaw_error;

    int step_count;
    int paused;
    double sim_time;
} SensorState;

#endif
