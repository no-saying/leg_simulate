#ifndef IDENT_FRAME_H
#define IDENT_FRAME_H

typedef struct {
    float pitch, pitch_rate, roll, roll_rate;
    float left_wheel_vel, right_wheel_vel;
    float acc_mag, acc_x, acc_z;
    float yaw, yaw_rate;
    float left_wheel_pos, right_wheel_pos;
    float lf_pos, lr_pos, rf_pos, rr_pos;
    float lf_tau, lr_tau, rf_tau, rr_tau;
    float left_wheel_tau, right_wheel_tau;
    float lf_vel, lr_vel, rf_vel, rr_vel;
    float acc_y;
    double quat[4];
    double body_pos[3];
    double body_vel[3];
    double sim_time;
    int step_count;
} IdentFrame;

#endif
