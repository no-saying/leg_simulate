#include "telemetry.h"

#include <math.h>
#include <stdio.h>

void telemetry_print_frame(const SensorState *s, double fps)
{
    double cp = cos(s->pitch);
    double sp = sin(s->pitch);
    double imu_ax = cp * s->acc[0] + sp * s->acc[2];
    double wheel_v_m = ((s->lw_vel - s->rw_vel) * 0.5) * 0.058;

    printf("[t=%.2f] pit=%+.1fd roll=%+.1fd yaw=%+.1fd ye=%+.1fd | "
           "gy=%.1f yr=%.1f | L=%.0f R=%.0f v=%.1f | "
           "tL=%+.0f tR=%+.0f | acc=%.0f ax=%+.2f wv=%+.2f | "
           "bx=%+.2f by=%+.2f bz=%+.2f xh=%+.2f es=%+.2f ox=%+.2f oy=%+.2f "
           "of=%+.2f vh=%+.2f tp=%+.1fd sw=%+.3f le=%+.3f llen=%+.3f ld=%+.3f "
           "ph=%.0f rs=%.2f cs=%.2f cm=%.0f lc=%.2f ll=%.1f sb=%.2f "
           "wt=%.2f pt=%.2f ab=%.0f im=%.0f sl=%.0f "
           "raw[a%.0f i%.0f s%.0f z%.0f] wf=%+.2f fir=%+.2f wk=%+.2f "
           "lk=%+.2f pk=%+.2f thr=%.2f ith=%.2f wa=%+.1f ia=%+.1f wr=%.1f | FPS=%.0f\n",
           s->sim_time, s->pitch * 57.3, s->roll * 57.3, s->yaw * 57.3,
           s->yaw_error * 57.3,
           s->gyro[1], s->yaw_rate,
           s->lw_vel, -s->rw_vel, (s->lw_vel - s->rw_vel) * 0.5,
           s->ctrl[2] * 10.0, -s->ctrl[5] * 10.0, s->acc_mag, imu_ax, wheel_v_m,
           s->body_pos[0], s->body_pos[1], s->body_pos[2],
           s->est_x, s->ekf_s, s->odom_x, s->odom_y, s->odom_forward,
           s->est_v, s->target_pitch * 57.3, s->leg_swing,
           s->leg_extend, s->leg_len, s->leg_side_extend,
           s->obs_phase, s->return_score, s->capture_score,
           s->contact_mode, s->support, s->leg_load, s->side_balance,
           s->wheel_trust, s->pos_trust,
           s->airborne, s->impact, s->slip,
           s->raw_airborne, s->raw_impact, s->raw_slip, s->raw_zupt,
           s->wheel_v_filt, s->wheel_fir, s->wheel_body_v, s->leg_kin_v,
           s->pitch_kin_v, s->wheel_thrash, s->imu_thrash,
           s->wheel_acc, s->imu_for_slip, s->wheel_imu_res, fps);
    fflush(stdout);
}
