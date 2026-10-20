/*
 * dashcam.c - 行车记录仪日志实现
 * ================================================================
 *
 * CSV 格式以列名表头为准(见 dashcam_open)，新列只允许追加在末尾，
 * 这样 tools/analyze_dashcam.py 等按列名解析的工具保持兼容。
 */
#include "dashcam.h"
#include "control_config.h"
#include "attitude_rates.h"
#include "leg_kin_v3.h"
#include "balance.h"
#include "controllers/supervisor/controller_supervisor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static FILE *g_dash_fp = NULL;
static FILE *g_event_fp = NULL;
static double g_dash_last_t = -1.0;
static double g_dash_interval = 0.05;   /* 采样间隔，可用 DASHCAM_HZ 覆盖。 */

static void dashcam_read_rate(void)
{
    const char *hz = getenv("DASHCAM_HZ");
    if (hz) {
        double h = atof(hz);
        if (h > 0.0 && h <= 2000.0)
            g_dash_interval = 1.0 / h;
    }
}

void dashcam_open(void)
{
    static unsigned int recording = 0;
    unsigned int index = ++recording;
    g_dash_last_t = -1.0;
    char ts[32];
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", tm);

    dashcam_read_rate();

    char path[256];
    snprintf(path, sizeof(path), "logs/dashcam_%s_%ld_%u.csv", ts, (long)getpid(), index);
    g_dash_fp = fopen(path, "w");
    if (g_dash_fp) {
        fprintf(g_dash_fp,
                "sim_t,wall_t,keys,manual,hold_pos,"
                "pit_deg,roll_deg,yaw_deg,pitch_rate,yaw_rate,"
                "lw_vel,rw_vel,wv_ms,body_v,acc_mag,acc_x,acc_z,"
                "bx,by,bz,xh,ox,oy,tp_deg,"
                "sw,le,ld,wheel_trust,pos_trust,"
                "airborne,impact,slip,leg_load,support,contact_mode,"
                "raw_a,raw_i,raw_s,raw_z,wf,wk,pk,thr,ith,wa,ia,wr,leg_len,le_man,"
                "rl_st,rl_a0,rl_a1,rl_ed,rl_sd,rl_oh,"
                "ctrl_lw,ctrl_rw,tau_lw,tau_rw,v_cmd,yaw_cmd,fallen,lqr_mix,"
                "pitch_euler_rate,sw_left_meas,sw_right_meas,pitch_control_rate,"
                "slip_v_imu,slip_v_res,slip_yaw_res,slip_blend,slip_yaw_blend,obstacle_retract,roll_rate_euler,"
                "left_wheel_res,right_wheel_res,left_obstacle,right_obstacle,left_retract,right_retract\n");
    }
    snprintf(path, sizeof(path), "logs/dashcam_%s_%ld_%u_events.log", ts, (long)getpid(), index);
    g_event_fp = fopen(path, "w");
    if (g_event_fp)
        fprintf(g_event_fp, "# sim_t key action wall_t\n");
    printf("[Dashcam] %s (+events)\n", path);
}

void dashcam_close(void)
{
    if (g_dash_fp) { fclose(g_dash_fp); g_dash_fp = NULL; }
    if (g_event_fp) { fclose(g_event_fp); g_event_fp = NULL; }
}

void dashcam_log_event(double sim_t, const char *key, const char *action, double wall_t)
{
    if (!g_event_fp) return;
    fprintf(g_event_fp, "%.3f %s %s %.3f\n", sim_t, key, action, wall_t);
    fflush(g_event_fp);
}

void dashcam_log_row(const SensorState *s, double wall_t, const char *keys,
                     int manual_active, double hold_pos)
{
    if (!g_dash_fp) return;
    if (g_dash_last_t >= 0.0 && s->sim_time - g_dash_last_t < g_dash_interval - 1e-6)
        return;
    g_dash_last_t = s->sim_time;
#if MODEL_V3
    double wv = -(s->lw_vel + s->rw_vel) * 0.5 * WHEEL_RADIUS;
#else
    double wv = (s->lw_vel - s->rw_vel) * 0.5 * WHEEL_RADIUS;
#endif
    double sw_left = s->leg_swing, sw_right = s->leg_swing;
#if MODEL_V3
    float length, swing;
    leg_kin_v3_calibrated_side((float)s->lf_pos, (float)s->lr_pos, 0, &length, &swing, NULL);
    sw_left = swing;
    leg_kin_v3_calibrated_side((float)s->rf_pos, (float)s->rr_pos, 1, &length, &swing, NULL);
    sw_right = swing;
#endif
    fprintf(g_dash_fp,
            "%.3f,%.3f,%s,%d,%.4f,"
            "%.2f,%.2f,%.2f,%.3f,%.3f,"
            "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,"
            "%.4f,%.4f,%.4f,%.3f,%.3f,"
            "%d,%d,%d,%.1f,%.3f,%d,"
            "%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,"
            "%d,%.3f,%.3f,%.3f,%.3f,%.3f,"
            "%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%d,%.3f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%.6f\n",
            s->sim_time, wall_t, keys, manual_active, hold_pos,
            s->pitch * 57.29578, s->roll * 57.29578, s->yaw * 57.29578,
            s->gyro[1], s->yaw_rate,
            s->lw_vel, s->rw_vel, wv, s->est_v,
            s->acc_mag, s->acc[0], s->acc[2],
            s->body_pos[0], s->body_pos[1], s->body_pos[2],
            s->est_x, s->odom_x, s->odom_y, s->target_pitch * 57.29578,
            s->leg_swing, s->leg_extend, s->leg_side_extend,
            s->wheel_trust, s->pos_trust,
            (int)s->airborne, (int)s->impact, (int)s->slip,
            s->leg_load, s->support, (int)s->contact_mode,
            (int)s->raw_airborne, (int)s->raw_impact,
            (int)s->raw_slip, (int)s->raw_zupt,
            s->wheel_v_filt, s->wheel_body_v, s->pitch_kin_v,
            s->wheel_thrash, s->imu_thrash,
            s->wheel_acc, s->imu_for_slip, s->wheel_imu_res,
            s->leg_len, s->leg_extend_manual,
            s->rl_state, s->rl_act0, s->rl_act1,
            s->rl_ext_delta, s->rl_swing_delta, s->rl_obs_h,
            s->ctrl[2], s->ctrl[5], s->lw_torque, s->rw_torque,
            balance_get_manual_v(), balance_get_manual_yaw(), balance_get_fallen(),
            controller_supervisor_lqr_mix(), attitude_pitch_rate(s->roll, s->gyro[1], s->gyro[2]),
            sw_left, sw_right, sensor_pitch_rate(s), balance_get_slip_inertial_v(),
            balance_get_slip_velocity_error(), balance_get_slip_yaw_error(),
            balance_get_slip_blend(), balance_get_slip_yaw_blend(),
            balance_get_obstacle_retract(),
            attitude_roll_rate(s->roll,s->pitch,s->gyro[0],s->gyro[1],s->gyro[2]),
            balance_get_wheel_mismatch(0),balance_get_wheel_mismatch(1),
            balance_get_wheel_obstacle(0),balance_get_wheel_obstacle(1),
            balance_get_obstacle_leg_retract(0),balance_get_obstacle_leg_retract(1));
    fflush(g_dash_fp);
}
