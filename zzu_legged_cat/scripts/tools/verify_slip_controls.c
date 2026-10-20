#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_internal.h"
#include "balance.h"
#include "control_config.h"
#include "vel_kf.h"
#include "leg_kin_v3.h"
#include "controllers/supervisor/controller_supervisor.h"

static void move_ball(mjModel *m, mjData *d, int b, double x, double y)
{
    int j = m->body_jntadr[b], a = m->jnt_qposadr[j];
    d->qpos[a] = x;
    d->qpos[a + 1] = y;
    d->qpos[a + 2] = -0.131;
    d->qpos[a + 3] = 1.0;
    for (int k = 4; k < 7; ++k) d->qpos[a + k] = 0.0;
    for (int k = 0; k < 6; ++k) d->qvel[m->jnt_dofadr[j] + k] = 0.0;
}

static double contact_speed(const mjModel *m, const mjData *d, const mjContact *c)
{
    double relative[3] = {0};
    for (int side = 0; side < 2; ++side) {
        int g = side ? c->geom2 : c->geom1;
        double v[6];
        mj_objectVelocity(m, d, mjOBJ_GEOM, g, v, 0);
        const mjtNum *p = d->geom_xpos + 3*g;
        double r[3] = {c->pos[0] - p[0], c->pos[1] - p[1], c->pos[2] - p[2]};
        double sign = side ? -1 : 1;
        relative[0] += sign * (v[3] + v[1]*r[2] - v[2]*r[1]);
        relative[1] += sign * (v[4] + v[2]*r[0] - v[0]*r[2]);
        relative[2] += sign * (v[5] + v[0]*r[1] - v[1]*r[0]);
    }
    double normal = 0, speed2 = 0;
    for (int k = 0; k < 3; ++k) {
        normal += relative[k] * c->frame[k];
        speed2 += relative[k] * relative[k];
    }
    return sqrt(fmax(0.0, speed2 - normal*normal));
}

int main(int argc, char **argv)
{
    const char *command = argc > 1 ? argv[1] : "W";
    double ext = argc > 2 ? atof(argv[2]) : 0;
    const char *terrain = argc > 3 ? argv[3] : "flat";
    int slope = !strcmp(terrain, "slope");
    double slope_deg = slope && argc > 4 ? atof(argv[4]) : 0;
    if (slope && (!isfinite(slope_deg) || fabs(slope_deg) > 30)) return 2;
    int single = !strcmp(terrain, "leftball") ? 1 : !strcmp(terrain, "rightball") ? 2 : 0;
    int balls = !strcmp(terrain, "ball") || single;
    int low_mu = argc > 3 && !strcmp(argv[3], "ice");
    int spin = !strcmp(command, "spin") || !strcmp(command, "reverse");
    float v = !strcmp(command, "W") ? -SIMPLE_MANUAL_V_MAX :
              !strcmp(command, "S") ? SIMPLE_MANUAL_V_MAX : 0;
    float yaw = spin ? (!strcmp(command, "reverse") ? -1 : 1) * SIMPLE_MANUAL_SPIN_YAW_RATE :
                !strcmp(command, "A") ? SIMPLE_MANUAL_YAW_RATE :
                !strcmp(command, "D") ? -SIMPLE_MANUAL_YAW_RATE : 0;
    if (!strcmp(command, "curve")) { v = -SIMPLE_MANUAL_V_MAX; yaw = SIMPLE_MANUAL_YAW_RATE; }
    Simulation sim;
    assert(simulation_load(&sim, "model/v3/COD-2026RoboMaster-Balance-v3.xml"));
    mjModel *m = sim.model;
    mjData *d = sim.data;
    int big[2] = {-1,-1};
    for (int b = 1; b < m->nbody; ++b) {
        const char *name = mj_id2name(m, mjOBJ_BODY, b);
        if (name && (strstr(name, "test_ball_") == name || strstr(name, "large_ball_") == name)) {
            move_ball(m, d, b, slope ? 0 : 100 + b, slope ? 100 + b : 100);
            if (!strcmp(name, "large_ball_01")) big[0] = b;
            if (!strcmp(name, "large_ball_02")) big[1] = b;
        }
    }
    int wheel[2] = {mj_name2id(m, mjOBJ_GEOM, "L_Wheel_mesh_1"),
                    mj_name2id(m, mjOBJ_GEOM, "R_Wheel_mesh_1")};
    int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
    if (slope) {
        /* Rotate both terrain and the initial robot about the ground origin. */
        double angle = -slope_deg*M_PI/180;
        mjtNum q[4] = {cos(angle/2),0,sin(angle/2),0};
        mjtNum p[3], rotated[3], original[4];
        mju_copy(m->geom_quat+4*floor,q,4);
        m->geom_sameframe[floor] = mjSAMEFRAME_NONE;
        for (int i = 0; i < 3; ++i) p[i] = d->qpos[i]-m->geom_pos[3*floor+i];
        mju_rotVecQuat(rotated,p,q);
        for (int i = 0; i < 3; ++i) d->qpos[i] = rotated[i]+m->geom_pos[3*floor+i];
        mju_copy(original,d->qpos+3,4);
        mju_mulQuat(d->qpos+3,q,original);
    }
    mj_forward(m, d);
    if (slope) {
        const mjtNum *r = d->geom_xmat+9*floor;
        assert(fabs(r[2]+sin(slope_deg*M_PI/180)) < 1e-10);
        assert(fabs(r[8]-cos(slope_deg*M_PI/180)) < 1e-10);
        printf("TERRAIN slope_deg=%.1f normal=%.6f,%.6f,%.6f\n",slope_deg,r[2],r[5],r[8]);
    }
    balance_reset_state();
    char path[256];
    snprintf(path, sizeof(path), "logs/manual_verification/slip_check_%s_%+.3f_%s.csv",
             command, ext, terrain);
    if (slope)
        snprintf(path,sizeof(path),"logs/manual_verification/slip_check_%s_%+.3f_slope_%+.1f.csv",
                 command,ext,slope_deg);
    FILE *fp = fopen(path, "w");
    assert(fp);
    fputs("time,true_v,control_v,wheel_v,imu_v,v_res,yaw_res,slip,raw_slip,yaw_rate,pitch,roll,ball_contact,contact_speed,mu,slip_blend,slip_yaw_blend,leg_l,leg_r,leg_common,leg_diff,retract,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,left_res,right_res,left_obstacle,right_obstacle,left_retract,right_retract,contact_mask,world_x,world_y,heading,controller_mode,fallen\n", fp);
    int placed = 0, ball_steps = 0, slip_steps = 0, raw_steps = 0, truth_steps = 0, detected_steps = 0;
    double max_pitch = 0, max_roll = 0, yaw_total = 0, prev_yaw = 0;
    double recovery_pitch = 0, recovery_roll = 0;
    double start[2] = {0}, end[2] = {0}, max_excursion = 0;
    double estimate_error = 0, wheel_error = 0;
    double max_retract = 0;
    double max_side_retract[2] = {0}, first_contact[2] = {-1,-1}, first_trigger[2] = {-1,-1};
    double start_yaw = 0, max_cross_track = 0, final_cross_track = 0;
    double release[2] = {0}, release_max = 0, release_net = 0, final_speed = 0;
    double all_pitch = 0, all_roll = 0, overlap_time = 0;
    int released = 0, upright = 1;
    double fallen_time = -1, initial_x = d->qpos[0], initial_y = d->qpos[1];
    double settling_excursion = 0, spin_path = 0, previous_xy[2] = {d->qpos[0],d->qpos[1]};
    int full_lqr_steps = 0;
    for (int step = 0; step < 20000; ++step) {
        double t = d->time;
        maybe_auto_enable_balance(&sim);
        balance_set_leg_extend_manual(t >= 3 ? (float)ext : 0);
        balance_set_manual(t >= 10 && t < 15 ? v : 0, t >= 10 && t < 15 ? yaw : 0,
                           t >= 10 && t < 15 && (v != 0 || yaw != 0));
        if (!placed && t >= 10) {
            start[0] = d->qpos[0]; start[1] = d->qpos[1];
            start_yaw = sim.state.yaw;
            if (balls) {
                double c = cos(sim.state.yaw), s = sin(sim.state.yaw);
                for (int i = 0; i < 2; ++i) {
                    if (single && i != single - 1) continue;
                    const mjtNum *p = d->geom_xpos + 3*wheel[i];
                    double x, y;
                    if (spin) {
                        double angle = yaw > 0 ? 0.8 : -0.8;
                        double rx = p[0]-start[0], ry = p[1]-start[1];
                        x = start[0]+cos(angle)*rx-sin(angle)*ry;
                        y = start[1]+sin(angle)*rx+cos(angle)*ry;
                    } else {
                        double distance = v < 0 ? -0.35 : 0.35;
                        x = p[0]+distance*c; y = p[1]+distance*s;
                    }
                    move_ball(m, d, big[i], x, y);
                }
                mj_forward(m, d);
            }
            placed = 1;
        }
        if (!released && t >= 15) {
            release[0] = d->qpos[0]; release[1] = d->qpos[1];
            released = 1;
        }
        double mu = low_mu && t >= 11 && t < 12 ? 0.02 : 1.0;
        m->geom_friction[3*floor] = mu;
        if (low_mu) {
            m->geom_friction[3*wheel[0]] = mu;
            m->geom_friction[3*wheel[1]] = mu;
        }
        assert(simulation_step(&sim, 0));
        SensorState *s = &sim.state;
        if (balance_get_fallen() && fallen_time < 0) fallen_time = t;
        if (t < 10) settling_excursion = fmax(settling_excursion,
            hypot(d->qpos[0]-initial_x,d->qpos[1]-initial_y));
        if (slope && (fabs(s->pitch) >= M_PI/2 || fabs(s->roll) >= M_PI/2)) upright = 0;
        double actual_v = d->qvel[0]*cos(s->yaw) + d->qvel[1]*sin(s->yaw);
        double tangent_speed = 0;
        int ball_contact = 0;
        int contact_mask = 0;
        for (int i = 0; i < d->ncon; ++i) {
            mjContact *contact = d->contact+i;
            int other = contact->geom1 == wheel[0] || contact->geom1 == wheel[1] ? contact->geom2 :
                        contact->geom2 == wheel[0] || contact->geom2 == wheel[1] ? contact->geom1 : -1;
            if (other < 0) continue;
            tangent_speed = fmax(tangent_speed, contact_speed(m,d,contact));
            const char *name = mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[other]);
            if (name && strstr(name,"large_ball_") == name) {
                ball_contact = 1;
                int side = contact->geom1 == wheel[0] || contact->geom2 == wheel[0] ? 0 : 1;
                contact_mask |= 1 << side;
                if (t >= 10 && first_contact[side] < 0) first_contact[side] = t;
            }
        }
        if (t >= 10) {
            if (contact_mask == 3) overlap_time += m->opt.timestep;
            double cross_track = -(d->qpos[0]-start[0])*sin(start_yaw) +
                                  (d->qpos[1]-start[1])*cos(start_yaw);
            max_cross_track = fmax(max_cross_track, fabs(cross_track));
            final_cross_track = cross_track;
            all_pitch = fmax(all_pitch, fabs(s->pitch)*180/M_PI);
            all_roll = fmax(all_roll, fabs(s->roll)*180/M_PI);
            if (fabs(s->pitch) >= M_PI/2 || fabs(s->roll) >= M_PI/2) upright = 0;
        }
        if (released) {
            release_net = hypot(d->qpos[0]-release[0], d->qpos[1]-release[1]);
            release_max = fmax(release_max, release_net);
            final_speed = hypot(d->qvel[0], d->qvel[1]);
        }
        if (t >= 10 && t < 15) {
            spin_path += hypot(d->qpos[0]-previous_xy[0],d->qpos[1]-previous_xy[1]);
            full_lqr_steps += controller_supervisor_mode() == CTRL_MODE_LQR;
            ball_steps += ball_contact;
            slip_steps += s->slip != 0;
            raw_steps += s->raw_slip != 0;
            if (tangent_speed > 0.25) { truth_steps++; detected_steps += s->slip != 0; }
            double delta = remainder(s->yaw-prev_yaw, 2*M_PI);
            yaw_total += delta;
            max_pitch = fmax(max_pitch,fabs(s->pitch)*180/M_PI);
            max_roll = fmax(max_roll,fabs(s->roll)*180/M_PI);
            max_excursion = fmax(max_excursion,hypot(d->qpos[0]-start[0],d->qpos[1]-start[1]));
            estimate_error += fabs(s->est_v-actual_v);
            wheel_error += fabs(s->wheel_body_v-actual_v);
            max_retract = fmax(max_retract,balance_get_obstacle_retract());
            for (int i = 0; i < 2; ++i) {
                max_side_retract[i] = fmax(max_side_retract[i],balance_get_obstacle_leg_retract(i));
                if (balance_get_wheel_obstacle(i) && first_trigger[i] < 0) first_trigger[i] = t;
            }
            end[0]=d->qpos[0]; end[1]=d->qpos[1];
        }
        previous_xy[0] = d->qpos[0]; previous_xy[1] = d->qpos[1];
        prev_yaw=s->yaw;
        if (t >= 17) {
            recovery_pitch = fmax(recovery_pitch,fabs(s->pitch)*180/M_PI);
            recovery_roll = fmax(recovery_roll,fabs(s->roll)*180/M_PI);
        }
        if (step%5 == 0) {
            float sw, ll, lr;
            leg_kin_v3_forward_calibrated(s->lf_pos,s->lr_pos,s->rf_pos,s->rr_pos,&sw,&ll,&lr);
            fprintf(fp,"%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%.4f,%.4f,%d,%.6f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
                    t,actual_v,s->est_v,s->wheel_body_v,balance_get_slip_inertial_v(),
                    balance_get_slip_velocity_error(),balance_get_slip_yaw_error(),
                    (int)s->slip,(int)s->raw_slip,s->yaw_rate,s->pitch*180/M_PI,s->roll*180/M_PI,
                    ball_contact,tangent_speed,m->geom_friction[3*floor],
                    balance_get_slip_blend(),balance_get_slip_yaw_blend(),ll,lr,s->leg_extend,s->leg_side_extend,
                    balance_get_obstacle_retract(),s->acc[0],s->acc[1],s->acc[2],s->gyro[0],s->gyro[1],s->gyro[2]);
            fprintf(fp,"%.6f,%.6f,%d,%d,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%d,%d\n",
                    balance_get_wheel_mismatch(0),balance_get_wheel_mismatch(1),
                    balance_get_wheel_obstacle(0),balance_get_wheel_obstacle(1),
                    balance_get_obstacle_leg_retract(0),balance_get_obstacle_leg_retract(1),contact_mask,
                    d->qpos[0],d->qpos[1],s->yaw,
                    (int)controller_supervisor_mode(),balance_get_fallen());
        }
        if (!slope) assert(!balance_get_fallen());
    }
    fclose(fp);
    printf("RESULT %s ext=%+.3f %s: travel=%.3f excursion=%.3f yaw=%.1f pitch=%.2f roll=%.2f "
           "ball_contact_ms=%d slip_ms=%d raw_ms=%d physical_slip_ms=%d detected_ms=%d "
           "control_MAE=%.4f wheel_MAE=%.4f retract_mm=%.2f recovery_pitch=%.2f recovery_roll=%.2f\n",command,ext,terrain,
           hypot(end[0]-start[0],end[1]-start[1]),max_excursion,yaw_total*180/M_PI,max_pitch,max_roll,
           ball_steps,slip_steps,raw_steps,truth_steps,detected_steps,estimate_error/5000,wheel_error/5000,1000*max_retract,
           recovery_pitch,recovery_roll);
    printf("SIDES left_contact=%.3f left_trigger=%.3f left_retract_mm=%.2f "
           "right_contact=%.3f right_trigger=%.3f right_retract_mm=%.2f\n",
           first_contact[0],first_trigger[0],1000*max_side_retract[0],
           first_contact[1],first_trigger[1],1000*max_side_retract[1]);
    printf("DRIFT overlap_ms=%.1f cross_track_max=%.4f cross_track_final=%.4f "
           "release_net=%.4f release_max=%.4f final_speed=%.4f "
           "all_pitch=%.2f all_roll=%.2f upright=%d\n",
           1000*overlap_time,max_cross_track,final_cross_track,release_net,release_max,
           final_speed,all_pitch,all_roll,upright);
    if (slope)
        printf("SLOPE degrees=%.1f settling_excursion=%.4f spin_path=%.4f "
               "spin_dx=%.4f spin_dy=%.4f fallen_time=%.3f full_lqr_ms=%d\n",
               slope_deg,settling_excursion,spin_path,end[0]-start[0],end[1]-start[1],
               fallen_time,full_lqr_steps);
    assert(!balls || ball_steps > 0);
    assert(!balls || single || overlap_time > 0);
    for(int i=0;i<mjNWARNING;++i) assert(!d->warning[i].number);
    simulation_reset(&sim);
    assert(fabs(balance_get_slip_inertial_v()) < 1e-7 && balance_get_slip_blend() == 0 && vel_kf_bias() == 0);
    assert(balance_get_obstacle_retract() == 0);
    for (int i = 0; i < 2; ++i)
        assert(balance_get_wheel_obstacle(i) == 0 && balance_get_obstacle_leg_retract(i) == 0);
    simulation_destroy(&sim);
    int recovered = recovery_pitch < 2 && recovery_roll < 2;
    int stable = ((balls || slope) ? upright && fallen_time < 0 : max_pitch < 10 && max_roll < 10) && recovered;
    int motion = (v == 0 || hypot(end[0]-start[0],end[1]-start[1]) > 0.5) &&
                 (yaw == 0 || (yaw_total*yaw > 0 && fabs(yaw_total)*180/M_PI > 60));
    int spin_centered = !spin || max_excursion < 0.20;
    printf("VERDICT %s stable=%d motion=%d spin_centered=%d\n",
           stable && motion && spin_centered ? "PASS" : "FAIL",stable,motion,spin_centered);
    return stable && motion && spin_centered ? 0 : 1;
}
