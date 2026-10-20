#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <mujoco/mujoco.h>
#include <omp.h>
#include "ident_frame.h"

typedef struct {
    void *library;
    void (*reset)(void);
    void (*set_active)(int);
    void (*step)(const IdentFrame *, double, float *);
    void (*spin_command)(const float *);
    void (*spin_observation)(float *);
    void (*spin_actions)(float *);
    void (*spin_mix_audit)(float *);
    void (*leg_jacobians)(const IdentFrame *, float *);
    void (*lqr_enabled)(int);
} Controller;

typedef struct {
    int count, threads;
    Controller *controllers;
    mjData **warm_data;
} ControllerBatch;

void rl_batch_destroy(ControllerBatch *batch)
{
    if (!batch) return;
    for (int i = 0; i < batch->count; ++i)
        if (batch->controllers[i].library) dlclose(batch->controllers[i].library);
    if (batch->warm_data) {
        for (int i = 0; i < batch->threads; ++i) mj_deleteData(batch->warm_data[i]);
        free(batch->warm_data);
    }
    free(batch->controllers);
    free(batch);
}

ControllerBatch *rl_batch_create(const char **paths, int count, int threads,
                                 char *error, int error_size)
{
    if (count < 1 || threads < 1) return NULL;
    ControllerBatch *batch = calloc(1, sizeof(*batch));
    if (!batch) return NULL;
    batch->count = count;
    batch->threads = threads < count ? threads : count;
    batch->controllers = calloc(count, sizeof(Controller));
    if (!batch->controllers) { free(batch); return NULL; }
    for (int i = 0; i < count; ++i) {
        Controller *c = batch->controllers+i;
        /* Each path is a distinct copy, isolating ALL controller static state. */
        c->library = dlopen(paths[i], RTLD_NOW | RTLD_LOCAL);
        if (!c->library) {
            snprintf(error,error_size,"controller %d: %s",i,dlerror());
            rl_batch_destroy(batch);
            return NULL;
        }
        c->reset = dlsym(c->library,"ident_reset");
        c->set_active = dlsym(c->library,"ident_set_rl_active");
        c->step = dlsym(c->library,"ident_robot_step");
        c->spin_command = dlsym(c->library,"ident_spin_command");
        c->spin_observation = dlsym(c->library,"ident_spin_observation");
        c->spin_actions = dlsym(c->library,"robot_control_get_spin_residual");
        c->spin_mix_audit = dlsym(c->library,"robot_control_get_spin_mix_audit");
        c->leg_jacobians = dlsym(c->library,"ident_leg_jacobians");
        c->lqr_enabled = dlsym(c->library,"controller_supervisor_set_lqr_enabled");
        if (!c->reset || !c->set_active || !c->step ||
            !c->spin_command || !c->spin_observation || !c->spin_actions || !c->lqr_enabled || !c->spin_mix_audit || !c->leg_jacobians) {
            snprintf(error,error_size,"controller %d: missing bridge symbol",i);
            rl_batch_destroy(batch);
            return NULL;
        }
    }
    return batch;
}

size_t rl_batch_frame_size(void) { return sizeof(IdentFrame); }

void rl_batch_reset(ControllerBatch *batch, const uint8_t *mask)
{
    #pragma omp parallel for num_threads(batch->threads) schedule(static)
    for (int i = 0; i < batch->count; ++i) if (mask[i]) {
        batch->controllers[i].reset();
        batch->controllers[i].set_active(0);
    }
}

void rl_batch_set_active(ControllerBatch *batch, const uint8_t *mask, int active)
{
    for (int i = 0; i < batch->count; ++i)
        if (mask[i]) batch->controllers[i].set_active(active);
}

void rl_batch_step(ControllerBatch *batch, const IdentFrame *frames,
                   const uint8_t *mask, double dt, float *outputs)
{
    #pragma omp parallel for num_threads(batch->threads) schedule(static)
    for (int i = 0; i < batch->count; ++i) if (mask[i])
        batch->controllers[i].step(frames+i,dt,outputs+6*i);
}

void rl_batch_spin_command(ControllerBatch *batch, const float *inputs, const uint8_t *mask)
{
    for (int i = 0; i < batch->count; ++i) if (mask[i])
        batch->controllers[i].spin_command(inputs+7*i);
}

void rl_batch_spin_observation(ControllerBatch *batch, float *obs)
{
    for (int i = 0; i < batch->count; ++i)
        batch->controllers[i].spin_observation(obs+28*i);
}

void rl_batch_spin_actions(ControllerBatch *batch, float *actions)
{
    for (int i = 0; i < batch->count; ++i)
        batch->controllers[i].spin_actions(actions+5*i);
}

void rl_batch_spin_mix_audit(ControllerBatch *batch, float *output)
{
    for (int i = 0; i < batch->count; ++i)
        batch->controllers[i].spin_mix_audit(output+8*i);
}

void rl_batch_leg_jacobians(ControllerBatch *batch, const IdentFrame *frames, float *output)
{
    for (int i = 0; i < batch->count; ++i)
        batch->controllers[i].leg_jacobians(frames+i, output+8*i);
}

void rl_batch_lqr_enabled(ControllerBatch *batch, const uint8_t *enabled)
{
    for (int i = 0; i < batch->count; ++i)
        batch->controllers[i].lqr_enabled(enabled[i] != 0);
}

static void warm_frame(const mjModel *m, const mjData *d, const int *qa,
                       const int *va, IdentFrame *f)
{
    memset(f,0,sizeof(*f));
    const mjtNum *b = d->sensordata;
    const double c = 0.70710678;
    double w = b[0]*c+b[1]*c, x = -b[0]*c+b[1]*c;
    double y = b[2]*c-b[3]*c, z = b[2]*c+b[3]*c;
    f->quat[0] = w; f->quat[1] = x; f->quat[2] = y; f->quat[3] = z;
    f->pitch = asin(fmax(-1,fmin(1,2*(w*y-x*z))));
    f->roll = atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    f->yaw = atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    f->roll_rate = b[4]; f->pitch_rate = -b[6]; f->yaw_rate = b[5];
    f->acc_x = b[7]; f->acc_y = -b[9]; f->acc_z = b[8];
    f->acc_mag = sqrt(b[7]*b[7]+b[8]*b[8]);
    f->lf_pos = d->qpos[qa[0]]; f->lr_pos = d->qpos[qa[1]];
    f->rf_pos = d->qpos[qa[2]]; f->rr_pos = d->qpos[qa[3]];
    f->lf_vel = d->qvel[va[0]]; f->lr_vel = d->qvel[va[1]];
    f->rf_vel = d->qvel[va[2]]; f->rr_vel = d->qvel[va[3]];
    f->left_wheel_pos = d->qpos[qa[4]]; f->right_wheel_pos = d->qpos[qa[5]];
    f->left_wheel_vel = d->qvel[va[4]]; f->right_wheel_vel = d->qvel[va[5]];
    f->lf_tau = d->actuator_force[0]; f->lr_tau = d->actuator_force[1];
    f->rf_tau = d->actuator_force[3]; f->rr_tau = d->actuator_force[4];
    f->left_wheel_tau = d->actuator_force[2]*10;
    f->right_wheel_tau = d->actuator_force[5]*10;
    for (int k = 0; k < 3; ++k) {
        f->body_pos[k] = d->qpos[k]; f->body_vel[k] = d->qvel[k];
    }
    f->step_count = (int)llround(d->time/m->opt.timestep); f->sim_time = d->time;
}

static int warmup_impl(ControllerBatch *batch, const mjModel *m, mjtNum *states,
                    const int *steps, const uint8_t *mask,
                    mjtNum *sensors, mjtNum *forces,
                    const mjtNum *floor_quats, const mjtNum *floor_friction,
                    mjtNum *imu_delta)
{
    if (m->nu != 6 || m->nsensordata < 10) return 0;
    int imu = mj_name2id(m,mjOBJ_SITE,"imu");
    int attitude = mj_name2id(m,mjOBJ_SENSOR,"body_quat");
    if (imu_delta && (imu < 0 || attitude < 0 ||
        !((m->sensor_objtype[attitude] == mjOBJ_BODY &&
           m->sensor_objid[attitude] == m->site_bodyid[imu]) ||
          (m->sensor_objtype[attitude] == mjOBJ_SITE && m->sensor_objid[attitude] == imu)))) return 0;
    int qa[6], va[6];
    const char *names[6] = {"Left_front_joint","Left_rear_joint",
        "Right_front_joint","Right_rear_joint","Left_Wheel_joint","Right_Wheel_joint"};
    for (int k = 0; k < 6; ++k) {
        int j = mj_name2id(m,mjOBJ_JOINT,names[k]);
        if (j < 0) return 0;
        qa[k] = m->jnt_qposadr[j]; va[k] = m->jnt_dofadr[j];
    }
    if (!batch->warm_data) {
        batch->warm_data = calloc(batch->threads,sizeof(mjData *));
        if (!batch->warm_data) return 0;
        for (int t = 0; t < batch->threads; ++t) batch->warm_data[t] = mj_makeData(m);
    }
    int nstate = mj_stateSize(m,mjSTATE_INTEGRATION);
    mjModel **models = NULL;
    int floor = mj_name2id(m,mjOBJ_GEOM,"floor");
    if (floor_quats) {
        if (floor < 0) return 0;
        models = calloc(batch->threads,sizeof(mjModel *));
        if (!models) return 0;
        for (int t = 0; t < batch->threads; ++t) {
            models[t] = mj_copyModel(NULL,m);
            if (!models[t]) {
                for (int j = 0; j < t; ++j) mj_deleteModel(models[j]);
                free(models);
                return 0;
            }
        }
    }
    #pragma omp parallel for num_threads(batch->threads) schedule(dynamic)
    for (int i = 0; i < batch->count; ++i) if (mask[i]) {
        int thread = omp_get_thread_num();
        const mjModel *mi = m;
        if (models) {
            mjModel *local = models[thread];
            memcpy(local->geom_quat+4*floor,floor_quats+4*i,4*sizeof(mjtNum));
            local->geom_sameframe[floor] = mjSAMEFRAME_NONE;
            memcpy(local->geom_friction+3*floor,floor_friction+3*i,3*sizeof(mjtNum));
            mi = local;
        }
        mjData *d = batch->warm_data[thread];
        mj_resetData(mi,d);
        mj_setState(mi,d,states+(size_t)i*nstate,mjSTATE_INTEGRATION);
        mj_forward(mi,d);
        mjtNum previous_gyro[3], offset[3], inverse_site[4];
        if (imu_delta) {
            memset(imu_delta+3*i,0,3*sizeof(mjtNum));
            memcpy(previous_gyro,d->sensordata+4,3*sizeof(mjtNum));
            mju_negQuat(inverse_site,mi->site_quat+4*imu);
            mju_rotVecQuat(offset,mi->site_pos+3*imu,inverse_site);
        }
        for (int step = 0; step < steps[i]; ++step) {
            IdentFrame frame;
            float ctrl[6];
            warm_frame(mi,d,qa,va,&frame);
            batch->controllers[i].step(&frame,mi->opt.timestep,ctrl);
            for (int k = 0; k < 6; ++k) d->ctrl[k] = ctrl[k];
            mj_step(mi,d);
            if (imu_delta) {
                /* Integrate measured specific force at the base origin. No
                 * simulated position, velocity, qacc or contact input. */
                const mjtNum *b = d->sensordata;
                mjtNum alpha[3], wxr[3], centripetal[3], tangent[3], center[3];
                mjtNum world_site[4], inverse_inertial[4], body[4], world_acc[3];
                for (int k = 0; k < 3; ++k) {
                    alpha[k] = (b[4+k]-previous_gyro[k])/mi->opt.timestep;
                    previous_gyro[k] = b[4+k];
                }
                mju_cross(wxr,b+4,offset);
                mju_cross(centripetal,b+4,wxr);
                mju_cross(tangent,alpha,offset);
                for (int k = 0; k < 3; ++k) center[k] = b[7+k]-centripetal[k]-tangent[k];
                if (mi->sensor_objtype[attitude] == mjOBJ_BODY) {
                    mju_negQuat(inverse_inertial,mi->body_iquat+4*mi->site_bodyid[imu]);
                    mju_mulQuat(body,b,inverse_inertial);
                    mju_mulQuat(world_site,body,mi->site_quat+4*imu);
                } else memcpy(world_site,b,4*sizeof(mjtNum));
                mju_rotVecQuat(world_acc,center,world_site);
                for (int k = 0; k < 3; ++k)
                    imu_delta[3*i+k] += (world_acc[k]+mi->opt.gravity[k])*mi->opt.timestep;
            }
        }
        mj_getState(mi,d,states+(size_t)i*nstate,mjSTATE_INTEGRATION);
        memcpy(sensors+(size_t)i*m->nsensordata,d->sensordata,m->nsensordata*sizeof(mjtNum));
        memcpy(forces+(size_t)i*m->nu,d->actuator_force,m->nu*sizeof(mjtNum));
    }
    if (models) {
        for (int t = 0; t < batch->threads; ++t) mj_deleteModel(models[t]);
        free(models);
    }
    return 1;
}

int rl_batch_warmup(ControllerBatch *b, const mjModel *m, mjtNum *state,
                    const int *steps, const uint8_t *mask, mjtNum *sensors, mjtNum *forces)
{
    return warmup_impl(b,m,state,steps,mask,sensors,forces,NULL,NULL,NULL);
}

int rl_batch_warmup_terrain(ControllerBatch *b, const mjModel *m, mjtNum *state,
                    const int *steps, const uint8_t *mask, mjtNum *sensors, mjtNum *forces,
                    const mjtNum *quats, const mjtNum *friction)
{
    return warmup_impl(b,m,state,steps,mask,sensors,forces,quats,friction,NULL);
}

int rl_batch_warmup_terrain_imu(ControllerBatch *b, const mjModel *m, mjtNum *state,
                    const int *steps, const uint8_t *mask, mjtNum *sensors, mjtNum *forces,
                    const mjtNum *quats, const mjtNum *friction, mjtNum *imu_delta)
{
    return warmup_impl(b,m,state,steps,mask,sensors,forces,quats,friction,imu_delta);
}
