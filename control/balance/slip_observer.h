#ifndef SLIP_OBSERVER_H
#define SLIP_OBSERVER_H

#include <math.h>
#include <string.h>
#include "control_config.h"

typedef struct {
    float error, acceleration_error, enter_t, exit_t;
    int ready, raw, active;
} WheelMismatch;

/* Inputs have matched filtering and refer to the same base velocity. */
static inline void wheel_mismatch_step(WheelMismatch *w, float error, int valid, float dt)
{
    if (!valid) {
        w->ready = w->raw = 0;
        w->enter_t = w->exit_t = 0.0f;
        w->acceleration_error = 0.0f;
        return;
    }
    float derivative = w->ready ? (error - w->error) / dt : 0.0f;
    w->acceleration_error += dt / (0.020f + dt) * (derivative - w->acceleration_error);
    w->error = error;
    w->ready = 1;
    w->raw = fabsf(error) > WHEEL_OBSTACLE_V_ENTER ||
        (fabsf(error) > WHEEL_OBSTACLE_V_MIN &&
         fabsf(w->acceleration_error) > WHEEL_OBSTACLE_A_ENTER);
    if (w->raw) {
        w->exit_t = 0.0f;
        w->enter_t += dt;
        float delay = fabsf(error) > SIMPLE_SLIP_FAST_V ? SIMPLE_SLIP_FAST_T : WHEEL_OBSTACLE_ENTER_T;
        if (w->enter_t >= delay) w->active = 1;
    } else {
        w->enter_t = 0.0f;
        if (fabsf(error) < WHEEL_OBSTACLE_V_EXIT) w->exit_t += dt;
        else w->exit_t = 0.0f;
        if (w->exit_t >= WHEEL_OBSTACLE_EXIT_T) w->active = 0;
    }
}

typedef struct {
    float vx, vy, ax, ay;
    float left, right;
    float enter_t, exit_t, age, warmup_t;
    float linear_enter_t, linear_exit_t;
    float error_slow, grip_t;
    float velocity_error, yaw_error;
    int ready, active, raw, linear_active;
    WheelMismatch wheel[2];
} SlipObserver;

static inline void slip_observer_reset(SlipObserver *s)
{
    memset(s, 0, sizeof(*s));
}

/* Wheel inputs already refer to base velocity: remove each leg's motion,
 * pitch rotation and the expected yaw differential before calling this. */
static inline float slip_observer_step(SlipObserver *s, float left, float right,
                                      float ax, float ay, float yaw, float lateral_ref,
                                      int valid, float dt)
{
    float c = cosf(yaw), n = sinf(yaw);
    if (!valid && !s->ready) {
        return 0.5f * (left + right);
    }
    if (!s->ready) {
        s->left = left;
        s->right = right;
        float v = 0.5f * (left + right);
        s->vx = c * v - n * lateral_ref;
        s->vy = n * v + c * lateral_ref;
        s->ready = 1;
    }
    float alpha = dt / (0.040f + dt);
    s->ax += alpha * (c * ax - n * ay - s->ax);
    s->ay += alpha * (n * ax + c * ay - s->ay);
    s->vx += s->ax * dt;
    s->vy += s->ay * dt;
    float v = c * s->vx + n * s->vy;
    if (!valid) {
        /* Airborne/impact encoders cannot initialize or reset inertial speed.
         * Carry an already confirmed slip through the loss of support. */
        s->raw = 0;
        s->enter_t = 0.0f;
        s->linear_enter_t = 0.0f;
        for (int i = 0; i < 2; ++i) wheel_mismatch_step(&s->wheel[i], 0, 0, dt);
        if (s->linear_active) s->age += dt;
        return v;
    }
    s->left += alpha * (left - s->left);
    s->right += alpha * (right - s->right);
    float common = 0.5f * (s->left + s->right);
    s->velocity_error = common - v;
    s->yaw_error = 0.5f * (s->left - s->right);
    if (s->warmup_t < 0.15f) {
        s->warmup_t += dt;
        float gain = dt / (0.020f + dt);
        s->vx += gain * (c * common - n * lateral_ref - s->vx);
        s->vy += gain * (n * common + c * lateral_ref - s->vy);
        return c * s->vx + n * s->vy;
    }
    wheel_mismatch_step(&s->wheel[0], s->velocity_error + s->yaw_error, 1, dt);
    wheel_mismatch_step(&s->wheel[1], s->velocity_error - s->yaw_error, 1, dt);
    float error_prev = s->error_slow;
    s->error_slow += dt / (0.080f + dt) * (s->velocity_error - s->error_slow);
    float acceleration_error = (s->error_slow - error_prev) / dt;
    if (s->linear_active && s->age > SIMPLE_SLIP_IMU_HORIZON &&
        fabsf(acceleration_error) < 0.8f && fabsf(s->yaw_error) < 0.12f)
        s->grip_t += dt;
    else
        s->grip_t = 0.0f;
    /* After the short inertial horizon, consistent acceleration and yaw
     * permit reacquisition. Constant common slip remains unobservable
     * without an independent absolute velocity sensor. */
    if (s->grip_t > 0.20f) {
        s->vx = c * common - n * lateral_ref;
        s->vy = n * common + c * lateral_ref;
        s->velocity_error = s->error_slow = 0.0f;
        s->linear_active = s->active = 0;
        s->enter_t = s->linear_enter_t = s->exit_t = s->linear_exit_t = 0.0f;
        s->age = s->grip_t = 0.0f;
        for (int i = 0; i < 2; ++i) {
            memset(&s->wheel[i], 0, sizeof(s->wheel[i]));
            s->wheel[i].error = (i ? -1.0f : 1.0f) * s->yaw_error;
            s->wheel[i].ready = 1;
        }
    }
    float wheel_error = fmaxf(fabsf(s->velocity_error + s->yaw_error),
                              fabsf(s->velocity_error - s->yaw_error));
    int linear_raw = fabsf(s->velocity_error) > SIMPLE_SLIP_V_ENTER ||
        (fabsf(acceleration_error) > SIMPLE_SLIP_A_ENTER && fabsf(s->velocity_error) > 0.08f);
    s->raw = linear_raw ||
             fabsf(s->yaw_error) > SIMPLE_SLIP_YAW_V_ENTER ||
             wheel_error > SIMPLE_SLIP_WHEEL_V_ENTER;
    s->raw |= s->wheel[0].raw || s->wheel[1].raw;
    if (s->raw) {
        s->enter_t += dt;
        s->exit_t = 0.0f;
        float enter_time = wheel_error > SIMPLE_SLIP_FAST_V ? SIMPLE_SLIP_FAST_T : SIMPLE_SLIP_ENTER_T;
        if (s->enter_t >= enter_time) s->active = 1;
    } else {
        s->enter_t = 0.0f;
        s->exit_t += dt;
        if (s->exit_t >= SIMPLE_SLIP_EXIT_T) s->active = 0;
    }
    if (linear_raw) {
        s->linear_enter_t += dt;
        s->linear_exit_t = 0.0f;
        float enter_time = fabsf(s->velocity_error) > SIMPLE_SLIP_FAST_V ? SIMPLE_SLIP_FAST_T : SIMPLE_SLIP_ENTER_T;
        if (s->linear_enter_t >= enter_time) s->linear_active = 1;
    } else {
        s->linear_enter_t = 0.0f;
        s->linear_exit_t += dt;
        if (s->linear_exit_t >= SIMPLE_SLIP_EXIT_T) s->linear_active = 0;
    }
    s->active |= s->wheel[0].active || s->wheel[1].active;
    if ((s->wheel[0].active || s->wheel[1].active) &&
        fabsf(s->velocity_error) > SIMPLE_SLIP_V_ENTER)
        s->linear_active = 1;
    if (s->linear_active) s->age += dt;
    else s->age = 0.0f;

    /* Re-anchor only below the detection thresholds. The finite fallback
     * bounds inertial drift when wheel/IMU agreement cannot be established. */
    float gain = (!s->active && !s->raw && fabsf(s->velocity_error) <= SIMPLE_SLIP_V_ENTER)
                 ? dt / (0.20f + dt) : 0.0f;
    if (s->age > SIMPLE_SLIP_IMU_HORIZON) gain = dt / (2.0f + dt);
    float tx = c * common - n * lateral_ref;
    float ty = n * common + c * lateral_ref;
    if (s->wheel[0].active != s->wheel[1].active) {
        float healthy = s->wheel[0].active ? s->right : s->left;
        if (!s->wheel[s->wheel[0].active ? 1 : 0].raw) {
            gain = dt / (0.040f + dt);
            tx = c * healthy - n * lateral_ref;
            ty = n * healthy + c * lateral_ref;
        }
    }
    s->vx += gain * (tx - s->vx);
    s->vy += gain * (ty - s->vy);
    float speed = hypotf(s->vx, s->vy);
    if (speed > 5.0f) { s->vx *= 5.0f / speed; s->vy *= 5.0f / speed; }
    return c * s->vx + n * s->vy;
}

#endif
