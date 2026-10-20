#ifndef LEG_ALLOCATION_H
#define LEG_ALLOCATION_H

#include <math.h>

typedef struct {
    float offset;
    float left, right, error;
} LegAllocation;

static inline float leg_allocation_clip(float value, float lo, float hi)
{
    return fmaxf(lo, fminf(hi, value));
}

/* Extension is positive towards the short stop. Move the mean reference to
 * retain roll authority; limit differential until that height is available. */
static inline void leg_allocation_step(LegAllocation *a, float common, float differential,
                                      float lo, float hi, int supported, int turning,
                                      float dt, float move_rate, float restore_rate)
{
    if (!supported) {
        a->offset = a->error = 0.0f;
        a->left = leg_allocation_clip(common + differential, lo, hi);
        a->right = leg_allocation_clip(common - differential, lo, hi);
        return;
    }
    float c = leg_allocation_clip(common, lo, hi);
    float d = leg_allocation_clip(differential, -0.5f*(hi-lo), 0.5f*(hi-lo));
    float magnitude = fabsf(d);
    float target = fminf(0.0f, hi-magnitude-c);
    /* Retain attained short-leg clearance through a spin's cross-slope zero
     * crossings, rather than pumping the mean height twice per revolution. */
    if (turning && a->offset < target && a->offset < 0.0f && c-a->offset >= hi)
        target = a->offset;
    float rate = fabsf(target) > fabsf(a->offset) ? move_rate : restore_rate;
    a->offset += leg_allocation_clip(target-a->offset, -rate*dt, rate*dt);
    a->offset = leg_allocation_clip(a->offset, lo-c, hi-c);
    c += a->offset;
    float available = fmaxf(0.0f, hi-c);
    d = leg_allocation_clip(d, -available, available);
    a->left = leg_allocation_clip(c+d, lo, hi);
    a->right = leg_allocation_clip(c-d, lo, hi);
    a->error = differential-0.5f*(a->left-a->right);
}

#endif
