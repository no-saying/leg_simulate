#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "leg_allocation.h"

static void run(float dt)
{
    const float lo = -0.093f, hi = 0.070f;
    LegAllocation a = {0};
    for (int i = 0; i < (int)(1.0f/dt); ++i) {
        leg_allocation_step(&a, hi, 0, lo, hi, 1, 1, dt, 0.12f, 0.02f);
        assert(fabsf(a.left-hi) < 1e-7f && fabsf(a.right-hi) < 1e-7f);
    }
    for (int i = 0; i < (int)(1.0f/dt); ++i) {
        float previous = a.offset;
        leg_allocation_step(&a, hi, 0.012f, lo, hi, 1, 1, dt, 0.12f, 0.02f);
        assert(fabsf(a.offset-previous) <= 0.12f*dt+1e-7f);
        assert(a.left <= hi+1e-7f && a.right >= lo-1e-7f);
    }
    /* 5-degree-class cross slope: preserve the full 24 mm leg difference. */
    assert(fabsf(a.left-hi) < 1e-7f);
    assert(fabsf(a.right-(hi-0.024f)) < 1e-7f);
    for (int i = 0; i < (int)(1.0f/dt); ++i)
        leg_allocation_step(&a, hi, 0, lo, hi, 1, 1, dt, 0.12f, 0.02f);
    assert(fabsf(a.offset+0.012f) < 1e-7f);
    leg_allocation_step(&a, hi, -0.012f, lo, hi, 1, 1, dt, 0.12f, 0.02f);
    assert(fabsf(a.right-hi) < 1e-7f);
    assert(fabsf(a.left-(hi-0.024f)) < 1e-7f);
    for (int i = 0; i < (int)(2.0f/dt); ++i) {
        float previous = a.offset;
        leg_allocation_step(&a, hi, 0, lo, hi, 1, 0, dt, 0.12f, 0.02f);
        assert(fabsf(a.offset-previous) <= 0.02f*dt+1e-7f);
    }
    assert(fabsf(a.offset) < 1e-7f);
    for (int i = 0; i < (int)(1.0f/dt); ++i)
        leg_allocation_step(&a, lo, -0.02f, lo, hi, 1, 1, dt, 0.12f, 0.02f);
    assert(fabsf(a.left-lo) < 1e-7f);
    assert(fabsf(a.right-(lo+0.02f)) < 1e-7f);
    assert(a.offset == 0);
    /* Commanded jump references bypass the stance height correction. */
    leg_allocation_step(&a, hi, 0, lo, hi, 0, 0, dt, 0.12f, 0.02f);
    assert(a.offset == 0 && a.error == 0 && a.left == hi && a.right == hi);
    for (int i = 0; i < (int)(10.0f/dt); ++i) {
        float t = i*dt;
        float c = 0.15f*sinf(t), d = 0.2f*cosf(2*t);
        leg_allocation_step(&a, c, d, lo, hi, 1, 1, dt, 0.12f, 0.02f);
        assert(isfinite(a.left) && isfinite(a.right));
        assert(a.left >= lo-1e-7f && a.left <= hi+1e-7f);
        assert(a.right >= lo-1e-7f && a.right <= hi+1e-7f);
        assert(fabsf(a.error-(d-0.5f*(a.left-a.right))) < 1e-7f);
    }
}

int main(void)
{
    run(0.001f);
    run(0.002f);
    run(0.005f);
    puts("PASS: flat height, short/long stops, full roll travel, spin clearance hold, recovery, bypass, range sweep");
    return 0;
}
