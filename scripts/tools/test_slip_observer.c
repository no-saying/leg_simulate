#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "slip_observer.h"
#include "vel_kf.h"

static void test_dt(float dt)
{
    SlipObserver s;
    slip_observer_reset(&s);
    /* Real forward acceleration must agree with both wheel encoders. */
    for (int k = 0; k < (int)(2.0f/dt); ++k) {
        float v = 0.3f*k*dt;
        slip_observer_step(&s,v,v,0.3f,0,0,0,1,dt);
        assert(!s.active);
    }
    /* Heading changes during a constant speed turn. The lateral force is
     * centripetal acceleration, not evidence of forward wheel spin. */
    slip_observer_reset(&s);
    for (int k = 0; k < (int)(5.0f/dt); ++k) {
        slip_observer_step(&s,0.7f,0.7f,0,1.4f,2*k*dt,0,1,dt);
        assert(!s.active);
        assert(fabsf(s.velocity_error)<0.1f);
    }
    slip_observer_reset(&s);
    for (int k = 0; k < (int)(0.2f/dt); ++k)
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    /* One wheel spins, while the body and gyro remain stationary. */
    float detection = 0;
    for (int k = 0; k < (int)(0.3f/dt); ++k) {
        slip_observer_step(&s,1.0f,0,0,0,0,0,1,dt);
        if (s.active && !detection) detection = (k+1)*dt;
    }
    assert(detection>0 && detection<0.15f);
    assert(s.wheel[0].active && !s.wheel[1].active);
    for (int k = 0; k < (int)(1.0f/dt); ++k)
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    assert(!s.active);
    /* Locked wheels while a previously measured body velocity persists. */
    slip_observer_reset(&s);
    for (int k = 0; k < (int)(0.2f/dt); ++k)
        slip_observer_step(&s,0.7f,0.7f,0,0,0,0,1,dt);
    for (int k = 0; k < (int)(0.3f/dt); ++k)
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    assert(s.active);
    assert(s.linear_active);
    assert(s.wheel[0].active && s.wheel[1].active);
    float coasting = s.vx;
    for (int k=0;k<(int)(0.1f/dt);++k) {
        slip_observer_step(&s,10,10,0,0,0,0,0,dt);
        assert(s.active && s.linear_active);
        assert(fabsf(s.vx-coasting)<1e-6f);
    }
    /* Opposite wheel errors indicate yaw slip without invalidating linear speed. */
    slip_observer_reset(&s);
    for (int k=0;k<(int)(0.2f/dt);++k)
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    for (int k=0;k<(int)(0.3f/dt);++k)
        slip_observer_step(&s,0.6f,-0.6f,0,0,0,0,1,dt);
    assert(s.active && !s.linear_active && fabsf(s.vx)<1e-6f);
    assert(s.wheel[0].active && s.wheel[1].active);
    /* A growing encoder error must stop pulling the inertial estimate along. */
    slip_observer_reset(&s);
    for (int k=0;k<(int)(0.2f/dt);++k)
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    for (int k=0;k<(int)(0.3f/dt);++k) {
        float wheel = 4.0f*k*dt;
        slip_observer_step(&s,wheel,wheel,0,0,0,0,1,dt);
    }
    assert(s.active && s.linear_active && fabsf(s.vx)<0.12f);
    /* Brief encoder spikes and airborne wheels must not latch a slip. */
    slip_observer_reset(&s);
    for (int k=0;k<(int)(0.2f/dt);++k)
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    for (int k=0;k<(int)(0.01f/dt);++k)
        slip_observer_step(&s,1,0,0,0,0,0,1,dt);
    for (int k=0;k<(int)(0.3f/dt);++k) {
        slip_observer_step(&s,0,0,0,0,0,0,1,dt);
        assert(!s.active);
    }
    float before_air = s.vx;
    slip_observer_step(&s,10,10,0,0,0,0,0,dt);
    assert(!s.active && fabsf(s.vx-before_air)<0.000001f);
    slip_observer_step(&s,0,0,0,0,0,0,1,dt);
    assert(!s.active && fabsf(s.vx)<0.02f);
    slip_observer_reset(&s);
    slip_observer_step(&s,10,10,0,0,0,0,0,dt);
    assert(!s.ready);
    /* Independent timers: alternating sides cannot accumulate one trigger. */
    WheelMismatch left = {0}, right = {0};
    for (int k=0;k<(int)(0.1f/dt);++k) {
        int phase = (int)lroundf(k*dt/0.005f) % 2;
        wheel_mismatch_step(&left,phase ? 0 : 0.5f,1,dt);
        wheel_mismatch_step(&right,phase ? 0.5f : 0,1,dt);
        assert(!left.active && !right.active);
    }
    for (int k=0;k<(int)(0.1f/dt);++k) {
        wheel_mismatch_step(&left,0,1,dt);
        wheel_mismatch_step(&right,0.5f,1,dt);
    }
    assert(!left.active && right.active);
}

int main(void)
{
    test_dt(0.001f);
    test_dt(0.002f);
    test_dt(0.005f);
    vel_kf_reset();
    float bias=vel_kf_bias();
    for(int k=0;k<500;++k) vel_kf_step(0,5,0,0,0.001f);
    assert(vel_kf_bias()==bias);
    vel_kf_set_velocity(0.3f);
    vel_kf_reset();
    assert(vel_kf_step(0,0,0,0,0.001f)==0);
    puts("PASS: independent wheel timers, acceleration, constant speed turns, single-wheel spin, locked wheels, spike rejection, airborne/recontact, 1/2/5 ms timing, KF reset/bias freeze");
    return 0;
}
