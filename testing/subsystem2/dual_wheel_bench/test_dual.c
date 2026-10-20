#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "dual_logic.h"
static void near(float a,float b,float tolerance) { assert(fabsf(a-b)<=tolerance); }
static void test_geometry_and_outputs(void) {
    near(raw_to_mm(0,-252405),20000,0.01f);
    near(raw_to_mm(1,252830),20000,0.01f);
    near(heading_deg(0,0),0,0.001f);
    near(heading_deg(TRACK_MM*3.14159265f/8,-TRACK_MM*3.14159265f/8),45,0.001f);
    DualLevels a=dual_levels(0,150),b=dual_levels(1,150);
    assert(a.a==150 && a.b==0 && b.a==0 && b.b==150);
    a=dual_levels(0,-150); b=dual_levels(1,-150);
    assert(a.a==0 && a.b==150 && b.a==150 && b.b==0);
    a=dual_levels(0,9999); assert(a.a==250 && a.b==0);
    a=dual_levels(1,-9999); assert(a.a==250 && a.b==0);
    a=dual_levels(2,250); assert(a.a==0 && a.b==0);
    DualCommand c;
    assert(command_make('f',80,&c) && c.sign[0]==1 && c.sign[1]==1 && !c.bounded);
    assert(command_make('b',80,&c) && c.sign[0]==-1 && c.sign[1]==-1);
    assert(command_make('r',80,&c) && c.sign[0]==1 && c.sign[1]==-1);
    assert(command_make('l',80,&c) && c.sign[0]==-1 && c.sign[1]==1);
    assert(command_make('d',80,&c) && c.bounded && c.target_mm[0]==100);
    near(position_demand(&c,0,0),60,0.001f);
    near(position_demand(&c,0,90),30,0.001f);
    assert(position_demand(&c,0,98)==0 && position_demand(&c,0,120)==0);
    assert(command_make('e',80,&c) && c.sign[0]==1 && c.sign[1]==-1);
    assert(command_make('q',80,&c) && c.sign[0]==-1 && c.sign[1]==1);
    assert(!command_make('x',80,&c));
}
static void test_stops(void) {
    DualGuard g={.consumed=true};
    assert(!guard_start(&g,0,true,false));
    guard_tick(&g,0,false,false);
    assert(!guard_start(&g,1,false,false));
    assert(!guard_start(&g,1,true,true));
    assert(guard_start(&g,1,true,false));
    guard_tick(&g,50001,true,true);
    assert(!g.active && g.reason==WHY_GP20 && g.duration_us==50000);
    assert(!guard_start(&g,2000000,true,false));
    guard_tick(&g,2000000,false,false);
    assert(guard_start(&g,2000001,true,false));
    guard_tick(&g,2100001,true,false);
    assert(!g.active && g.reason==WHY_LATE);
    guard_tick(&g,4000000,false,false);
    assert(guard_start(&g,4000000,true,false));
    for(uint64_t t=4010000;t<7000000;t+=10000) {
        g.last_control_us=t; guard_tick(&g,t,true,false); assert(g.active);
    }
    guard_tick(&g,7000000,true,false); assert(!g.active && g.reason==WHY_LIMIT);
    guard_tick(&g,8000000,false,false);
    assert(guard_start(&g,8000000,true,false));
    guard_tick(&g,8000500,false,false);
    assert(!g.active && g.reason==WHY_RELEASE);
    guard_tick(&g,10000000,false,false);
    assert(guard_start(&g,10000000,true,false));
    guard_stop(&g,10001000,WHY_SERIAL);
    assert(!g.active && g.reason==WHY_SERIAL && g.duration_us==1000);
    guard_stop(&g,10002000,WHY_USB); assert(g.reason==WHY_SERIAL); /* First cause retained. */
}
static void test_offline(void) {
    OfflineArm o={0};
    assert(!offline_step(&o,0,false,false,false)); /* Boot/unplug alone never starts. */
    o.state=OFF_UNPLUG;
    assert(!offline_step(&o,0,true,true,false) && o.state==OFF_UNPLUG);
    assert(!offline_step(&o,1,false,true,false) && o.state==OFF_BASE_RELEASE);
    assert(!offline_step(&o,2,false,false,false) && o.state==OFF_PRESS);
    assert(!offline_step(&o,3,false,true,false) && o.state==OFF_RELEASE);
    assert(!offline_step(&o,4,false,false,false) && o.state==OFF_COUNTDOWN);
    assert(!offline_step(&o,2000003,false,false,false));
    assert(offline_step(&o,2000004,false,false,false) && o.state==OFF_NONE);
    assert(!offline_step(&o,3000000,false,true,false)); /* One shot only. */
    o=(OfflineArm){.state=OFF_COUNTDOWN,.start_at=1};
    assert(!offline_step(&o,2,true,false,false) && o.state==OFF_NONE); /* USB return cancels. */
    o=(OfflineArm){.state=OFF_COUNTDOWN,.start_at=1};
    assert(!offline_step(&o,2,false,false,true) && o.state==OFF_NONE); /* GP20 cancels. */
    o=(OfflineArm){.state=OFF_COUNTDOWN,.start_at=1};
    assert(!offline_step(&o,2,false,true,false) && o.state==OFF_NONE); /* GP21 cancels. */
}
static void test_pi_model(void) {
    for(unsigned w=0;w<2;++w) for(int sign=-1;sign<=1;sign+=2) {
        WheelPI p={0}; float v=0;
        for(int i=0;i<300;++i) {
            int32_t u=pi_step(&p,w,sign,80,v,0.01f);
            assert(u*sign>=0 && abs(u)<=PWM_LIMIT);
            float equilibrium=fmaxf(0,(float)(u*sign)-40.0f);
            v+=(equilibrium-v)*0.1f; /* Independent first-order motor approximation. */
        }
        near(v,80,12); /* Validates initial model behaviour, NOT physical tuning. */
        (void)pi_step(&p,w,sign,100,0,10.0f);
        assert(p.integral<=80 && p.integral>=-80);
        assert(pi_step(&p,w,sign,0,v,0.01f)==0 && p.integral==0 && p.ramp==0);
    }
}
int main(void) {
    test_geometry_and_outputs(); test_stops(); test_offline(); test_pi_model();
    puts("PASS: calibration/signs, PWM cap, command geometry, timer/hold/stop guards, offline one-shot/cancellation, PI model");
    return 0;
}
