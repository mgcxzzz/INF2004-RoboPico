#include <assert.h>
#include <stdio.h>
#include "line_logic.h"
#include "../dual_wheel_bench/dual_logic.h"
#include "../line_motion_integration/logic.h"
int main(void) {
    LineCalibration c={{132,132},{350,450},7};
    assert(calibration_valid(&c));
    assert(line_level(0,132,350)==0 && line_level(132,132,350)==0);
    assert(line_level(241,132,350)==500 && line_level(4095,132,350)==1000);
    LineCalibration bad=c;bad.mask=3;assert(!calibration_valid(&bad));
    bad=c;bad.black[0]=200;assert(!calibration_valid(&bad));
    bad=c;bad.black[1]=4096;assert(!calibration_valid(&bad));
    AnalogTracker t={0};
    AnalogDemand d=analog_demand(&t,&bad,241,132,0,0);assert(d.stop);
    d=analog_demand(&t,&c,241,132,0,0);assert(!d.stop && d.target[0]==65 && d.target[1]==65);
    d=analog_demand(&t,&c,300,132,10000,1);assert(!d.stop && d.target[0]>d.target[1]);
    d=analog_demand(&t,&c,190,132,20000,2);assert(!d.stop && d.target[0]<d.target[1]);
    d=analog_demand(&t,&c,190,450,30000,3);assert(d.decision==AL_RIGHT_GUARD && d.target[0]==95 && d.target[1]==35);
    t=(AnalogTracker){0};
    d=analog_demand(&t,&c,132,132,10000,0);assert(!d.stop && d.decision==AL_SEEK_LEFT);
    d=analog_demand(&t,&c,132,132,659999,39);assert(!d.stop);
    d=analog_demand(&t,&c,132,132,660000,39);assert(d.stop && d.decision==AL_LOST && !d.target[0] && !d.target[1]);
    t=(AnalogTracker){0};
    (void)analog_demand(&t,&c,132,132,1,0);
    d=analog_demand(&t,&c,132,132,2,40);assert(d.stop);
    d=analog_demand(&t,&c,241,132,3,41);assert(!d.stop && !t.white);
    d=analog_demand(&t,&c,132,132,4,42);assert(!d.stop && t.white_since==4);
    t=(AnalogTracker){0};
    d=analog_demand(&t,&c,350,450,500,0);assert(!d.stop && d.target[0]==35 && d.target[1]==35);
    d=analog_demand(&t,&c,350,450,120499,7);assert(!d.stop);
    d=analog_demand(&t,&c,350,450,120500,8);assert(d.stop && d.decision==AL_BOTH_BLACK);
    d=analog_demand(&t,&c,241,132,120501,9);assert(!d.stop && !t.both);
    /* Every valid input yields forward-only bounded wheel targets; no hidden
       spin/reverse command, including saturated and out-of-calibration ADC. */
    for(unsigned l=0;l<=4095;l+=17) for(unsigned r=0;r<=4095;r+=19) {
        t=(AnalogTracker){0};d=analog_demand(&t,&c,(uint16_t)l,(uint16_t)r,0,0);
        assert(d.level[0]<=1000 && d.level[1]<=1000);
        assert(d.target[0]>=0 && d.target[0]<=95 && d.target[1]>=0 && d.target[1]<=95);
        if(!d.stop) assert(d.target[0]>=35 && d.target[1]>=35);
    }
    DualGuard g={0};assert(guard_start(&g,1000,true,false));
    guard_tick(&g,1100,false,false);assert(!g.active && g.reason==WHY_RELEASE);
    g=(DualGuard){0};assert(guard_start(&g,1000,true,false));
    guard_tick(&g,1100,true,true);assert(!g.active && g.reason==WHY_GP20);
    g=(DualGuard){0};assert(guard_start(&g,1000,true,false));
    guard_tick(&g,101000,true,false);assert(!g.active && g.reason==WHY_LATE);
    BenchArm arm={0};
    assert(!bench_arm_step(&arm,100,0,0,1,1));
    assert(!bench_arm_step(&arm,3000100,0,0,1,1));
    assert(!bench_arm_step(&arm,3200000,0,0,0,0));
    assert(!bench_arm_step(&arm,3500000,0,0,1,0));
    assert(!bench_arm_step(&arm,4000000,0,0,0,0));
    assert(bench_arm_step(&arm,6000000,0,0,0,0));
    puts("PASS: separate calibration, normalization, edge steering sign, right assist, bounded recovery/crossing, target bounds, release/stop/stale guards, offline deliberate sequence");
}
