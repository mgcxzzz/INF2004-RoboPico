#include <assert.h>
#include <stdio.h>
#include "line_logic.h"
#include "../dual_wheel_bench/dual_logic.h"
#include "../line_motion_integration/logic.h"
int main(void) {
    LineRearm ready={0};
    line_rearm_step(&ready,0,true,true,false);assert(!ready.ready);
    line_rearm_step(&ready,999999,true,true,false);assert(!ready.ready);
    line_rearm_step(&ready,1000000,true,true,false);assert(ready.ready);
    /* GP21 can be held AFTER readiness for f. Readiness alone never starts. */
    line_rearm_step(&ready,1100000,false,true,false);assert(ready.ready);
    assert(line_start_block(false,false,true,false,ready.ready,1000,0,true,LINE_USB_HOLD)==START_OK);
    assert(line_start_block(false,false,true,false,true,150,450,false,LINE_USB_FREE)==START_OK);
    assert(line_start_block(false,false,true,false,true,1000,0,false,LINE_BANK)==START_OK);
    assert(line_start_block(false,false,true,false,true,149,0,true,LINE_USB_HOLD)==START_LEFT_WHITE);
    assert(line_start_block(false,false,true,false,true,1000,451,true,LINE_USB_HOLD)==START_RIGHT_DARK);
    assert(line_start_block(false,false,true,false,true,1000,1000,true,LINE_USB_HOLD)==START_RIGHT_DARK);
    assert(line_start_block(false,false,false,false,true,500,0,true,LINE_USB_HOLD)==START_CAL);
    assert(line_start_block(true,false,true,false,true,500,0,true,LINE_USB_HOLD)==START_ACTIVE);
    assert(line_start_block(false,true,true,false,true,500,0,true,LINE_USB_HOLD)==START_SETTLING);
    assert(line_start_block(false,false,true,true,true,500,0,true,LINE_USB_HOLD)==START_GP20);
    assert(line_start_block(false,false,true,false,false,500,0,true,LINE_USB_HOLD)==START_REARM);
    assert(line_start_block(false,false,true,false,true,500,0,false,LINE_USB_HOLD)==START_HOLD_GP21);
    assert(line_start_block(false,false,true,false,true,500,0,true,LINE_USB_FREE)==START_RELEASE_GP21);
    assert(!line_countdown_cancel(true,false,false));
    assert(line_countdown_cancel(false,false,false));
    assert(line_countdown_cancel(true,true,false));
    assert(line_countdown_cancel(true,false,true));
    /* Repeat an actual guard start/stop cycle without reboot. Each new request
       needs settling to finish and a fresh stationary/released interval. */
    DualGuard repeat={0};
    assert(guard_start(&repeat,1100000,true,false));
    ready=(LineRearm){0};
    line_rearm_step(&ready,1200000,false,true,true);assert(!ready.ready);
    guard_tick(&repeat,1210000,false,false);assert(!repeat.active && !repeat.consumed);
    line_rearm_step(&ready,2210000,true,true,true);assert(!ready.ready); /* still settling */
    line_rearm_step(&ready,2220000,false,true,false);assert(!ready.ready); /* GP21 held */
    line_rearm_step(&ready,2230000,true,false,false);assert(!ready.ready); /* wheel moving */
    line_rearm_step(&ready,2300000,true,true,false);assert(!ready.ready);
    line_rearm_step(&ready,3299999,true,true,false);assert(!ready.ready);
    line_rearm_step(&ready,3300000,true,true,false);assert(ready.ready);
    assert(guard_start(&repeat,3300000,true,false) && repeat.generation==2);
    guard_tick(&repeat,3310000,true,true);assert(!repeat.active && repeat.reason==WHY_GP20);
    line_rearm_step(&ready,3310000,true,true,true);assert(!ready.ready);
    LineBreakaway kick={0};
    assert(line_breakaway(&kick,0,65,19,0,100)==100 && !kick.active);
    assert(line_breakaway(&kick,200000,65,20,0,100)==250 && kick.active && kick.end_us==320000);
    assert(line_breakaway(&kick,250000,65,25,10,100)==250 && kick.active);
    /* Speed acquisition ends the pulse early, then a stall cannot restart
       it during cooldown. Steady speed uses PI PWM rather than a fixed duty. */
    assert(line_breakaway(&kick,260000,65,26,20,110)==110 && !kick.active);
    assert(line_breakaway(&kick,359999,65,35,0,110)==110 && !kick.active);
    assert(line_breakaway(&kick,360000,65,36,0,110)==250 && kick.active);
    assert(line_breakaway(&kick,479999,65,45,0,110)==250 && kick.active);
    assert(line_breakaway(&kick,480000,65,46,0,110)==110 && !kick.active);
    assert(line_breakaway(&kick,580000,65,55,0,110)==250 && kick.active);
    assert(line_breakaway(&kick,581000,0,55,0,250)==0 && !kick.active);
    assert(line_breakaway(&kick,600000,65,60,65,175)==175 && !kick.active);
    assert(line_breakaway(&kick,600001,65,60,65,999)==250);
    assert(line_breakaway(&kick,600002,65,60,65,-5)==0);
    assert(line_breakaway(&kick,600003,65,60,-9,110)==110 && !kick.active);
    /* Irregular control intervals cannot extend the recorded pulse deadline
       or bypass its cooldown; the hardware watchdog handles missed updates. */
    kick=(LineBreakaway){0};
    for(uint64_t us=0;us<1000000;us+=13000) {
        bool prior=kick.active;uint64_t prior_end=kick.end_us;
        int32_t u=line_breakaway(&kick,us,65,65,0,110);
        assert(u>=0 && u<=250);
        if(prior && us<prior_end) assert(kick.end_us==prior_end);
        if(prior && us>=prior_end) assert(!kick.active && u==110);
    }
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
    puts("PASS: loose start/full-black/both-dark refusal; repeat rearm/guard cycles; hands-free countdown cancellation; 25% pulse/cooldown; calibration/steering/loss/stop bounds; offline sequence");
}
