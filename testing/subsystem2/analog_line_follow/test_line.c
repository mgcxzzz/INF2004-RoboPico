#include <assert.h>
#include <stdio.h>
#include "line_logic.h"
#include "wheel_control.h"
#include "symmetric_logic.h"
#include "../dual_wheel_bench/dual_logic.h"
#include "../line_motion_integration/logic.h"
int main(void) {
    /* New placement: both-black is normal, differential steering sign must
       reverse relative to the historical single-edge controller. */
    LineCalibration sc={{0,0},{1000,1000},7};SymmetricTracker st={0};
    AnalogDemand sd=symmetric_demand(&st,&sc,1000,1000,0,0);
    assert(!sd.stop && sd.decision==SYM_STRAIGHT && sd.target[0]==45 && sd.target[1]==45);
    sd=symmetric_demand(&st,&sc,1000,1000,3000000,100);
    assert(!sd.stop && sd.target[0]==45 && sd.target[1]==45); /* no crossing stop */
    sd=symmetric_demand(&st,&sc,1000,0,3100000,101);
    assert(sd.decision==SYM_LEFT && sd.target[0]==30 && sd.target[1]==60);
    sd=symmetric_demand(&st,&sc,0,1000,3200000,102);
    assert(sd.decision==SYM_RIGHT && sd.target[0]==60 && sd.target[1]==30);
    sd=symmetric_demand(&st,&sc,980,950,3200001,102);assert(sd.target[0]==45 && sd.target[1]==45);
    /* Loss uses the last reliable side, latched for this loss episode. */
    sd=symmetric_demand(&st,&sc,0,0,3300000,103);
    assert(!sd.stop && sd.decision==SYM_RECOVER_RIGHT && sd.target[0]==60 && sd.target[1]==30);
    sd=symmetric_demand(&st,&sc,0,0,3949999,127);assert(!sd.stop);
    sd=symmetric_demand(&st,&sc,0,0,3950000,127);
    assert(sd.stop && sd.decision==AL_LOST && !sd.target[0] && !sd.target[1]);
    st=(SymmetricTracker){0};
    sd=symmetric_demand(&st,&sc,0,0,0,0);assert(sd.stop); /* no invented left turn */
    st=(SymmetricTracker){0};
    (void)symmetric_demand(&st,&sc,1000,0,100,0);
    sd=symmetric_demand(&st,&sc,0,0,500101,0);assert(sd.stop); /* stale side */
    st=(SymmetricTracker){0};
    (void)symmetric_demand(&st,&sc,1000,0,100,0);
    sd=symmetric_demand(&st,&sc,0,0,500100,0);
    assert(!sd.stop && sd.decision==SYM_RECOVER_LEFT && sd.target[0]==30 && sd.target[1]==60);
    sd=symmetric_demand(&st,&sc,0,0,500101,25);assert(sd.stop); /* path bound */
    sd=symmetric_demand(&st,&sc,0,1000,500102,26);
    assert(!sd.stop && !st.white && st.direction==-1);
    sd=symmetric_demand(&st,&sc,0,0,500103,27);assert(!sd.stop && sd.decision==SYM_RECOVER_RIGHT);
    sc.mask=0;sd=symmetric_demand(&st,&sc,1000,1000,500104,28);assert(sd.stop);sc.mask=7;
    /* Weak/noisy asymmetry does not become a remembered recovery direction. */
    st=(SymmetricTracker){0};
    (void)symmetric_demand(&st,&sc,350,0,0,0);
    sd=symmetric_demand(&st,&sc,0,0,1,0);assert(sd.stop);
    LineCalibration measured={{184,184},{3464,3692},7};
    st=(SymmetricTracker){0};
    sd=symmetric_demand(&st,&measured,3425,3631,0,0);assert(sd.target[0]==45 && sd.target[1]==45);
    sd=symmetric_demand(&st,&measured,204,3717,1,0);assert(sd.target[0]>sd.target[1]);
    sd=symmetric_demand(&st,&measured,3676,271,2,0);assert(sd.target[0]<sd.target[1]);
    /* Equal calibrated levels give mirror-symmetric bounded forward requests. */
    for(unsigned l=0;l<=1000;l+=7) for(unsigned r=0;r<=1000;r+=11) {
        SymmetricTracker a={0},b={0};
        AnalogDemand da=symmetric_demand(&a,&sc,(uint16_t)l,(uint16_t)r,0,0);
        AnalogDemand db=symmetric_demand(&b,&sc,(uint16_t)r,(uint16_t)l,0,0);
        assert(da.stop==db.stop && da.target[0]==db.target[1] && da.target[1]==db.target[0]);
        assert(da.target[0]>=0 && da.target[0]<=60 && da.target[1]>=0 && da.target[1]<=60);
        if(!da.stop) assert(da.target[0]>=30 && da.target[1]>=30);
    }
    assert(profile_start_block(5,false,false,true,false,true,1000,1000,false,LINE_BANK)==START_OK);
    assert(profile_start_block(5,false,false,true,false,true,1000,700,false,LINE_USB_FREE)==START_OK);
    assert(profile_start_block(5,false,false,true,false,true,700,1000,false,LINE_USB_FREE)==START_OK);
    assert(profile_start_block(5,false,false,true,false,true,649,649,false,LINE_BANK)==START_LINE_ABSENT);
    assert(profile_start_block(5,false,false,true,false,true,0,650,false,LINE_BANK)==START_OK);
    assert(profile_start_block(5,false,false,false,false,true,1000,1000,false,LINE_BANK)==START_CAL);
    assert(profile_start_block(5,false,false,true,true,true,1000,1000,false,LINE_BANK)==START_GP20);
    assert(profile_start_block(5,false,false,true,false,false,1000,1000,false,LINE_BANK)==START_REARM);
    assert(profile_start_block(5,false,true,true,false,true,1000,1000,false,LINE_BANK)==START_SETTLING);
    assert(profile_start_block(5,false,false,true,false,true,1000,1000,true,LINE_USB_FREE)==START_RELEASE_GP21);
    puts("PASS: symmetric two-black straight, left/right mirror steering, measured calibration samples, recent-direction loss bounds/refusal, symmetric start gates");
    /* Response profile must ignore optics but NEVER bypass stop/rearm gates. */
    assert(run_start_block(true,false,false,true,false,true,0,0,false,LINE_USB_FREE)==START_OK);
    assert(run_start_block(true,false,false,true,false,true,1000,1000,false,LINE_BANK)==START_OK);
    assert(run_start_block(false,false,false,true,false,true,0,0,false,LINE_USB_FREE)==START_LEFT_WHITE);
    assert(run_start_block(true,false,false,true,true,true,0,0,false,LINE_USB_FREE)==START_GP20);
    assert(run_start_block(true,false,false,true,false,false,0,0,false,LINE_USB_FREE)==START_REARM);
    assert(run_start_block(true,false,true,true,false,true,0,0,false,LINE_USB_FREE)==START_SETTLING);
    assert(run_start_block(true,false,false,false,false,true,0,0,false,LINE_USB_FREE)==START_CAL);
    assert(run_start_block(true,false,false,true,false,true,0,0,true,LINE_USB_FREE)==START_RELEASE_GP21);
    assert(run_start_block(true,true,false,true,false,true,0,0,false,LINE_USB_FREE)==START_ACTIVE);
    AnalogDemand response=response_demand(0);
    assert(!response.stop && response.decision==5 && response.target[0]==65 && response.target[1]==65);
    response=response_demand(1999999);assert(response.target[0]==65 && response.target[1]==65);
    response=response_demand(2000000);
    assert(!response.stop && response.decision==6 && response.target[0]==35 && response.target[1]==95);
    response=response_demand(3999999);assert(response.target[0]==35 && response.target[1]==95);
    response=response_demand(4000000);
    assert(!response.stop && response.decision==7 && response.target[0]==95 && response.target[1]==35);
    response=response_demand(5999999);assert(!response.stop);
    response=response_demand(6000000);assert(response.stop && !response.target[0] && !response.target[1]);
    /* Through all three stages the V5 PI/ramp/kick remains bounded. */
    WheelPI response_pi[2]={{0}};LineBreakaway response_kick[2]={{0}};
    for(uint64_t us=0;us<RESPONSE_US;us+=10000) {
        response=response_demand(us);
        for(unsigned w=0;w<2;++w) {
            float feedback=w==0?47.0f:36.0f;
            int32_t pwm=line_speed_step(&response_pi[w],w,response.target[w],feedback,0.01f);
            pwm=line_breakaway(&response_kick[w],us,response.target[w],response_pi[w].ramp,feedback,pwm);
            assert(pwm>=0 && pwm<=PWM_LIMIT);
        }
    }
    /* Fixed measured errors from the V4 equal-speed stage: the new controller
       must increase effort faster, without a feedforward or PWM-cap change. */
    for(unsigned w=0;w<2;++w) {
        WheelPI old={.ramp=65},tuned={.ramp=65};
        int32_t old_u=0,new_u=0;
        for(unsigned i=0;i<100;++i) {
            float measured=w==0?52.0f:40.0f;
            old_u=pi_step(&old,w,1,65,measured,0.01f);
            new_u=line_speed_step(&tuned,w,65,measured,0.01f);
        }
        assert(new_u>old_u && new_u<=PWM_LIMIT);
        float before=tuned.integral;
        for(unsigned i=0;i<50;++i) (void)line_speed_step(&tuned,w,35,65,0.01f);
        assert(tuned.integral<before); /* overspeed removes accumulated effort */
        assert(line_speed_step(&tuned,w,0,90,0.01f)==0 && !tuned.ramp && !tuned.integral);
        tuned=(WheelPI){.ramp=95,.integral=130};
        float held=tuned.integral;
        assert(line_speed_step(&tuned,w,95,0,0.01f)==250 && tuned.integral==held);
        (void)line_speed_step(&tuned,w,95,120,0.01f);
        assert(tuned.integral<held); /* saturation cannot prevent unwinding */
        tuned=(WheelPI){.ramp=35,.integral=-130};held=tuned.integral;
        assert(line_speed_step(&tuned,w,35,120,0.01f)==0 && tuned.integral==held);
        (void)line_speed_step(&tuned,w,35,0,0.01f);assert(tuned.integral>held);
        tuned=(WheelPI){0};
        for(unsigned i=0;i<10000;++i) {
            float dt=i%2?0.013f:0.007f;
            int32_t u=line_speed_step(&tuned,w,i%300<150?95:35,i%200<100?0:150,dt);
            assert(u>=0 && u<=250 && fabsf(tuned.integral)<=140);
        }
    }
    puts("PASS: V5 stronger underspeed correction, overspeed unwinding, zero reset, both saturation bounds and variable-step PWM bounds");
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
    assert(profile_start_block(3,false,false,true,false,true,1000,0,false,LINE_BANK)==START_OK);
    assert(profile_start_block(3,false,false,true,false,true,849,0,false,LINE_BANK)==START_LEFT_WHITE);
    assert(profile_start_block(3,false,false,true,false,true,850,450,false,LINE_USB_FREE)==START_OK);
    assert(profile_start_block(3,false,false,true,false,true,1000,451,false,LINE_BANK)==START_RIGHT_DARK);
    assert(profile_start_block(3,false,false,true,true,true,1000,0,false,LINE_BANK)==START_GP20);
    assert(profile_start_block(3,false,false,true,false,false,1000,0,false,LINE_BANK)==START_REARM);
    assert(profile_start_block(3,false,false,false,false,true,1000,0,false,LINE_BANK)==START_CAL);
    LineCalibration plateau={{0,0},{1000,1000},7};
    AnalogTracker bt={0};
    AnalogDemand bd=black_straight_demand(&bt,&plateau,1000,0,0,0);
    assert(!bd.stop && bd.decision==8 && bd.target[0]==45 && bd.target[1]==45);
    bd=black_straight_demand(&bt,&plateau,750,0,1,0);
    assert(bd.decision==8 && bd.target[0]==45 && bt.black_hold);
    bd=black_straight_demand(&bt,&plateau,650,0,2,0);
    assert(!bt.black_hold && bd.decision==9 && bd.target[0]==30 && bd.target[1]==60);
    bd=black_straight_demand(&bt,&plateau,750,0,3,0);assert(bd.decision==9);
    bd=black_straight_demand(&bt,&plateau,850,0,4,0);assert(bd.decision==8 && bt.black_hold);
    bd=black_straight_demand(&bt,&plateau,900,650,5,0);
    assert(!bd.stop && !bt.black_hold && bd.decision==AL_RIGHT_GUARD && bd.target[0]==60 && bd.target[1]==30);
    bd=black_straight_demand(&bt,&plateau,0,0,10,0);assert(!bd.stop && bd.decision==AL_SEEK_LEFT);
    bd=black_straight_demand(&bt,&plateau,0,0,650010,0);
    assert(bd.stop && bd.decision==AL_LOST && !bd.target[0] && !bd.target[1]);
    bt=(AnalogTracker){0};
    (void)black_straight_demand(&bt,&plateau,0,0,1,0);
    bd=black_straight_demand(&bt,&plateau,0,0,2,40);assert(bd.stop);
    bt=(AnalogTracker){0};
    bd=black_straight_demand(&bt,&plateau,1000,1000,0,0);
    assert(!bd.stop && bd.decision==AL_BOTH_BLACK && bd.target[0]==35 && bd.target[1]==35);
    bd=black_straight_demand(&bt,&plateau,1000,1000,120000,0);
    assert(bd.stop && !bd.target[0] && !bd.target[1]);
    LineCalibration no_cal={0};
    bd=black_straight_demand(&bt,&no_cal,1000,0,1,0);assert(bd.stop);
    /* Sample the full ADC range in both hysteresis states. */
    for(unsigned l=0;l<=4095;l+=31) for(unsigned r=0;r<=4095;r+=37) for(unsigned hold=0;hold<2;++hold) {
        bt=(AnalogTracker){.black_hold=hold!=0};
        bd=black_straight_demand(&bt,&plateau,(uint16_t)l,(uint16_t)r,0,0);
        assert(bd.target[0]>=0 && bd.target[0]<=60 && bd.target[1]>=0 && bd.target[1]<=60);
        if(!bd.stop) assert(bd.target[0]>=30 && bd.target[1]>=30);
    }
    puts("PASS: profile4 black=straight, threshold hysteresis, right assist, white timeout/distance, crossing stop and start gates");
    /* Profile5 deliberately replaces only BOTH-WHITE recovery. It must not
       inherit the650ms stop, nor bypass its own time/path or crossing guards. */
    AnalogTracker ct={0};
    AnalogDemand cd=curve_demand(&ct,&plateau,500,0,0,0);
    assert(!cd.stop && cd.target[0]==45 && cd.target[1]==45);
    cd=curve_demand(&ct,&plateau,0,0,100,5);
    assert(!cd.stop && cd.decision==10 && cd.target[0]==0 && cd.target[1]==45);
    cd=curve_demand(&ct,&plateau,0,0,650100,20);assert(!cd.stop && cd.decision==10);
    cd=curve_demand(&ct,&plateau,0,0,1200099,34);assert(!cd.stop);
    cd=curve_demand(&ct,&plateau,0,0,1200100,34);
    assert(cd.stop && cd.decision==AL_LOST && !cd.target[0] && !cd.target[1]);
    ct=(AnalogTracker){0};
    (void)curve_demand(&ct,&plateau,0,0,1,5);
    cd=curve_demand(&ct,&plateau,0,0,2,35);assert(cd.stop);
    cd=curve_demand(&ct,&plateau,500,0,3,35);
    assert(!cd.stop && !ct.white && cd.target[0]==45 && cd.target[1]==45);
    cd=curve_demand(&ct,&plateau,0,0,4,36);assert(!cd.stop && ct.white_since==4 && ct.white_mm==36);
    ct=(AnalogTracker){0};
    cd=curve_demand(&ct,&plateau,1000,1000,1,0);assert(!cd.stop && cd.decision==AL_BOTH_BLACK);
    cd=curve_demand(&ct,&plateau,1000,1000,120001,0);assert(cd.stop && !cd.target[0] && !cd.target[1]);
    cd=curve_demand(&ct,&no_cal,0,0,1,0);assert(cd.stop);
    WheelPI stopped_wheel={.integral=100,.ramp=45};LineBreakaway stopped_kick={.active=true,.end_us=9999999};
    assert(line_speed_step(&stopped_wheel,0,0,45,0.01f)==0 && !stopped_wheel.integral && !stopped_wheel.ramp);
    assert(line_breakaway(&stopped_kick,10,0,0,45,0)==0 && !stopped_kick.active);
    assert(profile_start_block(4,false,false,true,false,true,500,0,false,LINE_BANK)==START_OK);
    assert(profile_start_block(4,false,false,true,false,true,0,0,false,LINE_BANK)==START_LEFT_WHITE);
    puts("PASS: profile5 left-zero/right45 recovery, extended deadline, reduced path cap, reacquisition reset, invalid-cal/crossing stop, zero-wheel PI/kick reset");
    LineCalibration c={{132,132},{350,450},7};
    assert(calibration_valid(&c));
    assert(line_level(0,132,350)==0 && line_level(132,132,350)==0);
    assert(line_level(241,132,350)==500 && line_level(4095,132,350)==1000);
    LineCalibration bad=c;bad.mask=3;assert(!calibration_valid(&bad));
    bad=c;bad.black[0]=200;assert(!calibration_valid(&bad));
    bad=c;bad.black[1]=4096;assert(!calibration_valid(&bad));
    AnalogTracker t={0};
    AnalogDemand d=analog_demand(&t,&bad,241,132,0,0);assert(d.stop);
    d=analog_demand(&t,&c,241,132,0,0);assert(!d.stop && d.target[0]==45 && d.target[1]==45);
    d=analog_demand(&t,&c,300,132,10000,1);assert(!d.stop && d.target[0]>d.target[1]);
    d=analog_demand(&t,&c,190,132,20000,2);assert(!d.stop && d.target[0]<d.target[1]);
    d=analog_demand(&t,&c,190,450,30000,3);assert(d.decision==AL_RIGHT_GUARD && d.target[0]==60 && d.target[1]==30);
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
    /* V6 extremes retain steering direction while halving the V5 differential.
       Crossing the whole optical range changes either request by at most30mm/s. */
    t=(AnalogTracker){0};
    AnalogDemand full_black=analog_demand(&t,&c,350,132,0,0);
    AnalogDemand full_white=analog_demand(&t,&c,132,132,1,0);
    assert(full_black.target[0]==60 && full_black.target[1]==30);
    assert(full_white.target[0]==30 && full_white.target[1]==60);
    assert(full_black.target[0]-full_white.target[0]==30);
    d=analog_demand(&t,&c,241,132,2,0);assert(d.target[0]==45 && d.target[1]==45);
    LineCalibration contact={{177,183},{3639,3629},7};
    assert(calibration_valid(&contact));
    assert(line_level(1908,177,3639)==500);
    t=(AnalogTracker){0};
    d=analog_demand(&t,&contact,1908,183,0,0);assert(d.target[0]==45 && d.target[1]==45);
    /* Every valid input yields forward-only bounded wheel targets; no hidden
       spin/reverse command, including saturated and out-of-calibration ADC. */
    for(unsigned l=0;l<=4095;l+=17) for(unsigned r=0;r<=4095;r+=19) {
        t=(AnalogTracker){0};d=analog_demand(&t,&c,(uint16_t)l,(uint16_t)r,0,0);
        assert(d.level[0]<=1000 && d.level[1]<=1000);
        assert(d.target[0]>=0 && d.target[0]<=60 && d.target[1]>=0 && d.target[1]<=60);
        if(!d.stop) assert(d.target[0]>=30 && d.target[1]>=30);
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
    puts("PASS: V6 45mm/s cruise, 30..60 targets, reduced steering reversal, close-height calibration; loose start/full-black/both-dark refusal; repeat rearm/guard cycles; hands-free countdown cancellation; 25% pulse/cooldown; calibration/steering/loss/stop bounds; offline sequence");
}
