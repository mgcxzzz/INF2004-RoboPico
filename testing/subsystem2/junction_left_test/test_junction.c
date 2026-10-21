#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "junction_logic.h"
#include "signed_wheel.h"
#include "../line_motion_integration/logic.h"

static const LineCalibration cal={{184,184},{3464,3692},7};
static JunctionInput sample(uint64_t now,float mean,bool side) {
    JunctionInput i={.now=now,.wheel={mean,mean},.adc={3464,3692},.side_black=side};
    return i;
}
static Junction centre_fixture(void) {
    Junction j;JunctionInput i=sample(1000,0,false);j_begin(&j,true,&i);
    (void)junction_step(&j,&cal,&i);i=sample(101000,4,false);(void)junction_step(&j,&cal,&i);
    assert(j.side_armed);
    i=sample(200000,100,true);(void)junction_step(&j,&cal,&i);
    i=sample(220000,101,true);(void)junction_step(&j,&cal,&i);
    assert(j.phase==J_CENTRE && fabsf(j.leading_mm-100)<0.01f);
    i=sample(800000,118,false);(void)junction_step(&j,&cal,&i);
    i=sample(820000,119,false);(void)junction_step(&j,&cal,&i);
    assert(j.striped && fabsf(j.centre_mm-201)<0.01f);
    return j;
}
static void test_geometry_and_qualification(void) {
    Junction j=centre_fixture();JunctionInput i=sample(1000000,199,false);
    AnalogDemand d=junction_step(&j,&cal,&i);assert(j.phase==J_CENTRE && d.target[0]>0);
    i=sample(1010000,200,false);d=junction_step(&j,&cal,&i);
    assert(j.phase==J_SETTLE && d.target[0]==0 && d.target[1]==0);
    i=sample(1020000,200,false);(void)junction_step(&j,&cal,&i);
    i=sample(1310000,200,false);d=junction_step(&j,&cal,&i);
    assert(j.phase==J_TURN && d.target[0]==0 && d.target[1]==0);
    i.now+=10000;d=junction_step(&j,&cal,&i);assert(d.target[0]<0 && d.target[1]>0);
    /* Old-line black cannot immediately finish the turn. */
    assert(!j.off_line && !d.stop);
    j=centre_fixture();i=sample(900000,140,true);i.wheel[0]+=25;
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_CENTRE_DRIFT);
    i=sample(1000,0,true);j_begin(&j,true,&i);(void)junction_step(&j,&cal,&i);
    i.now+=20000;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_NO_CROSS);
    /* A brief black pulse must not count as a cross. */
    i=sample(1000,0,false);j_begin(&j,true,&i);(void)junction_step(&j,&cal,&i);
    i=sample(101000,5,false);(void)junction_step(&j,&cal,&i);
    i=sample(200000,10,true);(void)junction_step(&j,&cal,&i);
    i=sample(210000,11,false);d=junction_step(&j,&cal,&i);assert(j.phase==J_APPROACH && !d.stop);
    j=centre_fixture();j.striped=false;i=sample(900000,151,true);
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_BAD_STRIPE);
    j=centre_fixture();i=sample(900000,120,false);i.path=900;
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_PATH_LIMIT);
}
static void simulate(bool turn,bool branch) {
    Junction j;JunctionInput i=sample(1000,0,false);j_begin(&j,turn,&i);
    bool centred=false,rotated=false,exited=false;
    for(unsigned tick=0;tick<2600;++tick) {
        float mean=j_mean(&i);
        i.side_black=mean>=100 && mean<118;
        i.adc[0]=3464;i.adc[1]=3692;
        if(j.phase==J_TURN) {
            float angle=-(heading_deg(i.wheel[0]-j.turn_origin[0],i.wheel[1]-j.turn_origin[1]));
            if(angle>3 && !(branch && angle>=88 && angle<=95)) i.adc[0]=i.adc[1]=184;
        }
        AnalogDemand d=junction_step(&j,&cal,&i);
        if(j.phase==J_SETTLE) centred=true;
        if(j.phase==J_TURN_SETTLE) rotated=true;
        if(j.phase==J_EXIT) exited=true;
        if(d.stop) {
            assert(centred);
            if(turn && !branch) assert(j.reason==J_BRANCH_MISSING && !exited);
            else {
                assert(j.reason==J_COMPLETE);
                assert(rotated==turn && exited==turn);
                if(turn) assert(j.turn_degrees>=88 && j.turn_degrees<96);
            }
            return;
        }
        for(unsigned w=0;w<2;++w) {
            assert(abs(d.target[w])<=50);
            i.speed[w]=(float)d.target[w];i.wheel[w]+=i.speed[w]*0.01f;
            i.path+=fabsf(i.speed[w])*0.005f;
        }
        i.now+=10000;
    }
    assert(!"sequence failed to terminate");
}
static void test_turn_rejects_false_branch(void) {
    Junction j={.phase=J_TURN,.phase_us=1000};JunctionInput i=sample(1000,0,false);
    i.wheel[0]=-86;i.wheel[1]=86;
    AnalogDemand d=junction_step(&j,&cal,&i);
    assert(!d.stop && !j.off_line && j.phase==J_TURN); /* never left original black */
    j.off_line=true;i.adc[1]=184;
    for(unsigned n=0;n<10;++n) { i.now+=10000;d=junction_step(&j,&cal,&i); }
    assert(j.phase==J_TURN && !j.line_timing); /* only one sensor on branch */
    i.now=6001000;d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_TURN_TIMEOUT);
    j=(Junction){.phase=J_SETTLE,.phase_us=1000};i=sample(2001000,100,false);i.speed[0]=10;
    d=junction_step(&j,&cal,&i);assert(d.stop && j.reason==J_SETTLE_TIMEOUT);
}
static void test_signed_controller_and_stops(void) {
    for(unsigned w=0;w<2;++w) for(int desired=-50;desired<=50;++desired) {
        SignedWheel p={0};
        for(unsigned n=0;n<100;++n) {
            int32_t u=junction_wheel_step(&p,w,(int16_t)desired,desired*0.8f,0.01f,1000+n*10000);
            assert(abs(u)<=250 && (desired==0?u==0:u*desired>=0));
            DualLevels v=dual_levels(w,u);assert(v.a<=250 && v.b<=250 && !(v.a && v.b));
        }
        assert(junction_wheel_step(&p,w,0,40,0.01f,2000000)==0);
        assert(p.pi.integral==0 && p.pi.ramp==0 && !p.kick.active);
    }
    DualGuard g={0};assert(guard_start(&g,1000000,true,false));
    g.deadline_us=26000000;guard_tick(&g,1001000,true,true);assert(!g.active && g.reason==WHY_GP20);
    g=(DualGuard){0};assert(guard_start(&g,1000000,true,false));
    guard_tick(&g,1001000,false,false);assert(!g.active && g.reason==WHY_RELEASE);
    g=(DualGuard){0};assert(guard_start(&g,1000000,true,false));
    guard_tick(&g,1100000,true,false);assert(!g.active && g.reason==WHY_LATE);
    assert(side_valid(&(SideCalibration){0,1,3}));assert(side_valid(&(SideCalibration){1,0,3}));
    assert(!side_valid(&(SideCalibration){1,1,3}));assert(!side_valid(&(SideCalibration){0,1,1}));
}
int main(void) {
    test_geometry_and_qualification();simulate(false,true);simulate(true,true);simulate(true,false);
    test_turn_rejects_false_branch();test_signed_controller_and_stops();
    puts("PASS: junction qualification/geometry/staged centring/settling/left branch/rejection/signed PWM/guards");
    return 0;
}
