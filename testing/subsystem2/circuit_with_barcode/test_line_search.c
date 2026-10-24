#include <assert.h>
#include <stdio.h>
#include "barcode_record.h"
#include "../circuit_left_test/circuit_wheel.h"
#include "maneuver_power.h"
#ifndef CIRCUIT_SWEEP_SEARCH
#error Build with CIRCUIT_SWEEP_SEARCH=1
#endif
static const LineCalibration cal={{184,184},{3464,3692},7};
static void continuous_white_wait(void) {
    CircuitTracker t={0};uint64_t now=1000;
    (void)circuit_line_demand(&t,&cal,3464,184,now,0);
    now+=10000;AnalogDemand d=circuit_line_demand(&t,&cal,184,184,now,0);
    assert(t.search.waiting && !t.search.active);
    uint64_t first=now;
    now=first+LS_BRIDGE_US;d=circuit_line_demand(&t,&cal,184,184,now,2);
    assert(!d.stop && !d.target[0] && !d.target[1] && !t.search.active);
    now=first+1900000;d=circuit_line_demand(&t,&cal,3464,184,now,2);
    assert(!t.search.waiting && !t.search.active && t.core.white);
    now+=20000;d=circuit_line_demand(&t,&cal,184,184,now,2);uint64_t second=now;
    assert(t.search.waiting && t.search.wait_since==second);
    now=first+LS_CONFIRM_US;d=circuit_line_demand(&t,&cal,184,184,now,2);
    assert(!t.search.active && !d.target[0] && !d.target[1]);
    now=second+LS_CONFIRM_US-1;d=circuit_line_demand(&t,&cal,184,184,now,2);assert(!t.search.active);
    ++now;d=circuit_line_demand(&t,&cal,184,184,now,2);assert(t.search.active && d.decision==19);
    BarcodeNav nav={.start_us=1000};
    barcode_recovery_events(&nav,now,0,true,true,false,11,&t,&d,0);
    assert(nav.event_count==1 && nav.events[0].type==B_RECOVERY_SWEEP && nav.events[0].flags==1);
    t.core.white=false;t.search=(LineSearch){0};d=(AnalogDemand){.target={45,45}};
    barcode_recovery_events(&nav,now+10000,1,true,false,true,19,&t,&d,0);
    assert(nav.event_count==2 && nav.events[1].type==B_RECOVERY_FOUND);
    LineSearch s={0};assert(!line_search_ready(&s,1000,0,0));
    assert(!line_search_ready(&s,LS_CONFIRM_US,170,190));
    assert(line_search_ready(&s,1000+LS_CONFIRM_US,170,190));
    assert(!line_search_ready(&s,1001+LS_CONFIRM_US,200,0) && !s.waiting);
}
static void schedule_tests(void) {
    const unsigned at[]={0,350000,1050000,2100000,3500000};
    for(int direction=-1;direction<=1;direction+=2) {
        LineSearch s;line_search_begin(&s,1000,10,direction);
        for(unsigned stage=0;stage<5;++stage) {
            LineSearchDemand d=line_search_step(&s,1000+at[stage],10,false);
            assert(!d.reason && d.stage==stage);
            int wanted=stage%2?-direction:direction;
            assert(d.target[wanted>0?1:0]==LS_SPEED && !d.target[wanted>0?0:1]);
            d=line_search_step(&s,1000+at[stage],10,true);
            assert(d.target[wanted>0?1:0]==LS_CONFIRM_SPEED);
            if(stage) assert(line_search_step(&s,999+at[stage],10,false).stage==stage-1);
        }
        assert(!line_search_step(&s,1000+LS_TOTAL_US-1,10+LS_MAX_MM-1,false).reason);
        assert(line_search_step(&s,1000+LS_TOTAL_US,10,false).reason==LS_TIMEOUT);
        assert(line_search_step(&s,1100,10+LS_MAX_MM,false).reason==LS_DISTANCE);
    }
}
static void mirrored_reacquire(int direction) {
    CircuitTracker t={0};uint64_t now=1000;
    uint16_t l=direction>0?3464:184,r=direction>0?184:3692;
    AnalogDemand d=circuit_line_demand(&t,&cal,l,r,now,0);assert(!d.stop);
    assert(t.core.direction==direction);
    now+=10000;d=circuit_line_demand(&t,&cal,184,184,now,0);
    assert(!d.stop && !t.search.active);
    now+=LS_CONFIRM_US-1;d=circuit_line_demand(&t,&cal,184,184,now,0);
    assert(!d.stop && !t.search.active);
    ++now;d=circuit_line_demand(&t,&cal,184,184,now,0);
    assert(t.search.active && t.search.first==direction && d.decision==19);
    assert(d.target[direction>0?1:0]==LS_SPEED);
    /* A20ms line flash does not restart or terminate the search budget. */
    uint64_t origin=t.search.since;
    now+=20000;d=circuit_line_demand(&t,&cal,l,r,now,0);
    now+=20000;d=circuit_line_demand(&t,&cal,184,184,now,0);
    assert(t.search.active && t.search.since==origin && !d.stop);
    now=origin+350000;d=circuit_line_demand(&t,&cal,184,184,now,0);
    assert(d.decision==20 && d.target[direction>0?0:1]==LS_SPEED);
    /* The opposite sensor alone can confirm a line. Both-black is not
       required, preserving acquisition on a bend immediately after a turn. */
    l=direction>0?184:3464;r=direction>0?3692:184;
    now+=10000;d=circuit_line_demand(&t,&cal,l,r,now,0);
    assert(t.search.active && d.target[direction>0?0:1]==LS_CONFIRM_SPEED);
    now+=40000;d=circuit_line_demand(&t,&cal,l,r,now,0);
    assert(!d.stop && !t.core.white && !t.search.active);
    int16_t expected[2];symmetric_targets(direction>0?-1000:1000,expected);
    assert(d.target[0]==expected[0] && d.target[1]==expected[1]);
}
static void short_gap_and_visible(void) {
    CircuitTracker t={0};uint64_t now=1000;
    circuit_line_demand(&t,&cal,3464,184,now,0);
    now+=10000;circuit_line_demand(&t,&cal,184,184,now,0);
    now+=50000;circuit_line_demand(&t,&cal,3464,184,now,0);
    now+=40000;AnalogDemand d=circuit_line_demand(&t,&cal,3464,184,now,0);
    assert(!d.stop && !t.search.active && !t.core.white);
    for(unsigned k=0;k<1000;++k) {
        now+=10000;d=circuit_line_demand(&t,&cal,3464,184,now,k);
        assert(!d.stop && !t.search.active && d.target[0]==0 && d.target[1]==45);
    }
    LineCalibration bad=cal;bad.mask=0;t=(CircuitTracker){0};
    assert(circuit_line_demand(&t,&bad,184,184,now,0).stop && !t.search.active);
}
static void effort_and_queue(void) {
    for(unsigned outer=0;outer<2;++outer) {
        SignedWheel p={0};ManeuverPower power={0};
        int32_t pwm=maneuver_wheel_step(&power,&p,outer,LS_SPEED,0,.01f,1000);
        assert(pwm==M_PEAK && p.peak_active && p.peak_used);
        DualLevels levels=maneuver_levels(outer,pwm,false);
        assert((outer==0?levels.a:levels.b)==pwm);
        assert(line_search_outer(true,outer==0?45:0,outer==1?45:0)==(int)outer);
        assert(!maneuver_wheel_step(&power,&p,outer,0,20,.01f,2000));
    }
    assert(line_search_outer(false,0,45)==-1 && line_search_outer(true,30,45)==-1);
    Circuit c;BarcodeNav n;CircuitInput i={.now=1000,.adc={184,184}};
    circuit_begin(&c,&i);barcode_nav_begin(&n,&i);
    n.pending='A';n.cross_ready=n.cross_gate=true;n.cross_width=18;
    AnalogDemand d=barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(!d.stop && !c.busy && n.pending=='A' && !n.allow_cross && c.ignored==1);
    i.now+=LS_CONFIRM_US;d=barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(!d.stop && c.follower.search.active && d.decision==19);
    i.now+=LS_TOTAL_US;d=barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(d.stop && c.fault && c.reason==LS_TIMEOUT && n.pending=='A');
}
static void alignment_allows_search_budget(void) {
    Circuit c={.busy=true,.phase=J_ACQUIRE,.turn={.phase=J_ACQUIRE,.phase_us=1000}};
    BarcodeNav nav={.busy=true,.action='A'};
    CircuitInput i={.now=1000,.adc={184,184}};
    AnalogDemand d=barcode_circuit_step(&c,&nav,&cal,&i,1);assert(!d.stop);
    i.now+=LS_CONFIRM_US;d=barcode_circuit_step(&c,&nav,&cal,&i,1);assert(c.joining.search.active && !d.stop);
    i.now+=3500000;i.path=200;
    d=barcode_circuit_step(&c,&nav,&cal,&i,1);
    assert(!d.stop && c.phase==J_ACQUIRE && d.decision==23);
    i.now+=10000;i.path=LS_MAX_MM;
    d=barcode_circuit_step(&c,&nav,&cal,&i,1);assert(d.stop && c.reason==LS_DISTANCE);
}
int main(void) {
    continuous_white_wait();schedule_tests();mirrored_reacquire(1);mirrored_reacquire(-1);
    short_gap_and_visible();effort_and_queue();alignment_allows_search_budget();
    puts("PASS: five widening stages/both directions; zero-tick timeout/travel bound; brief gap/flash handling; opposite-sensor acquisition; visible-line parity; invalid-cal rejection; maneuver60%/bounded75% effort; off-wheel zero; pending command preserved/no junction while lost.");
}
