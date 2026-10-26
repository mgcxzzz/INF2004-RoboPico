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
    now=first+1000000;d=circuit_line_demand(&t,&cal,3464,184,now,2);
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
    static const int endpoints[]={35,60,90,120,150};
    for(int direction=-1;direction<=1;direction+=2) {
        LineSearch s={0};line_search_begin(&s,1000,10,direction);uint64_t now=1000;
        for(unsigned stage=0;stage<5;++stage) {
            LineSearchDemand d=line_search_step(&s,now,10,false);
            assert(!d.reason && d.stage==stage);
            int wanted=stage%2?-direction:direction;
            assert(d.target[0]==-wanted*LS_SPEED && d.target[1]==wanted*LS_SPEED);
            d=line_search_step(&s,now,10,true);assert(!d.target[0] && !d.target[1]);
            s.angle=wanted*(endpoints[stage]-5);
            d=line_search_step(&s,now,10,false);assert(abs(d.target[0])==LS_CONFIRM_SPEED);
            s.angle=wanted*endpoints[stage];s.speed=10;now+=10000;
            d=line_search_step(&s,now,10,false);assert(s.settling && !d.target[0] && !d.target[1]);
            s.speed=0;now+=10000;d=line_search_step(&s,now,10,false);
            now+=79999;d=line_search_step(&s,now,10,false);assert(s.stage==stage);
            ++now;d=line_search_step(&s,now,10,false);
            if(stage<4)assert(s.stage==stage+1 && !d.reason);
            else assert(d.reason==LS_TIMEOUT);
        }
        s=(LineSearch){0};line_search_begin(&s,1000,10,direction);
        assert(!line_search_step(&s,1000+LS_TOTAL_US-1,10+LS_MAX_MM-1,false).reason);
        assert(line_search_step(&s,1000+LS_TOTAL_US,10,false).reason==LS_TIMEOUT);
        assert(line_search_step(&s,1100,10+LS_MAX_MM,false).reason==LS_DISTANCE);
        s.centre=30;assert(line_search_step(&s,1100,10,false).reason==LS_DRIFT);
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
    assert(d.decision==19); /* Time cannot advance an encoder-angle endpoint. */
    /* The opposite sensor alone can confirm a line. Both-black is not
       required, preserving acquisition on a bend immediately after a turn. */
    l=direction>0?184:3464;r=direction>0?3692:184;
    now+=10000;d=circuit_line_demand(&t,&cal,l,r,now,0);
    assert(t.search.active && !d.target[0] && !d.target[1]);
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
        DualLevels levels=maneuver_levels(outer,pwm,true);
        assert((outer==0?levels.a:levels.b)==pwm);
        assert(line_search_outer(true,outer==0?45:0,outer==1?45:0)==(int)outer);
        assert(!maneuver_wheel_step(&power,&p,outer,0,20,.01f,2000));
    }
    assert(line_search_outer(false,0,45)==-1 && line_search_outer(true,30,45)==-1);
    assert(line_search_outer(true,-100,100)==1 && line_search_outer(true,100,-100)==0);
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
    assert(!d.stop && c.phase==J_ACQUIRE && d.decision==19);
    i.now+=10000;i.path=LS_MAX_MM;
    d=barcode_circuit_step(&c,&nav,&cal,&i,1);assert(d.stop && c.reason==LS_DISTANCE);
}
static void stationary_return_model(void) {
    for(int direction=-1;direction<=1;direction+=2) {
        Circuit c;BarcodeNav nav;CircuitInput i={.now=1000,.adc={184,184}};
        circuit_begin(&c,&i);barcode_nav_begin(&nav,&i);
        c.follower.core.direction=direction;c.follower.core.direction_us=i.now;
        AnalogDemand d=barcode_circuit_step(&c,&nav,&cal,&i,1);
        i.now+=LS_CONFIRM_US;d=barcode_circuit_step(&c,&nav,&cal,&i,1);
        assert(c.follower.search.active);
        double travel=0;bool sighted=false,found=false;unsigned ticks=0;
        for(;ticks<1200 && !d.stop;++ticks) {
            i.speed[0]=d.target[0];i.speed[1]=d.target[1];
            for(unsigned w=0;w<2;++w)i.wheel[w]+=i.speed[w]*.01;
            travel+=(fabsf(i.speed[0])+fabsf(i.speed[1]))*.005;i.path=travel;i.now+=10000;
            double angle=(i.wheel[1]-i.wheel[0])*180/(110*3.14159265358979323846);
            bool on_line=fabs(angle+45*direction)<2;
            i.adc[0]=on_line?3464:184;i.adc[1]=184;
            d=barcode_circuit_step(&c,&nav,&cal,&i,1);
            assert(fabs((i.wheel[0]+i.wheel[1])*.5)<.001);
            if(on_line && c.follower.core.white) {sighted=true;assert(!d.target[0] && !d.target[1]);}
            if(sighted && !c.follower.core.white) {found=true;break;}
        }
        assert(found && !d.stop && !c.follower.search.active);
        printf("stationary model: first=%d, return after%u ms, centre=%.2fmm\n",direction,ticks*10,(i.wheel[0]+i.wheel[1])*.5);
    }
    CircuitTracker a={0};a.search=(LineSearch){.first=1,.angle=54,.origin_angle=12,.centre=20,.origin_centre=19};
    CircuitTracker old=a;barcode_mirror_tracker(&a);assert(a.search.first==-1 && a.search.angle==-54);
    barcode_mirror_tracker(&a);assert(!memcmp(&a,&old,sizeof(a)));
}
int main(void) {
    continuous_white_wait();schedule_tests();mirrored_reacquire(1);mirrored_reacquire(-1);
    short_gap_and_visible();effort_and_queue();alignment_allows_search_budget();stationary_return_model();
    puts("PASS: encoder-angle stationary sweeps/both directions; settled reversals; zero-tick timeout/travel/drift bounds;1.5s wait; flash handling; stop-on-sighting/opposite-sensor acquisition; pending command preserved/no junction while lost.");
}
