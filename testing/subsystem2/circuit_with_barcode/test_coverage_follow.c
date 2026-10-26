#include <assert.h>
#include <stdio.h>
#ifndef CIRCUIT_COVERAGE_FOLLOW
#error Test the production coverage follower with CIRCUIT_COVERAGE_FOLLOW=1
#endif
#include "smooth_follow.h"
#include "straight_pwm.h"
static const LineCalibration cal={{176,175},{3483,3498},7};
static AnalogDemand levels(CircuitTracker *t,unsigned left,unsigned right,uint64_t now,unsigned mm) {
    AnalogDemand d={.level={(uint16_t)left,(uint16_t)right}};t->loss_reason=0;
    return coverage_line_demand(t,d,now,mm);
}
static void partial_line_and_hysteresis(void) {
    Circuit c;BarcodeNav n;SmoothFollow s={0};CircuitInput i={.now=1000};
    circuit_begin(&c,&i);barcode_nav_begin(&n,&i);
    for(unsigned mirror=0;mirror<2;++mirror)for(unsigned raw=800;raw<=1000;raw+=50) {
        i.now+=10000;i.adc[mirror]=3000;i.adc[1-mirror]=raw;
        AnalogDemand d=barcode_drive_step(&c,&n,&cal,&i,1,&s,.01f);
        assert(!d.stop && d.target[0]==45 && d.target[1]==45 && d.decision==CV_CENTRE);
        assert(d.level[1-mirror]>=188 && !c.follower.core.white && !s.paired);
        float speed[2]={45,45};assert(!straight_pwm_eligible(&c,&s,&d,speed,false));
    }
    CircuitTracker t={0};AnalogDemand d=levels(&t,900,180,1000,0);assert(d.decision==CV_CENTRE);
    d=levels(&t,900,130,2000,0);assert(d.target[0]==45 && d.target[1]==45);
    d=levels(&t,900,119,3000,0);assert(d.target[0]==35 && d.target[1]==50);
    d=levels(&t,900,130,4000,0);assert(d.target[0]==30 && d.target[1]==30);
    d=levels(&t,900,180,5000,0);assert(d.target[0]==45 && d.target[1]==45);
}
static void white_timing_and_mirror(void) {
    for(unsigned mirror=0;mirror<2;++mirror) {
        CircuitTracker t={0};unsigned l=mirror?0:900,r=mirror?900:0;
        AnalogDemand d=levels(&t,l,r,1000,0);unsigned inner=mirror?1:0,outer=1-inner;
        assert(d.target[inner]==35 && d.target[outer]==50);
        d=levels(&t,l,r,30999,0);assert(d.target[inner]==35);
        d=levels(&t,l,r,31000,0);assert(d.target[inner]==25 && d.target[outer]==50);
        d=levels(&t,l,r,120999,0);assert(d.target[inner]==25);
        d=levels(&t,l,r,121000,0);assert(!d.target[inner] && d.target[outer]==45);
        /* Remain confirmed white through the60..119 band. */
        d=levels(&t,mirror?90:900,mirror?900:90,131000,0);assert(!d.target[inner]);
        d=levels(&t,900,200,141000,0); /* a partial-black return is useful */
        if(!mirror)assert(d.target[0]==45 && d.target[1]==45);
    }
    CircuitTracker t={0};(void)levels(&t,900,900,1000,0);
    (void)levels(&t,900,0,11000,0);
    AnalogDemand d=levels(&t,900,190,21000,0);assert(d.target[0]==45 && d.target[1]==45);
    assert(!t.coverage.white[1]); /* one10ms edge flash never becomes hard */
    t=(CircuitTracker){0};(void)levels(&t,900,0,1000,0);(void)levels(&t,900,0,31000,0);
    d=levels(&t,0,900,41000,0);assert(d.target[0]==50 && d.target[1]==35);
    d=levels(&t,0,900,71000,0);assert(d.target[0]==50 && d.target[1]==25);
}
static void loss_search_and_return(void) {
    CircuitTracker t={0};(void)levels(&t,900,190,1000,0);(void)levels(&t,900,190,101000,0);
    assert(circuit_memory(&t,101000)==3);
    (void)levels(&t,0,0,111000,0);
    AnalogDemand d=levels(&t,0,0,141000,0);assert(t.core.white && !d.stop && t.search.waiting);
    assert(d.target[0]==35 && d.target[1]==35);
    d=levels(&t,0,0,141000+LS_BRIDGE_US,0);assert(!d.target[0] && !d.target[1]);
    uint64_t start=141000+LS_CONFIRM_US;
    d=levels(&t,0,0,start,0);assert(t.search.active && d.decision==19);
    assert(d.target[0]==-LS_SPEED && d.target[1]==LS_SPEED);
    d=levels(&t,0,190,start+10000,0);assert(t.core.white && !d.target[0] && !d.target[1]);
    d=levels(&t,0,190,start+30000,0);assert(!t.core.white && !t.search.active && !d.stop);
    assert(d.target[0]>d.target[1]);
    t=(CircuitTracker){0};(void)levels(&t,100,100,1000,0);
    d=levels(&t,100,100,81000,0);assert(t.core.white && !d.stop);
    d=levels(&t,100,100,81000+LS_CONFIRM_US,0);assert(t.search.active);
    d=levels(&t,100,100,81000+LS_CONFIRM_US+LS_TOTAL_US,0);
    assert(d.stop && t.loss_reason==LS_TIMEOUT); /* no unbounded grey-band wait */
    Circuit c;BarcodeNav nav;SmoothFollow smooth={0};CircuitInput i={.now=1000,.adc={176,175}};
    circuit_begin(&c,&i);barcode_nav_begin(&nav,&i);nav.pending='A';
    (void)barcode_drive_step(&c,&nav,&cal,&i,1,&smooth,.01f);
    i.now+=CV_WHITE_US;(void)barcode_drive_step(&c,&nav,&cal,&i,1,&smooth,.01f);
    i.now+=LS_CONFIRM_US;d=barcode_drive_step(&c,&nav,&cal,&i,1,&smooth,.01f);
    assert(c.follower.search.active && nav.pending=='A' && !nav.allow_cross);
    nav.cross_ready=nav.cross_gate=true;nav.cross_width=18;
    i.now+=10000;d=barcode_drive_step(&c,&nav,&cal,&i,1,&smooth,.01f);
    assert(!d.stop && !c.busy && nav.pending=='A' && c.ignored==1);
}
static void mirror_alignment_and_calibration(void) {
    CircuitTracker t={0};(void)levels(&t,900,0,1000,0);(void)levels(&t,900,0,31000,0);
    CircuitTracker before=t;barcode_mirror_tracker(&t);assert(t.coverage.white[0] && !t.coverage.white[1]);
    barcode_mirror_tracker(&t);assert(!memcmp(&t,&before,sizeof(t)));
    for(unsigned right=0;right<2;++right) {
        Circuit c={.busy=true,.phase=J_ACQUIRE,.turn={.phase=J_ACQUIRE,.phase_us=1000}};
        BarcodeNav n={.busy=true,.action=right?'F':'A'};
        CircuitInput i={.now=1000,.adc={3000,800}};
        AnalogDemand d=barcode_circuit_step(&c,&n,&cal,&i,1);
        assert(!d.stop && d.target[0]==45 && d.target[1]==45);
    }
    CircuitTracker q={0};LineCalibration bad=cal;bad.mask=0;
    assert(circuit_line_demand(&q,&bad,3000,800,1000,0).stop);
}
int main(void) {
    partial_line_and_hysteresis();white_timing_and_mirror();loss_search_and_return();mirror_alignment_and_calibration();
    puts("PASS: actual3000/800..1000 ADC pairs drive straight; coverage hysteresis; mirrored gentle/firm/hard timing; brief flashes; sign reversal; lower-confidence bounded recovery;20ms return; mirrored branch alignment; unchanged raw levels/calibration.");
}
