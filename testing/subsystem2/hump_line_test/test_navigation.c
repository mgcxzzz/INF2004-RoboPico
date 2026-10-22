#include <assert.h>
#include <stdio.h>
#include "hump_navigation.h"

static const LineCalibration cal={{184,184},{3464,3692},7};
static uint16_t raw(unsigned w,unsigned level) {
    return (uint16_t)(cal.white[w]+((cal.black[w]-cal.white[w])*level+999)/1000);
}
static HumpNavigationOutput step(HumpNavigation *n,unsigned l,unsigned r,uint64_t now,
                                  float path,float heading) {
    return hump_navigation_step(n,&cal,raw(0,l),raw(1,r),now,path,heading,40,40,HUMP_ASSIST);
}
static HumpNavigation armed(void) {
    HumpNavigation n={0};
    for(unsigned k=0;k<=15;++k) {
        HumpNavigationOutput o=step(&n,900,900,1000000+k*10000,0.4f*k,2.5f);
        assert(!o.demand.stop && !n.active);
        assert(n.armed==(k==15));
    }
    return n;
}
static HumpNavigation carrying(void) {
    HumpNavigation n=armed();
    HumpNavigationOutput o=step(&n,51,338,1200000,10,2.5f);
    assert(n.active && !n.armed && o.assist_allowed && !o.demand.stop);
    assert(o.demand.decision==HUMP_CARRY && o.demand.target[0]==40 && o.demand.target[1]==40);
    return n;
}
static void bounded_loss(void) {
    HumpNavigation n=carrying();
    HumpNavigationOutput o=step(&n,39,132,1700000,15,2.5f);
    assert(!o.demand.stop && o.assist_allowed && o.demand.decision==HUMP_CARRY);
    /* Continuously BOTH WHITE past the old1s and proposed2s cutoffs. */
    o=step(&n,0,0,2200000,30,2.5f);
    assert(!o.demand.stop && o.assist_allowed);
    o=step(&n,0,0,3200000,70,2.5f);
    assert(!o.demand.stop && o.assist_allowed);
    o=step(&n,0,0,7199999,249.999f,2.5f);
    assert(!o.demand.stop);
    o=step(&n,0,0,7200000,249.999f,2.5f);
    assert(o.demand.stop && o.reason==HUMP_WHY_TIME && !o.assist_allowed);
    assert(o.demand.target[0]==0 && o.demand.target[1]==0);
    n=carrying();o=step(&n,0,0,1210000,250,2.5f);
    assert(o.demand.stop && o.reason==HUMP_WHY_DISTANCE);
    n=carrying();o=step(&n,0,0,1210000,10,10.5f);
    assert(!o.demand.stop && o.demand.target[0]==32 && o.demand.target[1]==48);
    o=step(&n,0,0,1220000,10,10.51f);
    assert(o.demand.stop && o.reason==HUMP_WHY_HEADING);
    n=carrying();o=step(&n,0,0,1210000,10,-5.5f);
    assert(!o.demand.stop && o.demand.target[0]==48 && o.demand.target[1]==32);
}
static void eligibility(void) {
    HumpNavigation n={0};HumpNavigationOutput o=step(&n,0,0,1000000,0,0);
    assert(o.demand.stop && !n.active); /* all-white has no unconditional grant */
    n=armed();o=step(&n,0,0,2150001,10,2.5f);
    assert(o.demand.stop && !n.active && !n.armed); /* stale centred observation */
    n=armed();o=step(&n,0,0,1200000,10,11);
    assert(o.demand.stop && !n.active); /* heading outside entry bounds */
    n=(HumpNavigation){0};
    for(unsigned k=0;k<50;++k) {
        o=hump_navigation_step(&n,&cal,raw(0,900),raw(1,900),1000000+k*10000,
                              0,0,0,0,HUMP_ASSIST);
        assert(!n.armed); /* parked black readings do not arm */
    }
    for(unsigned k=0;k<50;++k) o=step(&n,900,400,1500000+k*10000,0,0);
    assert(!n.armed); /* large optical asymmetry is not trusted straight tracking */
}
static void reacquisition_and_flicker(void) {
    HumpNavigation n=carrying();
    HumpNavigationOutput o=step(&n,900,900,1300000,14,2.5f);
    assert(n.active && o.demand.decision==HUMP_REACQUIRE);
    o=step(&n,900,900,1379999,17,2.5f);assert(n.active);
    o=step(&n,900,900,1380000,17,2.5f);
    assert(!n.active && !n.armed && !o.demand.stop && o.demand.decision==SYM_STRAIGHT);
    o=step(&n,0,0,1390000,18,2.5f);assert(o.demand.stop); /* no immediate blind rearm */
    n=carrying();
    for(unsigned k=0;k<590;++k) {
        /* One sensor black or brief both-black flashes cannot extend deadline. */
        unsigned l=k%6<3?900:0,r=k%6<2?900:0;
        o=step(&n,l,r,1210000+k*10000,11,2.5f);
        assert(n.active && !o.demand.stop && n.carry_since==1200000);
    }
    o=step(&n,900,900,7200000,11,2.5f);
    assert(o.demand.stop && o.reason==HUMP_WHY_TIME);
    n=carrying();
    o=step(&n,900,900,6200000,210,2.5f);
    assert(n.active && o.demand.decision==HUMP_REACQUIRE);
    o=step(&n,900,900,6280000,213,2.5f);
    assert(!n.active && !o.demand.stop); /* reacquisition in the extended window */
}
static void ordinary_white_rules_unchanged(void) {
    HumpNavigation n={0};
    HumpNavigationOutput o=step(&n,900,0,1000000,0,0);
    assert(!n.armed && !n.active);
    o=step(&n,0,0,1010000,1,0);
    assert(!o.demand.stop && !n.active && !o.assist_allowed);
    o=step(&n,0,0,1659999,20,0);assert(!o.demand.stop);
    o=step(&n,0,0,1660000,20,0);assert(o.demand.stop); /* unarmed fallback650ms */
}
static void normal_equivalence_and_assistance(void) {
    for(unsigned profile=0;profile<=1;++profile) {
        HumpNavigation n={0};SymmetricTracker ref={0};
        for(unsigned k=0;k<100;++k) {
            unsigned l=k<30?900:k<60?100:900,r=k<60?900:100;
            uint64_t now=1000000+k*10000;
            HumpNavigationOutput o=hump_navigation_step(&n,&cal,raw(0,l),raw(1,r),now,k,0,40,40,profile);
            AnalogDemand d=symmetric_demand(&ref,&cal,raw(0,l),raw(1,r),now,k);
            assert(o.demand.stop==d.stop && o.demand.decision==d.decision);
            assert(o.demand.target[0]==d.target[0] && o.demand.target[1]==d.target[1]);
        }
        if(!profile) {
            HumpNavigationOutput o=hump_navigation_step(&n,&cal,184,184,3000000,110,0,40,40,profile);
            assert(!n.active && o.demand.stop); /* profile1 retains V3 loss policy */
        }
    }
    HumpNavigation n=carrying();HumpWheel loaded={.pi={.ramp=40,.integral=60}};
    bool raised=false;
    for(unsigned k=0;k<45;++k) {
        uint64_t now=1210000+k*10000;
        HumpNavigationOutput o=step(&n,39,132,now,10,2.5f);
        int32_t pwm=hump_wheel_step(&loaded,1,o.demand.target[1],0,0.01f,now,HUMP_ASSIST,o.assist_allowed);
        assert(!o.demand.stop && pwm<=420);
        if(pwm>300) raised=true;
    }
    assert(raised && loaded.extra>0); /* weak optics no longer revoke the grant */
}
static void orange_v3_log_regression(void) {
    /* Actual sampled levels/path/heading from the supplied failed V3 log.
       Test optical decisions only; this does not simulate future motor motion. */
    const struct { unsigned ms,l,r;float path,heading,vl,vr; } row[]={
        {4313,893,782,154.22f,-1.02f,31.8f,38},
        {4440,788,678,158.14f,-1.22f,30,33.7f},
        {4567,533,564,162.34f,-1.63f,32,38.3f},
        {4694,265,502,166.18f,-1.59f,27.5f,27.7f},
        {4821,261,401,169.90f,-1.42f,33.5f,31},
        {4948,51,338,173.54f,-1.58f,26.6f,28.4f},
        {5075,10,181,177.38f,-1.70f,28.3f,31},
        {5202,13,156,179.44f,-0.87f,19.3f,3.6f},
        {5329,110,189,179.44f,-0.87f,0.6f,0.1f},
        {5406,39,132,180.03f,-1.33f,3.3f,15.1f}
    };
    HumpNavigation n={0};SymmetricTracker old={0};
    for(unsigned k=0;k<sizeof(row)/sizeof(row[0]);++k) {
        uint64_t now=(uint64_t)row[k].ms*1000;
        uint16_t l=raw(0,row[k].l),r=raw(1,row[k].r);
        HumpNavigationOutput o=hump_navigation_step(&n,&cal,l,r,now,row[k].path,row[k].heading,
                                                    row[k].vl,row[k].vr,HUMP_ASSIST);
        AnalogDemand v3=symmetric_demand(&old,&cal,l,r,now,(uint32_t)row[k].path);
        assert(!o.demand.stop);
        if(row[k].ms>=4948) assert(n.active && o.assist_allowed);
        if(row[k].ms==5406) assert(v3.stop && o.demand.decision==HUMP_CARRY);
    }
}
int main(void) {
    bounded_loss();eligibility();reacquisition_and_flicker();normal_equivalence_and_assistance();
    orange_v3_log_regression();ordinary_white_rules_unchanged();
    puts("PASS: both-white carry past1s/2s, exact6s/240mm bounds,8deg heading,80ms reacquisition after5s, no flicker renewal, unarmed650ms fallback retained");
    return 0;
}
