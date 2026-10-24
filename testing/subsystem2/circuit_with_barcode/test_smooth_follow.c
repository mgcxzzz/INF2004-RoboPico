#include <assert.h>
#include <stdio.h>
#include "smooth_follow.h"
static const LineCalibration cal={{136,138},{261,299},7};
static AnalogDemand apply(SmoothFollow *s,Circuit *c,unsigned l,unsigned r) {
    AnalogDemand d={.level={(uint16_t)l,(uint16_t)r},.decision=SYM_STRAIGHT};
    symmetric_targets((int)l-(int)r,d.target);
    if(d.target[0]!=d.target[1])d.decision=d.target[0]<d.target[1]?SYM_LEFT:SYM_RIGHT;
    smooth_follow_apply(s,c,.01f,&d);return d;
}
static void noise_and_mirror(void) {
    Circuit c={.phase=C_FOLLOW};SmoothFollow a={0},b={0};
    int prev_raw[2]={45,45},prev[2]={45,45},raw_change=0,filtered_change=0;
    (void)apply(&a,&c,700,700);(void)apply(&b,&c,700,700);
    for(unsigned k=0;k<400;++k) {
        unsigned l=k%2?500:730,r=k%2?730:500;
        int16_t raw[2];symmetric_targets((int)l-(int)r,raw);
        AnalogDemand x=apply(&a,&c,l,r),y=apply(&b,&c,r,l);
        assert(x.target[0]==y.target[1] && x.target[1]==y.target[0]);
        for(unsigned w=0;w<2;++w) {
            raw_change+=abs(raw[w]-prev_raw[w]);prev_raw[w]=raw[w];
            filtered_change+=abs(x.target[w]-prev[w]);prev[w]=x.target[w];
            assert(x.target[w]>=40 && x.target[w]<=50);
        }
    }
    assert(filtered_change<raw_change/3);
    printf("target variation: old=%d smooth=%d\n",raw_change,filtered_change);
}
static void sustained_bends_and_bypass(void) {
    for(unsigned mirror=0;mirror<2;++mirror) {
        Circuit c={.phase=C_FOLLOW};SmoothFollow s={0};(void)apply(&s,&c,700,700);
        unsigned l=mirror?200:800,r=mirror?800:200;int16_t raw[2];symmetric_targets((int)l-(int)r,raw);
        AnalogDemand d=apply(&s,&c,l,r);
        assert(abs(d.target[0]-45)<abs(raw[0]-45));
        for(unsigned k=0;k<20;++k)d=apply(&s,&c,l,r);
        assert(abs(d.target[0]-raw[0])<=1 && abs(d.target[1]-raw[1])<=1);
        /* Strong departures pass through in this control tick, even when
         * their direction reverses a previous mild correction. */
        l=mirror?1000:0;r=mirror?0:1000;d=apply(&s,&c,l,r);symmetric_targets((int)l-(int)r,raw);
        assert(!s.active && d.target[0]==raw[0] && d.target[1]==raw[1]);
        c.follower.core.white=true;d=apply(&s,&c,500,200);symmetric_targets(300,raw);
        assert(!s.active && d.target[0]==raw[0] && d.target[1]==raw[1]);
    }
    Circuit c={.phase=C_FOLLOW};SmoothFollow s={0};
    AnalogDemand stop={.stop=true,.level={700,700},.target={0,0},.decision=AL_LOST};
    (void)apply(&s,&c,700,700);smooth_follow_apply(&s,&c,.01f,&stop);
    assert(!s.active && stop.stop && !stop.target[0] && !stop.target[1]);
    for(unsigned phase=J_CENTRE;phase<=J_ACQUIRE;++phase) {
        c.phase=(uint8_t)phase;c.busy=true;
        AnalogDemand d={.level={700,500},.target={0,45},.decision=SYM_LEFT},before=d;
        smooth_follow_apply(&s,&c,.01f,&d);
        assert(!s.active && !memcmp(&d,&before,sizeof(d)));
    }
}
static void weak_boundary_and_profile_transition(void) {
    Circuit c={.phase=C_FOLLOW};SmoothFollow s={0};AnalogDemand d={0};
    int prev[2]={45,45};(void)apply(&s,&c,700,700);
    for(unsigned k=0;k<100;++k) {
        /* Crossing strongest400 switches old faint/normal target scaling.
         * Exercise the actual demand, not a duplicate of its implementation. */
        CircuitTracker tracker={0};
        d=circuit_line_demand(&tracker,&cal,136,k%2?201:203,1000+k*10000,0);
        smooth_follow_apply(&s,&c,.01f,&d);
        for(unsigned w=0;w<2;++w) {assert(abs(d.target[w]-prev[w])<=4);prev[w]=d.target[w];}
    }
    for(unsigned profile=1;profile<=3;++profile) {
        CircuitInput i={.now=1000,.adc={261,299}};BarcodeNav n;SmoothFollow smooth={0};
        circuit_begin(&c,&i);barcode_nav_begin(&n,&i);
        d=barcode_drive_step(&c,&n,&cal,&i,profile,&smooth,.01f);
        assert(d.target[0]==45 && d.target[1]==45);
        for(unsigned k=0;k<150;++k) {i.now+=10000;d=barcode_drive_step(&c,&n,&cal,&i,profile,&smooth,.01f);}
        assert(abs(d.target[0]-circuit_cruise(profile))<=1 && d.target[0]==d.target[1]);
        i.now+=10000;i.adc[0]=221;
        d=barcode_drive_step(&c,&n,&cal,&i,profile,&smooth,.01f);
        assert(!c.fast && d.target[0]<=50 && d.target[1]<=50);
    }
}
int main(void) {
    noise_and_mirror();sustained_bends_and_bypass();weak_boundary_and_profile_transition();
    puts("PASS: reduced alternating target variation; mirrored steering; sustained bends converge; urgent/stop/search/junction bypass; weak400 boundary slew; all speed profiles retained.");
}
