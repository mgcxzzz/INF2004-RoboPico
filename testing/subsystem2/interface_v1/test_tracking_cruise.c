#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tracking_cruise.h"
static void stable_unequal(void) {
    for(unsigned profile=1;profile<=3;profile+=2) {
        TrackingCruise a={0},b={0};Circuit c={.phase=C_FOLLOW};ScanSmooth optical={0};
        CircuitInput i={.speed={45,45}};AnalogDemand x={.target={35,50},.decision=25,.level={850,300}};
        AnalogDemand y=x;y.level[0]=500;y.level[1]=950; /* same control/motion history, entirely different brightness */
        for(unsigned k=0;k<300;++k) {
            AnalogDemand u=tracking_cruise_step(&a,&c,&i,profile,&optical,x,.01f);
            AnalogDemand v=tracking_cruise_step(&b,&c,&i,profile,&optical,y,.01f);
            assert(u.target[0]==v.target[0] && u.target[1]==v.target[1]);
            assert(u.target[1]-u.target[0]==15); /* keep real optical steering */
            if(k>220)assert(u.target[1]==(profile==3?135:60));
        }
        assert(a.fast);
        AnalogDemand urgent={.target={0,45},.decision=12,.level={900,0}};
        AnalogDemand got=tracking_cruise_step(&a,&c,&i,profile,&optical,urgent,.01f);
        assert(!memcmp(&urgent,&got,sizeof(got)) && !a.active); /* no smooth heading lock through departure */
    }
}
int main(void) {
    stable_unequal();
    TrackingCruise s={0};Circuit c={.phase=C_FOLLOW};CircuitInput i={.speed={20,60}};ScanSmooth o={0};
    AnalogDemand d={.target={40,50},.decision=25};
    for(unsigned k=0;k<400;++k)(void)tracking_cruise_step(&s,&c,&i,3,&o,d,.01f);
    assert(!s.fast); /* actual encoder turning must not qualify as straight */
    i.speed[0]=i.speed[1]=45;s=(TrackingCruise){0};
    for(unsigned k=0;k<400;++k) {d.target[0]=k%2?20:50;d.target[1]=k%2?50:20;(void)tracking_cruise_step(&s,&c,&i,3,&o,d,.01f);}
    assert(!s.fast); /* steering hunting is not stable */
    d=(AnalogDemand){.target={45,45},.decision=24};o.scanning=true;
    AnalogDemand v=tracking_cruise_step(&s,&c,&i,3,&o,d,.01f);assert(v.target[0]==45 && !s.active);
    o.scanning=false;c.busy=true;d=(AnalogDemand){.target={0,65},.decision=0};
    v=tracking_cruise_step(&s,&c,&i,3,&o,d,.01f);assert(v.target[0]==0 && v.target[1]==65);
    c.busy=false;c.follower.core.white=true;d.stop=true;
    v=tracking_cruise_step(&s,&c,&i,3,&o,d,.01f);assert(v.stop && !s.active);
    printf("PASS brightness-independent cruise qualification, unequal sensor levels, actual-turn/hunting rejection, steering preserved, immediate urgent/search/turn/scan bypass; state=%zu bytes.\n",sizeof(TrackingCruise));
}
