#ifndef BUDDY_TRACKING_CRUISE_H
#define BUDDY_TRACKING_CRUISE_H
#include "v12_2/scan_smooth.h"
/* Speed qualification uses control/motion history, never ADC intensity or
 * equality of the two front readings. The calibrated legacy optical tracker
 * still owns line visibility, steering direction, urgent bends and recovery. */
typedef struct {
    float mean,steer,chatter,turn;
    unsigned good_us:19,bad_us:17,active:1,fast:1;
} TrackingCruise;
static inline AnalogDemand tracking_cruise_step(TrackingCruise *s,const Circuit *c,
    const CircuitInput *i,unsigned profile,const ScanSmooth *optical,AnalogDemand d,float dt) {
    if(profile==2 || c->busy || c->phase!=C_FOLLOW || d.stop || c->follower.core.white ||
       (d.decision!=24 && d.decision!=25)) {*s=(TrackingCruise){0};return d;}
    if(optical->scanning) {*s=(TrackingCruise){0};return d;} /* original45mm/s scan behavior */
    dt=clampf(dt,0,.05f);unsigned us=(unsigned)(dt*1000000+.5f);
    float sum=(float)d.target[0]+d.target[1];
    float steer=sum>1?(d.target[1]-d.target[0])/sum:0;
    float speed_sum=fabsf(i->speed[0])+fabsf(i->speed[1]);
    float turn=speed_sum>1?(i->speed[1]-i->speed[0])/speed_sum:0;
    if(!s->active) {
        *s=(TrackingCruise){.active=true,.mean=fminf(sum*.5f,45),.steer=steer,.turn=turn};
    }
    float alpha=dt/(.18f+dt);
    s->steer+=alpha*(steer-s->steer);s->chatter+=alpha*(fabsf(steer-s->steer)-s->chatter);
    s->turn+=alpha*(turn-s->turn);
    bool moving=i->speed[0]>0 && i->speed[1]>0;
    /* Relative steering/turn tolerances, not colour thresholds. Hysteresis
     * avoids toggling cruise with individual sparse encoder observations. */
    bool settled=moving && fabsf(s->steer)<.30f && s->chatter<.055f && fabsf(s->turn)<.12f;
    bool departing=!moving || fabsf(steer)>.40f || s->chatter>.10f || fabsf(s->turn)>.22f;
    if(settled) {unsigned t=s->good_us+us;s->good_us=t>400000?400000:t;s->bad_us=0;}
    else {s->good_us=0;if(departing) {unsigned t=s->bad_us+us;s->bad_us=t>120000?120000:t;}else s->bad_us=0;}
    if(s->good_us>=400000)s->fast=true;
    if(s->bad_us>=120000)s->fast=false;
    float half=(d.target[1]-d.target[0])*.5f;
    float ceiling=s->fast?(profile==3?135:60):50;
    float desired=s->fast?ceiling-fabsf(half):fminf(sum*.5f,45);
    s->mean+=clampf(desired-s->mean,-240*dt,80*dt);
    s->mean=fminf(s->mean,(profile==3?135:60)-fabsf(half));
    d.target[0]=(int16_t)lroundf(fmaxf(0,s->mean-half));
    d.target[1]=(int16_t)lroundf(fmaxf(0,s->mean+half));
    return d;
}
#endif
