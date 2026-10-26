#ifndef BARCODE_SMOOTH_FOLLOW_H
#define BARCODE_SMOOTH_FOLLOW_H
#include "barcode_nav.h"
#include "ramp_profile.h"

typedef struct { float target[2],paired_seconds;bool active,paired; } SmoothFollow;
enum { B_STRAIGHT_HOLD=24 };
/* This adapter changes only visible-line output targets. Raw calibrated
 * levels still own loss detection, recovery memory and junction acceptance.
 * No extra optical delay is added to the shared follower or barcode reader. */
static inline void smooth_follow_apply(SmoothFollow *s,const Circuit *c,float dt,AnalogDemand *d) {
    bool following=c->phase==C_FOLLOW;
    const CircuitTracker *t=&c->follower;
    int error=(int)d->level[0]-d->level[1],magnitude=abs(error);
    unsigned strongest=d->level[0]>d->level[1]?d->level[0]:d->level[1];
    /* A clear departure or strong near-pivot demand must retain its original bend
     * authority immediately. The same is true of every blind search/stop. */
    bool near_pivot=d->target[0]<=10 || d->target[1]<=10;
    bool eligible=following && !c->busy && !d->stop && !t->core.white &&
        strongest>=200 && magnitude<700 && !(near_pivot && strongest>=400) &&
        (d->decision==SYM_STRAIGHT || d->decision==SYM_LEFT ||
         d->decision==SYM_RIGHT || d->decision==C_FAST);
    if(!eligible) { *s=(SmoothFollow){0};return; }
    dt=clampf(dt,0,.05f);
    /* V15: two clearly black spots need not have identical reflectance on
     * worn tape. Qualify a small-error plateau for100ms, then hold equal
     * cruise targets. Exit without delay on an edge or larger imbalance.
     * Raw optical levels, loss detection and strong bend bypass stay V14. */
    unsigned weakest=d->level[0]<d->level[1]?d->level[0]:d->level[1];
    if(s->paired && (weakest<600 || magnitude>300))s->paired=false;
    bool qualify=weakest>=700 && magnitude<=220;
    if(!s->paired) {
        s->paired_seconds=qualify?s->paired_seconds+dt:0;
        if(s->paired_seconds>=.100f)s->paired=true;
    }
    float desired[2]={(float)d->target[0],(float)d->target[1]};
    /* Reduce only small, well-visible balance corrections, blending back to
     * the original gain by350/1000 error. Stronger bends keep the old gain. */
    if(d->level[0]>=300 && d->level[1]>=300 && magnitude<350) {
        float mean=(desired[0]+desired[1])*.5f;
        float half=(desired[1]-desired[0])*.5f*(.65f+.35f*magnitude/350.0f);
        desired[0]=mean-half;desired[1]=mean+half;
    }
    if(s->paired && d->decision!=C_FAST)desired[0]=desired[1]=45;
    if(!s->active) {
        s->active=true;s->target[0]=desired[0];s->target[1]=desired[1];
    } else {
        /* Leaving90/135 cruising must not carry a high-speed target into a
         * correction. Preserve the old correction ceiling immediately. */
        if((s->target[0]>55 || s->target[1]>55) && desired[0]<=50 && desired[1]<=50)
            for(unsigned w=0;w<2;++w) s->target[w]=fminf(s->target[w],50);
        float urgency=clampf((magnitude-250)/450.0f,0,1);
        float tau=.060f-.040f*urgency,rate=200+400*urgency;
        float alpha=dt/(tau+dt);
        for(unsigned w=0;w<2;++w) {
            float change=(desired[w]-s->target[w])*alpha;
            s->target[w]+=clampf(change,-rate*dt,rate*dt);
        }
    }
    for(unsigned w=0;w<2;++w) d->target[w]=(int16_t)(s->target[w]+.5f);
    d->decision=d->target[0]<d->target[1]?SYM_LEFT:
        d->target[0]>d->target[1]?SYM_RIGHT:c->fast?C_FAST:s->paired?B_STRAIGHT_HOLD:SYM_STRAIGHT;
}
static inline AnalogDemand barcode_drive_step(Circuit *c,BarcodeNav *n,const LineCalibration *cal,
                                              const CircuitInput *i,unsigned profile,SmoothFollow *s,float dt) {
    AnalogDemand d=barcode_circuit_step(c,n,cal,i,profile);
    if(profile==2 && c->fast && !d.stop)
        d.target[0]=d.target[1]=RAMP2_SPEED;
#ifdef CIRCUIT_COVERAGE_FOLLOW
    /* Coverage logic owns the optical target state in follow AND outgoing
     * alignment. Retire the V15 brightness-balancing/paired-PWM adapter. */
    (void)dt;*s=(SmoothFollow){0};
#else
    smooth_follow_apply(s,c,dt,&d);
#endif
    return d;
}
#endif
