#ifndef BARCODE_STRAIGHT_PWM_H
#define BARCODE_STRAIGHT_PWM_H
#include "smooth_follow.h"
/* This is an output adapter, not a replacement motor controller. It is
 * intentionally unavailable for startup, underspeed, edge corrections,
 * loaded capture, ramp, turning or search. All those retain V14 authority. */
static inline bool straight_pwm_eligible(const Circuit *c,const SmoothFollow *s,
                                         const AnalogDemand *d,const float speed[2],bool special) {
    return !special && c->phase==C_FOLLOW && !c->busy && !c->follower.core.white &&
        s->paired && !d->stop && speed[0]>=25 && speed[1]>=25 &&
        d->target[0]>=35 && d->target[0]<=55 && d->target[1]>=35 && d->target[1]<=55;
}
static inline int32_t straight_pwm_limit(int32_t previous,int32_t requested,float dt,bool eligible) {
    if(!eligible || previous<=0 || requested<=0)return requested;
    dt=clampf(dt,0,.05f);
    float change=clampf((float)(requested-previous),-1600*dt,800*dt);
    return (int32_t)(previous+change+.5f);
}
#endif
