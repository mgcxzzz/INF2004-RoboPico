#ifndef BARCODE_VISIBLE_WHEEL_H
#define BARCODE_VISIBLE_WHEEL_H
#include "smooth_follow.h"

typedef struct { float output,extra;uint64_t deficit_since;bool active,deficit; } VisibleWheel;
/* Physical PWM units (0..1000). No repeated max-PWM breakaway pulses in
 * ordinary visible-line following. Separate U/search/ramp/branch controllers
 * retain their previous limits and behaviour. */
static inline bool visible_wheel_eligible(const Circuit *c,const AnalogDemand *d,bool capture_window) {
    return c->phase==C_FOLLOW && !c->busy && !d->stop && !capture_window &&
        !c->follower.core.white && (d->level[0]>=200 || d->level[1]>=200) &&
        (d->decision==SYM_STRAIGHT || d->decision==SYM_LEFT ||
         d->decision==SYM_RIGHT || d->decision==C_FAST);
}
static inline void visible_wheel_leave(VisibleWheel *s,SignedWheel *p) {
    if(!s->active)return;
    *s=(VisibleWheel){0};p->pi.integral=0;p->kick=(LineBreakaway){0};
    p->peak_active=p->peak_used=p->was_turning=false;p->peak_end_us=0;
}
static inline int32_t visible_wheel_step(VisibleWheel *s,SignedWheel *p,unsigned w,
                                        int16_t wanted,float speed,float dt,uint64_t now,
                                        bool loaded,unsigned profile,int32_t previous_output) {
    if(wanted<=0) { *s=(VisibleWheel){0};*p=(SignedWheel){0};return 0; }
    dt=clampf(dt,0,.05f);
    if(!s->active) {
        *s=(VisibleWheel){.active=true,.output=(float)(previous_output>0?previous_output:0)};
        p->pi.integral=0;p->sign=1;p->kick=(LineBreakaway){0};
        p->peak_active=p->peak_used=p->was_turning=false;p->peak_end_us=0;
    }
    float limit=loaded?420:(float)circuit_forward_cap(profile);
    float before=p->pi.ramp;
    p->pi.ramp+=clampf(wanted-p->pi.ramp,-300*dt,100*dt);
    float error=p->pi.ramp-speed;
    bool falling=p->pi.ramp<before;
    bool deficit=!falling && p->pi.ramp>=20 && speed>=-8 &&
        speed<p->pi.ramp-fmaxf(8,.25f*p->pi.ramp);
    if(!deficit)s->deficit=false;
    else if(!s->deficit) {s->deficit=true;s->deficit_since=now;}
    if(deficit && now-s->deficit_since>=80000)s->extra=fminf(90,s->extra+180*dt);
    else if(error<=3)s->extra=fmaxf(0,s->extra-50*dt);
    /* Shed load compensation when the requested correction slows this wheel.
     * Bounded PWM fall prevents the old jump from maximum kick to low output. */
    if(falling || speed>wanted+15) {s->extra=0;if(p->pi.integral>0)p->pi.integral=fmaxf(0,p->pi.integral-600*dt);}
    float ff=1.2f*feedforward(w,1,p->pi.ramp);
    float candidate=clampf(p->pi.integral+3.6f*error*dt,-168,168);
    float low=fminf(limit,fmaxf(0,s->output-1200*dt)),high=fminf(limit,s->output+600*dt);
    float proposed=ff+1.2f*error+candidate+s->extra;
    /* Include slew saturation in anti-windup: a delayed output must not build
     * a large integral that fires as soon as the output catches up. */
    if(!((proposed>high && error>0) || (proposed<low && error<0)))p->pi.integral=candidate;
    float desired=clampf(ff+1.2f*error+p->pi.integral+s->extra,0,limit);
    if(speed < -8) {s->output=0;s->extra=0;p->pi.integral=0;return 0;}
    /* A low target must not keep pushing a fast inner wheel. Zero/coast and
     * all stop paths remain immediate; only positive drive is slew limited. */
    if(wanted<=15 && speed>wanted+8)desired=0;
    s->output=clampf(clampf(desired,low,high),0,limit);
    return (int32_t)(s->output+.5f);
}
#endif
