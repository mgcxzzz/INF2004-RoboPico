#ifndef BARCODE_RAMP_PROFILE_H
#define BARCODE_RAMP_PROFILE_H
#include "barcode_nav.h"
#include "../circuit_left_test/circuit_wheel.h"

enum { RAMP2_SPEED=180,RAMP2_SUSTAIN=500,RAMP2_BOOST=650,
       RAMP2_QUALIFY_US=40000,RAMP2_BOOST_US=1500000,RAMP2_RECOVER_US=200000 };
typedef struct {
    uint64_t deficit_since,boost_until,recovered_since;
    float extra;
    bool active,deficit,boost_used,recovered;
} RampWheel;
static inline bool ramp_profile_active(unsigned profile,const Circuit *c,const AnalogDemand *d) {
    return profile==2 && c->phase==C_FOLLOW && !c->busy && !d->stop &&
        !c->follower.core.white && (d->level[0]>=400 || d->level[1]>=400);
}
static inline unsigned barcode_profile_cruise(unsigned profile) {
    return profile==2?RAMP2_SPEED:(unsigned)circuit_cruise(profile);
}
/* Higher effort follows EACH wheel's own target, so an intentionally slow or
 * off inner wheel is never driven toward the outer wheel's speed. Physical
 * PWM units are used here; do not apply the historical20% multiplier again. */
static inline int32_t ramp_wheel_step(RampWheel *s,SignedWheel *p,unsigned w,int16_t wanted,
                                     float speed,float dt,uint64_t now) {
    if(wanted<=0) { *s=(RampWheel){0};*p=(SignedWheel){0};return 0; }
    if(!s->active || p->sign!=1) {
        *s=(RampWheel){.active=true};
        p->pi.integral=0;p->sign=1;p->was_turning=false;
        p->peak_used=p->peak_active=false;p->peak_end_us=0;p->kick=(LineBreakaway){0};
    }
    dt=clampf(dt,0,.05f);
    bool falling=p->pi.ramp>wanted;
    if(falling) p->pi.ramp=wanted;
    else p->pi.ramp+=fminf(wanted-p->pi.ramp,400*dt);
    float error=p->pi.ramp-speed;
    bool loaded=!falling && wanted>=20 && p->pi.ramp>=20 && speed>=-8 &&
        error>fmaxf(12,.30f*p->pi.ramp);
    if(!loaded) s->deficit=false;
    else {
        if(!s->deficit) { s->deficit=true;s->deficit_since=now; }
        if(now-s->deficit_since>=RAMP2_QUALIFY_US && !s->boost_used) {
            s->boost_used=true;s->boost_until=now+RAMP2_BOOST_US;
        }
    }
    /* Persistent stalls cannot continually renew the high-power deadline.
     * Re-arm only after200ms genuinely close to the requested wheel speed. */
    bool recovered=!falling && p->pi.ramp>=20 && speed>=.85f*p->pi.ramp &&
        fabsf(error)<=fmaxf(8,.15f*p->pi.ramp);
    if(!recovered) s->recovered=false;
    else {
        if(!s->recovered) {s->recovered=true;s->recovered_since=now;}
        if(now-s->recovered_since>=RAMP2_RECOVER_US) {s->boost_used=false;s->boost_until=0;}
    }
    bool boost=loaded && s->boost_used && now<s->boost_until;
    if(boost) s->extra=fminf(220,s->extra+2000*dt);
    else s->extra=fmaxf(0,s->extra-1600*dt);
    if(falling || error<=0) {s->extra=0;if(p->pi.integral>0)p->pi.integral=0;}
    /* Coast immediately when a wheel is already too fast for its target.
     * No reverse braking and no positive boost through a slowdown command. */
    if(speed>wanted+10 || error < -10) {p->pi.integral=0;s->extra=0;return 0;}
    float ff=(float)effort_scale20((int32_t)(feedforward(w,1,p->pi.ramp)+.5f));
    float limit=boost?RAMP2_BOOST:RAMP2_SUSTAIN;
    float candidate=clampf(p->pi.integral+10*error*dt,-180,180);
    float proposed=ff+3.5f*error+candidate+s->extra;
    if(!((proposed>limit && error>0)||(proposed<0 && error<0))) p->pi.integral=candidate;
    return (int32_t)(clampf(ff+3.5f*error+p->pi.integral+s->extra,0,limit)+.5f);
}
static inline void ramp_wheel_leave(RampWheel *s,SignedWheel *p,int16_t wanted) {
    if(s->active) {
        *s=(RampWheel){0};p->pi.integral=0;p->kick=(LineBreakaway){0};
        if(p->pi.ramp>wanted) p->pi.ramp=fmaxf(wanted,0);
    }
}
/* Integer-only output envelope shared by the control commit and1ms ISR. */
static inline int32_t barcode_output_cap(int32_t pwm,bool ramp,uint64_t until,uint64_t now) {
    if(!ramp) return boosted_junction_peak_cap(pwm,until,now);
    int limit=now<until?RAMP2_BOOST:RAMP2_SUSTAIN;
    return pwm<0?0:pwm>limit?limit:pwm;
}
static inline DualLevels barcode_output_levels(unsigned w,int32_t pwm,bool ramp,bool turning,unsigned profile) {
    if(!ramp) return circuit_levels(w,pwm,turning,profile);
    DualLevels v={0};if(w>1)return v;
    if(pwm<0)pwm=0;
    if(pwm>RAMP2_BOOST)pwm=RAMP2_BOOST;
    if(w==0)v.a=(uint16_t)pwm;else v.b=(uint16_t)pwm;return v;
}
#endif
