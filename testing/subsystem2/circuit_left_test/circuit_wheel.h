#ifndef CIRCUIT_WHEEL_H
#define CIRCUIT_WHEEL_H
#include "../junction_left_test/boosted_wheel.h"
static inline unsigned circuit_forward_cap(unsigned profile) { return profile==3?420:profile==2?360:300; }
/* Carry normal turn feedforward through a loaded capture correction, but do
   not grant a fresh48% start pulse. Only the selected OUTER wheel calls this. */
static inline void circuit_capture_prepare(SignedWheel *p) {
    if(p->sign!=1) *p=(SignedWheel){.sign=1};
    p->pi.integral=0;p->peak_used=true;p->peak_active=false;p->peak_end_us=0;
}
/* Remove extra cruise compensation before using the normal correction PI.
   Measured speed still controls output; this cannot stop inertia instantly. */
static inline void circuit_leave_fast(SignedWheel *p) {
    p->pi.integral=0;
    if(p->pi.ramp>50) p->pi.ramp=50;
}
static inline int32_t circuit_wheel_step(SignedWheel *p,unsigned w,int16_t wanted,float speed,
                                        float dt,uint64_t now,bool turning,unsigned profile) {
    if(turning) return boosted_junction_step(p,w,wanted,speed,dt,now,true);
    if(profile==1) {
        /* Shared historical controller remains unchanged. In this circuit,
           a falling correction target needs400 rather than100mm/s^2 ramp
           deceleration. Remove retained positive I while actually overspeed. */
        if(wanted>0 && p->sign==1 && p->pi.ramp>wanted) {
            p->pi.ramp=fmaxf((float)wanted,p->pi.ramp-300.0f*dt);
            if(speed>wanted+5 && p->pi.integral>0) p->pi.integral=0;
        }
        int32_t out=boosted_junction_step(p,w,wanted,speed,dt,now,false);
        /* A low-speed inner wheel must coast instead of continuing to push
           through a correction while still faster than its new target. */
        if(wanted>0 && wanted<=15 && speed>wanted+5) {
            p->pi.integral=0;return 0;
        }
        return out;
    }
    if(wanted<=0) { *p=(SignedWheel){0};return 0; }
    if(p->sign!=1) *p=(SignedWheel){.sign=1};
    if(p->was_turning) p->pi.integral=0;
    p->was_turning=p->peak_used=p->peak_active=false;p->peak_end_us=0;
    /* Faster profiles decelerate the requested speed at200mm/s^2. Actual
       motion is measured; the approach-speed guard must still pass. */
    p->pi.ramp+=clampf(wanted-p->pi.ramp,-200.0f*dt,100.0f*dt);
    float error=p->pi.ramp-speed,ff=feedforward(w,1,p->pi.ramp);
    float limit=profile==3?350:300;
    float candidate=clampf(p->pi.integral+LINE_SPEED_KI*error*dt,-LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
    float proposed=ff+LINE_SPEED_KP*error+candidate;
    if(!((proposed>limit && error>0)||(proposed<0 && error<0))) p->pi.integral=candidate;
    int32_t out=(int32_t)(clampf(ff+LINE_SPEED_KP*error+p->pi.integral,0,limit)+0.5f);
    int32_t kicked=line_breakaway(&p->kick,now,wanted,p->pi.ramp,speed,out>PWM_LIMIT?PWM_LIMIT:out);
    if(kicked>out) out=kicked;
    return effort_scale20(out);
}
static inline DualLevels circuit_levels(unsigned w,int32_t pwm,bool turning,unsigned profile) {
    DualLevels v={0};if(w>1) return v;
    int limit=turning?J_BOOST_PEAK:(int)circuit_forward_cap(profile);
    if(pwm>limit) pwm=limit;
    if(pwm<0) pwm=0; /* circuit is forward-only, including rolling turns */
    if(w==0) v.a=(uint16_t)pwm;else v.b=(uint16_t)pwm;
    return v;
}
#endif
