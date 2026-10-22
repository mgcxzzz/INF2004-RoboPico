#ifndef HUMP_CONTROL_H
#define HUMP_CONTROL_H
#include "../analog_line_follow/wheel_control.h"
#include "../analog_line_follow/symmetric_logic.h"
#include "../effort_scale.h"

enum { HUMP_BASELINE,HUMP_ASSIST,HUMP_PWM_MAX=350,HUMP_EXTRA_MAX=150,
       HUMP_QUALIFY_US=100000,HUMP_RUN_US=10000000,HUMP_RUN_MM=500,HUMP_TRACE_US=125000 };
typedef struct {
    WheelPI pi;
    LineBreakaway kick;
    uint64_t deficit_since;
    float extra;
    bool qualifying;
} HumpWheel;
static inline unsigned hump_base_ceiling(unsigned profile) { return profile==HUMP_ASSIST?HUMP_PWM_MAX:PWM_LIMIT; }
static inline unsigned hump_ceiling(unsigned profile) { return (unsigned)effort_scale20((int32_t)hump_base_ceiling(profile)); }
/* Extra effort is a bounded response to commanded-wheel underspeed, not a
 * detector of inclines. No known ramp height/length, IMU or faster speed target.
 * Assistance requires a line-confidence grant, or V4's explicitly bounded
 * straight-hump carry grant. Ordinary optical search has no such grant.
 * All effort is scaled1.20x. Overspeed reduces/coasts drive. */
static inline int32_t hump_unscaled_wheel_step(HumpWheel *p,unsigned w,int16_t wanted,float speed,
                                      float dt,uint64_t now,unsigned profile,bool load_allowed) {
    if(wanted<=0) { *p=(HumpWheel){0};return 0; }
    if(profile!=HUMP_ASSIST) {
        p->extra=0;p->qualifying=false;
        int32_t u=line_speed_step(&p->pi,w,wanted,speed,dt);
        return line_breakaway(&p->kick,now,wanted,p->pi.ramp,speed,u);
    }
    p->pi.ramp+=clampf(wanted-p->pi.ramp,-100.0f*dt,100.0f*dt);
    float error=p->pi.ramp-speed;
    /* A requested acceleration/deceleration is not itself evidence of load.
       Wait until the existing speed ramp has reached the current request. */
    /* Include slow steering requests down to the existing blocked-wheel guard's
       20 mm/s floor. A smaller request keeps ordinary effort; zero stays off. */
    bool eligible=load_allowed && wanted>=20 && fabsf(p->pi.ramp-wanted)<0.5f;
    if(!eligible || error<=-3) { p->extra=0;p->qualifying=false; }
    else if(error>=15) {
        if(!p->qualifying) { p->qualifying=true;p->deficit_since=now; }
        if(now-p->deficit_since>=HUMP_QUALIFY_US) p->extra=fminf(HUMP_EXTRA_MAX,p->extra+750.0f*dt);
    } else {
        p->qualifying=false;
        if(error<=5) p->extra=fmaxf(0,p->extra-500.0f*dt);
        if(p->extra<0.001f) p->extra=0;
    }
    float limit=p->extra>0?HUMP_PWM_MAX:PWM_LIMIT;
    float ff=feedforward(w,1,p->pi.ramp);
    float candidate=clampf(p->pi.integral+LINE_SPEED_KI*error*dt,-LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
    float proposed=ff+LINE_SPEED_KP*error+candidate+p->extra;
    if(!((proposed>limit && error>0) || (proposed<0 && error<0))) p->pi.integral=candidate;
    int32_t out=(int32_t)(clampf(ff+LINE_SPEED_KP*error+p->pi.integral+p->extra,0,limit)+0.5f);
    int32_t kicked=line_breakaway(&p->kick,now,wanted,p->pi.ramp,speed,out>PWM_LIMIT?PWM_LIMIT:out);
    if(kicked>out) out=kicked;
    return out;
}
static inline int32_t hump_wheel_step(HumpWheel *p,unsigned w,int16_t wanted,float speed,
                                      float dt,uint64_t now,unsigned profile,bool visible) {
    return effort_scale20(hump_unscaled_wheel_step(p,w,wanted,speed,dt,now,profile,visible));
}
/* Shared dual_levels clips25%; map physical30%/42% ceilings explicitly.
 * Negative values cannot command reverse in this forward-only experiment. */
static inline DualLevels hump_levels(unsigned w,int32_t pwm,unsigned profile) {
    DualLevels v={0};if(w>1) return v;
    if(pwm<0) pwm=0;
    if((unsigned)pwm>hump_ceiling(profile)) pwm=(int32_t)hump_ceiling(profile);
    if(w==0) v.a=(uint16_t)pwm;else v.b=(uint16_t)pwm;
    return v;
}
#endif
