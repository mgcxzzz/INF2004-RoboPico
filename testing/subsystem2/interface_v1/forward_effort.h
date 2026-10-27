#ifndef BUDDY_FORWARD_EFFORT_H
#define BUDDY_FORWARD_EFFORT_H
#include "v12_2/baseline/testing/subsystem2/circuit_left_test/circuit_wheel.h"
/* V1.3 experiment: one PI for ordinary forward tracking AND its qualified
 * outer-wheel assistance. Assistance opens the existing ceiling; it no longer
 * installs pivot feedforward or clears the integral. No second load integrator.
 * Planned turns/capture, search, external commands and ramp keep their driver. */
typedef struct {
    float duty;
    unsigned deficit_us:16, pulse_us:17, previous_target:8;
    unsigned active:1, start_used:1, pulse:1;
} ForwardEffort;
enum { FORWARD_RISE_PWM_S=1800, FORWARD_START_QUALIFY_US=60000,
       FORWARD_START_US=100000, FORWARD_START_PWM=300 };
static inline int32_t forward_effort_step(ForwardEffort *s,SignedWheel *p,unsigned w,
    int16_t wanted,float speed,float dt,bool assisted,unsigned profile,int32_t previous) {
    if(wanted<=0) {*s=(ForwardEffort){0};*p=(SignedWheel){0};return 0;}
    dt=clampf(dt,0,.05f);
    unsigned us=(unsigned)(dt*1e6f+.5f);
    float cap=assisted?420:(float)circuit_forward_cap(profile);
    if(!s->active) {
        *s=(ForwardEffort){.active=true,.duty=clampf(previous,0,cap),.previous_target=(unsigned)wanted};
        /* Match ordinary PI to actual preceding effort on entry. Never seed
         * the actuator alone with loaded duty while leaving PI at zero. */
        if(p->sign!=1)*p=(SignedWheel){.sign=1};
        p->pi.ramp=fminf(p->pi.ramp,(float)wanted);
        p->pi.integral=previous>0?clampf(s->duty/1.2f-feedforward(w,1,p->pi.ramp)-
            LINE_SPEED_KP*(p->pi.ramp-speed),-LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT):0;
        p->was_turning=p->peak_active=p->peak_used=false;p->peak_end_us=0;p->kick=(LineBreakaway){0};
    }
    p->pi.ramp+=clampf(wanted-p->pi.ramp,-400*dt,100*dt);
    float ff=feedforward(w,1,p->pi.ramp),error=p->pi.ramp-speed;
    bool falling=wanted<(int)s->previous_target;
    if(falling && speed>wanted+5 && p->pi.integral>0)p->pi.integral=0;
    /* Deliberate slow/off and overspeed have immediate negative authority.
     * Never preserve a high duty via an output down-slew. */
    bool overspeed=speed>fminf((float)wanted,p->pi.ramp)+8;
    bool start_eligible=!s->start_used && !falling && wanted>=20 && p->pi.ramp>=20 && speed>=-8 && speed<8;
    if(start_eligible) {
        unsigned n=s->deficit_us+us;s->deficit_us=n>FORWARD_START_QUALIFY_US?FORWARD_START_QUALIFY_US:n;
        if(s->deficit_us>=FORWARD_START_QUALIFY_US) {s->start_used=true;s->pulse=true;s->pulse_us=0;}
    } else s->deficit_us=0;
    if(s->pulse) {
        unsigned n=s->pulse_us+us;s->pulse_us=n>FORWARD_START_US?FORWARD_START_US:n;
        if(s->pulse_us>=FORWARD_START_US || speed>=20 || wanted<20 || falling || overspeed) {
            s->pulse=false;
            /* Feed starting effort into the same bounded PI state, rather than
             * dropping immediately to an unrelated low running estimate. */
            p->pi.integral=clampf(s->duty/1.2f-ff-LINE_SPEED_KP*error,
                -LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
        }
    }
    if(overspeed && p->pi.integral>0)p->pi.integral=0;
    float candidate=clampf(p->pi.integral+LINE_SPEED_KI*error*dt,-LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
    float proposed=1.2f*(ff+LINE_SPEED_KP*error+candidate);
    /* Do not accumulate positive I against the actuator ceiling/rise limit. */
    if(!s->pulse && !((proposed>fminf(cap,s->duty+FORWARD_RISE_PWM_S*dt) && error>0) ||
        (proposed<0 && error<0)))p->pi.integral=candidate;
    float desired=clampf(1.2f*(ff+LINE_SPEED_KP*error+p->pi.integral),0,cap);
    if(s->pulse && !overspeed)desired=fmaxf(desired,FORWARD_START_PWM);
    if(speed>wanted+20 || (wanted<=15 && speed>wanted+5))desired=0;
    s->duty=fminf(desired,s->duty+FORWARD_RISE_PWM_S*dt);
    s->duty=clampf(s->duty,0,cap);s->previous_target=(unsigned)wanted;
    return (int32_t)(s->duty+.5f);
}
#endif
