#ifndef JUNCTION_SIGNED_WHEEL_H
#define JUNCTION_SIGNED_WHEEL_H
#include "../analog_line_follow/wheel_control.h"
#include "../analog_line_follow/line_logic.h"
typedef struct { WheelPI pi;LineBreakaway kick;int8_t sign; } SignedWheel;
enum { J_TURN_FEEDFORWARD=270,J_TURN_PWM_LIMIT=300 };
_Static_assert(J_TURN_FEEDFORWARD<=J_TURN_PWM_LIMIT && J_TURN_PWM_LIMIT<=999,
               "Invalid turn effort limits");
/* V3: in-place turn only uses27% loaded feedforward and30% maximum.
 * Forward phases retain V11's25% ceiling. This is a provisional load estimate,
 * NOT a forced minimum output: signed PI can reduce effort for overspeed.
 * Caller must settle before reversing; zero clears PI and boost. */
static inline int32_t junction_wheel_step(SignedWheel *p,unsigned w,int16_t wanted,
                                         float speed,float dt,uint64_t now,bool turning) {
    int sign=wanted>0?1:wanted<0?-1:0;
    if(sign!=p->sign) { *p=(SignedWheel){.sign=(int8_t)sign}; }
    if(!sign) { *p=(SignedWheel){0};return 0; }
    float magnitude=(float)(wanted*sign),along=speed*sign;
    p->pi.ramp+=clampf(magnitude-p->pi.ramp,-100.0f*dt,100.0f*dt);
    float error=p->pi.ramp-along,ff=feedforward(w,sign,p->pi.ramp);
    if(turning) ff=fmaxf(ff,J_TURN_FEEDFORWARD);
    int limit=turning?J_TURN_PWM_LIMIT:PWM_LIMIT;
    float candidate=clampf(p->pi.integral+LINE_SPEED_KI*error*dt,-LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
    float u=ff+LINE_SPEED_KP*error+candidate;
    if(!((u>limit && error>0) || (u<0 && error<0))) p->pi.integral=candidate;
    int32_t out=(int32_t)(clampf(ff+LINE_SPEED_KP*error+p->pi.integral,0,(float)limit)+0.5f);
    /* The shared breakaway helper clamps to25%. Do not let it silently clip
       the new turn PI; still use its unchanged bounded pulse for lower output. */
    int32_t boosted=line_breakaway(&p->kick,now,(int16_t)magnitude,p->pi.ramp,along,
                                   out>PWM_LIMIT?PWM_LIMIT:out);
    if(!turning || boosted>out) out=boosted;
    return sign*out;
}
/* Shared dual_levels also clamps at25%. Only the explicitly gated opposite-
 * wheel turn may use30%; every other phase retains its exact bridge mapping. */
static inline DualLevels junction_levels(unsigned w,int32_t pwm,bool turning) {
    if(!turning) return dual_levels(w,pwm);
    DualLevels v={0};if(w>1) return v;
    if(pwm>J_TURN_PWM_LIMIT) pwm=J_TURN_PWM_LIMIT;
    if(pwm < -J_TURN_PWM_LIMIT) pwm=-J_TURN_PWM_LIMIT;
    uint16_t magnitude=(uint16_t)(pwm<0?-pwm:pwm);
    bool a_side=w==0?pwm>0:pwm<0;
    if(a_side) v.a=magnitude;else v.b=magnitude;
    return v;
}
#endif
