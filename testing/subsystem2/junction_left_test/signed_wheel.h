#ifndef JUNCTION_SIGNED_WHEEL_H
#define JUNCTION_SIGNED_WHEEL_H
#include "../analog_line_follow/wheel_control.h"
#include "../analog_line_follow/line_logic.h"
typedef struct { WheelPI pi;LineBreakaway kick;int8_t sign; } SignedWheel;
/* Same V11 gains/ramp/cap/boost, with signed feedback and direction-specific
 * feedforward. Caller must settle before reversing; zero clears PI and boost. */
static inline int32_t junction_wheel_step(SignedWheel *p,unsigned w,int16_t wanted,
                                         float speed,float dt,uint64_t now) {
    int sign=wanted>0?1:wanted<0?-1:0;
    if(sign!=p->sign) { *p=(SignedWheel){.sign=(int8_t)sign}; }
    if(!sign) { *p=(SignedWheel){0};return 0; }
    float magnitude=(float)(wanted*sign),along=speed*sign;
    p->pi.ramp+=clampf(magnitude-p->pi.ramp,-100.0f*dt,100.0f*dt);
    float error=p->pi.ramp-along,ff=feedforward(w,sign,p->pi.ramp);
    float candidate=clampf(p->pi.integral+LINE_SPEED_KI*error*dt,-LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
    float u=ff+LINE_SPEED_KP*error+candidate;
    if(!((u>PWM_LIMIT && error>0) || (u<0 && error<0))) p->pi.integral=candidate;
    int32_t out=(int32_t)(clampf(ff+LINE_SPEED_KP*error+p->pi.integral,0,PWM_LIMIT)+0.5f);
    return sign*line_breakaway(&p->kick,now,(int16_t)magnitude,p->pi.ramp,along,out);
}
#endif
