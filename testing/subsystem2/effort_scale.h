#ifndef BUDDY23_EFFORT_SCALE_H
#define BUDDY23_EFFORT_SCALE_H
#include <stdint.h>
/* Experimental 20% RELATIVE output increase, rounded to nearest permille.
 * Controllers retain their original effort units and anti-windup limits;
 * adapters and logs use physical PWM. Zero and direction are preserved. */
static inline int32_t effort_scale20(int32_t pwm) {
    if(pwm>999) pwm=999;
    if(pwm < -999) pwm=-999;
    int32_t magnitude=pwm<0?-pwm:pwm;
    magnitude=(magnitude*6+2)/5;
    if(magnitude>999) magnitude=999;
    return pwm<0?-magnitude:magnitude;
}
#endif
