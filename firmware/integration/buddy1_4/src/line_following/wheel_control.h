#ifndef ANALOG_WHEEL_CONTROL_H
#define ANALOG_WHEEL_CONTROL_H
#include "src/motion/dual_logic.h"

/* V5 provisional forward-only speed PI. Tune locally: historical dual-wheel
 * firmware keeps its original gains. Feedforward and acceleration are unchanged.
 * More integral authority addresses persistent loaded error without raising PWM.
 * Units: output permille, speed mm/s, dt seconds. No derivative or reverse drive. */
#define LINE_SPEED_KP 1.0f
#define LINE_SPEED_KI 3.0f
#define LINE_SPEED_I_LIMIT 140.0f
static inline int32_t line_speed_step(WheelPI *p, unsigned w, float wanted, float speed, float dt) {
    if (wanted <= 0) {
        *p = (WheelPI){0};
        return 0;
    }
    p->ramp += clampf(wanted - p->ramp, -100.0f * dt, 100.0f * dt);
    float error = p->ramp - speed;
    float ff = feedforward(w, 1, p->ramp);
    float candidate =
        clampf(p->integral + LINE_SPEED_KI * error * dt, -LINE_SPEED_I_LIMIT, LINE_SPEED_I_LIMIT);
    float u = ff + LINE_SPEED_KP * error + candidate;
    /* Freeze only integration that drives further into saturation. Error in
       the opposite direction must be able to unwind a previously high I. */
    if (!((u > PWM_LIMIT && error > 0) || (u < 0 && error < 0)))
        p->integral = candidate;
    return (int32_t)(clampf(ff + LINE_SPEED_KP * error + p->integral, 0, PWM_LIMIT) + 0.5f);
}
#endif
