/**
 * @file wheel_control.h
 * @brief Wheel control.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef ANALOG_WHEEL_CONTROL_H
#define ANALOG_WHEEL_CONTROL_H
#include "src/motion/dual_logic.h"

/* V5 provisional forward-only speed PI. Tune locally: historical dual-wheel
 * firmware keeps its original gains. Feedforward and acceleration are
 * unchanged. More integral authority addresses persistent loaded error without
 * raising PWM. Units: output permille, speed mm/s, dt seconds. No derivative or
 * reverse drive. */
#define LINE_SPEED_KP 1.0f
#define LINE_SPEED_KI 3.0f
#define LINE_SPEED_I_LIMIT 140.0f
static inline int32_t line_speed_step (wheel_pi_t * p_data, unsigned wheel,
                                       float wanted, float speed,
                                       float delta_time)
{
    if (wanted <= 0)
    {
        *p_data = (wheel_pi_t){0};
        return 0;
    }
    p_data->ramp += clampf(wanted - p_data->ramp, -100.0f * delta_time,
                           100.0f * delta_time);
    float error = p_data->ramp - speed;
    float ff_value = feedforward(wheel, 1, p_data->ramp);
    float candidate =
        clampf(p_data->integral + LINE_SPEED_KI * error * delta_time,
               -LINE_SPEED_I_LIMIT, LINE_SPEED_I_LIMIT);
    float u_value = ff_value + LINE_SPEED_KP * error + candidate;
    /* Freeze only integration that drives further into saturation. Error in
       the opposite direction must be able to unwind a previously high I. */
    if (!(((u_value > PWM_LIMIT) && (error > 0)) ||
          ((u_value < 0) && (error < 0))))
    {
        p_data->integral = candidate;
    }
    return (int32_t)(clampf(ff_value + LINE_SPEED_KP * error + p_data->integral,
                            0, PWM_LIMIT) +
                     0.5f);
}
#endif

/* End of file. */
