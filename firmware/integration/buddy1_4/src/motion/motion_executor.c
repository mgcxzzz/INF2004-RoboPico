/**
 * @file motion_executor.c
 * @brief Motion executor.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "src/motion/motion_executor.h"
#include <math.h>
#include <string.h>
/* New finite-move bounds only. They do not replace any V12.2 navigation,
 * calibration, speed/PWM, recovery or junction constants. */
enum
{
    EXEC_TIMEOUT = NAV_FAULT_TIMEOUT,
    EXEC_SETTLE_TIMEOUT = NAV_FAULT_SETTLE,
    EXEC_HEADING = NAV_FAULT_HEADING,
    EXEC_DIRECTION = NAV_FAULT_DIRECTION
};
static float bound (float value_x, float low, float high)
{
    return value_x < low ? low : value_x > high ? high : value_x;
}
bool motion_executor_start (motion_executor_t * p_event_value,
                            const motion_feedback_t * p_flags_value, bool turn,
                            float goal, unsigned speed,
                            motion_turn_style_t style)
{
    *p_event_value = (motion_executor_t){
        .active = true,
        .turning = turn,
        .style = style,
        .goal = goal,
        .speed = (float)speed,
        .origin = {p_flags_value->wheel_mm[0], p_flags_value->wheel_mm[1]},
        .since = p_flags_value->now_us};
    double travel =
        turn ? fabs(goal) * 110 * 3.14159265358979323846 / 180 : fabs(goal);
    double seconds = 3 * travel / speed + 3;
    if (seconds > 60)
    {
        seconds = 60;
    }
    p_event_value->deadline =
        p_flags_value->now_us + (uint64_t)(seconds * 1000000);
    return true;
}
motion_command_state_t
motion_executor_step (motion_executor_t * p_event_value,
                      const motion_feedback_t * p_flags_value)
{
    float l_value =
              (float)(p_flags_value->wheel_mm[0] - p_event_value->origin[0]),
          result =
              (float)(p_flags_value->wheel_mm[1] - p_event_value->origin[1]);
    p_event_value->progress_mm = (l_value + result) * .5f;
    p_event_value->rotation_deg =
        (result - l_value) * 180 / (110 * 3.14159265358979323846f);
    if (!p_event_value->active)
    {
        return p_event_value->fault ? MOTION_CMD_FAILED : MOTION_CMD_DONE;
    }
    p_event_value->target[0] = p_event_value->target[1] = 0;
    if (p_flags_value->now_us >= p_event_value->deadline)
    {
        p_event_value->fault = EXEC_TIMEOUT;
    }
    if ((!p_event_value->turning) && (fabsf(p_event_value->rotation_deg) > 12))
    {
        p_event_value->fault = EXEC_HEADING;
    }
    float sign = p_event_value->goal > 0 ? 1 : -1;
    float along = sign * (p_event_value->turning ? p_event_value->rotation_deg
                                                 : p_event_value->progress_mm);
    if (along < -5)
    {
        p_event_value->fault = EXEC_DIRECTION;
    }
    if (p_event_value->fault)
    {
        p_event_value->active = false;
        return MOTION_CMD_FAILED;
    }
    float remaining = fabsf(p_event_value->goal) - along;
    if ((!p_event_value->settling) && (remaining <= 1))
    {
        p_event_value->settling = true;
        p_event_value->settle_since = p_flags_value->now_us;
    }
    if (p_event_value->settling)
    {
        bool still = (fabsf(p_flags_value->speed_mm_s[0]) < 5) &&
                     (fabsf(p_flags_value->speed_mm_s[1]) < 5);
        if (!still)
        {
            p_event_value->still_timing = false;
        }
        else if (!p_event_value->still_timing)
        {
            p_event_value->still_timing = true;
            p_event_value->still_since = p_flags_value->now_us;
        }
        if ((p_event_value->still_timing) &&
            (p_flags_value->now_us - p_event_value->still_since >= 120000))
        {
            p_event_value->active = false;
            return MOTION_CMD_DONE;
        }
        if (p_flags_value->now_us - p_event_value->settle_since >= 2000000)
        {
            p_event_value->fault = EXEC_SETTLE_TIMEOUT;
            p_event_value->active = false;
            return MOTION_CMD_FAILED;
        }
        return MOTION_CMD_RUNNING;
    }
    float speed = p_event_value->speed;
    if (p_event_value->turning)
    {
        if ((remaining < 12) && (speed > 30))
        {
            speed = 30;
        }
        if (MOTION_IN_PLACE == p_event_value->style)
        {
            p_event_value->target[0] = (int16_t)(-sign * speed);
            p_event_value->target[1] = (int16_t)(sign * speed);
        }
        else
        {
            p_event_value->target[sign > 0 ? 1 : 0] = (int16_t)speed;
        }
    }
    else
    {
        if ((remaining < 10) && (speed > 25))
        {
            speed = 25;
        }
        float correction = bound((result - l_value) * 1.2f +
                                     .08f * (p_flags_value->speed_mm_s[1] -
                                             p_flags_value->speed_mm_s[0]),
                                 -8, 8);
        p_event_value->target[0] = (int16_t)lroundf(sign * speed + correction);
        p_event_value->target[1] = (int16_t)lroundf(sign * speed - correction);
    }
    return MOTION_CMD_RUNNING;
}

/* End of file. */
