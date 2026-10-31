#include "src/motion/motion_executor.h"
#include <math.h>
#include <string.h>
/* New finite-move bounds only. They do not replace any V12.2 navigation,
 * calibration, speed/PWM, recovery or junction constants. */
enum {
    EXEC_TIMEOUT = NAV_FAULT_TIMEOUT,
    EXEC_SETTLE_TIMEOUT = NAV_FAULT_SETTLE,
    EXEC_HEADING = NAV_FAULT_HEADING,
    EXEC_DIRECTION = NAV_FAULT_DIRECTION
};
static float bound(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}
bool motion_executor_start(MotionExecutor *e, const MotionFeedback *f, bool turn, float goal,
                           unsigned speed, MotionTurnStyle style) {
    *e = (MotionExecutor){.active = true,
                          .turning = turn,
                          .style = style,
                          .goal = goal,
                          .speed = (float)speed,
                          .origin = {f->wheel_mm[0], f->wheel_mm[1]},
                          .since = f->now_us};
    double travel = turn ? fabs(goal) * 110 * 3.14159265358979323846 / 180 : fabs(goal);
    double seconds = 3 * travel / speed + 3;
    if (seconds > 60)
        seconds = 60;
    e->deadline = f->now_us + (uint64_t)(seconds * 1000000);
    return true;
}
MotionCommandState motion_executor_step(MotionExecutor *e, const MotionFeedback *f) {
    float l = (float)(f->wheel_mm[0] - e->origin[0]), r = (float)(f->wheel_mm[1] - e->origin[1]);
    e->progress_mm = (l + r) * .5f;
    e->rotation_deg = (r - l) * 180 / (110 * 3.14159265358979323846f);
    if (!e->active)
        return e->fault ? MOTION_CMD_FAILED : MOTION_CMD_DONE;
    e->target[0] = e->target[1] = 0;
    if (f->now_us >= e->deadline)
        e->fault = EXEC_TIMEOUT;
    if (!e->turning && fabsf(e->rotation_deg) > 12)
        e->fault = EXEC_HEADING;
    float sign = e->goal > 0 ? 1 : -1;
    float along = sign * (e->turning ? e->rotation_deg : e->progress_mm);
    if (along < -5)
        e->fault = EXEC_DIRECTION;
    if (e->fault) {
        e->active = false;
        return MOTION_CMD_FAILED;
    }
    float remaining = fabsf(e->goal) - along;
    if (!e->settling && remaining <= 1) {
        e->settling = true;
        e->settle_since = f->now_us;
    }
    if (e->settling) {
        bool still = fabsf(f->speed_mm_s[0]) < 5 && fabsf(f->speed_mm_s[1]) < 5;
        if (!still)
            e->still_timing = false;
        else if (!e->still_timing) {
            e->still_timing = true;
            e->still_since = f->now_us;
        }
        if (e->still_timing && f->now_us - e->still_since >= 120000) {
            e->active = false;
            return MOTION_CMD_DONE;
        }
        if (f->now_us - e->settle_since >= 2000000) {
            e->fault = EXEC_SETTLE_TIMEOUT;
            e->active = false;
            return MOTION_CMD_FAILED;
        }
        return MOTION_CMD_RUNNING;
    }
    float speed = e->speed;
    if (e->turning) {
        if (remaining < 12 && speed > 30)
            speed = 30;
        if (e->style == MOTION_IN_PLACE) {
            e->target[0] = (int16_t)(-sign * speed);
            e->target[1] = (int16_t)(sign * speed);
        } else
            e->target[sign > 0 ? 1 : 0] = (int16_t)speed;
    } else {
        if (remaining < 10 && speed > 25)
            speed = 25;
        float correction =
            bound((r - l) * 1.2f + .08f * (f->speed_mm_s[1] - f->speed_mm_s[0]), -8, 8);
        e->target[0] = (int16_t)lroundf(sign * speed + correction);
        e->target[1] = (int16_t)lroundf(sign * speed - correction);
    }
    return MOTION_CMD_RUNNING;
}
