/**
 * @file circuit_feedback.h
 * @brief Circuit feedback.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_FEEDBACK_H
#define CIRCUIT_FEEDBACK_H
#include <stdbool.h>
#include <stdint.h>
enum
{
    C_PROGRESS_TICKS = 4,
    C_STALL_US = 750000
};
typedef struct
{
    int64_t anchor;
    uint64_t since;
    bool watching;
} circuit_feedback_t;
/* Forward count accumulates across control ticks. Four transitions are about
   0.32mm with this robot's measured CPR. Slow/bursty motion is not discarded
   because any one10ms sample contains fewer than two transitions. */
static inline bool circuit_stalled (circuit_feedback_t * s, uint64_t now,
                                    int64_t forward_ticks, int target,
                                    float ramp)
{
    if (target < 20 || ramp < 20)
    {
        *s = (circuit_feedback_t){0};
        return false;
    }
    if (!s->watching)
    {
        *s = (circuit_feedback_t){
            .anchor = forward_ticks, .since = now, .watching = true};
        return false;
    }
    if (forward_ticks - s->anchor >= C_PROGRESS_TICKS)
    {
        s->anchor = forward_ticks;
        s->since = now;
    }
    return now - s->since >= C_STALL_US;
}
#endif

/* End of file. */
