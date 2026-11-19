/**
 * @file motion_executor.h
 * @brief Motion executor.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_MOTION_EXECUTOR_H
#define BUDDY_MOTION_EXECUTOR_H
#include "src/motion/motion_port.h"
typedef struct
{
    bool active;
    bool turning;
    bool settling;
    bool still_timing;
    motion_turn_style_t style;
    float goal;
    float speed;
    float progress_mm;
    float rotation_deg;
    double origin[2];
    uint64_t since;
    uint64_t deadline;
    uint64_t settle_since;
    uint64_t still_since;
    int16_t target[2];
    unsigned fault : 10;
#ifdef MOTION_STEERED_DRIVE
    /* Drive heading setpoint as right-minus-left wheel travel (mm), set only
     * by the lease holder through the coordinator; 0 = original straight
     * hold. The 12 deg heading fault is unchanged. */
    float steer_mm;
#endif
} motion_executor_t;
bool motion_executor_start (motion_executor_t * p_event,
                            const motion_feedback_t * p_flags, bool turn,
                            float goal, unsigned speed,
                            motion_turn_style_t style);
motion_command_state_t motion_executor_step (motion_executor_t * p_event,
                                             const motion_feedback_t * p_flags);
#endif

/* End of file. */
