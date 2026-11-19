/**
 * @file obstacle_policy.h
 * @brief Hardware-free Buddy5 RTOS adapter rules.
 * @details Shared by production and host tests. The obstacle algorithms are
 * the ported SDK modules in firmware/integration/buddy1_4/extensions/
 * buddy5_obstacles; the sequencing is b5_mission.c.
 * @see app_program/obstacle_navigation/README.md
 */
#ifndef RTOS_OBSTACLE_POLICY_H
#define RTOS_OBSTACLE_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include "include/interfaces/motion_api.h"
#include "extensions/buddy5_obstacles/profile/b5_config.h"

/* GP17 ECHO edge bits in IO_BANK0 INTR2/INTE2/INTS2 (GPIO16..23, 4 bits per
 * pin: level-low, level-high, edge-low, edge-high). GP20 Stop/E-stop uses
 * bit 18 (4 << 16) of the same register and is never changed here. */
enum
{
    RB5_ECHO_SHIFT = 4U * (B5_PIN_US_ECHO - 16U),
    RB5_ECHO_FALL = 4U << RB5_ECHO_SHIFT,
    RB5_ECHO_RISE = 8U << RB5_ECHO_SHIFT,
    RB5_ECHO_EDGES = RB5_ECHO_FALL | RB5_ECHO_RISE,
    RB5_BUTTON_EDGES = 0x00ff0000U,
    RB5_TASK_PRIORITY = 20,      /* TEAM.md Buddy5 band, core1 below motion */
    RB5_TASK_STACK = 3072,       /* bytes; deepest chain (o report -> tm_printf)
                                    ~1.8 KB, see the v0.2.12 long-width handoff */
    RB5_READY_MAX_MS = 100U,     /* start gate: readiness must be this fresh */
    RB5_READY_PING_US = 250000U, /* v0.6.1 b5_auto_ready(): good ping age   */
    RB5_FRAME_SPACING_US = 25000U,  /* SDK probe runtimes: one frame / 25 ms  */
    RB5_STATUS_PERIOD_US = 500000U, /* E5 mission status cadence             */
    RB5_SERVO_LIMIT_DD = 460,   /* +/-46 deg: 55 deg hits the battery mount */
    RB5_RECORD_MIN_FREE = 512U, /* records only while the 1 KiB TX queue is
                                   at least half free (Buddy1-4 keep room) */
    RB5_STATUS_MIN_FREE = 256U  /* E5 status needs this much free space    */
};
_Static_assert(B5_PIN_US_ECHO >= 16U && B5_PIN_US_ECHO <= 19U,
               "ECHO must stay in INTR2 below GP20");
_Static_assert((RB5_ECHO_EDGES & RB5_BUTTON_EDGES) == 0U,
               "ECHO edges must not alias GP20/21");
_Static_assert(B5_PIN_US_TRIG != B5_PIN_US_ECHO && B5_PIN_SERVO_PWM == 12U,
               "Servo must stay on PWM slice 6A, away from motor slices 4/5");

/* SDK servo law: 0.1 deg (-left/+right), 0 = 1450 us; 50 Hz, 1 us ticks.
 * This is the APPLIED PWM command, never a measured horn angle. */
static inline unsigned rb5_servo_pulse_us (int angle_dd)
{
    if (angle_dd < -900)
    {
        angle_dd = -900;
    }
    if (angle_dd > 900)
    {
        angle_dd = 900;
    }
    return B5_SERVO_MIN_US +
           ((B5_SERVO_MAX_US - B5_SERVO_MIN_US) * (unsigned)(900 - angle_dd)) /
               1800U;
}

/* Frames in which Buddy3's optical provider must NOT run: the session is
 * active, the coordinator is outside AUTO and Buddy5 holds the lease, or
 * Buddy5's own successful hand-back is awaiting the coordinator update that
 * returns AUTO. A coordinator FAULT is excluded so the existing control-task
 * E-stop path still handles it. Network Pause is handled by its own branch. */
static inline bool rb5_external_frame (const motion_status_t * p_status,
                                       bool handback_pending)
{
    return (p_status->session_active) && (p_status->mode != MOTION_AUTO) &&
           (p_status->mode != MOTION_FAULT) &&
           ((MOTION_BUDDY5 == p_status->owner) ||
            ((handback_pending) && (MOTION_BUDDY3 == p_status->owner)));
}

/* Frames whose (external) motor demand belongs to Buddy5, judged after its
 * decision step. Any other external demand outside Pause keeps the existing
 * control-task E-stop behaviour. */
static inline bool rb5_owned_demand (const motion_status_t * p_status,
                                     bool external, bool handback_pending)
{
    return (external) && (rb5_external_frame(p_status, handback_pending));
}

/* SDK rule: a Pause while Buddy5 holds the lease cancels Buddy5 and the
 * robot stays stopped (the old lease cannot resume it). */
static inline bool rb5_pause_cancels (const motion_status_t * p_status)
{
    return (p_status->session_active) && (MOTION_BUDDY5 == p_status->owner);
}

/* Min/max trackers for sonar diagnostics; zero means "no sample yet". */
static inline void rb5_track (uint32_t value, uint32_t * p_min,
                              uint32_t * p_max)
{
    if ((0U == *p_max) || (value < *p_min))
    {
        *p_min = value;
    }
    if (value > *p_max)
    {
        *p_max = value;
    }
}

#endif

/* End of file. */
