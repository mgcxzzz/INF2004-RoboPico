/**
 * @file    b5_iface.h
 * @brief   Buddy 5 interface to the rest of the car (the Section 4 arrows).
 *
 * Obstacle Scan and Recovery never drives the motors. Everything it needs
 * from, or asks of, another task goes through the functions in this file:
 *
 *   IN  (others -> Buddy 5)
 *     b5_post_event()            Navigation Coordinator / system events
 *     b5_motion_get_estimate()   Motion Control distance + heading (FR-MOT-03)
 *     b5_line_detected()         Line and Barcode line-detected status
 *
 *   OUT (Buddy 5 -> others)
 *     b5_nav_*()                 requests to Navigation Coordinator
 *     b5_telemetry_publish()     status to WiFi and MQTT (telemetry queue)
 *     b5_report_fault()          fault to Safety Supervisor
 *
 * Minimum-bit data (Tribunal 1): the records below are C bit-fields sized
 * to each value's range (see b5_pack.h for the table). Buddy 5 only ever
 * hands other tasks a copy or a const pointer during a call, and what goes
 * to MQTT is the explicitly packed frame from b5_pack.h, never these
 * structs, so the compiler's bit-field layout never crosses a boundary.
 *
 * The OUT functions and the two IN getters are implemented by the other
 * buddies. b5_stubs.c gives weak default versions so Buddy 5 builds and
 * fails safe on its own; a strong definition elsewhere replaces each one.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#ifndef B5_IFACE_H
#define B5_IFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "b5_pack.h"
#include "obstacle_profile.h"

/* ------------------------------------------------------------------------
 * Events into Buddy 5 (bit mask for b5_post_event)
 * --------------------------------------------------------------------- */
#define B5_EV_MISSION_START   (1U << 0)  /* Ready -> Running            */
#define B5_EV_MISSION_STOP    (1U << 1)  /* stop / pause / safe stop    */
#define B5_EV_STOP_ACCEPTED   (1U << 2)  /* reply to b5_nav_request_stop */
#define B5_EV_PLAN_ACCEPTED   (1U << 3)  /* reply to b5_nav_send_plan    */
#define B5_EV_PLAN_REJECTED   (1U << 4)  /* Nav refused the bypass       */
#define B5_EV_MOTION_DONE     (1U << 5)  /* requested motion finished    */
#define B5_EV_MOTION_FAILED   (1U << 6)  /* requested motion failed      */
#define B5_EV_FAULT_RESET     (1U << 7)  /* fault acknowledged, reset    */
#define B5_EV_ALL             (0xFFU)

/* ------------------------------------------------------------------------
 * Types
 * --------------------------------------------------------------------- */

/** States of statechart 6.5. */
typedef enum
{
    B5_ST_IDLE = 0,
    B5_ST_GUARD,
    B5_ST_CONFIRM,
    B5_ST_HALTING,
    B5_ST_COARSE_SWEEP,
    B5_ST_EDGE_REFINE,
    B5_ST_PLANNING,
    B5_ST_BYPASSING,
    B5_ST_RECOVERING,
    B5_ST_BLOCKED,
    B5_ST_FAULT,
    B5_ST_COUNT
} b5_state_t;

/** Why Buddy 5 entered Fault. */
typedef enum
{
    B5_FAULT_NONE = 0,
    B5_FAULT_ECHO_TIMEOUT,      /* sensor stopped responding            */
    B5_FAULT_SERVO,             /* PWM not holding the commanded pulse  */
    B5_FAULT_LINE_LOST,         /* 3 failed line searches               */
    B5_FAULT_BLOCKED,           /* blocked, reverse attempts exhausted  */
    B5_FAULT_STOP_TIMEOUT,      /* Nav never accepted the stop request  */
    B5_FAULT_MOTION             /* bypass or rejoin motion failed       */
} b5_fault_t;

/**
 * Motion Control estimate (FR-MOT-03), agreed with Buddy 2 in Tribunal 1:
 *  - distance_mm is the signed NET distance (forward minus reverse);
 *  - heading_dd is POSITIVE when the car has turned RIGHT (clockwise);
 *  - both are 16-bit and allowed to WRAP. Buddy 5 only ever uses the
 *    difference over one bypass (about 1-2 m, a few turns), taken with
 *    b5_wrap_diff16(), which stays correct across a wrap because the
 *    window is far below half the range (+-32 m, +-327 deg).
 */
typedef struct
{
    int16_t distance_mm;        /**< signed net distance, wraps          */
    int16_t heading_dd;         /**< heading change, 0.1 deg, wraps      */
} b5_motion_est_t;

/** Difference of two wrapping 16-bit values (now - then). */
static inline int32_t
b5_wrap_diff16(int16_t now, int16_t then)
{
    return (int32_t)(int16_t)(uint16_t)((uint16_t)now - (uint16_t)then);
}

/** Bypass plan sent to Navigation Coordinator (FR-USO-04). */
typedef struct
{
    uint32_t side       : 2;    /**< b5_side_t: go round on this side    */
    uint32_t lateral_mm : 12;   /**< sideways shift off the line         */
    uint32_t forward_mm : 12;   /**< travel along the line to pass it    */
    uint32_t closest_mm : 12;   /**< obstacle range when planned         */
    uint32_t width_mm   : 12;   /**< corrected obstacle width            */
} b5_plan_t;

/** Line-rejoin request sent to Navigation Coordinator (FR-USO-06). */
typedef struct
{
    int32_t  lateral_mm : 14;   /**< car is this far RIGHT of the line   */
    int32_t  heading_dd : 11;   /**< car heading relative to the line    */
    uint32_t attempt    : 2;    /**< 0, 1, 2                             */
    uint32_t search_mm  : 12;   /**< sweep width for this search leg     */
} b5_rejoin_t;

/** Buddy 5 status record. Written only by the obstacle task. */
typedef struct
{
    uint32_t     state         : 4;  /**< b5_state_t                     */
    uint32_t     fault         : 3;  /**< b5_fault_t                     */
    uint32_t     blocked_tries : 2;  /**< 0..2                           */
    uint32_t     recover_tries : 2;  /**< 0..3                           */
    int32_t      servo_dd      : 11; /**< -900..+900, + = right          */
    uint32_t     range_mm      : 12; /**< last guard reading, 0..4000    */
    uint32_t     pings         : 16; /**< wrapping counter               */
    uint32_t     no_echo       : 16; /**< pings with no reply, wrapping  */
    b5_profile_t profile;            /**< last Planning result           */
} b5_status_t;

/* ------------------------------------------------------------------------
 * Provided by Buddy 5
 * --------------------------------------------------------------------- */
ER          b5_init(void);                  /* drivers + objects (phase 1) */
ER          b5_start(void);                 /* start the task (phase 2)   */
void        b5_get_status(b5_status_t * p_status);
ER          b5_post_event(UINT events);     /* any task or handler        */
b5_state_t  b5_get_state(void);             /* current state, any task    */
char const * b5_state_name(b5_state_t state);

/* Status -> 16-byte binary telemetry frame (layout in b5_pack.h). */
void        b5_status_to_frame(b5_status_t const * p_status,
                               uint8_t p_frame[B5_TLM_FRAME_BYTES]);

/* ------------------------------------------------------------------------
 * Provided by other buddies (weak defaults in b5_stubs.c)
 * --------------------------------------------------------------------- */

/* Navigation Coordinator (team). Each request is answered with an event. */
void b5_nav_request_slow(bool b_slow);                 /* FR-USO-02      */
void b5_nav_request_stop(void);                        /* -> STOP_ACCEPTED */
void b5_nav_send_plan(b5_plan_t const * p_plan);       /* -> PLAN_*, MOTION_* */
void b5_nav_request_rejoin(b5_rejoin_t const * p_req); /* -> MOTION_*     */
void b5_nav_request_reverse(int32_t distance_mm);      /* -> MOTION_*     */
void b5_nav_blocked_route(b5_profile_t const * p_prof);/* FR-USO-07       */
void b5_nav_release_motion(void);   /* line following may resume          */

/* Motion Control (Buddy 2) and Line and Barcode (Buddy 3). */
bool b5_motion_get_estimate(b5_motion_est_t * p_est);
bool b5_line_detected(void);

/* WiFi and MQTT (Buddy 1): queue a packed binary frame for MQTT. It is
 * B5_TLM_FRAME_BYTES long, little-endian; decode with b5_tlm_unpack(). */
void b5_telemetry_publish(uint8_t const * p_frame, uint32_t len);

/* Safety Supervisor (team). The matching bit is also set in the shared
 * fault mask (system/sys_fault.h). */
void b5_report_fault(b5_fault_t fault);

#endif /* B5_IFACE_H */
