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
 * Motion Control estimate (FR-MOT-03), cumulative since boot.
 * heading_dd is POSITIVE when the car has turned to the RIGHT (clockwise).
 */
typedef struct
{
    int32_t distance_mm;        /**< signed distance travelled           */
    int32_t heading_dd;         /**< heading change, tenths of a degree  */
} b5_motion_est_t;

/** Bypass plan sent to Navigation Coordinator (FR-USO-04). */
typedef struct
{
    b5_side_t side;             /**< go round on this side               */
    int32_t   lateral_mm;       /**< sideways shift off the line         */
    int32_t   forward_mm;       /**< travel along the line to pass it    */
    int32_t   closest_mm;       /**< obstacle range when planned         */
    int32_t   width_mm;         /**< corrected obstacle width            */
} b5_plan_t;

/** Line-rejoin request sent to Navigation Coordinator (FR-USO-06). */
typedef struct
{
    int32_t lateral_mm;         /**< car is this far RIGHT of the line   */
    int32_t heading_dd;         /**< car heading relative to the line    */
    int32_t search_mm;          /**< sweep width for this search leg     */
    uint8_t attempt;            /**< 0, 1, 2                             */
} b5_rejoin_t;

/** Snapshot published to telemetry. */
typedef struct
{
    b5_state_t   state;
    b5_fault_t   fault;
    int32_t      range_mm;      /**< last guard reading                  */
    int16_t      servo_dd;
    uint32_t     pings;
    uint32_t     no_echo;       /**< pings with no response              */
    uint8_t      blocked_tries;
    uint8_t      recover_tries;
    b5_profile_t profile;       /**< last Planning result                */
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

/* WiFi and MQTT (Buddy 1) and Safety Supervisor (team). */
void b5_telemetry_publish(b5_status_t const * p_status);
void b5_report_fault(b5_fault_t fault);

#endif /* B5_IFACE_H */
