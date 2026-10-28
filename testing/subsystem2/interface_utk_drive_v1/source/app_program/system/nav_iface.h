/**
 * @file    nav_iface.h
 * @brief   Navigation Coordinator: public interface for every buddy.
 *
 * Navigation Coordinator holds the car's operating state (Section 5) and
 * decides which behaviour may move the car (Section 3): line following,
 * a barcode turn, an obstacle bypass, or a line search. It is the only
 * task that sends commands to Motion Control (FR-SYS-02).
 *
 * Every other task talks to it by posting a message with one of the
 * nav_post_*() functions below. They never block: if the inbox is full the
 * call returns E_TMOUT and the message is dropped (and counted).
 * Call them from TASK context only, not from an interrupt handler.
 *
 *   WiFi and MQTT (Buddy 1)    nav_post_command()
 *   Motion Control (Buddy 2)   nav_post_motion_result()
 *   Line and Barcode (Buddy 3) nav_line_drive() (direct, while granted),
 *                              nav_post_line_event(), nav_post_barcode()
 *                              (nav_post_line_speeds() still works, queued)
 *   Obstacle Scan (Buddy 5)    through b5_nav_*() in nav_b5.c
 *   Safety Supervisor (team)   nav_post_safety()
 *
 * In the other direction Navigation Coordinator calls functions those
 * buddies provide (section "Provided by other buddies" at the bottom).
 * nav_stubs.c gives weak defaults so this module builds on its own; a
 * normal definition with the same name anywhere else replaces each one.
 *
 * Units: mm, mm/s, tenths of a degree ("dd"). Heading and turn angles are
 * POSITIVE to the RIGHT (clockwise seen from above).
 *
 * Minimum-bit data (Tribunal 1): the coordinator's own records and its
 * inbox messages are bit-fields sized to each value's range, and its
 * telemetry is a binary frame in the team schema (sys_tlm.h). Function
 * parameters keep ordinary C types: they travel in 32-bit registers, so
 * narrowing them saves nothing; values are clamped where they are stored.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#ifndef NAV_IFACE_H
#define NAV_IFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "sys_tlm.h"

/* ------------------------------------------------------------------------
 * Operating state (system-level statechart, Section 5)
 * --------------------------------------------------------------------- */
typedef enum
{
    NAV_ST_READY = 0,       /* motors off, waiting for start          */
    NAV_ST_RUNNING,         /* autonomous run                         */
    NAV_ST_PAUSED,          /* motors stopped, tasks keep running     */
    NAV_ST_SAFE_STOP,       /* controlled stop after a stop command   */
    NAV_ST_COMPLETED,       /* course finished                        */
    NAV_ST_ESTOP,           /* emergency stop, latched                */
    NAV_ST_FAULT,           /* unrecoverable fault                    */
    NAV_ST_COUNT
} nav_state_t;

/* Which behaviour currently owns motion while RUNNING. */
typedef enum
{
    NAV_OWNER_NONE = 0,
    NAV_OWNER_LINE,         /* Line and Barcode wheel-speed proposals */
    NAV_OWNER_BARCODE,      /* executing a decoded barcode turn       */
    NAV_OWNER_SEARCH,       /* line lost: sweeping to find it         */
    NAV_OWNER_OBSTACLE,     /* Obstacle Scan and Recovery (Buddy 5)   */
    NAV_OWNER_COUNT
} nav_owner_t;

/* ------------------------------------------------------------------------
 * Messages into Navigation Coordinator
 * --------------------------------------------------------------------- */

/* Supervisory commands from WiFi and MQTT. Emergency stop does NOT come
 * here: WiFi sends it straight to Safety Supervisor (Section 4). */
typedef enum
{
    NAV_CMD_START = 1,
    NAV_CMD_PAUSE,
    NAV_CMD_RESUME,
    NAV_CMD_STOP,
    NAV_CMD_RESET,
    NAV_CMD_COURSE_FINISHED     /* end of course detected            */
} nav_cmd_t;

/* Result of one Motion Control command (FR-MOT-04). */
typedef enum
{
    NAV_MOTION_DONE = 1,
    NAV_MOTION_CANCELLED,
    NAV_MOTION_FAILED
} nav_motion_result_t;

/* Line and Barcode events (FR-LINE-02). */
typedef enum
{
    NAV_LINE_LOST = 1,
    NAV_LINE_FOUND,
    NAV_LINE_JUNCTION
} nav_line_event_t;

/* Decoded barcode navigation commands (FR-LINE-04).
 *
 * Barcode turns search for the line instead of turning a fixed angle
 * (the course does not say how sharp a branch is):
 *  1. Line and Barcode posts LEFT or RIGHT once the wheel axle is over the
 *     next junction (or where the line ended); UTURN right after the
 *     barcode; STRAIGHT right after the barcode, and keeps following.
 *  2. The coordinator takes control, stops, and spins slowly on the spot:
 *     LEFT to the left, RIGHT and UTURN to the right.
 *  3. Line and Barcode watches the LEADING front sensor (left for LEFT,
 *     right for RIGHT and UTURN) and posts NAV_LINE_FOUND each time it
 *     goes from white to black.
 *  4. The coordinator hands line control back on the first NAV_LINE_FOUND
 *     (for a U-turn, the first after NAV_BC_UTURN_MIN_DD). A full turn
 *     with no line is a line-lost fault. */
typedef enum
{
    NAV_BARCODE_LEFT = 1,
    NAV_BARCODE_RIGHT,
    NAV_BARCODE_STRAIGHT,
    NAV_BARCODE_UTURN
} nav_barcode_t;

#define NAV_UTURN_TURNS_RIGHT   (1)     /* 1: U-turns spin right; 0: left */

/* Safety Supervisor notifications. */
typedef enum
{
    NAV_SAFETY_ESTOP = 1,       /* emergency stop accepted            */
    NAV_SAFETY_FAULT,           /* unrecoverable fault                */
    NAV_SAFETY_CLEARED          /* cause cleared; a reset may follow  */
} nav_safety_t;

/* Motion Control command ids are 15 bits: 1..NAV_CMD_ID_MAX, then they
 * wrap back to 1 (0 means "no command"). An id above this can never be
 * one of ours, so its result is ignored. */
#define NAV_CMD_ID_MAX      (0x7FFFU)

/* Status for telemetry: see the frame layout and nav_tlm_t in sys_tlm.h. */

/* ------------------------------------------------------------------------
 * Provided by Navigation Coordinator
 * --------------------------------------------------------------------- */
ER           nav_init(void);            /* inbox (phase 1)            */
ER           nav_start(void);           /* task (phase 2)             */
nav_state_t  nav_get_state(void);
nav_owner_t  nav_get_owner(void);
char const * nav_state_name(nav_state_t state);
char const * nav_owner_name(nav_owner_t owner);

/* Line and Barcode (Buddy 3): steer directly while line control is
 * granted (no inbox in the steering loop). Returns E_OK if the speeds went
 * to Motion Control, E_OBJ if line control is not granted now (dropped).
 * Call every line cycle: Motion Control treats a wheel-speed target older
 * than NAV_LINE_STALE_MS as stale and stops (Week 6 feedback). */
ER nav_line_drive(int16_t left_mm_s, int16_t right_mm_s);
#define NAV_LINE_STALE_MS   (50U)   /* 5 line cycles of 10 ms */

ER nav_post_command(nav_cmd_t cmd);
ER nav_post_motion_result(uint32_t cmd_id, nav_motion_result_t result);
ER nav_post_line_speeds(int16_t left_mm_s, int16_t right_mm_s);
ER nav_post_line_event(nav_line_event_t event);
ER nav_post_barcode(nav_barcode_t command);
ER nav_post_safety(nav_safety_t event);

/* ------------------------------------------------------------------------
 * Provided by other buddies (weak defaults in nav_stubs.c)
 * --------------------------------------------------------------------- */

/* Motion Control (Buddy 2). Every bounded command carries cmd_id, and
 * a wheel-speed target not renewed within NAV_LINE_STALE_MS means stop
 * (stale-target timeout, Week 6 statechart 6.2 "stale speed targets"). */
/* Motion Control (Buddy 2). Every bounded command carries cmd_id, and
 * Motion Control must answer it with nav_post_motion_result(cmd_id, ...).
 * A new command, including mc_set_wheel_speeds, replaces the current one
 * (the "handover" in statechart 6.2); the replaced command is answered
 * with NAV_MOTION_CANCELLED. */
ER mc_set_wheel_speeds(int16_t left_mm_s, int16_t right_mm_s); /* FR-MOT-05 */
ER mc_drive_distance(int32_t distance_mm, int16_t speed_mm_s,
                     uint32_t cmd_id);                        /* FR-MOT-01 */
ER mc_turn(int32_t angle_dd, int16_t speed_mm_s, uint32_t cmd_id);
ER mc_stop(uint32_t cmd_id);            /* controlled stop to standstill */

/* Line and Barcode (Buddy 3): may it propose wheel speeds right now?
 * (the "line control granted" guard in statechart 6.3) */
void line_set_control(bool b_granted);

/* Operating-state broadcast to the tasks that need it. */
void line_notify_state(nav_state_t state);
void imu_notify_state(nav_state_t state);
void wifi_notify_state(nav_state_t state);

/* WiFi and MQTT (Buddy 1): queue the NAV_TLM_FRAME_BYTES-byte binary
 * frame (bit map in sys_tlm.h) for MQTT. Called from the coordinator
 * task; copy the bytes and return, do not block. */
void nav_telemetry_publish(uint8_t const * p_frame, uint32_t len);

/* Safety Supervisor (team): code is one of NAV_FAULT_* (8 bits). */
void nav_report_fault(uint8_t code);

/* Called on reset from Fault so the team's start-up code can re-run
 * initialisation and recalibration before the car is Ready again. */
void nav_reinitialise(void);

/* Safety Supervisor: the "[tasks healthy]" guard on start (Section 5),
 * and a check-in the coordinator makes every time it wakes. */
bool nav_start_allowed(void);
void nav_heartbeat(void);

#endif /* NAV_IFACE_H */
