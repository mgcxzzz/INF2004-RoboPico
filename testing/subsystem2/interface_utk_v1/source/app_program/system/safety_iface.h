/**
 * @file    safety_iface.h
 * @brief   Safety Supervisor: public interface for every buddy.
 *
 * The Safety Supervisor is the highest-priority task (Section 3). It
 *  - accepts an emergency stop and inhibits motor drive within 100 ms
 *    (FR-WIFI-02, NFR-SYS-04), latched until released and reset;
 *  - accepts unrecoverable faults from any task and brings the car to a
 *    safe stopped state (FR-SYS-03), latched until cleared and reset;
 *  - watches the periodic tasks: a task that stops checking in while the
 *    car is running is treated as an unrecoverable fault;
 *  - measures the time from accepting a stop to Motion Control confirming
 *    drive is off, the evidence NFR-SYS-04 asks for.
 *
 * Everything posted here goes through a kernel event flag, so every
 * safety_*() call below is safe from a task OR an interrupt handler and
 * never blocks.
 *
 * Minimum-bit data (Tribunal 1): fault codes are 8 bits (each source
 * numbers its own), counters 8 bits, stop times 10 bits (ms, saturating),
 * and the telemetry is a binary frame in the team schema (sys_tlm.h) that
 * also carries the shared fault mask.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#ifndef SAFETY_IFACE_H
#define SAFETY_IFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "sys_tlm.h"

/* Who is reporting (also the slot used for health monitoring). */
typedef enum
{
    SAFETY_SRC_WIFI = 0,        /* Buddy 1 */
    SAFETY_SRC_MOTION,          /* Buddy 2 */
    SAFETY_SRC_LINE,            /* Buddy 3 */
    SAFETY_SRC_IMU,             /* Buddy 4 */
    SAFETY_SRC_OBSTACLE,        /* Buddy 5 */
    SAFETY_SRC_NAV,             /* Navigation Coordinator */
    SAFETY_SRC_SAFETY,          /* the supervisor itself */
    SAFETY_SRC_COUNT
} safety_src_t;

/* Supervisor state. */
typedef enum
{
    SAFETY_ST_NORMAL = 0,
    SAFETY_ST_ESTOP,            /* emergency stop latched            */
    SAFETY_ST_FAULT,            /* unrecoverable fault latched       */
    SAFETY_ST_COUNT
} safety_state_t;

/* Fault codes raised by the supervisor itself (8 bits, like every
 * source's codes; the source is recorded separately). */
#define SAFETY_CODE_TASK_SILENT     (0x01U)     /* missed check-ins   */
#define SAFETY_CODE_STOP_SLOW       (0x02U)     /* drive-off > 100 ms */

/* Longest health-check timeout (stored in 16 bits, ms). */
#define SAFETY_WATCH_MAX_MS         (65535U)

/* Status for telemetry: see the frame layout and safety_tlm_t in
 * sys_tlm.h. */

/* ------------------------------------------------------------------------
 * Provided by the Safety Supervisor
 * --------------------------------------------------------------------- */
ER             safety_init(void);              /* event flag (phase 1) */
ER             safety_start(void);             /* task (phase 2)       */
safety_state_t safety_get_state(void);
bool           safety_drive_inhibited(void);    /* poll every control cycle */
char const *   safety_state_name(safety_state_t state);

/* WiFi and MQTT (Buddy 1). */
ER   safety_post_estop(void);           /* validated emergency stop     */
ER   safety_request_reset(void);        /* on a reset command, see README */

/* Any task. critical = unrecoverable (FR-SYS-03); otherwise a warning
 * that is logged and published but does not stop the car. code is 8 bits,
 * numbered by the reporting source. */
ER   safety_report_fault(safety_src_t src, uint8_t code, bool b_critical);

/* Health monitoring for periodic tasks: declare once, then check in at
 * least every timeout_ms (at most SAFETY_WATCH_MAX_MS) while the car is
 * running. */
ER   safety_watch(safety_src_t src, uint32_t timeout_ms);
void safety_kick(safety_src_t src);

/* Motion Control (Buddy 2): call once drive is actually off after
 * mc_inhibit(true). The supervisor times the stop from this. */
void safety_ack_drive_off(void);

/* ------------------------------------------------------------------------
 * Provided by other buddies (weak defaults in safety_stubs.c)
 * --------------------------------------------------------------------- */

/* Motion Control (Buddy 2): true = remove motor drive NOW and refuse
 * commands until false. Called from the supervisor task; keep it short.
 * Then call safety_ack_drive_off(). */
void mc_inhibit(bool b_inhibit);

/* WiFi and MQTT (Buddy 1): queue the SAFETY_TLM_FRAME_BYTES-byte binary
 * frame (bit map in sys_tlm.h) for MQTT. Called from the supervisor task:
 * copy the bytes and return, never block. */
void safety_telemetry_publish(uint8_t const * p_frame, uint32_t len);

#endif /* SAFETY_IFACE_H */
