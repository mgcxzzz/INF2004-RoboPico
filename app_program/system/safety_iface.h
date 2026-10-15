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
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#ifndef SAFETY_IFACE_H
#define SAFETY_IFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

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

/* Fault codes raised by the supervisor itself. */
#define SAFETY_CODE_TASK_SILENT     (0x0601U)   /* missed check-ins  */
#define SAFETY_CODE_STOP_SLOW       (0x0602U)   /* drive-off > 100 ms */

/* Status snapshot for telemetry. */
typedef struct
{
    safety_state_t state;
    bool           b_inhibit;       /* motor drive inhibited          */
    safety_src_t   fault_src;       /* last critical fault            */
    uint32_t       fault_code;
    safety_src_t   warn_src;        /* last warning                   */
    uint32_t       warn_code;
    uint32_t       estops;          /* counters since boot            */
    uint32_t       faults;
    uint32_t       warnings;
    uint32_t       last_stop_ms;    /* accept -> drive off, last stop */
    uint32_t       max_stop_ms;     /* worst seen (NFR-SYS-04: < 100) */
    uint32_t       slow_stops;      /* stops that took >= 100 ms      */
} safety_status_t;

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
 * that is logged and published but does not stop the car. */
ER   safety_report_fault(safety_src_t src, uint32_t code, bool b_critical);

/* Health monitoring for periodic tasks: declare once, then check in at
 * least every timeout_ms while the car is running. */
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

/* WiFi and MQTT (Buddy 1): copy the status into the telemetry queue. */
void safety_telemetry_publish(safety_status_t const * p_status);

#endif /* SAFETY_IFACE_H */
