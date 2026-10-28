/**
 * @file    subsystem1.h
 * @brief   Subsystem 1 -- WiFi and MQTT (Buddy 1). Student ID 2502153.
 *
 * Statechart 6.1 (Week 6): Disconnected -> Connecting -> Operational
 * (publish telemetry and heartbeat, validate and dispatch commands) ->
 * Reconnect Wait on any loss, then back to Disconnected. Link loss never
 * changes the operating state or touches navigation or the motors.
 *
 * Requirements: FR-WIFI-01..05, NFR-WIFI-01 (>= 95 % delivered in 5 min),
 * NFR-WIFI-02 (heartbeat ~1 s, no gap > 2 s). Week 8: every message is a
 * compact binary frame in the team schema (system/sys_tlm.h).
 *
 * Public interface: the *_telemetry_publish() hooks declared by the
 * other buddies (nav_iface.h, safety_iface.h, b5_iface.h, s3_iface.h,
 * imu_iface.h) are all implemented in task_wifi.c, plus the generic
 * b1_telemetry_publish() below for any buddy without its own hook.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#include <stdint.h>

#define S1_PRI_WIFI         (12)        /* band 12-13 (TEAM.md section 4)  */
#define S1_TICK_MS          (100)       /* link check and health check-in  */
#define S1_WATCH_MS         (3000U)     /* Safety Supervisor health timeout */
#define S1_OUT_BUF_BYTES    (2048)      /* outgoing queue (all buddies)    */
#define S1_CMD_BUF_BYTES    (64)        /* incoming commands               */
#define S1_CONSOLE_LOG      (1)         /* 1 = print link and command events */

/* Queue a packed frame (team schema) for MQTT topic is28/tlm. Copies the
 * bytes and returns at once; a full queue drops the frame and counts it.
 * Safe from any task on either core (not from an interrupt). */
void b1_telemetry_publish(uint8_t const * p_frame, uint32_t len);

/* Integration API for later: get one coherent map/valid/revision word.
 * Task context on either core; never waits. E_OBJ before initialisation,
 * E_TMOUT if busy. Retry next cycle; never use an uninitialised snapshot.
 * Interpret with b1_mapping_lookup() (b1_mapping.h). This API does not
 * decode a barcode, store a pending letter, or command any movement. */
ER b1_barcode_mapping_get(uint32_t * p_snapshot);

/* Private to subsystem 1, shared between its own .c files. */
IMPORT ER   b1_create_objects(void);
IMPORT void b1_task(INT stacd, void * p_exinf);
IMPORT ER   b1_glue_start(void);       /* b1_branch_glue.c */

/* Template compatibility (b1_branch_glue.c): the template's subsystem 4
 * monitor still pauses and resumes "the LED task" (IF-05). Buddy 1 has no
 * LED, so the glue gives it an idle stand-in. Remove after the merge. */
#define S1_PRI_LED          (13)
IMPORT ID   tid_led;

#endif /* SUBSYSTEM1_H */
