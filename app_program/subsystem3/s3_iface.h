/**
 * @file    s3_iface.h
 * @brief   Buddy 3 (Line and Barcode): interface with the other subsystems.
 *
 * Provided by Buddy 3 to the others:
 *     line_set_control()      Navigation Coordinator grants / revokes steering
 *     line_notify_state()     Navigation Coordinator's operating state
 *     b5_line_detected()      Buddy 5: either front sensor sees the line
 *     s3_get_state()          for tests and the console
 *
 * Used by Buddy 3 (all through the Navigation Coordinator, never the
 * motors directly):
 *     nav_line_drive()        wheel speeds, only while control is granted
 *     nav_post_line_event()   line lost / found / junction
 *     nav_post_barcode()      one validated event per barcode (FR-LINE-05)
 *
 * Provided by other buddies (weak defaults in s3_stubs.c):
 *     s3_motion_get_distance()   Buddy 2: net signed distance, int16 mm,
 *                                 may wrap (same rule as Buddy 5)
 *     line_telemetry_publish()   Buddy 1: queue the binary frame (sys_tlm.h)
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#ifndef S3_IFACE_H
#define S3_IFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#include "../system/sys_tlm.h"

/* Statechart 6.3 (3 bits). Mission Active = FOLLOW, READ, AWAIT. */
typedef enum
{
    S3_ST_INIT = 0,         /* Initialising                              */
    S3_ST_IDLE,             /* Idle / Ready                              */
    S3_ST_FOLLOW,           /* Line Following                            */
    S3_ST_READ,             /* Read Barcode (+ Decode and Request)       */
    S3_ST_AWAIT,            /* Await Navigation                          */
    S3_ST_FAULT,            /* Fault / Inhibited                         */
    S3_ST_COUNT
} s3_state_t;

ER           s3_init(void);         /* pins, interrupt, event flag   */
ER           s3_start(void);        /* the Line and Barcode task     */
s3_state_t   s3_get_state(void);
char const * s3_state_name(s3_state_t state);

/* One cycle of the task (the task calls it every S3_PERIOD_MS; host and
 * on-target tests call it directly). */
void         s3_step(void);

/* Provided by other buddies (weak defaults in s3_stubs.c). */
bool s3_motion_get_distance(int16_t * p_mm);
void line_telemetry_publish(uint8_t const * p_frame, uint32_t len);

/* Hardware access, in s3_ir.c (replaced by a mock in the host test). */
ER       s3_ir_init(void);
uint32_t s3_ir_read(void);                          /* S3_SENSE_* bits */
bool     s3_ir_pop_edge(uint32_t * p_t_us, bool * p_b_black);
uint32_t s3_ir_overflows(void);                     /* wrapping count  */
uint32_t s3_time_us(void);                          /* sys_time_us()   */

#endif /* S3_IFACE_H */
