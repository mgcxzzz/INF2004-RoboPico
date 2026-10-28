/**
 * @file    nav_internal.h
 * @brief   Navigation Coordinator inbox message format (module-private).
 *
 * Only nav_coordinator.c and the glue files in nav/src include this.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#ifndef NAV_INTERNAL_H
#define NAV_INTERNAL_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    NAV_MSG_COMMAND = 1,    /* a = nav_cmd_t                              */
    NAV_MSG_MOTION,         /* a = cmd_id, b = nav_motion_result_t        */
    NAV_MSG_LINE_SPEEDS,    /* a = left mm/s, b = right mm/s              */
    NAV_MSG_LINE_EVENT,     /* a = nav_line_event_t                       */
    NAV_MSG_BARCODE,        /* a = nav_barcode_t                          */
    NAV_MSG_SAFETY,         /* a = nav_safety_t                           */
    NAV_MSG_B5_SLOW,        /* a = 1 slow, 0 normal                       */
    NAV_MSG_B5_STOP,
    NAV_MSG_B5_PLAN,        /* a = side (1 right, 2 left), b = lateral mm,
                               c = forward mm                             */
    NAV_MSG_B5_REJOIN,      /* a = lateral mm (+ = right of line),
                               b = heading dd, c = search mm              */
    NAV_MSG_B5_REVERSE,     /* a = distance mm                            */
    NAV_MSG_B5_BLOCKED,
    NAV_MSG_B5_RELEASE      /* a = 1 if Buddy 5 is in Fault               */
} nav_msg_type_t;

/* One inbox message: 8 bytes instead of 16 (Tribunal 1, minimum bits).
 *   type  4 bits      13 message types
 *   a    16 bits      cmd id (1..0x7FFF), wheel speed, lateral mm,
 *                     reverse distance, side, flags
 *   b    16 bits      result, wheel speed, lateral or heading
 *   c    12 bits      forward or search distance, 0..4095 mm
 * nav_post_msg() clamps every value to its field before storing it.
 * The message is copied into the kernel message buffer and out again, so
 * sender and coordinator never share it: bit-fields are safe here, and it
 * never leaves the chip, so their compiler-chosen layout does not matter. */
typedef struct
{
    uint32_t type : 4;
    uint32_t c    : 12;
    int32_t  a    : 16;
    int32_t  b    : 16;
} nav_msg_t;

#define NAV_MSG_A_MIN       (-32768)
#define NAV_MSG_A_MAX       (32767)
#define NAV_MSG_C_MAX       (4095)

ER nav_post_msg(int32_t type, int32_t a, int32_t b, int32_t c);

#endif /* NAV_INTERNAL_H */
