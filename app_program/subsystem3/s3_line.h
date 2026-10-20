/**
 * @file    s3_line.h
 * @brief   Line position, steering proposal and line events from the two
 *          digital front sensors (FR-LINE-01, FR-LINE-02).
 *
 * Pure C: no kernel, no hardware, tested on the PC and on the Pico.
 * Every decision is made per millimetre travelled, not per cycle, so the
 * thresholds mean the same at any speed (see s3_config.h for the sensor
 * arrangement and why a junction is not a curve).
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#ifndef S3_LINE_H
#define S3_LINE_H

#include <stdbool.h>
#include <stdint.h>

/* Sensor bits, as returned by the IR driver (1 = black). */
#define S3_SENSE_LEFT           (1U << 0)
#define S3_SENSE_RIGHT          (1U << 1)
#define S3_SENSE_BARCODE        (1U << 2)

/* Where the line was last seen (2 bits). */
typedef enum
{
    S3_SIDE_NONE = 0,
    S3_SIDE_LEFT,
    S3_SIDE_RIGHT,
    S3_SIDE_BOTH
} s3_side_t;

/* Events from one step (bit set; any combination). */
#define S3_LEV_LOST             (1U << 0)   /* no contact for S3_LOST_MM      */
#define S3_LEV_FOUND            (1U << 1)   /* contact again after LOST       */
#define S3_LEV_JUNCTION         (1U << 2)   /* both black for S3_JUNCTION_MM  */
#define S3_LEV_STUCK            (1U << 3)   /* both black for S3_STUCK_MM     */

/* Tracker record: owned by the Line and Barcode task only (bit-fields,
 * minimum bits; distances saturate at 4095 mm, never wrap). */
typedef struct
{
    uint32_t last_side    : 2;      /* s3_side_t: side last seen      */
    uint32_t b_lost       : 1;
    uint32_t b_junction   : 1;      /* reported, waiting for clear    */
    uint32_t b_stuck      : 1;
    uint32_t no_line_mm   : 12;     /* travel since any contact        */
    uint32_t both_mm      : 12;     /* travel with both sensors black  */
} s3_line_t;

/* One step's output. Speeds are only USED while line control is granted;
 * the caller decides. */
typedef struct
{
    int16_t  left_mm_s;
    int16_t  right_mm_s;
    uint8_t  side;                  /* s3_side_t now                  */
    uint8_t  events;                /* S3_LEV_*                       */
} s3_line_out_t;

void s3_line_reset(s3_line_t * p_ln);

/**
 * One line-following step.
 * @param sensors   S3_SENSE_* bits, 1 = black
 * @param moved_mm  travel since the previous step (>= 0)
 */
void s3_line_step(s3_line_t * p_ln, uint32_t sensors, uint32_t moved_mm,
                  s3_line_out_t * p_out);

#endif /* S3_LINE_H */
