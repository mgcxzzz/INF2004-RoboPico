/**
 * @file    s3_reader.h
 * @brief   Read Barcode: turn timestamped GP19 edges into bar and space
 *          widths, measured in distance travelled (FR-LINE-03).
 *
 * Week 6 feedback: "calculate how many encoder-distance samples a narrow
 * bar gives at nominal speed; if few, use edge interrupts on GP19 with
 * encoder-count timestamps". At 150 mm/s and a 10 ms task a 3 mm narrow
 * bar gets only 2 samples (1.5 mm print: 1), so polling is not enough.
 * Instead the interrupt timestamps every edge in microseconds, and the
 * task places each edge on the distance axis by interpolating between
 * the motion-estimate readings it takes every cycle:
 *
 *     d(edge) = d0 + (d1 - d0) * (t_edge - t0) / (t1 - t0)
 *
 * So a bar reads the same width at any speed, even if the speed changes
 * during the barcode. If Motion Control gives no distance (or the car
 * hardly moved), the reader falls back to the edge times themselves.
 *
 * Pure C, tested on the PC (test_s3.c).
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#ifndef S3_READER_H
#define S3_READER_H

#include <stdbool.h>
#include <stdint.h>

#include "s3_barcode.h"
#include "s3_config.h"

#define S3_RD_MAX_EDGES     (S3_BC_MAX_ELEMENTS + 1U)   /* 30 */
#define S3_RD_Q             (16)        /* distances in mm x 16 */

typedef enum
{
    S3_RD_IDLE = 0,         /* waiting for the first bar               */
    S3_RD_READING,          /* edges arriving                          */
    S3_RD_READY,            /* quiet after the last bar: decode now    */
    S3_RD_TIMEOUT,          /* read took too long                      */
    S3_RD_OVERFLOW          /* more edges than any valid barcode       */
} s3_rd_status_t;

/* Reader record: owned by the Line and Barcode task only. Edge positions
 * are kept relative to the first edge: distance in 16 bits (mm x 16, up
 * to 4095 mm, saturating), time in 32 bits (timestamps stay 32-bit). */
typedef struct
{
    uint32_t t0_us;             /* previous anchor                     */
    int32_t  d0_q;
    uint32_t t1_us;             /* latest anchor                       */
    int32_t  d1_q;
    uint32_t start_t_us;        /* first edge                          */
    int32_t  start_d_q;
    uint32_t edge_t[S3_RD_MAX_EDGES];   /* us since the first edge     */
    uint16_t edge_d[S3_RD_MAX_EDGES];   /* mm x 16 since the first edge */
    uint32_t count     : 6;     /* edges stored, 0..30                 */
    uint32_t b_black   : 1;     /* level after the last edge           */
    uint32_t b_overflow: 1;
    uint32_t b_anchored: 1;     /* at least one anchor taken           */
    uint32_t b_dist_ok : 1;     /* latest anchor came from Motion Control */
} s3_reader_t;

void s3_rd_reset(s3_reader_t * p_rd);

/** Every cycle: where the car is now (odometer, mm x 16) and when.
 *  b_dist_ok = the distance is real (Motion Control); if not, the end of
 *  a barcode is judged by time instead. */
void s3_rd_anchor(s3_reader_t * p_rd, uint32_t t_us, int32_t d_q,
                  bool b_dist_ok);

/** One GP19 edge, oldest first. b_black = level after the edge. */
void s3_rd_edge(s3_reader_t * p_rd, uint32_t t_us, bool b_black);

/** Status after this cycle's edges (uses the latest anchor as "now"). */
s3_rd_status_t s3_rd_poll(s3_reader_t const * p_rd);

/**
 * Element widths, bar first: by distance (mm x 16) if b_dist, else by
 * time (us). Returns the count, 0 if the car hardly moved over the
 * barcode and distance widths were asked for.
 */
uint32_t s3_rd_widths(s3_reader_t const * p_rd, uint32_t * p_w,
                      uint32_t w_max, bool b_dist);

/**
 * Decode the finished read: by distance first (immune to speed changes);
 * if that fails (e.g. Motion Control's 1 mm steps are coarse against
 * 1.5 mm bars) or there is no distance, by time. Each attempt has the
 * full Code 39 checks, so a fallback can fail but not guess.
 * @param p_b_dist  receives which measurement gave the result
 */
s3_bc_result_t s3_rd_decode(s3_reader_t const * p_rd, char * p_text,
                            uint32_t text_max, bool * p_b_dist);

#endif /* S3_READER_H */
