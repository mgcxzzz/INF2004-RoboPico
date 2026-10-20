/**
 * @file    s3_barcode.h
 * @brief   Code 39 decoding from measured bar and space widths.
 *
 * Pure C: no kernel, no hardware, so it is tested on the PC
 * (testing/host/test_s3.c) and on the Pico (subsystem3_test.c).
 *
 * A Code 39 character is 9 elements, bar first: 5 bars and 4 spaces, of
 * which exactly 3 are wide. Characters are separated by a narrow gap, and
 * every barcode starts and ends with '*'. So "*A*" is 3 x 9 + 2 = 29
 * elements. Widths may be in any unit (mm x 16 or microseconds): only
 * their ratios matter, so the read does not depend on the car's speed.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#ifndef S3_BARCODE_H
#define S3_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_C39_ELEMENTS         (9U)
#define S3_C39_CHARS            (44U)   /* 0-9, A-Z, - . space $ / + % *  */
#define S3_C39_NONE             (63U)   /* "no character" in a 6-bit field */

/* Result of one decode (3 bits in telemetry). */
typedef enum
{
    S3_BC_OK = 0,           /* valid "*...*", text returned              */
    S3_BC_BAD_LENGTH,       /* element count is not 10 x chars - 1       */
    S3_BC_BAD_RATIO,        /* wide and narrow not clearly different     */
    S3_BC_BAD_CHAR,         /* a 9-element group is not a Code 39 char   */
    S3_BC_NO_START_STOP,    /* first or last character is not '*'        */
    S3_BC_BACKWARDS,        /* valid only when read in reverse           */
    S3_BC_RESULT_COUNT
} s3_bc_result_t;

/** Code 39 pattern of a character (bit 8 = first element, 1 = wide),
 *  or 0 if it is not a Code 39 character. */
uint32_t s3_c39_pattern(char c);

/** Character for a 9-bit pattern, or '\0' if none. */
char     s3_c39_char(uint32_t pattern);

/** Index 0..43 of a character in the Code 39 set, or S3_C39_NONE. */
uint32_t s3_c39_index(char c);

/** Character at an index, or '?' for S3_C39_NONE / out of range. */
char     s3_c39_at(uint32_t index);

/**
 * Decode element widths, bar first, read in the direction of travel.
 * @param p_w      widths (any unit), n of them
 * @param p_text   receives the characters between the '*'s, '\0'-ended
 * @param text_max size of p_text (at least S3_BC_MAX_CHARS)
 * A barcode that only decodes reversed returns S3_BC_BACKWARDS and no
 * text: the team decided a backwards read gives no command.
 */
s3_bc_result_t s3_bc_decode(uint32_t const * p_w, uint32_t n,
                            char * p_text, uint32_t text_max);

/** Write the element widths of "*text*" (narrow, wide units) into p_w.
 *  Used by the tests and the bench self-check. Returns the count, 0 if
 *  it does not fit or a character is not Code 39. */
uint32_t s3_bc_encode(char const * p_text, uint32_t narrow, uint32_t wide,
                      uint32_t * p_w, uint32_t w_max);

#endif /* S3_BARCODE_H */
