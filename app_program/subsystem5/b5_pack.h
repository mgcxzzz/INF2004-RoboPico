/**
 * @file    b5_pack.h
 * @brief   Minimum-bit data representation for Buddy 5 (Tribunal 1 change).
 *
 * Every Buddy 5 value is stored in only the bits its valid range needs:
 *
 *   value                     valid range              bits
 *   ------------------------  -----------------------  ----------------
 *   state                     11 states                4
 *   fault code                7 codes                  3
 *   bypass side               none / right / left      2
 *   small counters            0..3                     2
 *   servo / edge angle        -900..+900 (0.1 deg)     11 signed
 *   range, widths, distances  0..4000 mm               12
 *   lateral offsets           -2048..+2047 mm          12 signed
 *   dead-reckoning offsets    -8192..+8191 mm          14 signed
 *   confidence                0..100 %                 7
 *   echo width                0..65535 us              16
 *   ping / no-echo counters   wrapping                 16
 *
 * Two ways of packing are used, on purpose:
 *
 *  - C bit-fields for data owned by the obstacle task (scan table, task
 *    context, status record). Only one task ever writes them, and other
 *    tasks only see a copy taken under a mutex, so the read-modify-write
 *    a bit-field store does can never race.
 *
 *  - Explicit masks and shifts (this file) for data that crosses a
 *    boundary where bit-field layout or a race would matter: the word the
 *    echo interrupt publishes, and the binary telemetry frame that goes
 *    to MQTT and is decoded on a PC. The layout here is fixed by the code,
 *    not by the compiler, and the frame is little-endian.
 *
 * Values are always clamped before packing, so an out-of-range value can
 * never wrap round to a small, believable one (e.g. 4396 mm in 12 bits
 * would otherwise read as 300 mm, exactly the stop distance).
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#ifndef B5_PACK_H
#define B5_PACK_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------------
 * Field widths and limits
 * --------------------------------------------------------------------- */
#define B5_BITS_STATE       (4U)
#define B5_BITS_FAULT       (3U)
#define B5_BITS_SIDE        (2U)
#define B5_BITS_COUNT       (2U)
#define B5_BITS_ANGLE       (11U)
#define B5_BITS_MM          (12U)
#define B5_BITS_PCT         (7U)
#define B5_BITS_ECHO_US     (16U)
#define B5_BITS_SEQ         (8U)
#define B5_BITS_COUNTER     (16U)

#define B5_ANGLE_MIN_DD     (-900)      /* full left                    */
#define B5_ANGLE_MAX_DD     (900)       /* full right                   */
#define B5_MM_MAX           (4095)      /* 12-bit distance ceiling      */
#define B5_SMM_MIN          (-2048)     /* 12-bit signed distance       */
#define B5_SMM_MAX          (2047)
#define B5_POSE_MIN_MM      (-8192)     /* 14-bit signed dead reckoning */
#define B5_POSE_MAX_MM      (8191)
#define B5_COUNT_MAX        (3U)
#define B5_ECHO_US_MAX      (65535U)

/* ------------------------------------------------------------------------
 * Generic helpers: clamp, then mask and shift
 * --------------------------------------------------------------------- */

/** Clamp a signed value into [lo, hi]. */
static inline int32_t
b5_clamp(int32_t value, int32_t lo, int32_t hi)
{
    return (value < lo) ? lo : ((value > hi) ? hi : value);
}

/** Mask of the low n bits (n = 1..32). */
static inline uint32_t
b5_mask(uint32_t bits)
{
    return (bits >= 32U) ? 0xFFFFFFFFU : ((1UL << bits) - 1UL);
}

/** Insert an unsigned field of `bits` bits at `shift` into a word. */
static inline uint32_t
b5_put_u(uint32_t word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t m = b5_mask(bits);

    return (word & ~(m << shift)) | ((value & m) << shift);
}

/** Extract an unsigned field. */
static inline uint32_t
b5_get_u(uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & b5_mask(bits);
}

/** Insert a signed (two's complement) field. */
static inline uint32_t
b5_put_s(uint32_t word, uint32_t shift, uint32_t bits, int32_t value)
{
    return b5_put_u(word, shift, bits, (uint32_t)value);
}

/** Extract a signed field and restore its sign (sign extension). */
static inline int32_t
b5_get_s(uint32_t word, uint32_t shift, uint32_t bits)
{
    uint32_t raw  = b5_get_u(word, shift, bits);
    uint32_t sign = 1UL << (bits - 1U);

    return (int32_t)(raw ^ sign) - (int32_t)sign;
}

/* ------------------------------------------------------------------------
 * Echo result word (written only by the echo interrupt)
 *
 *   bits  0..15  echo width in us (saturates at 65535)
 *   bits 16..23  ping sequence number the echo belongs to
 *   bit  24      VALID     a complete rise/fall pair was measured
 *   bit  25      SATURATED the width did not fit in 16 bits
 *
 * The interrupt builds the whole word, then stores it with one aligned
 * 32-bit write, which is atomic on the Cortex-M0+. The task reads it with
 * one load, so it can never see a new width with an old sequence number.
 * --------------------------------------------------------------------- */
#define B5_ECHO_SHIFT_US    (0U)
#define B5_ECHO_SHIFT_SEQ   (16U)
#define B5_ECHO_FLAG_VALID  (1UL << 24)
#define B5_ECHO_FLAG_SAT    (1UL << 25)

static inline uint32_t
b5_echo_pack(uint32_t width_us, uint8_t seq)
{
    uint32_t word  = B5_ECHO_FLAG_VALID;

    if (width_us > B5_ECHO_US_MAX)
    {
        width_us = B5_ECHO_US_MAX;
        word    |= B5_ECHO_FLAG_SAT;
    }

    word = b5_put_u(word, B5_ECHO_SHIFT_US, B5_BITS_ECHO_US, width_us);
    word = b5_put_u(word, B5_ECHO_SHIFT_SEQ, B5_BITS_SEQ, seq);

    return word;
}

static inline uint32_t
b5_echo_width_us(uint32_t word)
{
    return b5_get_u(word, B5_ECHO_SHIFT_US, B5_BITS_ECHO_US);
}

static inline uint8_t
b5_echo_seq(uint32_t word)
{
    return (uint8_t)b5_get_u(word, B5_ECHO_SHIFT_SEQ, B5_BITS_SEQ);
}

/* ------------------------------------------------------------------------
 * Binary telemetry frame (Buddy 5 -> Buddy 1 -> MQTT -> PC)
 *
 * 16 bytes, little-endian, four 32-bit words:
 *
 *   word 0  bits  0..3   format version (B5_TLM_VERSION)
 *           bits  4..7   state
 *           bits  8..10  fault code
 *           bits 11..12  bypass side
 *           bits 13..14  blocked tries
 *           bits 15..16  recovery tries
 *           bits 17..27  servo angle, signed 0.1 deg (+ = right)
 *           bit  28      obstacle found in the last Planning
 *   word 1  bits  0..11  last guard range, mm
 *           bits 12..23  closest obstacle range, mm
 *           bits 24..30  confidence, %
 *   word 2  bits  0..11  corrected obstacle width, mm
 *           bits 12..23  bypass sideways shift, mm
 *   word 3  bits  0..15  pings sent (wrapping)
 *           bits 16..31  pings with no echo (wrapping)
 *
 * The text the same data needed before was several times larger.
 * --------------------------------------------------------------------- */
#define B5_TLM_VERSION      (1U)
#define B5_TLM_FRAME_BYTES  (16U)

/** Unpacked view of the frame (what a decoder on the PC rebuilds). */
typedef struct
{
    uint8_t  version;
    uint8_t  state;
    uint8_t  fault;
    uint8_t  side;
    uint8_t  blocked_tries;
    uint8_t  recover_tries;
    int16_t  servo_dd;
    bool     b_found;
    uint16_t range_mm;
    uint16_t closest_mm;
    uint8_t  confidence_pct;
    uint16_t width_mm;
    uint16_t lateral_mm;
    uint16_t pings;
    uint16_t no_echo;
} b5_tlm_t;

void b5_tlm_pack(b5_tlm_t const * p_in, uint8_t p_frame[B5_TLM_FRAME_BYTES]);
bool b5_tlm_unpack(uint8_t const p_frame[B5_TLM_FRAME_BYTES],
                   b5_tlm_t * p_out);

#endif /* B5_PACK_H */
