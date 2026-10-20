/**
 * @file    sys_tlm.h
 * @brief   IS28 telemetry schema: the ONE header every buddy uses for the
 *          compact binary MQTT messages (Tribunal 1, minimum-bit data).
 *
 * Tribunal 1 decisions this file implements (IS28_ImmTri1, Version B):
 *  - "one standard packet format shared by all devices, with consistent
 *    bit layout and scaling rules" (Buddy 1);
 *  - "define common field positions, widths, signs and scaling; all
 *    producers and the MQTT receiver follow the schema";
 *  - R3: "publish the struct in one shared header used by all buddies
 *    instead of duplicated copies";
 *  - R4: "check or cap every value before packing it".
 *
 * ---------------------------------------------------------------------
 * Rules for every frame
 * ---------------------------------------------------------------------
 *  1. A frame is a 3-byte common header followed by the payload.
 *  2. Multi-byte values and 32-bit payload words are little-endian.
 *     Inside a word, bit 0 is the least significant bit.
 *  3. Each field has a fixed position and width, listed below. Units are
 *     part of the field: mm, mm/s, 0.1 degree (dd), ms, %. Angles and
 *     headings are + = RIGHT.
 *  4. Signed fields are two's complement in their own width; the receiver
 *     sign-extends them (sys_get_s()).
 *  5. Every value is clamped (or saturated) to its field BEFORE packing,
 *     never wrapped, except counters marked "wrapping", which the
 *     receiver only ever compares by difference.
 *  6. Reserved bits are sent as 0 and ignored by the receiver.
 *  7. TLM_VERSION changes only when an existing layout changes in an
 *     incompatible way. Adding a new source does not change it.
 *  8. The packing is done with masks and shifts (never C bit-fields,
 *     whose layout is up to the compiler), so the PC decodes exactly what
 *     the Pico encoded.
 *
 * Common header (3 bytes)
 *   byte 0  bits 0..3  schema version (TLM_VERSION)
 *           bits 4..7  source (TLM_SRC_*)
 *   byte 1             payload length in bytes (0..255)
 *   byte 2             sequence number, per source, wrapping 0..255. The
 *                      receiver counts gaps for the >= 95 % delivery test.
 *
 * A receiver checks the version, the source and that the frame length is
 * 3 + payload length, then decodes the payload for that source.
 *
 * To add your subsystem: pick your TLM_SRC_* below, add a payload section
 * like the ones that follow (positions, widths, units, signedness), a
 * decoded-view struct and pack/unpack prototypes. Never re-define a frame
 * somewhere else.
 *
 * Group IS28 (team). Coding standard: Barr-C:2018.
 */

#ifndef SYS_TLM_H
#define SYS_TLM_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------------
 * Generic helpers: clamp, then mask and shift
 * --------------------------------------------------------------------- */

static inline int32_t
sys_clamp(int32_t value, int32_t lo, int32_t hi)
{
    return (value < lo) ? lo : ((value > hi) ? hi : value);
}

/** Mask of the low n bits (n = 1..32). */
static inline uint32_t
sys_mask(uint32_t bits)
{
    return (bits >= 32U) ? 0xFFFFFFFFU : ((1UL << bits) - 1UL);
}

/** Saturate an unsigned value to the largest that fits in `bits` bits. */
static inline uint32_t
sys_sat_u(uint32_t value, uint32_t bits)
{
    uint32_t max = sys_mask(bits);

    return (value > max) ? max : value;
}

static inline uint32_t
sys_put_u(uint32_t word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t m = sys_mask(bits);

    return (word & ~(m << shift)) | ((value & m) << shift);
}

static inline uint32_t
sys_get_u(uint32_t word, uint32_t shift, uint32_t bits)
{
    return (word >> shift) & sys_mask(bits);
}

/** Insert a signed (two's complement) field. Clamp it first. */
static inline uint32_t
sys_put_s(uint32_t word, uint32_t shift, uint32_t bits, int32_t value)
{
    return sys_put_u(word, shift, bits, (uint32_t)value);
}

/** Extract a signed field and restore its sign (sign extension). */
static inline int32_t
sys_get_s(uint32_t word, uint32_t shift, uint32_t bits)
{
    uint32_t raw  = sys_get_u(word, shift, bits);
    uint32_t sign = 1UL << (bits - 1U);

    return (int32_t)(raw ^ sign) - (int32_t)sign;
}

static inline void
sys_put_le32(uint8_t * p, uint32_t word)
{
    p[0] = (uint8_t)(word & 0xFFU);
    p[1] = (uint8_t)((word >> 8) & 0xFFU);
    p[2] = (uint8_t)((word >> 16) & 0xFFU);
    p[3] = (uint8_t)((word >> 24) & 0xFFU);
}

static inline uint32_t
sys_get_le32(uint8_t const * p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------------------
 * Common header
 * --------------------------------------------------------------------- */
#define TLM_VERSION             (2U)    /* 1 = Buddy 5's first frame, no header */
#define TLM_HDR_BYTES           (3U)

/* Sources (4 bits). 0 is never sent. */
#define TLM_SRC_WIFI            (1U)    /* S1 WiFi and MQTT       (to define) */
#define TLM_SRC_MOTION          (2U)    /* S2 Motion Control      (to define) */
#define TLM_SRC_LINE            (3U)    /* S3 Line and Barcode    (to define) */
#define TLM_SRC_IMU             (4U)    /* S4 IMU Terrain         (to define) */
#define TLM_SRC_OBSTACLE        (5U)    /* S5 Obstacle Scan       (below)     */
#define TLM_SRC_NAV             (6U)    /* Navigation Coordinator (below)     */
#define TLM_SRC_SAFETY          (7U)    /* Safety Supervisor      (below)     */

/** Write the 3-byte header at the start of a frame. */
static inline void
tlm_put_header(uint8_t * p_frame, uint32_t source, uint32_t payload_bytes,
               uint8_t seq)
{
    p_frame[0] = (uint8_t)((TLM_VERSION & 0xFU) | ((source & 0xFU) << 4));
    p_frame[1] = (uint8_t)sys_sat_u(payload_bytes, 8U);
    p_frame[2] = seq;
}

/**
 * Check a received frame's header: right version, the expected source
 * and frame length = header + payload length. Returns its sequence number
 * through p_seq. A frame that fails is not decoded at all.
 */
static inline bool
tlm_check_header(uint8_t const * p_frame, uint32_t frame_len,
                 uint32_t source, uint32_t payload_bytes, uint8_t * p_seq)
{
    if ((frame_len != (TLM_HDR_BYTES + payload_bytes)) ||
        ((p_frame[0] & 0xFU) != TLM_VERSION) ||
        (((uint32_t)p_frame[0] >> 4) != source) ||
        ((uint32_t)p_frame[1] != payload_bytes))
    {
        return false;
    }

    *p_seq = p_frame[2];
    return true;
}

/* ------------------------------------------------------------------------
 * Source 5: Obstacle Scan and Recovery (Buddy 5). Payload 16 bytes.
 * Packed by b5_tlm_pack() in subsystem5/b5_pack.c.
 *
 *   word 0  bits  0..3   state (b5_state_t, 11 values)
 *           bits  4..6   fault code (b5_fault_t, 7 values)
 *           bits  7..8   bypass side (0 none, 1 right, 2 left)
 *           bits  9..10  blocked tries (0..3)
 *           bits 11..12  recovery tries (0..3)
 *           bits 13..23  servo angle, SIGNED, dd, -900..+900 (+ = right)
 *           bit  24      obstacle found in the last Planning
 *           bits 25..31  reserved
 *   word 1  bits  0..11  last guard range, mm, 0..4095 (clamped)
 *           bits 12..23  closest obstacle range, mm, 0..4095
 *           bits 24..30  confidence, %, 0..100
 *           bit  31      reserved
 *   word 2  bits  0..11  corrected obstacle width, mm, 0..4095
 *           bits 12..23  bypass sideways shift, mm, 0..4095
 *           bits 24..31  reserved
 *   word 3  bits  0..15  pings sent (wrapping)
 *           bits 16..31  pings with no echo (wrapping)
 * --------------------------------------------------------------------- */
#define B5_TLM_PAYLOAD_BYTES    (16U)
#define B5_TLM_FRAME_BYTES      (TLM_HDR_BYTES + B5_TLM_PAYLOAD_BYTES)

typedef struct
{
    uint8_t  seq;
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
bool b5_tlm_unpack(uint8_t const * p_frame, uint32_t len, b5_tlm_t * p_out);

/* ------------------------------------------------------------------------
 * Source 6: Navigation Coordinator. Payload 6 bytes.
 *
 *   word 0  bits  0..2   operating state (nav_state_t, 7 values)
 *           bits  3..5   motion owner (nav_owner_t, 5 values)
 *           bit   6      reduced speed requested by Buddy 5
 *           bits  7..9   line-search step (0 = not searching)
 *           bits 10..17  messages dropped (saturates at 255)
 *           bits 18..31  reserved
 *   bytes 4..5           commands sent to Motion Control (wrapping)
 * --------------------------------------------------------------------- */
#define NAV_TLM_PAYLOAD_BYTES   (6U)
#define NAV_TLM_FRAME_BYTES     (TLM_HDR_BYTES + NAV_TLM_PAYLOAD_BYTES)

#define NAV_BITS_STATE          (3U)
#define NAV_BITS_OWNER          (3U)
#define NAV_BITS_STEP           (3U)
#define NAV_BITS_DROPPED        (8U)
#define NAV_DROPPED_MAX         (255U)

typedef struct
{
    uint8_t  seq;
    uint8_t  state;             /* nav_state_t */
    uint8_t  owner;             /* nav_owner_t */
    bool     b_slow;
    uint8_t  search_step;
    uint8_t  dropped;
    uint16_t commands_sent;
} nav_tlm_t;

void nav_tlm_pack(nav_tlm_t const * p_in,
                  uint8_t p_frame[NAV_TLM_FRAME_BYTES]);
bool nav_tlm_unpack(uint8_t const * p_frame, uint32_t len, nav_tlm_t * p_out);

/* ------------------------------------------------------------------------
 * Source 7: Safety Supervisor. Payload 12 bytes.
 *
 *   word 0  bits  0..1   supervisor state (safety_state_t, 3 values)
 *           bit   2      motor drive inhibited
 *           bits  3..5   source of the last critical fault (7 = none)
 *           bits  6..8   source of the last warning (7 = none)
 *           bits  9..16  last fault code (8 bits, the source's own code)
 *           bits 17..24  last warning code
 *           bits 25..31  reserved
 *   word 1  bits  0..7   emergency stops since boot (wrapping)
 *           bits  8..15  critical faults since boot (wrapping)
 *           bits 16..23  warnings since boot (wrapping)
 *           bits 24..31  stops slower than 100 ms (wrapping)
 *   word 2  bits  0..9   last stop, accept -> drive off, ms (saturates at
 *                        1023 = "far too slow")
 *           bits 10..19  worst stop so far, ms (saturates at 1023)
 *           bits 20..27  shared fault mask (sys_fault.h, SYS_FAULT_*)
 *           bits 28..31  reserved
 * --------------------------------------------------------------------- */
#define SAFETY_TLM_PAYLOAD_BYTES (12U)
#define SAFETY_TLM_FRAME_BYTES  (TLM_HDR_BYTES + SAFETY_TLM_PAYLOAD_BYTES)

#define SAFETY_BITS_STATE       (2U)
#define SAFETY_BITS_SRC         (3U)
#define SAFETY_BITS_CODE        (8U)
#define SAFETY_BITS_COUNTER     (8U)
#define SAFETY_BITS_STOP_MS     (10U)
#define SAFETY_BITS_MASK        (8U)
#define SAFETY_SRC_NONE         (7U)    /* "no fault yet" in a src field */
#define SAFETY_STOP_MS_MAX      (1023U)

typedef struct
{
    uint8_t  seq;
    uint8_t  state;             /* safety_state_t */
    bool     b_inhibit;
    uint8_t  fault_src;         /* safety_src_t, or SAFETY_SRC_NONE */
    uint8_t  warn_src;
    uint8_t  fault_code;
    uint8_t  warn_code;
    uint8_t  estops;
    uint8_t  faults;
    uint8_t  warnings;
    uint8_t  slow_stops;
    uint16_t last_stop_ms;
    uint16_t max_stop_ms;
    uint8_t  fault_mask;        /* SYS_FAULT_* bits */
} safety_tlm_t;

void safety_tlm_pack(safety_tlm_t const * p_in,
                     uint8_t p_frame[SAFETY_TLM_FRAME_BYTES]);
bool safety_tlm_unpack(uint8_t const * p_frame, uint32_t len,
                       safety_tlm_t * p_out);

/* ------------------------------------------------------------------------
 * Sources 1..4 (Buddies 1..4): add your payload sections here, in the
 * same style, before your first frame goes out.
 * --------------------------------------------------------------------- */

#endif /* SYS_TLM_H */
