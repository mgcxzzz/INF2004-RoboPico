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

static inline int32_t sys_clamp(int32_t value, int32_t lo, int32_t hi) {
    return (value < lo) ? lo : ((value > hi) ? hi : value);
}

/** Mask of the low n bits (n = 1..32). */
static inline uint32_t sys_mask(uint32_t bits) {
    return (bits >= 32U) ? 0xFFFFFFFFU : ((1UL << bits) - 1UL);
}

/** Saturate an unsigned value to the largest that fits in `bits` bits. */
static inline uint32_t sys_sat_u(uint32_t value, uint32_t bits) {
    uint32_t max = sys_mask(bits);

    return (value > max) ? max : value;
}

static inline uint32_t sys_put_u(uint32_t word, uint32_t shift, uint32_t bits, uint32_t value) {
    uint32_t m = sys_mask(bits);

    return (word & ~(m << shift)) | ((value & m) << shift);
}

static inline uint32_t sys_get_u(uint32_t word, uint32_t shift, uint32_t bits) {
    return (word >> shift) & sys_mask(bits);
}

/** Insert a signed (two's complement) field. Clamp it first. */
static inline uint32_t sys_put_s(uint32_t word, uint32_t shift, uint32_t bits, int32_t value) {
    return sys_put_u(word, shift, bits, (uint32_t)value);
}

/** Extract a signed field and restore its sign (sign extension). */
static inline int32_t sys_get_s(uint32_t word, uint32_t shift, uint32_t bits) {
    uint32_t raw = sys_get_u(word, shift, bits);
    uint32_t sign = 1UL << (bits - 1U);

    return (int32_t)(raw ^ sign) - (int32_t)sign;
}

static inline void sys_put_le32(uint8_t *p, uint32_t word) {
    p[0] = (uint8_t)(word & 0xFFU);
    p[1] = (uint8_t)((word >> 8) & 0xFFU);
    p[2] = (uint8_t)((word >> 16) & 0xFFU);
    p[3] = (uint8_t)((word >> 24) & 0xFFU);
}

static inline uint32_t sys_get_le32(uint8_t const *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------------------
 * Common header
 * --------------------------------------------------------------------- */
#define TLM_VERSION (2U) /* 1 = Buddy 5's first frame, no header */
#define TLM_HDR_BYTES (3U)

/* Sources (4 bits). 0 is never sent. */
#define TLM_SRC_WIFI (1U)        /* S1 WiFi and MQTT       (below)     */
#define TLM_SRC_MOTION (2U)      /* S2 Motion Control      (to define) */
#define TLM_SRC_LINE (3U)        /* S3 Line and Barcode    (below)     */
#define TLM_SRC_IMU (4U)         /* S4 IMU Terrain         (to define) */
#define TLM_SRC_OBSTACLE (5U)    /* S5 Obstacle Scan       (below)     */
#define TLM_SRC_NAV (6U)         /* Navigation Coordinator (below)     */
#define TLM_SRC_SAFETY (7U)      /* Safety Supervisor      (below)     */
#define TLM_SRC_BARCODE_MAP (8U) /* S1 configuration readback          */

/** Write the 3-byte header at the start of a frame. */
static inline void tlm_put_header(uint8_t *p_frame, uint32_t source, uint32_t payload_bytes,
                                  uint8_t seq) {
    p_frame[0] = (uint8_t)((TLM_VERSION & 0xFU) | ((source & 0xFU) << 4));
    p_frame[1] = (uint8_t)sys_sat_u(payload_bytes, 8U);
    p_frame[2] = seq;
}

/**
 * Check a received frame's header: right version, the expected source
 * and frame length = header + payload length. Returns its sequence number
 * through p_seq. A frame that fails is not decoded at all.
 */
static inline bool tlm_check_header(uint8_t const *p_frame, uint32_t frame_len, uint32_t source,
                                    uint32_t payload_bytes, uint8_t *p_seq) {
    if ((frame_len != (TLM_HDR_BYTES + payload_bytes)) || ((p_frame[0] & 0xFU) != TLM_VERSION) ||
        (((uint32_t)p_frame[0] >> 4) != source) || ((uint32_t)p_frame[1] != payload_bytes)) {
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
#define B5_TLM_PAYLOAD_BYTES (16U)
#define B5_TLM_FRAME_BYTES (TLM_HDR_BYTES + B5_TLM_PAYLOAD_BYTES)

typedef struct {
    uint8_t seq;
    uint8_t state;
    uint8_t fault;
    uint8_t side;
    uint8_t blocked_tries;
    uint8_t recover_tries;
    int16_t servo_dd;
    bool b_found;
    uint16_t range_mm;
    uint16_t closest_mm;
    uint8_t confidence_pct;
    uint16_t width_mm;
    uint16_t lateral_mm;
    uint16_t pings;
    uint16_t no_echo;
} b5_tlm_t;

void b5_tlm_pack(b5_tlm_t const *p_in, uint8_t p_frame[B5_TLM_FRAME_BYTES]);
bool b5_tlm_unpack(uint8_t const *p_frame, uint32_t len, b5_tlm_t *p_out);

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
#define NAV_TLM_PAYLOAD_BYTES (6U)
#define NAV_TLM_FRAME_BYTES (TLM_HDR_BYTES + NAV_TLM_PAYLOAD_BYTES)

#define NAV_BITS_STATE (3U)
#define NAV_BITS_OWNER (3U)
#define NAV_BITS_STEP (3U)
#define NAV_BITS_DROPPED (8U)
#define NAV_DROPPED_MAX (255U)

typedef struct {
    uint8_t seq;
    uint8_t state; /* nav_state_t */
    uint8_t owner; /* nav_owner_t */
    bool b_slow;
    uint8_t search_step;
    uint8_t dropped;
    uint16_t commands_sent;
} nav_tlm_t;

void nav_tlm_pack(nav_tlm_t const *p_in, uint8_t p_frame[NAV_TLM_FRAME_BYTES]);
bool nav_tlm_unpack(uint8_t const *p_frame, uint32_t len, nav_tlm_t *p_out);

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
#define SAFETY_TLM_FRAME_BYTES (TLM_HDR_BYTES + SAFETY_TLM_PAYLOAD_BYTES)

#define SAFETY_BITS_STATE (2U)
#define SAFETY_BITS_SRC (3U)
#define SAFETY_BITS_CODE (8U)
#define SAFETY_BITS_COUNTER (8U)
#define SAFETY_BITS_STOP_MS (10U)
#define SAFETY_BITS_MASK (8U)
#define SAFETY_SRC_NONE (7U) /* "no fault yet" in a src field */
#define SAFETY_STOP_MS_MAX (1023U)

typedef struct {
    uint8_t seq;
    uint8_t state; /* safety_state_t */
    bool b_inhibit;
    uint8_t fault_src; /* safety_src_t, or SAFETY_SRC_NONE */
    uint8_t warn_src;
    uint8_t fault_code;
    uint8_t warn_code;
    uint8_t estops;
    uint8_t faults;
    uint8_t warnings;
    uint8_t slow_stops;
    uint16_t last_stop_ms;
    uint16_t max_stop_ms;
    uint8_t fault_mask; /* SYS_FAULT_* bits */
} safety_tlm_t;

void safety_tlm_pack(safety_tlm_t const *p_in, uint8_t p_frame[SAFETY_TLM_FRAME_BYTES]);
bool safety_tlm_unpack(uint8_t const *p_frame, uint32_t len, safety_tlm_t *p_out);

/* ------------------------------------------------------------------------
 * Source 3: Line and Barcode (Buddy 3). Payload 12 bytes.
 * Packed by s3_tlm_pack() in subsystem3/s3_pack.c.
 *
 *   word 0  bits  0..2   state (s3_state_t, 6 values)
 *           bits  3..5   sensors, 1 = black: bit 3 left, 4 right, 5 barcode
 *           bit   6      line control granted by the coordinator
 *           bits  7..8   side the line is on now (0 none, 1 left, 2 right,
 *                        3 both)
 *           bits  9..14  last barcode character, Code 39 index 0..43
 *                        (0-9, A-Z, - . space $ / + % *), 63 = none
 *           bits 15..17  last decode result (0 ok, 1 bad length, 2 bad
 *                        wide/narrow ratio, 3 bad character, 4 no start or
 *                        stop, 5 backwards, 6 timeout or overflow)
 *           bits 18..20  last command sent (0 none, 1 left, 2 right,
 *                        3 straight, 4 U-turn)
 *           bit  21      last read measured widths by distance (1) or by
 *                        time (0)
 *           bits 22..31  reserved
 *   word 1  bits  0..7   barcodes decoded and sent (wrapping)
 *           bits  8..15  bad reads, no command sent (wrapping)
 *           bits 16..23  backwards reads, no command sent (wrapping)
 *           bits 24..31  junctions (wrapping)
 *   word 2  bits  0..7   line-lost events (wrapping)
 *           bits  8..15  barcode edges lost to a full buffer (wrapping)
 *           bits 16..31  reserved
 * --------------------------------------------------------------------- */
#define S3_TLM_PAYLOAD_BYTES (12U)
#define S3_TLM_FRAME_BYTES (TLM_HDR_BYTES + S3_TLM_PAYLOAD_BYTES)

typedef struct {
    uint8_t seq;
    uint8_t state;
    uint8_t sensors; /* S3_SENSE_* bits */
    bool b_granted;
    uint8_t side;
    uint8_t last_char; /* Code 39 index or 63 */
    uint8_t last_result;
    uint8_t last_cmd;
    bool b_dist;
    uint8_t barcodes_ok;
    uint8_t bad_reads;
    uint8_t backwards;
    uint8_t junctions;
    uint8_t lost;
    uint8_t overflows;
} s3_tlm_t;

void s3_tlm_pack(s3_tlm_t const *p_in, uint8_t p_frame[S3_TLM_FRAME_BYTES]);
bool s3_tlm_unpack(uint8_t const *p_frame, uint32_t len, s3_tlm_t *p_out);

/* ------------------------------------------------------------------------
 * Source 1: WiFi and MQTT (Buddy 1). Payload 12 bytes. Sent as the
 * heartbeat on topic is28/hb about once a second (NFR-WIFI-02), and at
 * once when the operating state changes. The header sequence number counts
 * heartbeats. Packed by b1_tlm_pack() in subsystem1/b1_proto.c.
 *
 *   word 0  bits  0..1   link state (b1_net_t: 0 disconnected,
 *                        1 connecting, 2 operational, 3 reconnect wait)
 *           bits  2..4   operating state (nav_state_t; 7 = not known yet)
 *           bits  5..8   last command code (B1_CMD_*, 0 = none yet)
 *           bits  9..11  last command result (B1_ACK_*, 7 = none yet)
 *           bits 12..25  uptime, s, 0..16383 (saturates, about 4.5 h)
 *           bits 26..31  reserved
 *   word 1  bits  0..15  messages published (wrapping)
 *           bits 16..31  messages lost: queue full or publish failed
 *                        (wrapping)
 *   word 2  bits  0..7   commands accepted (wrapping)
 *           bits  8..15  commands rejected (wrapping)
 *           bits 16..23  MQTT (re)connections (wrapping)
 *           bits 24..30  outgoing queue fill, %, 0..100
 *           bit  31      reserved
 * --------------------------------------------------------------------- */
#define B1_TLM_PAYLOAD_BYTES (12U)
#define B1_TLM_FRAME_BYTES (TLM_HDR_BYTES + B1_TLM_PAYLOAD_BYTES)

#define B1_BITS_NET (2U)
#define B1_BITS_STATE (3U)
#define B1_BITS_CMD (4U)
#define B1_BITS_RESULT (3U)
#define B1_BITS_UPTIME (14U)
#define B1_BITS_FILL (7U)
#define B1_STATE_UNKNOWN (7U) /* operating state not reported yet */
#define B1_RESULT_NONE (7U)   /* no command received yet          */

typedef struct {
    uint8_t seq;
    uint8_t net;         /* b1_net_t */
    uint8_t state;       /* nav_state_t or B1_STATE_UNKNOWN */
    uint8_t last_cmd;    /* B1_CMD_* */
    uint8_t last_result; /* B1_ACK_* or B1_RESULT_NONE */
    uint16_t uptime_s;
    uint16_t published;
    uint16_t lost;
    uint8_t accepted;
    uint8_t rejected;
    uint8_t connects;
    uint8_t fill_pct;
} b1_tlm_t;

void b1_tlm_pack(b1_tlm_t const *p_in, uint8_t p_frame[B1_TLM_FRAME_BYTES]);
bool b1_tlm_unpack(uint8_t const *p_frame, uint32_t len, b1_tlm_t *p_out);

/* ------------------------------------------------------------------------
 * Supervisory commands, PC -> car, topic is28/cmd. No telemetry header.
 * Existing commands stay 4 bytes. BARCODE_MAP is 5 bytes (see below).
 * Checked by b1_cmd_check() (subsystem1/b1_proto.c), FR-WIFI-02/03.
 *
 *   byte 0  bits 0..3  schema version (TLM_VERSION)
 *           bits 4..7  command code (B1_CMD_*)
 *   byte 1             command id, chosen by the PC, echoed in the ack
 *   bytes 2..3         argument, little-endian (B1_CMD_CONFIG only:
 *                      bits 12..15 parameter, bits 0..11 value; else 0)
 *
 * Acknowledgement, car -> PC, topic is28/ack. 4 bytes, no header.
 *
 *   byte 0             byte 0 of the command, echoed
 *   byte 1             command id, echoed
 *   byte 2             result (B1_ACK_*)
 *   byte 3  bits 0..2  operating state when answered (nav_state_t, 7 =
 *                      not known yet); bits 3..7 reserved
 *
 * Steering and navigation commands are never carried out (FR-WIFI-03).
 * They have codes only so a test can send one and see it rejected.
 * --------------------------------------------------------------------- */
#define B1_CMD_BYTES (4U)
#define B1_ACK_BYTES (4U)

#define B1_CMD_NONE (0U)
#define B1_CMD_START (1U) /* Ready -> Running                 */
#define B1_CMD_PAUSE (2U)
#define B1_CMD_RESUME (3U)
#define B1_CMD_STOP (4U)         /* controlled stop (Safe Stop)      */
#define B1_CMD_RESET (5U)        /* back to Ready / clear latch      */
#define B1_CMD_ESTOP (6U)        /* straight to Safety Supervisor    */
#define B1_CMD_CONFIG (7U)       /* configuration update             */
#define B1_CMD_PING (8U)         /* ack + heartbeat at once (latency)*/
#define B1_CMD_BARCODE_MAP (9U)  /* assign four letters; 5 bytes     */
#define B1_CMD_BARCODE_GET (10U) /* read active map; 4 bytes, arg 0  */
#define B1_CMD_STEER (12U)       /* PROHIBITED: remote steering      */
#define B1_CMD_DRIVE (13U)       /* PROHIBITED: remote driving       */
#define B1_CMD_TURN (14U)        /* PROHIBITED: remote turning       */

#define B1_ACK_ACCEPTED (0U)    /* checked and passed on            */
#define B1_ACK_PROHIBITED (1U)  /* steering/navigation (FR-WIFI-03) */
#define B1_ACK_UNKNOWN (2U)     /* not a command code               */
#define B1_ACK_FORMAT (3U)      /* wrong length or schema version   */
#define B1_ACK_BUSY (4U)        /* valid, but could not be queued   */
#define B1_ACK_UNSUPPORTED (5U) /* configuration parameter unknown  */

/* Configuration parameters (B1_CMD_CONFIG, argument bits 12..15). */
#define B1_CFG_TELEMETRY                                                                           \
    (1U) /* value 0 = pause other buddies'
                                         * frames, 1 = forward them       */

/* Barcode mapping, configured before the demo through is28/cmd.
 * BARCODE_MAP: [version | 9<<4, command id, map low, map mid, map high].
 * Map bits 0..4 = FORWARD, 5..9 = UTURN, 10..14 = LEFT, 15..19 = RIGHT.
 * Each field is a letter index A=0..Z=25; bits 20..23 MUST be zero.
 * All four letters must be distinct. One command replaces the whole map.
 * BARCODE_GET uses the existing 4-byte command layout with argument 0.
 * Both get the ordinary 4-byte acknowledgement and a source-8 readback
 * on is28/tlm, including when ordinary telemetry forwarding is off.
 * No mapping is configured at boot; it is RAM-only and survives RESET,
 * but must be sent again after power loss. No default letters are used.
 *
 * Source 8 payload (5 bytes):
 *   word 0 bits 0..19 map as above (zero if not configured)
 *          bit 20     configured
 *          bits 21..28 revision, wrapping; increments on a changed map
 *          bits 29..31 reserved, zero
 *   byte 4           request id (same as the MAP or GET command)
 * Common-header sequence counts readbacks, not telemetry samples.
 * This reports configuration only, not a pending or executed movement.
 */
#define B1_MAP_CMD_BYTES (5U)
#define B1_MAP_PAYLOAD_BYTES (5U)
#define B1_MAP_FRAME_BYTES (TLM_HDR_BYTES + B1_MAP_PAYLOAD_BYTES)
#define B1_MAP_BITS (20U)
#define B1_MAP_MASK (0x000FFFFFUL)
#define B1_MAP_SLOTS (4U)

/* ------------------------------------------------------------------------
 * Sources 2 and 4 (Buddies 2 and 4): add your payload sections here, in
 * the same style, before your first frame goes out.
 * --------------------------------------------------------------------- */

/* Passive encoder + side-ramp snapshot: 20 sensing + 4 diagnostic bytes.
 * byte0 flags: bit0 fresh valid wheel odometry, bit1 valid side estimate,
 * bit2 one-sided mode, bit3 passive sensing; byte1 reserved zero.
 * int16 LE at offsets2/4/6: left/right/centre speed mm/s; offset8 roll dd;
 * offset10 signed right-minus-left drive-wheel height mm. Offset12 int32
 * signed NET centre distance mm (not accumulated absolute circuit travel).
 * uint16 LE at16/18: left/right invalid encoder transitions, saturated.
 * Validity is independent: side inclination can be available at rest even
 * if wheel odometry is not. Receiver also applies its arrival-age limit.
 */
#define B2_TLM_LEGACY_PAYLOAD_BYTES (20U)
#define B2_TLM_REASON_PAYLOAD_BYTES (24U)
#define B2_TLM_NUMERIC_PAYLOAD_BYTES (32U)
#define B2_TLM_ROBUST_PAYLOAD_BYTES (36U) /* v0.5 */
#define B2_TLM_PAYLOAD_BYTES (37U)        /* v0.6: 40-byte frame = B1_OUT_MAX, no spare room */
/* v0.6 per-axis MAD bytes: ceil(mg / 4), saturating at 255 (>= 1020 mg). */
#define B2_TLM_AXIS_MAD_LSB_MG (4U)
#define B2_TLM_FRAME_BYTES (TLM_HDR_BYTES + B2_TLM_PAYLOAD_BYTES)
#define B2_TLM_ODO_VALID (1U)
#define B2_TLM_SIDE_VALID (2U)
#define B2_TLM_SIDE_MODE (4U)
#define B2_TLM_PASSIVE (8U)
typedef struct {
    uint8_t seq, flags;
    int16_t left_mm_s, right_mm_s, centre_mm_s;
    int16_t roll_dd, side_height_mm;
    int32_t distance_mm;
    uint16_t invalid_left, invalid_right;
    /* Extension: exclusions ORed since previous source2 publication, current
     * ramp state (level/candidate/rising/returning bits0..3), interval counts. */
    uint8_t quality_reasons, ramp_state, quality_bad_samples, quality_total_samples;
    /* v0.4 interval maxima (not simultaneous), absolute rates. Offset30:
     * number of ticks with refreshed rates; offset31 reserved zero. */
    uint16_t max_spread_mg, max_accel_mm_s2, max_turn_dd_s;
    uint8_t rate_samples;
    /* v0.5: offset32 uint16 maximum axis MAD mg; offset34 minimum FIFO
     * sample count in the interval; offset35 reserved zero. */
    uint16_t max_mad_mg;
    uint8_t min_fifo_samples;
    /* v0.6 (37-byte payload only): interval maximum MAD of each axis,
     * fwd/lat/up, at offsets31/35/36 (bytes reserved zero in 32/36-byte
     * payloads). Each byte = ceil(mg/4), saturating at255; unpacked as
     * byte*4 mg, an upper bound within 3 mg. Exceeding the40mg guard is
     * preserved exactly (41 mg -> 11 -> 44 mg; 40 mg -> 10 -> 40 mg). */
    uint16_t max_axis_mad_mg[3];
} b2_tlm_t;
void b2_tlm_pack(b2_tlm_t const *in, uint8_t frame[B2_TLM_FRAME_BYTES]);
bool b2_tlm_unpack(uint8_t const *frame, uint32_t len, b2_tlm_t *out);

#endif /* SYS_TLM_H */
