/**
 * @file b5_mission_pack.h
 * @brief Buddy5 mission status, source 15 subtype 0xE5, version 4.
 * @details Car-confirmed snapshot (copied under DI by the Buddy5 task) sent
 * every 500 ms and on stage changes. It repeats the latest width/direction
 * results so a lost one-shot D3/BF frame cannot hide them, and carries the
 * record counts admitted to the network queue so the dashboard can locate a
 * loss (car vs transport). Version 1 (candidate v0.1) had a 54-byte payload
 * with byte 19 zero; version 2 keeps bytes 0-53 and adds the width policy
 * (19) and approach steering (54-58). Version 3 (long width probe, v0.2.12
 * candidate) keeps bytes 0-58 and adds the live width-probe pose (59-60).
 * Version 4 (straight-gate candidate) keeps the 61-byte layout and adds the
 * takeover gate in two formerly zero fields: flags b12-15 and byte 59 b6-7.
 * Frame [0xF2, 61, seq] + payload, LE:
 *  0 u32 Pico ms           4 u8 0xE5             5 u8 version 4
 *  6 u16 mission run       8 u8 stage            9 u8 hold/end reason
 * 10 u8 module end/reason 11 u8 coordinator: owner b0-2, mode b3-5,
 *                               session b6, stopped b7
 * 12 u16 flags (B5MP_F_*)  14 u16 last ping mm (0xFFFF none/no echo)
 * 16 u16 last ping age ms (0xFFFF never; saturates 65534)
 * 18 u8 last ping class b0-1 | why b2-5
 * 19 u8 width policy: 0 none, 1 measured, 2 skipped (clean range outside
 *    270-330 mm, probe not started, no motion), 3 attempted and failed
 *    (HOLD), 4 probe running
 * 20 i16 servo requested 0.1 deg  22 i16 servo applied PWM command 0.1 deg
 * 24 u8 direction decision (0 HOLD 1 LEFT 2 RIGHT)  25 u8 basis
 * 26 u8 state L b0-3 | state R b4-7  27 u8 direction end code
 * 28 u8 side used (0 none 1 right 2 left) 29 u8 width end code
 * 30 u8 width basis  31 u8 width status  32 u8 width reason
 * 33 u8 width retries used
 * 34 u16 width display mm100  36 u16 raw mm100  38 u16 corrected mm100
 *        (0xFFFF = not available)
 * 40 u16 width d0 mm  42 u16 trigger echo mm  44 u16 bypass start range mm
 * 46 u8 bypass state (B5A_*)  47 u8 rejoin phase (B5R_*)
 * 48 u8 width records produced 49 u8 width D1 admitted
 * 50 u8 direction records produced 51 u8 direction BE admitted
 * 52 u8 approach legs b0-3 | alignment search steps b4-7
 * 53 u8 records discarded from a previous run (saturates 255)
 * 54 i8 approach steering setpoint last sent, 0.1 deg (+LEFT, |v| <= 80)
 * 55 u8 largest |setpoint| this mission run, 0.1 deg
 * 56 u8 control frames at the +/-8 deg setpoint bound (saturates 255)
 * 57 u16 fresh range that decided the width policy, mm (0 = none)
 * 59 u8 width probe state b0-3 (B5WP_IDLE..B5WP_MID) | side b4-5 (0 none,
 *    1 left, 2 right; the side of the last stopped sample) | v4: close
 *    echoes counted toward the guard decision b6-7 (0-3; toward takeover
 *    while the gate is open, toward the NOT_STRAIGHT hold while closed)
 * v4: 12 u16 flags b12-15 = takeover gate reason (b5_straight_reason_t:
 *    0 unknown, 1 STRAIGHT (open), 2 settling, 3 stale, 4 unavailable,
 *    5 bend entry, 6 bend, 7 bend exit, 8 curve, 9 junction, 10 turn,
 *    11 line lost/recovery, 12 other)
 * 60 u8 width sample phase b0-2 (B5WP_PH_*) | stop step b3-7 (0-31)
 * @see docs/buddy5_rtos_integration.md
 */
#ifndef B5_MISSION_PACK_H
#define B5_MISSION_PACK_H

#include <stdint.h>
#include <string.h>

enum
{
    B5MP_MARKER = 0xE5U,
    B5MP_VERSION = 4U,
    B5MP_PAYLOAD = 61U,
    B5MP_FRAME = 64U,
    B5MP_NONE = 0xFFFFU
};
_Static_assert(B5MP_FRAME <= 64U, "WIFI_PACKET_MAX is 64");

enum
{
    B5MP_F_ARMED = 1U << 0,      /* mission session armed               */
    B5MP_F_REARM = 1U << 1,      /* waiting for a clear front           */
    B5MP_F_PING_FRESH = 1U << 2, /* last ping <= 250 ms old             */
    B5MP_F_WIDTH_ATTEMPTED = 1U << 3,
    B5MP_F_WIDTH_RESULT = 1U << 4, /* D3 computed in this mission run     */
    B5MP_F_DIRECTION_ATTEMPTED = 1U << 5,
    B5MP_F_DEFAULT_SIDE = 1U << 6, /* side is the labelled default        */
    B5MP_F_BYPASS = 1U << 7,
    B5MP_F_BACKLOG = 1U << 8, /* records still waiting to be sent    */
    B5MP_F_SERVO_SETTLED = 1U << 9,
    B5MP_F_HOLD = 1U << 10,
    B5MP_F_READY = 1U << 11, /* idle start gate satisfied           */
    B5MP_F_MASK = 0x0FFFU
};

typedef struct
{
    uint32_t ms;
    uint16_t run;
    uint8_t stage;
    uint8_t reason;
    uint8_t module_reason;
    uint8_t coordinator;
    uint16_t flags;
    uint16_t ping_mm;
    uint16_t ping_age_ms;
    uint8_t ping_class;
    uint8_t width_policy;
    int16_t servo_requested_dd;
    int16_t servo_applied_dd;
    uint8_t decision;
    uint8_t basis;
    uint8_t states;
    uint8_t direction_end;
    uint8_t side;
    uint8_t width_end;
    uint8_t width_basis;
    uint8_t width_status;
    uint8_t width_reason;
    uint8_t width_retries;
    uint16_t width_display_mm100;
    uint16_t width_raw_mm100;
    uint16_t width_corrected_mm100;
    uint16_t width_d0_mm;
    uint16_t trigger_mm;
    uint16_t bypass_range_mm;
    uint8_t bypass_state;
    uint8_t rejoin_phase;
    uint8_t width_produced;
    uint8_t width_admitted;
    uint8_t direction_produced;
    uint8_t direction_admitted;
    uint8_t approach;
    uint8_t discarded;
    int8_t steer_decideg;
    uint8_t steer_max_decideg;
    uint8_t steer_saturated;
    uint16_t width_gate_mm;
    uint8_t width_state;  /* b0-3 probe state | b4-5 sample side */
    uint8_t width_sample; /* b0-2 sample phase | b3-7 step */
    uint8_t gate_reason;  /* v4: b5_straight_reason_t (0-15)        */
    uint8_t gate_echoes;  /* v4: close echoes counted (0-3)         */
} b5_mission_status_t;

static inline void b5mp_put16 (uint8_t * p_out, uint16_t value)
{
    p_out[0] = (uint8_t)(value & 0xffU);
    p_out[1] = (uint8_t)(value >> 8);
}

static inline void b5mp_pack (const b5_mission_status_t * p_status,
                              uint8_t sequence, uint8_t out[B5MP_FRAME])
{
    memset(out, 0, B5MP_FRAME);
    out[0] = 0xF2U; /* source 15, schema version 2 */
    out[1] = (uint8_t)B5MP_PAYLOAD;
    out[2] = sequence;
    uint8_t * p_payload = out + 3;
    b5mp_put16(p_payload, (uint16_t)(p_status->ms & 0xffffU));
    b5mp_put16(p_payload + 2, (uint16_t)(p_status->ms >> 16));
    p_payload[4] = (uint8_t)B5MP_MARKER;
    p_payload[5] = (uint8_t)B5MP_VERSION;
    b5mp_put16(p_payload + 6, p_status->run);
    p_payload[8] = p_status->stage;
    p_payload[9] = p_status->reason;
    p_payload[10] = p_status->module_reason;
    p_payload[11] = p_status->coordinator;
    b5mp_put16(p_payload + 12,
               (uint16_t)((p_status->flags & B5MP_F_MASK) |
                          ((uint16_t)(p_status->gate_reason & 15U) << 12)));
    b5mp_put16(p_payload + 14, p_status->ping_mm);
    b5mp_put16(p_payload + 16, p_status->ping_age_ms);
    p_payload[18] = p_status->ping_class;
    p_payload[19] = p_status->width_policy;
    b5mp_put16(p_payload + 20, (uint16_t)p_status->servo_requested_dd);
    b5mp_put16(p_payload + 22, (uint16_t)p_status->servo_applied_dd);
    p_payload[24] = p_status->decision;
    p_payload[25] = p_status->basis;
    p_payload[26] = p_status->states;
    p_payload[27] = p_status->direction_end;
    p_payload[28] = p_status->side;
    p_payload[29] = p_status->width_end;
    p_payload[30] = p_status->width_basis;
    p_payload[31] = p_status->width_status;
    p_payload[32] = p_status->width_reason;
    p_payload[33] = p_status->width_retries;
    b5mp_put16(p_payload + 34, p_status->width_display_mm100);
    b5mp_put16(p_payload + 36, p_status->width_raw_mm100);
    b5mp_put16(p_payload + 38, p_status->width_corrected_mm100);
    b5mp_put16(p_payload + 40, p_status->width_d0_mm);
    b5mp_put16(p_payload + 42, p_status->trigger_mm);
    b5mp_put16(p_payload + 44, p_status->bypass_range_mm);
    p_payload[46] = p_status->bypass_state;
    p_payload[47] = p_status->rejoin_phase;
    p_payload[48] = p_status->width_produced;
    p_payload[49] = p_status->width_admitted;
    p_payload[50] = p_status->direction_produced;
    p_payload[51] = p_status->direction_admitted;
    p_payload[52] = p_status->approach;
    p_payload[53] = p_status->discarded;
    p_payload[54] = (uint8_t)p_status->steer_decideg; /* two's complement */
    p_payload[55] = p_status->steer_max_decideg;
    p_payload[56] = p_status->steer_saturated;
    b5mp_put16(p_payload + 57, p_status->width_gate_mm);
    p_payload[59] =
        (uint8_t)((p_status->width_state & 0x3FU) |
                  ((p_status->gate_echoes > 3U ? 3U
                                               : (p_status->gate_echoes & 3U))
                   << 6));
    p_payload[60] = p_status->width_sample;
}

#endif

/* End of file. */
