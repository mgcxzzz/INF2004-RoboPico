/**
 * @file encoder_fault.h
 * @brief Encoder fault.
 * @details Core1 owns normal motor commits. Safety paths may only remove output;
 * command/status access uses the existing kernel locks and copied records.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_ENCODER_FAULT_H
#define RTOS_ENCODER_FAULT_H

#include <stdbool.h>
#include <stdint.h>

/* Source 15, subtype 0xC4. All multi-byte fields are little-endian.
 * The recent quality window is for the triggering wheel only; lifetime
 * encoder counters and drive values cover both wheels.
 * Wire offsets: header 0..5; timer/kernel u32 at 6/10; direction duration
 * u17 at 14; recent bad/changes u20/u20 packed at 17; lifetime edges/invalid
 * u32 pairs at 22/26 (left), 30/34 (right); targets signed9 at 38; PWM
 * signed11 at 41; measured speed signed16 x10 at 44/46. */
#define BR_ENCODER_FAULT_PACKET_BYTES 48U

enum
{
    BR_ENCODER_FAULT_KIND_NONE = 0,
    BR_ENCODER_FAULT_KIND_BURST = 1,
    BR_ENCODER_FAULT_KIND_DENSITY = 2,
    BR_ENCODER_FAULT_KIND_DIRECTION = 3,
    BR_ENCODER_FAULT_WHEEL_NONE = 0,
    BR_ENCODER_FAULT_WHEEL_LEFT = 1,
    BR_ENCODER_FAULT_WHEEL_RIGHT = 2
};

typedef struct
{
    /* Native snapshot metadata is bounded; counters retain their full u32 range. */
    unsigned valid : 1;
    unsigned kind : 2;
    unsigned wheel : 2;
    uint32_t control_timer_us;
    uint32_t kernel_ms;
    uint32_t direction_duration_us;
    uint32_t recent_window_bad;
    uint32_t recent_window_changes;
    uint32_t lifetime_edges[2];
    uint32_t lifetime_invalid[2];
    int16_t optical_target_mm_s[2];
    int16_t previous_pwm[2];
    int16_t measured_speed_x10[2];
} bringup_encoder_fault_snapshot_t;

static inline void
br_encoder_fault_clear (bringup_encoder_fault_snapshot_t * p_snapshot)
{
    *p_snapshot = (bringup_encoder_fault_snapshot_t){0};
}

/* First-trip-wins until an accepted Start or eligible Reset clears the latch. */
static inline bool br_encoder_fault_capture_once (
    bringup_encoder_fault_snapshot_t * p_stored,
    const bringup_encoder_fault_snapshot_t * p_candidate)
{
    if (p_stored->valid)
    {
        return false;
    }
    *p_stored = *p_candidate;
    p_stored->valid = 1U;
    return true;
}

static inline void br_encoder_fault_put32 (uint8_t * p_out, unsigned at_value,
                                           uint32_t value)
{
    p_out[at_value] = (uint8_t)value;
    p_out[at_value + 1U] = (uint8_t)(value >> 8U);
    p_out[at_value + 2U] = (uint8_t)(value >> 16U);
    p_out[at_value + 3U] = (uint8_t)(value >> 24U);
}

static inline uint32_t
br_encoder_fault_signed_code (int16_t value, int16_t minimum, int16_t maximum,
                              unsigned bits, bool * p_clamped)
{
    int16_t bounded = value;
    if (bounded < minimum)
    {
        bounded = minimum;
        *p_clamped = true;
    }
    else if (bounded > maximum)
    {
        bounded = maximum;
        *p_clamped = true;
    }
    return (uint16_t)bounded & ((1UL << bits) - 1UL);
}

static inline void br_encoder_fault_put24 (uint8_t * p_out, unsigned at_value,
                                           uint32_t value)
{
    p_out[at_value] = (uint8_t)value;
    p_out[at_value + 1U] = (uint8_t)(value >> 8U);
    p_out[at_value + 2U] = (uint8_t)(value >> 16U);
}

/* 48 bytes total. flags: bit0 valid, bits1-2 kind, bits3-4 wheel,
 * bit5 indicates any bounded field was clamped, bits6-7 reserved. */
static inline void
br_encoder_fault_encode (const bringup_encoder_fault_snapshot_t * p_snapshot,
                         uint8_t seq,
                         uint8_t out[BR_ENCODER_FAULT_PACKET_BYTES])
{
    for (unsigned index = 0U; index < BR_ENCODER_FAULT_PACKET_BYTES; ++index)
    {
        out[index] = 0U;
    }
    out[0] = 0xf2U;
    out[1] = BR_ENCODER_FAULT_PACKET_BYTES - 3U;
    out[2] = seq;
    out[3] = 0xc4U;
    out[4] = 0x21U; /* v0.2.1 */
    if (!p_snapshot->valid)
    {
        return; /* Versioned invalid frame clears any older dashboard snapshot. */
    }

    bool clamped = false;
    uint32_t direction = p_snapshot->direction_duration_us;
    uint32_t recent_bad = p_snapshot->recent_window_bad;
    uint32_t recent_changes = p_snapshot->recent_window_changes;
    if (direction > 0x1ffffU)
    {
        direction = 0x1ffffU;
        clamped = true;
    }
    if (recent_bad > 0xfffffU)
    {
        recent_bad = 0xfffffU;
        clamped = true;
    }
    if (recent_changes > 0xfffffU)
    {
        recent_changes = 0xfffffU;
        clamped = true;
    }
    uint32_t target[2], pwm[2];
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
        target[wheel] = br_encoder_fault_signed_code(
            p_snapshot->optical_target_mm_s[wheel], -180, 180, 9U, &clamped);
        pwm[wheel] = br_encoder_fault_signed_code(
            p_snapshot->previous_pwm[wheel], -1000, 1000, 11U, &clamped);
    }
    out[5] =
        (uint8_t)(1U | ((p_snapshot->kind & 3U) << 1U) |
                  ((p_snapshot->wheel & 3U) << 3U) | (clamped ? 0x20U : 0U));
    br_encoder_fault_put32(out, 6U, p_snapshot->control_timer_us);
    br_encoder_fault_put32(out, 10U, p_snapshot->kernel_ms);
    out[14] = (uint8_t)direction;
    out[15] = (uint8_t)(direction >> 8U);
    out[16] = (uint8_t)(direction >> 16U);
    uint64_t quality = (uint64_t)recent_bad | ((uint64_t)recent_changes << 20U);
    for (unsigned index_value = 0U; index_value < 5U; ++index_value)
    {
        out[17U + index_value] = (uint8_t)(quality >> (8U * index_value));
    }
    br_encoder_fault_put32(out, 22U, p_snapshot->lifetime_edges[0]);
    br_encoder_fault_put32(out, 26U, p_snapshot->lifetime_invalid[0]);
    br_encoder_fault_put32(out, 30U, p_snapshot->lifetime_edges[1]);
    br_encoder_fault_put32(out, 34U, p_snapshot->lifetime_invalid[1]);
    uint32_t targets = target[0] | (target[1] << 9U);
    uint32_t pwms = pwm[0] | (pwm[1] << 11U);
    br_encoder_fault_put24(out, 38U, targets);
    br_encoder_fault_put24(out, 41U, pwms);
    for (unsigned wheel_value = 0U; wheel_value < 2U; ++wheel_value)
    {
        uint16_t speed = (uint16_t)p_snapshot->measured_speed_x10[wheel_value];
        out[44U + 2U * wheel_value] = (uint8_t)speed;
        out[45U + 2U * wheel_value] = (uint8_t)(speed >> 8U);
    }
}

#endif

/* End of file. */
