/**
 * @file drive_mode_telemetry.h
 * @brief Drive mode telemetry.
 * @details Application tasks exchange copied, bounded command and telemetry
 * records. The kernel Wi-Fi service owns the transport; reconnect never
 * restarts motion.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_DRIVE_MODE_TELEMETRY_H
#define RTOS_DRIVE_MODE_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Source15 subtype CB. The struct is copied state, never used as a wire
 * layout; rn_drive_mode_encode packs every field explicitly. */
typedef struct
{
    uint32_t uptime_ms;
    uint32_t revision;
    unsigned mode : 2;
    unsigned requested_mode : 2;
    unsigned last_result : 3;
    unsigned can_change : 1;
    unsigned session_active : 1;
    unsigned load_left : 7;
    unsigned load_right : 7;
} rn_drive_mode_telemetry_t;

static inline void rn_drive_mode_put32 (uint8_t * p_out, uint32_t value)
{
    p_out[0] = (uint8_t)value;
    p_out[1] = (uint8_t)(value >> 8U);
    p_out[2] = (uint8_t)(value >> 16U);
    p_out[3] = (uint8_t)(value >> 24U);
}

static inline bool
rn_drive_mode_encode (const rn_drive_mode_telemetry_t * p_in_value, uint8_t seq,
                      uint8_t out[17])
{
    if ((((((!p_in_value) || (!out)) || (p_in_value->mode > 2U)) ||
          (p_in_value->last_result > 5U)) ||
         (p_in_value->load_left > 100U)) ||
        (p_in_value->load_right > 100U))
    {
        return false;
    }
    for (unsigned index = 0; index < 17U; ++index)
    {
        out[index] = 0U;
    }
    out[0] = 0xf2U;
    out[1] = 14U;
    out[2] = seq;
    out[3] = 0xcbU;
    out[4] = 0x25U;
    rn_drive_mode_put32(out + 5U, p_in_value->uptime_ms);
    rn_drive_mode_put32(out + 9U, p_in_value->revision);
    out[13] = (uint8_t)(p_in_value->mode | (p_in_value->requested_mode << 2U) |
                        (p_in_value->last_result << 4U) |
                        (p_in_value->can_change ? 0x80U : 0U));
    out[14] = (uint8_t)((p_in_value->session_active ? 1U : 0U) |
                        (p_in_value->load_left ? 2U : 0U) |
                        (p_in_value->load_right ? 4U : 0U));
    uint16_t pwm = (uint16_t)p_in_value->load_left |
                   (uint16_t)(p_in_value->load_right << 7U);
    out[15] = (uint8_t)pwm;
    out[16] = (uint8_t)(pwm >> 8U);
    return true;
}

#endif

/* End of file. */
