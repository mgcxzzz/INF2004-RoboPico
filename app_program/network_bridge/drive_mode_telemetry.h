#ifndef RTOS_DRIVE_MODE_TELEMETRY_H
#define RTOS_DRIVE_MODE_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* Source15 subtype CB. The struct is copied state, never used as a wire
 * layout; rn_drive_mode_encode packs every field explicitly. */
typedef struct
{
    uint32_t uptime_ms, revision;
    unsigned mode : 2;
    unsigned requested_mode : 2;
    unsigned last_result : 3;
    unsigned can_change : 1;
    unsigned session_active : 1;
    unsigned load_left : 7;
    unsigned load_right : 7;
} RnDriveModeTelemetry;

static inline void rn_drive_mode_put32 (uint8_t * out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8U);
    out[2] = (uint8_t)(value >> 16U);
    out[3] = (uint8_t)(value >> 24U);
}

static inline bool rn_drive_mode_encode (const RnDriveModeTelemetry * in,
                                         uint8_t seq, uint8_t out[17])
{
    if (!in || !out || in->mode > 2U || in->last_result > 5U ||
        in->load_left > 100U || in->load_right > 100U)
    {
        return false;
    }
    for (unsigned i = 0; i < 17U; ++i)
    {
        out[i] = 0U;
    }
    out[0] = 0xf2U;
    out[1] = 14U;
    out[2] = seq;
    out[3] = 0xcbU;
    out[4] = 0x25U;
    rn_drive_mode_put32(out + 5U, in->uptime_ms);
    rn_drive_mode_put32(out + 9U, in->revision);
    out[13] =
        (uint8_t)(in->mode | (in->requested_mode << 2U) |
                  (in->last_result << 4U) | (in->can_change ? 0x80U : 0U));
    out[14] = (uint8_t)((in->session_active ? 1U : 0U) |
                        (in->load_left ? 2U : 0U) | (in->load_right ? 4U : 0U));
    uint16_t pwm = (uint16_t)in->load_left | (uint16_t)(in->load_right << 7U);
    out[15] = (uint8_t)pwm;
    out[16] = (uint8_t)(pwm >> 8U);
    return true;
}

#endif

/* End of file. */
