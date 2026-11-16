/**
 * @file drive_diagnostics.h
 * @brief Drive diagnostics.
 * @details ADC/control ownership covers sensor acquisition, planning and calibration.
 * Other tasks consume protected copies; flash and stored exports require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_DRIVE_DIAGNOSTICS_H
#define RTOS_DRIVE_DIAGNOSTICS_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "src/platform/compact_signed.h"

/* Copied by the ADC/control owner. Canonical wire bits never depend on native
 * compiler bitfield layout. C8 v0.2.2: 18-byte frame, 15-byte payload.
 * p0 subtype, p1 codec version, p2 uptime32, p6 run16,
 * p8 bend phase2/side2/blend-percent7/recovery phase3 (two reserved bits),
 * p10 sweep stage3/yaw signed13 in 0.1deg,
 * p12 targets signed9+signed9/saturation1 (five reserved bits).
 */
enum
{
    RL_DRIVE_DIAG_BYTES = 18
};
/* Two previously unused bits retain the 56-byte controller-row ABI. */
static inline void rl_row_bend_phase (unsigned phase, uint32_t * p_meta,
                                      uint32_t * p_aux)
{
    *p_meta = (*p_meta & 0x7fffffffU) | ((uint32_t)(phase & 1U) << 31);
    *p_aux = (*p_aux & 0x7fffffffU) | ((uint32_t)((phase >> 1) & 1U) << 31);
}
typedef struct
{
    unsigned bend_phase : 2;
    unsigned bend_side : 2;
    unsigned bend_percent : 7;
    unsigned recovery_phase : 3;
    unsigned sweep_stage : 3;
    unsigned yaw_bits : 13;
    unsigned left_bits : 9;
    unsigned right_bits : 9;
    unsigned saturated : 1;
} rl_drive_diagnostic_t;

static inline int rl_diag_clamp (int value, int low, int high, bool * p_clipped)
{
    if (value < low)
    {
        *p_clipped = true;
        return low;
    }
    if (value > high)
    {
        *p_clipped = true;
        return high;
    }
    return value;
}
static inline rl_drive_diagnostic_t
rl_drive_diag_make (unsigned phase, unsigned side, unsigned percent,
                    unsigned recovery, unsigned stage, int yaw_x10, int left,
                    int right)
{
    bool clipped = false;
    rl_drive_diagnostic_t demand = {0};
    demand.bend_phase = (unsigned)rl_diag_clamp((int)phase, 0, 3, &clipped);
    demand.bend_side = (unsigned)rl_diag_clamp((int)side, 0, 2, &clipped);
    demand.bend_percent =
        (unsigned)rl_diag_clamp((int)percent, 0, 100, &clipped);
    demand.recovery_phase =
        (unsigned)rl_diag_clamp((int)recovery, 0, 7, &clipped);
    demand.sweep_stage = (unsigned)rl_diag_clamp((int)stage, 0, 4, &clipped);
    demand.yaw_bits =
        compact_signed_pack(rl_diag_clamp(yaw_x10, -4096, 4095, &clipped), 13U);
    demand.left_bits =
        compact_signed_pack(rl_diag_clamp(left, -180, 180, &clipped), 9U);
    demand.right_bits =
        compact_signed_pack(rl_diag_clamp(right, -180, 180, &clipped), 9U);
    demand.saturated = clipped;
    return demand;
}
static inline void rl_diag_put (uint8_t * p_data, unsigned bit, unsigned width,
                                uint32_t value)
{
    for (unsigned item_index = 0; item_index < width; ++item_index)
    {
        if ((value >> item_index) & 1U)
        {
            p_data[(bit + item_index) / 8] |=
                (uint8_t)(1U << ((bit + item_index) % 8));
        }
    }
}
static inline void rl_drive_diag_pack (const rl_drive_diagnostic_t * p_demand,
                                       uint32_t uptime, uint16_t run,
                                       uint8_t seq,
                                       uint8_t out[RL_DRIVE_DIAG_BYTES])
{
    memset(out, 0, RL_DRIVE_DIAG_BYTES);
    out[0] = 0xf2;
    out[1] = 15;
    out[2] = seq;
    uint8_t * p_data = out + 3;
    p_data[0] = 0xc8;
    p_data[1] = 0x22;
    rl_diag_put(p_data, 16, 32, uptime);
    rl_diag_put(p_data, 48, 16, run);
    rl_diag_put(p_data, 64, 2, p_demand->bend_phase);
    rl_diag_put(p_data, 66, 2, p_demand->bend_side);
    rl_diag_put(p_data, 68, 7, p_demand->bend_percent);
    rl_diag_put(p_data, 75, 3, p_demand->recovery_phase);
    rl_diag_put(p_data, 80, 3, p_demand->sweep_stage);
    rl_diag_put(p_data, 83, 13, p_demand->yaw_bits);
    rl_diag_put(p_data, 96, 9, p_demand->left_bits);
    rl_diag_put(p_data, 105, 9, p_demand->right_bits);
    rl_diag_put(p_data, 114, 1, p_demand->saturated);
}
#endif

/* End of file. */
