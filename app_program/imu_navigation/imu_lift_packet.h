/**
 * @file imu_lift_packet.h
 * @brief Imu lift packet.
 * @details The IMU task owns sensor acquisition and calibration; readers use
 * snapshots. Ramp requests expire through the coordinator; lift safety may
 * remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_IMU_LIFT_PACKET_H
#define RTOS_IMU_LIFT_PACKET_H
#include <stdbool.h>
#include <stdint.h>

/* Copied diagnostic values only. Detector arithmetic is not quantised here. */
enum
{
    RI_LIFT_PACKET_BYTES = 30
};
typedef struct
{
    uint32_t trip_ms;
    uint32_t sample_ms;
    uint16_t events;
    unsigned up_bits : 13;
    unsigned norm_mg : 12;
    unsigned reason : 2;
    unsigned latched : 1;
    unsigned armed : 1;
    unsigned fresh : 1;
    unsigned can_reset : 1;
    unsigned saturated : 1;
    unsigned up_ms : 5;
    unsigned tilt_ms : 7;
    unsigned upright_ms : 9;
} ri_lift_packet_t;
typedef struct
{
    uint32_t report_ms;
    uint32_t peak_sample_ms;
    uint32_t window_start_ms;
    int16_t min_cos_q15;
    uint16_t norm_mg;
    uint16_t events; /* norm at min-cos */
    int16_t up_mg;   /* lifetime maximum calibrated-up projection */
    uint8_t tilt_ms;
    uint8_t up_ms;
    /* Independent since-Reset histories; not tied to peak_sample_ms. */
    uint8_t last_candidate;
    uint8_t last_rejection;
    bool armed;
    bool current_valid;
    bool latched;
    bool saturated;
} ri_lift_peak_packet_t;
static inline void ri_lift_put16 (uint8_t * p_data, uint16_t value)
{
    p_data[0] = (uint8_t)value;
    p_data[1] = (uint8_t)(value >> 8);
}
static inline void ri_lift_put32 (uint8_t * p_data, uint32_t value)
{
    for (unsigned second = 0; second < 4; ++second)
    {
        p_data[second] = (uint8_t)(value >> (8 * second));
    }
}
static inline void ri_lift_packet_encode (const ri_lift_packet_t * p_state,
                                          uint8_t seq, uint32_t now_ms,
                                          uint16_t run, uint8_t out[30])
{
    for (unsigned index = 0; index < RI_LIFT_PACKET_BYTES; ++index)
    {
        out[index] = 0;
    }
    out[0] = 0xf2;
    out[1] = 27;
    out[2] = seq;
    uint8_t * p_data = out + 3;
    p_data[0] = 0xc9;
    p_data[1] = 0x25;
    ri_lift_put32(p_data + 2, now_ms);
    ri_lift_put32(p_data + 6, p_state->trip_ms);
    ri_lift_put32(p_data + 10, p_state->sample_ms);
    ri_lift_put16(p_data + 14, p_state->events);
    ri_lift_put16(p_data + 16, (uint16_t)p_state->up_bits & 8191U);
    ri_lift_put16(p_data + 18, (uint16_t)p_state->norm_mg & 4095U);
    p_data[20] = (uint8_t)(p_state->reason | p_state->latched << 2 |
                           p_state->armed << 3 | p_state->fresh << 4 |
                           p_state->can_reset << 5 | p_state->saturated << 6);
    p_data[21] = p_state->up_ms;
    p_data[22] = p_state->tilt_ms;
    ri_lift_put16(p_data + 23, p_state->upright_ms);
    ri_lift_put16(p_data + 25, run);
}

enum
{
    RI_LIFT_PEAK_PACKET_BYTES = 28
};
static inline void
ri_lift_peak_packet_encode (const ri_lift_peak_packet_t * p_state, uint8_t seq,
                            uint8_t out[RI_LIFT_PEAK_PACKET_BYTES])
{
    for (unsigned index = 0; index < RI_LIFT_PEAK_PACKET_BYTES; ++index)
    {
        out[index] = 0;
    }
    out[0] = 0xf2;
    out[1] = 25;
    out[2] = seq;
    uint8_t * p_data = out + 3;
    p_data[0] = 0xca;
    p_data[1] = 1;
    ri_lift_put32(p_data + 2, p_state->report_ms);
    ri_lift_put32(p_data + 6, p_state->peak_sample_ms);
    ri_lift_put16(p_data + 10, (uint16_t)p_state->min_cos_q15);
    ri_lift_put16(p_data + 12, p_state->norm_mg & 0x0fffU);
    ri_lift_put16(p_data + 14, (uint16_t)p_state->up_mg & 0x1fffU);
    p_data[16] = p_state->tilt_ms & 0x7fU;
    p_data[17] = p_state->up_ms & 0x1fU;
    p_data[18] =
        (uint8_t)((p_state->last_candidate & 3U) |
                  ((p_state->last_rejection & 3U) << 2) |
                  (p_state->armed << 4) | (p_state->current_valid << 5) |
                  (p_state->latched << 6) | (p_state->saturated << 7));
    ri_lift_put32(p_data + 19, p_state->window_start_ms);
    ri_lift_put16(p_data + 23, p_state->events);
}
#endif

/* End of file. */
