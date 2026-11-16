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
    uint32_t trip_ms, sample_ms;
    uint16_t events;
    signed up_mg : 13;
    unsigned norm_mg : 12;
    unsigned reason : 2, latched : 1, armed : 1, fresh : 1, can_reset : 1,
        saturated : 1;
    unsigned up_ms : 5, tilt_ms : 7, upright_ms : 9;
} RiLiftPacket;
typedef struct
{
    uint32_t report_ms, peak_sample_ms, window_start_ms;
    int16_t min_cos_q15;
    uint16_t norm_mg, events; /* norm at min-cos */
    int16_t up_mg;            /* lifetime maximum calibrated-up projection */
    uint8_t tilt_ms, up_ms;
    /* Independent since-Reset histories; not tied to peak_sample_ms. */
    uint8_t last_candidate, last_rejection;
    bool armed, current_valid, latched, saturated;
} RiLiftPeakPacket;
static inline void ri_lift_put16 (uint8_t * p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static inline void ri_lift_put32 (uint8_t * p, uint32_t v)
{
    for (unsigned b = 0; b < 4; ++b)
    {
        p[b] = (uint8_t)(v >> (8 * b));
    }
}
static inline void ri_lift_packet_encode (const RiLiftPacket * s, uint8_t seq,
                                          uint32_t now_ms, uint16_t run,
                                          uint8_t out[30])
{
    for (unsigned i = 0; i < RI_LIFT_PACKET_BYTES; ++i)
    {
        out[i] = 0;
    }
    out[0] = 0xf2;
    out[1] = 27;
    out[2] = seq;
    uint8_t * p = out + 3;
    p[0] = 0xc9;
    p[1] = 0x25;
    ri_lift_put32(p + 2, now_ms);
    ri_lift_put32(p + 6, s->trip_ms);
    ri_lift_put32(p + 10, s->sample_ms);
    ri_lift_put16(p + 14, s->events);
    ri_lift_put16(p + 16, (uint16_t)s->up_mg & 8191U);
    ri_lift_put16(p + 18, (uint16_t)s->norm_mg & 4095U);
    p[20] = (uint8_t)(s->reason | s->latched << 2 | s->armed << 3 |
                      s->fresh << 4 | s->can_reset << 5 | s->saturated << 6);
    p[21] = s->up_ms;
    p[22] = s->tilt_ms;
    ri_lift_put16(p + 23, s->upright_ms);
    ri_lift_put16(p + 25, run);
}

enum
{
    RI_LIFT_PEAK_PACKET_BYTES = 28
};
static inline void
ri_lift_peak_packet_encode (const RiLiftPeakPacket * s, uint8_t seq,
                            uint8_t out[RI_LIFT_PEAK_PACKET_BYTES])
{
    for (unsigned i = 0; i < RI_LIFT_PEAK_PACKET_BYTES; ++i)
    {
        out[i] = 0;
    }
    out[0] = 0xf2;
    out[1] = 25;
    out[2] = seq;
    uint8_t * p = out + 3;
    p[0] = 0xca;
    p[1] = 1;
    ri_lift_put32(p + 2, s->report_ms);
    ri_lift_put32(p + 6, s->peak_sample_ms);
    ri_lift_put16(p + 10, (uint16_t)s->min_cos_q15);
    ri_lift_put16(p + 12, s->norm_mg & 0x0fffU);
    ri_lift_put16(p + 14, (uint16_t)s->up_mg & 0x1fffU);
    p[16] = s->tilt_ms & 0x7fU;
    p[17] = s->up_ms & 0x1fU;
    p[18] =
        (uint8_t)((s->last_candidate & 3U) | ((s->last_rejection & 3U) << 2) |
                  (s->armed << 4) | (s->current_valid << 5) |
                  (s->latched << 6) | (s->saturated << 7));
    ri_lift_put32(p + 19, s->window_start_ms);
    ri_lift_put16(p + 23, s->events);
}
#endif

/* End of file. */
