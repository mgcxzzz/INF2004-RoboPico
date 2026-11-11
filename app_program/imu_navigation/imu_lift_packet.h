#ifndef RTOS_IMU_LIFT_PACKET_H
#define RTOS_IMU_LIFT_PACKET_H
#include <stdbool.h>
#include <stdint.h>

/* Copied diagnostic values only. Detector arithmetic is not quantised here. */
enum { RI_LIFT_PACKET_BYTES = 30 };
typedef struct {
    uint32_t trip_ms, sample_ms;
    uint16_t events;
    signed up_mg : 13;
    unsigned norm_mg : 12;
    unsigned reason : 2, latched : 1, armed : 1, fresh : 1, can_reset : 1, saturated : 1;
    unsigned up_ms : 5, tilt_ms : 6, upright_ms : 9;
} RiLiftPacket;
static inline void ri_lift_put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline void ri_lift_put32(uint8_t *p, uint32_t v) {
    for (unsigned b = 0; b < 4; ++b) p[b] = (uint8_t)(v >> (8*b));
}
static inline void ri_lift_packet_encode(const RiLiftPacket *s, uint8_t seq,
                                         uint32_t now_ms, uint16_t run, uint8_t out[30]) {
    for (unsigned i = 0; i < RI_LIFT_PACKET_BYTES; ++i) out[i] = 0;
    out[0] = 0xf2; out[1] = 27; out[2] = seq;
    uint8_t *p = out + 3;
    p[0] = 0xc9; p[1] = 0x23;
    ri_lift_put32(p+2, now_ms); ri_lift_put32(p+6, s->trip_ms);
    ri_lift_put32(p+10, s->sample_ms); ri_lift_put16(p+14, s->events);
    ri_lift_put16(p+16, (uint16_t)s->up_mg & 8191U);
    ri_lift_put16(p+18, (uint16_t)s->norm_mg & 4095U);
    p[20] = (uint8_t)(s->reason | s->latched<<2 | s->armed<<3 | s->fresh<<4 |
                      s->can_reset<<5 | s->saturated<<6);
    p[21] = s->up_ms; p[22] = s->tilt_ms;
    ri_lift_put16(p+23, s->upright_ms); ri_lift_put16(p+25, run);
}
#endif
