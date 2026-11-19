#ifndef B5_EVENT_PACK_H
#define B5_EVENT_PACK_H
/* v0.5 Buddy5 acquisition/hand-back event, sent on the EXISTING telemetry
 * topic as an SDK source-15 frame, subtype marker 0xB5 (dispatch by payload
 * length 32 and payload[4], like the other source-15 layouts).
 *
 * Frame: [0xF2, 32, seq] + payload (little endian):
 *  0 u32 Pico uptime ms        4 u8 0xB5 marker      5 u8 version(1) | flags<<4
 *  6 u8 event code            7 u8 B5 state          8 u8 rejoin phase
 *  9 u8 B5 reason             10 u8 result           11 u8 detail
 * 12 u8 search turns used     13 u8 detector rejects 14 i16 A (x0.1)
 * 16 i16 B (x0.1)             18 i16 encoder heading since adoption (0.01 deg, +LEFT)
 * 20 u16 level L  22 u16 level R (0..1000)   24 u16 bg L  26 u16 bg R
 * 28 u16 event sequence       30 u16 events overwritten before sending
 * flags: bit0 geometry-only image.
 * Event meanings (A, B, detail, result) are tabulated in AUTOMATIC_V0_5.md. */
#include <stdint.h>
#include <string.h>
#include "extensions/buddy5_obstacles/automatic_bypass.h"

#define B5_EVENT_SOURCE   (15u)
#define B5_EVENT_MARKER   (0xB5u)
#define B5_EVENT_VERSION  (1u)
#define B5_EVENT_PAYLOAD  (32u)
#define B5_EVENT_FRAME    (3u + B5_EVENT_PAYLOAD)

static inline void b5_event_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline unsigned b5_event_pack(const B5Event *e, bool geometry_only, uint16_t dropped,
                                     uint8_t seq, uint8_t out[B5_EVENT_FRAME]) {
    memset(out, 0, B5_EVENT_FRAME);
    out[0] = (uint8_t)((B5_EVENT_SOURCE << 4) | 2u); /* schema version 2 header */
    out[1] = B5_EVENT_PAYLOAD;
    out[2] = seq;
    uint8_t *p = out + 3;
    p[0] = (uint8_t)e->ms; p[1] = (uint8_t)(e->ms >> 8);
    p[2] = (uint8_t)(e->ms >> 16); p[3] = (uint8_t)(e->ms >> 24);
    p[4] = B5_EVENT_MARKER;
    p[5] = (uint8_t)(B5_EVENT_VERSION | ((geometry_only ? 1u : 0u) << 4));
    p[6] = e->code; p[7] = e->state; p[8] = e->phase; p[9] = e->reason;
    p[10] = e->result; p[11] = e->detail; p[12] = e->search; p[13] = e->rejects;
    b5_event_put16(p + 14, (uint16_t)e->a);
    b5_event_put16(p + 16, (uint16_t)e->b);
    b5_event_put16(p + 18, (uint16_t)e->heading_cdeg);
    b5_event_put16(p + 20, e->level[0]);
    b5_event_put16(p + 22, e->level[1]);
    b5_event_put16(p + 24, e->bg[0]);
    b5_event_put16(p + 26, e->bg[1]);
    b5_event_put16(p + 28, e->seq);
    b5_event_put16(p + 30, dropped);
    return B5_EVENT_FRAME;
}
#endif
