#ifndef B5_SCAN_PACK_H
#define B5_SCAN_PACK_H
/* v0.6.3 DIAGNOSTIC ONLY (no commands, never read by a decision).
 * Source 15, two new layouts, distinct by payload length AND marker:
 *
 *  0xB7 scan acquisition record, payload 36, version 1. One per acquired
 *  angle in acquisition order (coarse / refine / edge retry):
 *   0 u32 ms            4 u8 0xB7        5 u8 version
 *   6 u16 scan id       8 u8 index       9 u8 kind (0 coarse,1 refine,2 retry)
 *  10 i16 angle dd     12..14 u8 ping class x3: bits0-1 cls (0 echo,1 no-return,
 *                              2 fault), bits4-6 fault why
 *  15 u8 reserved(0)   16..20 u16 x3 value: echo range mm, else width us (clamped)
 *  22 u16 safety range mm after merge at this angle   24 u8 safety flags
 *  25 u8 perception flags                              26 u16 perception range
 *  28..32 u16 x3 rise latency us (trigger end -> rise; 0 = no rise)
 *  34 u16 records overwritten before sending (cumulative)
 *  flags: b0 valid b1 weak(no-return clear) b2 partial b3 block-only b4 stable
 *
 *  0xB8 clearance diagnosis, payload 44, version 1. One per plan:
 *   0 u32 ms  4 u8 0xB8  5 u8 version  6 u16 scan id  8 u16 decision seq (B6)
 *  10/11 u8 reason L/R (B5_CLR_*)  12/13 u8 pairs L/R  14/15 u8 sample flags L/R
 *  16/18 i16 offending angle dd L/R (0x7FFF none)  20/22 u16 range mm L/R
 *  24/26 i16 value L/R  28/30 i16 edge dd L/R  32/34 i16 envelope edge x mm L/R
 *  36 i16 corridor depth mm  38 i16 comparison window dd
 *  40 u8 acquisitions in scan  41/42 u8 edge index L/R  43 u8 reserved(0)
 * Layout decoder: tools/dashboard/decode_b5_scan.py. */
#include <stdint.h>
#include <string.h>
#define B5_SCAN_REC_FRAME   39u
#define B5_CLEAR_FRAME      47u
#define B5_SCAN_REC_MAX     48u
typedef struct {
    uint32_t ms;
    uint16_t scan_id;
    uint8_t index, kind;
    int16_t angle_dd;
    uint8_t cls[3];
    uint16_t value[3];
    uint16_t latency_us[3];
    uint16_t safety_mm, perception_mm;
    uint8_t safety_flags, perception_flags;
} B5ScanRec;
typedef struct {
    uint32_t ms;
    uint16_t scan_id, decision_seq;
    uint8_t reason[2], pairs[2], flags[2], edge_index[2];
    int16_t angle_dd[2], value[2], edge_dd[2], near_edge_x[2];
    uint16_t range_mm[2];
    int16_t corridor_y, lim_dd;
    uint8_t records;
} B5ClearRec;
static inline void b5_scan_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void b5_scan_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void b5_scan_rec_pack(const B5ScanRec *r, uint16_t lost, uint8_t seq,
                                    uint8_t out[B5_SCAN_REC_FRAME]) {
    memset(out, 0, B5_SCAN_REC_FRAME);
    out[0] = 0xF2; out[1] = 36; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, r->ms); p[4] = 0xB7; p[5] = 1;
    b5_scan_u16(p + 6, r->scan_id); p[8] = r->index; p[9] = r->kind;
    b5_scan_u16(p + 10, (uint16_t)r->angle_dd);
    for (unsigned i = 0; i < 3; ++i) {
        p[12 + i] = r->cls[i];
        b5_scan_u16(p + 16 + 2 * i, r->value[i]);
        b5_scan_u16(p + 28 + 2 * i, r->latency_us[i]);
    }
    b5_scan_u16(p + 22, r->safety_mm); p[24] = r->safety_flags;
    p[25] = r->perception_flags; b5_scan_u16(p + 26, r->perception_mm);
    b5_scan_u16(p + 34, lost);
}
static inline void b5_clear_pack(const B5ClearRec *c, uint8_t seq, uint8_t out[B5_CLEAR_FRAME]) {
    memset(out, 0, B5_CLEAR_FRAME);
    out[0] = 0xF2; out[1] = 44; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, c->ms); p[4] = 0xB8; p[5] = 1;
    b5_scan_u16(p + 6, c->scan_id); b5_scan_u16(p + 8, c->decision_seq);
    for (unsigned s = 0; s < 2; ++s) {
        p[10 + s] = c->reason[s]; p[12 + s] = c->pairs[s]; p[14 + s] = c->flags[s];
        b5_scan_u16(p + 16 + 2 * s, (uint16_t)c->angle_dd[s]);
        b5_scan_u16(p + 20 + 2 * s, c->range_mm[s]);
        b5_scan_u16(p + 24 + 2 * s, (uint16_t)c->value[s]);
        b5_scan_u16(p + 28 + 2 * s, (uint16_t)c->edge_dd[s]);
        b5_scan_u16(p + 32 + 2 * s, (uint16_t)c->near_edge_x[s]);
        p[41 + s] = c->edge_index[s];
    }
    b5_scan_u16(p + 36, (uint16_t)c->corridor_y); b5_scan_u16(p + 38, (uint16_t)c->lim_dd);
    p[40] = c->records;
}
#endif
