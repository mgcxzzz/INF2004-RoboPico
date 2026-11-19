#ifndef B5_EDGE_PROBE_PACK_H
#define B5_EDGE_PROBE_PACK_H
/* BE/BF records and summaries: edge_probe v0.1.1 v2 layout, version byte 3
 * (B5EP_DIR_WIRE: 25 mm near allowance). See edge_probe v0.1.1 CONTRACT.md. */
#include <string.h>
#include "extensions/buddy5_obstacles/edge_probe.h"
#include "extensions/buddy5_obstacles/b5_scan_pack.h"
#define B5_EP_REC_FRAME 51u
#define B5_EP_SUM_FRAME 47u
static inline void b5_ep_rec_pack(const B5EPRec *r, uint16_t run, uint16_t d0,
    uint16_t lost, uint8_t seq, uint8_t out[B5_EP_REC_FRAME]) {
    memset(out, 0, B5_EP_REC_FRAME);
    out[0] = 0xF2; out[1] = 48; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, r->ms); p[4] = 0xBE; p[5] = (uint8_t)B5EP_DIR_WIRE;
    b5_scan_u16(p + 6, run); b5_scan_u16(p + 8, r->index);
    p[10] = r->phase; p[11] = r->n;
    b5_scan_u16(p + 12, (uint16_t)r->heading_cdeg);
    b5_scan_u16(p + 14, (uint16_t)r->servo_dd);
    b5_scan_u16(p + 16, (uint16_t)r->applied_dd);
    b5_scan_u16(p + 18, (uint16_t)r->travel_mm10);
    b5_scan_u16(p + 20, r->expected_mm); b5_scan_u16(p + 22, r->anchor_mm);
    for (unsigned i = 0; i < 3; ++i) {
        p[24 + i] = r->cls[i]; b5_scan_u16(p + 28 + 2 * i, r->value[i]);
    }
    p[27] = r->evidence;
    b5_scan_u16(p + 34, d0); b5_scan_u16(p + 36, lost);
    b5_scan_u16(p + 38, (uint16_t)r->net_x_mm10);
    b5_scan_u16(p + 40, (uint16_t)r->net_y_mm10);
    b5_scan_u16(p + 42, B5EP_SERVO_LIMIT_DD);
    b5_scan_u16(p + 44, B5EP_PROBE_MM * 10u);
    b5_scan_u16(p + 46, (uint16_t)r->correction_cdeg);
}
static inline void b5_ep_sum_pack(const B5EPSummary *m, uint8_t seq,
                                 uint8_t out[B5_EP_SUM_FRAME]) {
    memset(out, 0, B5_EP_SUM_FRAME);
    out[0] = 0xF2; out[1] = 44; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, m->ms); p[4] = 0xBF; p[5] = (uint8_t)B5EP_DIR_WIRE;
    b5_scan_u16(p + 6, m->run); p[8] = m->end; p[9] = m->decision;
    p[10] = m->state_l; p[11] = m->state_r;
    b5_scan_u16(p + 12, m->records); b5_scan_u16(p + 14, m->lost);
    b5_scan_u16(p + 16, m->d0);
    b5_scan_u16(p + 18, (uint16_t)m->advance_l_mm10);
    b5_scan_u16(p + 20, (uint16_t)m->reverse_l_mm10);
    b5_scan_u16(p + 22, (uint16_t)m->advance_r_mm10);
    b5_scan_u16(p + 24, (uint16_t)m->reverse_r_mm10);
    b5_scan_u16(p + 26, (uint16_t)m->turn_l_cdeg);
    b5_scan_u16(p + 28, (uint16_t)m->turn_r_cdeg);
    b5_scan_u16(p + 30, (uint16_t)m->turn_back_cdeg);
    b5_scan_u16(p + 32, (uint16_t)m->final_heading_cdeg);
    b5_scan_u16(p + 34, (uint16_t)m->net_x_mm10);
    b5_scan_u16(p + 36, (uint16_t)m->net_y_mm10);
    b5_scan_u16(p + 38, m->flags);
    b5_scan_u16(p + 40, m->correction_count);
    b5_scan_u16(p + 42, (uint16_t)m->correction_total_cdeg);
}
#endif
