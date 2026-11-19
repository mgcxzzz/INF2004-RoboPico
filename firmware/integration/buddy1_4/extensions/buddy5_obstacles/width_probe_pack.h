#ifndef B5_WIDTH_PROBE_PACK_H
#define B5_WIDTH_PROBE_PACK_H
/* EXPERIMENTAL width diagnostic wire format, version 4 (B5WP_WIRE_VERSION; v4 = v3 layout and
 * phases with near allowance 25 and front_gap against the face plane - 25;
 * same byte layout as v1/v2). v3 semantics: expected = REF plane + pose for every
 * record, anchor = REF (d0, index 0); phases 5 LAD_OUT, 6 LAD_RET, 7 MID;
 * evidence 3 = TRANSITION; summary flags bits 11..15 (see width_probe.h). Source15 subtypes
 * 0xD1 (56-byte record) and 0xD2 (58-byte summary). Distinct from the
 * direction probe BE/BF frames, which are unchanged. Little-endian. */
#include <string.h>
#include "extensions/buddy5_obstacles/width_probe.h"
#include "extensions/buddy5_obstacles/b5_scan_pack.h"
#define B5_WP_REC_PAYLOAD 56u
#define B5_WP_SUM_PAYLOAD 58u
#define B5_WP_REC_FRAME (B5_WP_REC_PAYLOAD + 3u)
#define B5_WP_SUM_FRAME (B5_WP_SUM_PAYLOAD + 3u)
_Static_assert(B5_WP_REC_FRAME <= 64u && B5_WP_SUM_FRAME <= 64u, "WIFI_PACKET_MAX is 64");
/* D3 width result, version B5WP_RESULT_WIRE (1), payload 58 (frame 61):
 * 0 ms u32, 4 0xD3, 5 ver, 6 run, 8 end, 9 basis, 10 status, 11 reason,
 * 12 display_mm100, 14 raw_mm100, 16 corrected_mm100, 18 sampled_mm10,
 * 20 span_lo_mm10, 22 span_hi_mm10, 24 flags u16,
 * 26 L {status u8, n u8, a_mm10 u16, b_mm10 u16, base_qmm i16}, 34 R {same},
 * 42 d0, 44 records, 46 lost, 48 offset_mm10 (100), 50 range_lo_mm (100),
 * 52 range_hi_mm (200), 54 high_threshold_mm10 (100), 56 rule_id, 57 width_result (D2).
 * 0xFFFF = not available. Little-endian. */
#define B5_WP_RES_PAYLOAD 58u
#define B5_WP_RES_FRAME (B5_WP_RES_PAYLOAD + 3u)
_Static_assert(B5_WP_RES_FRAME <= 64u, "WIFI_PACKET_MAX is 64");
/* D4 v1 records a rejected triplet before a bounded same-pose retry.
 * Retry packets are independent of accepted D1 record indices. */
#define B5_WP_RETRY_PAYLOAD 32u
#define B5_WP_RETRY_FRAME (B5_WP_RETRY_PAYLOAD + 3u)
_Static_assert(B5_WP_RETRY_FRAME <= 64u, "retry packet fits MQTT transport");
static inline void b5_wp_retry_pack(const B5WPRetry *r, uint8_t seq,
                                    uint8_t out[B5_WP_RETRY_FRAME]) {
    memset(out, 0, B5_WP_RETRY_FRAME);
    out[0] = 0xF2; out[1] = B5_WP_RETRY_PAYLOAD; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, r->ms); p[4] = 0xD4; p[5] = 1u;
    b5_scan_u16(p + 6, r->run);
    p[8] = r->phase; p[9] = r->side; p[10] = r->step; p[11] = r->attempt;
    b5_scan_u16(p + 12, r->expected_mm);
    b5_scan_u16(p + 14, (uint16_t)r->heading_cdeg);
    b5_scan_u16(p + 16, (uint16_t)r->applied_dd);
    for (unsigned i = 0; i < 3u; ++i) {
        p[18 + i] = r->cls[i];
        b5_scan_u16(p + 22 + 2 * i, r->value[i]);
    }
    b5_scan_u16(p + 28, 50u);  /* extra dwell in ms */
    b5_scan_u16(p + 30, 15u);  /* reference spread limit remains 15 mm */
}
static inline void b5_wp_rec_pack(const B5WPRec *r, uint16_t d0, uint16_t lost, uint8_t seq,
                                  uint8_t out[B5_WP_REC_FRAME]) {
    memset(out, 0, B5_WP_REC_FRAME);
    out[0] = 0xF2; out[1] = B5_WP_REC_PAYLOAD; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, r->ms); p[4] = 0xD1; p[5] = (uint8_t)B5WP_WIRE_VERSION;
    b5_scan_u16(p + 6, r->run); b5_scan_u16(p + 8, r->index);
    p[10] = r->phase; p[11] = r->side; p[12] = r->step; p[13] = r->n;
    b5_scan_u16(p + 14, (uint16_t)r->heading_cdeg);
    b5_scan_u16(p + 16, (uint16_t)r->servo_dd);
    b5_scan_u16(p + 18, (uint16_t)r->applied_dd);
    b5_scan_u16(p + 20, (uint16_t)r->path_mm10);
    b5_scan_u16(p + 22, (uint16_t)r->seg_cmd_mm10);
    b5_scan_u16(p + 24, (uint16_t)r->seg_meas_mm10);
    b5_scan_u16(p + 26, r->expected_mm);
    b5_scan_u16(p + 28, r->anchor_mm);
    b5_scan_u16(p + 30, r->anchor_index);
    for (unsigned i = 0; i < 3; ++i) {
        p[32 + i] = r->cls[i];
        b5_scan_u16(p + 36 + 2 * i, r->value[i]);
    }
    p[35] = r->evidence;
    b5_scan_u16(p + 42, d0); b5_scan_u16(p + 44, lost);
    b5_scan_u16(p + 46, (uint16_t)r->net_x_mm10);
    b5_scan_u16(p + 48, (uint16_t)r->net_y_mm10);
    b5_scan_u16(p + 50, (uint16_t)r->correction_cdeg);
    b5_scan_u16(p + 52, (uint16_t)r->axis_x_mm10);
    b5_scan_u16(p + 54, (uint16_t)r->front_gap_mm10);
}
static inline void b5_wp_sum_pack(const B5WPSummary *m, uint8_t seq,
                                  uint8_t out[B5_WP_SUM_FRAME]) {
    memset(out, 0, B5_WP_SUM_FRAME);
    out[0] = 0xF2; out[1] = B5_WP_SUM_PAYLOAD; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, m->ms); p[4] = 0xD2; p[5] = (uint8_t)B5WP_WIRE_VERSION;
    b5_scan_u16(p + 6, m->run);
    p[8] = m->end; p[9] = m->width_result; p[10] = m->state_l; p[11] = m->state_r;
    p[12] = m->steps_l; p[13] = m->steps_r;
    b5_scan_u16(p + 14, m->records); b5_scan_u16(p + 16, m->lost);
    b5_scan_u16(p + 18, m->d0);
    b5_scan_u16(p + 20, (uint16_t)m->out_l_mm10);
    b5_scan_u16(p + 22, (uint16_t)m->out_r_mm10);
    b5_scan_u16(p + 24, (uint16_t)m->res_l_mm10);
    b5_scan_u16(p + 26, (uint16_t)m->res_r_mm10);
    b5_scan_u16(p + 28, (uint16_t)m->turn_l_cdeg);
    b5_scan_u16(p + 30, (uint16_t)m->turn_r_cdeg);
    b5_scan_u16(p + 32, (uint16_t)m->final_heading_cdeg);
    b5_scan_u16(p + 34, (uint16_t)m->net_x_mm10);
    b5_scan_u16(p + 36, (uint16_t)m->net_y_mm10);
    b5_scan_u16(p + 38, m->flags);
    b5_scan_u16(p + 40, m->correction_count);
    b5_scan_u16(p + 42, (uint16_t)m->correction_total_cdeg);
    b5_scan_u16(p + 44, (uint16_t)m->l_in_mm10);
    b5_scan_u16(p + 46, (uint16_t)m->l_far_mm10);
    b5_scan_u16(p + 48, (uint16_t)m->r_in_mm10);
    b5_scan_u16(p + 50, (uint16_t)m->r_far_mm10);
    b5_scan_u16(p + 52, m->width_lo_mm10);
    b5_scan_u16(p + 54, m->width_hi_mm10);
    b5_scan_u16(p + 56, (uint16_t)m->min_front_gap_mm10);
}
static inline void b5_wp_result_pack(const B5WEResult *m, uint32_t ms, uint16_t run,
                                     uint8_t end, uint16_t d0, uint16_t records, uint16_t lost,
                                     uint8_t width_result, uint8_t seq,
                                     uint8_t out[B5_WP_RES_FRAME]) {
    memset(out, 0, B5_WP_RES_FRAME);
    out[0] = 0xF2; out[1] = B5_WP_RES_PAYLOAD; out[2] = seq;
    uint8_t *p = out + 3;
    b5_scan_u32(p, ms); p[4] = 0xD3; p[5] = (uint8_t)B5WP_RESULT_WIRE;
    b5_scan_u16(p + 6, run);
    p[8] = end; p[9] = m->basis; p[10] = m->status; p[11] = m->reason;
    b5_scan_u16(p + 12, m->display_mm100);
    b5_scan_u16(p + 14, m->raw_mm100);
    b5_scan_u16(p + 16, m->corrected_mm100);
    b5_scan_u16(p + 18, m->sampled_mm10);
    b5_scan_u16(p + 20, m->span_lo_mm10);
    b5_scan_u16(p + 22, m->span_hi_mm10);
    b5_scan_u16(p + 24, m->flags);
    for (unsigned k = 0; k < 2u; ++k) {
        uint8_t *s = p + 26 + 8 * k;
        int32_t b = m->side[k].base4;
        s[0] = m->side[k].status; s[1] = m->side[k].n;
        b5_scan_u16(s + 2, m->side[k].a10);
        b5_scan_u16(s + 4, m->side[k].b10);
        b5_scan_u16(s + 6, (uint16_t)(int16_t)(b > 32767 ? 32767 : b < -32768 ? -32768 : b));
    }
    b5_scan_u16(p + 42, d0); b5_scan_u16(p + 44, records); b5_scan_u16(p + 46, lost);
    b5_scan_u16(p + 48, B5WE_OFFSET_MM100 / 10u);
    b5_scan_u16(p + 50, B5WE_MIN_MM100 / 100u);
    b5_scan_u16(p + 52, B5WE_MAX_MM100 / 100u);
    b5_scan_u16(p + 54, B5WE_HIGH_Q * 10u / 4u);
    p[56] = (uint8_t)B5WE_RULE_ID; p[57] = width_result;
}
#endif
