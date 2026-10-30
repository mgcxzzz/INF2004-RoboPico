/* Generates the combined reporting path for the dashboard tests, using the REAL C code:
 * original decoder + peak harness on a real failed Y capture (BARCODE_PEAK_ACTIVE=1), the real
 * ramp-assist packer interleaved, the real mapping readback (wp_map_frame) and the real source 10/11
 * packers (wpk_line / wpk_nav) filled the same way wifi_harness.h fills them from nav.
 * Output: one "TLM <hex>" line per frame, in send order. */
#define main integration_main
#include "test_peak_integration.c"
#undef main
#include "../wifi_packets.h"

static uint8_t seq_line, seq_nav;
static void emit(const uint8_t *p, size_t n) { (void)wifi_send(WIFI_TOPIC_TLM, p, n); }
static void line_status(unsigned event) {                       /* as wifi_line_status() */
    uint8_t pk[WPK_LINE_BYTES];
    WifiLinePacket p = { .seq = seq_line++, .uptime_ms = (uint32_t)(now_us / 1000), .event = (uint8_t)event,
        .adc_left = 160, .adc_right = 170, .adc_barcode = 176, .calibrated_level_left = 0, .calibrated_level_right = 0,
        .last_decoded_letter = (uint8_t)((nav.last_code >= 'A' && nav.last_code <= 'Z') ? nav.last_code : 0),
        .pending_letter = (uint8_t)nav.pending, .pending_action = (uint8_t)nav.pending_action, .pending_revision = nav.pending_revision,
        .executing_action = 0, .executing_revision = 0 };
    if (wpk_line(&p, pk)) emit(pk, sizeof pk);
}
static void nav_event(unsigned event, char letter, char action, unsigned revision) {   /* as wifi_event() */
    uint8_t pk[WPK_NAV_BYTES];
    WifiNavPacket p = { .seq = seq_nav++, .uptime_ms = (uint32_t)(now_us / 1000), .event = (uint8_t)event,
        .letter = (uint8_t)((letter >= 'A' && letter <= 'Z') ? letter : 0), .action = (uint8_t)action, .map_revision = (uint8_t)revision,
        .command_id = 0, .generation = 1, .fault_bitmap = 0, .owner = 1, .mode = 2, .command_state = 0, .detail = 0 };
    if (wpk_nav(&p, pk)) emit(pk, sizeof pk);
}

int main(void) {
    now_us = 1000000; ntlm = 0;
    begin(&nav, &side_analog);
    uint8_t map[WP_MAP_FRAME_BYTES]; wp_map_frame(nav.mapping, 7, 0, map); emit(map, sizeof map);   /* readback, request id 7 */
    barcode_peak_begin(now_us);
    line_status(WPK_EVENT_STATUS);
    const BpCapture *c = &bp_captures[CAP_Y]; uint8_t rseq = 0; unsigned seen = 0;
    for (unsigned i = 0; i < c->n + 300; ++i) {
        double x = i < c->n ? c->path100[i] / 100.0 : 212 + 0.1 * (i - c->n);
        step(x, i < c->n ? c->adc[i] : white_noise(i), true, &nav, &side_analog);
        for (; seen < nav.event_sequence; ++seen) {                /* as wifi_barcode_events(): B_DECODE -> DECODED + QUEUED */
            const BarcodeEvent *e = &nav.events[(nav.event_head + B_EVENTS - (nav.event_sequence - seen)) % B_EVENTS];
            if (e->type != B_DECODE) continue;
            nav_event(WPK_EVENT_DECODED, (char)e->code, (char)e->action, e->map_revision);
            nav_event(e->action == '-' ? WPK_EVENT_UNASSIGNED : WPK_EVENT_QUEUED, (char)e->code, (char)e->action, e->map_revision);
        }
        if (i % 100 == 0) {
            RampAssistTlm t = { .uptime_ms = (uint32_t)(now_us / 1000), .event = RA_EVT_PERIODIC,
                                .flags = (uint8_t)(1u | (2u << 4) | 0x80u), .height_mm = 3, .int_max = 4, .int_min = -1, .valid = 25 };
            uint8_t f[RA_TLM_FRAME_BYTES]; ra_tlm_pack(&t, rseq++, f); emit(f, sizeof f);
        }
        if (i % 25 == 0) line_status(WPK_EVENT_STATUS);
    }
    if (nav.last_code != 'Y' || nav.pending != 'Y') { fprintf(stderr, "generator: expected a peak-decoded Y\n"); return 1; }
    for (unsigned i = 0; i < ntlm; ++i) { printf("TLM "); for (unsigned k = 0; k < tlm_len[i]; ++k) printf("%02x", tlm[i][k]); printf("\n"); }
    return 0;
}
