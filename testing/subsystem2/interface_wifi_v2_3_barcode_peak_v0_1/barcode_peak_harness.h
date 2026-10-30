/* Barcode peak decoder v0.1 glue for the original SDK Wi-Fi v2.3 main
 * (v12_2/baseline/.../circuit_with_barcode/main.c). Included after
 * wifi_harness.h so it can use nav, side_analog, wifi_send and wifi_forward.
 *
 * Runs barcode_peak.h on every GP27 sample the original decoder sees (same
 * 2 ms ADC average, same encoder path, only in an active AUTO run), next to
 * the unchanged original 63-bank decoder.
 *
 * BARCODE_PEAK_ACTIVE=0 (shadow): logs only. Navigation uses the original
 *   decoder exactly as in v2.3.
 * BARCODE_PEAK_ACTIVE=1 (active): when the peak decoder reads a complete *X*
 *   frame that the original decoder has NOT already accepted, the letter is
 *   queued through the original path (barcode_queue_code + B_DECODE event, so
 *   the dashboard shows it and the junction logic acts on it). If the original
 *   decoder already accepted that frame, nothing is changed. If the original
 *   decoder later reads a different letter for the same frame, its existing
 *   conflict rule cancels the pending action.
 *
 * Every bar group of >= 6 bars sends one source-15 frame (see barcode_peak.h)
 * and, on USB, one "BPEAK" console line. */
#include "barcode_peak.h"
#ifndef BARCODE_PEAK_ACTIVE
#define BARCODE_PEAK_ACTIVE 0
#endif

static BarcodePeak bp_state;
static uint16_t bp_groups;
static uint8_t bp_seq;
static uint32_t bp_decodes, bp_applied;

static void barcode_peak_begin(void) {
    bp_reset(&bp_state); bp_groups = 0; bp_decodes = bp_applied = 0;
}

static void barcode_peak_sample(uint16_t adc, uint64_t now, const double wheel[2]) {
    if (nav.busy) { bp_reset(&bp_state); return; }          /* turning/U-turn: no labels */
    double mm = (wheel[0] + wheel[1]) * .5;
    BpResult r;
    if (!bp_sample(&bp_state, adc, mm, &r) || r.nbars < BP_REPORT_MIN_BARS) return;
    if (bp_groups < 65535u) ++bp_groups;
    uint8_t flags = BARCODE_PEAK_ACTIVE ? 1u : 0u;
    bool overlap = side_analog.accepted && fabs(r.start_mm - side_analog.accepted_start) <= 12 &&
                   fabs(r.end_mm - side_analog.accepted_end) <= 12;
    if (r.decoded) {
        ++bp_decodes; flags |= 2u;
        if (r.reverse) flags |= 16u;
        if (overlap) { flags |= 8u; if (nav.last_code != r.code) flags |= 32u; }
#if BARCODE_PEAK_ACTIVE
        else {
            barcode_queue_code(&nav, r.code);
            nav.cross_candidate = nav.cross_ready = false;
            side_analog.accepted = true; side_analog.accepted_start = r.start_mm; side_analog.accepted_end = r.end_mm;
            AnalogBank *v = &side_analog.bank[side_analog.selected];
            barcode_reader_reset(&v->reader, v->stable, now, mm);   /* a half-read noise frame must not block the junction search */
            barcode_event(&nav, B_DECODE, now, mm, r.end_mm - r.start_mm, r.code,
                          barcode_decode_action(&nav, r.code), 0x40u | (r.reverse ? 1u : 0u), r.pattern);
            flags |= 4u; ++bp_applied;
        }
#endif
    }
    if (r.overflow) flags |= 64u;
    uint8_t frame[BP_TLM_FRAME];
    bp_tlm_pack(&r, (uint32_t)(now / 1000u), flags, bp_groups, bp_seq++, frame);
    if (wifi_forward) (void)wifi_send(WIFI_TOPIC_TLM, frame, sizeof(frame));
    if (stdio_usb_connected())
        printf("BPEAK v0.%u %s bars=%u code=%c reason=%u dir=%s removed=%u unit=%.2fmm nspace<=%.2f wspace>=%.2f nrise>=%.0f base=%.0f orig=%s%s\n",
               BP_VERSION, BARCODE_PEAK_ACTIVE ? "active" : "shadow", r.nbars, r.decoded ? r.code : '-', r.reason,
               r.reverse ? "rev" : "fwd", r.removed, r.unit_mm, r.max_narrow_space, r.min_wide_space, r.min_narrow_rise, r.base,
               overlap ? "same-frame" : "none", (flags & 4u) ? " APPLIED" : "");
}
