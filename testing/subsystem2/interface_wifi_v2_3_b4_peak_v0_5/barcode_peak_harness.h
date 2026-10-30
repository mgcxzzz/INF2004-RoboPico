/* Barcode peak decoder v0.1 glue, COMBINED build (v2.3 + Buddy4 v0.2 + ramp
 * assist v0.2 + peak decoder). Ported from
 * interface_wifi_v2_3_barcode_peak_v0_1/barcode_peak_harness.h; the decoder
 * (barcode_peak.h) is byte-identical to that folder.
 *
 * Included by v12_2/.../circuit_with_barcode/main.c after wifi_harness.h,
 * buddy4_harness.h and ramp_assist_harness.h, so it can use nav, side_analog,
 * wifi_send and wifi_forward. Runs on every GP27 sample the original decoder
 * sees (same 2 ms ADC average, same encoder path, active AUTO run only),
 * next to the unchanged original 63-bank decoder.
 *
 * BARCODE_PEAK_ACTIVE=0 (shadow): logs only; navigation exactly as before.
 * BARCODE_PEAK_ACTIVE=1 (active):
 *  - original decoder has NOT accepted this label: the peak letter is queued
 *    through the original path (barcode_queue_code -> runtime MQTT mapping,
 *    B_DECODE event -> dashboard DECODED/QUEUED, original junction logic);
 *  - original decoder already accepted this label with the SAME letter:
 *    nothing changes (duplicate suppressed);
 *  - original decoder already accepted this label with a DIFFERENT letter:
 *    the pending action is cancelled (not silently kept), like the original
 *    decoder's own bank-conflict rule; B_ANALOG_CONFLICT is logged and
 *    source-15 flags bit5 reports it;
 *  - original decoder reads a different letter AFTER the peak decoder for
 *    the same label: the original conflict rule cancels the pending action.
 *
 * Telemetry: source 15, 27-byte frame, payload 24, subtype 0xB1 (layout in
 * barcode_peak.h). Combined-build additions, both inside the peak frame's own
 * fields (no shared diagnostic bits are touched):
 *  - flags bit7 = 1: combined build (bit7 is never set by the standalone build);
 *  - reason 10 (BP_R_RUN_START) with bars 0: one announce frame at every Start,
 *    so the build is identifiable before any barcode is seen.
 * The ramp-assist source-15 frame (payload 28) is unchanged. */
#include "barcode_peak.h"
#ifndef BARCODE_PEAK_ACTIVE
#define BARCODE_PEAK_ACTIVE 0
#endif
#ifndef BARCODE_PEAK_COMBINED
#define BARCODE_PEAK_COMBINED 1
#endif
#ifndef RAMP_ASSIST_ACTIVE
#define RAMP_ASSIST_ACTIVE 0
#endif
#define BP_R_RUN_START   10u
#define BP_FLAG_COMBINED 0x80u

static BarcodePeak bp_state;
static uint16_t bp_groups;
static uint8_t bp_seq;
static uint32_t bp_decodes, bp_applied, bp_conflicts;

static uint8_t bp_build_flags(void) {
    return (uint8_t)((BARCODE_PEAK_ACTIVE ? 1u : 0u) | (BARCODE_PEAK_COMBINED ? BP_FLAG_COMBINED : 0u));
}
static void bp_send(const BpResult *r, uint64_t now, uint8_t flags) {
    uint8_t frame[BP_TLM_FRAME];
    bp_tlm_pack(r, (uint32_t)(now / 1000u), flags, bp_groups, bp_seq++, frame);
    if (wifi_forward) (void)wifi_send(WIFI_TOPIC_TLM, frame, sizeof(frame));
}

static void barcode_peak_begin(uint64_t now) {
    bp_reset(&bp_state); bp_groups = 0; bp_decodes = bp_applied = bp_conflicts = 0;
    BpResult r; memset(&r, 0, sizeof(r)); r.code = '?'; r.reason = BP_R_RUN_START;
    bp_send(&r, now, bp_build_flags());
    if (stdio_usb_connected())
        printf("BPEAK build buddy1234_peak v0.5: v2.3 + Buddy4 v0.2 + ramp assist v0.6 %s + peak decoder v0.%u %s\n",
               RAMP_ASSIST_ACTIVE ? "active" : "shadow", BP_VERSION, BARCODE_PEAK_ACTIVE ? "active" : "shadow");
}

static void barcode_peak_sample(uint16_t adc, uint64_t now, const double wheel[2]) {
    if (nav.busy) { bp_reset(&bp_state); return; }          /* turning/U-turn: no labels */
    double mm = (wheel[0] + wheel[1]) * .5;
    BpResult r;
    if (!bp_sample(&bp_state, adc, mm, &r) || r.nbars < BP_REPORT_MIN_BARS) return;
    if (bp_groups < 65535u) ++bp_groups;
    uint8_t flags = bp_build_flags();
    bool overlap = side_analog.accepted && fabs(r.start_mm - side_analog.accepted_start) <= 12 &&
                   fabs(r.end_mm - side_analog.accepted_end) <= 12;
    if (r.decoded) {
        ++bp_decodes; flags |= 2u;
        if (r.reverse) flags |= 16u;
        if (overlap) {
            flags |= 8u;
            if (nav.last_code != r.code) {
                flags |= 32u; ++bp_conflicts;
#if BARCODE_PEAK_ACTIVE
                /* Two decoders disagree on one label: never keep either movement silently. */
                if (nav.pending == nav.last_code) nav.pending = 0;
                nav.cross_candidate = nav.cross_ready = false;
                side_analog.conflict = true; side_analog.conflict_end = mm;
                barcode_event(&nav, B_ANALOG_CONFLICT, now, mm, 0, r.code, 0, 0x40u | (r.reverse ? 1u : 0u), r.pattern);
#endif
            }
        }
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
    bp_send(&r, now, flags);
    if (stdio_usb_connected())
        printf("BPEAK v0.%u %s bars=%u code=%c reason=%u dir=%s removed=%u unit=%.2fmm nspace<=%.2f wspace>=%.2f nrise>=%.0f base=%.0f orig=%s%s%s\n",
               BP_VERSION, BARCODE_PEAK_ACTIVE ? "active" : "shadow", r.nbars, r.decoded ? r.code : '-', r.reason,
               r.reverse ? "rev" : "fwd", r.removed, r.unit_mm, r.max_narrow_space, r.min_wide_space, r.min_narrow_rise, r.base,
               overlap ? "same-frame" : "none", (flags & 4u) ? " APPLIED" : "", (flags & 32u) ? " CONFLICT" : "");
}
