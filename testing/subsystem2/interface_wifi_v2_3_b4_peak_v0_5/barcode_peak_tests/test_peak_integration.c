/* Combined-build integration tests: the REAL original decoder (barcode_analog.h,
 * barcode_nav.h, runtime MQTT mapping in wifi_protocol.c) running next to the
 * REAL peak harness (barcode_peak_harness.h) exactly as main.c calls them.
 * No SDK, radio or motors. Build twice (see run_barcode_peak_tests.sh):
 *   -DBARCODE_PEAK_ACTIVE=1  (combined firmware setting)
 *   -DBARCODE_PEAK_ACTIVE=0  (shadow: must not change navigation at all)
 * With -DPRINT_TLM it also prints every source-15 frame (peak + ramp assist,
 * interleaved) as "TLM <hex>" for the Python and dashboard decoders. */
#include <stdio.h>
#include <stdlib.h>
#include "../v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h"
#include "../ramp_assist.h"
#include "bp_captured_fixtures.h"

/* ---- the globals and transport main.c/wifi_harness.h provide ---- */
static BarcodeNav nav;
static BarcodeAnalog side_analog;
static bool wifi_forward = true;
#define WIFI_TOPIC_TLM 0u
#define MAX_TLM 512
static uint8_t tlm[MAX_TLM][64]; static unsigned tlm_len[MAX_TLM], ntlm;
static bool wifi_send(unsigned topic, const uint8_t *p, size_t n) {
    (void)topic; if (ntlm < MAX_TLM && n <= 64) { memcpy(tlm[ntlm], p, n); tlm_len[ntlm++] = (unsigned)n; } return true;
}
static bool stdio_usb_connected(void) { return false; }
#define RAMP_ASSIST_ACTIVE 1
#include "../barcode_peak_harness.h"

static int checks, fails;
#define CHECK(c, msg) do { ++checks; if (!(c)) { ++fails; printf("FAIL %s (line %d)\n", msg, __LINE__); } } while (0)

/* ---- synthetic labels and optics (same model as test_barcode_peak.c) ---- */
typedef struct { double a, b; } Iv;
static uint16_t code_of(char c) { for (uint16_t p = 0; p < 512; ++p) if (bp_character(p) == c) return p; return 0; }
static unsigned label(char c, double x0, double u, bool rev, Iv *iv) {
    uint16_t pats[3] = { code_of('*'), code_of(c), code_of('*') }; int el[29]; unsigned k = 0;
    for (unsigned ch = 0; ch < 3; ++ch) { for (int bit = 8; bit >= 0; --bit) el[k++] = (pats[ch] >> bit) & 1; if (ch < 2) el[k++] = 0; }
    if (rev) for (unsigned i = 0; i < 14; ++i) { int t = el[i]; el[i] = el[28 - i]; el[28 - i] = t; }
    double x = x0; unsigned n = 0;
    for (unsigned i = 0; i < 29; ++i) { double w = el[i] ? 3 * u : u; if (i % 2 == 0) iv[n++] = (Iv){ x, x + w }; x += w; }
    return n;
}
static double cov(const Iv *iv, unsigned n, double x, double s) {
    double c = 0, k = 1.0 / (s * sqrt(2.0));
    for (unsigned i = 0; i < n; ++i) c += 0.5 * (erf((iv[i].b - x) * k) - erf((iv[i].a - x) * k));
    return c < 0 ? 0 : c > 1 ? 1 : c;
}
/* BOTH: blurred enough that narrow bars are much lower than wide bars, yet with enough contrast that
 * the ORIGINAL decoder also reads the label (so both decoders see it). STRIPE: a junction stripe. */
static uint16_t adc_both(const Iv *iv, unsigned n, double x) { return (uint16_t)(175 + 2800 * pow(cov(iv, n, x, 1.6), 5) + 0.5); }
static uint16_t adc_stripe(const Iv *iv, unsigned n, double x) { return (uint16_t)(175 + 2800 * cov(iv, n, x, 0.8) + 0.5); }
static uint16_t white_noise(unsigned k) { return (uint16_t)(173 + (k * 2654435761u >> 29)); }   /* 173..180 */

/* ---- drive exactly like main.c: original sample, then peak sample ---- */
static uint64_t now_us;
static void step(double x, uint16_t adc, bool with_peak, BarcodeNav *nv, BarcodeAnalog *an) {
    now_us += 2000; double wheel[2] = { x, x };
    barcode_analog_sample(an, nv, adc, now_us, wheel);
    if (with_peak) barcode_peak_sample(adc, now_us, wheel);   /* only ever with the main-like globals */
}
typedef enum { BOTH, STRIPE } Model;
static void run(const Iv *iv, unsigned n, double from, double to, Model m, bool with_peak, BarcodeNav *nv, BarcodeAnalog *an) {
    unsigned k = 0;
    for (double x = from; x < to; x += 0.09, ++k) {
        uint16_t a = m == BOTH ? adc_both(iv, n, x) : adc_stripe(iv, n, x), w = white_noise(k);
        step(x, a > w ? a : w, with_peak, nv, an);
    }
}
/* A real GP27 capture that the original decoder fails on the car (and in replay), from path 0. */
static double feed_capture(unsigned cap, double stop_mm, bool with_peak, BarcodeNav *nv, BarcodeAnalog *an) {
    const BpCapture *c = &bp_captures[cap]; double x = 0;
    for (unsigned i = 0; i < c->n; ++i) { x = c->path100[i] / 100.0; if (x > stop_mm) break; step(x, c->adc[i], with_peak, nv, an); }
    return x;
}
static uint32_t mapping_YZAF(void) {          /* forward Y, U-turn Z, left A, right F (dashboard order) */
    uint32_t word = (uint32_t)('Y' - 'A') | (uint32_t)('Z' - 'A') << 5 | (uint32_t)('A' - 'A') << 10 | (uint32_t)('F' - 'A') << 15;
    return wp_map_replace(0, word);
}
static void begin(BarcodeNav *nv, BarcodeAnalog *an) {
    CircuitInput in = { .now = now_us };
    barcode_nav_begin(nv, &in); nv->mapping = mapping_YZAF(); double w[2] = { 0, 0 };
    barcode_analog_begin(an, nv, 175, now_us, w);
}
static void begin_main(void) { begin(&nav, &side_analog); barcode_peak_begin(now_us); }
#define CAP_Z 2           /* z_fine_v03_run1: bars 28..165 mm, path to 222 mm */
#define CAP_Y 3           /* y_original */
static unsigned count_events(const BarcodeNav *n, unsigned type, char code, unsigned flagmask) {
    unsigned c = 0;
    for (unsigned k = 0; k < n->event_count; ++k) {
        const BarcodeEvent *e = &n->events[(n->event_head + B_EVENTS - 1 - k) % B_EVENTS];
        if (e->type == type && (!code || e->code == (uint8_t)code) && (!flagmask || (e->flags & flagmask))) ++c;
    }
    return c;
}
/* last peak frame (subtype 0xB1) with reason != run-start */
static const uint8_t *last_peak_frame(void) {
    for (unsigned i = ntlm; i-- > 0;) if (tlm_len[i] == 27 && tlm[i][7] == 0xB1 && tlm[i][10] != BP_R_RUN_START) return tlm[i];
    return NULL;
}

static void test_low_contrast_applied(void) {
    ntlm = 0; begin_main();
    feed_capture(CAP_Z, 1e9, true, &nav, &side_analog);
    const uint8_t *f = last_peak_frame();
    CHECK(side_analog.selections == 0 && side_analog.candidates == 0, "real failed capture: the original decoder reads nothing (as on the car)");
#if BARCODE_PEAK_ACTIVE
    CHECK(nav.pending == 'Z' && nav.pending_action == 'U' && nav.decoded_total == 1, "peak letter queued through the original path with the runtime MQTT mapping (Z -> U-turn)");
    CHECK(count_events(&nav, B_DECODE, 'Z', 0x40) == 1, "one B_DECODE event marked as peak (flag 0x40) -> dashboard DECODED/QUEUED");
    CHECK(f && (f[8] & 0x07) == 0x07 && (f[8] & 0x80) && f[9] == 'Z', "source-15 peak frame: active, decoded, applied, combined-build bit7, code Z");
#else
    CHECK(nav.pending == 0 && nav.decoded_total == 0 && count_events(&nav, B_DECODE, 0, 0) == 0, "shadow: nothing queued");
    CHECK(f && (f[8] & 0x07) == 0x02 && (f[8] & 0x80) && f[9] == 'Z', "shadow source-15 frame: decoded, not applied, combined-build bit7");
#endif
    CHECK(ntlm >= 1 && tlm_len[0] == 27 && tlm[0][10] == BP_R_RUN_START && (tlm[0][8] & 0x80) && tlm[0][11] == 0, "run-start announce frame identifies the combined build");
    ntlm = 0; begin_main(); feed_capture(CAP_Y, 1e9, true, &nav, &side_analog);
    for (unsigned k = 0; k < 300; ++k) step(212 + 0.1 * k, white_noise(k), true, &nav, &side_analog);
#if BARCODE_PEAK_ACTIVE
    CHECK(nav.decoded_total == 1 && nav.last_code == 'Y' && nav.pending == 'Y' && nav.pending_action == 'S', "real Y capture: peak queues Y -> forward (mapping)");
#endif
}

static void test_duplicate_suppressed(void) {
    /* both decoders read the same label: one letter, and nav/decoder state bit-identical to the original alone */
    static BarcodeNav ref_nav; static BarcodeAnalog ref_an;
    uint64_t t0 = now_us; ntlm = 0; begin_main(); now_us = t0; begin(&ref_nav, &ref_an);
    Iv iv[16]; unsigned n = label('A', 60, 3, true, iv); Iv j[1] = { { 300, 318 } };      /* + junction stripe 300..318 */
    uint64_t t1 = now_us; run(iv, n, 0, 280, BOTH, true, &nav, &side_analog); run(j, 1, 280, 420, STRIPE, true, &nav, &side_analog);
    now_us = t1; run(iv, n, 0, 280, BOTH, false, &ref_nav, &ref_an); run(j, 1, 280, 420, STRIPE, false, &ref_nav, &ref_an);
    const uint8_t *f = last_peak_frame();
    CHECK(side_analog.selections == 1 && nav.decoded_total == 1 && nav.decoded_a == 1, "original decoder reads A once; no second queue from the peak decoder");
    CHECK(f && (f[8] & 0x02) && (f[8] & 0x08) && !(f[8] & 0x04) && !(f[8] & 0x20) && f[9] == 'A', "peak frame: decoded A, same frame as original, not applied, no conflict");
    CHECK(count_events(&nav, B_CROSS, 0, 0) == 1, "original cross-junction detection still confirms the junction after the label");
    CHECK(memcmp(&nav, &ref_nav, sizeof nav) == 0, "nav state (pending, events, counters) identical to the original decoder alone");
    CHECK(memcmp(&side_analog, &ref_an, sizeof side_analog) == 0, "original decoder state identical with and without the peak harness");
}

static void test_conflict_original_first(void) {
    /* original decoder accepted F for this label (white-box), peak then reads Z: must not keep F silently */
    ntlm = 0; begin_main();
    double x = feed_capture(CAP_Z, 167.0, true, &nav, &side_analog);
    barcode_queue_code(&nav, 'F'); side_analog.accepted = true; side_analog.accepted_start = 26.6; side_analog.accepted_end = 166.5;
    CHECK(nav.pending == 'F' && x < 177.0, "setup: original decoder's F pending before the peak decision");
    const BpCapture *c = &bp_captures[CAP_Z];
    for (unsigned i = 0; i < c->n; ++i) if (c->path100[i] / 100.0 > x) step(c->path100[i] / 100.0, c->adc[i], true, &nav, &side_analog);
    const uint8_t *f = last_peak_frame();
    CHECK(f && (f[8] & 0x02) && (f[8] & 0x08) && (f[8] & 0x20) && !(f[8] & 0x04) && f[9] == 'Z', "peak frame: decoded Z, same frame, CONFLICT flag, not applied");
#if BARCODE_PEAK_ACTIVE
    CHECK(nav.pending == 0, "conflicting letters on one label: pending movement cancelled, not silently kept");
    CHECK(count_events(&nav, B_ANALOG_CONFLICT, 'Z', 0x40) == 1, "conflict logged as B_ANALOG_CONFLICT (peak flag)");
#else
    CHECK(nav.pending == 'F', "shadow: original behaviour kept (logs the conflict only)");
#endif
    CHECK(nav.decoded_total == 1, "no extra decode counted");
}

static void test_conflict_peak_first(void) {
    /* peak reads Z (original fails); later the original decoder reads A over the SAME label positions */
    ntlm = 0; begin_main(); feed_capture(CAP_Z, 1e9, true, &nav, &side_analog);
#if BARCODE_PEAK_ACTIVE
    CHECK(nav.pending == 'Z', "setup: peak queued Z");
    Iv iv[16]; unsigned n = label('A', 26.6, 2.96, false, iv);
    run(iv, n, 0, 240, BOTH, true, &nav, &side_analog);                     /* same wheel positions again */
    CHECK(side_analog.candidates >= 1, "setup: the original decoder now reads a letter on that label");
    CHECK(nav.pending == 0, "original decoder disagrees afterwards: its existing conflict rule cancels the pending action");
    CHECK(count_events(&nav, B_ANALOG_CONFLICT, 'A', 0) >= 1, "original B_ANALOG_CONFLICT event");
    CHECK(nav.decoded_a == 0, "the original A is not queued over the conflict");
#endif
}

static void test_unmapped_letter(void) {
    ntlm = 0; begin_main();
    uint32_t word = (uint32_t)('Y' - 'A') | (uint32_t)('B' - 'A') << 5 | (uint32_t)('A' - 'A') << 10 | (uint32_t)('F' - 'A') << 15;
    nav.mapping = wp_map_replace(0, word);                      /* Z not mapped */
    feed_capture(CAP_Z, 1e9, true, &nav, &side_analog);
#if BARCODE_PEAK_ACTIVE
    CHECK(nav.decoded_total == 1 && nav.unsupported == 1 && nav.pending == 0, "unmapped Z: counted as unsupported, no movement queued (runtime mapping)");
    CHECK(count_events(&nav, B_DECODE, 'Z', 0x40) == 1, "B_DECODE with action '-' -> dashboard UNASSIGNED");
#else
    CHECK(nav.decoded_total == 0, "shadow: nothing counted");
#endif
}

static void test_cross_after_peak(void) {
    ntlm = 0; begin_main(); double x = feed_capture(CAP_Z, 1e9, true, &nav, &side_analog);
    Iv j[1] = { { 260, 278 } }; run(j, 1, x + 0.09, 400, STRIPE, true, &nav, &side_analog);
#if BARCODE_PEAK_ACTIVE
    CHECK(count_events(&nav, B_CROSS, 'Z', 0) == 1 && nav.cross_ready, "junction after a peak-decoded label is confirmed by the original cross logic with pending Z");
#else
    CHECK(count_events(&nav, B_CROSS, 0, 0) == 0, "shadow: no pending letter, no junction action (as original)");
#endif
}

static void test_no_barcode_inactive(void) {
    /* no barcode: white noise, one junction stripe, two short bumps -> state identical to the original alone */
    static BarcodeNav ref_nav; static BarcodeAnalog ref_an;
    uint64_t t0 = now_us; ntlm = 0; begin_main(); now_us = t0; begin(&ref_nav, &ref_an);
    Iv bumps[2] = { { 420, 421.5 }, { 600, 601 } }, j[2] = { { 150, 168 }, { 780, 798 } };
    uint64_t t1 = now_us;
    run(j, 2, 0, 400, STRIPE, true, &nav, &side_analog); run(bumps, 2, 400, 700, BOTH, true, &nav, &side_analog); run(j, 2, 700, 1000, STRIPE, true, &nav, &side_analog);
    now_us = t1;
    run(j, 2, 0, 400, STRIPE, false, &ref_nav, &ref_an); run(bumps, 2, 400, 700, BOTH, false, &ref_nav, &ref_an); run(j, 2, 700, 1000, STRIPE, false, &ref_nav, &ref_an);
    CHECK(memcmp(&nav, &ref_nav, sizeof nav) == 0 && memcmp(&side_analog, &ref_an, sizeof side_analog) == 0,
          "no barcode: navigation and original decoder state bit-identical with the peak harness running");
    CHECK(ntlm == 1, "no barcode: only the run-start announce frame is sent");
}

static void test_busy_reset(void) {
    ntlm = 0; begin_main();
    const BpCapture *c = &bp_captures[CAP_Z];
    for (unsigned i = 0; i < c->n; ++i) {
        double x = c->path100[i] / 100.0;
        nav.busy = x > 90 && x < 92;                 /* a manoeuvre in the middle of the label */
        step(x, c->adc[i], true, &nav, &side_analog);
    }
    nav.busy = false;
    CHECK(nav.decoded_total == 0 && (last_peak_frame() == NULL || !(last_peak_frame()[8] & 2)), "manoeuvre in the middle of a label: peak state reset, no partial decode");
}

/* ---- mixed source-15 stream: ramp-assist frames interleaved with peak frames ---- */
static void test_mixed_stream(void) {
    ntlm = 0; begin_main();
    unsigned peak_frames = 0, ramp_frames = 0; uint8_t rseq = 0; const BpCapture *c = &bp_captures[CAP_Y];
    for (unsigned i = 0; i < c->n + 300; ++i) {
        double x = i < c->n ? c->path100[i] / 100.0 : 212 + 0.1 * (i - c->n);
        step(x, i < c->n ? c->adc[i] : white_noise(i), true, &nav, &side_analog);
        if (i % 100 == 0) {                                   /* ramp-assist report interleaved (~every 200 ms) */
            RampAssistTlm t = { .uptime_ms = (uint32_t)(now_us / 1000), .event = RA_EVT_PERIODIC,
                                .flags = (uint8_t)(1u | (2u << 4) | 0x80u), .alpha_milli = 0, .height_mm = 3, .peak_mm = 0,
                                .int_max = 4, .int_min = -1, .valid = 25 };
            uint8_t f[RA_TLM_FRAME_BYTES]; ra_tlm_pack(&t, rseq++, f); (void)wifi_send(WIFI_TOPIC_TLM, f, sizeof f);
        }
    }
    for (unsigned i = 0; i < ntlm; ++i) {
        if (tlm_len[i] == 27 && tlm[i][0] == 0xF2 && tlm[i][1] == 24 && tlm[i][7] == 0xB1) ++peak_frames;
        else if (tlm_len[i] == 31 && tlm[i][0] == 0xF2 && tlm[i][1] == 28) ++ramp_frames;
    }
    CHECK(peak_frames == 2 && ramp_frames >= 10 && peak_frames + ramp_frames == ntlm, "mixed stream: every source-15 frame is exactly one of the two validated layouts");
#ifdef PRINT_TLM
    for (unsigned i = 0; i < ntlm; ++i) { printf("TLM "); for (unsigned k = 0; k < tlm_len[i]; ++k) printf("%02x", tlm[i][k]); printf("\n"); }
#endif
}

int main(void) {
    now_us = 1000000;
    test_low_contrast_applied(); test_duplicate_suppressed(); test_conflict_original_first(); test_conflict_peak_first();
    test_unmapped_letter(); test_cross_after_peak(); test_no_barcode_inactive(); test_busy_reset(); test_mixed_stream();
    if (fails) { printf("FAILED %d of %d checks (BARCODE_PEAK_ACTIVE=%d)\n", fails, checks, BARCODE_PEAK_ACTIVE); return 1; }
    printf("PASS combined peak integration (BARCODE_PEAK_ACTIVE=%d): %d checks (real original decoder + runtime mapping + peak harness: apply, duplicate, both conflict orders, unmapped, junction, no-barcode identity, busy reset, mixed source-15)\n",
           BARCODE_PEAK_ACTIVE, checks);
    return 0;
}
