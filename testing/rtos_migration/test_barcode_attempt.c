#define APP_RTOS_BARCODE 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_program/barcode_navigation/barcode_model.h"

static RbDecoder decoder;
static LinePlanner planner;
static RbRaw pinned[RB_RAW_ROWS];
static uint64_t now_us = 1000000;
static double position_mm;

static void add_raw(unsigned count, uint32_t interval_us) {
    for (unsigned k = 0; k < count; ++k) {
        now_us += interval_us;
        position_mm += 0.09;
        rb_raw(&decoder, &planner.nav, (uint16_t)(175 + (k & 3)), now_us, position_mm);
    }
}

static void snapshot_raw(void) {
    memcpy(pinned, decoder.raw, sizeof(pinned));
}

static void assert_raw_pinned(void) {
    assert(memcmp(pinned, decoder.raw, sizeof(pinned)) == 0);
}

static void begin_attempt(unsigned bars, double first_bar_mm) {
    (void)bars;
    decoder.peak.nbars = BP_REPORT_MIN_BARS;
    for (unsigned k = 0; k < BP_REPORT_MIN_BARS; ++k)
        decoder.peak.bar[k].mm = first_bar_mm + 2.0 * k;
    now_us += 2000;
    position_mm += 0.09;
    double wheel[2] = {position_mm, position_mm};
    rb_sample(&decoder, &planner.nav, 175, now_us, wheel);
    assert(decoder.raw_capture_active);
}

int main(void) {
    CircuitInput input = {.now = now_us, .wheel = {0, 0}};
    rl_begin(&planner, &input, 175);
    rb_begin(&decoder, &planner.nav, 175, now_us, input.wheel);

    /* Ten 2ms samples produce four records: the first and then one at least
     * every 5ms. Restart the decoder before checking long-run coverage. */
    add_raw(10, 2000);
    assert(decoder.preroll_count == 4);
    input = (CircuitInput){.now = now_us, .wheel = {position_mm, position_mm}};
    rb_begin(&decoder, &planner.nav, 175, now_us, input.wheel);

    /* The 5ms logger receives 2ms calls, retains a bounded 512-row pre-roll,
     * and does not expose idle movement as a diagnostic capture. */
    add_raw(3000, 2000);
    assert(decoder.samples == 3000);
    assert(decoder.preroll_count == RB_RAW_PREROLL_ROWS);
    assert(decoder.raw_count == 0 && decoder.raw_head == 0);
    assert(!decoder.raw_frozen && decoder.attempt.sequence == 0);
    now_us += RB_RAW_INTERVAL_US;
    position_mm += 0.09;
    rb_raw(&decoder, &planner.nav, 0x0ABC, now_us, position_mm);
    const RbRaw *last = &decoder.preroll[(decoder.preroll_head + RB_RAW_PREROLL_ROWS - 1) %
                                         RB_RAW_PREROLL_ROWS];
    assert(rb_get32(last->bytes) == (uint32_t)((now_us - planner.nav.start_us) / 1000));
    assert((int32_t)rb_get32(last->bytes + 4) == (int32_t)lround(position_mm * 100));
    assert(last->bytes[8] == 0xBC && last->bytes[9] == 0x0A);

    /* Attempt 1 succeeds. The raw rows include pre-roll and become immutable
     * after report, even as ordinary white samples keep arriving. */
    begin_attempt(15, position_mm - 25.0);
    assert(decoder.raw_count == RB_RAW_PREROLL_ROWS);
    assert(decoder.attempt.sequence == 1);
    assert(decoder.attempt.flags & RB_ATTEMPT_START_VALID);
    add_raw(12, 6000);
    BpResult first = {.decoded = true, .code = 'Y', .reason = BP_OK, .nbars = 15};
    rb_attempt_finish(&decoder, &first, true);
    bp_reset(&decoder.peak);
    assert(decoder.raw_frozen && !decoder.raw_capture_active);
    assert(decoder.attempt.code == 'Y' && decoder.attempt.reason == BP_OK);
    assert(decoder.attempt.flags & RB_ATTEMPT_DECODED);
    assert(decoder.attempt.flags & RB_ATTEMPT_FINALIZED);
    snapshot_raw();
    add_raw(40, 6000);
    assert_raw_pinned();

    /* Attempt 2 is a failed 14-bar group. A later one-bar junction/noise group
     * does not promote its rolling pre-roll over the reported barcode. */
    begin_attempt(14, position_mm - 25.0);
    assert(decoder.attempt.code == 0); /* No stale Y from attempt 1. */
    add_raw(8, 6000);
    BpResult failed = {.decoded = false, .code = '?', .reason = BP_R_FEW, .nbars = 14};
    rb_attempt_finish(&decoder, &failed, true);
    bp_reset(&decoder.peak);
    assert(decoder.attempt.sequence == 2 && decoder.attempt.bars == 14);
    assert(decoder.attempt.reason == BP_R_FEW && decoder.attempt.code == 0);
    assert(!(decoder.attempt.flags & RB_ATTEMPT_DECODED));
    snapshot_raw();

    input = (CircuitInput){.now = now_us, .wheel = {position_mm, position_mm}};
    rb_partial_reset(&decoder, &planner.nav, 175, now_us, input.wheel);
    bool saw_single_bar = false;
    double junction_start = position_mm + 5.0;
    for (unsigned k = 0; k < 360; ++k) {
        now_us += 2000;
        position_mm += 0.09;
        double wheel[2] = {position_mm, position_mm};
        uint16_t adc = position_mm >= junction_start && position_mm < junction_start + 6.0
                           ? 3000
                           : 175;
        rb_sample(&decoder, &planner.nav, adc, now_us, wheel);
        if (decoder.peak.nbars) {
            saw_single_bar = true;
            assert(decoder.peak.nbars == 1);
        }
    }
    assert(saw_single_bar);
    assert(decoder.attempt.sequence == 2);
    assert(decoder.raw_frozen && !decoder.raw_capture_active);
    assert_raw_pinned();

    /* Attempt 3 has an original-decoder success but a peak-decoder failure.
     * The current letter is retained, and a long capture wraps at 2048 rows. */
    begin_attempt(14, position_mm - 25.0);
    planner.nav.last_code = 'F';
    ++planner.nav.decoded_total;
    rb_attempt_original_success(&decoder, &planner.nav);
    assert(decoder.attempt.code == 'F');
    add_raw(2100, 6000);
    rb_attempt_finish(&decoder, &failed, true);
    bp_reset(&decoder.peak);
    assert(decoder.attempt.sequence == 3);
    assert(decoder.attempt.code == 'F' && decoder.attempt.reason == BP_R_FEW);
    assert(decoder.attempt.flags & RB_ATTEMPT_ORIGINAL_ACCEPTED);
    assert(decoder.attempt.flags & RB_ATTEMPT_DECODED);
    assert(decoder.attempt.flags & RB_ATTEMPT_TRUNCATED);
    assert(!(decoder.attempt.flags & RB_ATTEMPT_START_VALID));
    assert(decoder.raw_count == RB_RAW_ROWS && decoder.attempt.raw_count == RB_RAW_ROWS);
    assert(decoder.attempt.raw_overwritten == 564);

    uint8_t frame[RB_ATTEMPT_FRAME];
    rb_attempt_pack(&decoder.attempt, 0x11223344, 0x5566, 0x77, frame);
    assert(frame[0] == 0xF2 && frame[1] == 27 && frame[2] == 0x77);
    assert(frame[3] == 0xC5 && rb_get32(frame + 4) == 0x11223344);
    assert(rb_get32(frame + 8) == 3);
    assert(rb_get32(frame + 20) == 564);
    assert(frame[24] == 0x66 && frame[25] == 0x55);
    uint32_t meta = rb_get32(frame + 26);
    assert((meta & 0x3fU) == (decoder.attempt.flags & 0x3fU));
    assert(((meta >> 6) & 31U) == ('F' - 'A' + 1U));
    assert(((meta >> 11) & 15U) == BP_R_FEW);
    assert(((meta >> 15) & 31U) == 14);
    assert((meta >> 20) == RB_RAW_ROWS);
    FILE *fixture = fopen("testing/rtos_migration/build-host/barcode_attempt.bin", "wb");
    assert(fixture && fwrite(frame,1,sizeof(frame),fixture)==sizeof(frame)); fclose(fixture);

    snapshot_raw();
    add_raw(40, 6000);
    assert_raw_pinned();

    /* Resume preserves the stopped diagnostic. A partial attempt is preserved
     * as unfinished, and a fresh run clears its sequence and raw snapshot. */
    input = (CircuitInput){.now = now_us, .wheel = {position_mm, position_mm}};
    rb_resume(&planner, &decoder, &input, 175);
    assert(decoder.attempt.sequence == 3);
    assert(decoder.attempt.flags & RB_ATTEMPT_FINALIZED);
    assert_raw_pinned();
    add_raw(100, 6000);
    begin_attempt(8, position_mm - 25.0);
    assert(decoder.attempt.code == 0); /* No stale F on an unfinished attempt. */
    add_raw(20, 6000);
    input = (CircuitInput){.now = now_us, .wheel = {position_mm, position_mm}};
    rb_resume(&planner, &decoder, &input, 175);
    assert(decoder.attempt.sequence == 4);
    assert(decoder.attempt.reason == RB_ATTEMPT_IN_PROGRESS);
    assert(!(decoder.attempt.flags & RB_ATTEMPT_FINALIZED));
    assert(decoder.raw_frozen && !decoder.raw_capture_active);
    rb_begin(&decoder, &planner.nav, 175, now_us, input.wheel);
    assert(decoder.attempt.sequence == 0 && decoder.attempt.flags == 0);
    assert(decoder.raw_count == 0 && decoder.preroll_count == 0);

    puts("PASS barcode attempt raw: bounded 5ms pre-roll, pinned failed/successful attempts, original-only success, truncation, C5 packing, Resume and run reset.");
}
