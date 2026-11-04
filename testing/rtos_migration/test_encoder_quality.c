#include <assert.h>
#include <stdio.h>
#include "app_program/rtos_bringup/encoder_quality.h"

int main(void) {
    BrEncoderQuality q = {0};
    uint32_t edges = 0, bad = 0, now = 0;
    br_encoder_reset(&q, now, edges, bad);
    /*100 scattered errors over100seconds do not become a lifetime fault. */
    for (unsigned n = 1; n <= 10000; ++n) {
        now += 10000;
        edges += 6;
        if (n % 100 == 0) {
            ++bad;
            edges += 2;
        }
        assert(br_encoder_quality(&q, now, edges, bad) == BR_ENC_OK);
    }
    br_encoder_reset(&q, 0, 0, 0);
    assert(br_encoder_quality(&q, 10000, 16, 8) == BR_ENC_BURST);
    /* Crossing a bin boundary cannot hide a concentrated burst. */
    br_encoder_reset(&q, 0, 0, 0);
    assert(br_encoder_quality(&q, 99000, 8, 4) == BR_ENC_OK);
    assert(br_encoder_quality(&q, 101000, 16, 8) == BR_ENC_BURST);
    /*8 errors spread across800ms: dense feedback, not a100ms burst. */
    br_encoder_reset(&q, 0, 0, 0);
    for (unsigned n = 1; n <= 7; ++n)
        assert(br_encoder_quality(&q, n * 100000, n * 11, n) == BR_ENC_OK);
    assert(br_encoder_quality(&q, 800000, 88, 8) == BR_ENC_DENSITY);
    /* Same8 errors with >160 observed changes: below5%, keep monitoring. */
    br_encoder_reset(&q, 0, 0, 0);
    for (unsigned n = 1; n <= 8; ++n)
        assert(br_encoder_quality(&q, n * 100000, n * 101, n) == BR_ENC_OK);
    assert(q.window_bad == 8 && q.window_changes == 800);
    assert(br_encoder_quality(&q, 1800000, 808, 8) == BR_ENC_OK && q.window_bad == 0);
    /* Pause does not reset raw totals; stale errors age out naturally. */
    br_encoder_reset(&q, 0, 1000, 6);
    for (unsigned n = 1; n <= 6; ++n)
        assert(br_encoder_quality(&q, n * 200000, 1000 + n * 100, 6 + n) == BR_ENC_OK);
    assert(br_encoder_quality(&q, 3000000, 1600, 12) == BR_ENC_OK);
    assert(br_encoder_quality(&q, 3010000, 1620, 14) == BR_ENC_OK);
    /* uint32 TIMER wrap, bounded saturation and long owner gaps. */
    br_encoder_reset(&q, UINT32_MAX - 50000, UINT32_MAX - 10, 4);
    assert(br_encoder_quality(&q, 50000, 20, 5) == BR_ENC_OK);
    assert(br_encoder_quality(&q, 60000, 40, 12) == BR_ENC_BURST);
    br_encoder_reset(&q, 0, 0, 0);
    assert(br_encoder_quality(&q, 10000, UINT32_MAX, UINT32_MAX) == BR_ENC_BURST);
    assert(q.window_bad == UINT16_MAX);
    puts(
        "PASS encoder quality: sparse/repeated errors, boundary bursts, density, Pause, wrap and saturation");
    return 0;
}
