#include <assert.h>
#include <stdio.h>
#include "app_program/rtos_bringup/bringup_policy.h"
#include "src/motion/encoder_and_guard.h"

int main(void) {
    assert(br_clock_elapsed_ok(1000000U, 1000U));
    assert(br_clock_elapsed_ok(1002000U, 1000U));
    assert(!br_clock_elapsed_ok(1531474U, 1000U)); /* V1.1 disagreement. */
    assert(!br_clock_elapsed_ok(23426U, 1000U));   /* ~42.7x slow TIMER. */
    assert(!br_clock_elapsed_ok(1000000U, 23U));
    assert(!br_clock_elapsed_ok(0U, 1000U));
    assert(!br_clock_elapsed_ok(1000000U, 0U));
    assert(!br_clock_elapsed_ok(2000000U, 2000U));
    uint16_t ready = BR_READY | BR_CLOCK_OK;
    assert(br_can_start(ready, 2000000U, 0U, 3U, true));
    assert(!br_can_start(BR_READY, 2000000U, 0U, 3U, true));
    assert(!br_can_start(BR_CLOCK_OK, 2000000U, 0U, 3U, true));
    assert(!br_can_start(ready | BR_ACTIVE, 2000000U, 0U, 3U, true));
    for (unsigned latch = BR_ESTOP; latch <= BR_ENCODER; latch <<= 1U) {
        if ((latch & BR_LATCHED) != 0U) {
            assert(!br_can_start(ready | latch, 2000000U, 0U, 3U, true));
            assert(!br_outputs_allowed(ready | BR_ACTIVE | latch, false, 3U, true));
        }
    }
    assert(!br_can_start(ready, 999999U, 0U, 3U, true));
    assert(!br_can_start(ready, 2000000U, 0U, 2U, true));
    assert(!br_can_start(ready, 2000000U, 0U, 3U, false));
    assert(br_can_start(ready, 2000000U, UINT32_MAX - 1000000U, 3U, true));
    assert(br_command_fresh(20U, UINT32_MAX - 20U));
    assert(!br_command_fresh(500001U, 0U));
    assert(br_outputs_allowed(ready | BR_ACTIVE, false, 3U, true));
    assert(!br_outputs_allowed(ready | BR_ACTIVE, true, 3U, true));
    assert(!br_outputs_allowed(ready | BR_ACTIVE, false, 2U, true));
    assert(!br_outputs_allowed(ready | BR_ACTIVE, false, 3U, false));
    assert(!br_outputs_allowed(ready, false, 3U, true));
    assert(!br_outputs_allowed(BR_READY | BR_ACTIVE, false, 3U, true));
    BenchEncoder encoder = {0};
    const uint8_t forward[] = {1, 3, 2, 0};
    for (unsigned i = 0; i < 4; ++i)
        bench_encoder_update(&encoder, forward[i]);
    assert(encoder.position == 4 && encoder.invalid == 0);
    const uint8_t reverse[] = {2, 3, 1, 0};
    for (unsigned i = 0; i < 4; ++i)
        bench_encoder_update(&encoder, reverse[i]);
    assert(encoder.position == 0 && encoder.invalid == 0);
    bench_encoder_update(&encoder, 3);
    assert(encoder.invalid == 1 && encoder.position == 0);
    puts(
        "PASS: clock disagreement/freeze, start gates, urgent-stop output gates, wraparound and encoder decoding");
    return 0;
}
