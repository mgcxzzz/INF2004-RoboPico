#include <assert.h>
#include <stdio.h>
#include "bench_logic.h"
int main(void) {
    for (uint8_t selected=0; selected<2; ++selected) {
        for (uint8_t drive=0; drive<3; ++drive) {
            BenchLevels v = bench_levels(selected, selected, drive, 25);
            assert(v.a == (drive == DRIVE_A ? 250 : 0));
            assert(v.b == (drive == DRIVE_B ? 250 : 0));
            v = bench_levels(selected, selected ^ 1u, drive, 25);
            assert(v.a == 0 && v.b == 0); /* Other motor always off. */
        }
    }
    BenchLevels v = bench_levels(0, 0, DRIVE_A, 15); assert(v.a == 150 && v.b == 0);
    v = bench_levels(1, 1, DRIVE_B, 40); assert(v.a == 0 && v.b == 400);
    v = bench_levels(1, 1, DRIVE_B, 41); assert(v.a == 0 && v.b == 0);
    v = bench_levels(2, 0, DRIVE_A, 25); assert(v.a == 0 && v.b == 0);
    v = bench_levels(0, 0, 3, 25); assert(v.a == 0 && v.b == 0);
    BenchControl c = {.consumed = true};
    assert(!bench_start(&c, 0, true, false, DRIVE_A));
    bench_tick(&c, 0, false, false);
    assert(!bench_start(&c, 0, false, false, DRIVE_A));
    assert(!bench_start(&c, 0, true, true, DRIVE_A));
    assert(!bench_start(&c, 0, true, false, 3));
    assert(bench_start(&c, 0, true, false, DRIVE_A));
    bench_tick(&c, PULSE_US-1, true, false); assert(c.drive == DRIVE_A);
    bench_tick(&c, PULSE_US, true, false); assert(c.drive == DRIVE_OFF);
    assert(c.stop_reason == STOP_TIMEOUT && c.last_run_us == PULSE_US && c.stop_count == 1);
    uint64_t t = PULSE_US + REST_US;
    assert(!bench_start(&c, t, true, false, DRIVE_B)); /* Held: no retrigger. */
    bench_tick(&c, PULSE_US+1, false, false);
    assert(!bench_start(&c, t-1, true, false, DRIVE_B)); /* Rest interval. */
    assert(bench_start(&c, t, true, false, DRIVE_B));
    bench_tick(&c, t+1000000, true, true); assert(c.drive == DRIVE_OFF);
    assert(c.stop_reason == STOP_GP20 && c.last_run_us == 1000000 && c.stop_count == 2);
    bench_tick(&c, t+1000001, true, false);
    assert(c.stop_reason == STOP_GP20 && c.stop_count == 2); /* Preserve first cause. */
    t += 2 * REST_US;
    assert(!bench_start(&c, t, true, false, DRIVE_A)); /* Stop consumes hold. */
    bench_tick(&c, t, false, false);
    assert(bench_start(&c, t+1, true, false, DRIVE_A));
    bench_tick(&c, t+1000001, false, false); assert(c.drive == DRIVE_OFF);
    assert(c.stop_reason == STOP_GP21_RELEASE && c.last_run_us == 1000000);
    t += 2 * REST_US + 1;
    bench_tick(&c, t, false, false);
    assert(bench_start(&c, t, true, false, DRIVE_A));
    bench_stop(&c, t+50000, STOP_SERIAL);
    assert(c.drive == DRIVE_OFF && c.stop_reason == STOP_SERIAL && c.last_run_us == 50000);
    t += REST_US + 50000;
    bench_tick(&c, t, false, false);
    assert(bench_start(&c, t, true, false, DRIVE_B));
    bench_stop(&c, t+100000, STOP_USB);
    assert(c.drive == DRIVE_OFF && c.stop_reason == STOP_USB && c.last_run_us == 100000);
    BenchEncoder e = {0};
    const uint8_t forward[] = {1,3,2,0};
    const uint8_t reverse[] = {2,3,1,0};
    for (unsigned i=0;i<4;i++) bench_encoder_update(&e, forward[i]);
    assert(e.position == 4 && e.a_edges == 2 && e.b_edges == 2 && e.invalid == 0);
    for (unsigned i=0;i<4;i++) bench_encoder_update(&e, reverse[i]);
    assert(e.position == 0 && e.a_edges == 4 && e.b_edges == 4);
    bench_encoder_update(&e, 0); assert(e.position == 0);
    bench_encoder_update(&e, 3); assert(e.invalid == 1 && e.position == 0);
    e = (BenchEncoder){.position=INT32_MAX, .a_edges=UINT32_MAX, .b_edges=UINT32_MAX};
    bench_encoder_update(&e, 1);
    assert(e.position == INT32_MAX && e.b_edges == UINT32_MAX);
    e = (BenchEncoder){.position=INT32_MIN}; bench_encoder_update(&e, 2);
    assert(e.position == INT32_MIN);
    puts("PASS: one-wheel output selection, duty bounds, quadrature, invalid edges, saturation, boot inhibit, hold, stop, timeout, rearm, cooldown");
    return 0;
}
