#include <assert.h>
#include <stdio.h>
#include "bench_logic.h"
int main(void) {
    BenchControl c = {.consumed = true};
    assert(!bench_start(&c, 0, true, false, DRIVE_A));
    bench_tick(&c, 0, false, false);
    assert(!bench_start(&c, 0, false, false, DRIVE_A));
    assert(!bench_start(&c, 0, true, true, DRIVE_A));
    assert(!bench_start(&c, 0, true, false, 3));
    assert(bench_start(&c, 0, true, false, DRIVE_A));
    bench_tick(&c, PULSE_US-1, true, false); assert(c.drive == DRIVE_A);
    bench_tick(&c, PULSE_US, true, false); assert(c.drive == DRIVE_OFF);
    assert(!bench_start(&c, 2000000, true, false, DRIVE_B)); /* Held: no retrigger. */
    bench_tick(&c, 500001, false, false);
    assert(!bench_start(&c, 600000, true, false, DRIVE_B)); /* Rest interval. */
    assert(bench_start(&c, 1500000, true, false, DRIVE_B));
    bench_tick(&c, 1500001, true, true); assert(c.drive == DRIVE_OFF);
    assert(!bench_start(&c, 3000000, true, false, DRIVE_A)); /* Stop consumes hold. */
    bench_tick(&c, 3000000, false, false);
    assert(bench_start(&c, 3000001, true, false, DRIVE_A));
    bench_tick(&c, 3000002, false, false); assert(c.drive == DRIVE_OFF);
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
    puts("PASS: quadrature, invalid edges, saturation, boot inhibit, hold, stop, timeout, rearm, cooldown");
    return 0;
}
