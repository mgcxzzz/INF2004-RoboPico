#define BUDDY_COORDINATED_BARCODE 1
#define BARCODE_PEAK_ACTIVE 1
#include "src/motion/motion_port.h"
#define main existing_peak_tests_main
#include "tests/barcode/test_peak_integration.c"
#undef main
int main(void) {
    now_us = 1000000;
    MotionPort port = {0};
    motion_init(&port);
    motion_session_begin(now_us);
    begin_main();
    const BpCapture *c = &bp_captures[CAP_Z];
    unsigned i = 0;
    for (; i < c->n && c->path100[i] < 10000; ++i)
        step(c->path100[i] / 100.0, c->adc[i], true, &nav, &side_analog);
    CHECK(bp_state.nbars > 0, "setup partial barcode");
    now_us += 30000000;
    double wheel[2] = {100, 100};
    barcode_reader_reset(&nav.reader, side_analog.black, now_us, 100);
    barcode_analog_begin(&side_analog, &nav, 175, now_us, wheel);
    barcode_partial_reset(); /* same hook as the application's Resume path */
    CHECK(bp_state.nbars == 0, "resume resets the peak reader too");
    for (; i < c->n; ++i)
        step(c->path100[i] / 100.0, c->adc[i], true, &nav, &side_analog);
    CHECK(nav.decoded_total == 0 && nav.pending == 0,
          "a frame cannot complete across the interruption");
    begin_main();
    feed_capture(CAP_Z, 1e9, true, &nav, &side_analog);
    CHECK(nav.decoded_total == 1 && nav.last_code == 'Z',
          "a fresh complete frame still works after reset");
    barcode_partial_reset();
    CHECK(nav.pending == 'Z', "already verified pending route is preserved");
    printf("%s barcode handback lifecycle: %d checks\n", fails ? "FAIL" : "PASS", checks);
    return fails != 0;
}
