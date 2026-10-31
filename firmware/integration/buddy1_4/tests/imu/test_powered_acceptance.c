/* Powered-ramp acceptance regressions. Includes the real processing unit to
 * exercise its terrain state machine at controlled post-filter boundaries.
 * No hardware or motion-controller implementation is mocked into this unit. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "src/imu/imu_proc.c"
static unsigned failures, checks;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        checks++;                                                                                  \
        if (!(x)) {                                                                                \
            failures++;                                                                            \
            printf("FAIL line %d: %s\n", __LINE__, #x);                                            \
        }                                                                                          \
    } while (0)
static b4_ctx_t c;
static b4_in_t in;
static b4_out_t out;
static void start(void) {
    b4_reset(&c);
    memset(&in, 0, sizeof(in));
    c.state = IMU_ST_HUMP;
    c.b_mission = true;
    c.b_rates = true;
    c.side_candidate = true;
    c.side_rising = true;
    c.side_level_ready = true;
    c.side_peak_mm = 10;
    c.side_last_h_mm = 10;
    c.side_have_odo = true;
    c.side_start_us = 200000;
    c.side_last_dist = 20;
    c.speed_mm_s = 45;
    in.t_us = 200000;
    in.gap_us = 20000;
    in.b_motion = true;
    in.distance_mm = 20;
}
static void step(int height, int reason, int accel, int turn, int heading_delta, bool fresh) {
    memset(&out, 0, sizeof(out));
    in.t_us += 20000;
    in.distance_mm++;
    in.heading_dd = (int16_t)(in.heading_dd + heading_delta);
    in.b_motion = fresh;
    c.side_height_mm = (int16_t)height;
    c.side_quality_reason = (uint8_t)reason;
    c.accel_mm_s2 = accel;
    c.turn_dd_s = turn;
    side_terrain(&c, &in, reason == 0, &out);
}
static void finish(void) {
    for (unsigned i = 0; i < 20; i++)
        step(0, 0, 0, 0, 0, true);
}
/* Full b4_tick pipeline: encoder stair-steps with low average yaw must not
 * abort a 10 mm profile. A sustained real turn still must not be accepted. */
static unsigned pipeline_profile(bool real_turn) {
    b4_ctx_t ctx;
    b4_reset(&ctx);
    ctx.state = IMU_ST_TRACKING;
    ctx.b_mission = true;
    ctx.g_mg = 1000;
    ctx.base[2] = 1000;
    ctx.lp[2] = 1000 * 256;
    int32_t um = 0;
    int16_t heading = 0;
    for (unsigned tick = 0; tick < 100; tick++) {
        b4_in_t v = {0};
        b4_out_t result = {0};
        bool raised = tick >= 25 && tick < 60;
        um += 900;
        if (real_turn)
            heading += 10;
        else if (tick % 4 == 0)
            heading += 5;
        v.b_motion = true;
        v.b_fine_motion = true;
        v.distance_um = um;
        v.distance_mm = (int16_t)(um / 1000);
        v.heading_dd = heading;
        v.t_us = (tick + 1) * 20000;
        v.gap_us = 20000;
        v.b_read_ok = true;
        v.n = 8;
        for (unsigned j = 0; j < v.n; j++) {
            int16_t noise = (int16_t)((tick + j) % 5) - 2;
            v.s[j].fwd_mg = noise;
            v.s[j].lat_mg = (raised ? 89 : 0) + noise;
            v.s[j].up_mg = (raised ? 996 : 1000) + noise;
        }
        b4_tick(&ctx, &v, &result);
    }
    if (!real_turn)
        CHECK(ctx.max_height_mm >= 9 && ctx.max_height_mm <= 11);
    return ctx.humps;
}
int main(void) {
    /* Capture1: fresh clean encoders, smoothed turn2.4deg/s, but one
     * quantised20ms heading increment exceeds the raw4-deci-degree gate. */
    start();
    step(10, 0, 0, 24, 5, true);
    CHECK(c.side_candidate && c.rejected == 0);
    CHECK(!(c.side_quality_reason & B4_Q_TURN));
    for (unsigned i = 0; i < 15; i++)
        step(10, 0, 0, 24, 0, true);
    finish();
    CHECK(c.humps == 1 && c.last_height_mm == 10);
    /* Capture2: transient longitudinal acceleration210..306mm/s2.
     * Excluded height must never increase peak or prove ground return. */
    start();
    step(70, B4_Q_ACCEL, 210, 20, 0, true);
    step(70, B4_Q_ACCEL, 306, 20, 0, true);
    CHECK(c.side_candidate && c.side_skip_us == 40000);
    CHECK(c.side_peak_mm == 10);
    CHECK(c.side_return_us == 0);
    for (unsigned i = 0; i < 15; i++)
        step(10, 0, 0, 20, 0, true);
    finish();
    CHECK(c.humps == 1 && c.last_height_mm == 10);
    start();
    for (unsigned i = 0; i < 5; i++)
        step(70, B4_Q_ACCEL, 250, 0, 0, true);
    CHECK(c.side_candidate);
    step(70, B4_Q_ACCEL, 250, 0, 0, true);
    CHECK(!c.side_candidate && c.rejected == 1 && c.humps == 0);
    start();
    for (unsigned i = 0; i < 15; i++)
        step(10, 0, 0, 0, 0, true);
    for (unsigned i = 0; i < 4; i++) {
        step(70, B4_Q_ACCEL, 250, 0, 0, true);
        step(10, 0, 0, 0, 0, true);
    }
    /* Force coverage accounting above25% even if the return is long. */
    c.side_skip_us = 1000000;
    finish();
    CHECK(c.humps == 0 && c.rejected == 1);
    /* Strict failure paths stay strict. */
    start();
    step(10, 0, 0, 0, 0, false);
    CHECK(!c.side_candidate && c.humps == 0);
    start();
    c.side_last_dist = 50;
    step(10, 0, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    step(10, B4_Q_TURN, 0, 201, 5, true);
    CHECK(!c.side_candidate);
    start();
    step(10, B4_Q_GRAVITY, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    step(10, B4_Q_TIMING, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    step(10, B4_Q_IMPACT, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    step(10, B4_Q_UNAVAILABLE, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    in.b_overrun = true;
    step(10, 0, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    c.b_mission = false;
    step(10, 0, 0, 0, 0, true);
    CHECK(!c.side_candidate);
    start();
    c.side_start_us = 0;
    in.t_us = B4_SIDE_TIMEOUT_MS * 1000U;
    step(10, 0, 0, 0, 0, true);
    CHECK(!c.side_candidate && c.humps == 0);
    /* At rest a tilt cannot satisfy the minimum travel requirement. */
    start();
    c.side_travel_mm = 0;
    in.distance_mm = 20;
    for (unsigned i = 0; i < 12; i++) {
        in.distance_mm--;
        step(0, 0, 0, 0, 0, true);
    }
    CHECK(c.humps == 0 && c.rejected == 1);
    CHECK(pipeline_profile(false) == 1);
    CHECK(pipeline_profile(true) == 0);
    printf("powered acceptance: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
