#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "src/imu/imu_lift_guard.h"

static const int16_t k_base[3] = {0, 0, 1000};

static void feed(imu_lift_guard_t *guard, imu_lift_vec_t sample, uint8_t count,
                 uint32_t now_ms, bool fresh) {
    imu_lift_vec_t samples[B4_FIFO_DEPTH];
    assert(count <= B4_FIFO_DEPTH);
    for (uint8_t i = 0; i < count; ++i)
        samples[i] = sample;
    imu_lift_guard_batch(guard, k_base, 1000, samples, count, B4_SAMPLE_US, now_ms, fresh);
}

static void test_idle_upright_and_reset_readiness(void) {
    imu_lift_guard_t guard;
    imu_lift_guard_init(&guard);
    assert(!guard.armed && !guard.latched && !imu_lift_guard_reset_ready(&guard));

    /* A typical 30-degree road ramp remains below the upset threshold. */
    feed(&guard, (imu_lift_vec_t){.lat_mg = 500, .up_mg = 866}, 32, 100, true);
    feed(&guard, (imu_lift_vec_t){.lat_mg = 500, .up_mg = 866}, 32, 180, true);
    assert(guard.armed && !guard.latched && guard.reason == 0U);

    /* Upright samples arm Start/reset only after 500 ms of fresh observations. */
    for (uint32_t t = 0; t < 6; ++t)
        feed(&guard, (imu_lift_vec_t){.up_mg = 1000}, 32, 300 + t * 80, true);
    feed(&guard, (imu_lift_vec_t){.up_mg = 1000}, 8, 780, true);
    assert(guard.upright_samples == IMU_LIFT_UPRIGHT_SAMPLES);
    assert(imu_lift_guard_reset_ready(&guard));

    /* Failed/empty/late/overrun batches break consecutive dwell but preserve
     * the calibrated baseline's armed state and never clear a latched event. */
    feed(&guard, (imu_lift_vec_t){.up_mg = 1000}, 1, 800, false);
    assert(guard.armed && !guard.current_valid && guard.upright_samples == 0U);
    assert(!imu_lift_guard_reset_ready(&guard));
}

static void test_upward_acceleration_dwell_and_latch(void) {
    imu_lift_guard_t guard;
    imu_lift_guard_init(&guard);

    /* A single high-g tap is not a lift. Normal acceleration below +0.4g also
     * remains below the upward-pickup threshold. */
    feed(&guard, (imu_lift_vec_t){.up_mg = 3000}, 1, 1000, true);
    feed(&guard, (imu_lift_vec_t){.up_mg = 1300}, 12, 1030, true);
    assert(!guard.latched);

    /* Require twelve consecutive samples (30 ms) at >= baseline + 400 mg. */
    feed(&guard, (imu_lift_vec_t){.up_mg = 1500}, 11, 1060, true);
    assert(!guard.latched && guard.up_samples == 11U);
    feed(&guard, (imu_lift_vec_t){.up_mg = 1500}, 1, 1062, true);
    assert(guard.latched && guard.reason == IMU_LIFT_REASON_UP_ACCEL);
    assert(guard.event_count == 1U && guard.event_time_ms == 1062U);
    assert(guard.up_projection_mg == 1500 && guard.norm_mg == 1500U);

    /* The first event is retained, even if more qualifying batches arrive. */
    feed(&guard, (imu_lift_vec_t){.up_mg = 1700}, 8, 1080, true);
    assert(guard.event_count == 1U && guard.event_time_ms == 1062U);

    /* Reset is blocked until 500 ms of fresh upright samples, then clears the
     * latch while retaining the saturating lifetime event counter. */
    assert(!imu_lift_guard_clear_latch(&guard));
    for (uint32_t t = 0; t < 6; ++t)
        feed(&guard, (imu_lift_vec_t){.up_mg = 1000}, 32, 1160 + t * 80, true);
    feed(&guard, (imu_lift_vec_t){.up_mg = 1000}, 8, 1640, true);
    assert(imu_lift_guard_clear_latch(&guard));
    assert(!guard.latched && guard.reason == 0U && guard.event_time_ms == 0U);
    assert(guard.event_count == 1U);
}

static void test_steep_tilt_and_ramp_negative(void) {
    imu_lift_guard_t guard;
    imu_lift_guard_init(&guard);

    /* Sustained 30-degree incline: no false upset. */
    feed(&guard, (imu_lift_vec_t){.lat_mg = 500, .up_mg = 866}, 32, 2000, true);
    feed(&guard, (imu_lift_vec_t){.lat_mg = 500, .up_mg = 866}, 32, 2080, true);
    assert(!guard.latched && guard.tilt_samples == 0U);

    /* Approximately 61 degrees with a gravity-like magnitude qualifies only
     * after twenty consecutive samples (50 ms); 60 degrees is excluded. */
    imu_lift_vec_t sixty = {.lat_mg = 866, .up_mg = 500};
    feed(&guard, sixty, 20, 2140, true);
    assert(!guard.latched);
    imu_lift_vec_t steep = {.lat_mg = 875, .up_mg = 485};
    feed(&guard, steep, 19, 2190, true);
    assert(!guard.latched && guard.tilt_samples == 19U);
    feed(&guard, steep, 1, 2192, true);
    assert(guard.latched && guard.reason == IMU_LIFT_REASON_TILT);
    assert(guard.event_count == 1U && guard.norm_mg >= 999U && guard.norm_mg <= 1001U);
}

static void test_invalid_batch_and_metric_saturation(void) {
    imu_lift_guard_t guard;
    imu_lift_guard_init(&guard);
    feed(&guard, (imu_lift_vec_t){.up_mg = 1500}, 8, 3000, true);
    uint8_t before = (uint8_t)guard.up_samples;
    assert(before == 8U && !guard.latched);

    /* A timing gap/overrun/failure gives no dwell credit. */
    feed(&guard, (imu_lift_vec_t){.up_mg = 1500}, 8, 3020, false);
    assert(guard.armed && !guard.current_valid && guard.up_samples == 0U);
    feed(&guard, (imu_lift_vec_t){.up_mg = 1500}, 8, 3040, true);
    assert(guard.up_samples == 8U && !guard.latched);

    /* C9 diagnostic values use native signed-13/unsigned-12 limits and flag
     * clipping instead of wrapping. */
    feed(&guard, (imu_lift_vec_t){.up_mg = -5000}, 1, 3042, true);
    assert(guard.up_projection_mg == -4096);
    assert(guard.norm_mg == 4095U && guard.saturated);
}

int main(void) {
    test_idle_upright_and_reset_readiness();
    test_upward_acceleration_dwell_and_latch();
    test_steep_tilt_and_ramp_negative();
    test_invalid_batch_and_metric_saturation();
    puts("PASS IMU lift guard: ramps/spikes excluded, upward and >60 degree dwell, reset readiness, saturation");
    return 0;
}
