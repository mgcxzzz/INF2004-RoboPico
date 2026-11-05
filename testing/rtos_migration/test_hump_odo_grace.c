/* Focused RTOS-only regression for bounded odometry-gap hump grace. */
#ifndef APP_RTOS_IMU
#define APP_RTOS_IMU 1
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "src/imu/imu_proc.c"

void ri_calibration_restart(uint8_t reasons, const b4_ctx_t *ctx, const int16_t mean[3]) {
    (void)reasons;
    (void)ctx;
    (void)mean;
}

static b4_ctx_t c;
static b4_in_t in;
static b4_out_t out;

static void reset_fixture(bool level_ready) {
    b4_reset(&c);
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    c.state = IMU_ST_TRACKING;
    c.b_mission = true;
    c.side_level_ready = level_ready;
    c.side_level_us = level_ready ? (B4_SIDE_RETURN_MS * 1000U) : 0U;
    c.side_have_odo = true;
    c.side_last_dist = 100;
    c.side_last_head = 0;
    c.speed_mm_s = 45;
    in.t_us = 1000000U;
    in.gap_us = 20000U;
    in.b_motion = true;
    in.distance_mm = 100;
}

static void step_side(int16_t height_mm, bool b_motion, int16_t distance_delta,
                      int16_t heading_delta, bool b_quality, uint8_t quality_reason) {
    in.t_us += in.gap_us;
    if (b_motion) {
        in.distance_mm = (int16_t)(in.distance_mm + distance_delta);
        in.heading_dd = (int16_t)(in.heading_dd + heading_delta);
    }
    in.b_motion = b_motion;
    in.b_overrun = false;
    c.side_height_mm = height_mm;
    c.side_quality_reason = quality_reason | (b_motion ? 0U : B4_Q_UNAVAILABLE);
    memset(&out, 0, sizeof(out));
    side_terrain(&c, &in, b_quality, &out);
}

static void start_candidate(bool rising) {
    c.side_candidate = true;
    c.side_rising = rising;
    c.state = rising ? IMU_ST_HUMP : IMU_ST_TRACKING;
    c.side_level_ready = true;
    c.side_peak_mm = 10;
    c.side_last_h_mm = 10;
    c.side_start_us = in.t_us;
    c.side_last_dist = in.distance_mm;
    c.side_last_head = in.heading_dd;
    c.side_have_odo = true;
    c.side_travel_mm = 0;
    c.side_up_us = rising ? (B4_SIDE_START_MS * 1000U) : 0U;
}

static void clean_hump(void) {
    reset_fixture(true);
    step_side(10, true, 1, 0, true, 0U); /* arm candidate */
    assert(c.side_candidate && !c.side_rising);
    step_side(10, true, 1, 0, true, 0U);
    step_side(10, true, 1, 0, true, 0U);
    assert(c.side_rising);
    for (unsigned i = 0U; i < 11U; i++) {
        step_side(10, true, 1, 0, true, 0U);
    }
    for (unsigned i = 0U; i < 10U; i++) {
        step_side(0, true, 1, 0, true, 0U);
    }
    assert(c.humps == 1U && c.last_height_mm == 10U);
}

static void test_level_and_clean_profiles(void) {
    reset_fixture(false);
    for (unsigned i = 0U; i < 100U; i++) {
        step_side(0, true, 1, 0, true, 0U);
    }
    assert(c.humps == 0U && !c.side_candidate && c.side_level_ready);
    clean_hump();
}

static void test_prearm_gap_and_continuity(void) {
    reset_fixture(true);
    step_side(40, false, 0, 0, true, 0U);
    assert(c.side_level_ready && !c.side_candidate);
    assert(c.side_diag_hold_count == 1U);
    step_side(0, true, 1, 0, true, 0U);
    assert(c.side_level_ready && c.side_diag_recover_count == 1U);
    assert(c.side_odo_gap_last_ms == 20U);
    step_side(10, true, 1, 0, true, 0U);
    assert(c.side_candidate);

    reset_fixture(true);
    start_candidate(true);
    for (unsigned i = 0U; i < 5U; i++) {
        step_side(10, false, 0, 0, true, 0U);
    }
    assert(c.side_candidate && c.side_odo_gap_last_ms == B4_SIDE_SKIP_RUN_MS);
    step_side(10, true, 1, 0, true, 0U);
    assert(c.side_candidate && c.side_diag_recover_count == 1U);

    reset_fixture(false);
    c.side_level_us = 180000U;
    step_side(0, false, 0, 0, true, 0U);
    assert(!c.side_level_ready && c.side_level_us == 0U);
    step_side(0, true, 1, 0, true, 0U);
    assert(!c.side_level_ready && c.side_level_us == 20000U);

    reset_fixture(true);
    for (unsigned i = 0U; i < 6U; i++) {
        step_side(0, false, 0, 0, true, 0U);
    }
    assert(!c.side_level_ready);
    step_side(0, true, 1, 5, true, 0U);
    assert(!c.side_level_ready && c.side_level_us == 20000U);
    step_side(0, true, 1, 5, true, 0U);
    assert(!c.side_level_ready && c.side_level_us == 40000U);
}

static void test_candidate_pause_and_rebase(void) {
    reset_fixture(true);
    start_candidate(true);
    c.side_peak_mm = 12;
    c.side_last_h_mm = 12;
    c.side_travel_mm = 4;
    c.side_returning = true;
    c.side_return_us = 40000U;
    step_side(70, false, 0, 0, true, 0U);
    assert(c.side_candidate && c.side_rising);
    assert(c.side_peak_mm == 12 && c.side_travel_mm == 4);
    assert(c.side_up_us == (B4_SIDE_START_MS * 1000U));
    assert(c.side_return_us == 40000U && c.side_returning);
    assert(c.side_skip_us == 20000U && c.side_diag_hold_count == 1U);

    step_side(12, true, 20, 0, true, 0U); /* recovery motion is excluded */
    assert(c.side_candidate && c.side_peak_mm == 12 && c.side_travel_mm == 4);
    assert(c.side_diag_recover_count == 1U && c.side_odo_gap_last_ms == 20U);

    b4_ctx_t motion;
    b4_in_t sample = {0};
    b4_reset(&motion);
    motion.hump.d_mm = 400;
    motion.head_dd = 200;
    motion.b_have_last = true;
    motion.last_dist = 100;
    motion.last_head = 300;
    motion.hist_n = B4_RATE_N;
    sample.t_us = 2000000U;
    sample.b_motion = false;
    update_motion(&motion, &sample);
    assert(motion.hist_n == 0U && !motion.b_rates);
    sample.t_us += 20000U;
    sample.b_motion = true;
    sample.distance_mm = 120;
    sample.heading_dd = 320;
    update_motion(&motion, &sample);
    assert(motion.hump.d_mm == 400 && motion.head_dd == 200);
    assert(motion.last_dist == 120 && motion.last_head == 320 && !motion.b_rates);
    sample.t_us += 20000U;
    sample.distance_mm = 121;
    sample.heading_dd = 325;
    update_motion(&motion, &sample);
    assert(motion.hump.d_mm == 401 && motion.head_dd == 205);
    assert(motion.hist_n == 2U && !motion.b_rates);
}

static void test_gap_deadline_and_budget(void) {
    reset_fixture(true);
    start_candidate(true);
    for (unsigned i = 0U; i < 6U; i++) {
        step_side(10, false, 0, 0, true, 0U);
    }
    assert(!c.side_candidate && c.humps == 0U);
    assert(c.side_diag_disarm_count == 1U);
    assert(c.side_diag_last_clear_reason == B4_SIDE_CLEAR_ODO_GAP_LONG);
    assert(c.side_odo_gap_last_ms == 120U);

    reset_fixture(true);
    start_candidate(true);
    for (unsigned i = 0U; i < 2U; i++) {
        step_side(10, true, 1, 0, true, 0U);
    }
    for (unsigned episode = 0U; episode < 3U; episode++) {
        for (unsigned i = 0U; i < 4U; i++) {
            step_side(60, false, 0, 0, true, 0U);
        }
        step_side(10, true, 4, 0, true, 0U);
    }
    for (unsigned i = 0U; i < 8U; i++) {
        step_side(10, true, 1, 0, true, 0U);
    }
    for (unsigned i = 0U; i < 10U && c.side_candidate; i++) {
        step_side(0, true, 1, 0, true, 0U);
    }
    assert(!c.side_candidate && c.humps == 0U && c.rejected == 1U);
    assert(c.side_diag_last_clear_reason == B4_SIDE_CLEAR_SKIP_BUDGET);
    assert(c.side_diag_last_failure_reason == B4_SIDE_CLEAR_SKIP_BUDGET);

    reset_fixture(true);
    start_candidate(true);
    c.side_travel_mm = 5;
    for (unsigned i = 0U; i < 4U; i++) {
        step_side(10, false, 0, 0, true, 0U);
    }
    step_side(10, true, 1, 0, false, B4_Q_ACCEL);
    assert(c.side_candidate && c.side_skip_run_us == 100000U && c.side_travel_mm == 5);
    step_side(10, true, 1, 0, false, B4_Q_ACCEL);
    assert(!c.side_candidate && c.side_travel_mm == 0); /* rejected profile is cleared */
    assert(c.side_diag_last_clear_reason == B4_SIDE_CLEAR_MIXED_SKIP_RUN);

    reset_fixture(true);
    start_candidate(true);
    c.side_returning = true;
    c.side_return_us = 180000U;
    c.side_start_us = in.t_us;
    in.t_us = c.side_start_us + (B4_SIDE_TIMEOUT_MS * 1000U) - 10000U;
    step_side(0, true, 1, 0, true, 0U);
    assert(!c.side_candidate && c.humps == 0U);
    assert(c.side_diag_last_clear_reason == B4_SIDE_CLEAR_TIMEOUT);
}

static void test_turn_reverse_stop_and_rate_warmup(void) {
    reset_fixture(true);
    start_candidate(false);
    step_side(10, true, -1, 0, true, B4_Q_REVERSE);
    assert(!c.side_candidate && c.side_diag_last_clear_reason == B4_SIDE_CLEAR_REVERSE);

    reset_fixture(true);
    start_candidate(false);
    c.b_rates = true;
    c.turn_dd_s = B4_TURN_DD_S + 1;
    step_side(10, true, 1, 0, true, B4_Q_TURN);
    assert(!c.side_candidate && c.side_diag_last_clear_reason == B4_SIDE_CLEAR_TURN);

    reset_fixture(true);
    start_candidate(true);
    step_side(10, false, 0, 0, true, 0U);
    step_side(10, true, 2, 9, true, 0U); /* outage yaw exceeds the 20 deg/s bound */
    assert(!c.side_candidate && c.side_diag_last_clear_reason == B4_SIDE_CLEAR_TURN);

    reset_fixture(true);
    start_candidate(true);
    c.side_diag_last_failure_reason = B4_SIDE_CLEAR_TURN;
    b4_mission(&c, B4_MISSION_STOP, in.t_us, &out);
    assert(!c.side_candidate && !c.side_level_ready);
    assert(c.side_diag_last_clear_reason == B4_SIDE_CLEAR_STOP_RESET);
    assert(c.side_diag_last_failure_reason == B4_SIDE_CLEAR_TURN);

    reset_fixture(true);
    start_candidate(true);
    c.side_odo_rate_guard_active = 1U;
    c.side_odo_rate_anchor_head = in.heading_dd;
    c.side_odo_rate_anchor_us = in.t_us;
    step_side(10, true, 1, 5, true, 0U);
    assert(c.side_candidate); /* one quantized 5 dd step is below the 200ms average */

    reset_fixture(true);
    start_candidate(true);
    c.side_odo_rate_guard_active = 1U;
    c.side_odo_rate_anchor_head = in.heading_dd;
    c.side_odo_rate_anchor_us = in.t_us;
    step_side(10, true, 1, 50, true, 0U);
    assert(!c.side_candidate && c.side_diag_last_clear_reason == B4_SIDE_CLEAR_TURN);

    reset_fixture(true);
    start_candidate(true);
    c.side_odo_rate_guard_active = 1U;
    c.side_odo_rate_anchor_head = in.heading_dd;
    c.side_odo_rate_anchor_us = in.t_us;
    for (unsigned i = 0U; i < 10U && c.side_candidate; i++) {
        step_side(10, true, 1, 5, true, 0U);
    }
    assert(!c.side_candidate && c.side_diag_last_clear_reason == B4_SIDE_CLEAR_TURN);

    reset_fixture(true);
    start_candidate(true);
    c.side_odo_rate_guard_active = 1U;
    c.side_odo_rate_anchor_head = in.heading_dd;
    c.side_odo_rate_anchor_us = in.t_us;
    for (unsigned i = 0U; i < 2U; i++) {
        step_side(10, false, 0, 0, true, 0U);
        step_side(10, true, 2, 7, true, 0U);
        assert(c.side_candidate && c.side_odo_rate_anchor_head == 0);
    }
    step_side(10, false, 0, 0, true, 0U);
    step_side(10, true, 2, 10, true, 0U);
    assert(!c.side_candidate && c.side_diag_last_clear_reason == B4_SIDE_CLEAR_TURN);

    reset_fixture(true);
    start_candidate(true);
    step_side(10, false, 0, 0, true, 0U);
    step_side(10, true, 2, 7, true, 0U);
    assert(c.side_candidate && c.side_odo_rate_guard_active);
    in.heading_dd = 0; /* invalid odo snapshot carries a placeholder zero */
    step_side(10, false, 0, 0, true, 0U);
    assert(c.side_candidate && (c.side_quality_reason & B4_Q_TURN) == 0U);
    step_side(10, true, 2, 7, true, 0U);
    assert(c.side_candidate && c.side_diag_recover_count == 2U);
}

static void test_unconfirmed_clear_and_outage_reverse(void) {
    reset_fixture(true);
    start_candidate(false);
    step_side(0, true, 1, 0, true, 0U);
    assert(!c.side_candidate && c.side_diag_early_clear_count == 1U);
    assert(c.side_diag_last_failure_reason == B4_SIDE_CLEAR_HEIGHT_BEFORE_RISE);
    assert(c.rejected == 0U && c.humps == 0U && out.n_events == 0U);
    step_side(0, true, 1, 0, true, 0U);
    assert(c.side_diag_early_clear_count == 1U);

    reset_fixture(true);
    start_candidate(false);
    for (unsigned i = 0; i < 7; ++i)
        step_side(10, false, 0, 0, true, 0U);
    assert(!c.side_candidate && c.side_diag_early_clear_count == 1U);
    assert(c.rejected == 0U && c.humps == 0U && out.n_events == 0U);

    reset_fixture(true);
    start_candidate(true);
    step_side(10, false, 0, 0, true, 0U);
    step_side(10, true, -2, 0, true, 0U);
    assert(!c.side_candidate && c.side_diag_last_failure_reason == B4_SIDE_CLEAR_REVERSE);
}

static void test_complete_hump_with_short_gaps(void) {
    reset_fixture(true);
    step_side(60, false, 0, 0, true, 0U); /* no invalid peak/start credit */
    step_side(0, true, 1, 0, true, 0U);
    for (unsigned i = 0; i < 12; ++i)
        step_side(10, true, 1, 0, true, 0U);
    assert(c.side_rising && c.side_peak_mm == 10);
    int32_t travel = c.side_travel_mm;
    step_side(70, false, 0, 0, true, 0U);
    step_side(10, true, 20, 0, true, 0U);
    assert(c.side_peak_mm == 10 && c.side_travel_mm == travel);
    for (unsigned i = 0; i < 8; ++i)
        step_side(10, true, 1, 0, true, 0U);
    for (unsigned i = 0; i < 2; ++i)
        step_side(0, true, 1, 0, true, 0U);
    step_side(70, false, 0, 0, true, 0U); /* return timer pauses too */
    assert(c.side_return_us == 40000U);
    for (unsigned i = 0; i < 8; ++i)
        step_side(0, true, 1, 0, true, 0U);
    assert(c.humps == 1 && c.rejected == 0 && c.max_height_mm == 10);
    assert(c.side_diag_hold_count == 3 && c.side_diag_recover_count == 3);
}

static void test_actual_shock_cancels_held_candidate(void) {
    reset_fixture(true);
    start_candidate(true);
    step_side(10, false, 0, 0, true, 0U);
    c.base[AX_UP] = 1000;
    c.g_mg = 1000;
    c.lp[AX_UP] = 1000 * LP_ONE;
    in.t_us += in.gap_us;
    in.b_read_ok = true;
    in.n = 8;
    for (unsigned i = 0; i < in.n; ++i)
        in.s[i] = (b4_vec_t){.fwd_mg = 5000, .up_mg = 1000, .lat_mg = 0};
    memset(&out, 0, sizeof(out));
    b4_tick(&c, &in, &out);
    assert(!c.side_candidate && !c.side_level_ready && c.humps == 0);
    assert(c.side_diag_last_failure_reason == B4_SIDE_CLEAR_SHOCK);
    assert(c.state == IMU_ST_IMPACT);
}

int main(void) {
    test_level_and_clean_profiles();
    test_prearm_gap_and_continuity();
    test_candidate_pause_and_rebase();
    test_gap_deadline_and_budget();
    test_turn_reverse_stop_and_rate_warmup();
    test_unconfirmed_clear_and_outage_reverse();
    test_complete_hump_with_short_gaps();
    test_actual_shock_cancels_held_candidate();
    puts("PASS RTOS hump odometry grace: flat/clean hump, prearm continuity, bounded hold, zero uncertain travel, 25% and 8s rejection, reverse/turn/stop safety, 200ms heading rebase.");
    return 0;
}
