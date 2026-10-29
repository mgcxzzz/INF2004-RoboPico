/* Buddy4 add-on glue for the v2.3 SDK firmware. Included by the v2.3 main
 * (circuit_with_barcode/main.c) after wifi_harness.h, like v2.3's own
 * harness_adapter.h / wifi_harness.h, so it can read v2.3 state. It only
 * OBSERVES v2.3: it never writes motion, encoder, coordinator or network
 * state. All Buddy4 processing lives in buddy4/buddy4_service.c.
 *
 * Scheduling: one IMU period (20 ms) runs on core 0's superloop, aligned
 * just after a 2 ms barcode-ADC sample so the I2C FIFO read (~2 ms) falls in
 * the sampling gap; it runs at most 2 ms late if no alignment occurs.
 * Run state -> Buddy4 mission: v2.3 run active in MOTION_AUTO = Start (or
 * Resume within the same guard generation); anything else = Stop; an
 * accepted v2.3 MQTT Reset = Reset (clears an IMU fault, recalibrates). */
#include "buddy4/buddy4_v23.h"

#define BUDDY4_ALIGN_US   (700u)    /* run only this soon after a side sample */
#define BUDDY4_SLACK_US   (2000u)   /* ...unless already this late            */

static int64_t  b4h_ticks[2];
static uint32_t b4h_invalid[2];
static bool     b4h_have, b4h_clean, b4h_ready, b4h_run_prev, b4h_reset_pending;
static uint64_t b4h_drained_us, b4h_due_since;
static uint32_t b4h_run_generation;

/* control_step: called right after v2.3 drains its encoders (copy e[]),
 * before its own active-run check. Observer only. Interval rule as the RTOS
 * odometry: clean = no new invalid transition and 0 < interval <= 50 ms. */
static void buddy4_odometry_note(const BenchEncoder e[2], uint64_t now, uint64_t elapsed) {
    bool clean = b4h_have && elapsed > 0u && elapsed <= 50000u &&
                 e[0].invalid == b4h_invalid[0] && e[1].invalid == b4h_invalid[1];
    for (unsigned w = 0; w < 2; ++w) { b4h_ticks[w] += e[w].position; b4h_invalid[w] = e[w].invalid; }
    b4h_clean = clean; b4h_have = true; b4h_drained_us = now;
}

/* Accepted v2.3 MQTT Reset (wifi_harness.h WP_CMD_RESET). */
static void buddy4_reset_request(void) { b4h_reset_pending = true; }

static bool buddy4_send(const uint8_t *frame, uint32_t len) {
    return wifi_forward && wifi_send(WIFI_TOPIC_TLM, frame, (size_t)len);
}

static void buddy4_init(void) {
    if (clock_get_hz(clk_sys) != 125000000u) {      /* driver timing is for 125 MHz */
        if (stdio_usb_connected()) puts("BUDDY4: disabled, clk_sys is not 125 MHz.");
        return;
    }
    b4v23_init(buddy4_send, stdio_usb_connected());
    b4h_ready = true;
    if (stdio_usb_connected())
        puts("BUDDY4 ADD-ON v0.2: powered-ramp acceptance fix; GY-511 I2C1 GP2/GP3; sources 2/4; estimator never commands motors.");
}

static void buddy4_poll(uint64_t now) {
    if (!b4h_ready || !b4v23_due(now)) { b4h_due_since = 0u; return; }
    if (b4h_due_since == 0u) b4h_due_since = now;
    if (now - last_side_sample > BUDDY4_ALIGN_US && now - b4h_due_since < BUDDY4_SLACK_US) return;
    b4h_due_since = 0u;

    MotionStatus st = nav_get_status();
    bool run = guard.active && st.session_active && st.mode == MOTION_AUTO;
    b4v23_mission_t m = B4V23_NONE;
    if (b4h_reset_pending) {
        m = B4V23_RESET; b4h_reset_pending = false;          /* run edge next period */
    } else if (run && !b4h_run_prev) {
        m = (b4h_run_generation == guard.generation) ? B4V23_RESUME : B4V23_START;
        b4h_run_generation = guard.generation; b4h_run_prev = true;
    } else if (!run && b4h_run_prev) {
        m = B4V23_STOP; b4h_run_prev = false;
    }

    b4v23_odo_t o = {0};
    if (b4h_have) {
        double l = wheel_mm(0, b4h_ticks[0]), r = wheel_mm(1, b4h_ticks[1]);
        o.have = true; o.clean = b4h_clean;
        o.age_ms = (uint32_t)((now - b4h_drained_us) / 1000u);
        o.centre_mm = (l + r) * 0.5;
        o.heading_dd = (l - r) / (double)TRACK_MM * (1800.0 / 3.14159265358979323846);
        o.left_mm_s = velocity[0]; o.right_mm_s = velocity[1];
        o.invalid_left = b4h_invalid[0]; o.invalid_right = b4h_invalid[1];
    }
    b4v23_tick(now, m, &o);
}
