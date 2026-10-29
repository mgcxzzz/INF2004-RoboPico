/* v2.3 + Buddy4 ramp-assist v0.2 glue (unchanged from v0.1 apart from the
 * detector-version flag bits). Included by the v2.3 main
 * (circuit_with_barcode/main.c) after buddy4_harness.h, like the other
 * harness headers, so it can read v2.3 state.
 *
 * It only sets ContinuousDrive.boost_frac/boost_cap (the existing
 * continuous drive controller, profile 1/3 ordinary line following) and
 * sends a source-15 telemetry frame. Targets, gains, PWM/I/slew limits,
 * steering, line search, junctions, barcode, coordinator, stop paths and
 * MQTT are not modified.
 *
 * RAMP_ASSIST_ACTIVE=0 (shadow build): the decision logic, timing and
 * telemetry run identically but boost_frac stays 0, so the motors get the
 * original v2.3 output; telemetry reports the PWM that WOULD be added. */
#include "ramp_assist.h"
#ifndef RAMP_ASSIST_ACTIVE
#define RAMP_ASSIST_ACTIVE 0
#endif
#define RA_REPORT_US 500000u

static RampAssist ra_state;
static uint32_t   ra_generation;
static bool       ra_run_seen;
static uint64_t   ra_last_report_us;
static uint8_t    ra_seq, ra_max_i[2], ra_sat_ticks;
static int16_t    ra_pwm_now[2], ra_peak_pwm;

static int16_t ra_added_pwm(unsigned w, float alpha_milli) {
    const ContinuousDrive *s = &continuous_wheel[w];
    if (!s->active || alpha_milli <= 0.0f) return 0;
    float v = continuous_feedforward(w, s->reference) * alpha_milli / 1000.0f;
    if (v > (float)RA_PWM_CAP) v = (float)RA_PWM_CAP;
    return (int16_t)(v < 0.0f ? 0.0f : v + 0.5f);
}

static void ra_report(uint64_t now, uint8_t event) {
    RampAssistTlm t = {0};
    uint8_t frame[RA_TLM_FRAME_BYTES];
    t.uptime_ms = (uint32_t)(now / 1000u);
    t.event = event;
    t.flags = (uint8_t)((RAMP_ASSIST_ACTIVE ? 1u : 0u) | (ra_state.assisting ? 2u : 0u) |
                        (ra_state.latched ? 4u : 0u) | (ra_state.last_valid ? 8u : 0u) |
                        (uint8_t)((RA_DETECTOR_VERSION & 7u) << 4) |
                        0x80u); /* Buddy4 powered-acceptance estimator v0.2 */
    t.last_end = ra_state.last_end;
    t.activations = ra_state.activations;
    t.alpha_milli = (uint16_t)(ra_state.alpha_milli + 0.5f);
    t.pwm_l = ra_pwm_now[0]; t.pwm_r = ra_pwm_now[1];
    t.height_mm = ra_state.last_valid ? ra_i8(ra_state.last_mm) : (int8_t)-128;
    t.peak_mm = ra_i8(ra_state.peak_mm);
    t.int_max = ra_state.int_valid ? ra_i8(ra_state.int_max_mm) : (int8_t)-128;
    t.int_min = ra_state.int_valid ? ra_i8(ra_state.int_min_mm) : (int8_t)127;
    t.valid = ra_state.int_valid; t.invalid = ra_state.int_invalid; t.longest = ra_state.int_longest;
    t.max_i_l = ra_max_i[0]; t.max_i_r = ra_max_i[1]; t.sat_ticks = ra_sat_ticks;
    t.active_ms = (uint16_t)(ra_state.assisting ? (now - ra_state.active_since_us) / 1000u > 65535u ? 65535u :
                             (now - ra_state.active_since_us) / 1000u : (ra_state.active_ms > 65535u ? 65535u : ra_state.active_ms));
    t.peak_pwm = ra_peak_pwm;
    ra_tlm_pack(&t, ra_seq++, frame);
    if (wifi_forward) (void)wifi_send(WIFI_TOPIC_TLM, frame, sizeof(frame));
    ra_interval_reset(&ra_state);
    ra_max_i[0] = ra_max_i[1] = 0; ra_sat_ticks = 0;
    ra_last_report_us = now;
}

/* control_step, run active: after the existing mode selection and before the
 * per-wheel drive steps. `following` = continuous drive selected and no
 * line-recovery (white) state. */
static void ramp_assist_control(uint64_t now, float dt, bool following) {
    if (!ra_run_seen || guard.generation != ra_generation) {
        ra_reset(&ra_state); ra_generation = guard.generation; ra_run_seen = true;
        ra_peak_pwm = 0; ra_pwm_now[0] = ra_pwm_now[1] = 0; ra_max_i[0] = ra_max_i[1] = 0; ra_sat_ticks = 0;
        ra_last_report_us = now;
    }
    /* Observation of the existing controller after the previous step. */
    bool sat = false;
    for (unsigned w = 0; w < 2; ++w) {
        const ContinuousDrive *s = &continuous_wheel[w];
        if (!s->active) continue;
        float m = s->integral < 0.0f ? -s->integral : s->integral;
        uint8_t mi = (uint8_t)(m > 100.0f ? 100.0f : m);
        if (mi > ra_max_i[w]) ra_max_i[w] = mi;
        if (m >= (float)CONTINUOUS_I_MAX - 1.0f) sat = true;
    }
    if (sat && ra_sat_ticks < 255) ra_sat_ticks++;

    MotionStatus st = nav_get_status();
    b4v23_tilt_t tilt = {0};
    if (b4h_ready) b4v23_tilt(&tilt);
    RampAssistInput in = { .now_us = now, .dt = dt, .run_active = true,
        .auto_mode = st.session_active && st.mode == MOTION_AUTO, .following = following,
        .tilt_seq = tilt.seq, .tilt_us = tilt.t_us, .tilt_valid = tilt.valid, .tilt_mm = tilt.height_mm };
    bool was = ra_state.assisting;
    float alpha = ramp_assist_step(&ra_state, &in);
    if (!was && ra_state.assisting) ra_peak_pwm = 0;
    for (unsigned w = 0; w < 2; ++w) {
        ContinuousDrive *s = &continuous_wheel[w];
        s->boost_cap = (float)RA_PWM_CAP;
#if RAMP_ASSIST_ACTIVE
        s->boost_frac = s->active ? alpha / 1000.0f : 0.0f;   /* never seeds a fresh activation */
#else
        s->boost_frac = 0.0f;                                  /* shadow: motors unchanged */
#endif
        ra_pwm_now[w] = ra_added_pwm(w, alpha);
        if (ra_pwm_now[w] > ra_peak_pwm) ra_peak_pwm = ra_pwm_now[w];
    }
    if (ra_state.pending_event != 0xff) {
        uint8_t ev = ra_state.pending_event; ra_state.pending_event = 0xff;
        ra_report(now, ev);
    } else if (now - ra_last_report_us >= RA_REPORT_US) ra_report(now, RA_EVT_PERIODIC);
}

/* control_step, run NOT active (Stop, ESTOP, fault, run end): end any
 * assistance (reason STOPPED) and send one final report. */
static void ramp_assist_halt(uint64_t now) {
    if (!ra_run_seen) return;
    RampAssistInput in = { .now_us = now, .run_active = false };
    (void)ramp_assist_step(&ra_state, &in);
    for (unsigned w = 0; w < 2; ++w) { continuous_wheel[w].boost_frac = 0.0f; ra_pwm_now[w] = 0; }
    uint8_t ev = ra_state.pending_event != 0xff ? ra_state.pending_event : RA_EVT_PERIODIC;
    ra_state.pending_event = 0xff;
    ra_report(now, ev);
    ra_run_seen = false;
}

/* Output to seed the legacy controller when the continuous mode is left:
 * remove this step's added PWM so it is never carried into another
 * controller's integral. */
static int32_t ramp_assist_unboosted(unsigned w, int32_t output) {
    int32_t b = (int32_t)(continuous_wheel[w].boost_pwm + 0.5f);
    return output - b < 0 ? 0 : output - b;
}
