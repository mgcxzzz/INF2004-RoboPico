#ifdef APP_RTOS_IMU
#include "imu_service.h"
#include "ramp_model.h"
#include "../network_bridge/network_bridge.h"
#include <math.h>
static RampAssist state;
static bool seen;
static uint64_t last_report;
static uint8_t max_i[2], saturated;
static int16_t added[2], peak_pwm;
static T_SPLOCK report_lock;
static RampAssistTlm report;
static MotionRampAssist report_approved;

static void publish(uint64_t now, uint8_t event, const MotionRampAssist *approved,
                    const int16_t target[2]) {
    uint64_t active_ms = state.assisting ? (now - state.active_since_us) / 1000 : state.active_ms;
    RampAssistTlm t = {.uptime_ms = (uint32_t)(now / 1000),
                       .event = event,
                       .flags = (uint8_t)(1U | (state.assisting ? 2U : 0U) |
                                          (state.latched ? 4U : 0U) | (state.last_valid ? 8U : 0U) |
                                          ((RA_DETECTOR_VERSION & 7U) << 4) | 0x80U),
                       .last_end = state.last_end,
                       .activations = state.activations,
                       .alpha_milli = (uint16_t)(state.alpha_milli + .5f),
                       .pwm_l = added[0],
                       .pwm_r = added[1],
                       .height_mm = state.last_valid ? ra_i8(state.last_mm) : -128,
                       .peak_mm = ra_i8(state.peak_mm),
                       .int_max = state.int_valid ? ra_i8(state.int_max_mm) : -128,
                       .int_min = state.int_valid ? ra_i8(state.int_min_mm) : 127,
                       .valid = state.int_valid,
                       .invalid = state.int_invalid,
                       .longest = state.int_longest,
                       .max_i_l = max_i[0],
                       .max_i_r = max_i[1],
                       .sat_ticks = saturated,
                       .active_ms = (uint16_t)(active_ms > 65535 ? 65535 : active_ms),
                       .peak_pwm = peak_pwm};
    UINT key;
    (void)ISpinLock(&report_lock, &key);
    report = t;
    report_approved = *approved;
    (void)ISpinUnlock(&report_lock, key);
    if (rn_drive_telemetry_enabled() && rn_drive_online()) {
        uint8_t frame[RA_TLM_FRAME_BYTES];
        ra_tlm_pack(&t, rn_next_sequence(15), frame);
        (void)rn_tx_push(RN_TLM, frame, sizeof(frame), board_kernel_ms());
        uint8_t detail[24] = {0xF2, 21, rn_next_sequence(15)};
        uint8_t *p = detail + 3;
        uint32_t ms = (uint32_t)(now / 1000);
        for (unsigned k = 0; k < 4; ++k)
            p[k] = (uint8_t)(ms >> (8 * k));
        p[4] = 0xC1;
        p[5] = (uint8_t)(8U | (approved->hold_milli ? 1U : 0U) | (approved->boost_milli ? 2U : 0U) |
                         (approved->push_pwm_x100 ? 4U : 0U));
        p[6] = (uint8_t)approved->boost_milli;
        ra_put16(p + 7, approved->push_pwm_x100);
        ra_put16(p + 9, approved->hold_milli);
        for (unsigned w = 0; w < 2; ++w) {
            float effective =
                target[w] >= 0 ? fmaxf(target[w], RA_HOLD_MIN_MM_S * approved->hold_milli / 1000.f)
                               : target[w];
            ra_put16(p + 11 + 2 * w, (uint16_t)(int16_t)(target[w] * 10));
            ra_put16(p + 15 + 2 * w, (uint16_t)(int16_t)lroundf(effective * 10));
        }
        p[19] = (uint8_t)motion_get_status().owner;
        p[20] = approved->ttl_ms;
        (void)rn_tx_push(RN_TLM, detail, sizeof(detail), board_kernel_ms());
    }
    ra_interval_reset(&state);
    max_i[0] = max_i[1] = saturated = 0;
    last_report = now;
}
MotionRampAssist ri_ramp_step(uint64_t now, float dt, bool following, const float speed[2],
                              const int16_t target[2], const ContinuousDrive drive[2]) {
    if (!seen) {
        ra_reset(&state);
        seen = true;
        peak_pwm = 0;
        max_i[0] = max_i[1] = saturated = 0;
        last_report = now;
    }
    bool sat = false;
    for (unsigned w = 0; w < 2; ++w)
        if (drive[w].active) {
            float m = fabsf(drive[w].integral);
            unsigned value = m > 255 ? 255 : (unsigned)m;
            if (value > max_i[w])
                max_i[w] = (uint8_t)value;
            float limit = fmaxf(drive[w].hold_i_max, CONTINUOUS_I_MAX);
            sat |= m >= limit - 1;
        }
    if (sat && saturated < 255)
        ++saturated;
    b4v23_tilt_t tilt;
    ri_tilt_snapshot(&tilt);
    MotionStatus status = motion_get_status();
    bool was = state.assisting;
    MotionRampAssist requested =
        ri_ramp_request(&state, &tilt, now, dt, true,
                        status.session_active && status.mode == MOTION_AUTO, following, speed);
    if (!was && state.assisting)
        peak_pwm = 0;
    motion_assist_context(following);
    (void)nav_request_ramp_assist(MOTION_BUDDY4, &requested);
    MotionRampAssist approved = nav_get_ramp_assist();
    for (unsigned w = 0; w < 2; ++w) {
        float extra = fminf(RA_PWM_CAP, continuous_feedforward(w, drive[w].reference) *
                                            approved.boost_milli / 1000.f);
        extra += approved.push_pwm_x100 / 100.f;
        added[w] = drive[w].active ? (int16_t)lroundf(extra) : 0;
        if (added[w] > peak_pwm)
            peak_pwm = added[w];
    }
    if (state.pending_event != 0xff || now - last_report >= 500000) {
        uint8_t event = state.pending_event == 0xff ? RA_EVT_PERIODIC : state.pending_event;
        state.pending_event = 0xff;
        publish(now, event, &approved, target);
    }
    return approved;
}
void ri_ramp_halt(uint64_t now) {
    motion_assist_context(false);
    if (!seen)
        return;
    RampAssistInput in = {.now_us = now, .run_active = false};
    (void)ramp_assist_step(&state, &in);
    added[0] = added[1] = 0;
    MotionRampAssist empty = {0};
    const int16_t target[2] = {0};
    uint8_t event = state.pending_event == 0xff ? RA_EVT_PERIODIC : state.pending_event;
    state.pending_event = 0xff;
    publish(now, event, &empty, target);
    seen = false;
}
void ri_ramp_report(void) {
    UINT key;
    (void)ISpinLock(&report_lock, &key);
    RampAssistTlm t = report;
    MotionRampAssist a = report_approved;
    (void)ISpinUnlock(&report_lock, key);
    tm_printf(
        (UB *)"RAMP detector=6 enabled=1 assisting=%u activations=%u end=%u alpha_milli=%u push_x100=%u hold_milli=%u added_pwm=%d,%d max_i=%u,%u active_ms=%u\n",
        !!(t.flags & 2), t.activations, t.last_end, a.boost_milli, a.push_pwm_x100, a.hold_milli,
        t.pwm_l, t.pwm_r, t.max_i_l, t.max_i_r, t.active_ms);
}
#endif
