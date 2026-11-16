/**
 * @file ramp_bridge.c
 * @brief Ramp bridge.
 * @details The IMU task owns sensor acquisition and calibration; readers use snapshots.
 * Ramp requests expire through the coordinator; lift safety may remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifdef APP_RTOS_IMU
#include "imu_service.h"
#include "ramp_model.h"
#include "../network_bridge/network_bridge.h"
#include <math.h>
static ramp_assist_t g_state;
static bool g_seen;
static uint64_t g_last_report;
static uint8_t g_max_i[2];
static uint8_t g_saturated;
static int16_t g_added[2];
static int16_t g_peak_pwm;
static T_SPLOCK g_report_lock;
static ramp_assist_tlm_t g_report;
static motion_ramp_assist_t g_report_approved;

static void publish (uint64_t now, uint8_t event,
                     const motion_ramp_assist_t * p_approved,
                     const int16_t target[2])
{
    uint64_t active_ms = g_state.assisting
                             ? (now - g_state.active_since_us) / 1000
                             : g_state.active_ms;
    ramp_assist_tlm_t sample = {
        .uptime_ms = (uint32_t)(now / 1000),
        .event = event,
        .flags = (uint8_t)(1U | (g_state.assisting ? 2U : 0U) |
                           (g_state.latched ? 4U : 0U) |
                           (g_state.last_valid ? 8U : 0U) |
                           ((RA_DETECTOR_VERSION & 7U) << 4) | 0x80U),
        .last_end = g_state.last_end,
        .activations = g_state.activations,
        .alpha_milli = (uint16_t)(g_state.alpha_milli + .5f),
        .pwm_l = g_added[0],
        .pwm_r = g_added[1],
        .height_mm = g_state.last_valid ? ra_i8(g_state.last_mm) : -128,
        .peak_mm = ra_i8(g_state.peak_mm),
        .int_max = g_state.int_valid ? ra_i8(g_state.int_max_mm) : -128,
        .int_min = g_state.int_valid ? ra_i8(g_state.int_min_mm) : 127,
        .valid = g_state.int_valid,
        .invalid = g_state.int_invalid,
        .longest = g_state.int_longest,
        .max_i_l = g_max_i[0],
        .max_i_r = g_max_i[1],
        .sat_ticks = g_saturated,
        .active_ms = (uint16_t)(active_ms > 65535 ? 65535 : active_ms),
        .peak_pwm = g_peak_pwm};
    UINT key;
    (void)ISpinLock(&g_report_lock, &key);
    g_report = sample;
    g_report_approved = *p_approved;
    (void)ISpinUnlock(&g_report_lock, key);
    if (rn_drive_telemetry_enabled() && rn_drive_online())
    {
        uint8_t frame[RA_TLM_FRAME_BYTES];
        ra_tlm_pack(&sample, rn_next_sequence(15), frame);
        (void)rn_tx_push(RN_TLM, frame, sizeof(frame), board_kernel_ms());
        uint8_t detail[24] = {0xF2, 21, rn_next_sequence(15)};
        uint8_t * p_data = detail + 3;
        uint32_t milliseconds = (uint32_t)(now / 1000);
        for (unsigned item_index = 0; item_index < 4; ++item_index)
        {
            p_data[item_index] = (uint8_t)(milliseconds >> (8 * item_index));
        }
        p_data[4] = 0xC1;
        p_data[5] = (uint8_t)(8U | (p_approved->hold_milli ? 1U : 0U) |
                              (p_approved->boost_milli ? 2U : 0U) |
                              (p_approved->push_pwm_x100 ? 4U : 0U));
        p_data[6] = (uint8_t)p_approved->boost_milli;
        ra_put16(p_data + 7, p_approved->push_pwm_x100);
        ra_put16(p_data + 9, p_approved->hold_milli);
        for (unsigned wheel = 0; wheel < 2; ++wheel)
        {
            float effective =
                target[wheel] >= 0
                    ? fmaxf(target[wheel],
                            RA_HOLD_MIN_MM_S * p_approved->hold_milli / 1000.f)
                    : target[wheel];
            ra_put16(p_data + 11 + 2 * wheel,
                     (uint16_t)(int16_t)(target[wheel] * 10));
            ra_put16(p_data + 15 + 2 * wheel,
                     (uint16_t)(int16_t)lroundf(effective * 10));
        }
        p_data[19] = (uint8_t)motion_get_status().owner;
        p_data[20] = p_approved->ttl_ms;
        (void)rn_tx_push(RN_TLM, detail, sizeof(detail), board_kernel_ms());
    }
    ra_interval_reset(&g_state);
    g_max_i[0] = g_max_i[1] = g_saturated = 0;
    g_last_report = now;
}
motion_ramp_assist_t ri_ramp_step (float delta_time_value, bool following,
                                   const float speed[2],
                                   const int16_t target[2],
                                   const continuous_drive_t drive[2])
{
    b4v23_tilt_t tilt;
    ri_tilt_snapshot(&tilt);
    /* Core0 may publish during the control task's earlier ADC/planning work.
     * A loop-start timestamp can precede this copied sample and underflow the
     * unsigned age checks. Both values use the full64-bit hardware timer; read
     * current time only after the protected copy, before updating any timers. */
    const uint64_t now = board_time_us();
    if (!g_seen)
    {
        ra_reset(&g_state);
        g_seen = true;
        g_peak_pwm = 0;
        g_max_i[0] = g_max_i[1] = g_saturated = 0;
        g_last_report = now;
    }
    bool sat = false;
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        if (drive[wheel].active)
        {
            float m_value = fabsf(drive[wheel].integral);
            unsigned value = m_value > 255 ? 255 : (unsigned)m_value;
            if (value > g_max_i[wheel])
            {
                g_max_i[wheel] = (uint8_t)value;
            }
            float limit = fmaxf(drive[wheel].hold_i_max, CONTINUOUS_I_MAX);
            sat |= m_value >= limit - 1;
        }
    }
    if (sat && g_saturated < 255)
    {
        ++g_saturated;
    }
    motion_status_t status = motion_get_status();
    bool was = g_state.assisting;
    motion_ramp_assist_t requested = ri_ramp_request(
        &g_state, &tilt, now, delta_time_value, true,
        status.session_active && status.mode == MOTION_AUTO, following, speed);
    if (!was && g_state.assisting)
    {
        g_peak_pwm = 0;
    }
    motion_assist_context(following);
    (void)nav_request_ramp_assist(MOTION_BUDDY4, &requested);
    motion_ramp_assist_t approved = nav_get_ramp_assist();
    for (unsigned wheel_value = 0; wheel_value < 2; ++wheel_value)
    {
        float extra = fminf(
            RA_PWM_CAP,
            continuous_feedforward(wheel_value, drive[wheel_value].reference) *
                approved.boost_milli / 1000.f);
        extra += approved.push_pwm_x100 / 100.f;
        g_added[wheel_value] =
            drive[wheel_value].active ? (int16_t)lroundf(extra) : 0;
        if (g_added[wheel_value] > g_peak_pwm)
        {
            g_peak_pwm = g_added[wheel_value];
        }
    }
    if (g_state.pending_event != 0xff || now - g_last_report >= 500000)
    {
        uint8_t event = g_state.pending_event == 0xff ? RA_EVT_PERIODIC
                                                      : g_state.pending_event;
        g_state.pending_event = 0xff;
        publish(now, event, &approved, target);
    }
    return approved;
}
void ri_ramp_halt (uint64_t now)
{
    motion_assist_context(false);
    if (!g_seen)
    {
        return;
    }
    ramp_assist_input_t in_value = {.now_us = now, .run_active = false};
    (void)ramp_assist_step(&g_state, &in_value);
    g_added[0] = g_added[1] = 0;
    motion_ramp_assist_t empty = {0};
    const int16_t target[2] = {0};
    uint8_t event =
        g_state.pending_event == 0xff ? RA_EVT_PERIODIC : g_state.pending_event;
    g_state.pending_event = 0xff;
    publish(now, event, &empty, target);
    g_seen = false;
}
void ri_ramp_report (void)
{
    UINT key;
    (void)ISpinLock(&g_report_lock, &key);
    ramp_assist_tlm_t sample = g_report;
    motion_ramp_assist_t first = g_report_approved;
    (void)ISpinUnlock(&g_report_lock, key);
    tm_printf((UB *)"RAMP detector=6 enabled=1 assisting=%u activations=%u "
                    "end=%u alpha_milli=%u push_x100=%u hold_milli=%u "
                    "added_pwm=%d,%d max_i=%u,%u active_ms=%u\n",
              !!(sample.flags & 2), sample.activations, sample.last_end,
              first.boost_milli, first.push_pwm_x100, first.hold_milli,
              sample.pwm_l, sample.pwm_r, sample.max_i_l, sample.max_i_r,
              sample.active_ms);
}
#endif

/* End of file. */
