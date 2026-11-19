/**
 * @file line_service.c
 * @brief Line service.
 * @details ADC/control ownership covers sensor acquisition, planning and
 * calibration. Other tasks consume protected copies; flash and stored exports
 * require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "line_service.h"
#ifdef APP_RTOS_RELEASE
#include "wireless_calibration.h"
#endif
#include "include/interfaces/nav_api.h"
#ifdef APP_RTOS_BUDDY5
#include "../obstacle_navigation/obstacle_service.h"
#endif
#ifdef APP_RTOS_BARCODE
#include "../barcode_navigation/barcode_model.h"
static rb_decoder_t g_barcode;
static uint32_t g_barcode_peak_sent;
static uint32_t g_barcode_status_ms;
static bool g_barcode_dump;
static void barcode_publish (uint64_t now);
static void barcode_export (void);
#endif
#ifdef APP_RTOS_NETWORK
#include "../network_bridge/network_bridge.h"
#include "../network_bridge/drive_mode_telemetry.h"
#include "src/network/wifi_protocol.h"
static wifi_motion_packet_t g_telemetry;
static uint16_t g_run_id;
static uint32_t g_run_elapsed_ms;
static double g_run_path;
#endif

static ID g_commands;
static T_SPLOCK g_copy_lock;
static line_planner_t g_planner;
static line_power_t g_power;
static line_calibration_t g_calibration_value;
#ifdef APP_RTOS_RELEASE
static rl_cal_session_t g_wireless_cal;
#endif
static circuit_input_t g_input;
static float g_filtered[2];
static uint16_t g_adc[3];
static uint32_t g_cal_origin; /* 0 preset, 1 SDK record28, 2 own saved page, 3
                                 RAM capture. */
static bool g_adc_ok;
static bool g_filter_ready;
static bool g_was_active;
static bool g_start_offered;
static rl_adc_health_t g_adc_health;
static bool g_adc_recovering;
static uint32_t g_adc_retries;
static uint32_t g_adc_recoveries;
static uint32_t g_adc_frames;
static uint64_t g_side_due;
static uint64_t g_countdown;
static uint64_t g_run_start;
static line_calibration_t g_run_calibration;
static unsigned g_run_cal_origin;
static unsigned g_previous_phase;
static unsigned g_last_noted_phase;
static line_log_row_t g_rows[RL_LOG_ROWS];
static unsigned g_row_count;
static unsigned g_row_head;
static unsigned g_row_overwritten;
static unsigned g_row_sequence;
static bool g_dump_requested;
static bool g_dumping;
static bool g_recording;
#ifdef APP_RTOS_RELEASE
#include "release_export.h"
#include "drive_diagnostics.h"
static rl_export_lease_t g_wireless_export;
static float g_release_run_centre;
#endif
static volatile uint32_t g_cancel_serial;
static uint32_t g_last_cancel_us;
static uint32_t g_start_serial;
static struct
{
    line_calibration_t cal;
    uint16_t adc[3];
    uint16_t level[2];
    uint32_t cal_origin;
    uint32_t rows;
    uint32_t overwritten;
    uint32_t sequence;
    uint32_t cost;
    unsigned phase;
    unsigned decision;
    unsigned reason;
    unsigned pending;
    unsigned action;
    unsigned flags;
    uint32_t junction_aux;
    rl_adc_diagnostics_t adc_diag;
#ifdef APP_RTOS_RELEASE
    rl_drive_diagnostic_t drive_diagnostic;
#endif
    uint32_t adc_retries;
    uint32_t adc_recoveries;
    uint32_t adc_frames;
#ifdef APP_RTOS_IMU
    uint32_t side_polls;
    uint32_t side_adc_total_us;
    uint32_t side_adc_max_us;
    uint32_t side_work_total_us;
    uint32_t side_work_max_us;
    uint32_t side_poll_total_us;
    uint32_t side_poll_max_us;
#endif
    unsigned adc_state : 2;
    unsigned adc_good : 2;
    uint16_t load_pwm_packed;
    bool mode_start_offered;
#ifdef APP_RTOS_BARCODE
    rb_junction_sensor_t junction_sensor;
    bool junction_pending;
    uint32_t decoded;
    uint32_t peak_decodes;
    uint32_t peak_applied;
    uint32_t conflicts;
    uint32_t gaps;
    uint32_t max_gap;
    uint32_t samples;
    uint32_t max_sample;
    uint16_t barcode_min;
    uint16_t barcode_max;
    uint16_t barcode_adc;
    uint16_t threshold;
    unsigned cross_threshold : 12;
    unsigned cross_hysteresis : 12;
    unsigned cross_valid : 1;
    unsigned cross_armed : 1;
    unsigned cross_black : 1;
    uint8_t bank;
    uint8_t last_code;
    uint8_t pending_action;
    uint8_t pending_revision;
    uint8_t executing_action;
    uint8_t executing_revision;
    unsigned scanning : 1;
    unsigned raw_frozen : 1;
#ifdef APP_RTOS_RELEASE
    rb_attempt_info_t attempt;
#endif
#endif
} g_view;

static UINT enter (void)
{
    UINT key;
    (void)ISpinLock(&g_copy_lock, &key);
    return key;
}
static void leave (UINT key)
{
    (void)ISpinUnlock(&g_copy_lock, key);
}

#ifdef APP_RTOS_IMU
/* Count every due channel-1 read attempt, including failures. Lifetime totals
 * wrap. */
static void side_profile_record (uint32_t adc_us, uint32_t work_us,
                                 uint32_t poll_us)
{
    UINT key = enter();
    ++g_view.side_polls;
    g_view.side_adc_total_us += adc_us;
    if (adc_us > g_view.side_adc_max_us)
    {
        g_view.side_adc_max_us = adc_us;
    }
    g_view.side_work_total_us += work_us;
    if (work_us > g_view.side_work_max_us)
    {
        g_view.side_work_max_us = work_us;
    }
    g_view.side_poll_total_us += poll_us;
    if (poll_us > g_view.side_poll_max_us)
    {
        g_view.side_poll_max_us = poll_us;
    }
    leave(key);
}
#endif

static void acquisition_failed (uint64_t now)
{
    g_adc_ok = false;
    g_filter_ready = false;
    g_adc_recovering = true;
    g_countdown = 0;
    rl_adc_failed(&g_adc_health, now);
    /* Cancellation is out of queue and invalidates any armed Start. */
    bringup_status_t status;
    bringup_get_status(&status);
    if (status.flags & BR_ACTIVE)
    {
        bringup_stop(BR_REASON_ADC, BR_ESTOP);
    }
    else
    {
        rtos_line_cancel();
    }
}

/* A failed front pair is never half-published. During qualification, all
 * three channels must be freshly read in three consecutive control frames. */
static bool acquire_frame (bool include_side)
{
    uint16_t next[3] = {g_adc[0], g_adc[1], g_adc[2]};
    if (((!board_line_adc_read(0, 32, true, &next[0])) ||
         (!board_line_adc_read(2, 32, true, &next[2]))) ||
        (include_side && (!board_line_adc_read(1, 4, false, &next[1]))))
    {
        acquisition_failed(board_time_us());
        return false;
    }
    memcpy(g_adc, next, sizeof(g_adc));
    ++g_adc_frames;
    return true;
}

ER rtos_line_init (void)
{
    T_CMBF definition = {0};
    definition.mbfatr = TA_TFIFO;
    definition.bufsz = 256;
    definition.maxmsz = 5;
    g_commands = tk_cre_mbf(&definition);
    if (g_commands < E_OK)
    {
        return g_commands;
    }
    const barcode_saved_t * p_saved =
        (const barcode_saved_t *)(uintptr_t)(0x10000000U + RL_MAIN_OFFSET);
    g_calibration_value =
        rl_boot_cal((const uint8_t *)(uintptr_t)(0x10000000U + RL_CAL_OFFSET),
                    p_saved, &g_cal_origin);
    if (board_line_adc_init())
    {
        g_adc_health = (rl_adc_health_t){.state = RL_ADC_QUALIFYING};
    }
    else
    {
        acquisition_failed(board_time_us());
    }
    /* A peripheral fault gates line readiness, not motor/encoder init. It
     * can recover while stopped; object-creation failure above is still fatal.
     */
    return E_OK;
}

bool rtos_line_command (unsigned command, uint32_t received_us)
{
    uint8_t msg[5] = {(uint8_t)command, (uint8_t)received_us,
                      (uint8_t)(received_us >> 8), (uint8_t)(received_us >> 16),
                      (uint8_t)(received_us >> 24)};
    return E_OK == tk_snd_mbf(g_commands, msg, 5, TMO_POL);
}

void rtos_line_cancel (void)
{
    UINT key = enter();
    g_last_cancel_us = (uint32_t)board_time_us();
    ++g_cancel_serial;
    leave(key);
}

bool rtos_line_start_valid (void)
{
    UINT key = enter();
    bool valid = g_start_serial == g_cancel_serial;
#ifdef APP_RTOS_RELEASE
    valid = valid &&
            (!rl_export_busy(&g_wireless_export, (uint32_t)board_time_us()));
#endif
    leave(key);
    return valid;
}

void rtos_line_side_poll (uint64_t now)
{
    if ((!g_adc_ok) || (now < g_side_due))
    {
        return;
    }
#ifdef APP_RTOS_IMU
    uint64_t poll_started = board_time_us();
#endif
    g_side_due = now + 2000;
#ifdef APP_RTOS_IMU
    uint64_t adc_started = board_time_us();
#endif
    if (!board_line_adc_read(1, 4, false, &g_adc[1]))
    {
#ifdef APP_RTOS_IMU
        uint32_t adc_us = (uint32_t)(board_time_us() - adc_started);
#endif
        acquisition_failed(board_time_us());
#ifdef APP_RTOS_IMU
        side_profile_record(adc_us, 0,
                            (uint32_t)(board_time_us() - poll_started));
#endif
        return;
    }
#ifdef APP_RTOS_IMU
    uint32_t adc_us = (uint32_t)(board_time_us() - adc_started);
    uint32_t work_us = 0;
#endif
    if (g_was_active
#ifdef APP_RTOS_NETWORK
        && !rn_drive_paused()
#endif
    )
    {
        bringup_status_t safety;
        bringup_get_status(&safety);
        if (!(safety.flags & BR_ACTIVE))
        {
#ifdef APP_RTOS_IMU
            side_profile_record(adc_us, 0,
                                (uint32_t)(board_time_us() - poll_started));
#endif
            return;
        }
        /* Between 10 ms encoder drains, interpolate at the last measured
         * velocity, as a bounded-distance edge estimate, never a PWM change. */
        double wheel[2];
        double elapsed =
            now >= g_input.now ? (double)(now - g_input.now) / 1000000 : 0;
        for (unsigned wheel_value = 0; wheel_value < 2; ++wheel_value)
        {
            wheel[wheel_value] = g_input.wheel[wheel_value] +
                                 g_input.speed[wheel_value] * elapsed;
        }
#ifdef APP_RTOS_IMU
        uint64_t work_started = board_time_us();
#endif
#ifdef APP_RTOS_BARCODE
        uint64_t started = board_time_us();
        g_planner.nav.mapping = rn_mapping_snapshot();
        rb_sample(&g_barcode, &g_planner.nav, g_adc[1], now, wheel);
        g_planner.black = g_barcode.analog.black;
        g_planner.side_adc = g_adc[1];
        uint32_t cost = (uint32_t)(board_time_us() - started);
        if (cost > g_barcode.max_sample_us)
        {
            g_barcode.max_sample_us = cost;
        }
#else
        rl_side(&planner, adc[1], now, wheel);
#endif
#ifdef APP_RTOS_IMU
        work_us = (uint32_t)(board_time_us() - work_started);
#endif
    }
#ifdef APP_RTOS_IMU
    side_profile_record(adc_us, work_us,
                        (uint32_t)(board_time_us() - poll_started));
#endif
}

bool rtos_line_ready (void)
{
    return g_adc_ok && (calibration_valid(&g_calibration_value))
#ifdef APP_RTOS_RELEASE
           && !g_wireless_cal.session
#endif
        ;
}

bool rtos_line_pending (void)
{
    UINT key = enter();
    bool pending = (g_view.flags & 4U) && (g_start_serial == g_cancel_serial);
    leave(key);
    return pending;
}
bool rtos_line_mode_change_allowed (void)
{
    UINT key = enter();
    /* These dump/export flags and the countdown/start-offer copies are all
     * changed under this lock; the network core does not inspect line-owner
     * state directly. */
    bool line_idle =
        (((!(g_view.flags & 4U)) && (!g_view.mode_start_offered)) &&
         (!g_dumping)) &&
        (!g_dump_requested);
#ifdef APP_RTOS_RELEASE
    line_idle = line_idle && (!rl_export_busy(&g_wireless_export,
                                              (uint32_t)board_time_us()));
#endif
    leave(key);
    bringup_status_t supervisor;
    bringup_get_status(&supervisor);
    motion_speed_mode_status_t mode = motion_get_speed_mode_status();
    bool paused = false;
#ifdef APP_RTOS_NETWORK
    paused = rn_drive_paused();
#endif
    return ((((line_idle && (!(supervisor.flags & BR_ACTIVE))) && (!paused)) &&
             (!mode.session_active)) &&
            (mode.stopped)) &&
           MOTION_OFF == mode.motion_mode;
}
bool rtos_line_request_fresh (uint32_t received_us)
{
    UINT key = enter();
    bool valid = br_command_after_stop((uint32_t)board_time_us(), received_us,
                                       g_last_cancel_us, g_cancel_serial != 0);
    leave(key);
    return valid;
}
bool rtos_line_request_start (uint32_t received_us)
{
    UINT key = enter();
    bool allowed =
        (((!g_dumping) && (!g_dump_requested)) && (!g_countdown)) &&
        (!g_start_offered) &&
#ifdef APP_RTOS_RELEASE
        !rl_export_busy(&g_wireless_export, (uint32_t)board_time_us()) &&
#endif
        br_command_after_stop((uint32_t)board_time_us(), received_us,
                              g_last_cancel_us, g_cancel_serial != 0);
    uint32_t serial = g_cancel_serial;
    leave(key);
    if ((!allowed) || (!rtos_line_ready()))
    {
        return false;
    }
#ifdef APP_RTOS_BUDDY5
    if (!rb5_start_allowed())
    {
        return false; /* Start needs a fresh healthy centred sonar result. */
    }
#endif
#ifdef APP_RTOS_NETWORK
    if ((!rn_drive_online()) || (!bringup_can_start()))
    {
        return false;
    }
#endif
    key = enter();
    if (serial != g_cancel_serial)
    {
        leave(key);
        return false;
    }
    g_start_serial = serial;
    g_countdown = g_input.now + 2000000;
    g_view.flags |= 4U;
    leave(key);
    return true;
}
#ifdef APP_RTOS_NETWORK
void rtos_line_resume (void)
{
#ifdef APP_RTOS_BARCODE
    rb_resume(&g_planner, &g_barcode, &g_input, g_adc[1]);
#else
    char pending = planner.nav.pending;
    rl_begin(&planner, &input, adc[1]);
    if (pending)
    {
        (void)barcode_set_pending(&planner.nav, pending);
    }
#endif
    g_power = (line_power_t){0};
    g_last_noted_phase = UINT32_MAX;
}
void rtos_line_telemetry (wifi_motion_packet_t * p_out_value)
{
    UINT key = enter();
    *p_out_value = g_telemetry;
    leave(key);
}
#endif
#ifdef APP_RTOS_RELEASE
#include "wireless_calibration.inc"
#endif

static void calibration_command (unsigned command)
{
#ifdef APP_RTOS_RELEASE
    if ('w' == command || 'l' == command || 'r' == command || 'k' == command ||
        'c' == command)
    {
        rl_cal_abort(&g_wireless_cal, RL_CAL_CANCELLED);
    }
#endif
    if ((command != 'c') && (!g_adc_ok))
    {
        tm_printf((UB *)"CAL rejected: ADC not ready; existing calibration "
                        "retained.\n");
        return;
    }
    if ('c' == command)
    {
        g_calibration_value = successful_calibration();
        g_cal_origin = 0;
    }
    else if ('w' == command)
    {
        g_calibration_value.white[0] = g_adc[0];
        g_calibration_value.white[1] = g_adc[2];
        g_calibration_value.mask |= 1;
        g_cal_origin = 3;
    }
    else if ('l' == command || 'r' == command)
    {
        unsigned wheel = 'l' == command ? 0 : 1;
        g_calibration_value.black[wheel] = g_adc[0 == wheel ? 0 : 2];
        g_calibration_value.mask |= 2U << wheel;
        g_cal_origin = 3;
    }
    else if ('k' == command)
    {
        if (!rtos_line_ready())
        {
            tm_printf((UB *)"CAL_SAVE rejected: calibration or ADC invalid.\n");
            return;
        }
        uint8_t page[256];
        rl_cal_encode(page, &g_calibration_value);
        bool ok_value = board_line_save(page);
        if (ok_value)
        {
            g_cal_origin = 2;
        }
        tm_printf((UB *)"CAL_SAVE result=%s own_sector=0x%x; SDK/Wi-Fi records "
                        "untouched.\n",
                  ok_value ? "VERIFIED" : "FAILED", RL_CAL_OFFSET);
    }
    tm_printf(
        (UB *)"CAL_FRONT white=%u,%u black=%u,%u mask=%u valid=%u origin=%u\n",
        g_calibration_value.white[0], g_calibration_value.white[1],
        g_calibration_value.black[0], g_calibration_value.black[1],
        g_calibration_value.mask, calibration_valid(&g_calibration_value),
        g_cal_origin);
}

bool rtos_line_prepare (const motion_feedback_t * p_flags_value, double path,
                        float delta_time_value, bool active)
{
    (void)delta_time_value;
    g_start_offered = false;
    if (g_adc_ok)
    {
        (void)acquire_frame(false);
    }
    else
    {
        g_countdown = 0;
        if (rl_adc_can_retry(&g_adc_health, p_flags_value->now_us, active))
        {
            ++g_adc_retries;
            if (board_line_adc_init())
            {
                g_adc_health = (rl_adc_health_t){.state = RL_ADC_QUALIFYING};
            }
            else
            {
                acquisition_failed(board_time_us());
            }
        }
        if ((!active) && RL_ADC_QUALIFYING == g_adc_health.state &&
            (acquire_frame(true)))
        {
            g_adc_ok = rl_adc_good_frame(&g_adc_health);
            if (g_adc_ok)
            {
                if (g_adc_recovering)
                {
                    ++g_adc_recoveries;
                }
                g_adc_recovering = false;
                g_filter_ready = false;
                g_side_due = board_time_us() + 2000;
            }
        }
    }
#ifdef APP_RTOS_RELEASE
    wireless_cal_tick();
#endif
    g_input = (circuit_input_t){
        .now = p_flags_value->now_us,
        .wheel = {p_flags_value->wheel_mm[0], p_flags_value->wheel_mm[1]},
        .speed = {p_flags_value->speed_mm_s[0], p_flags_value->speed_mm_s[1]},
        .side_black = g_planner.black};
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        uint16_t raw = g_adc[0 == wheel ? 0 : 2];
        if (!g_filter_ready)
        {
            g_filtered[wheel] = raw;
        }
        else
        {
            g_filtered[wheel] += .35f * ((float)raw - g_filtered[wheel]);
        }
        g_input.adc[wheel] =
            (uint16_t)g_filtered[wheel]; /* Same truncation as SDK main. */
    }
    g_filter_ready = true;
    g_input.path = path; /* Average absolute accumulated wheel travel, including
                            U-turns. */
    if ((!active) && g_was_active)
    {
        g_countdown = 0;
    }
    g_was_active = active;
    if (g_countdown && (!rtos_line_start_valid()))
    {
        g_countdown = 0;
    }
#ifdef APP_RTOS_NETWORK
    if (!rn_drive_online())
    {
        g_countdown = 0;
    }
#endif
    uint8_t msg[5];
    for (unsigned count = 0;
         (count < 8) && 5 == tk_rcv_mbf(g_commands, msg, TMO_POL); ++count)
    {
        uint32_t received = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) |
                            ((uint32_t)msg[3] << 16) | ((uint32_t)msg[4] << 24);
        UINT key = enter();
        uint32_t serial = g_cancel_serial, stopped = g_last_cancel_us;
        leave(key);
        if (!br_command_after_stop((uint32_t)board_time_us(), received, stopped,
                                   serial != 0))
        {
            continue;
        }
        unsigned command = msg[0];
        if ('d' == command
#ifdef APP_RTOS_BARCODE
            || 'b' == command
#endif
        )
        {
            g_countdown = 0;
            UINT key = enter();
            if (!active)
            {
                g_dump_requested = true;
#ifdef APP_RTOS_BARCODE
                g_barcode_dump = 'b' == command;
#endif
            }
            leave(key);
            if (active)
            {
                tm_printf((UB *)"LOG rejected: stop before d.\n");
            }
        }
        else if (active)
        {
            tm_printf((UB *)"LINE command '%c' rejected while moving.\n",
                      command);
        }
        else if ('g' == command)
        {
            UINT key = enter();
            bool exporting = g_dumping || g_dump_requested;
            leave(key);
            if ((!exporting) && (rtos_line_request_start(received)))
            {
                tm_printf((UB *)"LINE start requested; 2s countdown; buttons/s "
                                "cancel.\n");
            }
            else if (exporting)
            {
                tm_printf((UB *)"LINE start rejected: log export still "
                                "pending/active.\n");
            }
            else if (!g_adc_ok)
            {
                tm_printf((UB *)"LINE start rejected: ADC not ready; see "
                                "ADC/ADC_FIRST/ADC_LAST.\n");
            }
#ifdef APP_RTOS_BUDDY5
            else if (!rb5_start_allowed())
            {
                tm_printf((UB *)"B5 START rejected: need a fresh healthy "
                                "centred sonar result; o=report.\n");
            }
#endif
            else
            {
                tm_printf((UB *)"LINE start rejected: calibration invalid.\n");
            }
        }
        else
        {
            g_countdown = 0;
            calibration_command(command);
        }
    }
    bringup_status_t state;
    bringup_get_status(&state);
    if ((((((board_button_pressed(20)) || (board_button_pressed(21))) ||
           g_bringup_irq_stop) ||
          (state.flags & BR_LATCHED)) ||
         (!br_usb_supervised(tm_usb_state()))) ||
        (!rtos_line_start_valid()))
    {
        g_countdown = 0;
    }
    bool start =
        (g_countdown && (p_flags_value->now_us >= g_countdown)) && (!active);
    if (start)
    {
        g_countdown = 0;
    }
    g_start_offered = start;
    UINT mode_key = enter();
    g_view.mode_start_offered = start;
    leave(mode_key);
    return start;
}

void rtos_line_begin (void)
{
#ifdef APP_RTOS_IMU
    ri_new_run();
#endif
    g_start_offered = false;
    rl_begin(&g_planner, &g_input, g_adc[1]);
#ifdef APP_RTOS_BARCODE
    g_planner.nav.mapping = rn_mapping_snapshot();
    rb_begin(&g_barcode, &g_planner.nav, g_adc[1], g_input.now, g_input.wheel);
    g_barcode_peak_sent = 0;
    g_barcode_status_ms = 0;
#endif
    g_power = (line_power_t){0};
    g_run_start = g_input.now;
#ifdef APP_RTOS_NETWORK
    ++g_run_id;
    g_run_elapsed_ms = 0;
    g_run_path = g_input.path;
#endif
    g_last_noted_phase = UINT32_MAX;
    UINT key = enter();
    g_run_calibration = g_calibration_value;
    g_run_cal_origin = g_cal_origin;
#ifdef APP_RTOS_RELEASE
    g_release_run_centre = (g_input.wheel[0] + g_input.wheel[1]) * .5f;
    g_wireless_export.active = 0;
#endif
    g_view.mode_start_offered = false;
    g_row_count = g_row_head = g_row_overwritten = g_row_sequence = 0;
    g_recording = true;
    leave(key);
    g_was_active = true;
}

analog_demand_t rtos_line_follow (float delta_time_value)
{
    g_previous_phase = g_planner.circuit.phase;
    g_input.side_black = g_planner.black;
    unsigned cruise_speed =
        drive_mode_speed(motion_get_speed_mode_status().mode);
#ifdef APP_RTOS_BARCODE
    g_planner.nav.mapping = rn_mapping_snapshot();
    return rb_follow_with_cruise(&g_planner, &g_barcode, &g_calibration_value,
                                 &g_input, delta_time_value, cruise_speed);
#else
    return rl_follow_with_cruise(&planner, &calibration, &input,
                                 delta_time_value, cruise_speed);
#endif
}

void rtos_line_feedback (motion_feedback_t * p_flags_value,
                         const analog_demand_t * p_demand_value)
{
    p_flags_value->line_level[0] =
        p_demand_value
            ? p_demand_value->level[0]
            : line_level(g_input.adc[0], g_calibration_value.white[0],
                         g_calibration_value.black[0]);
    p_flags_value->line_level[1] =
        p_demand_value
            ? p_demand_value->level[1]
            : line_level(g_input.adc[1], g_calibration_value.white[1],
                         g_calibration_value.black[1]);
    p_flags_value->navigation_busy = g_planner.circuit.busy;
    p_flags_value->pending_route = g_planner.nav.pending;
    if (g_planner.circuit.phase != g_last_noted_phase)
    {
        motion_note_navigation(g_planner.circuit.phase, 0);
        g_last_noted_phase = g_planner.circuit.phase;
    }
}

line_envelope_t rtos_line_power (const motion_output_t * p_output_value,
                                 analog_demand_t * p_demand_value,
                                 float delta_time_value)
{
    if (p_output_value->reset_controllers)
    {
        g_power = (line_power_t){0};
    }
    if (p_output_value->route)
    {
        (void)barcode_set_pending(&g_planner.nav, p_output_value->route);
    }
    p_demand_value->target[0] = p_output_value->target_mm_s[0];
    p_demand_value->target[1] = p_output_value->target_mm_s[1];
    return rl_power(&g_power, &g_planner, &g_input, p_demand_value,
                    g_previous_phase, delta_time_value);
}

void rtos_line_actual_output (unsigned wheel_value, int32_t output)
{
    g_power.output[wheel_value] = output;
}

void rtos_line_record (const bringup_status_t * p_state_value,
                       const analog_demand_t * p_demand_value, uint32_t cost)
{
    line_log_row_t row = {
        .ms = (uint32_t)((g_input.now - g_run_start) / 1000),
        .ticks = {p_state_value->ticks[0], p_state_value->ticks[1]},
        .speed = {p_state_value->speed_x10[0], p_state_value->speed_x10[1]},
        .target = {p_state_value->target[0], p_state_value->target[1]},
        .pwm = {p_state_value->pwm[0], p_state_value->pwm[1]},
        .dt_us = p_state_value->dt_us > 65535 ? 65535
                                              : (uint16_t)p_state_value->dt_us,
        .cost_us = cost > 65535 ? 65535 : (uint16_t)cost};
    uint16_t raw[3] = {g_adc[0], g_adc[2], g_adc[1]};
    rl_pack_optics(&row, raw, p_demand_value, g_planner.circuit.phase);
    uint8_t load_pwm[2] = {0};
    bool load_visible = (p_state_value->flags & BR_ACTIVE) != 0U;
#ifdef APP_RTOS_NETWORK
    load_visible = load_visible && (!rn_drive_paused());
#endif
    if (load_visible)
    {
        for (unsigned wheel = 0; wheel < 2; ++wheel)
        {
            float load = g_power.drive[wheel].load_pwm;
            if ((isfinite(load)) && (load > 0.0f))
            {
                load_pwm[wheel] = (uint8_t)lroundf(
                    clampf(load, 0.0f, DRIVE_MODE_LOAD_MAX_PWM));
            }
        }
    }
    rl_pack_steering(&row, g_input.adc,
                     (unsigned)(g_planner.steering.gain * 1000 + .5f),
                     (unsigned)(g_planner.steering.damp_weight * 1000 + .5f));
    rl_pack_junction(&row, &g_planner);
    row.optical_target[0] = g_planner.optical.target[0];
    row.optical_target[1] = g_planner.optical.target[1];
    const circuit_tracker_t * p_tracker = J_ACQUIRE == g_planner.circuit.phase
                                              ? &g_planner.circuit.joining
                                              : &g_planner.circuit.follower;
    row.meta =
        (p_demand_value->decision & 63U) |
        ((g_planner.circuit.reason & 63U) << 6) |
        ((uint32_t)g_planner.black << 12) |
        ((uint32_t)g_planner.circuit.busy << 13) |
        ((uint32_t)p_tracker->core.white << 14) |
        ((uint32_t)p_tracker->search.active << 15) |
#ifdef APP_RTOS_BARCODE
        (rl_route_bits(rb_log_route(&g_planner.nav, g_planner.nav.pending))
         << 16) |
        (rl_route_bits(rb_log_route(&g_planner.nav, g_planner.nav.action))
         << 18) |
#else
        (rl_route_bits(planner.nav.pending) << 16) |
        (rl_route_bits(planner.nav.action) << 18) |
#endif
        ((p_state_value->reason & 15U) << 20) |
        ((p_state_value->flags & 127U) << 24);
#ifdef APP_RTOS_RELEASE
    const continuous_steering_t * p_steering = &g_planner.steering;
    unsigned bend_phase = p_steering->bend_exiting     ? 3U
                          : p_steering->bend_confirmed ? 2U
                          : p_steering->bend_candidate ? 1U
                                                       : 0U;
    rl_row_bend_phase(bend_phase, &row.meta, &row.aux[1]);
    unsigned bend_side = continuous_bend_direction(p_steering) > 0   ? 1U
                         : continuous_bend_direction(p_steering) < 0 ? 2U
                                                                     : 0U;
    float reduction = (1.0f - p_steering->bend_speed_scale) * 500.0f;
    unsigned bend_percent = (p_steering->active) && (isfinite(reduction))
                                ? (unsigned)lroundf(clampf(reduction, 0, 100))
                                : 0U;
    unsigned recovery_phase = 0, stage = 0;
    float yaw = 0;
#if CIRCUIT_STATIONARY_SEARCH
    recovery_phase =
        (p_state_value->flags & BR_ACTIVE) && (p_tracker->core.white)
            ? (p_tracker->search.phase ? p_tracker->search.phase : 7U)
            : 0U;
    stage = p_tracker->search.stage;
    yaw = p_tracker->search.relative_yaw_deg;
#endif
    int yaw_x10 =
        isfinite(yaw) ? (int)lroundf(clampf(yaw * 10, -32767, 32767)) : 0;
    rl_drive_diagnostic_t drive_diagnostic = rl_drive_diag_make(
        bend_phase, bend_side, bend_percent, recovery_phase, stage, yaw_x10,
        p_state_value->target[0], p_state_value->target[1]);
    if (!isfinite(yaw))
    {
        drive_diagnostic.saturated = true;
    }
#endif
    UINT key = enter();
    g_view.cal = g_calibration_value;
    g_view.load_pwm_packed = (uint16_t)(load_pwm[0] & 0x7fU) |
                             (uint16_t)((uint16_t)(load_pwm[1] & 0x7fU) << 7U);
#ifdef APP_RTOS_RELEASE
    g_view.drive_diagnostic = drive_diagnostic;
#endif
    g_view.cal_origin = g_cal_origin;
    memcpy(g_view.adc, raw, sizeof(raw));
    for (unsigned wheel_value = 0; wheel_value < 2; ++wheel_value)
    {
        g_view.level[wheel_value] = line_level(
            g_input.adc[wheel_value], g_calibration_value.white[wheel_value],
            g_calibration_value.black[wheel_value]);
    }
    g_view.phase = g_planner.circuit.phase;
    g_view.decision = p_demand_value->decision;
    g_view.reason = g_planner.circuit.reason;
    g_view.pending = (uint8_t)g_planner.nav.pending;
    g_view.action = (uint8_t)g_planner.nav.action;
    g_view.cost = cost;
    g_view.flags = (g_adc_ok ? 1U : 0U) |
                   (calibration_valid(&g_calibration_value) ? 2U : 0U) |
                   (g_countdown ? 4U : 0U);
    g_view.junction_aux = row.aux[1];
    board_line_adc_diagnostics(&g_view.adc_diag);
    g_view.adc_state = g_adc_health.state;
    g_view.adc_good = g_adc_health.good;
    g_view.adc_retries = g_adc_retries;
    g_view.adc_recoveries = g_adc_recoveries;
    g_view.adc_frames = g_adc_frames;
#ifdef APP_RTOS_BARCODE
    g_view.decoded = g_planner.nav.decoded_total;
    g_view.peak_decodes = g_barcode.peak_decodes;
    g_view.peak_applied = g_barcode.peak_applied;
    g_view.conflicts = g_barcode.peak_conflicts;
    g_view.gaps = g_barcode.analog.gap_count;
    g_view.max_gap = g_barcode.analog.max_gap_us;
    g_view.samples = g_barcode.samples;
    g_view.max_sample = g_barcode.max_sample_us;
    g_view.barcode_min = g_barcode.analog.minimum;
    g_view.barcode_max = g_barcode.analog.maximum;
    g_view.barcode_adc = g_adc[1];
    g_view.threshold = barcode_analog_threshold(g_barcode.analog.selected);
    g_view.cross_threshold = g_barcode.junction.threshold;
    g_view.cross_hysteresis = g_barcode.junction.hysteresis;
    g_view.cross_valid = g_barcode.junction.valid;
    g_view.cross_armed = g_barcode.junction.armed;
    g_view.cross_black = g_barcode.junction.black;
    g_view.junction_sensor = g_barcode.junction;
    g_view.junction_pending = g_planner.nav.pending != 0;
    g_view.bank = g_barcode.analog.selected;
    g_view.last_code = g_planner.nav.last_code;
    g_view.pending_action = g_planner.nav.pending_action;
    g_view.pending_revision = g_planner.nav.pending_revision;
    g_view.executing_action = g_planner.nav.executing_action;
    g_view.executing_revision = g_planner.nav.executing_revision;
    g_view.scanning = g_barcode.analog.scanning;
    g_view.raw_frozen = g_barcode.raw_frozen;
#ifdef APP_RTOS_RELEASE
    g_view.attempt = g_barcode.attempt;
#endif
#endif
    if (g_recording)
    {
        row.sequence = ++g_row_sequence;
        g_rows[g_row_head] = row;
        g_row_head = (g_row_head + 1) % RL_LOG_ROWS;
        if (g_row_count < RL_LOG_ROWS)
        {
            ++g_row_count;
        }
        else
        {
            ++g_row_overwritten;
        }
        if (!(p_state_value->flags & BR_ACTIVE))
        {
            g_recording = false; /* Preserve one terminal stopped/fault row. */
        }
    }
#ifdef APP_RTOS_NETWORK
    if (g_recording)
    {
        g_run_elapsed_ms = (uint32_t)((g_input.now - g_run_start) / 1000);
    }
    g_telemetry = (wifi_motion_packet_t){
        .uptime_ms = p_state_value->kernel_ms,
        .elapsed_ms = g_run_elapsed_ms,
        .run_id = g_run_id,
        .phase = (uint8_t)g_planner.circuit.phase,
        .profile = RL_PROFILE,
        .flags =
            (p_state_value->flags & BR_ACTIVE ? 1U : 0U) |
            (calibration_valid(&g_calibration_value) ? 2U : 0U) |
            ((p_state_value->flags & BR_ACTIVE) && (!rn_drive_paused()) ? 4U
                                                                        : 0U) |
            (p_state_value->flags & BR_ESTOP ? 8U : 0U) | 0x40U
#ifdef APP_RTOS_BARCODE
            | 0x20U /* Step4 identification within the existing RTOS marker. */
#endif
#ifdef APP_RTOS_IMU
            | 0x10U /* Step5 plus Step4 features; no USB policy change. */
#endif
        ,
        .recovery_state = (uint8_t)(p_tracker->search.active ? 2
                                    : p_tracker->core.white  ? 1
                                                             : 0),
        .distance_mm = g_input.path - g_run_path < UINT32_MAX
                           ? (uint32_t)(g_input.path - g_run_path)
                           : UINT32_MAX,
        .adc_left = raw[0],
        .adc_right = raw[1],
        .adc_barcode = raw[2],
        .calibrated_level_left =
            calibration_valid(&g_calibration_value) ? g_view.level[0] : 0xffff,
        .calibrated_level_right =
            calibration_valid(&g_calibration_value) ? g_view.level[1] : 0xffff,
        .speed_left_0p1mm_s = p_state_value->speed_x10[0],
        .speed_right_0p1mm_s = p_state_value->speed_x10[1],
        .target_left_mm_s = p_state_value->target[0],
        .target_right_mm_s = p_state_value->target[1],
        .pwm_left = p_state_value->pwm[0],
        .pwm_right = p_state_value->pwm[1],
        .heading_cdeg = (int16_t)lroundf(
            remainderf(heading_deg(g_input.wheel[0], g_input.wheel[1]), 360) *
            100)};
    g_telemetry.line_levels = calibration_valid(&g_calibration_value)
                                  ? (g_view.level[0] >= 400 ? 1U : 0U) |
                                        (g_view.level[1] >= 400 ? 2U : 0U) |
                                        (g_planner.black ? 4U : 0U)
                                  : 0;
#endif
    g_view.rows = g_row_count;
    g_view.overwritten = g_row_overwritten;
    g_view.sequence = g_row_sequence;
    leave(key);
#ifdef APP_RTOS_BARCODE
    barcode_publish(g_input.now);
#endif
}

void rtos_line_report (void)
{
#ifdef APP_RTOS_BARCODE
    UINT bk_value = enter();
    uint32_t decoded = g_view.decoded, peak = g_view.peak_decodes,
             applied = g_view.peak_applied, conflicts = g_view.conflicts;
    uint32_t gaps = g_view.gaps, maxgap = g_view.max_gap,
             samples = g_view.samples, maxsample = g_view.max_sample;
    uint16_t minimum = g_view.barcode_min, maximum = g_view.barcode_max,
             threshold = g_view.threshold;
    unsigned bank = g_view.bank, last = g_view.last_code,
             pa_value = g_view.pending_action, rev = g_view.pending_revision;
    bool frozen = g_view.raw_frozen;
    unsigned cross_threshold = g_view.cross_threshold,
             cross_hysteresis = g_view.cross_hysteresis;
    unsigned cross_valid = g_view.cross_valid, cross_armed = g_view.cross_armed,
             cross_black = g_view.cross_black;
    leave(bk_value);
    tm_printf((UB *)"BARCODE decoded=%u peak=%u applied=%u conflicts=%u "
                    "last_ascii=%u pending_action=%u revision=%u min=%u max=%u "
                    "threshold=%u bank=%u samples=%u max_gap_us=%u "
                    "gaps_over10ms=%u max_sample_us=%u raw_frozen=%u\n",
              decoded, peak, applied, conflicts, last, pa_value, rev, minimum,
              maximum, threshold, bank, samples, maxgap, gaps, maxsample,
              frozen);
    tm_printf((UB *)"JUNCTION_SENSOR offset_mm=%u threshold=%u hysteresis=%u "
                    "valid=%u armed=%u black=%u\n",
              B_SIDE_AHEAD_MM, cross_threshold, cross_hysteresis, cross_valid,
              cross_armed, cross_black);
    tm_printf((UB *)"JUNCTION_GATE policy=confirmation clear_mm=150 "
                    "reject_bits=1:legacy,2:clearance,4:line_lost,8:not_"
                    "visible,16:search\n");
#endif
    UINT key = enter();
    /* No formatting while holding the shared packed-data lock. */
    line_calibration_t cal = g_view.cal;
    unsigned origin = g_view.cal_origin, flags = g_view.flags;
    uint16_t raw[3], levels[2];
    memcpy(raw, g_view.adc, sizeof(raw));
    memcpy(levels, g_view.level, sizeof(levels));
    unsigned phase = g_view.phase, decision = g_view.decision,
             reason = g_view.reason;
    unsigned pending = g_view.pending, action = g_view.action;
    unsigned count = g_view.rows, overwritten = g_view.overwritten;
    rl_adc_diagnostics_t diag = g_view.adc_diag;
    unsigned health = g_view.adc_state, good = g_view.adc_good;
    uint32_t retries = g_view.adc_retries, recoveries = g_view.adc_recoveries,
             frames = g_view.adc_frames;
#ifdef APP_RTOS_IMU
    uint32_t side_polls = g_view.side_polls,
             side_adc_total = g_view.side_adc_total_us,
             side_adc_max = g_view.side_adc_max_us,
             side_work_total = g_view.side_work_total_us,
             side_work_max = g_view.side_work_max_us,
             side_poll_total = g_view.side_poll_total_us,
             side_poll_max = g_view.side_poll_max_us;
#endif
    line_log_row_t junction = {.aux = {0, g_view.junction_aux}};
    leave(key);
#ifdef APP_RTOS_IMU
    tm_printf(
        (UB *)"SIDE_SERVICE timing=wall polls=%u adc_total_us=%u adc_max_us=%u "
              "decoder_planner_total_us=%u decoder_planner_max_us=%u "
              "poll_total_us=%u poll_max_us=%u\n",
        side_polls, side_adc_total, side_adc_max, side_work_total,
        side_work_max, side_poll_total, side_poll_max);
#endif
    tm_printf((UB *)"LINE profile=1 phase=%u decision=%u reason=%u pending=%u "
                    "action=%u flags=0x%x raw=%u,%u,%u level=%u,%u rows=%u "
                    "overwritten=%u\n",
              phase, decision, reason, pending, action, flags, raw[0], raw[1],
              raw[2], levels[0], levels[1], count, overwritten);
    tm_printf((UB *)"CAL_FRONT white=%u,%u black=%u,%u mask=%u valid=%u "
                    "origin=%u (0=preset 1=SDK28 2=RTOS_saved 3=RAM)\n",
              cal.white[0], cal.white[1], cal.black[0], cal.black[1], cal.mask,
              calibration_valid(&cal), origin);
    unsigned jf_value = (junction.aux[1] >> 12) & 63U;
    tm_printf((UB *)"JUNCTION policy=angle_only angle_x10=%d both_white=%u "
                    "prior_white=%u departure_ready=%u branch_accepted=%u "
                    "accepted_without_white=%u\n",
              rl_log_angle(&junction), jf_value & 1U, (jf_value >> 1) & 1U,
              (jf_value >> 2) & 1U, (jf_value >> 3) & 1U, (jf_value >> 4) & 1U);
    tm_printf((UB *)"ADC state=%u good=%u frames=%u conversions=%u "
                    "avoided_timeout=%u timeouts=%u errors=%u init_errors=%u "
                    "bad_arguments=%u retries=%u recoveries=%u ch0=%u ch1=%u "
                    "ch2=%u fail0=%u fail1=%u fail2=%u\n",
              health, good, frames, diag.conversions, diag.avoided_timeout,
              diag.timeouts, diag.errors, diag.init_errors, diag.bad_arguments,
              retries, recoveries, diag.read_ok[0], diag.read_ok[1],
              diag.read_ok[2], diag.read_fail[0], diag.read_fail[1],
              diag.read_fail[2]);
    const rl_adc_failure_t failures[2] = {diag.first, diag.last};
    for (unsigned count_value = 0; count_value < 2; ++count_value)
    {
        const rl_adc_failure_t * p_flags = &failures[count_value];
        static const char * const names[] = {
            "none",          "ready_timeout", "conversion_err",
            "reset_timeout", "bad_argument",  "init_ready_timeout"};
        tm_printf(
            (UB *)"ADC_%s reason=%s code=%u channel=%u stage=%u sample=%u "
                  "cs=0x%x elapsed_us=%u timer_us=%u clk_ctrl=0x%x "
                  "clk_div=0x%x adc_div=0x%x reset_done=0x%x\n",
            0 == count_value ? "FIRST" : "LAST",
            p_flags->reason < 6 ? names[p_flags->reason] : "unknown",
            p_flags->reason, p_flags->channel, p_flags->stage, p_flags->sample,
            p_flags->cs, p_flags->elapsed_us, p_flags->timer_us,
            p_flags->clk_ctrl, p_flags->clk_div, p_flags->adc_div,
            p_flags->resets_done);
    }
}

void rtos_line_dump_if_requested (void)
{
    UINT key = enter();
    if ((!g_dump_requested) || g_recording)
    {
        leave(key);
        return;
    }
    g_dump_requested = false;
    g_dumping = true;
    unsigned count = g_row_count, head = g_row_head,
             overwritten = g_row_overwritten;
    leave(key);
    bringup_status_t status;
    bringup_get_status(&status);
    if (status.flags & BR_ACTIVE)
    {
        goto done;
    }
#ifdef APP_RTOS_BARCODE
    if (g_barcode_dump)
    {
        barcode_export();
        goto done;
    }
#endif
    rtos_line_report();
    tm_printf((UB *)"LINE_LOG_BEGIN version=2 rows=%u overwritten=%u\n", count,
              overwritten);
    tm_printf((UB *)"LINE_LOG_CAL white=%u,%u black=%u,%u origin=%u\n",
              g_run_calibration.white[0], g_run_calibration.white[1],
              g_run_calibration.black[0], g_run_calibration.black[1],
              g_run_cal_origin);
    tm_printf((UB *)"LINE_LOG_POLICY name=angle_only departure_deg_gt=10 "
                    "departure_us=30000 branch_us=30000\n");
    tm_printf(
        (UB *)"sequence,ms,ticks_l,ticks_r,speed_l_x10,speed_r_x10,target_l,"
              "target_r,pwm_l,pwm_r,adc_l,adc_r,adc_side,level_l,level_r,phase,"
              "decision,reason,side_black,busy,white,search,pending,action,dt_"
              "us,cost_us,filtered_l,filtered_r,optical_target_l,optical_"
              "target_r,steer_gain_milli,damping_milli,stop_reason,control_"
              "flags,turn_degrees_x10,junction_both_white,junction_prior_white,"
              "junction_departure_ready,junction_branch_accepted,junction_"
              "accepted_without_white,junction_accept_event\n");
    for (unsigned count_value = 0; count_value < count; ++count_value)
    {
        while ((tm_usb_pending() > 1024) && 3 == tm_usb_state())
        {
            (void)tk_dly_tsk(2);
        }
        if (tm_usb_state() != 3)
        {
            break;
        }
        /* Starts are blocked while exporting, so rows remain frozen. */
        line_log_row_t result =
            g_rows[(head + RL_LOG_ROWS - count + count_value) % RL_LOG_ROWS];
        tm_printf(
            (UB *)"LINE_ROW "
                  "%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
                  "%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,"
                  "%u\n",
            result.sequence, result.ms, result.ticks[0], result.ticks[1],
            result.speed[0], result.speed[1], result.target[0],
            result.target[1], result.pwm[0], result.pwm[1],
            result.optics[0] & 4095U, (result.optics[0] >> 12) & 4095U,
            (result.optics[0] >> 24) | ((result.optics[1] & 15U) << 8),
            (result.optics[1] >> 4) & 1023U, (result.optics[1] >> 14) & 1023U,
            (result.optics[1] >> 24) & 15U, result.meta & 63U,
            (result.meta >> 6) & 63U, (result.meta >> 12) & 1U,
            (result.meta >> 13) & 1U, (result.meta >> 14) & 1U,
            (result.meta >> 15) & 1U, rl_route_ascii((result.meta >> 16) & 3U),
            rl_route_ascii((result.meta >> 18) & 3U), result.dt_us,
            result.cost_us, result.aux[0] & 4095U,
            (result.aux[0] >> 12) & 4095U, result.optical_target[0],
            result.optical_target[1],
            (result.aux[0] >> 24) | ((result.aux[1] & 3U) << 8),
            (result.aux[1] >> 2) & 1023U, (result.meta >> 20) & 15U,
            (result.meta >> 24) & 127U, rl_log_angle(&result),
            (result.aux[1] >> 12) & 1U, (result.aux[1] >> 13) & 1U,
            (result.aux[1] >> 14) & 1U, (result.aux[1] >> 15) & 1U,
            (result.aux[1] >> 16) & 1U, (result.aux[1] >> 17) & 1U);
    }
    tm_printf((UB *)"LINE_LOG_END rows=%u\n", count);
done:
    key = enter();
    g_dumping = false;
    leave(key);
}

#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_NETWORK)
void rtos_line_drive_mode_info (uint8_t seq, uint32_t now_ms, uint8_t out[17])
{
    uint16_t load_pwm;
    uint8_t requested = DRIVE_MODE_NONE, result = WP_ACK_ACCEPTED;
    UINT key = enter();
    load_pwm = g_view.load_pwm_packed;
    leave(key);

    motion_speed_mode_status_t mode = motion_get_speed_mode_status();
    rn_drive_speed_mode_snapshot(&requested, &result);
    if (requested > DRIVE_MODE_NONE)
    {
        requested = DRIVE_MODE_NONE;
    }
    if (result > WP_ACK_UNSUPPORTED)
    {
        result = WP_ACK_UNSUPPORTED;
    }
    bool can_change = rtos_line_mode_change_allowed();
    uint8_t load_left = (uint8_t)(load_pwm & 0x7fU);
    uint8_t load_right = (uint8_t)((load_pwm >> 7U) & 0x7fU);
    rn_drive_mode_telemetry_t snapshot = {.uptime_ms = now_ms,
                                          .revision = mode.revision,
                                          .mode = mode.mode,
                                          .requested_mode = requested,
                                          .last_result = result,
                                          .can_change = can_change,
                                          .session_active = mode.session_active,
                                          .load_left = load_left,
                                          .load_right = load_right};
    (void)rn_drive_mode_encode(&snapshot, seq, out);
}
#endif

void rtos_line_flash_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    for (;;)
    {
        board_line_flash_worker();
        (void)tk_dly_tsk(1);
    }
}
#ifdef APP_RTOS_BARCODE
#include "../barcode_navigation/barcode_runtime.inc"
#endif

#ifdef APP_RTOS_RELEASE
#include "release_exports.inc"
#endif

#ifdef APP_RTOS_RELEASE
void rtos_line_barcode_attempt_info (uint8_t seq, uint32_t now_ms,
                                     uint8_t out[30])
{
    UINT key = enter();
    rb_attempt_info_t attempt = g_view.attempt;
    uint16_t run = g_telemetry.run_id;
    leave(key);
    rb_attempt_pack(&attempt, now_ms, run, seq, out);
}
void rtos_line_junction_info (uint8_t seq, uint32_t now_ms, uint8_t out[20])
{
    UINT key = enter();
    rb_junction_sensor_t sensor = g_view.junction_sensor;
    bool pending = g_view.junction_pending;
    uint16_t run = g_telemetry.run_id;
    leave(key);
    rb_junction_pack(&sensor, pending, now_ms, run, seq, out);
}
void rtos_line_drive_info (uint8_t seq, uint32_t now_ms, uint8_t out[18])
{
    UINT key = enter();
    rl_drive_diagnostic_t diagnostic = g_view.drive_diagnostic;
    uint16_t run = g_telemetry.run_id;
    leave(key);
    rl_drive_diag_pack(&diagnostic, now_ms, run, seq, out);
}
#endif

/* End of file. */
