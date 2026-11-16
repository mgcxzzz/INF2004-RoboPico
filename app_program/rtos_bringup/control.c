/**
 * @file control.c
 * @brief Control.
 * @details Core1 owns normal motor commits. Safety paths may only remove
 * output; command/status access uses the existing kernel locks and copied
 * records.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Common migration motion/stop foundation. Step 1 uses USB fixed targets;
 * APP_RTOS_LINE adds the sensor/line provider. Both cross the unchanged shared
 * API/coordinator. Only this task calculates normal motor output. */
#include "bringup.h"
#ifdef APP_RTOS_IMU
#include "../imu_navigation/imu_service.h"
#endif
#include "src/motion/continuous_drive.h"
#ifdef APP_RTOS_NETWORK
#include "../network_bridge/network_bridge.h"
#endif
#ifdef APP_RTOS_LINE
#include "../line_navigation/line_service.h"
#include "encoder_quality.h"
static line_envelope_t g_committed_envelope;
#endif

ID g_bringup_commands;
static T_SPLOCK g_status_lock;
static bringup_status_t g_status;
static bringup_encoder_fault_snapshot_t g_encoder_fault_snapshot;
static uint32_t g_last_control_us;
static uint32_t g_deadline_us;
static uint32_t g_stopped_us;
static int16_t g_bench_target[2];
static volatile uint32_t g_urgent_stop;
static void start_pulse (unsigned selected);

static bool imu_release_blocked (void)
{
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
    return ri_safety_blocked();
#else
    return false;
#endif
}

static UINT status_enter (void)
{
    UINT key;
    (void)ISpinLock(&g_status_lock, &key);
    return key;
}

static void status_leave (UINT key)
{
    (void)ISpinUnlock(&g_status_lock, key);
}

void bringup_get_status (bringup_status_t * p_out_value)
{
    UINT key = status_enter();
    *p_out_value = g_status;
    status_leave(key);
}

void bringup_encoder_fault_packet (uint8_t seq,
                                   uint8_t out[BR_ENCODER_FAULT_PACKET_BYTES])
{
    bringup_encoder_fault_snapshot_t copy;
    UINT key = status_enter();
    copy = g_encoder_fault_snapshot;
    status_leave(key);
    br_encoder_fault_encode(&copy, seq, out);
}

void bringup_zero_status (void)
{
    uintptr_t api_key = board_motion_lock(NULL);
    UINT key = status_enter();
    g_status.pwm[0] = g_status.pwm[1] = g_status.target[0] =
        g_status.target[1] = 0;
#ifdef APP_RTOS_LINE
    g_committed_envelope = (line_envelope_t){0};
#endif
    status_leave(key);
    board_motion_unlock(NULL, api_key);
}
bool bringup_can_start (void)
{
    bool imu_blocked = imu_release_blocked();
    UINT key = status_enter();
    bool ok_value = ((br_can_start(g_status.flags, (uint32_t)board_time_us(),
                                   g_stopped_us, tm_usb_state(),
                                   (!board_button_pressed(20)) &&
                                       (!board_button_pressed(21)))) &&
                     (br_safety_allows_motion(g_status.flags, imu_blocked))) &&
                    (!g_bringup_irq_stop);
    status_leave(key);
#ifdef APP_RTOS_RELEASE
    ok_value = ok_value && (rn_drive_online());
#endif
    return ok_value;
}
bool bringup_clear_latch (void)
{
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
    bool imu_reset_allowed = ri_safety_can_reset();
#else
    bool imu_reset_allowed = true;
#endif
    UINT key = status_enter();
    bool buttons_released =
        (!board_button_pressed(20)) && (!board_button_pressed(21));
    bool ok_value =
        br_can_clear_latch(g_status.flags, buttons_released, imu_reset_allowed);
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
    /* Recheck under the IMU copy lock while the bring-up latch is protected.
     * The IMU task drops that lock before requesting this stop path. */
    if (ok_value && (g_status.flags & BR_IMU))
    {
        ok_value = ri_safety_reset_latch();
    }
#endif
    if (ok_value)
    {
        g_bringup_irq_stop = 0;
        g_status.flags &= (uint16_t)~BR_LATCHED;
        g_stopped_us = (uint32_t)board_time_us();
        br_encoder_fault_clear(&g_encoder_fault_snapshot);
    }
    status_leave(key);
#ifdef APP_RTOS_IMU
    if (ok_value)
    {
        ri_reset_request();
    }
#endif
    return ok_value;
}
void bringup_stop (uint8_t reason, uint16_t latch)
{
    bringup_stop_from(reason, latch, MOTION_BUDDY2);
}

void bringup_stop_from (uint8_t reason, uint16_t latch,
                        motion_owner_t requester)
{
    UINT key = status_enter();
    bool active = (g_status.flags & BR_ACTIVE) != 0U;
    bool new_latch = (g_status.flags & latch) != latch;
    g_status.flags = (uint16_t)((g_status.flags & ~BR_ACTIVE) | latch);
    g_status.reason = reason;
    g_status.pwm[0] = g_status.pwm[1] = 0;
    g_status.target[0] = g_status.target[1] = 0;
    if (active)
    {
        g_stopped_us = (uint32_t)board_time_us();
    }
    status_leave(key);
#ifdef APP_RTOS_LINE
    rtos_line_cancel(); /* Out-of-queue cancellation survives a full command
                           buffer. */
#endif
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
    rn_drive_cancel();
#endif
    (void)motion_stop(requester);
    motion_session_end(reason);
    bringup_zero_status();
    if (active || new_latch)
    {
        tm_printf((UB *)"STOP timer_us=%u reason=%u latch=0x%x\n",
                  (uint32_t)board_time_us(), reason, latch);
    }
}

#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
void bringup_imu_safety_trip (unsigned reason)
{
    uint8_t stop_reason;
    if (1U == reason)
    {
        stop_reason = BR_REASON_IMU_LIFT;
    }
    else if (2U == reason)
    {
        stop_reason = BR_REASON_IMU_OVERTURN;
    }
    else if (3U == reason)
    {
        stop_reason = BR_REASON_IMU_UNAVAILABLE;
    }
    else
    {
        return;
    }

    bringup_status_t current;
    bringup_get_status(&current);
    bool pending = false;
#ifdef APP_RTOS_LINE
    pending = rtos_line_pending();
#endif
    bool motion_in_progress = ((current.flags & BR_ACTIVE) != 0U) || pending;
    if (3U == reason && (!motion_in_progress))
    {
        return; /* Readiness still blocks Start, without a false run-stop latch.
                 */
    }
    if ((current.flags & BR_IMU) != 0U)
    {
        return; /* Preserve the first cause until explicit Reset. */
    }

    bringup_stop_from(stop_reason, BR_IMU, MOTION_BUDDY4);
}
#endif

ER bringup_create_objects (void)
{
    T_CMBF definition = {0};
    definition.mbfatr = TA_TFIFO;
    definition.bufsz = 256;
    definition.maxmsz = 5; /* ASCII command + received-at u32, explicit LE. */
    g_bringup_commands = tk_cre_mbf(&definition);
    return g_bringup_commands < E_OK ? g_bringup_commands : E_OK;
}

static void apply_outputs (void * p_raw)
{
#ifdef APP_RTOS_NET_BENCH
    (void)p_raw;
    board_motor_off(); /* Hardware output gate, independent of command parsing.
                        */
    return;
#endif
#ifdef APP_RTOS_LINE
    const line_envelope_t * p_envelope = p_raw;
#else
    const int32_t * pwm = p_raw;
#endif
    bool imu_blocked = imu_release_blocked();
    UINT key = status_enter();
    bool allowed =
        (!imu_blocked) && 0U == g_urgent_stop &&
        (br_outputs_allowed(
            g_status.flags, g_bringup_irq_stop != 0U, tm_usb_state(),
            (!board_button_pressed(20U)) && (!board_button_pressed(21U))));
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
    allowed = (allowed && (!rn_drive_paused())) && (rn_drive_online());
#endif
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
#ifdef APP_RTOS_LINE
        dual_levels_t levels =
            allowed ? rl_levels(p_envelope, wheel, board_time_us())
                    : (dual_levels_t){0};
        board_motor_levels(wheel, levels.a, levels.b);
        int sign = (0 == wheel ? levels.a : levels.b) ? 1 : -1;
        g_status.pwm[wheel] = (int16_t)(sign * (int)(levels.a + levels.b));
#else
        unsigned value = allowed && pwm[wheel] > 0 ? (unsigned)pwm[wheel] : 0U;
        board_motor_write(wheel, value);
        status.pwm[wheel] = (int16_t)value;
#endif
    }
#ifdef APP_RTOS_LINE
    g_committed_envelope = allowed ? *p_envelope : (line_envelope_t){0};
#endif
    status_leave(key);
}

void bringup_control_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    board_motor_init();
    bool gpio_ok = board_gpio_init(); /* This core owns IO_IRQ_BANK0 and encoder
                                         draining. */
#ifdef APP_RTOS_LINE
    ER line_result = rtos_line_init();
    gpio_ok = gpio_ok && E_OK == line_result;
    if (line_result != E_OK)
    {
        tm_printf((UB *)"LINE_INIT failed result=%d; starts inhibited.\n",
                  line_result);
    }
    double path = 0;
    uint32_t bad_direction_us[2] = {0, 0};
    br_encoder_quality_t encoder_quality[2] = {0};
#endif
    uint64_t previous = board_time_us(), due = previous + BR_CONTROL_US;
    uint32_t clock_start = (uint32_t)previous, kernel_start = board_kernel_ms();
#ifndef APP_RTOS_LINE
    uint32_t last_progress[2] = {(uint32_t)previous, (uint32_t)previous};
#endif
    int64_t totals[2] = {0, 0};
    float speed[2] = {0, 0};
#ifndef APP_RTOS_LINE
    continuous_drive_t drive[2] = {0};
    signed_wheel_t wheel[2] = {0};
#endif
    UINT key = status_enter();
    g_status.flags |= gpio_ok ? BR_READY : BR_ENCODER;
    g_status.min_dt_us = UINT32_MAX;
    g_stopped_us = (uint32_t)previous;
    g_last_control_us = (uint32_t)previous;
    status_leave(key);

    for (;;)
    {
        uint64_t now = board_time_us();
#ifdef APP_RTOS_LINE
        rtos_line_side_poll(now);
#endif
        if (now < due)
        {
            (void)tk_dly_tsk(1);
            continue;
        }
        uint32_t dt_us = (uint32_t)(now - previous);
        previous = now;
        due += BR_CONTROL_US;
        if (due <= now)
        {
            due = now + BR_CONTROL_US; /* No catch-up PWM bursts. */
        }
        int32_t delta[2];
        uint32_t edges[2], invalid[2];
        board_encoder_drain(delta, edges, invalid);
        float delta_time = (float)dt_us / 1000000.0f;
        for (unsigned wheel = 0U; wheel < 2U; ++wheel)
        {
            totals[wheel] += delta[wheel];
            speed[wheel] +=
                0.25f *
                (raw_to_mm(wheel, delta[wheel]) / delta_time - speed[wheel]);
#ifdef APP_RTOS_LINE
            path += fabs((double)raw_to_mm(wheel, delta[wheel])) * .5;
#endif
        }

        /* A gap while stopped cannot strand the first bench request. Keep
         * its timing evidence and require three NEW valid clock windows.
         * Never clear an existing stop latch or forgive an active-run gap. */
        key = status_enter();
        unsigned gap_action = br_control_gap_action(g_status.flags, dt_us);
        if (BR_GAP_REVALIDATE == gap_action)
        {
            g_status.flags &= (uint16_t)~BR_CLOCK_OK;
            g_status.clock_good = 0U;
            if (0U == (g_status.flags & BR_LATCHED))
            {
                g_status.reason = BR_REASON_LATE;
            }
        }
        status_leave(key);
        if (BR_GAP_STOP == gap_action)
        {
            bringup_stop(BR_REASON_LATE, BR_LATE);
        }
        else if (BR_GAP_REVALIDATE == gap_action)
        {
            clock_start = (uint32_t)now;
            kernel_start = board_kernel_ms();
            tm_printf((UB *)"IDLE_CONTROL_GAP dt_us=%u; motor start inhibited "
                            "pending 3 fresh clock windows; stop latches "
                            "preserved.\n",
                      dt_us);
        }

        if ((uint32_t)now - clock_start >= BR_CLOCK_WINDOW_US)
        {
            uint32_t kernel_now = board_kernel_ms();
            uint32_t kernel_delta = kernel_now - kernel_start;
            uint32_t timer_delta = (uint32_t)now - clock_start;
            bool good = (board_clock_registers_ok()) &&
                        (br_clock_elapsed_ok(timer_delta, kernel_delta));
            key = status_enter();
            g_status.clock_kernel_ms = kernel_delta;
            g_status.clock_timer_us = timer_delta;
            g_status.clock_samples++;
            g_status.clock_good =
                good ? (g_status.clock_good < BR_CLOCK_GOOD_WINDOWS
                            ? g_status.clock_good + 1U
                            : BR_CLOCK_GOOD_WINDOWS)
                     : 0U;
            if (g_status.clock_good >= BR_CLOCK_GOOD_WINDOWS)
            {
                g_status.flags |= BR_CLOCK_OK;
            }
            else
            {
                g_status.flags &= (uint16_t)~BR_CLOCK_OK;
            }
            bool running = (g_status.flags & BR_ACTIVE) != 0U;
            status_leave(key);
            if ((!good) && running)
            {
                bringup_stop(BR_REASON_CLOCK, BR_ESTOP);
            }
            clock_start = (uint32_t)now;
            kernel_start = kernel_now;
        }

        motion_feedback_t feedback = {
            .now_us = now,
            .wheel_mm = {raw_to_mm(0U, totals[0]), raw_to_mm(1U, totals[1])},
            .speed_mm_s = {speed[0], speed[1]}};
        bringup_status_t snapshot;
        bringup_get_status(&snapshot);
        bool active = (snapshot.flags & BR_ACTIVE) != 0U;
#ifdef APP_RTOS_LINE
        bool start_line =
            rtos_line_prepare(&feedback, path, delta_time, active);
        if ((!rtos_line_ready()) && active)
        {
            bringup_status_t current;
            bringup_get_status(&current);
            if (current.flags & BR_ACTIVE)
            {
                bringup_stop(BR_REASON_ADC, BR_ESTOP);
            }
            active = false;
        }
#ifdef APP_RTOS_NETWORK
        rn_drive_process(&feedback);
        if (!rn_drive_online_record(active || (rtos_line_pending())))
        {
            if (active || (rtos_line_pending()))
            {
                bringup_stop_from(BR_REASON_NETWORK, 0, MOTION_BUDDY1);
            }
            start_line = false;
        }
        bringup_get_status(&snapshot);
        active = !!(snapshot.flags & BR_ACTIVE);
        bool paused = rn_drive_paused();
#else
        bool paused = false;
#endif
        if (start_line && (rtos_line_ready()))
        {
            start_pulse(3);
            bringup_get_status(&snapshot);
            active = (snapshot.flags & BR_ACTIVE) != 0U;
            if (active)
            {
                rtos_line_begin();
#ifdef APP_RTOS_NETWORK
                rn_drive_begin();
#endif
                for (unsigned wheel_value = 0; wheel_value < 2; ++wheel_value)
                {
                    br_encoder_reset(&encoder_quality[wheel_value],
                                     (uint32_t)now, edges[wheel_value],
                                     invalid[wheel_value]);
                    bad_direction_us[wheel_value] = 0;
                }
                key = status_enter();
                g_status.encoder_kind = g_status.encoder_wheel = 0;
                status_leave(key);
            }
        }
#ifdef APP_RTOS_IMU
        motion_status_t imu_status = motion_get_status();
        bool imu_running =
            ((active && (!paused)) && (imu_status.session_active)) &&
            MOTION_AUTO == imu_status.mode;
        ri_observe(&feedback, invalid, dt_us, imu_running);
        if (!imu_running)
        {
            ri_ramp_halt(now);
        }
#endif
        analog_demand_t optical = {0};
        if (active && paused)
        {
            rtos_line_feedback(&feedback, NULL);
        }
        else if (active)
        {
            optical = rtos_line_follow(delta_time);
            rtos_line_feedback(&feedback, &optical);
            if (optical.stop)
            {
                bringup_stop(BR_REASON_API, 0);
                active = false;
            }
            else if (motion_submit_navigation(MOTION_BUDDY3, optical.target[0],
                                              optical.target[1]) != MOTION_OK)
            {
                /* A simultaneous normal STOP already invalidated the API
                 * session; do not turn that successful stop into a new fault.
                 */
                bringup_status_t current;
                bringup_get_status(&current);
                if (current.flags & BR_ACTIVE)
                {
                    bringup_stop(BR_REASON_API, BR_ESTOP);
                }
                active = false;
            }
        }
        motion_output_t demand = motion_update(&feedback);
        if ((active && (demand.external)) && (!paused))
        {
            bringup_status_t current;
            bringup_get_status(&current);
            if (current.flags & BR_ACTIVE)
            {
                bringup_stop(BR_REASON_API, BR_ESTOP);
            }
            active = false;
        }
#ifdef APP_RTOS_NETWORK
        if ((active && paused) && MOTION_FAULT == motion_get_status().mode)
        {
            bringup_stop(BR_REASON_API, BR_ESTOP);
            active = false;
        }
#endif
        line_envelope_t envelope = {0};
        if (active)
        {
            /* Age the quality window during Pause too. Resume is not a new
             * session and cannot turn sparse lifetime errors into a fault. */
            for (unsigned wheel_value_value = 0; wheel_value_value < 2;
                 ++wheel_value_value)
            {
                unsigned kind = br_encoder_quality(
                    &encoder_quality[wheel_value_value], (uint32_t)now,
                    edges[wheel_value_value], invalid[wheel_value_value]);
                bad_direction_us[wheel_value_value] =
                    ((!paused) && (optical.target[wheel_value_value] != 0)) &&
                            (speed[wheel_value_value] *
                                 (optical.target[wheel_value_value] > 0 ? 1
                                                                        : -1) <
                             -20)
                        ? bad_direction_us[wheel_value_value] + dt_us
                        : 0;
                if (bad_direction_us[wheel_value_value] >= 50000)
                {
                    kind = BR_ENC_DIRECTION;
                }
                bool trip = (!paused) && (kind != BR_ENC_OK);
                bringup_encoder_fault_snapshot_t candidate = {0};
                if (trip)
                {
                    candidate.valid = 1U;
                    candidate.kind = (uint8_t)kind;
                    candidate.wheel = (uint8_t)(wheel_value_value + 1U);
                    candidate.control_timer_us = (uint32_t)now;
                    candidate.kernel_ms = board_kernel_ms();
                    candidate.direction_duration_us =
                        BR_ENC_DIRECTION == kind
                            ? bad_direction_us[wheel_value_value]
                            : 0U;
                    candidate.recent_window_bad =
                        encoder_quality[wheel_value_value].window_bad;
                    candidate.recent_window_changes =
                        encoder_quality[wheel_value_value].window_changes;
                    for (unsigned index = 0U; index < 2U; ++index)
                    {
                        candidate.lifetime_edges[index] = edges[index];
                        candidate.lifetime_invalid[index] = invalid[index];
                        candidate.optical_target_mm_s[index] =
                            optical.target[index];
                        candidate.measured_speed_x10[index] =
                            (int16_t)continuous_clampf(speed[index] * 10.0f,
                                                       -32767, 32767);
                    }
                }
                key = status_enter();
                g_status.encoder_bad[wheel_value_value] =
                    encoder_quality[wheel_value_value].window_bad;
                g_status.encoder_changes[wheel_value_value] =
                    encoder_quality[wheel_value_value].window_changes;
                if (((!paused) && kind) && (!g_status.encoder_kind))
                {
                    g_status.encoder_kind = kind;
                    g_status.encoder_wheel = wheel_value_value + 1;
                }
                if (trip)
                {
                    candidate.previous_pwm[0] = g_status.pwm[0];
                    candidate.previous_pwm[1] = g_status.pwm[1];
                    (void)br_encoder_fault_capture_once(
                        &g_encoder_fault_snapshot, &candidate);
                }
                status_leave(key);
                if ((!paused) && kind)
                {
                    bringup_stop(BR_REASON_ENCODER, BR_ENCODER);
                    tm_printf(
                        (UB *)"ENCODER_TRIP wheel=%u trip_kind=%u "
                              "window_bad=%u window_changes=%u\n",
                        wheel_value_value + 1, kind,
                        encoder_quality[wheel_value_value].window_bad,
                        encoder_quality[wheel_value_value].window_changes);
                    active = false;
                    break;
                }
            }
        }
        if (active && (!paused))
        {
            uintptr_t api_key = board_motion_lock(NULL);
            line_envelope_t previous_envelope = g_committed_envelope;
            board_motion_unlock(NULL, api_key);
            for (unsigned wheel_value_value_value = 0;
                 wheel_value_value_value < 2; ++wheel_value_value_value)
            {
                rtos_line_actual_output(wheel_value_value_value,
                                        rl_cap(&previous_envelope,
                                               wheel_value_value_value,
                                               board_time_us()));
            }
            envelope = rtos_line_power(&demand, &optical, delta_time);
            bool committed =
                motion_commit(demand.generation, apply_outputs, &envelope);
            for (unsigned wheel_value_value_value_value = 0;
                 wheel_value_value_value_value < 2;
                 ++wheel_value_value_value_value)
            {
                rtos_line_actual_output(
                    wheel_value_value_value_value,
                    committed ? rl_cap(&envelope, wheel_value_value_value_value,
                                       board_time_us())
                              : 0);
            }
        }
#else
        motion_output_t demand = motion_update(&feedback);
        int32_t pwm[2] = {0, 0};
        for (unsigned wheel = 0U; wheel < 2U; ++wheel)
        {
            if (!active || demand.reset_controllers)
            {
                continuous_drive_reset(&drive[wheel], &wheel[wheel]);
            }
            if (active)
            {
                pwm[wheel] = continuous_drive_step(
                    &drive[wheel], &wheel[wheel], wheel,
                    demand.target_mm_s[wheel], speed[wheel], delta_time, now,
                    snapshot.pwm[wheel]);
            }
            if (!active || demand.target_mm_s[wheel] == 0 ||
                delta[wheel] != 0 || pwm[wheel] < 120)
            {
                last_progress[wheel] = (uint32_t)now;
            }
            else if ((uint32_t)now - last_progress[wheel] >= BR_STALL_US)
            {
                bringup_stop(BR_REASON_STALL, BR_STALL);
            }
        }
        if (active)
        {
            (void)motion_commit(demand.generation, apply_outputs, pwm);
        }
#endif

        key = status_enter();
        g_status.kernel_ms = board_kernel_ms();
        g_status.timer_us = (uint32_t)now;
        g_status.control_count++;
        g_status.dt_us = dt_us;
        if (dt_us < g_status.min_dt_us)
        {
            g_status.min_dt_us = dt_us;
        }
        if (dt_us > g_status.max_dt_us)
        {
            g_status.max_dt_us = dt_us;
        }
        if (dt_us > 15000U)
        {
            g_status.late_count++;
        }
        for (unsigned wheel_value_value_value_value_value = 0U;
             wheel_value_value_value_value_value < 2U;
             ++wheel_value_value_value_value_value)
        {
            /* Raw diagnostic count saturates; odometry remains 64-bit. */
            g_status.ticks[wheel_value_value_value_value_value] =
                totals[wheel_value_value_value_value_value] > INT32_MAX
                    ? INT32_MAX
                : totals[wheel_value_value_value_value_value] < INT32_MIN
                    ? INT32_MIN
                    : (int32_t)totals[wheel_value_value_value_value_value];
            g_status.edges[wheel_value_value_value_value_value] =
                edges[wheel_value_value_value_value_value];
            g_status.invalid[wheel_value_value_value_value_value] =
                invalid[wheel_value_value_value_value_value];
            g_status.speed_x10[wheel_value_value_value_value_value] =
                (int16_t)(continuous_clampf(
                    speed[wheel_value_value_value_value_value] * 10.0f, -32767,
                    32767));
            g_status.target[wheel_value_value_value_value_value] =
                (g_status.flags & BR_ACTIVE)
                    ? demand.target_mm_s[wheel_value_value_value_value_value]
                    : 0;
        }
        g_last_control_us = (uint32_t)now;
        uint32_t cost = (uint32_t)(board_time_us() - now);
        if (cost > g_status.max_cost_us)
        {
            g_status.max_cost_us = cost;
        }
        status_leave(key);
#ifdef APP_RTOS_LINE
        bringup_get_status(&snapshot);
        rtos_line_record(&snapshot, &optical, cost);
#endif
    }
}

void bringup_safety_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    for (;;)
    {
        bringup_status_t snapshot;
        UINT key = status_enter();
        snapshot = g_status;
        uint32_t last = g_last_control_us, end = g_deadline_us;
        uint32_t urgent = g_urgent_stop;
        g_urgent_stop = 0U;
        status_leave(key);
        uint32_t now = (uint32_t)board_time_us();
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
        unsigned imu_reason = ri_safety_reason();
        bool pending = rtos_line_pending();
        if (1U == imu_reason || 2U == imu_reason ||
            (3U == imu_reason &&
             (((snapshot.flags & BR_ACTIVE) != 0U) || pending)))
        {
            bringup_imu_safety_trip(imu_reason);
            bringup_get_status(&snapshot);
        }
#endif
        bool active = (snapshot.flags & BR_ACTIVE) != 0U;
#ifdef APP_RTOS_NETWORK
        unsigned net_urgent = rn_take_urgent();
        if (net_urgent)
        {
            bringup_stop_from(BR_REASON_NETWORK,
                              net_urgent & RN_URGENT_ESTOP ? BR_ESTOP : 0,
                              MOTION_BUDDY1);
        }
#ifdef APP_RTOS_LINE
        if (active && (!rn_drive_online()))
        {
            bringup_stop_from(BR_REASON_NETWORK, 0, MOTION_BUDDY1);
        }
#endif
#endif
        if ((snapshot.flags & BR_READY) != 0U)
        {
            if ((((g_bringup_irq_stop != 0U) || (board_button_pressed(20U))) ||
                 'e' == urgent) &&
                (active || 0U == (snapshot.flags & BR_ESTOP)))
            {
                bringup_stop('e' == urgent ? BR_REASON_SERIAL : BR_REASON_GP20,
                             BR_ESTOP);
            }
            else if ('s' == urgent)
            {
                bringup_stop(BR_REASON_SERIAL, 0U);
            }
            else if (active && (board_button_pressed(21U)))
            {
                bringup_stop(BR_REASON_GP21, 0U);
            }
            else if (active && (!br_usb_supervised(tm_usb_state())))
            {
                bringup_stop(BR_REASON_USB, 0U);
            }
            else if (active && (now - last >= BR_LATE_US))
            {
                bringup_stop(BR_REASON_LATE, BR_LATE);
            }
            else if ((active && end) && ((int32_t)(now - end) >= 0))
            {
                bringup_stop(BR_REASON_DONE, 0U);
            }
        }
#ifdef APP_RTOS_LINE
        /* Match the SDK's 1ms peak-expiry envelope. This does not calculate
         * PWM or edit PI state; it can only reduce the committed output. */
        uintptr_t api_key = board_motion_lock(NULL);
        key = status_enter();
        if ((g_status.flags & BR_ACTIVE) && (!g_bringup_irq_stop))
        {
            bool allowed =
                (!imu_release_blocked()) &&
                (br_outputs_allowed(g_status.flags, g_bringup_irq_stop != 0U,
                                    tm_usb_state(),
                                    (!board_button_pressed(20U)) &&
                                        (!board_button_pressed(21U))));
            for (unsigned wheel = 0; wheel < 2; ++wheel)
            {
                dual_levels_t cap = allowed ? rl_levels(&g_committed_envelope,
                                                        wheel, board_time_us())
                                            : (dual_levels_t){0};
                dual_levels_t actual = board_motor_current(wheel);
                dual_levels_t reduced = rl_reduce(actual, cap);
                if ((reduced.a != actual.a) || (reduced.b != actual.b))
                {
                    board_motor_levels(wheel, reduced.a, reduced.b);
                }
                int sign = (0 == wheel ? reduced.a : reduced.b) ? 1 : -1;
                g_status.pwm[wheel] =
                    (int16_t)(sign * (int)(reduced.a + reduced.b));
            }
        }
        status_leave(key);
        board_motion_unlock(NULL, api_key);
#endif
        (void)tk_dly_tsk(1);
    }
}

static void start_pulse (unsigned selected)
{
#ifdef APP_RTOS_NET_BENCH
    (void)selected;
    tm_printf((UB *)"NETWORK BENCH: motor starts disabled; use Step2 firmware "
                    "for driving.\n");
    return;
#endif
    bringup_status_t snapshot;
    UINT key = status_enter();
    snapshot = g_status;
    uint32_t stopped = g_stopped_us;
    status_leave(key);
    uint64_t now = board_time_us();
#ifdef APP_RTOS_RELEASE
    if (!rn_drive_online())
    {
        return;
    }
#endif
    if (((!br_can_start(snapshot.flags, (uint32_t)now, stopped, tm_usb_state(),
                        (!board_button_pressed(20U)) &&
                            (!board_button_pressed(21U)))) ||
         (!br_safety_allows_motion(snapshot.flags, imu_release_blocked()))) ||
        g_bringup_irq_stop)
    {
        tm_printf((UB *)"REQUEST rejected: wait for CLOCK PASS, release "
                        "buttons, clear a latch with x, and allow 1s rest.\n");
        return;
    }
    motion_session_begin(now);
    int16_t left = 2U == selected ? 0 : BR_SPEED_MM_S;
    int16_t right = 1U == selected ? 0 : BR_SPEED_MM_S;
    motion_result_t result =
        motion_submit_navigation(MOTION_BUDDY3, left, right);
    if (result != MOTION_OK)
    {
        bringup_stop(BR_REASON_API, 0U);
        return;
    }
    key = status_enter();
    bool accepted =
        0U == g_bringup_irq_stop && 0U == g_urgent_stop &&
        (br_safety_allows_motion(g_status.flags, imu_release_blocked()));
#ifdef APP_RTOS_LINE
    accepted = accepted && (rtos_line_start_valid());
#endif
    if (accepted)
    {
        g_deadline_us = (uint32_t)now + BR_PULSE_US;
#ifdef APP_RTOS_LINE
        g_deadline_us =
            0; /* Line session ends on a stop/fault, not a 3s bench pulse. */
#endif
        g_bench_target[0] = left;
        g_bench_target[1] = right;
        g_status.flags |= BR_ACTIVE;
        br_encoder_fault_clear(&g_encoder_fault_snapshot);
    }
    status_leave(key);
    if (!accepted)
    {
        motion_session_end(BR_REASON_SERIAL);
        return;
    }
#ifdef APP_RTOS_LINE
    tm_printf((UB *)"LINE accepted: Buddy3 -> NAV -> MOTION; profile1, "
                    "configured link supervision active.\n");
#else
    tm_printf((UB *)"REQUEST USB_BENCH -> Buddy3 target stub -> NAV -> MOTION: "
                    "L=%d R=%d mm/s; automatic stop after 3s.\n",
              left, right);
#endif
}

void bringup_nav_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    for (;;)
    {
        uint8_t msg[5];
        INT count = tk_rcv_mbf(g_bringup_commands, msg, 10);
        /* Keep the existing coordinator's 100ms stale-target watchdog.
         * The bench target provider, like Buddy 3 later, must keep reporting.
         */
        UINT key = status_enter();
        bool active = (g_status.flags & BR_ACTIVE) != 0U;
        int16_t left = g_bench_target[0], right = g_bench_target[1];
        status_leave(key);
#ifdef APP_RTOS_LINE
        (void)active;
        (void)left;
        (void)right;
#else
        if (active &&
            motion_submit_navigation(MOTION_BUDDY3, left, right) != MOTION_OK)
        {
            bringup_status_t after;
            bringup_get_status(&after);
            if ((after.flags & BR_ACTIVE) != 0U)
            {
                bringup_stop(BR_REASON_API, BR_ESTOP);
            }
        }
#endif
        if (count != 5)
        {
            continue;
        }
        uint32_t received = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8U) |
                            ((uint32_t)msg[3] << 16U) |
                            ((uint32_t)msg[4] << 24U);
        unsigned command = msg[0];
        if (!br_command_fresh((uint32_t)board_time_us(), received))
        {
            continue;
        }
        if ((command >= '1') && (command <= '3'))
#ifdef APP_RTOS_LINE
            tm_printf((UB *)"Step2 uses profile1 only; g starts following. Use "
                            "Step1 firmware for bench pulses.\n");
#else
            start_pulse(command - '0');
#endif
        else if ('s' == command || 'e' == command)
        {
            bringup_stop(BR_REASON_SERIAL, 'e' == command ? BR_ESTOP : 0U);
        }
        else if ('x' == command)
        {
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
            if (!bringup_clear_latch())
            {
                tm_printf(
                    (UB *)"RESET rejected: keep the car upright and still, "
                          "release buttons, and retry when IMU is fresh.\n");
            }
#else
            UINT key = status_enter();
            if (0U == (g_status.flags & BR_ACTIVE) &&
                (!board_button_pressed(20U)) && (!board_button_pressed(21U)) &&
                ((g_status.flags & BR_CLOCK_OK) != 0U))
            {
                g_bringup_irq_stop = 0U;
                g_status.flags &= (uint16_t)~BR_LATCHED;
                g_stopped_us = (uint32_t)board_time_us();
                br_encoder_fault_clear(&g_encoder_fault_snapshot);
            }
            status_leave(key);
#endif
            bringup_report();
        }
#ifdef APP_RTOS_LINE
        else if ('g' == command || 'w' == command || 'l' == command ||
                 'r' == command || 'c' == command || 'k' == command ||
                 'd' == command
#ifdef APP_RTOS_BARCODE
                 || 'b' == command
#endif
        )
        {
            if (!rtos_line_command(command, received))
            {
                tm_printf((UB *)"LINE command queue full; command rejected.\n");
            }
        }
        else if ('L' == command || 'R' == command || 'U' == command)
        {
#ifdef APP_RTOS_BARCODE
            tm_printf((UB *)"Step4: use an actual mapped barcode; canonical "
                            "USB L/R/U disabled.\n");
#else
            char route = command == 'L' ? 'A' : command == 'R' ? 'F' : 'Z';
            motion_result_t result =
                motion_submit_barcode(MOTION_BUDDY3, route);
            tm_printf((UB *)"USB route %c -> Buddy3 -> NAV result=%u; await "
                            "separate junction.\n",
                      command, result);
#endif
        }
#endif
        else if ('h' == command || '?' == command)
        {
            bringup_help();
        }
        else if ('p' == command || 'n' == command)
        {
            bringup_report();
            board_clock_report();
        }
        else if ('z' == command)
        {
            motion_event_t event;
            for (unsigned count_value = 0U;
                 (count_value < 12U) && (motion_pop_event(&event));
                 ++count_value)
            {
                tm_printf((UB *)"NAV time_us=%u owner=%u requester=%u event=%u "
                                "state=%u fault=%u result=%u target=%d,%d\n",
                          (uint32_t)event.now_us, event.owner, event.requester,
                          event.type, event.state, event.fault, event.result,
                          event.target_mm_s[0], event.target_mm_s[1]);
            }
        }
    }
}

void bringup_console_task (INT code, void * p_exinf_value)
{
    (void)code;
    (void)p_exinf_value;
    for (;;)
    {
        for (unsigned count = 0U; count < 32U; ++count)
        {
            INT value = tm_usb_console_get();
            if (value < 0)
            {
                break;
            }
            if ('\r' == value || '\n' == value || ' ' == value)
            {
                continue;
            }
            uint32_t now = (uint32_t)board_time_us();
            uint8_t msg[5] = {(uint8_t)value, (uint8_t)now,
                              (uint8_t)(now >> 8U), (uint8_t)(now >> 16U),
                              (uint8_t)(now >> 24U)};
            /* Urgent stop survives a full command queue. It is consumed by
             * the Safety Supervisor, which calls the same motion stop API. */
            if ('s' == value || 'e' == value)
            {
                UINT key = status_enter();
                if (g_urgent_stop != 'e')
                {
                    g_urgent_stop = (uint32_t)value;
                }
                status_leave(key);
            }
            (void)tk_snd_mbf(g_bringup_commands, msg, 5, TMO_POL);
        }
        (void)tk_dly_tsk(1);
    }
}

/* End of file. */
