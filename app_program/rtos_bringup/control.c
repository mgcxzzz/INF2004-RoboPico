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
static LineEnvelope committed_envelope;
#endif

ID bringup_commands;
static T_SPLOCK status_lock;
static BringupStatus status;
static BringupEncoderFaultSnapshot encoder_fault_snapshot;
static uint32_t last_control_us, deadline_us, stopped_us;
static int16_t bench_target[2];
static volatile uint32_t urgent_stop;
static void start_pulse(unsigned selected);

static UINT status_enter(void) {
    UINT key;
    (void)ISpinLock(&status_lock, &key);
    return key;
}

static void status_leave(UINT key) {
    (void)ISpinUnlock(&status_lock, key);
}

void bringup_get_status(BringupStatus *out) {
    UINT key = status_enter();
    *out = status;
    status_leave(key);
}

void bringup_encoder_fault_packet(uint8_t seq, uint8_t out[BR_ENCODER_FAULT_PACKET_BYTES]) {
    BringupEncoderFaultSnapshot copy;
    UINT key = status_enter();
    copy = encoder_fault_snapshot;
    status_leave(key);
    br_encoder_fault_encode(&copy, seq, out);
}

void bringup_zero_status(void) {
    uintptr_t api_key = board_motion_lock(NULL);
    UINT key = status_enter();
    status.pwm[0] = status.pwm[1] = status.target[0] = status.target[1] = 0;
#ifdef APP_RTOS_LINE
    committed_envelope = (LineEnvelope){0};
#endif
    status_leave(key);
    board_motion_unlock(NULL, api_key);
}
bool bringup_can_start(void) {
    UINT key = status_enter();
    bool ok = br_can_start(status.flags, (uint32_t)board_time_us(), stopped_us, tm_usb_state(),
                           !board_button_pressed(20) && !board_button_pressed(21)) &&
              !bringup_irq_stop;
    status_leave(key);
#ifdef APP_RTOS_RELEASE
    ok = ok && rn_drive_online();
#endif
    return ok;
}
bool bringup_clear_latch(void) {
    UINT key = status_enter();
    bool ok = !(status.flags & BR_ACTIVE) && !board_button_pressed(20) &&
              !board_button_pressed(21) && (status.flags & BR_CLOCK_OK);
    if (ok) {
        bringup_irq_stop = 0;
        status.flags &= (uint16_t)~BR_LATCHED;
        stopped_us = (uint32_t)board_time_us();
        br_encoder_fault_clear(&encoder_fault_snapshot);
    }
    status_leave(key);
#ifdef APP_RTOS_IMU
    if (ok)
        ri_reset_request();
#endif
    return ok;
}
void bringup_stop(uint8_t reason, uint16_t latch) {
    bringup_stop_from(reason, latch, MOTION_BUDDY2);
}

void bringup_stop_from(uint8_t reason, uint16_t latch, MotionOwner requester) {
    UINT key = status_enter();
    bool active = (status.flags & BR_ACTIVE) != 0U;
    bool new_latch = (status.flags & latch) != latch;
    status.flags = (uint16_t)((status.flags & ~BR_ACTIVE) | latch);
    status.reason = reason;
    status.pwm[0] = status.pwm[1] = 0;
    status.target[0] = status.target[1] = 0;
    if (active)
        stopped_us = (uint32_t)board_time_us();
    status_leave(key);
#ifdef APP_RTOS_LINE
    rtos_line_cancel(); /* Out-of-queue cancellation survives a full command buffer. */
#endif
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
    rn_drive_cancel();
#endif
    (void)motion_stop(requester);
    motion_session_end(reason);
    bringup_zero_status();
    if (active || new_latch)
        tm_printf((UB *)"STOP timer_us=%u reason=%u latch=0x%x\n", (uint32_t)board_time_us(),
                  reason, latch);
}

ER bringup_create_objects(void) {
    T_CMBF definition = {0};
    definition.mbfatr = TA_TFIFO;
    definition.bufsz = 256;
    definition.maxmsz = 5; /* ASCII command + received-at u32, explicit LE. */
    bringup_commands = tk_cre_mbf(&definition);
    return bringup_commands < E_OK ? bringup_commands : E_OK;
}

static void apply_outputs(void *raw) {
#ifdef APP_RTOS_NET_BENCH
    (void)raw;
    board_motor_off(); /* Hardware output gate, independent of command parsing. */
    return;
#endif
#ifdef APP_RTOS_LINE
    const LineEnvelope *envelope = raw;
#else
    const int32_t *pwm = raw;
#endif
    UINT key = status_enter();
    bool allowed = urgent_stop == 0U &&
                   br_outputs_allowed(status.flags, bringup_irq_stop != 0U, tm_usb_state(),
                                      !board_button_pressed(20U) && !board_button_pressed(21U));
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
    allowed = allowed && !rn_drive_paused() && rn_drive_online();
#endif
    for (unsigned w = 0U; w < 2U; ++w) {
#ifdef APP_RTOS_LINE
        DualLevels levels = allowed ? rl_levels(envelope, w, board_time_us()) : (DualLevels){0};
        board_motor_levels(w, levels.a, levels.b);
        int sign = (w == 0 ? levels.a : levels.b) ? 1 : -1;
        status.pwm[w] = (int16_t)(sign * (int)(levels.a + levels.b));
#else
        unsigned value = allowed && pwm[w] > 0 ? (unsigned)pwm[w] : 0U;
        board_motor_write(w, value);
        status.pwm[w] = (int16_t)value;
#endif
    }
#ifdef APP_RTOS_LINE
    committed_envelope = allowed ? *envelope : (LineEnvelope){0};
#endif
    status_leave(key);
}

void bringup_control_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    board_motor_init();
    bool gpio_ok = board_gpio_init(); /* This core owns IO_IRQ_BANK0 and encoder draining. */
#ifdef APP_RTOS_LINE
    ER line_result = rtos_line_init();
    gpio_ok = gpio_ok && line_result == E_OK;
    if (line_result != E_OK)
        tm_printf((UB *)"LINE_INIT failed result=%d; starts inhibited.\n", line_result);
    double path = 0;
    uint32_t bad_direction_us[2] = {0, 0};
    BrEncoderQuality encoder_quality[2] = {0};
#endif
    uint64_t previous = board_time_us(), due = previous + BR_CONTROL_US;
    uint32_t clock_start = (uint32_t)previous, kernel_start = board_kernel_ms();
#ifndef APP_RTOS_LINE
    uint32_t last_progress[2] = {(uint32_t)previous, (uint32_t)previous};
#endif
    int64_t totals[2] = {0, 0};
    float speed[2] = {0, 0};
#ifndef APP_RTOS_LINE
    ContinuousDrive drive[2] = {0};
    SignedWheel wheel[2] = {0};
#endif
    UINT key = status_enter();
    status.flags |= gpio_ok ? BR_READY : BR_ENCODER;
    status.min_dt_us = UINT32_MAX;
    stopped_us = (uint32_t)previous;
    last_control_us = (uint32_t)previous;
    status_leave(key);

    for (;;) {
        uint64_t now = board_time_us();
#ifdef APP_RTOS_LINE
        rtos_line_side_poll(now);
#endif
        if (now < due) {
            (void)tk_dly_tsk(1);
            continue;
        }
        uint32_t dt_us = (uint32_t)(now - previous);
        previous = now;
        due += BR_CONTROL_US;
        if (due <= now)
            due = now + BR_CONTROL_US; /* No catch-up PWM bursts. */
        int32_t delta[2];
        uint32_t edges[2], invalid[2];
        board_encoder_drain(delta, edges, invalid);
        float dt = (float)dt_us / 1000000.0f;
        for (unsigned w = 0U; w < 2U; ++w) {
            totals[w] += delta[w];
            speed[w] += 0.25f * (raw_to_mm(w, delta[w]) / dt - speed[w]);
#ifdef APP_RTOS_LINE
            path += fabs((double)raw_to_mm(w, delta[w])) * .5;
#endif
        }

        /* A gap while stopped cannot strand the first bench request. Keep
         * its timing evidence and require three NEW valid clock windows.
         * Never clear an existing stop latch or forgive an active-run gap. */
        key = status_enter();
        unsigned gap_action = br_control_gap_action(status.flags, dt_us);
        if (gap_action == BR_GAP_REVALIDATE) {
            status.flags &= (uint16_t)~BR_CLOCK_OK;
            status.clock_good = 0U;
            if ((status.flags & BR_LATCHED) == 0U)
                status.reason = BR_REASON_LATE;
        }
        status_leave(key);
        if (gap_action == BR_GAP_STOP) {
            bringup_stop(BR_REASON_LATE, BR_LATE);
        } else if (gap_action == BR_GAP_REVALIDATE) {
            clock_start = (uint32_t)now;
            kernel_start = board_kernel_ms();
            tm_printf(
                (UB *)"IDLE_CONTROL_GAP dt_us=%u; motor start inhibited pending 3 fresh clock windows; stop latches preserved.\n",
                dt_us);
        }

        if ((uint32_t)now - clock_start >= BR_CLOCK_WINDOW_US) {
            uint32_t kernel_now = board_kernel_ms();
            uint32_t kernel_delta = kernel_now - kernel_start;
            uint32_t timer_delta = (uint32_t)now - clock_start;
            bool good =
                board_clock_registers_ok() && br_clock_elapsed_ok(timer_delta, kernel_delta);
            key = status_enter();
            status.clock_kernel_ms = kernel_delta;
            status.clock_timer_us = timer_delta;
            status.clock_samples++;
            status.clock_good =
                good ? (status.clock_good < BR_CLOCK_GOOD_WINDOWS ? status.clock_good + 1U
                                                                  : BR_CLOCK_GOOD_WINDOWS)
                     : 0U;
            if (status.clock_good >= BR_CLOCK_GOOD_WINDOWS)
                status.flags |= BR_CLOCK_OK;
            else
                status.flags &= (uint16_t)~BR_CLOCK_OK;
            bool running = (status.flags & BR_ACTIVE) != 0U;
            status_leave(key);
            if (!good && running)
                bringup_stop(BR_REASON_CLOCK, BR_ESTOP);
            clock_start = (uint32_t)now;
            kernel_start = kernel_now;
        }

        MotionFeedback feedback = {.now_us = now,
                                   .wheel_mm = {raw_to_mm(0U, totals[0]), raw_to_mm(1U, totals[1])},
                                   .speed_mm_s = {speed[0], speed[1]}};
        BringupStatus snapshot;
        bringup_get_status(&snapshot);
        bool active = (snapshot.flags & BR_ACTIVE) != 0U;
#ifdef APP_RTOS_LINE
        bool start_line = rtos_line_prepare(&feedback, path, dt, active);
        if (!rtos_line_ready() && active) {
            BringupStatus current;
            bringup_get_status(&current);
            if (current.flags & BR_ACTIVE)
                bringup_stop(BR_REASON_ADC, BR_ESTOP);
            active = false;
        }
#ifdef APP_RTOS_NETWORK
        rn_drive_process(&feedback);
        if (!rn_drive_online_record(active || rtos_line_pending())) {
            if (active || rtos_line_pending())
                bringup_stop_from(BR_REASON_NETWORK, 0, MOTION_BUDDY1);
            start_line = false;
        }
        bringup_get_status(&snapshot);
        active = !!(snapshot.flags & BR_ACTIVE);
        bool paused = rn_drive_paused();
#else
        bool paused = false;
#endif
        if (start_line && rtos_line_ready()) {
            start_pulse(3);
            bringup_get_status(&snapshot);
            active = (snapshot.flags & BR_ACTIVE) != 0U;
            if (active) {
                rtos_line_begin();
#ifdef APP_RTOS_NETWORK
                rn_drive_begin();
#endif
                for (unsigned w = 0; w < 2; ++w) {
                    br_encoder_reset(&encoder_quality[w], (uint32_t)now, edges[w], invalid[w]);
                    bad_direction_us[w] = 0;
                }
                key = status_enter();
                status.encoder_kind = status.encoder_wheel = 0;
                status_leave(key);
            }
        }
#ifdef APP_RTOS_IMU
        MotionStatus imu_status = motion_get_status();
        bool imu_running =
            active && !paused && imu_status.session_active && imu_status.mode == MOTION_AUTO;
        ri_observe(&feedback, invalid, dt_us, imu_running);
        if (!imu_running)
            ri_ramp_halt(now);
#endif
        AnalogDemand optical = {0};
        if (active && paused) {
            rtos_line_feedback(&feedback, NULL);
        } else if (active) {
            optical = rtos_line_follow(dt);
            rtos_line_feedback(&feedback, &optical);
            if (optical.stop) {
                bringup_stop(BR_REASON_API, 0);
                active = false;
            } else if (motion_submit_navigation(MOTION_BUDDY3, optical.target[0],
                                                optical.target[1]) != MOTION_OK) {
                /* A simultaneous normal STOP already invalidated the API
                 * session; do not turn that successful stop into a new fault. */
                BringupStatus current;
                bringup_get_status(&current);
                if (current.flags & BR_ACTIVE)
                    bringup_stop(BR_REASON_API, BR_ESTOP);
                active = false;
            }
        }
        MotionOutput demand = motion_update(&feedback);
        if (active && demand.external && !paused) {
            BringupStatus current;
            bringup_get_status(&current);
            if (current.flags & BR_ACTIVE)
                bringup_stop(BR_REASON_API, BR_ESTOP);
            active = false;
        }
#ifdef APP_RTOS_NETWORK
        if (active && paused && motion_get_status().mode == MOTION_FAULT) {
            bringup_stop(BR_REASON_API, BR_ESTOP);
            active = false;
        }
#endif
        LineEnvelope envelope = {0};
        if (active) {
            /* Age the quality window during Pause too. Resume is not a new
             * session and cannot turn sparse lifetime errors into a fault. */
            for (unsigned w = 0; w < 2; ++w) {
                unsigned kind =
                    br_encoder_quality(&encoder_quality[w], (uint32_t)now, edges[w], invalid[w]);
                bad_direction_us[w] = !paused && optical.target[w] != 0 &&
                                              speed[w] * (optical.target[w] > 0 ? 1 : -1) < -20
                                          ? bad_direction_us[w] + dt_us
                                          : 0;
                if (bad_direction_us[w] >= 50000)
                    kind = BR_ENC_DIRECTION;
                bool trip = !paused && kind != BR_ENC_OK;
                BringupEncoderFaultSnapshot candidate = {0};
                if (trip) {
                    candidate.valid = 1U;
                    candidate.kind = (uint8_t)kind;
                    candidate.wheel = (uint8_t)(w + 1U);
                    candidate.control_timer_us = (uint32_t)now;
                    candidate.kernel_ms = board_kernel_ms();
                    candidate.direction_duration_us =
                        kind == BR_ENC_DIRECTION ? bad_direction_us[w] : 0U;
                    candidate.recent_window_bad = encoder_quality[w].window_bad;
                    candidate.recent_window_changes = encoder_quality[w].window_changes;
                    for (unsigned i = 0U; i < 2U; ++i) {
                        candidate.lifetime_edges[i] = edges[i];
                        candidate.lifetime_invalid[i] = invalid[i];
                        candidate.optical_target_mm_s[i] = optical.target[i];
                        candidate.measured_speed_x10[i] =
                            (int16_t)continuous_clampf(speed[i] * 10.0f, -32767, 32767);
                    }
                }
                key = status_enter();
                status.encoder_bad[w] = encoder_quality[w].window_bad;
                status.encoder_changes[w] = encoder_quality[w].window_changes;
                if (!paused && kind && !status.encoder_kind) {
                    status.encoder_kind = kind;
                    status.encoder_wheel = w + 1;
                }
                if (trip) {
                    candidate.previous_pwm[0] = status.pwm[0];
                    candidate.previous_pwm[1] = status.pwm[1];
                    (void)br_encoder_fault_capture_once(&encoder_fault_snapshot, &candidate);
                }
                status_leave(key);
                if (!paused && kind) {
                    bringup_stop(BR_REASON_ENCODER, BR_ENCODER);
                    tm_printf(
                        (UB *)"ENCODER_TRIP wheel=%u trip_kind=%u window_bad=%u window_changes=%u\n",
                        w + 1, kind, encoder_quality[w].window_bad,
                        encoder_quality[w].window_changes);
                    active = false;
                    break;
                }
            }
        }
        if (active && !paused) {
            uintptr_t api_key = board_motion_lock(NULL);
            LineEnvelope previous_envelope = committed_envelope;
            board_motion_unlock(NULL, api_key);
            for (unsigned w = 0; w < 2; ++w)
                rtos_line_actual_output(w, rl_cap(&previous_envelope, w, board_time_us()));
            envelope = rtos_line_power(&demand, &optical, dt);
            bool committed = motion_commit(demand.generation, apply_outputs, &envelope);
            for (unsigned w = 0; w < 2; ++w)
                rtos_line_actual_output(w, committed ? rl_cap(&envelope, w, board_time_us()) : 0);
        }
#else
        MotionOutput demand = motion_update(&feedback);
        int32_t pwm[2] = {0, 0};
        for (unsigned w = 0U; w < 2U; ++w) {
            if (!active || demand.reset_controllers)
                continuous_drive_reset(&drive[w], &wheel[w]);
            if (active)
                pwm[w] = continuous_drive_step(&drive[w], &wheel[w], w, demand.target_mm_s[w],
                                               speed[w], dt, now, snapshot.pwm[w]);
            if (!active || demand.target_mm_s[w] == 0 || delta[w] != 0 || pwm[w] < 120) {
                last_progress[w] = (uint32_t)now;
            } else if ((uint32_t)now - last_progress[w] >= BR_STALL_US) {
                bringup_stop(BR_REASON_STALL, BR_STALL);
            }
        }
        if (active)
            (void)motion_commit(demand.generation, apply_outputs, pwm);
#endif

        key = status_enter();
        status.kernel_ms = board_kernel_ms();
        status.timer_us = (uint32_t)now;
        status.control_count++;
        status.dt_us = dt_us;
        if (dt_us < status.min_dt_us)
            status.min_dt_us = dt_us;
        if (dt_us > status.max_dt_us)
            status.max_dt_us = dt_us;
        if (dt_us > 15000U)
            status.late_count++;
        for (unsigned w = 0U; w < 2U; ++w) {
            /* Raw diagnostic count saturates; odometry remains 64-bit. */
            status.ticks[w] = totals[w] > INT32_MAX   ? INT32_MAX
                              : totals[w] < INT32_MIN ? INT32_MIN
                                                      : (int32_t)totals[w];
            status.edges[w] = edges[w];
            status.invalid[w] = invalid[w];
            status.speed_x10[w] = (int16_t)(continuous_clampf(speed[w] * 10.0f, -32767, 32767));
            status.target[w] = (status.flags & BR_ACTIVE) ? demand.target_mm_s[w] : 0;
        }
        last_control_us = (uint32_t)now;
        uint32_t cost = (uint32_t)(board_time_us() - now);
        if (cost > status.max_cost_us)
            status.max_cost_us = cost;
        status_leave(key);
#ifdef APP_RTOS_LINE
        bringup_get_status(&snapshot);
        rtos_line_record(&snapshot, &optical, cost);
#endif
    }
}

void bringup_safety_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    for (;;) {
        BringupStatus snapshot;
        UINT key = status_enter();
        snapshot = status;
        uint32_t last = last_control_us, end = deadline_us;
        uint32_t urgent = urgent_stop;
        urgent_stop = 0U;
        status_leave(key);
        uint32_t now = (uint32_t)board_time_us();
        bool active = (snapshot.flags & BR_ACTIVE) != 0U;
#ifdef APP_RTOS_NETWORK
        unsigned net_urgent = rn_take_urgent();
        if (net_urgent)
            bringup_stop_from(BR_REASON_NETWORK, net_urgent & RN_URGENT_ESTOP ? BR_ESTOP : 0,
                              MOTION_BUDDY1);
#ifdef APP_RTOS_LINE
        if (active && !rn_drive_online())
            bringup_stop_from(BR_REASON_NETWORK, 0, MOTION_BUDDY1);
#endif
#endif
        if ((snapshot.flags & BR_READY) != 0U) {
            if ((bringup_irq_stop != 0U || board_button_pressed(20U) || urgent == 'e') &&
                (active || (snapshot.flags & BR_ESTOP) == 0U))
                bringup_stop(urgent == 'e' ? BR_REASON_SERIAL : BR_REASON_GP20, BR_ESTOP);
            else if (urgent == 's')
                bringup_stop(BR_REASON_SERIAL, 0U);
            else if (active && board_button_pressed(21U))
                bringup_stop(BR_REASON_GP21, 0U);
            else if (active && !br_usb_supervised(tm_usb_state()))
                bringup_stop(BR_REASON_USB, 0U);
            else if (active && now - last >= BR_LATE_US)
                bringup_stop(BR_REASON_LATE, BR_LATE);
            else if (active && end && (int32_t)(now - end) >= 0)
                bringup_stop(BR_REASON_DONE, 0U);
        }
#ifdef APP_RTOS_LINE
        /* Match the SDK's 1ms peak-expiry envelope. This does not calculate
         * PWM or edit PI state; it can only reduce the committed output. */
        uintptr_t api_key = board_motion_lock(NULL);
        key = status_enter();
        if ((status.flags & BR_ACTIVE) && !bringup_irq_stop) {
            bool allowed =
                br_outputs_allowed(status.flags, bringup_irq_stop != 0U, tm_usb_state(),
                                   !board_button_pressed(20U) && !board_button_pressed(21U));
            for (unsigned w = 0; w < 2; ++w) {
                DualLevels cap =
                    allowed ? rl_levels(&committed_envelope, w, board_time_us()) : (DualLevels){0};
                DualLevels actual = board_motor_current(w);
                DualLevels reduced = rl_reduce(actual, cap);
                if (reduced.a != actual.a || reduced.b != actual.b)
                    board_motor_levels(w, reduced.a, reduced.b);
                int sign = (w == 0 ? reduced.a : reduced.b) ? 1 : -1;
                status.pwm[w] = (int16_t)(sign * (int)(reduced.a + reduced.b));
            }
        }
        status_leave(key);
        board_motion_unlock(NULL, api_key);
#endif
        (void)tk_dly_tsk(1);
    }
}

static void start_pulse(unsigned selected) {
#ifdef APP_RTOS_NET_BENCH
    (void)selected;
    tm_printf((UB *)"NETWORK BENCH: motor starts disabled; use Step2 firmware for driving.\n");
    return;
#endif
    BringupStatus snapshot;
    UINT key = status_enter();
    snapshot = status;
    uint32_t stopped = stopped_us;
    status_leave(key);
    uint64_t now = board_time_us();
#ifdef APP_RTOS_RELEASE
    if (!rn_drive_online())
        return;
#endif
    if (!br_can_start(snapshot.flags, (uint32_t)now, stopped, tm_usb_state(),
                      !board_button_pressed(20U) && !board_button_pressed(21U)) ||
        bringup_irq_stop) {
        tm_printf((
            UB *)"REQUEST rejected: wait for CLOCK PASS, release buttons, clear a latch with x, and allow 1s rest.\n");
        return;
    }
    motion_session_begin(now);
    int16_t left = selected == 2U ? 0 : BR_SPEED_MM_S;
    int16_t right = selected == 1U ? 0 : BR_SPEED_MM_S;
    MotionResult result = motion_submit_navigation(MOTION_BUDDY3, left, right);
    if (result != MOTION_OK) {
        bringup_stop(BR_REASON_API, 0U);
        return;
    }
    key = status_enter();
    bool accepted =
        bringup_irq_stop == 0U && urgent_stop == 0U && (status.flags & BR_LATCHED) == 0U;
#ifdef APP_RTOS_LINE
    accepted = accepted && rtos_line_start_valid();
#endif
    if (accepted) {
        deadline_us = (uint32_t)now + BR_PULSE_US;
#ifdef APP_RTOS_LINE
        deadline_us = 0; /* Line session ends on a stop/fault, not a 3s bench pulse. */
#endif
        bench_target[0] = left;
        bench_target[1] = right;
        status.flags |= BR_ACTIVE;
        br_encoder_fault_clear(&encoder_fault_snapshot);
    }
    status_leave(key);
    if (!accepted) {
        motion_session_end(BR_REASON_SERIAL);
        return;
    }
#ifdef APP_RTOS_LINE
    tm_printf(
        (UB *)"LINE accepted: Buddy3 -> NAV -> MOTION; profile1, configured link supervision active.\n");
#else
    tm_printf(
        (UB *)"REQUEST USB_BENCH -> Buddy3 target stub -> NAV -> MOTION: L=%d R=%d mm/s; automatic stop after 3s.\n",
        left, right);
#endif
}

void bringup_nav_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    for (;;) {
        uint8_t msg[5];
        INT count = tk_rcv_mbf(bringup_commands, msg, 10);
        /* Keep the existing coordinator's 100ms stale-target watchdog.
         * The bench target provider, like Buddy 3 later, must keep reporting. */
        UINT key = status_enter();
        bool active = (status.flags & BR_ACTIVE) != 0U;
        int16_t left = bench_target[0], right = bench_target[1];
        status_leave(key);
#ifdef APP_RTOS_LINE
        (void)active;
        (void)left;
        (void)right;
#else
        if (active && motion_submit_navigation(MOTION_BUDDY3, left, right) != MOTION_OK) {
            BringupStatus after;
            bringup_get_status(&after);
            if ((after.flags & BR_ACTIVE) != 0U)
                bringup_stop(BR_REASON_API, BR_ESTOP);
        }
#endif
        if (count != 5)
            continue;
        uint32_t received = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8U) |
                            ((uint32_t)msg[3] << 16U) | ((uint32_t)msg[4] << 24U);
        unsigned command = msg[0];
        if (!br_command_fresh((uint32_t)board_time_us(), received))
            continue;
        if (command >= '1' && command <= '3')
#ifdef APP_RTOS_LINE
            tm_printf((
                UB *)"Step2 uses profile1 only; g starts following. Use Step1 firmware for bench pulses.\n");
#else
            start_pulse(command - '0');
#endif
        else if (command == 's' || command == 'e')
            bringup_stop(BR_REASON_SERIAL, command == 'e' ? BR_ESTOP : 0U);
        else if (command == 'x') {
            UINT key = status_enter();
            if ((status.flags & BR_ACTIVE) == 0U && !board_button_pressed(20U) &&
                !board_button_pressed(21U) && (status.flags & BR_CLOCK_OK) != 0U) {
                bringup_irq_stop = 0U;
                status.flags &= (uint16_t)~BR_LATCHED;
                stopped_us = (uint32_t)board_time_us();
                br_encoder_fault_clear(&encoder_fault_snapshot);
            }
            status_leave(key);
            bringup_report();
        }
#ifdef APP_RTOS_LINE
        else if (command == 'g' || command == 'w' || command == 'l' || command == 'r' ||
                 command == 'c' || command == 'k' || command == 'd'
#ifdef APP_RTOS_BARCODE
                 || command == 'b'
#endif
        ) {
            if (!rtos_line_command(command, received))
                tm_printf((UB *)"LINE command queue full; command rejected.\n");
        } else if (command == 'L' || command == 'R' || command == 'U') {
#ifdef APP_RTOS_BARCODE
            tm_printf((UB *)"Step4: use an actual mapped barcode; canonical USB L/R/U disabled.\n");
#else
            char route = command == 'L' ? 'A' : command == 'R' ? 'F' : 'Z';
            MotionResult result = motion_submit_barcode(MOTION_BUDDY3, route);
            tm_printf((UB *)"USB route %c -> Buddy3 -> NAV result=%u; await separate junction.\n",
                      command, result);
#endif
        }
#endif
        else if (command == 'h' || command == '?')
            bringup_help();
        else if (command == 'p' || command == 'n') {
            bringup_report();
            board_clock_report();
        } else if (command == 'z') {
            MotionEvent event;
            for (unsigned n = 0U; n < 12U && motion_pop_event(&event); ++n)
                tm_printf(
                    (UB *)"NAV time_us=%u owner=%u requester=%u event=%u state=%u fault=%u result=%u target=%d,%d\n",
                    (uint32_t)event.now_us, event.owner, event.requester, event.type, event.state,
                    event.fault, event.result, event.target_mm_s[0], event.target_mm_s[1]);
        }
    }
}

void bringup_console_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    for (;;) {
        for (unsigned n = 0U; n < 32U; ++n) {
            INT value = tm_usb_console_get();
            if (value < 0)
                break;
            if (value == '\r' || value == '\n' || value == ' ')
                continue;
            uint32_t now = (uint32_t)board_time_us();
            uint8_t msg[5] = {(uint8_t)value, (uint8_t)now, (uint8_t)(now >> 8U),
                              (uint8_t)(now >> 16U), (uint8_t)(now >> 24U)};
            /* Urgent stop survives a full command queue. It is consumed by
             * the Safety Supervisor, which calls the same motion stop API. */
            if (value == 's' || value == 'e') {
                UINT key = status_enter();
                if (urgent_stop != 'e')
                    urgent_stop = (uint32_t)value;
                status_leave(key);
            }
            (void)tk_snd_mbf(bringup_commands, msg, 5, TMO_POL);
        }
        (void)tk_dly_tsk(1);
    }
}
