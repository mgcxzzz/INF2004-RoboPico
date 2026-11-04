#include "line_service.h"
#ifdef APP_RTOS_BARCODE
#include "../barcode_navigation/barcode_model.h"
static RbDecoder barcode;
static uint32_t barcode_peak_sent, barcode_status_ms;
static bool barcode_dump;
static void barcode_publish(uint64_t now);
static void barcode_export(void);
#endif
#ifdef APP_RTOS_NETWORK
#include "../network_bridge/network_bridge.h"
static WifiMotionPacket telemetry;
static uint16_t run_id;
static uint32_t run_elapsed_ms;
static double run_path;
#endif

static ID commands;
static T_SPLOCK copy_lock;
static LinePlanner planner;
static LinePower power;
static LineCalibration calibration;
static CircuitInput input;
static float filtered[2];
static uint16_t adc[3];
static uint32_t cal_origin; /* 0 preset, 1 SDK record28, 2 own saved page, 3 RAM capture. */
static bool adc_ok, filter_ready, was_active, start_offered;
static RlAdcHealth adc_health;
static bool adc_recovering;
static uint32_t adc_retries, adc_recoveries, adc_frames;
static uint64_t side_due, countdown, run_start;
static LineCalibration run_calibration;
static unsigned run_cal_origin;
static unsigned previous_phase;
static unsigned last_noted_phase;
static LineLogRow rows[RL_LOG_ROWS];
static unsigned row_count, row_head, row_overwritten, row_sequence;
static bool dump_requested, dumping;
static bool recording;
static volatile uint32_t cancel_serial;
static uint32_t last_cancel_us, start_serial;
static struct {
    LineCalibration cal;
    uint16_t adc[3], level[2];
    uint32_t cal_origin, rows, overwritten, sequence, cost;
    unsigned phase, decision, reason, pending, action, flags;
    uint32_t junction_aux;
    RlAdcDiagnostics adc_diag;
    uint32_t adc_retries, adc_recoveries, adc_frames;
    unsigned adc_state : 2, adc_good : 2;
#ifdef APP_RTOS_BARCODE
    uint32_t decoded, peak_decodes, peak_applied, conflicts, gaps, max_gap, samples, max_sample;
    uint16_t barcode_min, barcode_max, barcode_adc, threshold;
    unsigned cross_threshold : 12, cross_hysteresis : 12;
    unsigned cross_valid : 1, cross_armed : 1, cross_black : 1;
    uint8_t bank, last_code, pending_action, pending_revision, executing_action, executing_revision;
    unsigned scanning : 1, raw_frozen : 1;
#endif
} view;

static UINT enter(void) {
    UINT key;
    (void)ISpinLock(&copy_lock, &key);
    return key;
}
static void leave(UINT key) {
    (void)ISpinUnlock(&copy_lock, key);
}

static void acquisition_failed(uint64_t now) {
    adc_ok = false;
    filter_ready = false;
    adc_recovering = true;
    countdown = 0;
    rl_adc_failed(&adc_health, now);
    /* Cancellation is out of queue and invalidates any armed Start. */
    BringupStatus status;
    bringup_get_status(&status);
    if (status.flags & BR_ACTIVE)
        bringup_stop(BR_REASON_ADC, BR_ESTOP);
    else
        rtos_line_cancel();
}

/* A failed front pair is never half-published. During qualification, all
 * three channels must be freshly read in three consecutive control frames. */
static bool acquire_frame(bool include_side) {
    uint16_t next[3] = {adc[0], adc[1], adc[2]};
    if (!board_line_adc_read(0, 32, true, &next[0]) ||
        !board_line_adc_read(2, 32, true, &next[2]) ||
        (include_side && !board_line_adc_read(1, 4, false, &next[1]))) {
        acquisition_failed(board_time_us());
        return false;
    }
    memcpy(adc, next, sizeof(adc));
    ++adc_frames;
    return true;
}

ER rtos_line_init(void) {
    T_CMBF definition = {0};
    definition.mbfatr = TA_TFIFO;
    definition.bufsz = 256;
    definition.maxmsz = 5;
    commands = tk_cre_mbf(&definition);
    if (commands < E_OK)
        return commands;
    const BarcodeSaved *saved = (const BarcodeSaved *)(uintptr_t)(0x10000000U + RL_MAIN_OFFSET);
    calibration =
        rl_boot_cal((const uint8_t *)(uintptr_t)(0x10000000U + RL_CAL_OFFSET), saved, &cal_origin);
    if (board_line_adc_init())
        adc_health = (RlAdcHealth){.state = RL_ADC_QUALIFYING};
    else
        acquisition_failed(board_time_us());
    /* A peripheral fault gates line readiness, not motor/encoder init. It
     * can recover while stopped; object-creation failure above is still fatal. */
    return E_OK;
}

bool rtos_line_command(unsigned command, uint32_t received_us) {
    uint8_t msg[5] = {(uint8_t)command, (uint8_t)received_us, (uint8_t)(received_us >> 8),
                      (uint8_t)(received_us >> 16), (uint8_t)(received_us >> 24)};
    return tk_snd_mbf(commands, msg, 5, TMO_POL) == E_OK;
}

void rtos_line_cancel(void) {
    UINT key = enter();
    last_cancel_us = (uint32_t)board_time_us();
    ++cancel_serial;
    leave(key);
}

bool rtos_line_start_valid(void) {
    UINT key = enter();
    bool valid = start_serial == cancel_serial;
    leave(key);
    return valid;
}

void rtos_line_side_poll(uint64_t now) {
    if (!adc_ok || now < side_due)
        return;
    side_due = now + 2000;
    if (!board_line_adc_read(1, 4, false, &adc[1])) {
        acquisition_failed(board_time_us());
        return;
    }
    if (was_active
#ifdef APP_RTOS_NETWORK
        && !rn_drive_paused()
#endif
    ) {
        BringupStatus safety;
        bringup_get_status(&safety);
        if (!(safety.flags & BR_ACTIVE))
            return;
        /* Between 10 ms encoder drains, interpolate at the last measured
         * velocity, as a bounded-distance edge estimate, never a PWM change. */
        double wheel[2];
        double elapsed = now >= input.now ? (double)(now - input.now) / 1000000 : 0;
        for (unsigned w = 0; w < 2; ++w)
            wheel[w] = input.wheel[w] + input.speed[w] * elapsed;
#ifdef APP_RTOS_BARCODE
        uint64_t started = board_time_us();
        planner.nav.mapping = rn_mapping_snapshot();
        rb_sample(&barcode, &planner.nav, adc[1], now, wheel);
        planner.black = barcode.analog.black;
        planner.side_adc = adc[1];
        uint32_t cost = (uint32_t)(board_time_us() - started);
        if (cost > barcode.max_sample_us)
            barcode.max_sample_us = cost;
#else
        rl_side(&planner, adc[1], now, wheel);
#endif
    }
}

bool rtos_line_ready(void) {
    return adc_ok && calibration_valid(&calibration);
}

bool rtos_line_pending(void) {
    UINT key = enter();
    bool pending = (view.flags & 4U) && start_serial == cancel_serial;
    leave(key);
    return pending;
}
bool rtos_line_request_fresh(uint32_t received_us) {
    UINT key = enter();
    bool valid = br_command_after_stop((uint32_t)board_time_us(), received_us, last_cancel_us,
                                       cancel_serial != 0);
    leave(key);
    return valid;
}
bool rtos_line_request_start(uint32_t received_us) {
    UINT key = enter();
    bool allowed = !dumping && !dump_requested && !countdown && !start_offered &&
                   br_command_after_stop((uint32_t)board_time_us(), received_us, last_cancel_us,
                                         cancel_serial != 0);
    uint32_t serial = cancel_serial;
    leave(key);
    if (!allowed || !rtos_line_ready())
        return false;
#ifdef APP_RTOS_NETWORK
    if (!rn_drive_online() || !bringup_can_start())
        return false;
#endif
    key = enter();
    if (serial != cancel_serial) {
        leave(key);
        return false;
    }
    start_serial = serial;
    countdown = input.now + 2000000;
    view.flags |= 4U;
    leave(key);
    return true;
}
#ifdef APP_RTOS_NETWORK
void rtos_line_resume(void) {
#ifdef APP_RTOS_BARCODE
    rb_resume(&planner, &barcode, &input, adc[1]);
#else
    char pending = planner.nav.pending;
    rl_begin(&planner, &input, adc[1]);
    if (pending)
        (void)barcode_set_pending(&planner.nav, pending);
#endif
    power = (LinePower){0};
    last_noted_phase = UINT32_MAX;
}
void rtos_line_telemetry(WifiMotionPacket *out) {
    UINT key = enter();
    *out = telemetry;
    leave(key);
}
#endif
static void calibration_command(unsigned command) {
    if (command != 'c' && !adc_ok) {
        tm_printf((UB *)"CAL rejected: ADC not ready; existing calibration retained.\n");
        return;
    }
    if (command == 'c') {
        calibration = successful_calibration();
        cal_origin = 0;
    } else if (command == 'w') {
        calibration.white[0] = adc[0];
        calibration.white[1] = adc[2];
        calibration.mask |= 1;
        cal_origin = 3;
    } else if (command == 'l' || command == 'r') {
        unsigned w = command == 'l' ? 0 : 1;
        calibration.black[w] = adc[w == 0 ? 0 : 2];
        calibration.mask |= 2U << w;
        cal_origin = 3;
    } else if (command == 'k') {
        if (!rtos_line_ready()) {
            tm_printf((UB *)"CAL_SAVE rejected: calibration or ADC invalid.\n");
            return;
        }
        uint8_t page[256];
        rl_cal_encode(page, &calibration);
        bool ok = board_line_save(page);
        if (ok)
            cal_origin = 2;
        tm_printf((UB *)"CAL_SAVE result=%s own_sector=0x%x; SDK/Wi-Fi records untouched.\n",
                  ok ? "VERIFIED" : "FAILED", RL_CAL_OFFSET);
    }
    tm_printf((UB *)"CAL_FRONT white=%u,%u black=%u,%u mask=%u valid=%u origin=%u\n",
              calibration.white[0], calibration.white[1], calibration.black[0],
              calibration.black[1], calibration.mask, calibration_valid(&calibration), cal_origin);
}

bool rtos_line_prepare(const MotionFeedback *f, double path, float dt, bool active) {
    (void)dt;
    start_offered = false;
    if (adc_ok) {
        (void)acquire_frame(false);
    } else {
        countdown = 0;
        if (rl_adc_can_retry(&adc_health, f->now_us, active)) {
            ++adc_retries;
            if (board_line_adc_init())
                adc_health = (RlAdcHealth){.state = RL_ADC_QUALIFYING};
            else
                acquisition_failed(board_time_us());
        }
        if (!active && adc_health.state == RL_ADC_QUALIFYING && acquire_frame(true)) {
            adc_ok = rl_adc_good_frame(&adc_health);
            if (adc_ok) {
                if (adc_recovering)
                    ++adc_recoveries;
                adc_recovering = false;
                filter_ready = false;
                side_due = board_time_us() + 2000;
            }
        }
    }
    input = (CircuitInput){.now = f->now_us,
                           .wheel = {f->wheel_mm[0], f->wheel_mm[1]},
                           .speed = {f->speed_mm_s[0], f->speed_mm_s[1]},
                           .side_black = planner.black};
    for (unsigned w = 0; w < 2; ++w) {
        uint16_t raw = adc[w == 0 ? 0 : 2];
        if (!filter_ready)
            filtered[w] = raw;
        else
            filtered[w] += .35f * ((float)raw - filtered[w]);
        input.adc[w] = (uint16_t)filtered[w]; /* Same truncation as SDK main. */
    }
    filter_ready = true;
    input.path = path; /* Average absolute accumulated wheel travel, including U-turns. */
    if (!active && was_active)
        countdown = 0;
    was_active = active;
    if (countdown && !rtos_line_start_valid())
        countdown = 0;
#ifdef APP_RTOS_NETWORK
    if (!rn_drive_online())
        countdown = 0;
#endif
    uint8_t msg[5];
    for (unsigned n = 0; n < 8 && tk_rcv_mbf(commands, msg, TMO_POL) == 5; ++n) {
        uint32_t received = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) | ((uint32_t)msg[3] << 16) |
                            ((uint32_t)msg[4] << 24);
        UINT key = enter();
        uint32_t serial = cancel_serial, stopped = last_cancel_us;
        leave(key);
        if (!br_command_after_stop((uint32_t)board_time_us(), received, stopped, serial != 0))
            continue;
        unsigned command = msg[0];
        if (command == 'd'
#ifdef APP_RTOS_BARCODE
            || command == 'b'
#endif
        ) {
            countdown = 0;
            UINT key = enter();
            if (!active) {
                dump_requested = true;
#ifdef APP_RTOS_BARCODE
                barcode_dump = command == 'b';
#endif
            }
            leave(key);
            if (active)
                tm_printf((UB *)"LOG rejected: stop before d.\n");
        } else if (active) {
            tm_printf((UB *)"LINE command '%c' rejected while moving.\n", command);
        } else if (command == 'g') {
            UINT key = enter();
            bool exporting = dumping || dump_requested;
            leave(key);
            if (!exporting && rtos_line_request_start(received)) {
                tm_printf((UB *)"LINE start requested; 2s countdown; buttons/s cancel.\n");
            } else if (exporting)
                tm_printf((UB *)"LINE start rejected: log export still pending/active.\n");
            else if (!adc_ok)
                tm_printf(
                    (UB *)"LINE start rejected: ADC not ready; see ADC/ADC_FIRST/ADC_LAST.\n");
            else
                tm_printf((UB *)"LINE start rejected: calibration invalid.\n");
        } else {
            countdown = 0;
            calibration_command(command);
        }
    }
    BringupStatus s;
    bringup_get_status(&s);
    if (board_button_pressed(20) || board_button_pressed(21) || bringup_irq_stop ||
        (s.flags & BR_LATCHED) || tm_usb_state() != 3 || !rtos_line_start_valid())
        countdown = 0;
    bool start = countdown && f->now_us >= countdown && !active;
    if (start)
        countdown = 0;
    start_offered = start;
    return start;
}

void rtos_line_begin(void) {
    start_offered = false;
    rl_begin(&planner, &input, adc[1]);
#ifdef APP_RTOS_BARCODE
    planner.nav.mapping = rn_mapping_snapshot();
    rb_begin(&barcode, &planner.nav, adc[1], input.now, input.wheel);
    barcode_peak_sent = 0;
    barcode_status_ms = 0;
#endif
    power = (LinePower){0};
    run_start = input.now;
#ifdef APP_RTOS_NETWORK
    ++run_id;
    run_elapsed_ms = 0;
    run_path = input.path;
#endif
    last_noted_phase = UINT32_MAX;
    run_calibration = calibration;
    run_cal_origin = cal_origin;
    UINT key = enter();
    row_count = row_head = row_overwritten = row_sequence = 0;
    recording = true;
    leave(key);
    was_active = true;
}

AnalogDemand rtos_line_follow(float dt) {
    previous_phase = planner.circuit.phase;
    input.side_black = planner.black;
#ifdef APP_RTOS_BARCODE
    planner.nav.mapping = rn_mapping_snapshot();
    return rb_follow(&planner, &barcode, &calibration, &input, dt);
#else
    return rl_follow(&planner, &calibration, &input, dt);
#endif
}

void rtos_line_feedback(MotionFeedback *f, const AnalogDemand *d) {
    f->line_level[0] =
        d ? d->level[0] : line_level(input.adc[0], calibration.white[0], calibration.black[0]);
    f->line_level[1] =
        d ? d->level[1] : line_level(input.adc[1], calibration.white[1], calibration.black[1]);
    f->navigation_busy = planner.circuit.busy;
    f->pending_route = planner.nav.pending;
    if (planner.circuit.phase != last_noted_phase) {
        motion_note_navigation(planner.circuit.phase, 0);
        last_noted_phase = planner.circuit.phase;
    }
}

LineEnvelope rtos_line_power(const MotionOutput *output, AnalogDemand *d, float dt) {
    if (output->reset_controllers)
        power = (LinePower){0};
    if (output->route)
        (void)barcode_set_pending(&planner.nav, output->route);
    d->target[0] = output->target_mm_s[0];
    d->target[1] = output->target_mm_s[1];
    return rl_power(&power, &planner, &input, d, previous_phase, dt);
}

void rtos_line_actual_output(unsigned w, int32_t output) {
    power.output[w] = output;
}

void rtos_line_record(const BringupStatus *s, const AnalogDemand *d, uint32_t cost) {
    LineLogRow row = {.ms = (uint32_t)((input.now - run_start) / 1000),
                      .ticks = {s->ticks[0], s->ticks[1]},
                      .speed = {s->speed_x10[0], s->speed_x10[1]},
                      .target = {s->target[0], s->target[1]},
                      .pwm = {s->pwm[0], s->pwm[1]},
                      .dt_us = s->dt_us > 65535 ? 65535 : (uint16_t)s->dt_us,
                      .cost_us = cost > 65535 ? 65535 : (uint16_t)cost};
    uint16_t raw[3] = {adc[0], adc[2], adc[1]};
    rl_pack_optics(&row, raw, d, planner.circuit.phase);
    rl_pack_steering(&row, input.adc, (unsigned)(planner.steering.gain * 1000 + .5f),
                     (unsigned)(planner.steering.damp_weight * 1000 + .5f));
    rl_pack_junction(&row, &planner);
    row.optical_target[0] = planner.optical.target[0];
    row.optical_target[1] = planner.optical.target[1];
    const CircuitTracker *tracker =
        planner.circuit.phase == J_ACQUIRE ? &planner.circuit.joining : &planner.circuit.follower;
    row.meta = (d->decision & 63U) | ((planner.circuit.reason & 63U) << 6) |
               ((uint32_t)planner.black << 12) | ((uint32_t)planner.circuit.busy << 13) |
               ((uint32_t)tracker->core.white << 14) | ((uint32_t)tracker->search.active << 15) |
#ifdef APP_RTOS_BARCODE
               (rl_route_bits(rb_log_route(&planner.nav, planner.nav.pending)) << 16) |
               (rl_route_bits(rb_log_route(&planner.nav, planner.nav.action)) << 18) |
#else
               (rl_route_bits(planner.nav.pending) << 16) |
               (rl_route_bits(planner.nav.action) << 18) |
#endif
               ((s->reason & 15U) << 20) | ((s->flags & 127U) << 24);
    UINT key = enter();
    view.cal = calibration;
    view.cal_origin = cal_origin;
    memcpy(view.adc, raw, sizeof(raw));
    for (unsigned w = 0; w < 2; ++w)
        view.level[w] = line_level(input.adc[w], calibration.white[w], calibration.black[w]);
    view.phase = planner.circuit.phase;
    view.decision = d->decision;
    view.reason = planner.circuit.reason;
    view.pending = (uint8_t)planner.nav.pending;
    view.action = (uint8_t)planner.nav.action;
    view.cost = cost;
    view.flags =
        (adc_ok ? 1U : 0U) | (calibration_valid(&calibration) ? 2U : 0U) | (countdown ? 4U : 0U);
    view.junction_aux = row.aux[1];
    board_line_adc_diagnostics(&view.adc_diag);
    view.adc_state = adc_health.state;
    view.adc_good = adc_health.good;
    view.adc_retries = adc_retries;
    view.adc_recoveries = adc_recoveries;
    view.adc_frames = adc_frames;
#ifdef APP_RTOS_BARCODE
    view.decoded = planner.nav.decoded_total;
    view.peak_decodes = barcode.peak_decodes;
    view.peak_applied = barcode.peak_applied;
    view.conflicts = barcode.peak_conflicts;
    view.gaps = barcode.analog.gap_count;
    view.max_gap = barcode.analog.max_gap_us;
    view.samples = barcode.samples;
    view.max_sample = barcode.max_sample_us;
    view.barcode_min = barcode.analog.minimum;
    view.barcode_max = barcode.analog.maximum;
    view.barcode_adc = adc[1];
    view.threshold = barcode_analog_threshold(barcode.analog.selected);
    view.cross_threshold = barcode.junction.threshold;
    view.cross_hysteresis = barcode.junction.hysteresis;
    view.cross_valid = barcode.junction.valid;
    view.cross_armed = barcode.junction.armed;
    view.cross_black = barcode.junction.black;
    view.bank = barcode.analog.selected;
    view.last_code = planner.nav.last_code;
    view.pending_action = planner.nav.pending_action;
    view.pending_revision = planner.nav.pending_revision;
    view.executing_action = planner.nav.executing_action;
    view.executing_revision = planner.nav.executing_revision;
    view.scanning = barcode.analog.scanning;
    view.raw_frozen = barcode.raw_frozen;
#endif
    if (recording) {
        row.sequence = ++row_sequence;
        rows[row_head] = row;
        row_head = (row_head + 1) % RL_LOG_ROWS;
        if (row_count < RL_LOG_ROWS)
            ++row_count;
        else
            ++row_overwritten;
        if (!(s->flags & BR_ACTIVE))
            recording = false; /* Preserve one terminal stopped/fault row. */
    }
#ifdef APP_RTOS_NETWORK
    if (recording)
        run_elapsed_ms = (uint32_t)((input.now - run_start) / 1000);
    telemetry = (WifiMotionPacket){
        .uptime_ms = s->kernel_ms,
        .elapsed_ms = run_elapsed_ms,
        .run_id = run_id,
        .phase = (uint8_t)planner.circuit.phase,
        .profile = RL_PROFILE,
        .flags = (s->flags & BR_ACTIVE ? 1U : 0U) | (calibration_valid(&calibration) ? 2U : 0U) |
                 (s->flags & BR_ACTIVE && !rn_drive_paused() ? 4U : 0U) |
                 (s->flags & BR_ESTOP ? 8U : 0U) | 0x40U
#ifdef APP_RTOS_BARCODE
                 | 0x20U /* Step4 identification within the existing RTOS marker. */
#endif
        ,
        .recovery_state = (uint8_t)(tracker->search.active ? 2
                                    : tracker->core.white  ? 1
                                                           : 0),
        .distance_mm =
            input.path - run_path < UINT32_MAX ? (uint32_t)(input.path - run_path) : UINT32_MAX,
        .adc_left = raw[0],
        .adc_right = raw[1],
        .adc_barcode = raw[2],
        .calibrated_level_left = calibration_valid(&calibration) ? view.level[0] : 0xffff,
        .calibrated_level_right = calibration_valid(&calibration) ? view.level[1] : 0xffff,
        .speed_left_0p1mm_s = s->speed_x10[0],
        .speed_right_0p1mm_s = s->speed_x10[1],
        .target_left_mm_s = s->target[0],
        .target_right_mm_s = s->target[1],
        .pwm_left = s->pwm[0],
        .pwm_right = s->pwm[1],
        .heading_cdeg =
            (int16_t)lroundf(remainderf(heading_deg(input.wheel[0], input.wheel[1]), 360) * 100)};
    telemetry.line_levels = calibration_valid(&calibration)
                                ? (view.level[0] >= 400 ? 1U : 0U) |
                                      (view.level[1] >= 400 ? 2U : 0U) | (planner.black ? 4U : 0U)
                                : 0;
#endif
    view.rows = row_count;
    view.overwritten = row_overwritten;
    view.sequence = row_sequence;
    leave(key);
#ifdef APP_RTOS_BARCODE
    barcode_publish(input.now);
#endif
}

void rtos_line_report(void) {
#ifdef APP_RTOS_BARCODE
    UINT bk = enter();
    uint32_t decoded = view.decoded, peak = view.peak_decodes, applied = view.peak_applied,
             conflicts = view.conflicts;
    uint32_t gaps = view.gaps, maxgap = view.max_gap, samples = view.samples,
             maxsample = view.max_sample;
    uint16_t minimum = view.barcode_min, maximum = view.barcode_max, threshold = view.threshold;
    unsigned bank = view.bank, last = view.last_code, pa = view.pending_action,
             rev = view.pending_revision;
    bool frozen = view.raw_frozen;
    unsigned cross_threshold = view.cross_threshold, cross_hysteresis = view.cross_hysteresis;
    unsigned cross_valid = view.cross_valid, cross_armed = view.cross_armed,
             cross_black = view.cross_black;
    leave(bk);
    tm_printf(
        (UB *)"BARCODE decoded=%u peak=%u applied=%u conflicts=%u last_ascii=%u pending_action=%u revision=%u min=%u max=%u threshold=%u bank=%u samples=%u max_gap_us=%u gaps_over10ms=%u max_sample_us=%u raw_frozen=%u\n",
        decoded, peak, applied, conflicts, last, pa, rev, minimum, maximum, threshold, bank,
        samples, maxgap, gaps, maxsample, frozen);
    tm_printf(
        (UB *)"JUNCTION_SENSOR offset_mm=%u threshold=%u hysteresis=%u valid=%u armed=%u black=%u\n",
        B_SIDE_AHEAD_MM, cross_threshold, cross_hysteresis, cross_valid, cross_armed, cross_black);
#endif
    UINT key = enter();
    /* No formatting while holding the shared packed-data lock. */
    LineCalibration cal = view.cal;
    unsigned origin = view.cal_origin, flags = view.flags;
    uint16_t raw[3], levels[2];
    memcpy(raw, view.adc, sizeof(raw));
    memcpy(levels, view.level, sizeof(levels));
    unsigned phase = view.phase, decision = view.decision, reason = view.reason;
    unsigned pending = view.pending, action = view.action;
    unsigned count = view.rows, overwritten = view.overwritten;
    RlAdcDiagnostics diag = view.adc_diag;
    unsigned health = view.adc_state, good = view.adc_good;
    uint32_t retries = view.adc_retries, recoveries = view.adc_recoveries, frames = view.adc_frames;
    LineLogRow junction = {.aux = {0, view.junction_aux}};
    leave(key);
    tm_printf(
        (UB *)"LINE profile=1 phase=%u decision=%u reason=%u pending=%u action=%u flags=0x%x raw=%u,%u,%u level=%u,%u rows=%u overwritten=%u\n",
        phase, decision, reason, pending, action, flags, raw[0], raw[1], raw[2], levels[0],
        levels[1], count, overwritten);
    tm_printf(
        (UB *)"CAL_FRONT white=%u,%u black=%u,%u mask=%u valid=%u origin=%u (0=preset 1=SDK28 2=RTOS_saved 3=RAM)\n",
        cal.white[0], cal.white[1], cal.black[0], cal.black[1], cal.mask, calibration_valid(&cal),
        origin);
    unsigned jf = (junction.aux[1] >> 12) & 63U;
    tm_printf(
        (UB *)"JUNCTION policy=angle_only angle_x10=%d both_white=%u prior_white=%u departure_ready=%u branch_accepted=%u accepted_without_white=%u\n",
        rl_log_angle(&junction), jf & 1U, (jf >> 1) & 1U, (jf >> 2) & 1U, (jf >> 3) & 1U,
        (jf >> 4) & 1U);
    tm_printf(
        (UB *)"ADC state=%u good=%u frames=%u conversions=%u avoided_timeout=%u timeouts=%u errors=%u init_errors=%u bad_arguments=%u retries=%u recoveries=%u ch0=%u ch1=%u ch2=%u fail0=%u fail1=%u fail2=%u\n",
        health, good, frames, diag.conversions, diag.avoided_timeout, diag.timeouts, diag.errors,
        diag.init_errors, diag.bad_arguments, retries, recoveries, diag.read_ok[0], diag.read_ok[1],
        diag.read_ok[2], diag.read_fail[0], diag.read_fail[1], diag.read_fail[2]);
    const RlAdcFailure failures[2] = {diag.first, diag.last};
    for (unsigned n = 0; n < 2; ++n) {
        const RlAdcFailure *f = &failures[n];
        static const char *const names[] = {"none",          "ready_timeout", "conversion_err",
                                            "reset_timeout", "bad_argument",  "init_ready_timeout"};
        tm_printf(
            (UB *)"ADC_%s reason=%s code=%u channel=%u stage=%u sample=%u cs=0x%x elapsed_us=%u timer_us=%u clk_ctrl=0x%x clk_div=0x%x adc_div=0x%x reset_done=0x%x\n",
            n == 0 ? "FIRST" : "LAST", f->reason < 6 ? names[f->reason] : "unknown", f->reason,
            f->channel, f->stage, f->sample, f->cs, f->elapsed_us, f->timer_us, f->clk_ctrl,
            f->clk_div, f->adc_div, f->resets_done);
    }
}

void rtos_line_dump_if_requested(void) {
    UINT key = enter();
    if (!dump_requested || recording) {
        leave(key);
        return;
    }
    dump_requested = false;
    dumping = true;
    unsigned count = row_count, head = row_head, overwritten = row_overwritten;
    leave(key);
    BringupStatus status;
    bringup_get_status(&status);
    if (status.flags & BR_ACTIVE)
        goto done;
#ifdef APP_RTOS_BARCODE
    if (barcode_dump) {
        barcode_export();
        goto done;
    }
#endif
    rtos_line_report();
    tm_printf((UB *)"LINE_LOG_BEGIN version=2 rows=%u overwritten=%u\n", count, overwritten);
    tm_printf((UB *)"LINE_LOG_CAL white=%u,%u black=%u,%u origin=%u\n", run_calibration.white[0],
              run_calibration.white[1], run_calibration.black[0], run_calibration.black[1],
              run_cal_origin);
    tm_printf((
        UB *)"LINE_LOG_POLICY name=angle_only departure_deg_gt=10 departure_us=30000 branch_us=30000\n");
    tm_printf((
        UB *)"sequence,ms,ticks_l,ticks_r,speed_l_x10,speed_r_x10,target_l,target_r,pwm_l,pwm_r,adc_l,adc_r,adc_side,level_l,level_r,phase,decision,reason,side_black,busy,white,search,pending,action,dt_us,cost_us,filtered_l,filtered_r,optical_target_l,optical_target_r,steer_gain_milli,damping_milli,stop_reason,control_flags,turn_degrees_x10,junction_both_white,junction_prior_white,junction_departure_ready,junction_branch_accepted,junction_accepted_without_white,junction_accept_event\n");
    for (unsigned n = 0; n < count; ++n) {
        while (tm_usb_pending() > 1024 && tm_usb_state() == 3)
            (void)tk_dly_tsk(2);
        if (tm_usb_state() != 3)
            break;
        /* Starts are blocked while exporting, so rows remain frozen. */
        LineLogRow r = rows[(head + RL_LOG_ROWS - count + n) % RL_LOG_ROWS];
        tm_printf(
            (UB *)"LINE_ROW %u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u\n",
            r.sequence, r.ms, r.ticks[0], r.ticks[1], r.speed[0], r.speed[1], r.target[0],
            r.target[1], r.pwm[0], r.pwm[1], r.optics[0] & 4095U, (r.optics[0] >> 12) & 4095U,
            (r.optics[0] >> 24) | ((r.optics[1] & 15U) << 8), (r.optics[1] >> 4) & 1023U,
            (r.optics[1] >> 14) & 1023U, (r.optics[1] >> 24) & 15U, r.meta & 63U,
            (r.meta >> 6) & 63U, (r.meta >> 12) & 1U, (r.meta >> 13) & 1U, (r.meta >> 14) & 1U,
            (r.meta >> 15) & 1U, rl_route_ascii((r.meta >> 16) & 3U),
            rl_route_ascii((r.meta >> 18) & 3U), r.dt_us, r.cost_us, r.aux[0] & 4095U,
            (r.aux[0] >> 12) & 4095U, r.optical_target[0], r.optical_target[1],
            (r.aux[0] >> 24) | ((r.aux[1] & 3U) << 8), (r.aux[1] >> 2) & 1023U,
            (r.meta >> 20) & 15U, (r.meta >> 24) & 127U, rl_log_angle(&r), (r.aux[1] >> 12) & 1U,
            (r.aux[1] >> 13) & 1U, (r.aux[1] >> 14) & 1U, (r.aux[1] >> 15) & 1U,
            (r.aux[1] >> 16) & 1U, (r.aux[1] >> 17) & 1U);
    }
    tm_printf((UB *)"LINE_LOG_END rows=%u\n", count);
done:
    key = enter();
    dumping = false;
    leave(key);
}

void rtos_line_flash_task(INT code, void *exinf) {
    (void)code;
    (void)exinf;
    for (;;) {
        board_line_flash_worker();
        (void)tk_dly_tsk(1);
    }
}
#ifdef APP_RTOS_BARCODE
#include "../barcode_navigation/barcode_runtime.inc"
#endif
