/* Included only after the baseline hardware globals and helpers. */
#include "src/telemetry/drive_diag_storage.h"
#define DIAG_OFFSET (PICO_FLASH_SIZE_BYTES - 0x55000u)
_Static_assert(DIAG_OFFSET + DIAG_BANK == PICO_FLASH_SIZE_BYTES - 0x51000u,
               "Diagnostic/nav banks must be adjacent");
static bool diag_new, diag_saved, diag_restored, diag_failed;
static bool diag_storage_fits(void) {
    return (uintptr_t)&__flash_binary_end <= XIP_BASE + DIAG_OFFSET;
}
static void load_drive_diag(void) {
    if (diag_storage_fits() && diag_bank_valid((const uint8_t *)(XIP_BASE + DIAG_OFFSET))) {
        diag_restore(&diag_capture, (const uint8_t *)(XIP_BASE + DIAG_OFFSET));
        diag_restored = diag_saved = true;
    }
}
static void save_drive_diag(void) {
    if (!diag_new || guard.active)
        return;
    if (!diag_storage_fits()) {
        diag_failed = true;
        return;
    }
    outputs_off();
    diag_capture.duration_us = guard.duration_us;
    diag_capture.reason = guard.reason;
    for (unsigned w = 0; w < 2; ++w)
        diag_capture.ticks[w] = cut_ticks[w];
    uint8_t header[DIAG_HEADER];
    diag_header(header, &diag_capture);
    if (!wifi_transport_flash_begin()) {
        diag_failed = true;
        return;
    }
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(DIAG_OFFSET, DIAG_BANK);
    flash_range_program(DIAG_OFFSET + DIAG_HEADER, (const uint8_t *)diag_capture.rows,
                        sizeof(diag_capture.rows));
    flash_range_program(DIAG_OFFSET, header, sizeof(header)); /* valid header last */
    restore_interrupts(irq);
    wifi_transport_flash_end();
    const uint8_t *bank = (const uint8_t *)(XIP_BASE + DIAG_OFFSET);
    diag_failed = !diag_bank_valid(bank) || memcmp(bank, header, sizeof(header)) ||
                  memcmp(bank + DIAG_HEADER, diag_capture.rows, sizeof(diag_capture.rows));
    diag_saved = !diag_failed;
    diag_restored = false;
    diag_new = false;
}
static bool diag_half_field(unsigned n) {
    return (n >= D_SPEED_L && n <= D_FF_R) || (n >= D_MEAN_BEFORE && n <= D_HALF);
}
static void dump_drive_diag(void) {
    if (guard.active || trace_settling) {
        puts("DIAG rejected: stop and wait5s before d.");
        return;
    }
    if (!diag_capture.begun || !diag_capture.count) {
        puts("DIAG unavailable: run INTERFACE WIFI v1 first.");
        return;
    }
    bool matches = diag_capture.duration_us == guard.duration_us &&
                   diag_capture.ticks[0] == cut_ticks[0] && diag_capture.ticks[1] == cut_ticks[1] &&
                   diag_capture.profile == recorded_mode && diag_capture.reason == guard.reason;
    for (unsigned w = 0; w < 2; ++w)
        if (diag_capture.white[w] != calibration.white[w] ||
            diag_capture.black[w] != calibration.black[w])
            matches = false;
    DiagValues first, last;
    diag_unpack(
        &diag_capture.rows[(diag_capture.head + DIAG_ROWS - diag_capture.count) % DIAG_ROWS],
        &first);
    diag_unpack(&diag_capture.rows[(diag_capture.head + DIAG_ROWS - 1) % DIAG_ROWS], &last);
    printf(
        "DRIVE_DIAG_BEGIN: base=interface1.1 control=continuous2 capture=4 rows=%u row_bytes=%u stored=%u restored=%u save_failed=%u matches_motion=%u frozen=%u trigger=%u trigger_us=%lu first_us=%" PRIi64
        " last_us=%" PRIi64 " post_remaining=%u\n",
        diag_capture.count, DIAG_ROW_BYTES, diag_saved, diag_restored, diag_failed, matches,
        diag_capture.frozen, diag_capture.trigger, (unsigned long)diag_capture.trigger_us,
        first.v[D_US], last.v[D_US], diag_capture.post);
    printf(
        "DIAG_TIMING: ticks=%lu max_interval_us=%lu intervals_ge20ms=%lu max_control_observed_us=%lu max_pack_us=%lu\n",
        (unsigned long)diag_capture.total, (unsigned long)diag_capture.max_gap,
        (unsigned long)diag_capture.late20, (unsigned long)diag_capture.max_cost,
        (unsigned long)diag_capture.max_pack);
    printf("DIAG_CAL: white=%u,%u black=%u,%u profile=%u duration_us=%" PRIu64 " stop=%u\n",
           diag_capture.white[0], diag_capture.white[1], diag_capture.black[0],
           diag_capture.black[1], diag_capture.profile, diag_capture.duration_us,
           diag_capture.reason);
    puts(
        "DIAG_FLAGS: 1=filtered,2=urgent_bypass,4=scan,8=filter_was_active,16=filter_active,32=white,64=load_prepare,128=leave_fast,256/512=legacy_kick_or_continuous_start_L/R,1024/2048=peak_L/R,4096/8192=loaded_L/R,16384=commit,32768=clipped,65536=optical_valid,131072=run_active_at_observation,262144=cruise_I_preserved");
    puts(
        "DIAG_STEERING: when filter_active, GAIN0..7 maps70..100percent; DAMP0..3 maps0..1 return damping weight. Coarse observational quantisation only.");
    puts(
        "DIAG_UNITS: US/DT/COST=microseconds; SPEED/REF/MEAN/HALF=mm/s; I/FF=native_PI_units(raw_before1.2_when_DRIVER0or3,physical_PWM_when_DRIVER1or2); FF=derived_base_at_REF_not_applied_output; DRIVER0=legacy_ordinary_or_loaded,1=maneuver,2=ramp,3=continuous; REQUEST/PWM=physical_permille; TICKS=raw_signed_encoder_delta");
    static const char *names[D_FIELDS] = {
#define DIAG_NAME(n, w, s) #n,
        DIAG_FIELDS(DIAG_NAME)
#undef DIAG_NAME
    };
    for (unsigned n = 0; n < D_FIELDS; ++n)
        printf("%s%s", n ? "," : "", names[n]);
    putchar('\n');
    for (unsigned k = 0; k < diag_capture.count; ++k) {
        if (!stdio_usb_connected())
            return;
        DiagValues v;
        diag_unpack(&diag_capture.rows[(diag_capture.head + DIAG_ROWS - diag_capture.count + k) %
                                       DIAG_ROWS],
                    &v);
        for (unsigned n = 0; n < D_FIELDS; ++n) {
            if (n)
                putchar(',');
            if (diag_half_field(n))
                printf("%.1f", (double)v.v[n] / 2);
            else
                printf("%" PRIi64, v.v[n]);
        }
        putchar('\n');
        stdio_flush();
        sleep_ms(5);
    }
    puts("DRIVE_DIAG_END: complete=1");
}
static void observe_drive_tick(const CircuitInput *i, const BenchEncoder e[2],
                               const int32_t next[2], unsigned wheel_profile, bool ramp,
                               bool maneuver, bool external, const bool loaded[2], uint64_t began,
                               uint64_t elapsed) {
    if (diag_observing) {
        diag_tick.v[D_US] = i->now - guard.start_us;
        diag_tick.v[D_DT] = elapsed;
        for (unsigned w = 0; w < 2; ++w) {
            diag_tick.v[D_TARGET_L + w] = demand.target[w];
            diag_tick.v[D_SPEED_L + w] = diag_half(velocity[w]);
            diag_tick.v[D_REF_L + w] = diag_half(controller[w].pi.ramp);
            diag_tick.v[D_I_L + w] = diag_half(controller[w].pi.integral);
            float ff =
                feedforward(w, controller[w].sign ? controller[w].sign : 1, controller[w].pi.ramp);
            if (continuous_active)
                ff = continuous_feedforward(w, controller[w].pi.ramp) / 1.2f;
            else if (maneuver)
                ff = fmaxf(400, (float)effort_scale20((int32_t)(ff + .5f)));
            else if (ramp)
                ff = (float)effort_scale20((int32_t)(ff + .5f));
            else if (loaded[w])
                ff = fmaxf(ff, J_TURN_FEEDFORWARD);
            if (!demand.target[w])
                ff = 0;
            diag_tick.v[D_FF_L + w] = diag_half(ff);
            diag_tick.v[D_REQUEST_L + w] = next[w];
            diag_tick.v[D_PWM_L + w] = platform_motor_read(w); /* includes ISR stop */
            diag_tick.v[D_TICKS_L + w] = e[w].position;
            if (controller[w].kick.active ||
                (continuous_active && continuous_wheel[w].start_active))
                diag_tick.v[D_FLAGS] |= w ? DF_KICK_R : DF_KICK_L;
            if (controller[w].peak_active)
                diag_tick.v[D_FLAGS] |= w ? DF_PEAK_R : DF_PEAK_L;
            if (loaded[w])
                diag_tick.v[D_FLAGS] |= w ? DF_LOADED_R : DF_LOADED_L;
        }
        diag_tick.v[D_PHASE] = circuit.phase;
        diag_tick.v[D_DECISION] = demand.decision;
        diag_tick.v[D_PROFILE] = wheel_profile;
        diag_tick.v[D_ASSIST] = assist_code;
        diag_tick.v[D_FEEDBACK] = feedback_mask;
        diag_tick.v[D_DRIVER] = continuous_active ? 3 : maneuver ? 1 : ramp ? 2 : 0;
        diag_tick.v[D_EXTERNAL] = external;
        if (current_tracker()->core.white)
            diag_tick.v[D_FLAGS] |= DF_WHITE;
        if (guard.active)
            diag_tick.v[D_FLAGS] |= DF_RUN_ACTIVE;
        diag_tick.v[D_MEAN] = diag_half(gentle.mean);
        diag_tick.v[D_HALF] = diag_half(gentle.half);
        if (continuous_steering.active) {
            diag_tick.v[D_GAIN] =
                (int64_t)lroundf(clampf((continuous_steering.gain - .70f) / .30f, 0, 1) * 7);
            diag_tick.v[D_DAMP] =
                (int64_t)lroundf(clampf(continuous_steering.damp_weight, 0, 1) * 3);
        }
    } else {
        diag_tick.v[D_DT] = elapsed;
    }
    uint64_t pack_start = time_us_64();
    int slot = diag_push(&diag_capture, &diag_tick);
    uint32_t pack = (uint32_t)(time_us_64() - pack_start);
    if (pack > diag_capture.max_pack)
        diag_capture.max_pack = pack;
    diag_cost(&diag_capture, slot, (uint32_t)(time_us_64() - began));
}
