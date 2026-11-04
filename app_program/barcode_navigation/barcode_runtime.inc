/* Included only by line_service.c. Core1 ADC/line owner reads live decoder
 * state. Core0 console reads copied view, or frozen data during gated export.
 * Network API below copies packets; this code never calls lwIP/CYW43. */
static void barcode_line_packet(unsigned event, uint32_t now) {
    BarcodeNav *n = &planner.nav;
    WifiLinePacket p = {.seq = rn_next_sequence(WPK_SRC_LINE),
                        .uptime_ms = now,
                        .event = event,
                        .adc_left = adc[0],
                        .adc_right = adc[2],
                        .adc_barcode = adc[1],
                        .calibrated_level_left =
                            line_level(input.adc[0], calibration.white[0], calibration.black[0]),
                        .calibrated_level_right =
                            line_level(input.adc[1], calibration.white[1], calibration.black[1]),
                        .last_decoded_letter = n->last_code,
                        .pending_letter = n->pending,
                        .pending_action = n->pending_action,
                        .pending_revision = n->pending_revision,
                        .executing_action = n->busy ? n->executing_action : 0,
                        .executing_revision = n->busy ? n->executing_revision : 0};
    uint8_t bytes[WPK_LINE_BYTES];
    if (wpk_line(&p, bytes))
        (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
}
static void barcode_event_packet(const BarcodeEvent *e, unsigned event, uint32_t now) {
    MotionStatus s = motion_get_status();
    WifiNavPacket p = {.seq = rn_next_sequence(WPK_SRC_NAV),
                       .uptime_ms = now,
                       .event = event,
                       .letter = e->code >= 'A' && e->code <= 'Z' ? e->code : 0,
                       .action = e->action,
                       .map_revision = e->map_revision,
                       .generation = s.generation,
                       .fault_bitmap = s.fault,
                       .owner = s.owner,
                       .mode = s.mode,
                       .command_state = s.command_state,
                       .detail = e->flags};
    uint8_t bytes[WPK_NAV_BYTES];
    if (wpk_nav(&p, bytes))
        (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
}
static void barcode_publish(uint64_t sample_time) {
    (void)sample_time;
    BarcodeNav *n = &planner.nav;
    uint32_t now = board_kernel_ms(); /* Queue freshness uses KERNEL time, not TIMER epoch. */
    if (!rn_drive_telemetry_enabled() || !rn_drive_online()) {
        barcode.event_seen = n->event_sequence;
        barcode_peak_sent = barcode.peak_serial;
        return;
    }
    uint32_t available = n->event_sequence - barcode.event_seen;
    if (available > n->event_count) {
        available = n->event_count;
        barcode.event_seen = n->event_sequence - available;
    }
    for (unsigned k = 0; k < 8 && available; ++k, --available) {
        const BarcodeEvent *e = &n->events[(n->event_head + B_EVENTS - available) % B_EVENTS];
        ++barcode.event_seen;
        unsigned event =
            e->type == B_DECODE           ? WPK_EVENT_DECODED
            : e->type == B_CROSS          ? WPK_EVENT_JUNCTION_CONFIRMED
            : e->type == B_DISPATCH       ? WPK_EVENT_DISPATCH
            : e->type == B_COMPLETE       ? WPK_EVENT_COMPLETED
            : e->type == B_RECOVERY_WAIT  ? WPK_EVENT_RECOVERY_WAIT
            : e->type == B_RECOVERY_SWEEP ? WPK_EVENT_RECOVERY_SWEEP
            : e->type == B_RECOVERY_FOUND ? WPK_EVENT_RECOVERY_FOUND
            : (e->type == B_BAD_FRAME || e->type == B_ANALOG_CONFLICT || e->type == B_RECOVERY_FAIL)
                ? WPK_EVENT_FAULT
                : 0;
        if (!event)
            continue;
        barcode_event_packet(e, event, now);
        if (e->type == B_DECODE) {
            barcode_event_packet(e, e->action == '-' ? WPK_EVENT_UNASSIGNED : WPK_EVENT_QUEUED,
                                 now);
            barcode_line_packet(WPK_EVENT_DECODED, now);
            tm_printf((UB *)"BARCODE_DECODE ms=%u letter=%c action=%c revision=%u flags=%u\n",
                      (uint32_t)e->ms, e->code, e->action, e->map_revision, e->flags);
        }
    }
    if (barcode_peak_sent != barcode.peak_serial) {
        uint8_t bytes[BP_TLM_FRAME];
        bp_tlm_pack(&barcode.last_peak, now, barcode.peak_flags, barcode.groups,
                    rn_next_sequence(15), bytes);
        (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
        barcode_peak_sent = barcode.peak_serial;
    }
    if (now - barcode_status_ms >= 250) {
        barcode_line_packet(WPK_EVENT_STATUS, now);
        barcode_status_ms = now;
    }
}
static void barcode_export_wait(void) {
    while (tm_usb_pending() > 1024 && tm_usb_state() == 3)
        (void)tk_dly_tsk(2);
}
static void barcode_export(void) {
    BarcodeNav *n = &planner.nav;
    tm_printf((UB *)"BARCODE_LOG_BEGIN version=1 events=%u raw=%u overwritten=%u frozen=%u\n",
              n->event_count, barcode.raw_count, barcode.raw_overwritten, barcode.raw_frozen);
    tm_printf((UB *)"BARCODE_LOG_CAL white=%u,%u black=%u,%u origin=%u\n", run_calibration.white[0],
              run_calibration.white[1], run_calibration.black[0], run_calibration.black[1],
              run_cal_origin);
    for (unsigned k = n->event_count; k > 0 && tm_usb_state() == 3; --k) {
        barcode_export_wait();
        const BarcodeEvent *e = &n->events[(n->event_head + B_EVENTS - k) % B_EVENTS];
        tm_printf((UB *)"BARCODE_EVENT %u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u\n", e->number,
                  (uint32_t)e->ms, e->path10, e->width100, e->type, e->code, e->action,
                  e->map_revision, e->flags, e->pattern[0], e->pattern[1], e->pattern[2]);
    }
    for (unsigned k = 0; k < barcode.raw_count && tm_usb_state() == 3; ++k) {
        barcode_export_wait();
        const uint8_t *p =
            barcode.raw[(barcode.raw_head + RB_RAW_ROWS - barcode.raw_count + k) % RB_RAW_ROWS]
                .bytes;
        tm_printf((UB *)"BARCODE_RAW %u,%u,%d,%u\n", k, rb_get32(p), (int32_t)rb_get32(p + 4),
                  p[8] | (unsigned)p[9] << 8);
    }
    tm_printf((UB *)"BARCODE_LOG_END events=%u raw=%u\n", n->event_count, barcode.raw_count);
}
