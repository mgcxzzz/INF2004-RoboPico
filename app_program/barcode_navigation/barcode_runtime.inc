/**
 * @file barcode_runtime.inc
 * @brief Barcode runtime.
 * @details The ADC owner feeds both optical decoders and junction
 * qualification. Accepted navigation crosses the coordinator; this module never
 * writes PWM.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Included only by line_service.c. Core1 ADC/line owner reads live decoder
 * state. Core0 console reads copied view, or frozen data during gated export.
 * Network API below copies packets; this code never calls lwIP/CYW43. */
static void barcode_line_packet (unsigned event, uint32_t now)
{
    barcode_nav_t * p_count = &g_planner.nav;
    wifi_line_packet_t data = {
        .seq = rn_next_sequence(WPK_SRC_LINE),
        .uptime_ms = now,
        .event = event,
        .adc_left = g_adc[0],
        .adc_right = g_adc[2],
        .adc_barcode = g_adc[1],
        .calibrated_level_left =
            line_level(g_input.adc[0], g_calibration_value.white[0],
                       g_calibration_value.black[0]),
        .calibrated_level_right =
            line_level(g_input.adc[1], g_calibration_value.white[1],
                       g_calibration_value.black[1]),
        .last_decoded_letter = p_count->last_code,
        .pending_letter = p_count->pending,
        .pending_action = p_count->pending_action,
        .pending_revision = p_count->pending_revision,
        .executing_action = p_count->busy ? p_count->executing_action : 0,
        .executing_revision = p_count->busy ? p_count->executing_revision : 0};
    uint8_t bytes[WPK_LINE_BYTES];
    if (wpk_line(&data, bytes))
    {
        (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
    }
}
static void barcode_event_packet (const barcode_event_t * p_event,
                                  unsigned event, uint32_t now)
{
    motion_status_t state = motion_get_status();
    wifi_nav_packet_t data = {.seq = rn_next_sequence(WPK_SRC_NAV),
                              .uptime_ms = now,
                              .event = event,
                              .letter = (p_event->code >= 'A') &&
                                                (p_event->code <= 'Z')
                                            ? p_event->code
                                            : 0,
                              .action = p_event->action,
                              .map_revision = p_event->map_revision,
                              .generation = state.generation,
                              .fault_bitmap = state.fault,
                              .owner = state.owner,
                              .mode = state.mode,
                              .command_state = state.command_state,
                              .detail = p_event->flags};
    uint8_t bytes[WPK_NAV_BYTES];
    if (wpk_nav(&data, bytes))
    {
        (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
    }
}
static void barcode_publish (uint64_t sample_time)
{
    (void)sample_time;
    barcode_nav_t * p_count = &g_planner.nav;
    uint32_t now = board_kernel_ms(); /* Queue freshness uses KERNEL time, not
                                         TIMER epoch. */
    if ((!rn_drive_telemetry_enabled()) || (!rn_drive_online()))
    {
        g_barcode.event_seen = p_count->event_sequence;
        g_barcode_peak_sent = g_barcode.peak_serial;
        return;
    }
    uint32_t available = p_count->event_sequence - g_barcode.event_seen;
    if (available > p_count->event_count)
    {
        available = p_count->event_count;
        g_barcode.event_seen = p_count->event_sequence - available;
    }
    for (unsigned item_index = 0; (item_index < 8) && available;
         ++item_index, --available)
    {
        const barcode_event_t * p_event =
            &p_count->events[(p_count->event_head + B_EVENTS - available) %
                             B_EVENTS];
        ++g_barcode.event_seen;
        unsigned event =
            B_DECODE == p_event->type           ? WPK_EVENT_DECODED
            : B_CROSS == p_event->type          ? WPK_EVENT_JUNCTION_CONFIRMED
            : B_DISPATCH == p_event->type       ? WPK_EVENT_DISPATCH
            : B_COMPLETE == p_event->type       ? WPK_EVENT_COMPLETED
            : B_RECOVERY_WAIT == p_event->type  ? WPK_EVENT_RECOVERY_WAIT
            : B_RECOVERY_SWEEP == p_event->type ? WPK_EVENT_RECOVERY_SWEEP
            : B_RECOVERY_FOUND == p_event->type ? WPK_EVENT_RECOVERY_FOUND
            : (B_BAD_FRAME == p_event->type ||
               B_ANALOG_CONFLICT == p_event->type ||
               B_RECOVERY_FAIL == p_event->type)
                ? WPK_EVENT_FAULT
                : 0;
        if (!event)
        {
            continue;
        }
        barcode_event_packet(p_event, event, now);
        if (B_DECODE == p_event->type)
        {
            barcode_event_packet(p_event,
                                 '-' == p_event->action ? WPK_EVENT_UNASSIGNED
                                                        : WPK_EVENT_QUEUED,
                                 now);
            barcode_line_packet(WPK_EVENT_DECODED, now);
            tm_printf((UB *)"BARCODE_DECODE ms=%u letter=%c action=%c "
                            "revision=%u flags=%u\n",
                      (uint32_t)p_event->ms, p_event->code, p_event->action,
                      p_event->map_revision, p_event->flags);
        }
    }
    if (g_barcode_peak_sent != g_barcode.peak_serial)
    {
        uint8_t bytes[BP_TLM_FRAME];
        bp_tlm_pack(&g_barcode.last_peak, now, g_barcode.peak_flags,
                    g_barcode.groups, rn_next_sequence(15), bytes);
        (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
        g_barcode_peak_sent = g_barcode.peak_serial;
    }
    if (now - g_barcode_status_ms >= 250)
    {
        barcode_line_packet(WPK_EVENT_STATUS, now);
        g_barcode_status_ms = now;
    }
}
static void barcode_export_wait (void)
{
    while ((tm_usb_pending() > 1024) && 3 == tm_usb_state())
    {
        (void)tk_dly_tsk(2);
    }
}
static void barcode_export (void)
{
    barcode_nav_t * p_count = &g_planner.nav;
    tm_printf((UB *)"BARCODE_LOG_BEGIN version=1 events=%u raw=%u "
                    "overwritten=%u frozen=%u\n",
              p_count->event_count, g_barcode.raw_count,
              g_barcode.raw_overwritten, g_barcode.raw_frozen);
    tm_printf((UB *)"BARCODE_LOG_CAL white=%u,%u black=%u,%u origin=%u\n",
              g_run_calibration.white[0], g_run_calibration.white[1],
              g_run_calibration.black[0], g_run_calibration.black[1],
              g_run_cal_origin);
    for (unsigned item_index = p_count->event_count;
         (item_index > 0) && 3 == tm_usb_state(); --item_index)
    {
        barcode_export_wait();
        const barcode_event_t * p_event =
            &p_count->events[(p_count->event_head + B_EVENTS - item_index) %
                             B_EVENTS];
        tm_printf((UB *)"BARCODE_EVENT %u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u\n",
                  p_event->number, (uint32_t)p_event->ms, p_event->path10,
                  p_event->width100, p_event->type, p_event->code,
                  p_event->action, p_event->map_revision, p_event->flags,
                  p_event->pattern[0], p_event->pattern[1],
                  p_event->pattern[2]);
    }
    for (unsigned item_index_value = 0;
         (item_index_value < g_barcode.raw_count) && 3 == tm_usb_state();
         ++item_index_value)
    {
        barcode_export_wait();
        const uint8_t * p_data =
            g_barcode
                .raw[(g_barcode.raw_head + RB_RAW_ROWS - g_barcode.raw_count +
                      item_index_value) %
                     RB_RAW_ROWS]
                .bytes;
        tm_printf((UB *)"BARCODE_RAW %u,%u,%d,%u\n", item_index_value,
                  rb_get32(p_data), (int32_t)rb_get32(p_data + 4),
                  p_data[8] | (unsigned)p_data[9] << 8);
    }
    tm_printf((UB *)"BARCODE_LOG_END events=%u raw=%u\n", p_count->event_count,
              g_barcode.raw_count);
}

/* End of file. */
