/**
 * @file release_exports.inc
 * @brief Release exports.
 * @details ADC/control ownership covers sensor acquisition, planning and
 * calibration. Other tasks consume protected copies; flash and stored exports
 * require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Included by the core1 line owner, after barcode_runtime.inc. No hardware
 * acquisition, lwIP calls, allocations or host waits occur during export. */
#include "src/navigation/nav_codec.h"

static uint32_t release_nav_sequence (unsigned * p_count)
{
    motion_event_t last;
    *p_count = motion_log_count();
    return (*p_count) && (motion_log_read(*p_count - 1U, &last)) ? last.sequence
                                                                 : 0U;
}
static void release_line_bytes (const line_log_row_t * p_result,
                                uint8_t out[56])
{
    rl_put32(out, p_result->ms);
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        rl_put32(out + 4 + 4 * wheel, (uint32_t)p_result->ticks[wheel]);
        rl_put32(out + 12 + 4 * wheel, p_result->optics[wheel]);
        rl_put16(out + 20 + 2 * wheel, (uint16_t)p_result->speed[wheel]);
        rl_put16(out + 24 + 2 * wheel, (uint16_t)p_result->target[wheel]);
        rl_put16(out + 28 + 2 * wheel, (uint16_t)p_result->pwm[wheel]);
        rl_put32(out + 44 + 4 * wheel, p_result->aux[wheel]);
        rl_put16(out + 52 + 2 * wheel,
                 (uint16_t)p_result->optical_target[wheel]);
    }
    rl_put16(out + 32, p_result->dt_us);
    rl_put16(out + 34, p_result->cost_us);
    rl_put32(out + 36, p_result->meta);
    rl_put32(out + 40, p_result->sequence);
}
void rtos_line_release_info (uint8_t seq, uint8_t out[15])
{
    UINT key = enter();
    line_calibration_t character = g_row_count ? g_run_calibration : g_view.cal;
    unsigned origin = g_row_count ? g_run_cal_origin : g_view.cal_origin;
    uint16_t run = g_telemetry.run_id;
    bool recorded = g_row_count != 0;
    leave(key);
    memset(out, 0, 15);
    out[0] = 0xf2;
    out[1] = 12;
    out[2] = seq;
    out[3] = 0xc2;
    out[4] = 0x20; /* v0.2.7; major low/minor high nibble */
    out[5] = 0x37 | (calibration_valid(&character) ? 0x40 : 0);
    out[6] = RL_PROFILE | ((origin & 3U) << 2) | (recorded ? 0x10 : 0);
    uint16_t levels[4] = {character.white[0], character.white[1],
                          character.black[0], character.black[1]};
    for (unsigned item_index = 0; item_index < 4; ++item_index)
    {
        for (unsigned bit = 0; bit < 12; ++bit)
        {
            out[7 + (12 * item_index + bit) / 8] |=
                ((levels[item_index] >> bit) & 1U)
                << ((12 * item_index + bit) % 8);
        }
    }
    rl_put16(out + 13, run);
}
uint8_t rtos_line_log_reply (const wifi_command_t * p_cmd)
{
    bringup_status_t state;
    bringup_get_status(&state);
    uint32_t now = (uint32_t)board_time_us();
    unsigned dataset = p_cmd->arg >> 12, index = p_cmd->arg & 4095U;
    if (dataset > 3)
    {
        return WP_ACK_FORMAT;
    }
    if ((((state.flags & BR_ACTIVE) || (rtos_line_pending())) || g_dumping) ||
        g_dump_requested)
    {
        return WP_ACK_BUSY;
    }
    unsigned total = dataset < 2    ? g_row_count
                     : 2 == dataset ? g_barcode.raw_count
                                    : 0;
    uint32_t nav_seq = 3 == dataset ? release_nav_sequence(&total) : 0;
    UINT key = enter();
    bool valid = 0 == index || (rl_export_same(&g_wireless_export, dataset,
                                               total, g_run_id, nav_seq, now));
    if (valid)
    {
        rl_export_open(&g_wireless_export, dataset, total, g_run_id, nav_seq,
                       now);
    }
    leave(key);
    if (!valid)
    {
        return WP_ACK_BUSY;
    }
    uint8_t data[64] = {0}, packet[64], length = 0;
    unsigned bytes = 0, status = index < total ? WPK_LOG_STATUS_OK
                                               : WPK_LOG_STATUS_UNAVAILABLE;
    if ((!status) && (dataset < 2))
    {
        line_log_row_t result =
            g_rows[(g_row_head + RL_LOG_ROWS - g_row_count + index) %
                   RL_LOG_ROWS];
        if (1 == dataset)
        {
            release_line_bytes(&result, data);
            bytes = 56;
        }
        else
        {
            float l_value = raw_to_mm(0, result.ticks[0]),
                  right = raw_to_mm(1, result.ticks[1]);
            float centre = (l_value + right) * .5f - g_release_run_centre;
            wifi_motion_packet_t data_value = {
                .uptime_ms = result.ms,
                .elapsed_ms = result.ms,
                .run_id = g_run_id,
                .phase = (uint8_t)((result.optics[1] >> 24) & 15U),
                .profile = RL_PROFILE,
                .flags = 0xc2 | (((result.meta >> 24) & BR_ACTIVE) ? 5U : 0U) |
                         (((result.meta >> 24) & BR_ESTOP) ? 8U : 0U),
                .speed_left_0p1mm_s = result.speed[0],
                .speed_right_0p1mm_s = result.speed[1],
                .target_left_mm_s = result.target[0],
                .target_right_mm_s = result.target[1],
                .pwm_left = result.pwm[0],
                .pwm_right = result.pwm[1],
                .adc_left = result.optics[0] & 4095U,
                .adc_right = (result.optics[0] >> 12) & 4095U,
                .adc_barcode =
                    (result.optics[0] >> 24) | ((result.optics[1] & 15U) << 8),
                .calibrated_level_left = (result.optics[1] >> 4) & 1023U,
                .calibrated_level_right = (result.optics[1] >> 14) & 1023U,
                .distance_mm = centre > 0 ? (uint32_t)centre : 0,
                .heading_cdeg = (int16_t)lroundf(
                    remainderf(heading_deg(l_value, right), 360) * 100),
                .line_levels =
                    (((result.optics[1] >> 4) & 1023U) >= 400 ? 1U : 0U) |
                    (((result.optics[1] >> 14) & 1023U) >= 400 ? 2U : 0U) |
                    ((result.meta >> 12) & 1U ? 4U : 0U),
                .recovery_state = (result.meta >> 15) & 1U ? 2U : 0U};
            if (!wpk_motion(&data_value, data))
            {
                return WP_ACK_BUSY;
            }
            bytes = WPK_MOTION_BYTES;
        }
    }
    else if ((!status) && 2 == dataset)
    {
        memcpy(data,
               g_barcode
                   .raw[(g_barcode.raw_head + RB_RAW_ROWS -
                         g_barcode.raw_count + index) %
                        RB_RAW_ROWS]
                   .bytes,
               RB_RAW_BYTES);
        bytes = RB_RAW_BYTES;
    }
    else if (!status)
    {
        motion_event_t event;
        if (!motion_log_read(index, &event))
        {
            return WP_ACK_BUSY;
        }
        nav_event_encode(data, &event);
        bytes = NAV_EVENT_BYTES;
        unsigned current_total;
        if ((release_nav_sequence(&current_total) != nav_seq) ||
            (current_total != total))
        {
            return WP_ACK_BUSY; /* Never combine shifted navigation-ring
                                   generations. */
        }
    }
    unsigned parts = status ? 1 : (bytes + 48U) / 49U;
    bool queued = true;
    for (unsigned part = 0; part < parts; ++part)
    {
        unsigned at_value = 49U * part, count = status ? 0 : bytes - at_value;
        if (count > 49U)
        {
            count = 49U;
        }
        if ((!wpk_log_chunk(rn_next_sequence(WPK_SRC_LOG_CHUNK), dataset,
                            p_cmd->id, index, total, part, parts, status,
                            data + at_value, count, packet, &length)) ||
            (!rn_tx_push(RN_TLM, packet, length, board_kernel_ms())))
        {
            queued = false;
        }
    }
    if (queued && (status || (index + 1U >= total)))
    {
        key = enter();
        g_wireless_export.active = 0;
        leave(key);
    }
    return WP_ACK_ACCEPTED; /* Missing chunks are retried by request ID/row. */
}

/* End of file. */
