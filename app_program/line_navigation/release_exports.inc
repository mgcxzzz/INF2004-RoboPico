/* Included by the core1 line owner, after barcode_runtime.inc. No hardware
 * acquisition, lwIP calls, allocations or host waits occur during export. */
#include "src/navigation/nav_codec.h"

static uint32_t release_nav_sequence(unsigned *count) {
    MotionEvent last;
    *count = motion_log_count();
    return *count && motion_log_read(*count - 1U, &last) ? last.sequence : 0U;
}
static void release_line_bytes(const LineLogRow *r, uint8_t out[56]) {
    rl_put32(out, r->ms);
    for (unsigned w = 0; w < 2; ++w) {
        rl_put32(out + 4 + 4*w, (uint32_t)r->ticks[w]);
        rl_put32(out + 12 + 4*w, r->optics[w]);
        rl_put16(out + 20 + 2*w, (uint16_t)r->speed[w]);
        rl_put16(out + 24 + 2*w, (uint16_t)r->target[w]);
        rl_put16(out + 28 + 2*w, (uint16_t)r->pwm[w]);
        rl_put32(out + 44 + 4*w, r->aux[w]);
        rl_put16(out + 52 + 2*w, (uint16_t)r->optical_target[w]);
    }
    rl_put16(out + 32, r->dt_us); rl_put16(out + 34, r->cost_us);
    rl_put32(out + 36, r->meta); rl_put32(out + 40, r->sequence);
}
void rtos_line_release_info(uint8_t seq, uint8_t out[15]) {
    UINT key = enter();
    LineCalibration c = row_count ? run_calibration : view.cal;
    unsigned origin = row_count ? run_cal_origin : view.cal_origin;
    uint16_t run = telemetry.run_id;
    bool recorded = row_count != 0;
    leave(key);
    memset(out, 0, 15);
    out[0] = 0xf2; out[1] = 12; out[2] = seq;
    out[3] = 0xc2; out[4] = 0x20; /* v0.2.0; major low/minor high nibble */
    out[5] = 0x30 | (calibration_valid(&c) ? 0x40 : 0);
    out[6] = RL_PROFILE | ((origin & 3U) << 2) | (recorded ? 0x10 : 0);
    uint16_t levels[4] = {c.white[0], c.white[1], c.black[0], c.black[1]};
    for (unsigned k = 0; k < 4; ++k)
        for (unsigned bit = 0; bit < 12; ++bit)
            out[7 + (12*k + bit)/8] |= ((levels[k] >> bit) & 1U) << ((12*k + bit)%8);
    rl_put16(out + 13, run);
}
uint8_t rtos_line_log_reply(const WifiCommand *cmd) {
    BringupStatus s;
    bringup_get_status(&s);
    uint32_t now = (uint32_t)board_time_us();
    unsigned dataset = cmd->arg >> 12, index = cmd->arg & 4095U;
    if (dataset > 3) return WP_ACK_FORMAT;
    if ((s.flags & BR_ACTIVE) || rtos_line_pending() || dumping || dump_requested)
        return WP_ACK_BUSY;
    unsigned total = dataset < 2 ? row_count : dataset == 2 ? barcode.raw_count : 0;
    uint32_t nav_seq = dataset == 3 ? release_nav_sequence(&total) : 0;
    UINT key = enter();
    bool valid = index == 0 || rl_export_same(&wireless_export, dataset, total,
                                               run_id, nav_seq, now);
    if (valid) rl_export_open(&wireless_export, dataset, total, run_id, nav_seq, now);
    leave(key);
    if (!valid) return WP_ACK_BUSY;
    uint8_t data[64] = {0}, packet[64], length = 0;
    unsigned bytes = 0, status = index < total ? WPK_LOG_STATUS_OK : WPK_LOG_STATUS_UNAVAILABLE;
    if (!status && dataset < 2) {
        LineLogRow r = rows[(row_head + RL_LOG_ROWS - row_count + index) % RL_LOG_ROWS];
        if (dataset == 1) { release_line_bytes(&r, data); bytes = 56; }
        else {
            float l = raw_to_mm(0, r.ticks[0]), right = raw_to_mm(1, r.ticks[1]);
            float centre = (l + right)*.5f - release_run_centre;
            WifiMotionPacket p = {.uptime_ms = r.ms, .elapsed_ms = r.ms,
                .run_id = run_id, .phase = (uint8_t)((r.optics[1] >> 24) & 15U),
                .profile = RL_PROFILE, .flags = 0xc2 | (((r.meta >> 24) & BR_ACTIVE) ? 5U : 0U) |
                         (((r.meta >> 24) & BR_ESTOP) ? 8U : 0U),
                .speed_left_0p1mm_s = r.speed[0], .speed_right_0p1mm_s = r.speed[1],
                .target_left_mm_s = r.target[0], .target_right_mm_s = r.target[1],
                .pwm_left = r.pwm[0], .pwm_right = r.pwm[1],
                .adc_left = r.optics[0] & 4095U, .adc_right = (r.optics[0] >> 12) & 4095U,
                .adc_barcode = (r.optics[0] >> 24) | ((r.optics[1] & 15U) << 8),
                .calibrated_level_left = (r.optics[1] >> 4) & 1023U,
                .calibrated_level_right = (r.optics[1] >> 14) & 1023U,
                .distance_mm = centre > 0 ? (uint32_t)centre : 0,
                .heading_cdeg = (int16_t)lroundf(remainderf(heading_deg(l, right), 360)*100),
                .line_levels = (((r.optics[1] >> 4) & 1023U) >= 400 ? 1U : 0U) |
                               (((r.optics[1] >> 14) & 1023U) >= 400 ? 2U : 0U) |
                               ((r.meta >> 12) & 1U ? 4U : 0U),
                .recovery_state = (r.meta >> 15) & 1U ? 2U : 0U};
            if (!wpk_motion(&p, data)) return WP_ACK_BUSY;
            bytes = WPK_MOTION_BYTES;
        }
    } else if (!status && dataset == 2) {
        memcpy(data, barcode.raw[(barcode.raw_head + RB_RAW_ROWS - barcode.raw_count + index) % RB_RAW_ROWS].bytes, RB_RAW_BYTES);
        bytes = RB_RAW_BYTES;
    } else if (!status) {
        MotionEvent event;
        if (!motion_log_read(index, &event)) return WP_ACK_BUSY;
        nav_event_encode(data, &event); bytes = NAV_EVENT_BYTES;
        unsigned current_total;
        if (release_nav_sequence(&current_total) != nav_seq || current_total != total)
            return WP_ACK_BUSY; /* Never combine shifted navigation-ring generations. */
    }
    unsigned parts = status ? 1 : (bytes + 48U)/49U;
    bool queued = true;
    for (unsigned part = 0; part < parts; ++part) {
        unsigned at = 49U*part, n = status ? 0 : bytes - at;
        if (n > 49U) n = 49U;
        if (!wpk_log_chunk(rn_next_sequence(WPK_SRC_LOG_CHUNK), dataset, cmd->id, index,
                           total, part, parts, status, data + at, n, packet, &length) ||
            !rn_tx_push(RN_TLM, packet, length, board_kernel_ms())) queued = false;
    }
    if (queued && (status || index + 1U >= total)) {
        key = enter(); wireless_export.active = 0; leave(key);
    }
    return WP_ACK_ACCEPTED; /* Missing chunks are retried by request ID/row. */
}
