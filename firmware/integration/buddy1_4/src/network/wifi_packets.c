#include "src/network/wifi_packets.h"
#include <stddef.h>

static void put_header(uint8_t *out, uint8_t src, uint8_t payload, uint8_t seq) {
    out[0] = (uint8_t)((src << 4) | 2u);
    out[1] = payload;
    out[2] = seq;
}
static void put_u16(uint8_t *p, uint16_t x) {
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
}
static void put_i16(uint8_t *p, int16_t x) {
    put_u16(p, (uint16_t)x);
}
static void put_u32(uint8_t *p, uint32_t x) {
    p[0] = (uint8_t)x;
    p[1] = (uint8_t)(x >> 8);
    p[2] = (uint8_t)(x >> 16);
    p[3] = (uint8_t)(x >> 24);
}
static bool valid_action(uint8_t c) {
    return c == (uint8_t)'S' || c == (uint8_t)'U' || c == (uint8_t)'L' || c == (uint8_t)'R' ||
           c == (uint8_t)'-' || c == 0u;
}
static bool valid_letter(uint8_t c) {
    return c == 0u || (c >= (uint8_t)'A' && c <= (uint8_t)'Z');
}

bool wpk_heartbeat(const WifiHeartbeat *in, uint8_t out[WPK_HEARTBEAT_BYTES]) {
    uint32_t w0, w1, w2;
    if (in == NULL || out == NULL)
        return false;
    w0 = (uint32_t)(in->net > 3u ? 3u : in->net) |
         ((uint32_t)(in->state > 7u ? 7u : in->state) << 2) |
         ((uint32_t)(in->last_cmd > 15u ? 15u : in->last_cmd) << 5) |
         ((uint32_t)(in->last_result > 7u ? 7u : in->last_result) << 9) |
         ((uint32_t)(in->uptime_s > 16383u ? 16383u : in->uptime_s) << 12);
    w1 = (uint32_t)in->published | ((uint32_t)in->lost << 16);
    w2 = (uint32_t)in->accepted | ((uint32_t)in->rejected << 8) | ((uint32_t)in->connects << 16) |
         ((uint32_t)(in->fill_pct > 100u ? 100u : in->fill_pct) << 24);
    put_header(out, WPK_SRC_HEARTBEAT, WPK_HEARTBEAT_PAYLOAD_BYTES, in->seq);
    put_u32(out + 3, w0);
    put_u32(out + 7, w1);
    put_u32(out + 11, w2);
    return true;
}

bool wpk_motion(const WifiMotionPacket *in, uint8_t out[WPK_MOTION_BYTES]) {
    if (in == NULL || out == NULL ||
        (in->calibrated_level_left > 1000u && in->calibrated_level_left != 0xffffu) ||
        (in->calibrated_level_right > 1000u && in->calibrated_level_right != 0xffffu) ||
        in->heading_cdeg < -18000 || in->heading_cdeg > 18000)
        return false;
    put_header(out, WPK_SRC_MOTION, WPK_MOTION_PAYLOAD_BYTES, in->seq);
    put_u32(out + 3, in->uptime_ms);
    put_u32(out + 7, in->elapsed_ms);
    put_u16(out + 11, in->run_id);
    out[13] = in->phase;
    out[14] = in->profile;
    out[15] = in->flags;
    out[16] = in->recovery_state;
    put_i16(out + 17, in->speed_left_0p1mm_s);
    put_i16(out + 19, in->speed_right_0p1mm_s);
    put_i16(out + 21, in->target_left_mm_s);
    put_i16(out + 23, in->target_right_mm_s);
    put_i16(out + 25, in->pwm_left);
    put_i16(out + 27, in->pwm_right);
    put_u16(out + 29, in->adc_left);
    put_u16(out + 31, in->adc_right);
    put_u16(out + 33, in->adc_barcode);
    put_u16(out + 35, in->calibrated_level_left);
    put_u16(out + 37, in->calibrated_level_right);
    put_u16(out + 39, 0xffffu);
    out[41] = in->line_levels;
    put_u32(out + 42, in->distance_mm);
    put_i16(out + 46, in->heading_cdeg);
    return true;
}

bool wpk_line(const WifiLinePacket *in, uint8_t out[WPK_LINE_BYTES]) {
    if (in == NULL || out == NULL || in->event > WPK_EVENT_UNASSIGNED ||
        !valid_letter(in->last_decoded_letter) || !valid_letter(in->pending_letter) ||
        !valid_action(in->pending_action) || !valid_action(in->executing_action) ||
        (in->calibrated_level_left > 1000u && in->calibrated_level_left != 0xffffu) ||
        (in->calibrated_level_right > 1000u && in->calibrated_level_right != 0xffffu))
        return false;
    put_header(out, WPK_SRC_LINE, WPK_LINE_PAYLOAD_BYTES, in->seq);
    put_u32(out + 3, in->uptime_ms);
    out[7] = in->event;
    put_u16(out + 8, in->adc_left);
    put_u16(out + 10, in->adc_right);
    put_u16(out + 12, in->adc_barcode);
    put_u16(out + 14, in->calibrated_level_left);
    put_u16(out + 16, in->calibrated_level_right);
    out[18] = in->last_decoded_letter;
    out[19] = in->pending_letter;
    out[20] = in->pending_action;
    out[21] = in->pending_revision;
    out[22] = in->executing_action;
    out[23] = in->executing_revision;
    return true;
}

bool wpk_nav(const WifiNavPacket *in, uint8_t out[WPK_NAV_BYTES]) {
    if (in == NULL || out == NULL || in->event > WPK_EVENT_UNASSIGNED ||
        !valid_letter(in->letter) || !valid_action(in->action))
        return false;
    put_header(out, WPK_SRC_NAV, WPK_NAV_PAYLOAD_BYTES, in->seq);
    put_u32(out + 3, in->uptime_ms);
    out[7] = in->event;
    out[8] = in->letter;
    out[9] = in->action;
    out[10] = in->map_revision;
    put_u32(out + 11, in->command_id);
    put_u32(out + 15, in->generation);
    put_u16(out + 19, in->fault_bitmap);
    out[21] = in->owner;
    out[22] = in->mode;
    out[23] = in->command_state;
    put_u16(out + 24, in->detail);
    return true;
}

bool wpk_log_chunk(uint8_t seq, uint8_t dataset, uint8_t request_id, uint16_t row_index,
                   uint16_t total_rows, uint8_t part_index, uint8_t part_count, uint8_t status,
                   const uint8_t *data, uint8_t length, uint8_t out[WIFI_PACKET_MAX],
                   uint8_t *out_len) {
    if (out == NULL || out_len == NULL || length > WPK_LOG_CHUNK_DATA_MAX || dataset > 3u ||
        total_rows > 4096u || status > WPK_LOG_STATUS_UNAVAILABLE ||
        (length != 0u && data == NULL) || (part_count == 0u) || part_index >= part_count ||
        part_count > 2u || row_index > 0x0fffu ||
        (status == WPK_LOG_STATUS_OK && row_index >= total_rows) ||
        (status != WPK_LOG_STATUS_OK && length != 0u))
        return false;
    put_header(out, WPK_SRC_LOG_CHUNK, (uint8_t)(WPK_LOG_CHUNK_HEADER_BYTES + length), seq);
    out[3] = dataset;
    out[4] = request_id;
    put_u16(out + 5, row_index);
    put_u16(out + 7, total_rows);
    out[9] = part_index;
    out[10] = part_count;
    out[11] = status;
    out[12] = length;
    out[13] = 0u;
    out[14] = 0u;
    if (length != 0u) {
        uint8_t i;
        for (i = 0u; i < length; ++i)
            out[15u + i] = data[i];
    }
    *out_len = (uint8_t)(WPK_HEADER_BYTES + WPK_LOG_CHUNK_HEADER_BYTES + length);
    return true;
}

bool wpk_summary(const WifiSummaryPacket *in, uint8_t out[WPK_SUMMARY_BYTES]) {
    if (in == NULL || out == NULL)
        return false;
    put_header(out, WPK_SRC_SUMMARY, WPK_SUMMARY_PAYLOAD_BYTES, in->seq);
    put_u32(out + 3, in->uptime_ms);
    put_u16(out + 7, in->run_id);
    out[9] = in->result;
    out[10] = in->flags;
    put_u32(out + 11, in->elapsed_ms);
    put_u32(out + 15, in->distance_mm);
    put_u16(out + 19, in->line_events);
    put_u16(out + 21, in->nav_events);
    put_u16(out + 23, in->recoveries);
    return true;
}

bool wpk_diagnostics(const WifiDiagnosticsPacket *in, uint8_t out[WPK_DIAGNOSTICS_BYTES]) {
    if (in == NULL || out == NULL || in->threshold_adc > 4095u || in->bank > 62u ||
        (in->flags & 0xf0u) != 0u)
        return false;
    put_header(out, WPK_SRC_DIAGNOSTICS, WPK_DIAGNOSTICS_PAYLOAD_BYTES, in->seq);
    put_u32(out + 3, in->uptime_ms);
    put_u16(out + 7, in->threshold_adc);
    out[9] = in->bank;
    out[10] = in->flags;
    put_u32(out + 11, in->decoded_total);
    put_u32(out + 15, in->turns);
    put_u32(out + 19, in->lefts);
    put_u32(out + 23, in->rights);
    put_u32(out + 27, in->uturns);
    put_u32(out + 31, in->bad_frames);
    put_u32(out + 35, in->unsupported);
    put_u32(out + 39, in->max_adc_gap_us);
    put_u32(out + 43, in->max_control_gap_us);
    put_u32(out + 47, in->tx_dropped);
    put_u32(out + 51, in->rx_dropped);
    put_u32(out + 55, in->invalid_left);
    put_u32(out + 59, in->invalid_right);
    return true;
}
