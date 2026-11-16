/**
 * @file wifi_packets.c
 * @brief Wifi packets.
 * @details Network helpers validate and explicitly encode bounded application records.
 * Transport calls belong to the configured SDK or RTOS owner, not these codecs.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "src/network/wifi_packets.h"
#include <stddef.h>

static void put_header (uint8_t * p_out, uint8_t src, uint8_t payload,
                        uint8_t seq)
{
    p_out[0] = (uint8_t)((src << 4) | 2u);
    p_out[1] = payload;
    p_out[2] = seq;
}
static void put_u16 (uint8_t * p_data, uint16_t value_x)
{
    p_data[0] = (uint8_t)value_x;
    p_data[1] = (uint8_t)(value_x >> 8);
}
static void put_i16 (uint8_t * p_data, int16_t value_x)
{
    put_u16(p_data, (uint16_t)value_x);
}
static void put_u32 (uint8_t * p_data, uint32_t value_x)
{
    p_data[0] = (uint8_t)value_x;
    p_data[1] = (uint8_t)(value_x >> 8);
    p_data[2] = (uint8_t)(value_x >> 16);
    p_data[3] = (uint8_t)(value_x >> 24);
}
static bool valid_action (uint8_t character)
{
    return character == (uint8_t)'S' || character == (uint8_t)'U' ||
           character == (uint8_t)'L' || character == (uint8_t)'R' ||
           character == (uint8_t)'-' || character == 0u;
}
static bool valid_letter (uint8_t character)
{
    return character == 0u ||
           (character >= (uint8_t)'A' && character <= (uint8_t)'Z');
}

bool wpk_heartbeat (const wifi_heartbeat_t * p_in_value_value,
                    uint8_t out[WPK_HEARTBEAT_BYTES])
{
    uint32_t w0_value, w1_value, w2_value;
    if (p_in_value_value == NULL || out == NULL)
    {
        return false;
    }
    w0_value =
        (uint32_t)(p_in_value_value->net > 3u ? 3u : p_in_value_value->net) |
        ((uint32_t)(p_in_value_value->state > 7u ? 7u : p_in_value_value->state)
         << 2) |
        ((uint32_t)(p_in_value_value->last_cmd > 15u
                        ? 15u
                        : p_in_value_value->last_cmd)
         << 5) |
        ((uint32_t)(p_in_value_value->last_result > 7u
                        ? 7u
                        : p_in_value_value->last_result)
         << 9) |
        ((uint32_t)(p_in_value_value->uptime_s > 16383u
                        ? 16383u
                        : p_in_value_value->uptime_s)
         << 12);
    w1_value = (uint32_t)p_in_value_value->published |
               ((uint32_t)p_in_value_value->lost << 16);
    w2_value = (uint32_t)p_in_value_value->accepted |
               ((uint32_t)p_in_value_value->rejected << 8) |
               ((uint32_t)p_in_value_value->connects << 16) |
               ((uint32_t)(p_in_value_value->fill_pct > 100u
                               ? 100u
                               : p_in_value_value->fill_pct)
                << 24);
    put_header(out, WPK_SRC_HEARTBEAT, WPK_HEARTBEAT_PAYLOAD_BYTES,
               p_in_value_value->seq);
    put_u32(out + 3, w0_value);
    put_u32(out + 7, w1_value);
    put_u32(out + 11, w2_value);
    return true;
}

bool wpk_motion (const wifi_motion_packet_t * p_in_value_value,
                 uint8_t out[WPK_MOTION_BYTES])
{
    if (p_in_value_value == NULL || out == NULL ||
        (p_in_value_value->calibrated_level_left > 1000u &&
         p_in_value_value->calibrated_level_left != 0xffffu) ||
        (p_in_value_value->calibrated_level_right > 1000u &&
         p_in_value_value->calibrated_level_right != 0xffffu) ||
        p_in_value_value->heading_cdeg < -18000 ||
        p_in_value_value->heading_cdeg > 18000)
    {
        return false;
    }
    put_header(out, WPK_SRC_MOTION, WPK_MOTION_PAYLOAD_BYTES,
               p_in_value_value->seq);
    put_u32(out + 3, p_in_value_value->uptime_ms);
    put_u32(out + 7, p_in_value_value->elapsed_ms);
    put_u16(out + 11, p_in_value_value->run_id);
    out[13] = p_in_value_value->phase;
    out[14] = p_in_value_value->profile;
    out[15] = p_in_value_value->flags;
    out[16] = p_in_value_value->recovery_state;
    put_i16(out + 17, p_in_value_value->speed_left_0p1mm_s);
    put_i16(out + 19, p_in_value_value->speed_right_0p1mm_s);
    put_i16(out + 21, p_in_value_value->target_left_mm_s);
    put_i16(out + 23, p_in_value_value->target_right_mm_s);
    put_i16(out + 25, p_in_value_value->pwm_left);
    put_i16(out + 27, p_in_value_value->pwm_right);
    put_u16(out + 29, p_in_value_value->adc_left);
    put_u16(out + 31, p_in_value_value->adc_right);
    put_u16(out + 33, p_in_value_value->adc_barcode);
    put_u16(out + 35, p_in_value_value->calibrated_level_left);
    put_u16(out + 37, p_in_value_value->calibrated_level_right);
    put_u16(out + 39, 0xffffu);
    out[41] = p_in_value_value->line_levels;
    put_u32(out + 42, p_in_value_value->distance_mm);
    put_i16(out + 46, p_in_value_value->heading_cdeg);
    return true;
}

bool wpk_line (const wifi_line_packet_t * p_in_value_value,
               uint8_t out[WPK_LINE_BYTES])
{
    if (p_in_value_value == NULL || out == NULL ||
        p_in_value_value->event > WPK_EVENT_UNASSIGNED ||
        !valid_letter(p_in_value_value->last_decoded_letter) ||
        !valid_letter(p_in_value_value->pending_letter) ||
        !valid_action(p_in_value_value->pending_action) ||
        !valid_action(p_in_value_value->executing_action) ||
        (p_in_value_value->calibrated_level_left > 1000u &&
         p_in_value_value->calibrated_level_left != 0xffffu) ||
        (p_in_value_value->calibrated_level_right > 1000u &&
         p_in_value_value->calibrated_level_right != 0xffffu))
    {
        return false;
    }
    put_header(out, WPK_SRC_LINE, WPK_LINE_PAYLOAD_BYTES,
               p_in_value_value->seq);
    put_u32(out + 3, p_in_value_value->uptime_ms);
    out[7] = p_in_value_value->event;
    put_u16(out + 8, p_in_value_value->adc_left);
    put_u16(out + 10, p_in_value_value->adc_right);
    put_u16(out + 12, p_in_value_value->adc_barcode);
    put_u16(out + 14, p_in_value_value->calibrated_level_left);
    put_u16(out + 16, p_in_value_value->calibrated_level_right);
    out[18] = p_in_value_value->last_decoded_letter;
    out[19] = p_in_value_value->pending_letter;
    out[20] = p_in_value_value->pending_action;
    out[21] = p_in_value_value->pending_revision;
    out[22] = p_in_value_value->executing_action;
    out[23] = p_in_value_value->executing_revision;
    return true;
}

bool wpk_nav (const wifi_nav_packet_t * p_in_value_value,
              uint8_t out[WPK_NAV_BYTES])
{
    if (p_in_value_value == NULL || out == NULL ||
        p_in_value_value->event > WPK_EVENT_UNASSIGNED ||
        !valid_letter(p_in_value_value->letter) ||
        !valid_action(p_in_value_value->action))
    {
        return false;
    }
    put_header(out, WPK_SRC_NAV, WPK_NAV_PAYLOAD_BYTES, p_in_value_value->seq);
    put_u32(out + 3, p_in_value_value->uptime_ms);
    out[7] = p_in_value_value->event;
    out[8] = p_in_value_value->letter;
    out[9] = p_in_value_value->action;
    out[10] = p_in_value_value->map_revision;
    put_u32(out + 11, p_in_value_value->command_id);
    put_u32(out + 15, p_in_value_value->generation);
    put_u16(out + 19, p_in_value_value->fault_bitmap);
    out[21] = p_in_value_value->owner;
    out[22] = p_in_value_value->mode;
    out[23] = p_in_value_value->command_state;
    put_u16(out + 24, p_in_value_value->detail);
    return true;
}

bool wpk_log_chunk (uint8_t seq, uint8_t dataset, uint8_t request_id,
                    uint16_t row_index, uint16_t total_rows, uint8_t part_index,
                    uint8_t part_count, uint8_t status,
                    const uint8_t * p_data_value, uint8_t length,
                    uint8_t out[WIFI_PACKET_MAX], uint8_t * p_out_len_value)
{
    if (out == NULL || p_out_len_value == NULL ||
        length > WPK_LOG_CHUNK_DATA_MAX || dataset > 3u || total_rows > 4096u ||
        status > WPK_LOG_STATUS_UNAVAILABLE ||
        (length != 0u && p_data_value == NULL) || (part_count == 0u) ||
        part_index >= part_count || part_count > 2u || row_index > 0x0fffu ||
        (status == WPK_LOG_STATUS_OK && row_index >= total_rows) ||
        (status != WPK_LOG_STATUS_OK && length != 0u))
    {
        return false;
    }
    put_header(out, WPK_SRC_LOG_CHUNK,
               (uint8_t)(WPK_LOG_CHUNK_HEADER_BYTES + length), seq);
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
    if (length != 0u)
    {
        uint8_t index;
        for (index = 0u; index < length; ++index)
        {
            out[15u + index] = p_data_value[index];
        }
    }
    *p_out_len_value =
        (uint8_t)(WPK_HEADER_BYTES + WPK_LOG_CHUNK_HEADER_BYTES + length);
    return true;
}

bool wpk_summary (const wifi_summary_packet_t * p_in_value_value,
                  uint8_t out[WPK_SUMMARY_BYTES])
{
    if (p_in_value_value == NULL || out == NULL)
    {
        return false;
    }
    put_header(out, WPK_SRC_SUMMARY, WPK_SUMMARY_PAYLOAD_BYTES,
               p_in_value_value->seq);
    put_u32(out + 3, p_in_value_value->uptime_ms);
    put_u16(out + 7, p_in_value_value->run_id);
    out[9] = p_in_value_value->result;
    out[10] = p_in_value_value->flags;
    put_u32(out + 11, p_in_value_value->elapsed_ms);
    put_u32(out + 15, p_in_value_value->distance_mm);
    put_u16(out + 19, p_in_value_value->line_events);
    put_u16(out + 21, p_in_value_value->nav_events);
    put_u16(out + 23, p_in_value_value->recoveries);
    return true;
}

bool wpk_diagnostics (const wifi_diagnostics_packet_t * p_in_value_value,
                      uint8_t out[WPK_DIAGNOSTICS_BYTES])
{
    if (p_in_value_value == NULL || out == NULL ||
        p_in_value_value->threshold_adc > 4095u ||
        p_in_value_value->bank > 62u || (p_in_value_value->flags & 0xf0u) != 0u)
    {
        return false;
    }
    put_header(out, WPK_SRC_DIAGNOSTICS, WPK_DIAGNOSTICS_PAYLOAD_BYTES,
               p_in_value_value->seq);
    put_u32(out + 3, p_in_value_value->uptime_ms);
    put_u16(out + 7, p_in_value_value->threshold_adc);
    out[9] = p_in_value_value->bank;
    out[10] = p_in_value_value->flags;
    put_u32(out + 11, p_in_value_value->decoded_total);
    put_u32(out + 15, p_in_value_value->turns);
    put_u32(out + 19, p_in_value_value->lefts);
    put_u32(out + 23, p_in_value_value->rights);
    put_u32(out + 27, p_in_value_value->uturns);
    put_u32(out + 31, p_in_value_value->bad_frames);
    put_u32(out + 35, p_in_value_value->unsupported);
    put_u32(out + 39, p_in_value_value->max_adc_gap_us);
    put_u32(out + 43, p_in_value_value->max_control_gap_us);
    put_u32(out + 47, p_in_value_value->tx_dropped);
    put_u32(out + 51, p_in_value_value->rx_dropped);
    put_u32(out + 55, p_in_value_value->invalid_left);
    put_u32(out + 59, p_in_value_value->invalid_right);
    return true;
}

/* End of file. */
