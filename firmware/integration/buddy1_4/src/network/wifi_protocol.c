/**
 * @file wifi_protocol.c
 * @brief Wifi protocol.
 * @details Network helpers validate and explicitly encode bounded application
 * records. Transport calls belong to the configured SDK or RTOS owner, not
 * these codecs.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "src/network/wifi_protocol.h"

static uint32_t map_slot (uint32_t word, unsigned slot)
{
    return (word >> (5u * slot)) & 31u;
}

bool wp_map_valid (uint32_t word)
{
    uint32_t seen = 0u;
    unsigned index_value;
    if ((word & ~((uint32_t)WP_MAP_MASK)) != 0u)
    {
        return false;
    }
    for (index_value = 0u; index_value < WP_MAP_SLOTS; ++index_value)
    {
        uint32_t index = map_slot(word, index_value);
        if ((index >= 26u) || ((seen & (1u << index)) != 0u))
        {
            return false;
        }
        seen |= 1u << index;
    }
    return true;
}

uint8_t wp_parse (const uint8_t * p_data_value, size_t original_len,
                  wifi_command_t * p_cmd_value)
{
    size_t expected;
    uint32_t param, value;
    if (NULL == p_cmd_value)
    {
        return WP_ACK_FORMAT;
    }
    p_cmd_value->byte0 = 0u;
    p_cmd_value->code = 0u;
    p_cmd_value->id = 0u;
    p_cmd_value->arg = 0u;
    p_cmd_value->mapping = 0u;
    if (NULL == p_data_value || 0u == original_len)
    {
        return WP_ACK_FORMAT;
    }
    p_cmd_value->byte0 = p_data_value[0];
    p_cmd_value->code = (uint8_t)(p_data_value[0] >> 4);
    if (original_len >= 2u)
    {
        p_cmd_value->id = p_data_value[1];
    }
    expected =
        (WP_CMD_MAP == p_cmd_value->code) ? WP_MAP_CMD_BYTES : WP_CMD_BYTES;
    if (((p_data_value[0] & 0x0fu) != WP_VERSION) || (original_len != expected))
    {
        return WP_ACK_FORMAT;
    }
    p_cmd_value->arg = (uint16_t)((uint16_t)p_data_value[2] |
                                  ((uint16_t)p_data_value[3] << 8));
    if (WP_CMD_MAP == p_cmd_value->code)
    {
        p_cmd_value->mapping =
            (uint32_t)p_cmd_value->arg | ((uint32_t)p_data_value[4] << 16);
        if (!wp_map_valid(p_cmd_value->mapping))
        {
            p_cmd_value->mapping = 0u;
            return WP_ACK_FORMAT;
        }
        return WP_ACK_ACCEPTED;
    }
    if (WP_CMD_STEER == p_cmd_value->code ||
        WP_CMD_DRIVE == p_cmd_value->code || WP_CMD_TURN == p_cmd_value->code)
    {
        return WP_ACK_PROHIBITED;
    }
    switch (p_cmd_value->code)
    {
    case WP_CMD_START:
    case WP_CMD_PAUSE:
    case WP_CMD_RESUME:
    case WP_CMD_STOP:
    case WP_CMD_RESET:
    case WP_CMD_ESTOP:
    case WP_CMD_PING:
        return WP_ACK_ACCEPTED;
    case WP_CMD_CONFIG:
        if (WP_CONFIG_CALIBRATION_KEY == (p_cmd_value->arg & 0xffu))
        {
            return (p_cmd_value->arg >> 8) <= 7u ? WP_ACK_ACCEPTED
                                                 : WP_ACK_FORMAT;
        }
        /* Stopped drive mode: low byte key2, high byte mode0..2. The legacy
         * telemetry1 nibble encoding0x1000/0x1001 remains unchanged. */
        if (WP_CONFIG_DRIVE_MODE_KEY == (p_cmd_value->arg & 0xffu))
        {
            return (p_cmd_value->arg >> 8) <= 2u ? WP_ACK_ACCEPTED
                                                 : WP_ACK_FORMAT;
        }
        param = (uint32_t)p_cmd_value->arg >> 12;
        value = (uint32_t)p_cmd_value->arg & 0x0fffu;
        if (param != 1u)
        {
            return WP_ACK_UNSUPPORTED;
        }
        return value <= 1u ? WP_ACK_ACCEPTED : WP_ACK_FORMAT;
    case WP_CMD_GET:
        return 0u == p_cmd_value->arg ? WP_ACK_ACCEPTED : WP_ACK_FORMAT;
    case WP_CMD_GET_LOG:
        return ((p_cmd_value->arg >> 12) <= WP_LOG_NAVIGATION)
                   ? WP_ACK_ACCEPTED
                   : WP_ACK_UNSUPPORTED;
    default:
        return WP_ACK_UNKNOWN;
    }
}

uint32_t wp_map_replace (uint32_t snapshot, uint32_t word)
{
    uint32_t revision;
    if (!wp_map_valid(word))
    {
        return snapshot;
    }
    if (((snapshot & WP_MAP_CONFIGURED) != 0u) &&
        ((snapshot & WP_MAP_MASK) == word))
    {
        return snapshot;
    }
    revision = ((snapshot >> WP_MAP_REV_SHIFT) + 1u) & 0xffu;
    return word | WP_MAP_CONFIGURED | (revision << WP_MAP_REV_SHIFT);
}

char wp_map_action (uint32_t snapshot, char letter)
{
    unsigned index;
    if (0u == (snapshot & WP_MAP_CONFIGURED) ||
        ((snapshot & ~WP_MAP_STATUS_MASK) != 0u) ||
        (!wp_map_valid(snapshot & WP_MAP_MASK)) || (letter < 'A') ||
        (letter > 'Z'))
    {
        return '-';
    }
    for (index = 0u; index < WP_MAP_SLOTS; ++index)
    {
        if (map_slot(snapshot, index) == (uint32_t)(letter - 'A'))
        {
            return "SULR"[index];
        }
    }
    return '-';
}

void wp_ack (const wifi_command_t * p_cmd_value, uint8_t result, uint8_t state,
             uint8_t out[4])
{
    if (NULL == out)
    {
        return;
    }
    out[0] = p_cmd_value != NULL ? p_cmd_value->byte0 : 0u;
    out[1] = p_cmd_value != NULL ? p_cmd_value->id : 0u;
    out[2] = result;
    out[3] = (uint8_t)(state & 7u);
}

void wp_map_frame (uint32_t snapshot, uint8_t identifier_value, uint8_t seq,
                   uint8_t out[8])
{
    uint32_t state = snapshot & WP_MAP_STATUS_MASK;
    out[0] = (uint8_t)((WP_SRC_MAP << 4) | WP_VERSION);
    out[1] = 5u;
    out[2] = seq;
    out[3] = (uint8_t)state;
    out[4] = (uint8_t)(state >> 8);
    out[5] = (uint8_t)(state >> 16);
    out[6] = (uint8_t)(state >> 24);
    out[7] = identifier_value;
}

/* End of file. */
