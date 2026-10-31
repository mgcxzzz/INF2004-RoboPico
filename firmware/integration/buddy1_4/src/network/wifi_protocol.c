#include "src/network/wifi_protocol.h"

static uint32_t map_slot(uint32_t word, unsigned slot) {
    return (word >> (5u * slot)) & 31u;
}

bool wp_map_valid(uint32_t word) {
    uint32_t seen = 0u;
    unsigned i;
    if ((word & ~((uint32_t)WP_MAP_MASK)) != 0u)
        return false;
    for (i = 0u; i < WP_MAP_SLOTS; ++i) {
        uint32_t index = map_slot(word, i);
        if (index >= 26u || (seen & (1u << index)) != 0u)
            return false;
        seen |= 1u << index;
    }
    return true;
}

uint8_t wp_parse(const uint8_t *p, size_t original_len, WifiCommand *cmd) {
    size_t expected;
    uint32_t param, value;
    if (cmd == NULL)
        return WP_ACK_FORMAT;
    cmd->byte0 = 0u;
    cmd->code = 0u;
    cmd->id = 0u;
    cmd->arg = 0u;
    cmd->mapping = 0u;
    if (p == NULL || original_len == 0u)
        return WP_ACK_FORMAT;
    cmd->byte0 = p[0];
    cmd->code = (uint8_t)(p[0] >> 4);
    if (original_len >= 2u)
        cmd->id = p[1];
    expected = (cmd->code == WP_CMD_MAP) ? WP_MAP_CMD_BYTES : WP_CMD_BYTES;
    if ((p[0] & 0x0fu) != WP_VERSION || original_len != expected)
        return WP_ACK_FORMAT;
    cmd->arg = (uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
    if (cmd->code == WP_CMD_MAP) {
        cmd->mapping = (uint32_t)cmd->arg | ((uint32_t)p[4] << 16);
        if (!wp_map_valid(cmd->mapping)) {
            cmd->mapping = 0u;
            return WP_ACK_FORMAT;
        }
        return WP_ACK_ACCEPTED;
    }
    if (cmd->code == WP_CMD_STEER || cmd->code == WP_CMD_DRIVE || cmd->code == WP_CMD_TURN)
        return WP_ACK_PROHIBITED;
    switch (cmd->code) {
    case WP_CMD_START:
    case WP_CMD_PAUSE:
    case WP_CMD_RESUME:
    case WP_CMD_STOP:
    case WP_CMD_RESET:
    case WP_CMD_ESTOP:
    case WP_CMD_PING:
        return WP_ACK_ACCEPTED;
    case WP_CMD_CONFIG:
        param = (uint32_t)cmd->arg >> 12;
        value = (uint32_t)cmd->arg & 0x0fffu;
        if (param != 1u)
            return WP_ACK_UNSUPPORTED;
        return value <= 1u ? WP_ACK_ACCEPTED : WP_ACK_FORMAT;
    case WP_CMD_GET:
        return cmd->arg == 0u ? WP_ACK_ACCEPTED : WP_ACK_FORMAT;
    case WP_CMD_GET_LOG:
        return ((cmd->arg >> 12) <= WP_LOG_NAVIGATION) ? WP_ACK_ACCEPTED : WP_ACK_UNSUPPORTED;
    default:
        return WP_ACK_UNKNOWN;
    }
}

uint32_t wp_map_replace(uint32_t snapshot, uint32_t word) {
    uint32_t revision;
    if (!wp_map_valid(word))
        return snapshot;
    if ((snapshot & WP_MAP_CONFIGURED) != 0u && (snapshot & WP_MAP_MASK) == word)
        return snapshot;
    revision = ((snapshot >> WP_MAP_REV_SHIFT) + 1u) & 0xffu;
    return word | WP_MAP_CONFIGURED | (revision << WP_MAP_REV_SHIFT);
}

char wp_map_action(uint32_t snapshot, char letter) {
    unsigned i;
    if ((snapshot & WP_MAP_CONFIGURED) == 0u || (snapshot & ~WP_MAP_STATUS_MASK) != 0u ||
        !wp_map_valid(snapshot & WP_MAP_MASK) || letter < 'A' || letter > 'Z')
        return '-';
    for (i = 0u; i < WP_MAP_SLOTS; ++i)
        if (map_slot(snapshot, i) == (uint32_t)(letter - 'A'))
            return "SULR"[i];
    return '-';
}

void wp_ack(const WifiCommand *cmd, uint8_t result, uint8_t state, uint8_t out[4]) {
    if (out == NULL)
        return;
    out[0] = cmd != NULL ? cmd->byte0 : 0u;
    out[1] = cmd != NULL ? cmd->id : 0u;
    out[2] = result;
    out[3] = (uint8_t)(state & 7u);
}

void wp_map_frame(uint32_t snapshot, uint8_t id, uint8_t seq, uint8_t out[8]) {
    uint32_t s = snapshot & WP_MAP_STATUS_MASK;
    out[0] = (uint8_t)((WP_SRC_MAP << 4) | WP_VERSION);
    out[1] = 5u;
    out[2] = seq;
    out[3] = (uint8_t)s;
    out[4] = (uint8_t)(s >> 8);
    out[5] = (uint8_t)(s >> 16);
    out[6] = (uint8_t)(s >> 24);
    out[7] = id;
}
