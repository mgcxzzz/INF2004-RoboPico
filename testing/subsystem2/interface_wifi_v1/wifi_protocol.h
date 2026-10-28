#ifndef WIFI_PROTOCOL_H
#define WIFI_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Schema v2 command and acknowledgement values, compatible with Buddy 1. */
enum {
    WP_VERSION = 2u,
    WP_STOP_REASON = 49u, WP_ESTOP_REASON = 50u,
    WP_CMD_START = 1u, WP_CMD_PAUSE = 2u, WP_CMD_RESUME = 3u,
    WP_CMD_STOP = 4u, WP_CMD_RESET = 5u, WP_CMD_ESTOP = 6u,
    WP_CMD_CONFIG = 7u, WP_CMD_PING = 8u, WP_CMD_MAP = 9u,
    WP_CMD_GET = 10u, WP_CMD_GET_LOG = 11u,
    WP_CMD_STEER = 12u, WP_CMD_DRIVE = 13u, WP_CMD_TURN = 14u,
    WP_ACK_ACCEPTED = 0u, WP_ACK_PROHIBITED = 1u, WP_ACK_UNKNOWN = 2u,
    WP_ACK_FORMAT = 3u, WP_ACK_BUSY = 4u, WP_ACK_UNSUPPORTED = 5u,
    WP_MAP_SLOTS = 4u, WP_MAP_MASK = 0x000fffffu,
    WP_MAP_CONFIGURED = 1u << 20,
    WP_MAP_REV_SHIFT = 21u,
    WP_MAP_STATUS_MASK = 0x1fffffffu,
    WP_CMD_BYTES = 4u, WP_MAP_CMD_BYTES = 5u,
    WP_ACK_BYTES = 4u, WP_MAP_FRAME_BYTES = 8u,
    WP_SRC_WIFI = 1u, WP_SRC_MAP = 8u,
    WP_HEADER_BYTES = 3u
};

/* GET_LOG arg: high nibble selects 0=motion, 1=diagnostic, 2=raw,
 * 3=navigation; low 12 bits are a row index. The transport/root enforces
 * stopped-only access. */
enum { WP_LOG_MOTION = 0u, WP_LOG_DIAGNOSTIC = 1u,
       WP_LOG_RAW = 2u, WP_LOG_NAVIGATION = 3u };

typedef struct { uint8_t byte0, code, id; uint16_t arg; uint32_t mapping; } WifiCommand;

uint8_t wp_parse(const uint8_t *p, size_t original_len, WifiCommand *cmd);
bool wp_map_valid(uint32_t word);
uint32_t wp_map_replace(uint32_t snapshot, uint32_t word);
char wp_map_action(uint32_t snapshot, char letter);
void wp_ack(const WifiCommand *cmd, uint8_t result, uint8_t state, uint8_t out[4]);
void wp_map_frame(uint32_t snapshot, uint8_t id, uint8_t seq, uint8_t out[8]);

#endif
