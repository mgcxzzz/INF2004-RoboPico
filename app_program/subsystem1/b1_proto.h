/**
 * @file    b1_proto.h
 * @brief   WiFi and MQTT (Buddy 1): command checking, acknowledgements and
 *          the Buddy 1 status frame. Pure C, no kernel or network header,
 *          so the host tests (testing/subsystem1/host) build it as is.
 *
 * Byte layouts are in the team schema, system/sys_tlm.h (source 1 and
 * "Supervisory commands"). This file only implements them.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#ifndef B1_PROTO_H
#define B1_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#include "../system/sys_tlm.h"

/* Link state, statechart 6.1 (2 bits in the status frame). */
typedef enum
{
    B1_NET_DISCONNECTED = 0,    /* no WiFi link, or stack not ready      */
    B1_NET_CONNECTING,          /* WiFi up, MQTT connect in progress     */
    B1_NET_OPERATIONAL,         /* MQTT connected: publish and listen    */
    B1_NET_RECONNECT_WAIT,      /* lost or refused: back-off, then retry */
    B1_NET_COUNT
} b1_net_t;

/* A command as received (fields are filled as far as the length allows,
 * so even a rejected command can be acknowledged with its id). */
typedef struct
{
    uint8_t  byte0;             /* version | code << 4, echoed in the ack */
    uint8_t  code;              /* B1_CMD_*                               */
    uint8_t  id;                /* echoed in the ack                      */
    uint16_t arg;               /* B1_CMD_CONFIG: param << 12 | value     */
} b1_cmd_t;

/**
 * Check one received command (FR-WIFI-02, FR-WIFI-03).
 * @return B1_ACK_ACCEPTED, or the reason it must not be carried out.
 *         Order: format (length, version), then prohibited steering /
 *         navigation, then unknown code, then configuration parameter.
 */
uint8_t b1_cmd_check(uint8_t const * p_msg, uint32_t len, b1_cmd_t * p_cmd);

/** True for the steering / navigation codes that are always refused. */
bool b1_cmd_is_navigation(uint8_t code);

/** Build a command (used by the tests and mirrored by the dashboard). */
void b1_cmd_pack(uint8_t code, uint8_t id, uint16_t arg,
                 uint8_t p_out[B1_CMD_BYTES]);

/** Build the acknowledgement for a command. state = nav_state_t or 7. */
void b1_ack_pack(b1_cmd_t const * p_cmd, uint8_t result, uint8_t state,
                 uint8_t p_out[B1_ACK_BYTES]);

/** Reconnect back-off after n failed attempts in a row (n >= 1):
 *  1 s, 2 s, 4 s, then 8 s for every further attempt. */
uint32_t b1_backoff_ms(uint32_t attempts);

/** Queue fill in percent, 0..100. */
uint8_t b1_fill_pct(uint32_t used_bytes, uint32_t size_bytes);

char const * b1_cmd_name(uint8_t code);
char const * b1_ack_name(uint8_t result);
char const * b1_net_name(uint8_t net);

#endif /* B1_PROTO_H */
