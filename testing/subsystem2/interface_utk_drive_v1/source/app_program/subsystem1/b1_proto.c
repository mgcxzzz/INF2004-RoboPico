/**
 * @file    b1_proto.c
 * @brief   WiFi and MQTT (Buddy 1): command checking, acknowledgements and
 *          the source-1 status frame (layouts in system/sys_tlm.h).
 *
 * Pure C: the same file runs on the Pico and in the host tests.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#include "b1_proto.h"
#include "b1_mapping.h"

#define B1_CFG_PARAM_SHIFT  (12U)
#define B1_CFG_VALUE_MASK   (0x0FFFU)
#define B1_BACKOFF_MAX_MS   (8000U)

/* ---------------------------------------------------------------------
 * Commands
 * ------------------------------------------------------------------ */

bool
b1_cmd_is_navigation (uint8_t code)
{
    return ((code == B1_CMD_STEER) || (code == B1_CMD_DRIVE) ||
            (code == B1_CMD_TURN));
}

uint8_t
b1_cmd_check (uint8_t const * p_msg, uint32_t len, b1_cmd_t * p_cmd)
{
    uint32_t param;
    uint32_t value;

    p_cmd->byte0 = 0U;
    p_cmd->code  = B1_CMD_NONE;
    p_cmd->id    = 0U;
    p_cmd->arg   = 0U;
    p_cmd->mapping = 0U;

    if (len >= 1U)
    {
        p_cmd->byte0 = p_msg[0];
        p_cmd->code  = (uint8_t)(p_msg[0] >> 4);
    }
    if (len >= 2U)
    {
        p_cmd->id = p_msg[1];
    }

    if ((len < 1U) || ((p_msg[0] & 0x0FU) != TLM_VERSION) ||
        (len != ((p_cmd->code == B1_CMD_BARCODE_MAP) ?
                  B1_MAP_CMD_BYTES : B1_CMD_BYTES)))
    {
        return B1_ACK_FORMAT;
    }

    p_cmd->arg = (uint16_t)((uint16_t)p_msg[2] | ((uint16_t)p_msg[3] << 8));

    if (p_cmd->code == B1_CMD_BARCODE_MAP)
    {
        p_cmd->mapping = (uint32_t)p_cmd->arg | ((uint32_t)p_msg[4] << 16);
        return b1_mapping_check(p_cmd->mapping) ?
               B1_ACK_ACCEPTED : B1_ACK_FORMAT;
    }

    if (b1_cmd_is_navigation(p_cmd->code))
    {
        return B1_ACK_PROHIBITED;           /* FR-WIFI-03 */
    }

    switch (p_cmd->code)
    {
        case B1_CMD_START:
        case B1_CMD_PAUSE:
        case B1_CMD_RESUME:
        case B1_CMD_STOP:
        case B1_CMD_RESET:
        case B1_CMD_ESTOP:
        case B1_CMD_PING:
            return B1_ACK_ACCEPTED;

        case B1_CMD_CONFIG:
            param = (uint32_t)p_cmd->arg >> B1_CFG_PARAM_SHIFT;
            value = (uint32_t)p_cmd->arg & B1_CFG_VALUE_MASK;
            if (param != B1_CFG_TELEMETRY)
            {
                return B1_ACK_UNSUPPORTED;
            }
            return (value <= 1U) ? B1_ACK_ACCEPTED : B1_ACK_FORMAT;

        case B1_CMD_BARCODE_GET:
            return (p_cmd->arg == 0U) ? B1_ACK_ACCEPTED : B1_ACK_FORMAT;

        default:
            return B1_ACK_UNKNOWN;
    }
}

void
b1_cmd_pack (uint8_t code, uint8_t id, uint16_t arg,
             uint8_t p_out[B1_CMD_BYTES])
{
    p_out[0] = (uint8_t)((TLM_VERSION & 0x0FU) | ((uint32_t)(code & 0x0FU) << 4));
    p_out[1] = id;
    p_out[2] = (uint8_t)(arg & 0xFFU);
    p_out[3] = (uint8_t)((arg >> 8) & 0xFFU);
}

void
b1_map_cmd_pack (uint8_t id, uint32_t mapping,
                 uint8_t p_out[B1_MAP_CMD_BYTES])
{
    p_out[0] = (uint8_t)(TLM_VERSION | (B1_CMD_BARCODE_MAP << 4));
    p_out[1] = id;
    p_out[2] = (uint8_t)mapping;
    p_out[3] = (uint8_t)(mapping >> 8);
    p_out[4] = (uint8_t)(mapping >> 16);
}

void
b1_ack_pack (b1_cmd_t const * p_cmd, uint8_t result, uint8_t state,
             uint8_t p_out[B1_ACK_BYTES])
{
    p_out[0] = p_cmd->byte0;
    p_out[1] = p_cmd->id;
    p_out[2] = result;
    p_out[3] = (uint8_t)sys_sat_u(state, B1_BITS_STATE);
}

/* ---------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

uint32_t
b1_backoff_ms (uint32_t attempts)
{
    uint32_t ms = 1000U;

    while ((attempts > 1U) && (ms < B1_BACKOFF_MAX_MS))
    {
        ms *= 2U;
        attempts--;
    }

    return (ms > B1_BACKOFF_MAX_MS) ? B1_BACKOFF_MAX_MS : ms;
}

uint8_t
b1_fill_pct (uint32_t used_bytes, uint32_t size_bytes)
{
    uint32_t pct;

    if (size_bytes == 0U)
    {
        return 0U;
    }

    pct = (used_bytes * 100U + (size_bytes / 2U)) / size_bytes;
    return (uint8_t)((pct > 100U) ? 100U : pct);
}

char const *
b1_cmd_name (uint8_t code)
{
    static char const * const names[16] =
    {
        "none", "START", "PAUSE", "RESUME", "STOP", "RESET", "ESTOP",
        "CONFIG", "PING", "BARCODE_MAP", "BARCODE_GET", "code11", "STEER", "DRIVE",
        "TURN", "code15"
    };

    return names[code & 0x0FU];
}

char const *
b1_ack_name (uint8_t result)
{
    static char const * const names[] =
    {
        "accepted", "rejected: steering/navigation is not allowed remotely",
        "rejected: unknown command", "rejected: bad format",
        "rejected: queue full, try again", "rejected: unsupported setting"
    };

    return (result <= B1_ACK_UNSUPPORTED) ? names[result] : "?";
}

char const *
b1_net_name (uint8_t net)
{
    static char const * const names[] =
    {
        "Disconnected", "Connecting", "Operational", "Reconnect wait"
    };

    return (net < (uint8_t)B1_NET_COUNT) ? names[net] : "?";
}

/* ---------------------------------------------------------------------
 * Status frame, source 1
 * ------------------------------------------------------------------ */

void
b1_tlm_pack (b1_tlm_t const * p_in, uint8_t p_frame[B1_TLM_FRAME_BYTES])
{
    uint8_t * p  = &p_frame[TLM_HDR_BYTES];
    uint32_t  w0 = 0U;
    uint32_t  w1 = 0U;
    uint32_t  w2 = 0U;

    w0 = sys_put_u(w0,  0U, B1_BITS_NET,    sys_sat_u(p_in->net, B1_BITS_NET));
    w0 = sys_put_u(w0,  2U, B1_BITS_STATE,  sys_sat_u(p_in->state, B1_BITS_STATE));
    w0 = sys_put_u(w0,  5U, B1_BITS_CMD,    sys_sat_u(p_in->last_cmd, B1_BITS_CMD));
    w0 = sys_put_u(w0,  9U, B1_BITS_RESULT, sys_sat_u(p_in->last_result,
                                                      B1_BITS_RESULT));
    w0 = sys_put_u(w0, 12U, B1_BITS_UPTIME, sys_sat_u(p_in->uptime_s,
                                                      B1_BITS_UPTIME));

    w1 = sys_put_u(w1,  0U, 16U, p_in->published);       /* wrapping */
    w1 = sys_put_u(w1, 16U, 16U, p_in->lost);            /* wrapping */

    w2 = sys_put_u(w2,  0U, 8U, p_in->accepted);         /* wrapping */
    w2 = sys_put_u(w2,  8U, 8U, p_in->rejected);
    w2 = sys_put_u(w2, 16U, 8U, p_in->connects);
    w2 = sys_put_u(w2, 24U, B1_BITS_FILL,
                   (p_in->fill_pct > 100U) ? 100U : p_in->fill_pct);

    tlm_put_header(p_frame, TLM_SRC_WIFI, B1_TLM_PAYLOAD_BYTES, p_in->seq);
    sys_put_le32(&p[0], w0);
    sys_put_le32(&p[4], w1);
    sys_put_le32(&p[8], w2);
}

bool
b1_tlm_unpack (uint8_t const * p_frame, uint32_t len, b1_tlm_t * p_out)
{
    uint8_t const * p = &p_frame[TLM_HDR_BYTES];
    uint32_t        w0;
    uint32_t        w1;
    uint32_t        w2;

    if (!tlm_check_header(p_frame, len, TLM_SRC_WIFI, B1_TLM_PAYLOAD_BYTES,
                          &p_out->seq))
    {
        return false;
    }

    w0 = sys_get_le32(&p[0]);
    w1 = sys_get_le32(&p[4]);
    w2 = sys_get_le32(&p[8]);

    p_out->net         = (uint8_t)sys_get_u(w0,  0U, B1_BITS_NET);
    p_out->state       = (uint8_t)sys_get_u(w0,  2U, B1_BITS_STATE);
    p_out->last_cmd    = (uint8_t)sys_get_u(w0,  5U, B1_BITS_CMD);
    p_out->last_result = (uint8_t)sys_get_u(w0,  9U, B1_BITS_RESULT);
    p_out->uptime_s    = (uint16_t)sys_get_u(w0, 12U, B1_BITS_UPTIME);
    p_out->published   = (uint16_t)sys_get_u(w1,  0U, 16U);
    p_out->lost        = (uint16_t)sys_get_u(w1, 16U, 16U);
    p_out->accepted    = (uint8_t)sys_get_u(w2,  0U, 8U);
    p_out->rejected    = (uint8_t)sys_get_u(w2,  8U, 8U);
    p_out->connects    = (uint8_t)sys_get_u(w2, 16U, 8U);
    p_out->fill_pct    = (uint8_t)sys_get_u(w2, 24U, B1_BITS_FILL);

    return true;
}
