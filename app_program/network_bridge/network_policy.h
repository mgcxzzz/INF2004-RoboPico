/**
 * @file network_policy.h
 * @brief Network policy.
 * @details Application tasks exchange copied, bounded command and telemetry
 * records. The kernel Wi-Fi service owns the transport; reconnect never
 * restarts motion.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_NETWORK_POLICY_H
#define RTOS_NETWORK_POLICY_H
#include "src/network/wifi_protocol.h"
enum
{
    RN_PLAN_STOP = 1,
    RN_PLAN_ESTOP = 2,
    RN_PLAN_MAP = 4,
    RN_PLAN_HEARTBEAT = 8
};
typedef struct
{
    uint8_t result;
    uint8_t flags;
} rn_command_plan_t;
static inline rn_command_plan_t
rn_bench_command (const uint8_t * p_data, uint32_t length, uint32_t received,
                  uint32_t now, uint32_t * p_mapping,
                  wifi_command_t * p_command)
{
    rn_command_plan_t plan = {.result = wp_parse(p_data, length, p_command)};
    if (WP_ACK_ACCEPTED == plan.result && (now - received > 500U))
    {
        plan.result = WP_ACK_BUSY;
    }
    if (plan.result != WP_ACK_ACCEPTED)
    {
        return plan;
    }
    switch (p_command->code)
    {
    case WP_CMD_PING:
        plan.flags = RN_PLAN_HEARTBEAT;
        break;
    case WP_CMD_MAP:
        *p_mapping = wp_map_replace(*p_mapping, p_command->mapping);
        plan.flags = RN_PLAN_MAP;
        break;
    case WP_CMD_GET:
        plan.flags = RN_PLAN_MAP;
        break;
    case WP_CMD_STOP:
    case WP_CMD_PAUSE:
        plan.flags = RN_PLAN_STOP;
        break;
    case WP_CMD_ESTOP:
        plan.flags = RN_PLAN_STOP | RN_PLAN_ESTOP;
        break;
    default:
        plan.result = WP_ACK_UNSUPPORTED;
        break;
    }
    return plan;
}
static inline bool rn_heartbeat_due (uint32_t now, uint32_t * p_last,
                                     bool forced)
{
    if (now - *p_last >= 1000U)
    {
        *p_last = now;
        return true;
    }
    return forced;
}
#endif

/* End of file. */
