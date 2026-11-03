#ifndef RTOS_NETWORK_POLICY_H
#define RTOS_NETWORK_POLICY_H
#include "src/network/wifi_protocol.h"
enum { RN_PLAN_STOP = 1, RN_PLAN_ESTOP = 2, RN_PLAN_MAP = 4, RN_PLAN_HEARTBEAT = 8 };
typedef struct {
    uint8_t result, flags;
} RnCommandPlan;
static inline RnCommandPlan rn_bench_command(const uint8_t *data, uint32_t length,
                                             uint32_t received, uint32_t now, uint32_t *mapping,
                                             WifiCommand *command) {
    RnCommandPlan plan = {.result = wp_parse(data, length, command)};
    if (plan.result == WP_ACK_ACCEPTED && now - received > 500U)
        plan.result = WP_ACK_BUSY;
    if (plan.result != WP_ACK_ACCEPTED)
        return plan;
    switch (command->code) {
    case WP_CMD_PING:
        plan.flags = RN_PLAN_HEARTBEAT;
        break;
    case WP_CMD_MAP:
        *mapping = wp_map_replace(*mapping, command->mapping);
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
static inline bool rn_heartbeat_due(uint32_t now, uint32_t *last, bool forced) {
    if (now - *last >= 1000U) {
        *last = now;
        return true;
    }
    return forced;
}
#endif
