#include "../rtos_bringup/bringup.h"
#include "network_bridge.h"
#include "network_policy.h"
#include "src/network/wifi_protocol.h"
#include "src/network/wifi_packets.h"
#include <string.h>

enum { TX_CAPACITY = 1024, ACK_CAPACITY = 256, RX_CAPACITY = 256 };
static ID tx, ack, rx;
static T_SPLOCK lock;
static volatile uint32_t ready;
static RnStatus status;
static uint32_t published, received, tx_dropped, rx_dropped;

static UINT enter(void) {
    UINT key;
    (void)ISpinLock(&lock, &key);
    return key;
}
static void leave(UINT key) {
    (void)ISpinUnlock(&lock, key);
}
static void count_drop(bool receive) {
    UINT key = enter();
    uint32_t *value = receive ? &rx_dropped : &tx_dropped;
    if (*value != UINT32_MAX)
        ++*value;
    leave(key);
}
static ID buffer(unsigned capacity, unsigned maximum) {
    T_CMBF definition = {0};
    definition.mbfatr = TA_TFIFO;
    definition.bufsz = capacity;
    definition.maxmsz = maximum;
    return tk_cre_mbf(&definition);
}
int32_t rn_init(void) {
    if ((tx = buffer(TX_CAPACITY, 70)) < E_OK)
        return tx;
    if ((ack = buffer(ACK_CAPACITY, 70)) < E_OK)
        return ack;
    if ((rx = buffer(RX_CAPACITY, 13)) < E_OK)
        return rx;
    __atomic_store_n(&ready, 1U, __ATOMIC_RELEASE);
    return E_OK;
}
bool rn_ready(void) {
    return __atomic_load_n(&ready, __ATOMIC_ACQUIRE) != 0;
}
static void put32(uint8_t *p, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i)
        p[i] = (uint8_t)(value >> (i * 8));
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
bool rn_tx_push(uint8_t topic, const uint8_t *data, uint8_t len, uint32_t now) {
    if (!rn_ready() || !data || len == 0 || len > RN_PACKET_MAX || topic > RN_HB)
        return false;
    uint8_t message[70];
    put32(message, now);
    message[4] = topic;
    message[5] = len;
    memcpy(message + 6, data, len);
    if (tk_snd_mbf(topic == RN_ACK ? ack : tx, message, len + 6, TMO_POL) == E_OK)
        return true;
    count_drop(false);
    return false;
}
bool rn_tx_take(uint8_t *topic, uint8_t data[RN_PACKET_MAX], uint8_t *len, uint32_t now) {
    if (!rn_ready())
        return false;
    /* Bound stale-message cleanup too: never block the kernel Wi-Fi service. */
    for (unsigned n = 0; n < 4; ++n) {
        uint8_t message[70];
        INT count = tk_rcv_mbf(ack, message, TMO_POL);
        if (count < 0)
            count = tk_rcv_mbf(tx, message, TMO_POL);
        if (count < 0)
            return false;
        if (count < 7 || message[5] > RN_PACKET_MAX || count != message[5] + 6 ||
            now - get32(message) > 2000U) {
            count_drop(false);
            continue;
        }
        *topic = message[4];
        *len = message[5];
        memcpy(data, message + 6, *len);
        return true;
    }
    return false;
}
bool rn_rx_push(const uint8_t *data, uint8_t copied, uint32_t original, uint32_t now) {
    if (!rn_ready() || !data || copied > RN_RX_MAX)
        return false;
    uint8_t message[13] = {0};
    put32(message, now);
    message[4] = original <= RN_RX_MAX && original == copied ? (uint8_t)original : 9;
    memcpy(message + 5, data, copied);
    if (tk_snd_mbf(rx, message, sizeof(message), TMO_POL) == E_OK) {
        UINT key = enter();
        if (received != UINT32_MAX)
            ++received;
        leave(key);
        return true;
    }
    count_drop(true);
    return false;
}
void rn_status_update(const RnStatus *value) {
    if (!rn_ready())
        return;
    UINT key = enter();
    status = *value;
    leave(key);
}
RnStatus rn_status(void) {
    UINT key = enter();
    RnStatus result = status;
    result.published = published;
    result.received = received;
    result.tx_dropped = tx_dropped;
    result.rx_dropped = rx_dropped;
    leave(key);
    return result;
}
void rn_publish_result(bool ok) {
    if (!ok) {
        count_drop(false);
        return;
    }
    UINT key = enter();
    if (published != UINT32_MAX)
        ++published;
    leave(key);
}
void rn_report(void) {
    RnStatus s = rn_status();
    tm_printf(
        (UB *)"NET BENCH motors=DISABLED configured=%u connected=%u subscribed=%u link=%d polls=%u last_ms=%u published=%u received=%u tx_drop=%u rx_drop=%u mqtt_result=%d\n",
        !!(s.flags & RN_CONFIGURED), !!(s.flags & RN_CONNECTED), !!(s.flags & RN_SUBSCRIBED),
        s.link, s.polls, s.last_poll_ms, s.published, s.received, s.tx_dropped, s.rx_dropped,
        s.mqtt_result);
    tm_printf((UB *)"NET_ADDRESS ip=%u.%u.%u.%u broker=%u.%u.%u.%u\n", s.ipv4 & 255U,
              (s.ipv4 >> 8) & 255U, (s.ipv4 >> 16) & 255U, s.ipv4 >> 24, s.broker_ipv4 & 255U,
              (s.broker_ipv4 >> 8) & 255U, (s.broker_ipv4 >> 16) & 255U, s.broker_ipv4 >> 24);
}
void rn_app_task(int32_t code, void *exinf) {
    (void)code;
    (void)exinf;
    uint32_t last_hb = board_kernel_ms(), mapping = 0, previous_connects = 0;
    uint8_t sequence = 0, accepted = 0, rejected = 0, last_command = 0, last_result = 0;
    for (;;) {
        uint32_t now = board_kernel_ms();
        RnStatus net = rn_status();
        BringupStatus motion;
        bringup_get_status(&motion);
        uint8_t state = motion.flags & BR_LATCHED ? 6 : motion.flags & BR_CLOCK_OK ? 1 : 0;
        bool force_hb = false;
        uint8_t message[13];
        for (unsigned n = 0; n < 4 && tk_rcv_mbf(rx, message, TMO_POL) == 13; ++n) {
            WifiCommand command;
            RnCommandPlan plan =
                rn_bench_command(message + 5, message[4], get32(message), now, &mapping, &command);
            uint8_t result = plan.result;
            bool send_map = (plan.flags & RN_PLAN_MAP) != 0;
            force_hb |= (plan.flags & RN_PLAN_HEARTBEAT) != 0;
            if (plan.flags & RN_PLAN_STOP)
                bringup_stop_from(BR_REASON_NETWORK, plan.flags & RN_PLAN_ESTOP ? BR_ESTOP : 0,
                                  MOTION_BUDDY1);
            if (plan.flags & RN_PLAN_ESTOP)
                state = 6;
            if (result == WP_ACK_ACCEPTED) {
                if (accepted != 255)
                    ++accepted;
            } else if (rejected != 255)
                ++rejected;
            uint8_t packet[8];
            wp_ack(&command, result, state, packet);
            (void)rn_tx_push(RN_ACK, packet, WP_ACK_BYTES, now);
            if (send_map) {
                wp_map_frame(mapping, command.id, sequence++, packet);
                (void)rn_tx_push(RN_TLM, packet, WP_MAP_FRAME_BYTES, now);
            }
            last_command = command.code;
            last_result = result;
        }
        if (net.connects != previous_connects) {
            uint8_t map[WP_MAP_FRAME_BYTES];
            wp_map_frame(mapping, 0, sequence++, map);
            (void)rn_tx_push(RN_TLM, map, sizeof(map), now);
            previous_connects = net.connects;
            force_hb = true;
        }
        if (rn_heartbeat_due(now, &last_hb, force_hb)) {
            WifiHeartbeat heartbeat = {
                .seq = sequence++,
                .net = net.flags & RN_SUBSCRIBED ? 2
                       : net.link == 3           ? 1
                                                 : 0,
                .state = state,
                .last_cmd = last_command,
                .last_result = last_result,
                .uptime_s = (uint16_t)(now / 1000U > 16383U ? 16383U : now / 1000U),
                .published = net.published > 65535U ? 65535U : (uint16_t)net.published,
                .lost = net.tx_dropped > 65535U ? 65535U : (uint16_t)net.tx_dropped,
                .accepted = accepted,
                .rejected = rejected,
                .connects = net.connects > 255U ? 255U : (uint8_t)net.connects};
            T_RMBF queue;
            if (tk_ref_mbf(tx, &queue) == E_OK)
                heartbeat.fill_pct = (uint8_t)((TX_CAPACITY - queue.frbufsz) * 100U / TX_CAPACITY);
            uint8_t packet[WPK_HEARTBEAT_BYTES];
            if (wpk_heartbeat(&heartbeat, packet))
                (void)rn_tx_push(RN_HB, packet, sizeof(packet), now);
        }
        (void)tk_dly_tsk(10);
    }
}
