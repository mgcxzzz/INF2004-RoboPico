#include "../rtos_bringup/bringup.h"
#include "network_bridge.h"
#include "network_policy.h"
#include "connected_policy.h"
#ifdef APP_RTOS_LINE
#include "../line_navigation/line_service.h"
#endif
#include "src/network/wifi_protocol.h"
#include "src/network/wifi_packets.h"
#include <string.h>

enum { TX_CAPACITY = 1024, ACK_CAPACITY = 256, RX_CAPACITY = 256 };
static ID tx, ack, rx, hb;
static uint32_t barrier, urgent;
static uint8_t accepted, rejected, last_command, last_result;
static volatile uint32_t app_last_ms;
uint32_t rn_app_last_ms(void) {
    return __atomic_load_n(&app_last_ms, __ATOMIC_ACQUIRE);
}
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
    if ((hb = buffer(128, 70)) < E_OK)
        return hb;
#if defined(APP_RTOS_LINE)
    int32_t drive = rn_drive_init();
    if (drive < E_OK)
        return drive;
#endif
    if ((rx = buffer(RX_CAPACITY, RN_RX_BYTES)) < E_OK)
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
    if (tk_snd_mbf(topic == RN_ACK  ? ack
                   : topic == RN_HB ? hb
                                    : tx,
                   message, len + 6, TMO_POL) == E_OK)
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
            count = tk_rcv_mbf(hb, message, TMO_POL);
        if (count < 0)
            count = tk_rcv_mbf(tx, message, TMO_POL);
        if (count < 0)
            return false;
        if (count < 7 || message[5] > RN_PACKET_MAX || count != message[5] + 6 ||
            now - get32(message) > (message[4] == RN_TLM ? 250U : 2000U)) {
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
    uint8_t message[RN_RX_BYTES] = {0};
    WifiCommand command;
    bool valid = original == copied && wp_parse(data, original, &command) == WP_ACK_ACCEPTED;
    UINT stamp_key = enter();
    if (valid && (command.code == WP_CMD_STOP || command.code == WP_CMD_ESTOP ||
                  command.code == WP_CMD_PAUSE)) {
        ++barrier;
        urgent |= command.code == WP_CMD_ESTOP  ? RN_URGENT_ESTOP
                  : command.code == WP_CMD_STOP ? RN_URGENT_STOP
                                                : RN_URGENT_PAUSE;
    }
    put32(message + 13, barrier);
    put32(message + 17, status.connects);
    put32(message + 21, (uint32_t)board_time_us());
    leave(stamp_key);
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
void rn_command_result(uint8_t code, uint8_t result) {
    UINT key = enter();
    if (result == WP_ACK_ACCEPTED) {
        if (accepted != 255)
            ++accepted;
    } else if (rejected != 255)
        ++rejected;
    last_command = code;
    last_result = result;
    leave(key);
}
uint32_t rn_barrier(void) {
    UINT key = enter();
    uint32_t value = barrier;
    leave(key);
    return value;
}
unsigned rn_take_urgent(void) {
    UINT key = enter();
    unsigned value = urgent & (RN_URGENT_STOP | RN_URGENT_ESTOP);
    urgent &= ~(RN_URGENT_STOP | RN_URGENT_ESTOP);
    leave(key);
    return value;
}
bool rn_take_pause(void) {
    UINT key = enter();
    bool value = (urgent & RN_URGENT_PAUSE) != 0;
    urgent &= ~RN_URGENT_PAUSE;
    leave(key);
    return value;
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
#ifdef APP_RTOS_LINE
    const char *mode = "USB_SUPERVISED";
#else
    const char *mode = "DISABLED";
#endif
    tm_printf(
        (UB *)"NET motors=%s configured=%u connected=%u subscribed=%u link=%d polls=%u last_ms=%u published=%u received=%u tx_drop=%u rx_drop=%u mqtt_result=%d\n",
        mode, !!(s.flags & RN_CONFIGURED), !!(s.flags & RN_CONNECTED), !!(s.flags & RN_SUBSCRIBED),
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
    uint8_t sequence = 0, map_sequence = 0;
    for (;;) {
        uint32_t now = board_kernel_ms();
        __atomic_store_n(&app_last_ms, now, __ATOMIC_RELEASE);
        RnStatus net = rn_status();
        BringupStatus motion;
        bringup_get_status(&motion);
        uint8_t state = rn_drive_state(motion.flags, false, false, motion.reason);
#ifdef APP_RTOS_LINE
        state = rn_drive_state(motion.flags, rn_drive_paused(), rtos_line_pending(), motion.reason);
        rn_drive_telemetry(now);
#endif
        bool force_hb = false;
        uint8_t message[RN_RX_BYTES];
        for (unsigned n = 0; n < 4 && tk_rcv_mbf(rx, message, TMO_POL) == RN_RX_BYTES; ++n) {
            WifiCommand command;
#ifdef APP_RTOS_LINE
            uint8_t parsed = wp_parse(message + 5, message[4], &command);
            if (!parsed && (command.code == WP_CMD_START || command.code == WP_CMD_PAUSE ||
                            command.code == WP_CMD_RESUME || command.code == WP_CMD_RESET)) {
                if (rn_drive_enqueue(message))
                    continue; /* ACK follows actual owner decision. */
                uint8_t reply[4];
                wp_ack(&command, WP_ACK_BUSY, state, reply);
                rn_command_result(command.code, WP_ACK_BUSY);
                (void)rn_tx_push(RN_ACK, reply, sizeof(reply), now);
                continue;
            }
#endif
            RnCommandPlan plan =
                rn_bench_command(message + 5, message[4], get32(message), now, &mapping, &command);
#ifdef APP_RTOS_LINE
            if (!parsed && command.code == WP_CMD_CONFIG && now - get32(message) <= 500U) {
                rn_drive_set_telemetry((command.arg & 4095U) != 0);
                plan.result = WP_ACK_ACCEPTED;
            }
#endif
            uint8_t result = plan.result;
            bool send_map = (plan.flags & RN_PLAN_MAP) != 0;
            force_hb |= (plan.flags & RN_PLAN_HEARTBEAT) != 0;
            if (plan.flags & RN_PLAN_STOP)
                bringup_stop_from(BR_REASON_NETWORK, plan.flags & RN_PLAN_ESTOP ? BR_ESTOP : 0,
                                  MOTION_BUDDY1);
            if (plan.flags & RN_PLAN_ESTOP)
                state = 5;
            rn_command_result(command.code, result);
            uint8_t packet[8];
            wp_ack(&command, result, state, packet);
            (void)rn_tx_push(RN_ACK, packet, WP_ACK_BYTES, now);
            if (send_map) {
                wp_map_frame(mapping, command.id, map_sequence++, packet);
                (void)rn_tx_push(RN_TLM, packet, WP_MAP_FRAME_BYTES, now);
            }
        }
        if (net.connects != previous_connects) {
            uint8_t map[WP_MAP_FRAME_BYTES];
            wp_map_frame(mapping, 0, map_sequence++, map);
            (void)rn_tx_push(RN_TLM, map, sizeof(map), now);
            previous_connects = net.connects;
            force_hb = true;
        }
        if (rn_heartbeat_due(now, &last_hb, force_hb)) {
            /* A Stop/Reset may have executed in this pass. Advertise the
             * current supervisor state, not the snapshot before dispatch. */
            bringup_get_status(&motion);
            state = rn_drive_state(motion.flags, false, false, motion.reason);
#ifdef APP_RTOS_LINE
            state =
                rn_drive_state(motion.flags, rn_drive_paused(), rtos_line_pending(), motion.reason);
#endif
            UINT key = enter();
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
            leave(key);
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
