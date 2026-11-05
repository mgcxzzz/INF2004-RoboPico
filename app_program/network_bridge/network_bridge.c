#include "../rtos_bringup/bringup.h"
#include "network_bridge.h"
#include "network_policy.h"
#include "connected_policy.h"
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
#include "../imu_navigation/imu_service.h"
#endif
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
/* Source9 motion and11 navigation describe the present, not queued history.
 * Other telemetry (including mapping replies), ACKs and heartbeat retain
 * their own MBFs. Every slot is copied under the short bridge lock. */
typedef struct {
    uint32_t ms;
    uint8_t data[RN_PACKET_MAX];
    uint8_t len : 7, ready : 1;
} LatestTelemetry;
static LatestTelemetry latest[2];
static unsigned next_latest;
static uint32_t tx_coalesced;
static uint32_t mapping_snapshot;
static uint8_t source_sequence[16];
#ifdef APP_RTOS_IMU
static struct {
    uint32_t samples, radio_us, timers_us, mqtt_us;
    uint32_t max_radio_us, max_timers_us, max_mqtt_us, max_total_us;
} service_timing;
#endif

static UINT enter(void) {
    UINT key;
    (void)ISpinLock(&lock, &key);
    return key;
}
static void leave(UINT key) {
    (void)ISpinUnlock(&lock, key);
}
#ifdef APP_RTOS_IMU
void rn_service_sample(uint32_t radio_us, uint32_t timers_us, uint32_t mqtt_us) {
    UINT key = enter();
    ++service_timing.samples;
    service_timing.radio_us = radio_us;
    service_timing.timers_us = timers_us;
    service_timing.mqtt_us = mqtt_us;
    if (radio_us > service_timing.max_radio_us)
        service_timing.max_radio_us = radio_us;
    if (timers_us > service_timing.max_timers_us)
        service_timing.max_timers_us = timers_us;
    if (mqtt_us > service_timing.max_mqtt_us)
        service_timing.max_mqtt_us = mqtt_us;
    uint32_t total = radio_us + timers_us + mqtt_us;
    if (total > service_timing.max_total_us)
        service_timing.max_total_us = total;
    leave(key);
}
#endif
uint32_t rn_mapping_snapshot(void) {
    UINT key = enter();
    uint32_t value = mapping_snapshot;
    leave(key);
    return value;
}
uint8_t rn_next_sequence(unsigned source) {
    if (source >= 16)
        return 0;
    UINT key = enter();
    uint8_t value = source_sequence[source]++;
    leave(key);
    return value;
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
    if (topic == RN_TLM && ((data[0] >> 4) == 9 ||
                            ((data[0] >> 4) == 11 && len >= 8 && data[7] == WPK_EVENT_STATUS))) {
        unsigned index = (data[0] >> 4) == 9 ? 0 : 1;
        UINT key = enter();
        if (latest[index].ready && tx_coalesced != UINT32_MAX)
            ++tx_coalesced;
        memcpy(latest[index].data, data, len);
        latest[index].len = len;
        latest[index].ms = now;
        latest[index].ready = true;
        leave(key);
        return true;
    }
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
    return rn_tx_take_budget(topic, data, len, now, true);
}
bool rn_tx_take_budget(uint8_t *topic, uint8_t data[RN_PACKET_MAX], uint8_t *len, uint32_t now,
                       bool telemetry) {
    if (!rn_ready())
        return false;
    /* Bound stale-message cleanup too: never block the kernel Wi-Fi service. */
    for (unsigned n = 0; n < 4; ++n) {
        uint8_t message[70];
        INT count = tk_rcv_mbf(ack, message, TMO_POL);
        if (count < 0)
            count = tk_rcv_mbf(hb, message, TMO_POL);
        if (count < 0 && telemetry)
            count = tk_rcv_mbf(tx, message, TMO_POL);
        if (count < 0)
            break;
        if (count < 7 || message[5] > RN_PACKET_MAX || count != message[5] + 6 ||
            now - get32(message) > (message[4] == RN_TLM ? 250U : 2000U)) {
            count_drop(false);
            if (n == 3)
                return false; /* Bound cleanup without bypassing remaining ACKs. */
            continue;
        }
        *topic = message[4];
        *len = message[5];
        memcpy(data, message + 6, *len);
        return true;
    }
    if (telemetry) {
        for (unsigned n = 0; n < 2; ++n) {
            UINT key = enter();
            unsigned index = next_latest;
            next_latest ^= 1;
            LatestTelemetry *slot = &latest[index];
            bool valid = slot->ready && now - slot->ms <= 250U;
            bool stale = slot->ready && !valid;
            if (valid) {
                *topic = RN_TLM;
                *len = slot->len;
                memcpy(data, slot->data, *len);
            }
            slot->ready = false;
            leave(key);
            if (stale)
                count_drop(false);
            if (valid)
                return true;
        }
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
    result.tx_coalesced = tx_coalesced;
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
    tm_printf(
        (UB *)"NET_TX coalesced=%u mem=%u errors=%u deferred=%u result=%d free_req=%u ring_used=%u tcp_free=%u nagle_off=%u admit_ms=%u complete_ms=%u\n",
        s.tx_coalesced, s.publish_mem, s.publish_error, s.telemetry_wait, s.publish_result,
        s.request_free, s.ring_used, s.tcp_free, s.nagle_off, s.last_admit_ms, s.last_complete_ms);
    tm_printf(
        (UB *)"NET_MEMORY heap_used=%u heap_max=%u heap_capacity=%u heap_errors=%u tcp_queue=%u tcp_retries=%u tcp_rto_ms=%u oversize=0\n",
        s.heap_used, s.heap_max, s.heap_capacity, s.heap_errors, s.tcp_queue, s.tcp_retries,
        s.tcp_rto_ms);
#ifdef APP_RTOS_IMU
    UINT key = enter();
    uint32_t sample[] = {service_timing.samples,      service_timing.radio_us,
                         service_timing.timers_us,    service_timing.mqtt_us,
                         service_timing.max_radio_us, service_timing.max_timers_us,
                         service_timing.max_mqtt_us,  service_timing.max_total_us};
    leave(key);
    tm_printf(
        (UB *)"NET_SERVICE core=0 samples=%u radio_us=%u timers_us=%u mqtt_us=%u max_radio_us=%u max_timers_us=%u max_mqtt_us=%u max_total_us=%u\n",
        sample[0], sample[1], sample[2], sample[3], sample[4], sample[5], sample[6], sample[7]);
#endif
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
                            command.code == WP_CMD_RESUME || command.code == WP_CMD_RESET
#ifdef APP_RTOS_RELEASE
                            || command.code == WP_CMD_GET_LOG
#endif
                            )) {
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
            UINT map_key = enter();
            mapping_snapshot = mapping;
            leave(map_key);
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
#ifdef APP_RTOS_RELEASE
        static uint32_t release_last;
        if (now - release_last >= 1000U) {
            release_last = now;
            uint8_t info[15];
            rtos_line_release_info(rn_next_sequence(15), info);
            (void)rn_tx_push(RN_TLM, info, sizeof(info), now);
#ifdef APP_RTOS_IMU
            if (rn_drive_telemetry_enabled()) {
                uint8_t diagnostics[63];
                ri_release_diagnostics(rn_next_sequence(15), now, diagnostics);
                (void)rn_tx_push(RN_TLM, diagnostics, sizeof(diagnostics), now);
            }
#endif
        }
#endif
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
