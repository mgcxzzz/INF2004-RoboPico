/* Production bridge/control with deterministic kernel MBFs and hardware stubs.
 * Coordinator/API/executor are real. This is not a timing/hardware test. */
#define APP_RTOS_LINE 1
#define APP_RTOS_NETWORK 1
#define RTOS_BRINGUP_H
#define RTOS_LINE_SERVICE_H
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "app_program/rtos_bringup/bringup_policy.h"
#include "firmware/integration/buddy1_4/src/motion/motion_port.h"
#include "firmware/integration/buddy1_4/src/network/wifi_packets.h"
typedef int ID;
typedef unsigned UINT;
typedef unsigned char UB;
typedef struct {
    unsigned unused;
} T_SPLOCK;
typedef struct {
    int mbfatr, bufsz, maxmsz;
} T_CMBF;
typedef struct {
    int frbufsz;
} T_RMBF;
typedef int INT;
enum { E_OK = 0, TA_TFIFO = 0, TMO_POL = 0 };
typedef struct {
    unsigned flags, reason;
} BringupStatus;
static BringupStatus hardware;
static uint32_t clock_ms = 10000, last_cancel;
static bool pending_start;
static unsigned starts, resumes, offs;
static int ISpinLock(T_SPLOCK *s, UINT *key) {
    (void)s;
    *key = 0;
    return 0;
}
static int ISpinUnlock(T_SPLOCK *s, UINT key) {
    (void)s;
    (void)key;
    return 0;
}
static void tm_printf(UB *fmt, ...) {
    (void)fmt;
}
static void tk_dly_tsk(unsigned ms) {
    (void)ms;
}
static struct {
    unsigned size, limit, used, head, count;
    uint8_t data[64][70], len[64];
} buffers[8];
static unsigned buffer_count;
static int tk_cre_mbf(T_CMBF *d) {
    unsigned id = ++buffer_count;
    assert(id < 8);
    buffers[id].size = d->bufsz;
    buffers[id].limit = d->maxmsz;
    return (int)id;
}
static int tk_snd_mbf(ID id, const void *data, int len, int timeout) {
    (void)timeout;
    if ((unsigned)len > buffers[id].limit ||
        buffers[id].used + (unsigned)len + 4 > buffers[id].size)
        return -1;
    unsigned at = (buffers[id].head + buffers[id].count) % 64;
    memcpy(buffers[id].data[at], data, (size_t)len);
    buffers[id].len[at] = (uint8_t)len;
    buffers[id].used += (unsigned)len + 4;
    ++buffers[id].count;
    return 0;
}
static int tk_rcv_mbf(ID id, void *data, int timeout) {
    (void)timeout;
    if (!buffers[id].count)
        return -1;
    unsigned at = buffers[id].head, len = buffers[id].len[at];
    memcpy(data, buffers[id].data[at], len);
    buffers[id].head = (at + 1) % 64;
    --buffers[id].count;
    buffers[id].used -= len + 4;
    return (int)len;
}
static int tk_ref_mbf(ID id, T_RMBF *out) {
    out->frbufsz = buffers[id].size - buffers[id].used;
    return 0;
}
static uint32_t board_kernel_ms(void) {
    return clock_ms;
}
static uint64_t board_time_us(void) {
    return (uint64_t)clock_ms * 1000;
}
static unsigned bringup_irq_stop;
static unsigned tm_usb_state(void) {
#ifdef APP_RTOS_RELEASE
    return 0; /* Exercise actual Resume with a powerbank/no terminal. */
#else
    return 3;
#endif
}
static bool board_button_pressed(unsigned p) {
    (void)p;
    return false;
}
static void bringup_get_status(BringupStatus *out) {
    *out = hardware;
}
static bool bringup_can_start(void) {
    return !(hardware.flags & (BR_ACTIVE | BR_LATCHED));
}
static bool bringup_clear_latch(void) {
    if (hardware.flags & BR_ACTIVE)
        return false;
    hardware.flags &= ~BR_LATCHED;
    return true;
}
static void bringup_zero_status(void) {}
static void rtos_line_cancel(void) {
    last_cancel = (uint32_t)board_time_us();
    pending_start = false;
}
static bool rtos_line_request_fresh(uint32_t received) {
    return br_command_after_stop((uint32_t)board_time_us(), received, last_cancel,
                                 last_cancel != 0);
}
static bool rtos_line_request_start(uint32_t received) {
    if (pending_start || !rtos_line_request_fresh(received))
        return false;
    pending_start = true;
    ++starts;
    return true;
}
static bool rtos_line_pending(void) {
    return pending_start;
}
static bool rtos_line_ready(void) {
    return true;
}
static void rtos_line_resume(void) {
    ++resumes;
}
static void rtos_line_feedback(MotionFeedback *f, const void *d) {
    (void)d;
    f->line_level[0] = 500;
}
static void rtos_line_telemetry(WifiMotionPacket *p) {
    *p = (WifiMotionPacket){0};
}
static void bringup_stop_from(uint8_t reason, uint16_t latch, MotionOwner who);
#ifdef APP_RTOS_RELEASE
#include "src/network/wifi_protocol.h"
static void rtos_line_release_info(uint8_t seq, uint8_t out[15]) { (void)seq;memset(out,0,15); }
static uint8_t rtos_line_log_reply(const WifiCommand *cmd) { (void)cmd;return hardware.flags&BR_ACTIVE?WP_ACK_BUSY:WP_ACK_ACCEPTED; }
#endif
#include "app_program/network_bridge/network_bridge.c"
#include "app_program/network_bridge/connected_control.c"
#include "app_program/network_bridge/delivery_policy.h"
static void bringup_stop_from(uint8_t reason, uint16_t latch, MotionOwner who) {
    hardware.flags = (hardware.flags & ~BR_ACTIVE) | latch;
    hardware.reason = reason;
    rtos_line_cancel();
    rn_drive_cancel();
    (void)motion_stop(who);
    motion_session_end(reason);
}
static uint64_t api_now(void *u) {
    (void)u;
    return board_time_us();
}
static void api_off(void *u) {
    (void)u;
    ++offs;
}
static void online(void) {
    RnStatus s = {.link = 3, .flags = 7, .polls = 1, .last_poll_ms = clock_ms, .last_complete_ms = clock_ms, .connects = 1};
    rn_status_update(&s);
    app_last_ms = clock_ms;
}
static void request(unsigned code) {
    uint8_t data[4] = {(uint8_t)((code << 4) | 2), 17, 0, 0}, msg[RN_RX_BYTES];
    assert(rn_rx_push(data, 4, 4, clock_ms));
    assert(tk_rcv_mbf(rx, msg, 0) == RN_RX_BYTES);
    assert(rn_drive_enqueue(msg));
}
int main(void) {
    assert(rn_init() == 0);
    assert(rn_live_command(true, 0x30));
    assert(rn_live_command(true, 0x38));
    assert(!rn_live_command(true, 0x31));
    assert(!rn_live_command(true, 0x39));
    assert(!rn_live_command(false, 0x30));
    online();
    assert(rn_drive_state(BR_CLOCK_OK, false, false, BR_REASON_BOOT) == 0);
    assert(rn_drive_state(BR_ACTIVE, false, false, 0) == 1);
    assert(rn_drive_state(BR_ACTIVE, true, false, 0) == 2);
    assert(rn_drive_state(BR_ESTOP, false, false, 0) == 5);
    assert(rn_drive_state(BR_ESTOP, false, false, BR_REASON_ADC) == 6);
    assert(rn_drive_state(BR_ENCODER, false, false, 0) == 6);
    assert(rn_drive_state(0, false, false, BR_REASON_NETWORK) == 3);
    RnStatus s = rn_status();
    assert(rn_link_usable(&s, clock_ms + 250));
    assert(!rn_link_usable(&s, clock_ms + 251));
    assert(rn_request_current(10, UINT32_MAX - 4, 2, 2, 1, 1));
    assert(!rn_request_current(1000, 0, 2, 2, 1, 1));
    assert(!rn_request_current(10, 0, 1, 2, 1, 1));
    assert(!rn_request_current(10, 0, 2, 2, 1, 2));
    uint8_t ping[4] = {0x82, 0, 0, 0}, stop[4] = {0x42, 1, 0, 0}, bad[8] = {0x62, 1, 0, 0};
    while (rn_rx_push(ping, 4, 4, clock_ms)) {
    }
    uint32_t before = rn_barrier();
    assert(!rn_rx_push(stop, 4, 4, clock_ms));
    assert(rn_barrier() == before + 1);
    assert(rn_take_urgent() == RN_URGENT_STOP);
    assert(!rn_rx_push(bad, 8, 9, clock_ms));
    assert(rn_take_urgent() == 0); /* No prefix execution. */
    uint8_t msg[RN_RX_BYTES];
    while (tk_rcv_mbf(rx, msg, 0) > 0) {
    }
    uint8_t value = 1, topic, len, payload[64];
    assert(rn_tx_push(RN_TLM, &value, 1, clock_ms));
    assert(rn_tx_push(RN_HB, &value, 1, clock_ms));
    assert(rn_tx_push(RN_ACK, &value, 1, clock_ms));
    assert(rn_tx_take(&topic, payload, &len, clock_ms) && topic == RN_ACK);
    assert(rn_tx_take(&topic, payload, &len, clock_ms) && topic == RN_HB);
    assert(!rn_tx_take(&topic, payload, &len, clock_ms + 251));
    /* Congested stream keeps only the newest copied sample per live source.
     * With no stream budget, ACK/HB still pass; map replies are never merged. */
    uint32_t coalesced = rn_status().tx_coalesced;
    uint8_t live_motion[2] = {0x92, 0}, live_nav[WPK_NAV_BYTES] = {0xb2, 23};
    for (unsigned n = 0; n < 100; ++n) {
        live_motion[1] = live_nav[2] = (uint8_t)n;
        assert(rn_tx_push(RN_TLM, live_motion, 2, clock_ms));
        assert(rn_tx_push(RN_TLM, live_nav, sizeof(live_nav), clock_ms));
    }
    assert(rn_status().tx_coalesced - coalesced == 198);
    live_motion[1] = 0; /* Buffer copy owns its bytes after caller returns. */
    uint8_t map_reply = 0x82;
    assert(rn_tx_push(RN_TLM, &map_reply, 1, clock_ms));
    assert(rn_tx_push(RN_HB, &value, 1, clock_ms));
    assert(rn_tx_push(RN_ACK, &value, 1, clock_ms));
    assert(rn_tx_take_budget(&topic, payload, &len, clock_ms, false) && topic == RN_ACK);
    assert(rn_tx_take_budget(&topic, payload, &len, clock_ms, false) && topic == RN_HB);
    assert(!rn_tx_take_budget(&topic, payload, &len, clock_ms, false));
    assert(rn_tx_take(&topic, payload, &len, clock_ms) && payload[0] == map_reply);
    unsigned seen_sources = 0;
    for (unsigned n = 0; n < 2; ++n) {
        assert(rn_tx_take(&topic, payload, &len, clock_ms));
        assert(topic == RN_TLM && (payload[0] == 0x92 ? len == 2 && payload[1] == 99
                                                      : len == WPK_NAV_BYTES && payload[2] == 99));
        seen_sources |= payload[0] == 0x92 ? 1 : payload[0] == 0xb2 ? 2 : 0;
    }
    assert(seen_sources == 3 && !rn_tx_take(&topic, payload, &len, clock_ms));
    live_nav[7] = WPK_EVENT_DECODED;
    assert(rn_tx_push(RN_TLM, live_nav, sizeof(live_nav), clock_ms));
    live_nav[7] = WPK_EVENT_DISPATCH;
    assert(rn_tx_push(RN_TLM, live_nav, sizeof(live_nav), clock_ms));
    assert(rn_tx_take(&topic, payload, &len, clock_ms) && payload[7] == WPK_EVENT_DECODED);
    assert(rn_tx_take(&topic, payload, &len, clock_ms) && payload[7] == WPK_EVENT_DISPATCH);
    assert(rn_tx_push(RN_TLM, live_motion, 2, clock_ms));
    assert(!rn_tx_take(&topic, payload, &len, clock_ms + 251));
    assert(!rn_telemetry_room(2, 1024, 11680));
    assert(!rn_telemetry_room(16, 203, 11680));
    assert(!rn_telemetry_room(16, 1024, 203));
    assert(rn_telemetry_room(3, 204, 204));
    MotionPort port = {.now_us = api_now, .outputs_off = api_off};
    motion_init(&port);
    MotionFeedback f = {.now_us = board_time_us(), .line_level = {500, 0}};
    request(WP_CMD_START);
    rn_drive_process(&f);
    assert(starts == 0); /* Early reconnect/retained Start rejected. */
    request(WP_CMD_START);
    /* A STOP arriving after queued Start invalidates it even before FIFO delivery. */
    assert(rn_rx_push(stop, 4, 4, clock_ms));
    rn_drive_process(&f);
    assert(starts == 0);
    (void)rn_take_urgent();
    while (tk_rcv_mbf(rx, msg, 0) > 0) {
    }
    clock_ms += 1100;
    online();
    f.now_us = board_time_us();
    request(WP_CMD_START);
    rn_drive_process(&f);
    assert(starts == 1 && pending_start);
    pending_start = false;
    hardware.flags = BR_ACTIVE | BR_READY | BR_CLOCK_OK;
    motion_session_begin(board_time_us());
    assert(motion_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
    (void)motion_update(&f);
    request(WP_CMD_PAUSE);
    rn_drive_process(&f);
    assert(rn_drive_paused() && offs > 0);
    (void)motion_update(&f);
    clock_ms += 160;
    online();
    f.now_us = board_time_us();
    (void)motion_update(&f);
    request(WP_CMD_RESUME);
    rn_drive_process(&f);
    assert(!rn_drive_paused() && resumes == 1 && motion_get_status().mode == MOTION_AUTO);
    assert(motion_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
    request(WP_CMD_RESUME);
    rn_drive_process(&f);
    assert(resumes == 1);
    /* An old queued Reset cannot clear a subsequent physical E-stop. */
    request(WP_CMD_RESET);
    ++clock_ms;
    bringup_stop_from(BR_REASON_GP20, BR_ESTOP, MOTION_BUDDY2);
    rn_drive_process(&f);
    assert(hardware.flags & BR_ESTOP);
    while (rn_tx_take(&topic, payload, &len, clock_ms)) {
    }
    unsigned motions = 0, navs = 0;
    for (unsigned n = 0; n < 20; ++n) {
        clock_ms += 50;
        online();
        rn_drive_telemetry(clock_ms);
        while (rn_tx_take(&topic, payload, &len, clock_ms)) {
            if ((payload[0] >> 4) == WPK_SRC_MOTION) {
                assert(len == WPK_MOTION_BYTES && payload[2] == motions++);
            } else if ((payload[0] >> 4) == WPK_SRC_NAV) {
                assert(len == WPK_NAV_BYTES && payload[2] == navs++);
            }
        }
    }
    assert(motions == 20 && navs == 4); /* No artificial gaps between sources. */
    puts(
        "PASS connected bridge: state/epochs/stale/flood STOP/priority/real coordinator Pause+Resume/late Reset/independent telemetry sequences");
    return 0;
}
