/* Application-side bridge: copied requests only. No lwIP/CYW43 calls here. */
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
#include "../line_navigation/line_service.h"
#include "network_bridge.h"
#include "connected_policy.h"
#include "src/network/wifi_packets.h"
#include "src/network/wifi_protocol.h"
static ID requests;
static volatile uint32_t paused;
static uint32_t lease;
static uint32_t seen_connects, connected_since;
static bool telemetry_enabled = true; /* Network application task owns this setting. */
void rn_drive_set_telemetry(bool enabled) {
    telemetry_enabled = enabled;
}

int32_t rn_drive_init(void) {
    T_CMBF d = {0};
    d.mbfatr = TA_TFIFO;
    d.bufsz = 256;
    d.maxmsz = RN_RX_BYTES;
    requests = tk_cre_mbf(&d);
    return requests < E_OK ? requests : E_OK;
}
bool rn_drive_enqueue(const uint8_t message[RN_RX_BYTES]) {
    return tk_snd_mbf(requests, message, RN_RX_BYTES, TMO_POL) == E_OK;
}
bool rn_drive_online(void) {
    RnStatus s = rn_status();
    return rn_link_usable(&s, board_kernel_ms()) && board_kernel_ms() - rn_app_last_ms() <= 250U;
}
bool rn_drive_paused(void) {
    return __atomic_load_n(&paused, __ATOMIC_ACQUIRE) != 0;
}
void rn_drive_cancel(void) {
    __atomic_store_n(&paused, 0, __ATOMIC_RELEASE);
}
void rn_drive_begin(void) {
    lease = 0;
    rn_drive_cancel();
}
static uint32_t u32(const uint8_t *p) {
    return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static uint8_t pause_line(void) {
    BringupStatus s;
    bringup_get_status(&s);
    if (!(s.flags & BR_ACTIVE)) {
        rtos_line_cancel();
        return WP_ACK_ACCEPTED;
    }
    MotionResult result = motion_pause(MOTION_BUDDY1, &lease);
    if (result != MOTION_OK)
        return WP_ACK_BUSY;
    __atomic_store_n(&paused, 1, __ATOMIC_RELEASE);
    bringup_zero_status();
    return WP_ACK_ACCEPTED;
}
/* Called after fresh acquisition and before any optical/PWM calculation. */
void rn_drive_process(void *raw) {
    MotionFeedback *feedback = raw;
    RnStatus connection = rn_status();
    if (connection.connects != seen_connects) {
        seen_connects = connection.connects;
        connected_since = board_kernel_ms();
    }
    if (rn_take_pause())
        (void)pause_line();
    uint8_t message[RN_RX_BYTES];
    for (unsigned n = 0; n < 2 && tk_rcv_mbf(requests, message, TMO_POL) == RN_RX_BYTES; ++n) {
        WifiCommand cmd;
        uint32_t now = board_kernel_ms();
        uint8_t result = wp_parse(message + 5, message[4], &cmd);
        RnStatus net = rn_status();
        BringupStatus s;
        bringup_get_status(&s);
        if (!result && !rn_request_current(now, u32(message), u32(message + 13), rn_barrier(),
                                           u32(message + 17), net.connects))
            result = WP_ACK_BUSY;
        if (!result && cmd.code != WP_CMD_PAUSE && !rtos_line_request_fresh(u32(message + 21)))
            result = WP_ACK_BUSY;
        if (!result) {
            switch (cmd.code) {
            case WP_CMD_START:
                /* Retained or queued Start delivered during resubscription is
                 * never an instruction to restart. Require a fresh user click
                 * after a full second of this connection. */
                if (!rn_drive_online() || now - connected_since < 1000U ||
                    u32(message) - connected_since < 1000U || !bringup_can_start() ||
                    !rtos_line_request_start(u32(message + 21)))
                    result = WP_ACK_BUSY;
                break;
            case WP_CMD_PAUSE:
                result = pause_line();
                break;
            case WP_CMD_RESUME:
                if (!rn_drive_online() || !rn_drive_paused() || !(s.flags & BR_ACTIVE) ||
                    (s.flags & BR_LATCHED) || bringup_irq_stop || tm_usb_state() != 3 ||
                    board_button_pressed(20) || board_button_pressed(21) || !rtos_line_ready() ||
                    motion_resume_line(MOTION_BUDDY1, lease, false) != MOTION_OK) {
                    result = WP_ACK_BUSY;
                } else {
                    /* Ordinary line reacquisition from current position. No old
                     * search deadline or 10ms steering integral survives a pause.
                     * Coordinator rejects interrupted junctions (NEED_RECOVERY). */
                    rtos_line_resume();
                    rn_drive_cancel();
                    rtos_line_feedback(feedback, NULL);
                    (void)motion_update(
                        feedback); /* Finish coordinator handback before new targets. */
                }
                break;
            case WP_CMD_RESET:
                if (!bringup_clear_latch())
                    result = WP_ACK_BUSY;
                break;
            default:
                result = WP_ACK_UNSUPPORTED;
                break;
            }
        }
        rn_command_result(cmd.code, result);
        bringup_get_status(&s);
        uint8_t ack[4];
        wp_ack(&cmd, result,
               rn_drive_state(s.flags, rn_drive_paused(), rtos_line_pending(), s.reason), ack);
        (void)rn_tx_push(RN_ACK, ack, sizeof(ack), now);
        tm_printf((UB *)"NET_COMMAND code=%u id=%u result=%u active=%u paused=%u pending=%u\n",
                  cmd.code, cmd.id, result, !!(s.flags & BR_ACTIVE), rn_drive_paused(),
                  rtos_line_pending());
    }
}
void rn_drive_telemetry(uint32_t now) {
    static uint32_t last_motion, last_nav;
    static uint8_t motion_sequence, nav_sequence;
    if (!telemetry_enabled || !rn_drive_online())
        return;
    if (now - last_motion >= 50) {
        last_motion = now;
        WifiMotionPacket p;
        rtos_line_telemetry(&p);
        p.seq = motion_sequence++;
        uint8_t bytes[WPK_MOTION_BYTES];
        if (wpk_motion(&p, bytes))
            (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
    }
    if (now - last_nav >= 250) {
        last_nav = now;
        MotionStatus s = motion_get_status();
        WifiNavPacket p = {.seq = nav_sequence++,
                           .uptime_ms = now,
                           .owner = s.owner,
                           .mode = s.mode,
                           .command_state = s.command_state,
                           .command_id = s.command_id,
                           .generation = s.generation,
                           .fault_bitmap = s.fault,
                           .event = WPK_EVENT_STATUS};
        uint8_t bytes[WPK_NAV_BYTES];
        if (wpk_nav(&p, bytes))
            (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
    }
}
#endif
