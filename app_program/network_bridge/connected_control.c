/**
 * @file connected_control.c
 * @brief Connected control.
 * @details Application tasks exchange copied, bounded command and telemetry
 * records. The kernel Wi-Fi service owns the transport; reconnect never
 * restarts motion.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Application-side bridge: copied requests only. No lwIP/CYW43 calls here. */
#if defined(APP_RTOS_LINE) && defined(APP_RTOS_NETWORK)
#include "../line_navigation/line_service.h"
#include "network_bridge.h"
#include "network_stop.h"
#include "connected_policy.h"
#include "src/network/wifi_packets.h"
#include "src/network/wifi_protocol.h"
#include "include/interfaces/nav_api.h"
#include "src/motion/drive_mode.h"
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
#include "../imu_navigation/imu_service.h"
#endif
static ID g_requests;
static volatile uint32_t g_paused;
static uint32_t g_lease;
static uint32_t g_seen_connects;
static uint32_t g_connected_since;
static T_SPLOCK g_network_stop_lock;
static rn_network_stop_snapshot_t g_network_stop_snapshot;
static volatile uint32_t g_telemetry_enabled = 1;
/* One32-bit atomic snapshot keeps request/result from different commands from
 * being combined by telemetry on the other core. Bits0..1=request,2..4=ACK. */
static volatile uint32_t g_speed_mode_snapshot = DRIVE_MODE_NONE;

static uint8_t speed_mode_ack (motion_result_t result)
{
    switch (result)
    {
    case MOTION_OK:
        return WP_ACK_ACCEPTED;
    case MOTION_BUSY:
    case MOTION_INACTIVE:
    case MOTION_NOT_OWNER:
    case MOTION_NOT_STOPPED:
        return WP_ACK_BUSY;
    default:
        return WP_ACK_UNSUPPORTED;
    }
}

uint8_t rn_drive_request_speed_mode (unsigned mode)
{
    uint8_t requested =
        drive_mode_valid(mode) ? (uint8_t)mode : DRIVE_MODE_NONE;
    uint8_t result;
    if (!drive_mode_valid(mode))
    {
        result = WP_ACK_UNSUPPORTED;
    }
    else if (!rtos_line_mode_change_allowed())
    {
        result = WP_ACK_BUSY;
    }
    else
    {
        result = speed_mode_ack(nav_request_speed_mode(MOTION_BUDDY1, mode));
    }
    __atomic_store_n(&g_speed_mode_snapshot,
                     (uint32_t)requested | ((uint32_t)result << 2U),
                     __ATOMIC_RELEASE);
    return result;
}

void rn_drive_speed_mode_snapshot (uint8_t * p_requested_mode_value,
                                   uint8_t * p_last_result_value)
{
    uint32_t snapshot =
        __atomic_load_n(&g_speed_mode_snapshot, __ATOMIC_ACQUIRE);
    if (p_requested_mode_value)
    {
        *p_requested_mode_value = (uint8_t)(snapshot & 3U);
    }
    if (p_last_result_value)
    {
        *p_last_result_value = (uint8_t)((snapshot >> 2U) & 7U);
    }
}

static bool imu_safety_blocked (void)
{
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
    return ri_safety_blocked();
#else
    return false;
#endif
}

static UINT network_stop_enter (void)
{
    UINT key;
    (void)ISpinLock(&g_network_stop_lock, &key);
    return key;
}
static void network_stop_leave (UINT key)
{
    (void)ISpinUnlock(&g_network_stop_lock, key);
}

void rn_network_stop_packet (uint8_t seq,
                             uint8_t out[RN_NETWORK_STOP_PACKET_BYTES])
{
    rn_network_stop_snapshot_t copy;
    UINT key = network_stop_enter();
    copy = g_network_stop_snapshot;
    network_stop_leave(key);
    rn_network_stop_encode(&copy, seq, out);
}
void rn_drive_set_telemetry (bool enabled)
{
    __atomic_store_n(&g_telemetry_enabled, enabled ? 1U : 0U, __ATOMIC_RELEASE);
}
bool rn_drive_telemetry_enabled (void)
{
    return __atomic_load_n(&g_telemetry_enabled, __ATOMIC_ACQUIRE) != 0;
}

int32_t rn_drive_init (void)
{
    T_CMBF demand = {0};
    demand.mbfatr = TA_TFIFO;
    demand.bufsz = 256;
    demand.maxmsz = RN_RX_BYTES;
    g_requests = tk_cre_mbf(&demand);
    return g_requests < E_OK ? g_requests : E_OK;
}
bool rn_drive_enqueue (const uint8_t message[RN_RX_BYTES])
{
    return E_OK == tk_snd_mbf(g_requests, message, RN_RX_BYTES, TMO_POL);
}
bool rn_drive_online_record (bool record_failure)
{
    rn_status_t state = rn_status();
    uint32_t app_last = rn_app_last_ms();
    uint32_t current_connected_since =
        __atomic_load_n(&g_connected_since, __ATOMIC_ACQUIRE);
    /* Read the clock after taking the cross-core timestamps so a fresh
     * publication cannot wrap the age calculation. */
    uint32_t now = board_kernel_ms();
    bool usable = (rn_link_usable(&state, now)) && (now - app_last <= 250U);
#ifdef APP_RTOS_RELEASE
    usable =
        usable && (rn_delivery_fresh(&state, now, current_connected_since));
#endif
    if ((!usable) && record_failure)
    {
        rn_network_stop_snapshot_t candidate = rn_network_stop_candidate(
            &state, now, app_last, current_connected_since);
        UINT key = network_stop_enter();
        (void)rn_network_stop_capture_once(&g_network_stop_snapshot,
                                           &candidate);
        network_stop_leave(key);
    }
    return usable;
}
bool rn_drive_online (void)
{
    return rn_drive_online_record(false);
}
bool rn_drive_paused (void)
{
    return __atomic_load_n(&g_paused, __ATOMIC_ACQUIRE) != 0;
}
void rn_drive_cancel (void)
{
    __atomic_store_n(&g_paused, 0, __ATOMIC_RELEASE);
}
void rn_drive_begin (void)
{
    g_lease = 0;
    rn_drive_cancel();
    UINT key = network_stop_enter();
    rn_network_stop_clear(&g_network_stop_snapshot);
    network_stop_leave(key);
}
static uint32_t u32 (const uint8_t * p_data)
{
    return p_data[0] | (uint32_t)p_data[1] << 8 | (uint32_t)p_data[2] << 16 |
           (uint32_t)p_data[3] << 24;
}
static uint8_t pause_line (void)
{
    bringup_status_t state;
    bringup_get_status(&state);
    if (!(state.flags & BR_ACTIVE))
    {
        rtos_line_cancel();
        return WP_ACK_ACCEPTED;
    }
    motion_result_t result = motion_pause(MOTION_BUDDY1, &g_lease);
    if (result != MOTION_OK)
    {
        return WP_ACK_BUSY;
    }
    __atomic_store_n(&g_paused, 1, __ATOMIC_RELEASE);
    bringup_zero_status();
    return WP_ACK_ACCEPTED;
}
/* Called after fresh acquisition and before any optical/PWM calculation. */
void rn_drive_process (void * p_raw)
{
    motion_feedback_t * p_feedback_value = p_raw;
    rn_status_t connection = rn_status();
    if (connection.connects != g_seen_connects)
    {
        g_seen_connects = connection.connects;
        __atomic_store_n(&g_connected_since, board_kernel_ms(),
                         __ATOMIC_RELEASE);
    }
    if (rn_take_pause())
    {
        (void)pause_line();
    }
    uint8_t message[RN_RX_BYTES];
    for (unsigned count = 0;
         (count < 2) && RN_RX_BYTES == tk_rcv_mbf(g_requests, message, TMO_POL);
         ++count)
    {
        wifi_command_t cmd;
        uint32_t now = board_kernel_ms();
        uint8_t result = wp_parse(message + 5, message[4], &cmd);
        rn_status_t net = rn_status();
        bringup_status_t state;
        bringup_get_status(&state);
        if ((!result) && (!rn_request_current(now, u32(message),
                                              u32(message + 13), rn_barrier(),
                                              u32(message + 17), net.connects)))
        {
            result = WP_ACK_BUSY;
        }
        if ((((!result) && (cmd.code != WP_CMD_PAUSE)) &&
             (cmd.code != WP_CMD_CONFIG)) &&
            (!rtos_line_request_fresh(u32(message + 21))))
        {
            result = WP_ACK_BUSY;
        }
        if (!result)
        {
            switch (cmd.code)
            {
            case WP_CMD_START:
                /* Retained or queued Start delivered during resubscription is
                 * never an instruction to restart. Require a fresh user click
                 * after a full second of this connection. */
                if (((((!rn_drive_online()) ||
                       (now - g_connected_since < 1000U)) ||
                      (u32(message) - g_connected_since < 1000U)) ||
                     (!bringup_can_start())) ||
                    (!rtos_line_request_start(u32(message + 21))))
                {
                    result = WP_ACK_BUSY;
                }
                break;
            case WP_CMD_PAUSE:
                result = pause_line();
                break;
            case WP_CMD_RESUME:
                if ((((((((((!rn_drive_online()) || (!rn_drive_paused())) ||
                           (!(state.flags & BR_ACTIVE))) ||
                          (!br_safety_allows_motion(state.flags,
                                                    imu_safety_blocked()))) ||
                         g_bringup_irq_stop) ||
                        (!br_usb_supervised(tm_usb_state()))) ||
                       (board_button_pressed(20))) ||
                      (board_button_pressed(21))) ||
                     (!rtos_line_ready())) ||
                    (motion_resume_line(MOTION_BUDDY1, g_lease, false) !=
                     MOTION_OK))
                {
                    result = WP_ACK_BUSY;
                }
                else
                {
                    /* Ordinary line reacquisition from current position. No old
                     * search deadline or 10ms steering integral survives a
                     * pause. Coordinator rejects interrupted junctions
                     * (NEED_RECOVERY). */
                    rtos_line_resume();
                    rn_drive_cancel();
                    rtos_line_feedback(p_feedback_value, NULL);
                    (void)motion_update(
                        p_feedback_value); /* Finish coordinator handback before
                                              new targets. */
                }
                break;
            case WP_CMD_CONFIG:
                if (2U == (cmd.arg & 0xffU))
                {
                    result = rn_drive_request_speed_mode(cmd.arg >> 8);
                }
#ifdef APP_RTOS_RELEASE
                else if (WP_CONFIG_CALIBRATION_KEY == (cmd.arg & 0xffU))
                {
                    unsigned operation = cmd.arg >> 8U;
                    if (operation != RL_CAL_READ &&
                        operation != RL_CAL_CANCEL &&
                        !rtos_line_request_fresh(u32(message + 21)))
                    {
                        result = WP_ACK_BUSY;
                    }
                    else
                    {
                        result = rtos_line_cal_command(operation, cmd.id);
                    }
                    uint8_t calibration[RL_CAL_FRAME_BYTES];
                    rtos_line_cal_info(rn_next_sequence(15), calibration);
                    (void)rn_tx_push(RN_TLM, calibration, sizeof(calibration),
                                     now);
                }
#endif
                else
                {
                    result = WP_ACK_UNSUPPORTED;
                }
                break;
#ifdef APP_RTOS_RELEASE
            case WP_CMD_GET_LOG:
                result = rtos_line_log_reply(&cmd);
                break;
#endif
            case WP_CMD_RESET:
                if (!bringup_clear_latch())
                {
                    result = WP_ACK_BUSY;
                }
                break;
            default:
                result = WP_ACK_UNSUPPORTED;
                break;
            }
        }
        rn_command_result(cmd.code, result);
        bringup_get_status(&state);
        uint8_t ack[4];
        wp_ack(&cmd, result,
               rn_drive_state(state.flags, rn_drive_paused(),
                              rtos_line_pending(), state.reason),
               ack);
        (void)rn_tx_push(RN_ACK, ack, sizeof(ack), now);
        tm_printf((UB *)"NET_COMMAND code=%u id=%u result=%u active=%u "
                        "paused=%u pending=%u\n",
                  cmd.code, cmd.id, result, !!(state.flags & BR_ACTIVE),
                  rn_drive_paused(), rtos_line_pending());
    }
}
void rn_drive_telemetry (uint32_t now)
{
    static uint32_t last_motion, last_nav;
    if ((!rn_drive_telemetry_enabled()) || (!rn_drive_online()))
    {
        return;
    }
    if (now - last_motion >= 50)
    {
        last_motion = now;
        wifi_motion_packet_t data;
        rtos_line_telemetry(&data);
        data.seq = rn_next_sequence(WPK_SRC_MOTION);
        uint8_t bytes[WPK_MOTION_BYTES];
        if (wpk_motion(&data, bytes))
        {
            (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
        }
    }
    if (now - last_nav >= 250)
    {
        last_nav = now;
        motion_status_t state = motion_get_status();
        wifi_nav_packet_t data_value = {.seq = rn_next_sequence(WPK_SRC_NAV),
                                        .uptime_ms = now,
                                        .owner = state.owner,
                                        .mode = state.mode,
                                        .command_state = state.command_state,
                                        .command_id = state.command_id,
                                        .generation = state.generation,
                                        .fault_bitmap = state.fault,
                                        .event = WPK_EVENT_STATUS};
        uint8_t bytes[WPK_NAV_BYTES];
        if (wpk_nav(&data_value, bytes))
        {
            (void)rn_tx_push(RN_TLM, bytes, sizeof(bytes), now);
        }
    }
}
#endif

/* End of file. */
