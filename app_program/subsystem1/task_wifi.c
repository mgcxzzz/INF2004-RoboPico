/**
 * @file    task_wifi.c
 * @brief   WiFi and MQTT (Buddy 1), task side: the outgoing telemetry
 *          queue, command checking and dispatch, and the link state of
 *          statechart 6.1.
 *
 *   other buddies --*_telemetry_publish()--> mbf_out --(TMO_POL)--> network
 *   network side --b1_net_command()--> mbf_cmd --> b1_task: check, then
 *       ESTOP  -> safety_post_estop()                (straight to Safety)
 *       RESET  -> safety_request_reset() + nav_post_command(NAV_CMD_RESET)
 *       START / PAUSE / RESUME / STOP -> nav_post_command()
 *       steering / navigation -> rejected (FR-WIFI-03), never dispatched
 *   and every command gets an acknowledgement on is28/ack.
 *
 * The network side (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c) runs in
 * the kernel's WiFi service task and reaches this file only through the
 * b1_net_*() functions, which never wait.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#include "subsystem1.h"
#include "b1_net.h"
#include "b1_proto.h"
#include "b1_mapping.h"
#include "../system/nav_iface.h"
#include "../system/safety_iface.h"
#include "../system/sys_fault.h"

/* Warning code to the Safety Supervisor (8 bits, numbered by Buddy 1).
 * A warning is logged and published; it never stops the car (Week 6:
 * losing WiFi or MQTT is not a system fault). */
#define B1_WARN_LINK_LOST   (0x01U)

#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#define B1_LINK_UP          (3)     /* CYW43_LINK_UP in cyw43.h */
#endif

/* Prototypes of hooks declared in other buddies' headers (s3_iface.h,
 * imu_iface.h) that are not on this branch. Same signatures. */
void line_telemetry_publish(uint8_t const * p_frame, uint32_t len);
void imu_telemetry_publish(uint8_t const * p_frame, uint32_t len);

/* ---------------------------------------------------------------------
 * State. Writers: [T] Buddy 1 task, [N] network side (WiFi service task),
 * [A] any task. Each counter has one writer; readers take a copy.
 * ------------------------------------------------------------------ */
static ID                g_mbf_out = 0;        /* outgoing queue          */
static ID                g_mbf_cmd = 0;        /* incoming commands       */
static ID                g_sem_drops = 0;      /* [A] frames dropped: a
                                                * semaphore count is an
                                                * atomic counter on both
                                                * cores                   */
static volatile uint8_t  g_net = (uint8_t)B1_NET_DISCONNECTED;   /* [N] */
static volatile uint8_t  g_shown = (uint8_t)B1_NET_DISCONNECTED; /* [T] */
static volatile uint8_t  g_nav_state = B1_STATE_UNKNOWN;         /* [A] */
static volatile bool     g_b_hb_now = false;                     /* [A] */
static volatile bool     g_b_forward = true;                     /* [T] */
static volatile uint8_t  g_last_cmd = B1_CMD_NONE;               /* [T] */
static volatile uint8_t  g_last_result = B1_RESULT_NONE;         /* [T] */
static volatile uint8_t  g_accepted = 0U;                        /* [T] */
static volatile uint8_t  g_rejected = 0U;                        /* [T] */
static volatile uint8_t  g_connects = 0U;                        /* [N] */
static volatile uint16_t g_published = 0U;                       /* [N] */
static volatile uint16_t g_net_lost = 0U;                        /* [N] */
static uint8_t           g_hb_seq = 0U;                          /* [N] */
static ID                g_mapping_mtx = 0;
static uint32_t          g_mapping = 0U;  /* map + valid + revision; mutex */
static uint8_t           g_mapping_seq = 0U;                    /* [T] */

/* <string.h> clashes with tk/syslib.h's size_t, so copy by hand. */
static void
copy_bytes (uint8_t * p_dst, uint8_t const * p_src, uint32_t n)
{
    uint32_t i;

    for (i = 0U; i < n; i++)
    {
        p_dst[i] = p_src[i];
    }
}

/* ---------------------------------------------------------------------
 * Objects (phase 1)
 * ------------------------------------------------------------------ */

ER
b1_create_objects (void)
{
    T_CMBF cmbf = { 0 };
    T_CSEM csem = { 0 };
    T_CMTX cmtx = { 0 };

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S1_OUT_BUF_BYTES;
    cmbf.maxmsz = 1 + B1_OUT_MAX;           /* kind byte + body */
    g_mbf_out   = tk_cre_mbf(&cmbf);
    if (g_mbf_out < E_OK)
    {
        return g_mbf_out;
    }

    cmbf.bufsz  = S1_CMD_BUF_BYTES;
    cmbf.maxmsz = B1_CMD_MAX;
    g_mbf_cmd   = tk_cre_mbf(&cmbf);
    if (g_mbf_cmd < E_OK)
    {
        return g_mbf_cmd;
    }

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem  = 0x7FFFFFFF;
    g_sem_drops  = tk_cre_sem(&csem);
    if (g_sem_drops < E_OK)
    {
        return g_sem_drops;
    }
    cmtx.mtxatr = TA_INHERIT;
    g_mapping_mtx = tk_cre_mtx(&cmtx);
    return (g_mapping_mtx < E_OK) ? g_mapping_mtx : E_OK;
}

ER
b1_barcode_mapping_get (uint32_t * p_snapshot)
{
    ER er;

    if (p_snapshot == 0)
    {
        return E_PAR;
    }
    if (g_mapping_mtx <= 0)
    {
        return E_OBJ;
    }
    er = tk_loc_mtx(g_mapping_mtx, TMO_POL);
    if (er != E_OK)
    {
        return er;
    }
    *p_snapshot = g_mapping;
    (void)tk_unl_mtx(g_mapping_mtx);
    return E_OK;
}

/* ---------------------------------------------------------------------
 * Outgoing queue
 * ------------------------------------------------------------------ */

static void
count_drop (void)
{
    if (g_sem_drops > 0)
    {
        (void)tk_sig_sem(g_sem_drops, 1);
    }
}

static uint32_t
drops (void)
{
    T_RSEM rsem;

    if ((g_sem_drops > 0) && (tk_ref_sem(g_sem_drops, &rsem) == E_OK))
    {
        return (uint32_t)rsem.semcnt;
    }
    return 0U;
}

/* Copy one message into the queue. Never waits: if the queue is full
 * (broker gone for a while), the message is dropped and counted. */
static bool
queue_out (uint8_t kind, uint8_t const * p_body, uint32_t len)
{
    uint8_t msg[1U + B1_OUT_MAX];

    if ((g_mbf_out <= 0) || (len == 0U) || (len > B1_OUT_MAX))
    {
        count_drop();
        return false;
    }

    msg[0] = kind;
    copy_bytes(&msg[1], p_body, len);

    if (tk_snd_mbf(g_mbf_out, msg, (INT)(len + 1U), TMO_POL) != E_OK)
    {
        count_drop();
        return false;
    }
    return true;
}

void
b1_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    if (g_b_forward)
    {
        (void)queue_out(B1_OUT_TLM, p_frame, len);
    }
}

/* The hooks every other buddy calls (their headers say: copy the bytes
 * and return, never block). All frames go out unchanged on is28/tlm. */
void
nav_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b1_telemetry_publish(p_frame, len);
}

void
safety_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b1_telemetry_publish(p_frame, len);
}

void
b5_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b1_telemetry_publish(p_frame, len);
}

void
line_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b1_telemetry_publish(p_frame, len);
}

void
imu_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b1_telemetry_publish(p_frame, len);
}

/* Operating state from the Navigation Coordinator: goes into the next
 * heartbeat, which is sent at once. */
void
wifi_notify_state (nav_state_t state)
{
    g_nav_state = (uint8_t)sys_sat_u((uint32_t)state, B1_BITS_STATE);
    g_b_hb_now  = true;
}

/* ---------------------------------------------------------------------
 * Bridge for the network side (b1_net.h). Never block.
 * ------------------------------------------------------------------ */

bool
b1_net_take (uint8_t * p_kind, uint8_t * p_body, uint32_t * p_len)
{
    uint8_t msg[1U + B1_OUT_MAX];
    INT     n;

    if (g_mbf_out <= 0)
    {
        return false;
    }

    n = tk_rcv_mbf(g_mbf_out, msg, TMO_POL);
    if (n < 2)
    {
        return false;
    }

    *p_kind = msg[0];
    *p_len  = (uint32_t)n - 1U;
    copy_bytes(p_body, &msg[1], *p_len);
    return true;
}

void
b1_net_command (uint8_t const * p_msg, uint32_t len)
{
    uint8_t buf[B1_CMD_MAX];
    uint32_t n = (len > B1_CMD_MAX) ? B1_CMD_MAX : len;

    if ((g_mbf_cmd <= 0) || (n == 0U))
    {
        return;
    }

    /* Keep the real length visible to the checker: a longer message is
     * passed on cut to B1_CMD_MAX bytes, longer than any valid command,
     * so it is still rejected rather than accepted as a valid prefix. */
    copy_bytes(buf, p_msg, n);
    if (tk_snd_mbf(g_mbf_cmd, buf, (INT)n, TMO_POL) != E_OK)
    {
        count_drop();
    }
}

uint32_t
b1_net_heartbeat (uint8_t * p_frame)
{
    b1_tlm_t t = { 0 };
    T_RMBF   rmbf;
    SYSTIM   now;
    uint64_t ms;

    (void)tk_get_otm(&now);
    ms = ((uint64_t)(uint32_t)now.hi << 32) | (uint64_t)now.lo;

    t.seq         = g_hb_seq++;
    t.net         = g_shown;
    t.state       = g_nav_state;
    t.last_cmd    = g_last_cmd;
    t.last_result = g_last_result;
    t.uptime_s    = (uint16_t)sys_sat_u((uint32_t)(ms / 1000U), B1_BITS_UPTIME);
    t.published   = g_published;
    t.lost        = (uint16_t)(g_net_lost + drops());
    t.accepted    = g_accepted;
    t.rejected    = g_rejected;
    t.connects    = g_connects;
    if ((g_mbf_out > 0) && (tk_ref_mbf(g_mbf_out, &rmbf) == E_OK))
    {
        t.fill_pct = b1_fill_pct((uint32_t)(S1_OUT_BUF_BYTES - rmbf.frbufsz),
                                 (uint32_t)S1_OUT_BUF_BYTES);
    }

    b1_tlm_pack(&t, p_frame);
    return B1_TLM_FRAME_BYTES;
}

bool
b1_net_heartbeat_now (void)
{
    if (g_b_hb_now)
    {
        g_b_hb_now = false;
        return true;
    }
    return false;
}

void
b1_net_set_state (uint8_t net)
{
    if ((net == (uint8_t)B1_NET_OPERATIONAL) &&
        (g_net != (uint8_t)B1_NET_OPERATIONAL))
    {
        g_connects++;
        g_b_hb_now = true;          /* announce ourselves at once */
    }
    g_net = net;
}

void
b1_net_count (bool b_published)
{
    if (b_published)
    {
        g_published++;
    }
    else
    {
        g_net_lost++;
    }
}

/* ---------------------------------------------------------------------
 * Command dispatch (statechart 6.1: Validate Command -> Dispatch / Reject)
 * ------------------------------------------------------------------ */

static uint8_t
mapping_command (b1_cmd_t const * p_cmd)
{
    uint32_t snapshot;
    uint8_t frame[B1_MAP_FRAME_BYTES];
    bool queued;

    if ((g_mapping_mtx <= 0) ||
        (tk_loc_mtx(g_mapping_mtx, TMO_POL) != E_OK))
    {
        return B1_ACK_BUSY;
    }
    if (p_cmd->code == B1_CMD_BARCODE_MAP)
    {
        g_mapping = b1_mapping_replace(g_mapping, p_cmd->mapping);
    }
    snapshot = g_mapping;
    (void)tk_unl_mtx(g_mapping_mtx);

    b1_mapping_frame(snapshot, p_cmd->id, g_mapping_seq++, frame);
    /* Configuration readback is independent of telemetry forwarding.
     * If the queue is full after SET, the map WAS applied: keep the ack
     * truthful. The PC must GET the active map if readback is lost. */
    queued = queue_out(B1_OUT_TLM, frame, B1_MAP_FRAME_BYTES);
    return (queued || (p_cmd->code == B1_CMD_BARCODE_MAP)) ?
           B1_ACK_ACCEPTED : B1_ACK_BUSY;
}

static uint8_t
dispatch (b1_cmd_t const * p_cmd)
{
    ER er = E_OK;

    switch (p_cmd->code)
    {
        case B1_CMD_ESTOP:
            er = safety_post_estop();       /* not via the coordinator */
            break;
        case B1_CMD_RESET:
            (void)safety_request_reset();   /* clears a released latch */
            er = nav_post_command(NAV_CMD_RESET);
            /* Fault-mask rule: clear our bit only after a reset AND once
             * the cause is gone; a command just arrived, so the link is up. */
            (void)sys_fault_clear(SYS_FAULT_WIFI_LOSS);
            break;
        case B1_CMD_START:
            er = nav_post_command(NAV_CMD_START);
            break;
        case B1_CMD_PAUSE:
            er = nav_post_command(NAV_CMD_PAUSE);
            break;
        case B1_CMD_RESUME:
            er = nav_post_command(NAV_CMD_RESUME);
            break;
        case B1_CMD_STOP:
            er = nav_post_command(NAV_CMD_STOP);
            break;
        case B1_CMD_CONFIG:
            g_b_forward = ((p_cmd->arg & 0x0FFFU) != 0U);   /* B1_CFG_TELEMETRY */
            break;
        case B1_CMD_PING:
            g_b_hb_now = true;
            break;
        case B1_CMD_BARCODE_MAP:
        case B1_CMD_BARCODE_GET:
            return mapping_command(p_cmd);
        default:
            er = E_PAR;                     /* b1_cmd_check() let it by? */
            break;
    }

    return (er == E_OK) ? B1_ACK_ACCEPTED : B1_ACK_BUSY;
}

static void
handle_command (uint8_t const * p_msg, uint32_t len)
{
    b1_cmd_t cmd;
    uint8_t  result = b1_cmd_check(p_msg, len, &cmd);
    uint8_t  ack[B1_ACK_BYTES];

    if (result == B1_ACK_ACCEPTED)
    {
        result = dispatch(&cmd);
    }

    if (result == B1_ACK_ACCEPTED)
    {
        g_accepted++;
    }
    else
    {
        g_rejected++;
    }
    g_last_cmd    = cmd.code;
    g_last_result = result;

    b1_ack_pack(&cmd, result, g_nav_state, ack);
    (void)queue_out(B1_OUT_ACK, ack, B1_ACK_BYTES);

#if S1_CONSOLE_LOG
    tm_printf((UB *)"[wifi] command %s (id %d): %s\n",
              (UB *)b1_cmd_name(cmd.code), (INT)cmd.id,
              (UB *)b1_ack_name(result));
#endif
}

/* ---------------------------------------------------------------------
 * Link state shown to everyone: Disconnected whenever the WiFi link is
 * down, otherwise what the network side reports.
 * ------------------------------------------------------------------ */

static uint8_t
link_state (void)
{
#if TM_WIFI_CYW43
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    if (!st.ready || (st.link_status != B1_LINK_UP))
    {
        return (uint8_t)B1_NET_DISCONNECTED;
    }
    return (g_net == (uint8_t)B1_NET_DISCONNECTED) ?
           (uint8_t)B1_NET_CONNECTING : g_net;
#else
    return (uint8_t)B1_NET_DISCONNECTED;
#endif
}

void
b1_task (INT stacd, void * p_exinf)
{
    uint8_t msg[B1_CMD_MAX];
    INT     n;

    (void)stacd;
    (void)p_exinf;

#if !(TM_WIFI_CYW43 && TM_NET_LWIP && TM_MQTT)
    tm_printf((UB *)"[wifi] built without WIFI=cyw43 NET=lwip MQTT=1: "
                    "no network, frames are queued and dropped\n");
#endif
    (void)safety_watch(SAFETY_SRC_WIFI, S1_WATCH_MS);

    for (;;)
    {
        n = tk_rcv_mbf(g_mbf_cmd, msg, S1_TICK_MS);
        if (n > 0)
        {
            handle_command(msg, (uint32_t)n);
        }

        {
            uint8_t shown = link_state();

            if (shown != g_shown)
            {
                if ((g_shown == (uint8_t)B1_NET_OPERATIONAL) &&
                    (shown != (uint8_t)B1_NET_OPERATIONAL))
                {
                    /* Recorded (shared mask, latched until a reset) and
                     * reported as a warning only: the car keeps going. */
                    (void)sys_fault_set(SYS_FAULT_WIFI_LOSS);
                    (void)safety_report_fault(SAFETY_SRC_WIFI,
                                              B1_WARN_LINK_LOST, false);
                }
                g_shown    = shown;
                g_b_hb_now = true;
#if S1_CONSOLE_LOG
                tm_printf((UB *)"[wifi] %s\n", (UB *)b1_net_name(shown));
#endif
            }
        }

        safety_kick(SAFETY_SRC_WIFI);   /* task alive, link or no link */
    }
}

#if APP_TEST
/* For subsystem1_test.c only. */
void
b1_test_counts (uint8_t * p_accepted, uint8_t * p_rejected)
{
    *p_accepted = g_accepted;
    *p_rejected = g_rejected;
}
#endif
