/**
 * @file    safety_supervisor.c
 * @brief   Safety Supervisor task (team task, highest priority).
 *
 *   NORMAL --emergency stop--> ESTOP --reset request--> NORMAL
 *   NORMAL --critical fault--> FAULT --reset request--> NORMAL
 *   (an emergency stop during FAULT keeps FAULT: it is the stronger state)
 *
 * On entering ESTOP or FAULT the supervisor, in this order:
 *   1. calls mc_inhibit(true) so Motion Control removes drive at once,
 *   2. tells Navigation Coordinator (Emergency Stop / Fault state),
 *   3. starts timing until Motion Control calls safety_ack_drive_off().
 *
 * Reports reach the task through one event flag: one bit each for e-stop,
 * reset request and drive-off acknowledge, plus one critical-fault bit and
 * one warning bit per source. Setting a flag is allowed from interrupts,
 * so every entry point is safe from anywhere.
 *
 * Minimum-bit data (Tribunal 1), and who may write what:
 *  - state and "drive inhibited", which other tasks poll, share ONE byte
 *    written with a single store by this task only;
 *  - each source's last fault and warning code is one byte, written only
 *    by that source and read here after its event-flag bit is seen. One
 *    byte per source on purpose: packing several sources into one word
 *    would make two reporters read-modify-write the same word, and a
 *    report could be lost;
 *  - the record behind the telemetry (codes, counters, stop times) is a
 *    bit-field that only this task touches;
 *  - timestamps stay 32-bit (team decision), health timeouts are 16-bit.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>

#include "nav_iface.h"
#include "safety_config.h"
#include "safety_iface.h"
#include "sys_fault.h"

#define EV_ESTOP            (1U << 0)
#define EV_RESET            (1U << 1)
#define EV_ACK              (1U << 2)
#define EV_FAULT_SHIFT      (8U)        /* bits  8..15: critical, by src */
#define EV_WARN_SHIFT       (16U)       /* bits 16..23: warning, by src  */
#define EV_SRC_MASK         ((1U << SAFETY_SRC_COUNT) - 1U)
#define EV_ALL              (EV_ESTOP | EV_RESET | EV_ACK |               \
                             (EV_SRC_MASK << EV_FAULT_SHIFT) |            \
                             (EV_SRC_MASK << EV_WARN_SHIFT))

#define MODE_STATE_MASK     (0x3U)      /* g_mode bits 0..1: state     */
#define MODE_INHIBIT        (1U << 2)   /* g_mode bit 2: drive cut     */

/* Record behind the telemetry frame; only the supervisor task uses it. */
typedef struct
{
    uint32_t fault_src      : 3;    /* safety_src_t or SAFETY_SRC_NONE */
    uint32_t warn_src       : 3;
    uint32_t fault_code     : 8;
    uint32_t warn_code      : 8;
    uint32_t b_awaiting_ack : 1;    /* timing a stop                  */
    uint32_t b_slow_noted   : 1;    /* slow-stop warning already sent */
    uint32_t b_was_running  : 1;
    uint32_t estops         : 8;    /* counters wrap; use differences */
    uint32_t faults         : 8;
    uint32_t warnings       : 8;
    uint32_t slow_stops     : 8;
    uint32_t last_stop_ms   : 10;   /* saturates at 1023              */
    uint32_t max_stop_ms    : 10;
    uint32_t tlm_seq        : 8;    /* telemetry sequence, wrapping   */
} safety_rec_t;

_Static_assert(SAFETY_SRC_COUNT <= SAFETY_SRC_NONE,
               "sources must fit in 3 bits with one value left for none");
_Static_assert(SAFETY_ST_COUNT <= 4, "state is 2 bits");

static ID                g_flg  = 0;
static ID                g_task = 0;
static volatile uint8_t  g_mode = 0U;       /* Normal, drive not cut  */
static safety_rec_t      g_rec;
static volatile uint8_t  g_fault_code[SAFETY_SRC_COUNT];
static volatile uint8_t  g_warn_code[SAFETY_SRC_COUNT];
static volatile uint32_t g_last_kick_ms[SAFETY_SRC_COUNT];
static uint16_t          g_watch_ms[SAFETY_SRC_COUNT];
static uint32_t          g_stop_t0_ms = 0U;
static uint8_t           g_published_fault_mask = 0U;

static char const * const g_state_names[SAFETY_ST_COUNT] =
{
    "Normal", "Emergency Stop", "Fault"
};

/* ---------------------------------------------------------------------
 * Entry points (any context)
 * ------------------------------------------------------------------ */

static uint32_t
now_ms (void)
{
    SYSTIM t;

    (void)tk_get_otm(&t);
    return (uint32_t)t.lo;
}

static ER
post (UINT bits)
{
    return (g_flg > 0) ? tk_set_flg(g_flg, bits) : E_NOEXS;
}

safety_state_t
safety_get_state (void)
{
    return (safety_state_t)(g_mode & MODE_STATE_MASK);
}

bool
safety_drive_inhibited (void)
{
    return ((g_mode & MODE_INHIBIT) != 0U);
}

/* Supervisor task only: state and inhibit in one byte store. */
static void
set_mode (safety_state_t state, bool b_inhibit)
{
    g_mode = (uint8_t)(((uint32_t)state & MODE_STATE_MASK) |
                       (b_inhibit ? MODE_INHIBIT : 0U));
}

ER   safety_post_estop (void)               { return post(EV_ESTOP); }
ER   safety_request_reset (void)            { return post(EV_RESET); }
void safety_ack_drive_off (void)            { (void)post(EV_ACK); }

char const *
safety_state_name (safety_state_t state)
{
    return (state < SAFETY_ST_COUNT) ? g_state_names[state] : "?";
}

/**
 * @brief  Report a fault. critical = unrecoverable: the car is stopped and
 *         stays stopped until a reset. Otherwise it is only logged.
 */
ER
safety_report_fault (safety_src_t src, uint8_t code, bool b_critical)
{
    if (src >= SAFETY_SRC_COUNT)
    {
        return E_PAR;
    }

    if (b_critical)
    {
        g_fault_code[src] = code;
        return post(1U << (EV_FAULT_SHIFT + (uint32_t)src));
    }

    g_warn_code[src] = code;
    return post(1U << (EV_WARN_SHIFT + (uint32_t)src));
}

/**
 * @brief  Declare that src will call safety_kick() at least every
 *         timeout_ms while the car is running. Call during start-up.
 */
ER
safety_watch (safety_src_t src, uint32_t timeout_ms)
{
    if (src >= SAFETY_SRC_COUNT)
    {
        return E_PAR;
    }

    g_last_kick_ms[src] = now_ms();
    g_watch_ms[src]     = (uint16_t)sys_sat_u(timeout_ms, 16U);
    return E_OK;
}

/**
 * @brief  "I am still alive." Call from the task's own loop (task context).
 */
void
safety_kick (safety_src_t src)
{
    if (src < SAFETY_SRC_COUNT)
    {
        g_last_kick_ms[src] = now_ms();
    }
}

/* ---------------------------------------------------------------------
 * Supervisor
 * ------------------------------------------------------------------ */

/* Build the telemetry frame (header + 12-byte payload, sys_tlm.h) and
 * hand it to Buddy 1. */
static void
publish (void)
{
    safety_tlm_t t;
    uint8_t      frame[SAFETY_TLM_FRAME_BYTES];

    t.seq          = (uint8_t)g_rec.tlm_seq;
    t.state        = (uint8_t)safety_get_state();
    t.b_inhibit    = safety_drive_inhibited();
    t.fault_src    = (uint8_t)g_rec.fault_src;
    t.warn_src     = (uint8_t)g_rec.warn_src;
    t.fault_code   = (uint8_t)g_rec.fault_code;
    t.warn_code    = (uint8_t)g_rec.warn_code;
    t.estops       = (uint8_t)g_rec.estops;
    t.faults       = (uint8_t)g_rec.faults;
    t.warnings     = (uint8_t)g_rec.warnings;
    t.slow_stops   = (uint8_t)g_rec.slow_stops;
    t.last_stop_ms = (uint16_t)g_rec.last_stop_ms;
    t.max_stop_ms  = (uint16_t)g_rec.max_stop_ms;
    t.fault_mask   = (uint8_t)sys_fault_get();

    safety_tlm_pack(&t, frame);
    g_rec.tlm_seq++;                    /* 8-bit field: wraps */
    safety_telemetry_publish(frame, SAFETY_TLM_FRAME_BYTES);
    g_published_fault_mask = t.fault_mask;
}

static void
inhibit_drive (void)
{
    if (!safety_drive_inhibited())
    {
        set_mode(safety_get_state(), true);
        g_stop_t0_ms         = now_ms();
        g_rec.b_awaiting_ack = 1U;
        g_rec.b_slow_noted   = 0U;
        mc_inhibit(true);               /* first, before anything else */
    }
}

static void
enter_estop (void)
{
    g_rec.estops++;                     /* 8-bit field: wraps */

    if (safety_get_state() == SAFETY_ST_FAULT)
    {
        publish();                      /* already stopped and latched */
        return;
    }

    inhibit_drive();
    set_mode(SAFETY_ST_ESTOP, true);
    (void)nav_post_safety(NAV_SAFETY_ESTOP);
    publish();
}

static void
enter_fault (safety_src_t src, uint8_t code)
{
    inhibit_drive();
    g_rec.faults++;
    g_rec.fault_src  = (uint32_t)src;
    g_rec.fault_code = code;

    if (safety_get_state() != SAFETY_ST_FAULT)
    {
        set_mode(SAFETY_ST_FAULT, true);
        (void)nav_post_safety(NAV_SAFETY_FAULT);
    }

    publish();
}

static void
note_warning (safety_src_t src, uint8_t code)
{
    g_rec.warnings++;
    g_rec.warn_src  = (uint32_t)src;
    g_rec.warn_code = code;
    publish();
}

/* Health is "OK" if every watched task has checked in recently. */
static bool
tasks_healthy (uint32_t now, safety_src_t * p_silent)
{
    uint32_t s;

    for (s = 0U; s < (uint32_t)SAFETY_SRC_COUNT; s++)
    {
        if ((g_watch_ms[s] != 0U) &&
            ((now - g_last_kick_ms[s]) > (uint32_t)g_watch_ms[s]))
        {
            *p_silent = (safety_src_t)s;
            return false;
        }
    }

    return true;
}

static void
handle_reset (void)
{
    safety_src_t silent;

    if (safety_get_state() == SAFETY_ST_NORMAL)
    {
        return;                         /* nothing latched */
    }

    if ((g_rec.b_was_running != 0U) && !tasks_healthy(now_ms(), &silent))
    {
        return;                         /* a task is still not responding */
    }

    set_mode(SAFETY_ST_NORMAL, false);
    g_rec.b_awaiting_ack = 0U;
    mc_inhibit(false);

    /* Cleared first, then the reset, so Navigation Coordinator always
     * sees them in this order and goes back to Ready. */
    (void)nav_post_safety(NAV_SAFETY_CLEARED);
    (void)nav_post_command(NAV_CMD_RESET);
    publish();
}

static void
handle_ack (void)
{
    if (g_rec.b_awaiting_ack != 0U)
    {
        uint32_t stop_ms = now_ms() - g_stop_t0_ms;

        /* Saturate to 10 bits: 1023 reads as "far too slow", never as a
         * small wrapped value. */
        g_rec.last_stop_ms = sys_sat_u(stop_ms, SAFETY_BITS_STOP_MS);

        if (g_rec.last_stop_ms > g_rec.max_stop_ms)
        {
            g_rec.max_stop_ms = g_rec.last_stop_ms;
        }

        if (stop_ms >= SAFETY_STOP_LIMIT_MS)
        {
            g_rec.slow_stops++;
        }

        g_rec.b_awaiting_ack = 0U;
        publish();
    }
}

static void
handle_events (UINT ptn)
{
    uint32_t s;

    if ((ptn & EV_ESTOP) != 0U)
    {
        enter_estop();
    }

    for (s = 0U; s < (uint32_t)SAFETY_SRC_COUNT; s++)
    {
        if ((ptn & (1U << (EV_FAULT_SHIFT + s))) != 0U)
        {
            enter_fault((safety_src_t)s, g_fault_code[s]);
        }

        if ((ptn & (1U << (EV_WARN_SHIFT + s))) != 0U)
        {
            note_warning((safety_src_t)s, g_warn_code[s]);
        }
    }

    if ((ptn & EV_ACK) != 0U)
    {
        handle_ack();
    }

    if ((ptn & EV_RESET) != 0U)
    {
        handle_reset();
    }
}

static void
periodic_checks (void)
{
    uint32_t     now     = now_ms();
    bool         running = (nav_get_state() == NAV_ST_RUNNING);
    safety_src_t silent;

    /* Tasks are only expected to check in while the car is running.
     * When a run starts, give everyone a fresh deadline. */
    if (running && (g_rec.b_was_running == 0U))
    {
        uint32_t s;

        for (s = 0U; s < (uint32_t)SAFETY_SRC_COUNT; s++)
        {
            g_last_kick_ms[s] = now;
        }
    }

    g_rec.b_was_running = running ? 1U : 0U;

    if (running && (safety_get_state() == SAFETY_ST_NORMAL) &&
        !tasks_healthy(now, &silent))
    {
        enter_fault(silent, SAFETY_CODE_TASK_SILENT);
    }

    /* NFR-SYS-04: flag a stop that is not confirmed within 100 ms. Keep
     * timing it, so the real figure is still recorded when it comes. */
    if ((g_rec.b_awaiting_ack != 0U) && (g_rec.b_slow_noted == 0U) &&
        ((now - g_stop_t0_ms) > SAFETY_STOP_LIMIT_MS))
    {
        g_rec.b_slow_noted = 1U;
        note_warning(SAFETY_SRC_SAFETY, SAFETY_CODE_STOP_SLOW);
    }

    /* Owners clear their own bits after reset. That can happen after
     * handle_reset(), including when Safety is already Normal. Publish
     * the changed snapshot even if no Safety state transition occurs. */
    if ((uint8_t)sys_fault_get() != g_published_fault_mask)
    {
        publish();
    }
}

static void
safety_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    publish();

    for (;;)
    {
        UINT ptn = 0U;

        if (tk_wai_flg(g_flg, EV_ALL, TWF_ORW | TWF_CLR, &ptn,
                       SAFETY_CHECK_MS) == E_OK)
        {
            handle_events(ptn);
        }

        periodic_checks();
    }
}

/**
 * @brief  Phase 1 (system_init): create the event flag. Called first of
 *         all, so every other subsystem can report to it from then on.
 */
ER
safety_init (void)
{
    T_CFLG cflg = { 0 };

    g_rec.fault_src = SAFETY_SRC_NONE;  /* nothing reported yet */
    g_rec.warn_src  = SAFETY_SRC_NONE;

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_flg        = tk_cre_flg(&cflg);

    if (g_flg < E_OK)
    {
        return g_flg;
    }

    return safety_watch(SAFETY_SRC_NAV, SAFETY_NAV_TIMEOUT_MS);
}

/**
 * @brief  Phase 2 (system_start): create and start the task.
 */
ER
safety_start (void)
{
    T_CTSK ctsk = { 0 };

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = safety_task;
    ctsk.itskpri = SAFETY_TASK_PRIORITY;
    ctsk.stksz   = SAFETY_TASK_STACK_BYTES;
    g_task       = tk_cre_tsk(&ctsk);

    if (g_task < E_OK)
    {
        return g_task;
    }

    return tk_sta_tsk(g_task, 0);
}
