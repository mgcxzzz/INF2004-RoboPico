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
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>

#include "nav_iface.h"
#include "safety_config.h"
#include "safety_iface.h"

#define EV_ESTOP            (1U << 0)
#define EV_RESET            (1U << 1)
#define EV_ACK              (1U << 2)
#define EV_FAULT_SHIFT      (8U)        /* bits  8..15: critical, by src */
#define EV_WARN_SHIFT       (16U)       /* bits 16..23: warning, by src  */
#define EV_SRC_MASK         ((1U << SAFETY_SRC_COUNT) - 1U)
#define EV_ALL              (EV_ESTOP | EV_RESET | EV_ACK |               \
                             (EV_SRC_MASK << EV_FAULT_SHIFT) |            \
                             (EV_SRC_MASK << EV_WARN_SHIFT))

static ID                g_flg  = 0;
static ID                g_task = 0;
static safety_status_t   g_st;
static volatile bool     g_b_inhibit = false;
static volatile uint32_t g_fault_code[SAFETY_SRC_COUNT];
static volatile uint32_t g_warn_code[SAFETY_SRC_COUNT];
static volatile uint32_t g_last_kick_ms[SAFETY_SRC_COUNT];
static uint32_t          g_watch_ms[SAFETY_SRC_COUNT];
static uint32_t          g_stop_t0_ms = 0U;
static bool              g_b_awaiting_ack = false;
static bool              g_b_slow_noted = false;
static bool              g_b_was_running = false;

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

safety_state_t safety_get_state (void)     { return g_st.state; }
bool safety_drive_inhibited (void)          { return g_b_inhibit; }
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
safety_report_fault (safety_src_t src, uint32_t code, bool b_critical)
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
    g_watch_ms[src]     = timeout_ms;
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

static void
publish (void)
{
    g_st.b_inhibit = g_b_inhibit;
    safety_telemetry_publish(&g_st);
}

static void
inhibit_drive (void)
{
    if (!g_b_inhibit)
    {
        g_b_inhibit      = true;
        g_stop_t0_ms     = now_ms();
        g_b_awaiting_ack = true;
        g_b_slow_noted   = false;
        mc_inhibit(true);               /* first, before anything else */
    }
}

static void
enter_estop (void)
{
    g_st.estops++;

    if (g_st.state == SAFETY_ST_FAULT)
    {
        publish();                      /* already stopped and latched */
        return;
    }

    inhibit_drive();
    g_st.state = SAFETY_ST_ESTOP;
    (void)nav_post_safety(NAV_SAFETY_ESTOP);
    publish();
}

static void
enter_fault (safety_src_t src, uint32_t code)
{
    inhibit_drive();
    g_st.faults++;
    g_st.fault_src  = src;
    g_st.fault_code = code;

    if (g_st.state != SAFETY_ST_FAULT)
    {
        g_st.state = SAFETY_ST_FAULT;
        (void)nav_post_safety(NAV_SAFETY_FAULT);
    }

    publish();
}

static void
note_warning (safety_src_t src, uint32_t code)
{
    g_st.warnings++;
    g_st.warn_src  = src;
    g_st.warn_code = code;
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
            ((now - g_last_kick_ms[s]) > g_watch_ms[s]))
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

    if (g_st.state == SAFETY_ST_NORMAL)
    {
        return;                         /* nothing latched */
    }

    if (g_b_was_running && !tasks_healthy(now_ms(), &silent))
    {
        return;                         /* a task is still not responding */
    }

    g_st.state       = SAFETY_ST_NORMAL;
    g_b_inhibit      = false;
    g_b_awaiting_ack = false;
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
    if (g_b_awaiting_ack)
    {
        g_st.last_stop_ms = now_ms() - g_stop_t0_ms;

        if (g_st.last_stop_ms > g_st.max_stop_ms)
        {
            g_st.max_stop_ms = g_st.last_stop_ms;
        }

        if (g_st.last_stop_ms >= SAFETY_STOP_LIMIT_MS)
        {
            g_st.slow_stops++;
        }

        g_b_awaiting_ack = false;
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
    if (running && !g_b_was_running)
    {
        uint32_t s;

        for (s = 0U; s < (uint32_t)SAFETY_SRC_COUNT; s++)
        {
            g_last_kick_ms[s] = now;
        }
    }

    g_b_was_running = running;

    if (running && (g_st.state == SAFETY_ST_NORMAL) &&
        !tasks_healthy(now, &silent))
    {
        enter_fault(silent, SAFETY_CODE_TASK_SILENT);
    }

    /* NFR-SYS-04: flag a stop that is not confirmed within 100 ms. Keep
     * timing it, so the real figure is still recorded when it comes. */
    if (g_b_awaiting_ack && !g_b_slow_noted &&
        ((now - g_stop_t0_ms) > SAFETY_STOP_LIMIT_MS))
    {
        g_b_slow_noted = true;
        note_warning(SAFETY_SRC_SAFETY, SAFETY_CODE_STOP_SLOW);
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
