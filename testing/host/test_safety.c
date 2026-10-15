/**
 * @file    test_safety.c
 * @brief   Host test of safety_supervisor.c and safety_glue.c.
 *
 * Both files are compiled unchanged against a mock kernel with a fake
 * millisecond clock. The test reports faults and stops the way the other
 * buddies would, lets the supervisor run for a set time, and checks what
 * it told Motion Control and the Navigation Coordinator.
 *
 * Build and run:  make -C testing/host
 */

#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>

#include <tk/tkernel.h>

#include "b5_iface.h"
#include "nav_config.h"
#include "nav_iface.h"
#include "safety_config.h"
#include "safety_iface.h"

/* ---------------------------------------------------------------------
 * Mock kernel
 * ------------------------------------------------------------------ */
static UINT     g_flag;
static FP       g_task;
static jmp_buf  g_out;
static uint32_t g_ms;
static int      g_idle_left;

ID tk_cre_tsk (T_CTSK const * p) { g_task = p->task; return 3; }
ER tk_sta_tsk (ID id, INT s)     { (void)id; (void)s; return E_OK; }
ID tk_cre_mbf (T_CMBF const * p) { (void)p; return 1; }
ER tk_snd_mbf (ID i, void const * m, INT s, TMO t)
{ (void)i; (void)m; (void)s; (void)t; return E_OK; }
INT tk_rcv_mbf (ID i, void * m, TMO t) { (void)i; (void)m; (void)t; return 0; }
ER tk_get_otm (SYSTIM * p)       { p->hi = 0; p->lo = g_ms; return E_OK; }
ID tk_cre_flg (T_CFLG const * p) { g_flag = p->iflgptn; return 1; }
ER tk_set_flg (ID id, UINT ptn)  { (void)id; g_flag |= ptn; return E_OK; }
ER tk_clr_flg (ID id, UINT ptn)  { (void)id; g_flag &= ptn; return E_OK; }

ER
tk_wai_flg (ID id, UINT ptn, UINT mode, UINT * p_ptn, TMO tmo)
{
    (void)id; (void)mode;

    if ((g_flag & ptn) != 0U)
    {
        *p_ptn = g_flag;
        g_flag = 0U;
        return E_OK;
    }

    if (g_idle_left-- <= 0)
    {
        longjmp(g_out, 1);
    }

    g_ms += (uint32_t)tmo;
    return E_TMOUT;
}

/* Let the supervisor run for about ms milliseconds of fake time. */
static void
run_for (uint32_t ms)
{
    g_idle_left = (int)(ms / SAFETY_CHECK_MS);

    if (setjmp(g_out) == 0)
    {
        g_task(0, NULL);
    }
}

/* ---------------------------------------------------------------------
 * Mock neighbours
 * ------------------------------------------------------------------ */
static int             g_inhibit_calls;
static bool            g_mc_inhibited;
static nav_safety_t    g_nav_safety[16];
static int             g_nav_safety_n;
static nav_cmd_t       g_nav_cmd;
static int             g_order_ok;      /* CLEARED seen before RESET */
static nav_state_t     g_nav_state = NAV_ST_READY;
static safety_status_t g_last;

void mc_inhibit (bool b) { g_mc_inhibited = b; g_inhibit_calls++; }
void safety_telemetry_publish (safety_status_t const * p) { g_last = *p; }
nav_state_t nav_get_state (void) { return g_nav_state; }

ER
nav_post_safety (nav_safety_t e)
{
    g_nav_safety[g_nav_safety_n++] = e;
    return E_OK;
}

ER
nav_post_command (nav_cmd_t c)
{
    g_nav_cmd  = c;
    g_order_ok = (g_nav_safety_n > 0) &&
                 (g_nav_safety[g_nav_safety_n - 1] == NAV_SAFETY_CLEARED);
    return E_OK;
}

static nav_safety_t last_nav (void) { return g_nav_safety[g_nav_safety_n - 1]; }

/* ---------------------------------------------------------------------
 * Checks
 * ------------------------------------------------------------------ */
static int g_pass, g_fail;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; }                                             \
        else { g_fail++; printf("  FAIL line %d: ", __LINE__);              \
               printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

static void
reset_all (void)
{
    (void)safety_request_reset();
    run_for(40);
}

int
main (void)
{
    if ((safety_init() != E_OK) || (safety_start() != E_OK))
    {
        printf("init failed\n");
        return EXIT_FAILURE;
    }
    run_for(40);

    printf("emergency stop, fast drive-off\n");
    (void)safety_post_estop();
    run_for(20);
    CHECK(g_mc_inhibited, "mc_inhibit(true)");
    CHECK(safety_drive_inhibited(), "inhibit flag for polling");
    CHECK(last_nav() == NAV_SAFETY_ESTOP, "coordinator told");
    CHECK(safety_get_state() == SAFETY_ST_ESTOP, "latched");
    CHECK(!nav_start_allowed(), "start refused while latched");
    g_ms += 30U;
    safety_ack_drive_off();
    run_for(20);
    CHECK((g_last.last_stop_ms >= 30U) && (g_last.last_stop_ms < 100U),
          "stop took %u ms", (unsigned)g_last.last_stop_ms);
    CHECK(g_last.slow_stops == 0U, "not slow");

    printf("reset after emergency stop\n");
    reset_all();
    CHECK(safety_get_state() == SAFETY_ST_NORMAL, "normal again");
    CHECK(!g_mc_inhibited && !safety_drive_inhibited(), "drive released");
    CHECK((g_nav_cmd == NAV_CMD_RESET) && g_order_ok,
          "CLEARED then RESET to coordinator");
    CHECK(nav_start_allowed(), "start allowed");

    printf("critical fault, slow drive-off\n");
    b5_report_fault(B5_FAULT_ECHO_TIMEOUT);
    run_for(160);
    CHECK(safety_get_state() == SAFETY_ST_FAULT, "fault latched");
    CHECK(last_nav() == NAV_SAFETY_FAULT, "coordinator told");
    CHECK((g_last.fault_src == SAFETY_SRC_OBSTACLE) &&
          (g_last.fault_code == (uint32_t)B5_FAULT_ECHO_TIMEOUT),
          "source and code recorded");
    CHECK(g_last.warn_code == SAFETY_CODE_STOP_SLOW,
          "slow stop flagged (> %u ms)", SAFETY_STOP_LIMIT_MS);
    safety_ack_drive_off();
    run_for(20);
    CHECK((g_last.slow_stops == 1U) && (g_last.max_stop_ms >= 100U),
          "slow stop measured %u ms", (unsigned)g_last.max_stop_ms);

    printf("emergency stop during a fault keeps the fault\n");
    (void)safety_post_estop();
    run_for(20);
    CHECK(safety_get_state() == SAFETY_ST_FAULT, "still fault");
    reset_all();

    printf("warnings do not stop the car\n");
    g_inhibit_calls = 0;
    b5_report_fault(B5_FAULT_LINE_LOST);
    nav_report_fault(NAV_FAULT_LINE_LOST);
    run_for(20);
    CHECK(safety_get_state() == SAFETY_ST_NORMAL, "still normal");
    CHECK(g_inhibit_calls == 0, "no inhibit");
    CHECK(g_last.warnings >= 2U, "warnings counted");
    nav_report_fault(NAV_FAULT_MOTION);
    run_for(20);
    CHECK(safety_get_state() == SAFETY_ST_FAULT, "motion failure is critical");
    reset_all();

    printf("health monitoring\n");
    (void)safety_watch(SAFETY_SRC_LINE, 100U);
    run_for(400);                       /* not running: no check-ins needed */
    CHECK(safety_get_state() == SAFETY_ST_NORMAL, "idle is fine");
    g_nav_state = NAV_ST_RUNNING;
    {
        int i;

        for (i = 0; i < 10; i++)        /* both tasks check in */
        {
            safety_kick(SAFETY_SRC_LINE);
            nav_heartbeat();
            run_for(40);
        }
    }
    CHECK(safety_get_state() == SAFETY_ST_NORMAL, "kicking: healthy");
    {
        int i;

        for (i = 0; i < 4; i++)         /* line task goes silent */
        {
            nav_heartbeat();
            run_for(40);
        }
    }
    CHECK(safety_get_state() == SAFETY_ST_FAULT, "silent task -> fault");
    CHECK((g_last.fault_src == SAFETY_SRC_LINE) &&
          (g_last.fault_code == SAFETY_CODE_TASK_SILENT),
          "silent task identified");

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
