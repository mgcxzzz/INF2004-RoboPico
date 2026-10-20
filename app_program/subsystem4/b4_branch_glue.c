/**
 * @file    b4_branch_glue.c
 * @brief   Lets the buddy4 branch build and run on its own.
 *
 * The shared code in app_program/system/ is an unchanged copy from the
 * buddy5 branch, minus sys_bench.c (which needs Buddy 3 and Buddy 5). This
 * file stands in for the few things that are then missing:
 *  - sys_bench_init() / sys_bench_start(), called by system.c, with the
 *    Robo Pico buttons (GP20 emergency stop, GP21 reset and start) and the
 *    console printout of the Navigation Coordinator and Safety Supervisor
 *    frames, same behaviour as sys_bench.c;
 *  - b5_post_event() / b5_get_state(), called by the coordinator.
 *
 * Each part compiles only while the real file is ABSENT (__has_include),
 * so after merging with buddy3 / buddy5 this file compiles to nothing and
 * there is no "multiple definition" and no merge conflict. Delete it after
 * the merge.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../subsystem5/b5_iface.h"
#include "../system/nav_iface.h"
#include "../system/safety_iface.h"
#include "../system/sys_config.h"

/* =====================================================================
 * Stand-in for system/sys_bench.c
 * ================================================================== */
#if !__has_include("../system/sys_bench.c")

#define BENCH_TICK_MS       (20)
#define BTN_ESTOP_GPIO      (20U)       /* Robo Pico button, active low */
#define BTN_RESET_GPIO      (21U)
#define PAD_PULL_UP         (1U << 3)
#define PAD_PULL_DOWN       (1U << 2)

#if SYS_CONSOLE_STATUS

void
nav_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    static nav_state_t last_nav = NAV_ST_COUNT;
    static nav_owner_t last_owner = NAV_OWNER_COUNT;
    nav_tlm_t          t;

    if (!nav_tlm_unpack(p_frame, len, &t))
    {
        tm_printf((UB *)"[nav] bad telemetry frame\n");
        return;
    }

    if (((nav_state_t)t.state != last_nav) ||
        ((nav_owner_t)t.owner != last_owner))
    {
        tm_printf((UB *)"[nav] %s, motion owner: %s\n",
                  (UB *)nav_state_name((nav_state_t)t.state),
                  (UB *)nav_owner_name((nav_owner_t)t.owner));
        last_nav   = (nav_state_t)t.state;
        last_owner = (nav_owner_t)t.owner;
    }
}

void
safety_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    static safety_state_t last = SAFETY_ST_COUNT;
    static uint8_t        faults = 0U;
    static uint8_t        warnings = 0U;
    static uint8_t        mask = 0U;
    safety_tlm_t          t;

    if (!safety_tlm_unpack(p_frame, len, &t))
    {
        tm_printf((UB *)"[safety] bad telemetry frame\n");
        return;
    }

    if ((safety_state_t)t.state != last)
    {
        tm_printf((UB *)"[safety] %s%s\n",
                  (UB *)safety_state_name((safety_state_t)t.state),
                  t.b_inhibit ? ", motor drive inhibited" : "");
        last = (safety_state_t)t.state;
    }

    if (t.faults != faults)
    {
        tm_printf((UB *)"[safety] fault from source %d, code 0x%x\n",
                  (INT)t.fault_src, (UINT)t.fault_code);
        faults = t.faults;
    }

    if (t.warnings != warnings)
    {
        tm_printf((UB *)"[safety] warning from source %d, code 0x%x\n",
                  (INT)t.warn_src, (UINT)t.warn_code);
        warnings = t.warnings;
    }

    if (t.fault_mask != mask)
    {
        tm_printf((UB *)"[safety] shared fault mask 0x%02x\n",
                  (UINT)t.fault_mask);
        mask = t.fault_mask;
    }
}

#endif /* SYS_CONSOLE_STATUS */

#if SYS_BENCH_BUTTONS

static void
bench_task (INT stacd, void * p_exinf)
{
    UINT last_estop = 1U;
    UINT last_reset = 1U;

    (void)stacd;
    (void)p_exinf;

    (void)gpio_set_pin(BTN_ESTOP_GPIO, GPIO_MODE_IN);
    (void)gpio_set_pin(BTN_RESET_GPIO, GPIO_MODE_IN);
    clr_w(GPIO(BTN_ESTOP_GPIO), PAD_PULL_DOWN);
    set_w(GPIO(BTN_ESTOP_GPIO), PAD_PULL_UP);
    clr_w(GPIO(BTN_RESET_GPIO), PAD_PULL_DOWN);
    set_w(GPIO(BTN_RESET_GPIO), PAD_PULL_UP);
    tm_printf((UB *)"[bench] GP20 button = emergency stop, "
                    "GP21 = reset and start\n");

    for (;;)
    {
        UINT estop = gpio_get_val(BTN_ESTOP_GPIO);
        UINT reset = gpio_get_val(BTN_RESET_GPIO);

        if ((last_estop == 1U) && (estop == 0U))
        {
            tm_printf((UB *)"\n[button] EMERGENCY STOP\n");
            (void)safety_post_estop();
        }

        if ((last_reset == 1U) && (reset == 0U))
        {
            tm_printf((UB *)"\n[button] reset and start\n");
            (void)safety_request_reset();
            (void)nav_post_command(NAV_CMD_RESET);
            (void)nav_post_command(NAV_CMD_START);
        }

        last_estop = estop;
        last_reset = reset;
        (void)tk_dly_tsk(BENCH_TICK_MS);
    }
}

#endif /* SYS_BENCH_BUTTONS */

EXPORT ER
sys_bench_init (void)
{
    return E_OK;
}

EXPORT ER
sys_bench_start (void)
{
#if SYS_BENCH_BUTTONS
    T_CTSK ctsk = { 0 };
    ID     tid;

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = bench_task;
    ctsk.itskpri = SYS_BENCH_PRIORITY;
    ctsk.stksz   = SYS_BENCH_STACK_BYTES;
    tid          = tk_cre_tsk(&ctsk);

    return (tid < E_OK) ? tid : tk_sta_tsk(tid, 0);
#else
    return E_OK;
#endif
}

#endif /* no system/sys_bench.c */

/* =====================================================================
 * Stand-in for Buddy 5's task (subsystem5/task_obstacle.c)
 * ================================================================== */
#if !__has_include("../subsystem5/task_obstacle.c")

ER
b5_post_event (UINT events)
{
    (void)events;
    return E_OK;
}

b5_state_t
b5_get_state (void)
{
    return B5_ST_IDLE;
}

#endif /* no subsystem5/task_obstacle.c */
