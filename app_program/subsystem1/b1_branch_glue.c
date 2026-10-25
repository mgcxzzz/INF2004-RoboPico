/**
 * @file    b1_branch_glue.c
 * @brief   Lets the buddy1 branch build and run on its own.
 *
 * The shared code in app_program/system/ is an unchanged copy from the
 * buddy5 branch, minus sys_bench.c (which needs Buddy 3 and Buddy 5), and
 * subsystems 2-4 are still the template examples. This file stands in for
 * what is then missing:
 *  - sys_bench_init() / sys_bench_start(), called by system.c: the two
 *    buttons, GP20 = emergency stop, GP21 = reset and start (also on the
 *    Maker Pi Pico), same behaviour as sys_bench.c;
 *  - b5_post_event() / b5_get_state(), called by the coordinator;
 *  - tid_led, the template subsystem 4 monitor's "LED task" (IF-05): an
 *    idle task that drives no pin.
 *
 * Each part compiles only while the real file is ABSENT (__has_include),
 * so after the merge it compiles to nothing (b1_glue_start() stays and
 * returns E_OK). Delete it after the merge.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "subsystem1.h"
#include "../subsystem5/b5_iface.h"
#include "../system/nav_iface.h"
#include "../system/safety_iface.h"
#include "../system/sys_config.h"

/* =====================================================================
 * Template subsystem 4 monitor: give it a harmless "LED task"
 * ================================================================== */
#if __has_include("../subsystem4/task_monitor.c")

EXPORT ID tid_led;

static void
idle_led_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);     /* never drives GP19 */
    }
}

EXPORT ER
b1_glue_start (void)
{
    tid_led = app_start_task(idle_led_task, S1_PRI_LED, 0, "led_idle");
    return (tid_led < E_OK) ? tid_led : E_OK;
}

#else

EXPORT ER
b1_glue_start (void)
{
    return E_OK;
}

#endif /* template monitor present */

/* =====================================================================
 * Stand-in for system/sys_bench.c (buttons only; telemetry goes to MQTT)
 * ================================================================== */
#if !__has_include("../system/sys_bench.c")

#define BENCH_TICK_MS       (20)
#define BTN_ESTOP_GPIO      (20U)       /* button, active low */
#define BTN_RESET_GPIO      (21U)
#define PAD_PULL_UP         (1U << 3)
#define PAD_PULL_DOWN       (1U << 2)

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
