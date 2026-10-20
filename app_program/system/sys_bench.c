/**
 * @file    sys_bench.c
 * @brief   Bench helpers until every subsystem is written (sys_config.h).
 *
 * SYS_SIM_CAR         simulated Motion Control (S2): each motion command
 *                     "drives" a virtual car, integrating x (mm right of
 *                     the line), distance and heading every 20 ms, and
 *                     answers the coordinator. A wheel-speed target older
 *                     than NAV_LINE_STALE_MS stops the car, as the real
 *                     Motion Control must.
 * SYS_SIM_LINE        simulated Line and Barcode (S3) instead of the real
 *                     Buddy 3 task: proposes 150 mm/s while it has line
 *                     control, and sees the line within 15 mm of it.
 * SYS_CONSOLE_STATUS  prints coordinator, safety, Buddy 3 and Buddy 5
 *                     status (stands in for S1's telemetry).
 * SYS_BENCH_BUTTONS   Robo Pico buttons: GP20 emergency stop, GP21 reset
 *                     and start.
 *
 * With all three on, the real Buddy 5 sensor and servo, the Navigation
 * Coordinator and the Safety Supervisor run together on a bare Pico W:
 * hold something closer than 300 mm in front of the sensor and watch the
 * car stop, scan, go round it and rejoin the line on the console.
 *
 * Group IS28 (shared). Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../subsystem3/s3_iface.h"
#include "../subsystem3/s3_barcode.h"
#include "../subsystem5/b5_iface.h"
#include "../subsystem5/obstacle_profile.h"
#include "nav_iface.h"
#include "safety_iface.h"
#include "sys_config.h"

#define SIM_TICK_MS         (20)
#define SIM_LINE_MM_S       (150)
#define SIM_TURN_DD_PER_S   (900)       /* 90 deg/s in place          */
#define SIM_ON_LINE_MM      (15)
#define BTN_ESTOP_GPIO      (20U)       /* Robo Pico button, active low */
#define BTN_RESET_GPIO      (21U)
#define PAD_PULL_UP         (1U << 3)
#define PAD_PULL_DOWN       (1U << 2)

#if SYS_SIM_CAR

typedef enum
{
    SIM_IDLE = 0,
    SIM_WHEELS,                 /* continuous wheel speeds            */
    SIM_DRIVE,
    SIM_TURN,
    SIM_STOP
} sim_kind_t;

/* Simulated car. Written by the bench task and by the mc_*() calls from
 * the coordinator and safety tasks, possibly on the other core; the lock
 * keeps each update whole. */
static ID         g_lock = 0;
static sim_kind_t g_kind = SIM_IDLE;
static uint32_t   g_cmd_id = 0U;
static int32_t    g_left_mm = 0;        /* remaining distance or angle */
static int32_t    g_speed = 0;
static int32_t    g_x_mm1000 = 0;       /* lateral offset, um          */
static int32_t    g_dist_mm1000 = 0;
static int32_t    g_heading_dd = 0;
static uint32_t   g_wheels_ms = 0U;     /* when the last target came  */
#if SYS_SIM_LINE
static bool       g_b_line_ctl = false;
#endif

#endif /* SYS_SIM_CAR */

#if SYS_CONSOLE_STATUS
static nav_state_t g_last_nav = NAV_ST_COUNT;
static nav_owner_t g_last_owner = NAV_OWNER_COUNT;
static b5_state_t  g_last_b5 = B5_ST_COUNT;
#endif

/* =====================================================================
 * Simulated Motion Control (S2) and Line and Barcode (S3)
 * ================================================================== */
#if SYS_SIM_CAR

static void lock (void)   { (void)tk_wai_sem(g_lock, 1, TMO_FEVR); }
static void unlock (void) { (void)tk_sig_sem(g_lock, 1); }

/* Finish the current command and tell the coordinator. Lock held. */
static void
finish (nav_motion_result_t result)
{
    uint32_t id = g_cmd_id;

    g_kind   = SIM_IDLE;
    g_cmd_id = 0U;

    if (id != 0U)
    {
        (void)nav_post_motion_result(id, result);
    }
}

static void
start (sim_kind_t kind, int32_t amount, int32_t speed, uint32_t id)
{
    lock();

    if (g_cmd_id != 0U)
    {
        finish(NAV_MOTION_CANCELLED);   /* handover */
    }

    g_kind    = kind;
    g_left_mm = amount;
    g_speed   = speed;
    g_cmd_id  = id;
    unlock();
}

static uint32_t
now_ms (void)
{
    SYSTIM t;

    (void)tk_get_otm(&t);
    return (uint32_t)t.lo;
}

ER
mc_set_wheel_speeds (int16_t left_mm_s, int16_t right_mm_s)
{
    start(SIM_WHEELS, 0, (left_mm_s + right_mm_s) / 2, 0U);
    g_wheels_ms = now_ms();
    return E_OK;
}

ER
mc_drive_distance (int32_t distance_mm, int16_t speed_mm_s, uint32_t cmd_id)
{
    tm_printf((UB *)"  [motion] drive %d mm\n", (INT)distance_mm);
    start(SIM_DRIVE, distance_mm, speed_mm_s, cmd_id);
    return E_OK;
}

ER
mc_turn (int32_t angle_dd, int16_t speed_mm_s, uint32_t cmd_id)
{
    (void)speed_mm_s;
    tm_printf((UB *)"  [motion] turn %d deg\n", (INT)(angle_dd / 10));
    start(SIM_TURN, angle_dd, SIM_TURN_DD_PER_S, cmd_id);
    return E_OK;
}

ER
mc_stop (uint32_t cmd_id)
{
    start(SIM_STOP, 0, 0, cmd_id);
    return E_OK;
}

bool
b5_motion_get_estimate (b5_motion_est_t * p_est)
{
    /* 16-bit wrapping values, as agreed with Buddy 2 (b5_iface.h). */
    lock();
    p_est->distance_mm = (int16_t)(uint16_t)(g_dist_mm1000 / 1000);
    p_est->heading_dd  = (int16_t)(uint16_t)g_heading_dd;
    unlock();
    return true;
}

#if SYS_SIM_LINE
void
line_set_control (bool b_granted)
{
    g_b_line_ctl = b_granted;
}

bool
b5_line_detected (void)
{
    int32_t x;

    lock();
    x = g_x_mm1000 / 1000;
    unlock();

    return ((x > -SIM_ON_LINE_MM) && (x < SIM_ON_LINE_MM));
}
#endif /* SYS_SIM_LINE */

/* Simulated Motion Control obeys the inhibit at once. */
void
mc_inhibit (bool b_inhibit)
{
    if (b_inhibit)
    {
        lock();
        finish(NAV_MOTION_CANCELLED);
        g_kind  = SIM_IDLE;
        g_speed = 0;
        unlock();
        safety_ack_drive_off();
    }
}

static void
sim_step (void)
{
    int32_t step;

    lock();

    /* Stale-target timeout: wheel speeds not renewed -> stop. */
    if ((g_kind == SIM_WHEELS) &&
        ((now_ms() - g_wheels_ms) > NAV_LINE_STALE_MS))
    {
        g_kind  = SIM_IDLE;
        g_speed = 0;
    }

    switch (g_kind)
    {
        case SIM_WHEELS:
        case SIM_DRIVE:
            step = (g_speed * SIM_TICK_MS);             /* um this tick */

            if (g_kind == SIM_DRIVE)
            {
                int32_t remain = g_left_mm * 1000;

                if (remain < 0)
                {
                    step = -step;
                }

                if (((remain >= 0) && (step > remain)) ||
                    ((remain < 0) && (step < remain)))
                {
                    step = remain;
                }

                g_left_mm -= step / 1000;
            }

            g_dist_mm1000 += step;
            g_x_mm1000    += (step * b5_sin_q15(g_heading_dd)) / 32768;

            if ((g_kind == SIM_DRIVE) && (g_left_mm == 0))
            {
                finish(NAV_MOTION_DONE);
            }
            break;

        case SIM_TURN:
            step = (SIM_TURN_DD_PER_S * SIM_TICK_MS) / 1000;

            if (g_left_mm < 0)
            {
                step = -step;
            }

            if (((g_left_mm >= 0) && (step > g_left_mm)) ||
                ((g_left_mm < 0) && (step < g_left_mm)))
            {
                step = g_left_mm;
            }

            g_heading_dd += step;
            g_left_mm    -= step;

            if (g_left_mm == 0)
            {
                finish(NAV_MOTION_DONE);
            }
            break;

        case SIM_STOP:
            finish(NAV_MOTION_DONE);
            break;

        default:
            break;
    }

    unlock();
}

#endif /* SYS_SIM_CAR */

/* =====================================================================
 * Status on the console (stands in for S1 telemetry)
 * ================================================================== */
#if SYS_CONSOLE_STATUS

/* Every subsystem sends binary frames in the team schema (sys_tlm.h);
 * decode them the same way the PC will, then print. */
void
nav_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    nav_tlm_t t;

    if (!nav_tlm_unpack(p_frame, len, &t))
    {
        tm_printf((UB *)"[nav] bad telemetry frame\n");
        return;
    }

    if (((nav_state_t)t.state != g_last_nav) ||
        ((nav_owner_t)t.owner != g_last_owner))
    {
        tm_printf((UB *)"[nav] %s, motion owner: %s%s\n",
                  (UB *)nav_state_name((nav_state_t)t.state),
                  (UB *)nav_owner_name((nav_owner_t)t.owner),
                  t.b_slow ? " (slow)" : "");
        g_last_nav   = (nav_state_t)t.state;
        g_last_owner = (nav_owner_t)t.owner;
    }
}

/* Buddy 3's frame (schema in sys_tlm.h, source 3). */
void
line_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    static uint8_t last_state = 0xFFU;
    static uint8_t last_ok = 0U;
    static uint8_t last_bad = 0U;
    static uint8_t last_back = 0U;
    s3_tlm_t       t;

    if (!s3_tlm_unpack(p_frame, len, &t))
    {
        tm_printf((UB *)"[line] bad telemetry frame\n");
        return;
    }

#if SYS_CONSOLE_SENSORS
    {
        static uint8_t last_sensors = 0xFFU;

        if (t.sensors != last_sensors)
        {
            tm_printf((UB *)"[line] sensors  L%u R%u B%u   (1 = black)\n",
                      (UINT)(t.sensors & 1U), (UINT)((t.sensors >> 1) & 1U),
                      (UINT)((t.sensors >> 2) & 1U));
            last_sensors = t.sensors;
        }
    }
#endif

    if (t.state != last_state)
    {
        tm_printf((UB *)"[line] %s  (sensors L%u R%u B%u)\n",
                  (UB *)s3_state_name((s3_state_t)t.state),
                  (UINT)(t.sensors & 1U), (UINT)((t.sensors >> 1) & 1U),
                  (UINT)((t.sensors >> 2) & 1U));
        last_state = t.state;
    }

    if ((t.barcodes_ok != last_ok) || (t.bad_reads != last_bad) ||
        (t.backwards != last_back))
    {
        tm_printf((UB *)"[line] barcode '%c': result %u, command %u "
                        "(ok %u, bad %u, backwards %u)\n",
                  (INT)s3_c39_at(t.last_char), (UINT)t.last_result,
                  (UINT)t.last_cmd, (UINT)t.barcodes_ok, (UINT)t.bad_reads,
                  (UINT)t.backwards);
        last_ok   = t.barcodes_ok;
        last_bad  = t.bad_reads;
        last_back = t.backwards;
    }
}

/* Buddy 5's frame (schema in sys_tlm.h, source 5); decode it the same
 * way the PC will, then print it. */
void
b5_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b5_tlm_t t;

    if (!b5_tlm_unpack(p_frame, len, &t))
    {
        tm_printf((UB *)"[b5]  bad telemetry frame\n");
        return;
    }

#if SYS_CONSOLE_SENSORS
    tm_printf((UB *)"[b5]  range %d mm\n", (INT)t.range_mm);
#endif

    if ((b5_state_t)t.state != g_last_b5)
    {
        tm_printf((UB *)"[b5]  %s  (range %d mm)\n",
                  (UB *)b5_state_name((b5_state_t)t.state),
                  (INT)t.range_mm);
        g_last_b5 = (b5_state_t)t.state;

        if (((b5_state_t)t.state == B5_ST_PLANNING) && t.b_found)
        {
            tm_printf((UB *)"[b5]  obstacle %d mm wide at %d mm, "
                            "go %s\n", (INT)t.width_mm,
                      (INT)t.closest_mm,
                      (t.side == (uint8_t)B5_SIDE_RIGHT) ? "right" :
                      (t.side == (uint8_t)B5_SIDE_LEFT) ? "left" : "nowhere");
        }
    }
}

void
safety_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    static safety_state_t last = SAFETY_ST_COUNT;
    static uint8_t        faults = 0U;
    static uint8_t        warnings = 0U;
    static uint16_t       stop_ms = 0xFFFFU;
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

    /* 8-bit counters wrap: a change, not a size, means "new event". */
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

    if (t.last_stop_ms != stop_ms)
    {
        tm_printf((UB *)"[safety] drive off %u ms after the stop "
                        "(worst %u ms, limit 100)\n",
                  (UINT)t.last_stop_ms, (UINT)t.max_stop_ms);
        stop_ms = t.last_stop_ms;
    }

    if (t.fault_mask != mask)
    {
        tm_printf((UB *)"[safety] shared fault mask 0x%02x\n",
                  (UINT)t.fault_mask);
        mask = t.fault_mask;
    }
}

#endif /* SYS_CONSOLE_STATUS */

/* =====================================================================
 * Robo Pico buttons: GP20 emergency stop, GP21 reset and start
 * ================================================================== */
#if SYS_BENCH_BUTTONS

static void
buttons_init (void)
{
    (void)gpio_set_pin(BTN_ESTOP_GPIO, GPIO_MODE_IN);
    (void)gpio_set_pin(BTN_RESET_GPIO, GPIO_MODE_IN);
    clr_w(GPIO(BTN_ESTOP_GPIO), PAD_PULL_DOWN);
    set_w(GPIO(BTN_ESTOP_GPIO), PAD_PULL_UP);
    clr_w(GPIO(BTN_RESET_GPIO), PAD_PULL_DOWN);
    set_w(GPIO(BTN_RESET_GPIO), PAD_PULL_UP);
}

/* Call every tick: act on a press (high -> low). */
static void
buttons_poll (void)
{
    static UINT last_estop = 1U;
    static UINT last_reset = 1U;
    UINT        estop = gpio_get_val(BTN_ESTOP_GPIO);
    UINT        reset = gpio_get_val(BTN_RESET_GPIO);

    if ((last_estop == 1U) && (estop == 0U))
    {
        tm_printf((UB *)"\n[button] EMERGENCY STOP\n");
        (void)safety_post_estop();
    }

    if ((last_reset == 1U) && (reset == 0U))
    {
        tm_printf((UB *)"\n[button] reset and start\n");
        (void)safety_request_reset();       /* clears a latched stop */
        (void)nav_post_command(NAV_CMD_RESET);
        (void)nav_post_command(NAV_CMD_START);
    }

    last_estop = estop;
    last_reset = reset;
}

#endif /* SYS_BENCH_BUTTONS */

/* =====================================================================
 * Bench task
 * ================================================================== */
#if SYS_SIM_CAR || SYS_BENCH_BUTTONS

static void
bench_task (INT stacd, void * p_exinf)
{
    uint32_t tick = 0U;

    (void)stacd;
    (void)p_exinf;

#if SYS_BENCH_BUTTONS
    buttons_init();
    tm_printf((UB *)"[bench] GP20 button = emergency stop, "
                    "GP21 = reset and start\n");
#endif

#if SYS_SIM_CAR
    tm_printf((UB *)"[bench] simulated car follows the line at %d mm/s\n",
              (INT)SIM_LINE_MM_S);
#endif

    for (;;)
    {
#if SYS_SIM_CAR
        sim_step();

        if ((SYS_SIM_AUTOSTART_MS != 0) &&
            (tick == (uint32_t)(SYS_SIM_AUTOSTART_MS / SIM_TICK_MS)))
        {
            tm_printf((UB *)"[bench] starting a run\n");
            (void)nav_post_command(NAV_CMD_START);
        }

#if SYS_SIM_LINE
        /* Line follower: propose speeds every tick while it has control
         * (faster than the stale-target timeout), and straighten up along
         * the line when it gets control back. */
        if (g_b_line_ctl)
        {
            lock();
            g_heading_dd = 0;
            unlock();
            (void)nav_line_drive(SIM_LINE_MM_S, SIM_LINE_MM_S);
        }
#endif
#endif

#if SYS_BENCH_BUTTONS
        buttons_poll();
#endif

        tick++;
        (void)tk_dly_tsk(SIM_TICK_MS);
    }
}

#endif /* SYS_SIM_CAR || SYS_BENCH_BUTTONS */

/**
 * @brief  Phase 1: the simulation's lock. Called by system_init().
 */
EXPORT ER
sys_bench_init (void)
{
#if SYS_SIM_CAR
    T_CSEM csem = { 0 };

    csem.sematr  = TA_TFIFO | TA_FIRST;
    csem.isemcnt = 1;
    csem.maxsem  = 1;
    g_lock       = tk_cre_sem(&csem);

    return (g_lock < E_OK) ? g_lock : E_OK;
#else
    return E_OK;
#endif
}

/**
 * @brief  Phase 2: the bench task, if any switch needs it.
 */
EXPORT ER
sys_bench_start (void)
{
#if SYS_SIM_CAR || SYS_BENCH_BUTTONS
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
