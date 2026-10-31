/**
 * @file    test_b5_task.c
 * @brief   Host test of the statechart in task_obstacle.c against a simulated car.
 *
 * task_obstacle.c is compiled unchanged against a tiny mock kernel
 * (mock/tk/tkernel.h). The mock is single-threaded: "other tasks" answer
 * Buddy 5's requests immediately by posting events, and time advances only
 * when Buddy 5 sleeps or waits. Each scenario checks the exact sequence of
 * states against statechart 6.5.
 *
 * Usage: test_b5_task <scenario>   (run by the Makefile for each one)
 */

#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tk/tkernel.h>

#include "b5_config.h"
#include "sys_hal.h"
#include "b5_iface.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "sys_fault.h"

#define PI          (3.14159265358979)
#define MAX_TRACE   (64)

/* ---------------------------------------------------------------------
 * Simulated world
 * ------------------------------------------------------------------ */
typedef struct { double x_left, x_right, y; } face_t;

static uint32_t   g_now_us;
static int16_t    g_servo_dd = B5_AHEAD_DD;
static face_t     g_face[3];
static int        g_faces;
static int        g_pings_until_obstacle;
static int        g_ping_count;
static int        g_dead_sensor;
static int        g_line_found_leg;     /* leg on which the line appears */
static int        g_line_now;
static int32_t    g_odometer_mm;
static int32_t    g_odo_step = 20;
static int        g_reverses;
static int g_scenario, g_confirm_missed, g_confirm_valid, g_scan_triplet;
static int g_releases, g_plans, g_stop_injected, g_bypass_odo_reads;


static b5_state_t g_trace[MAX_TRACE];
static int        g_trace_len;
static b5_fault_t g_fault_reported;
static jmp_buf    g_end;

/* ---------------------------------------------------------------------
 * Mock kernel
 * ------------------------------------------------------------------ */
static UINT g_flag[4];
static int  g_flags;
static FP   g_task_fn;

ID tk_cre_flg (T_CFLG const * p) { g_flag[g_flags] = p->iflgptn; return ++g_flags; }
ER tk_set_flg (ID id, UINT ptn)  { g_flag[id - 1] |= ptn; return E_OK; }
ER tk_clr_flg (ID id, UINT ptn)  { g_flag[id - 1] &= ptn; return E_OK; }
ID tk_cre_tsk (T_CTSK const * p) { g_task_fn = p->task; return 1; }
ER tk_sta_tsk (ID id, INT s)     { (void)id; (void)s; return E_OK; }
ER tk_dly_tsk (TMO ms)           { g_now_us += (uint32_t)ms * 1000U; return E_OK; }
ID tk_cre_mtx (T_CMTX const * p) { (void)p; return 1; }
ER tk_loc_mtx (ID id, TMO t)     { (void)id; (void)t; return E_OK; }
ER tk_unl_mtx (ID id)            { (void)id; return E_OK; }

/* Shared fault bitmask (system/sys_fault.c) */
static uint32_t g_fault_mask;
ER sys_fault_set (uint32_t b)    { g_fault_mask |= b; return E_OK; }
ER sys_fault_clear (uint32_t b)  { g_fault_mask &= ~b; return E_OK; }

ER
tk_wai_flg (ID id, UINT ptn, UINT mode, UINT * p_ptn, TMO tmo)
{
    UINT hit = g_flag[id - 1] & ptn;

    if (hit == 0U)
    {
        if (tmo == TMO_FEVR)
        {
            longjmp(g_end, 1);      /* nothing will ever arrive: done */
        }
        g_now_us += (uint32_t)tmo * 1000U;
        return E_TMOUT;
    }

    *p_ptn = g_flag[id - 1];

    if ((mode & TWF_BITCLR) != 0U)
    {
        g_flag[id - 1] &= ~hit;
    }
    else if ((mode & TWF_CLR) != 0U)
    {
        g_flag[id - 1] = 0U;
    }

    return E_OK;
}

/* ---------------------------------------------------------------------
 * Mock hardware (sys_hal.h, drv_servo.h, drv_ultrasonic.h)
 * ------------------------------------------------------------------ */
ER       sys_hal_init (void)          { return E_OK; }
uint32_t sys_time_us (void)           { return g_now_us; }
void     sys_busy_wait_us (uint32_t u){ g_now_us += u; }
void     sys_unreset (uint32_t m)     { (void)m; }
ER       sys_gpio_irq_attach (uint32_t g, uint32_t e, sys_gpio_cb_t c, INT l)
{ (void)g; (void)e; (void)c; (void)l; return E_OK; }
ER       b5_servo_init (void)        { return E_OK; }
int16_t  b5_servo_angle (void)       { return g_servo_dd; }
ER       b5_us_init (void)           { return E_OK; }

ER
b5_servo_move (int16_t angle_dd)
{
    g_now_us  += 30000U;
    g_servo_dd = angle_dd;
    return E_OK;
}

static ER
sim_ping (int32_t * p_mm)
{
    /* Signed servo angle (+ = right) -> angle from the right-hand x axis. */
    double a    = ((900 - g_servo_dd) / 10.0) * PI / 180.0;
    double half = 7.5 * PI / 180.0;
    double best = B5_RANGE_FAR_MM;
    int    f;

    g_now_us += B5_PING_PERIOD_MS * 1000U;
    g_ping_count++;

    if (g_scenario == 5 && g_trace_len && g_trace[g_trace_len-1] == B5_ST_CONFIRM) {
        if (!g_confirm_missed) { g_confirm_missed=1; g_confirm_valid=0; return E_TMOUT; }
        g_confirm_valid++;
    }
    if (g_scenario == 6 && g_trace_len && g_trace[g_trace_len-1] == B5_ST_COARSE_SWEEP) {
        if ((g_scan_triplet++ % 3) < 2) return E_TMOUT;
    }
    if (g_dead_sensor)
    {
        return E_TMOUT;
    }

    if (g_ping_count < g_pings_until_obstacle)
    {
        *p_mm = B5_RANGE_FAR_MM;
        return E_OK;
    }

    for (f = 0; f < g_faces; f++)
    {
        double lo = fmax(a - half, atan2(g_face[f].y, g_face[f].x_right));
        double hi = fmin(a + half, atan2(g_face[f].y, g_face[f].x_left));
        double t;

        for (t = lo; t <= hi; t += 0.002)
        {
            best = fmin(best, g_face[f].y / sin(t));
        }
    }

    *p_mm = (int32_t)lround(best);
    return E_OK;
}

/* Rule B driver view: what the simulated sensor's ECHO line did.
 *   g_stuck_high : ECHO already high before TRIG and never low (fault)
 *   E_TMOUT      : silent / disconnected sensor, no rise (fault)
 *   FAR          : completed no-return (pulse longer than ECHO_MAX, fell)
 *   otherwise    : echo of that width */
static int g_stuck_high, g_stuck_from_sweep, g_unplug_from_sweep;

ER
b5_us_ping_ex (b5_ping_t * p)
{
    int32_t mm = 0;
    ER      er = sim_ping(&mm);
    int     in_scan = g_trace_len &&
        ((g_trace[g_trace_len-1] == B5_ST_COARSE_SWEEP) ||
         (g_trace[g_trace_len-1] == B5_ST_EDGE_REFINE));

    if (g_stuck_high || (g_stuck_from_sweep && in_scan))
    {
        *p = b5_classify_ping(false, false, false, false, 0U);
        return E_OK;
    }
    if ((er != E_OK) || (g_unplug_from_sweep && in_scan))
    {
        *p = b5_classify_ping(true, false, false, false, 0U);
        return E_OK;
    }
    if (mm >= B5_RANGE_FAR_MM)
    {
        *p = b5_classify_ping(true, true, true, false, B5_ECHO_MAX_US + 5000U);
        return E_OK;
    }
    *p = b5_classify_ping(true, true, true, false,
                          (uint32_t)(((mm * 2000) + 342) / 343));
    return E_OK;
}

ER
b5_us_ping (int32_t * p_mm)
{
    b5_ping_t p;
    (void)b5_us_ping_ex(&p);
    if (p.cls == (uint32_t)B5_PING_FAULT) { return E_TMOUT; }
    *p_mm = (int32_t)p.range_mm;
    return E_OK;
}

/* ---------------------------------------------------------------------
 * The rest of the car, answering instantly
 * ------------------------------------------------------------------ */
void b5_nav_request_slow (bool b) { (void)b; }
void b5_nav_release_motion (void) { g_releases++; }
void b5_nav_blocked_route (b5_profile_t const * p) { (void)p; }

void
b5_nav_request_stop (void)
{
    if (g_scenario == 5 && g_confirm_valid < 3) abort();
    (void)b5_post_event(B5_EV_STOP_ACCEPTED);
}

void
b5_nav_send_plan (b5_plan_t const * p)
{
    (void)p;
    g_plans++;
    g_faces = 0;                        /* drove round it */
    (void)b5_post_event(B5_EV_PLAN_ACCEPTED | B5_EV_MOTION_DONE);
}

void
b5_nav_request_rejoin (b5_rejoin_t const * p)
{
    g_line_now = (p->attempt == g_line_found_leg);
    (void)b5_post_event(B5_EV_MOTION_DONE);
}

void
b5_nav_request_reverse (int32_t mm)
{
    int f;
    /* Rule B sim: backing up moves every face away from the sensor. */
    for (f = 0; f < g_faces; f++) { g_face[f].y += (double)(mm < 0 ? -mm : mm); }
    g_reverses++;
    (void)b5_post_event(B5_EV_MOTION_DONE);
}

bool
b5_motion_get_estimate (b5_motion_est_t * p)
{
    if (g_scenario == 7 && g_trace_len && g_trace[g_trace_len-1] == B5_ST_BYPASSING) return false;
    if (g_scenario == 9 && g_trace_len && g_trace[g_trace_len-1] == B5_ST_BYPASSING && ++g_bypass_odo_reads >= 2) return false;
    /* 16-bit and wrapping, like Buddy 2's real estimate. The odometer
     * starts near the wrap point so every scenario crosses it. */
    p->distance_mm = (int16_t)(uint16_t)(uint32_t)g_odometer_mm;
    p->heading_dd  = 0;
    g_odometer_mm += g_odo_step;        /* car creeps forward */
    return true;
}

bool
b5_line_detected (void)
{
    return (g_line_now != 0);
}

void
b5_report_fault (b5_fault_t f)
{
    g_fault_reported = f;
}

/* Telemetry arrives as the packed binary frame; decode it like the PC. */
static int g_bad_frames;

void
b5_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    b5_tlm_t   t;
    b5_state_t state;

    if (!b5_tlm_unpack(p_frame, len, &t))
    {
        g_bad_frames++;
        return;
    }

    state = (b5_state_t)t.state;
    if (g_scenario == 8 && state == B5_ST_PLANNING && !g_stop_injected) {
        g_stop_injected=1; (void)b5_post_event(B5_EV_MISSION_STOP);
    }

    if ((g_trace_len == 0) || (g_trace[g_trace_len - 1] != state))
    {
        if (g_trace_len < MAX_TRACE)
        {
            g_trace[g_trace_len++] = state;
        }
    }

    /* After one full lap through Guard, stop the run. */
    if ((state == B5_ST_GUARD) && (g_trace_len > 3) &&
        (g_trace[g_trace_len - 2] == B5_ST_RECOVERING))
    {
        (void)b5_post_event(B5_EV_MISSION_STOP);
    }
}

/* ---------------------------------------------------------------------
 * Scenarios
 * ------------------------------------------------------------------ */
static int
expect (b5_state_t const * p_want, int n, b5_fault_t fault, uint32_t mask)
{
    int i;
    int ok = (g_trace_len >= n) && (g_fault_mask == mask);

    for (i = 0; ok && (i < n); i++)
    {
        ok = (g_trace[i] == p_want[i]);
    }

    ok = ok && (g_fault_reported == fault) && (g_bad_frames == 0);

    printf("  states:");
    for (i = 0; i < g_trace_len; i++)
    {
        printf(" %s%s", b5_state_name(g_trace[i]),
               (i + 1 < g_trace_len) ? " ->" : "");
    }
    printf("\n  fault reported: %d (expected %d), shared fault mask 0x%02X,"
           " bad frames %d\n", g_fault_reported, fault,
           (unsigned)g_fault_mask, g_bad_frames);
    printf("  %s\n", ok ? "PASS" : "FAIL");

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

int
main (int argc, char ** argv)
{
    int scenario = (argc > 1) ? atoi(argv[1]) : 1;
    g_scenario = scenario;

    g_odometer_mm          = 32000;     /* crosses the 16-bit wrap */
    g_line_found_leg       = 0;
    g_pings_until_obstacle = 5;
    g_faces                = 1;
    /* Face inside D_stop (was 280 with D_stop 300). Rule B swept-footprint
     * check: from a ~130 mm stop the 45 deg spin and diagonal of the grown
     * car (assumed pivot, 50 mm margin) meet the >= 200 mm planning
     * envelope, so every bypass first goes Blocked -> reverse -> rescan.
     * The reverse moves the simulated faces away (b5_nav_request_reverse). */
    g_face[0]              = (face_t){ -50.0, 50.0, (double)(B5_D_STOP_MM - 20) };
    g_odo_step             = 500;       /* line behind: a reverse is allowed */

    switch (scenario)
    {
        case 2:         /* sensor dead from the start */
            g_dead_sensor = 1;
            break;
        case 3:         /* boxed in: blocked, reverse twice, then Fault */
            g_faces   = 3;
            g_face[1] = (face_t){ 120.0, 1500.0, 330.0 };
            g_face[2] = (face_t){ -1500.0, -120.0, 330.0 };
            g_odo_step = 500;           /* plenty of line behind */
            break;
        case 4:         /* line never found: 3 legs, then LINE_LOST */
            g_line_found_leg = 99;
            break;
        case 6:         /* 2 of 3 pings fault per angle: no line behind, so
                         * Blocked stays put (no reverse) as before */
            g_odo_step = 20;
            break;
        case 10:        /* rule B: ECHO stuck high from the start */
            g_stuck_high = 1;
            break;
        case 11:        /* rule B: ECHO sticks high once the sweep starts */
            g_stuck_from_sweep = 1;
            break;
        case 12:        /* rule B: sensor unplugged once the sweep starts */
            g_unplug_from_sweep = 1;
            break;
        case 13:        /* rule B: same run, counting it: the first plan from the
                         * ~130 mm stop fails the footprint check, exactly one
                         * reverse, one rescan, exactly one plan sent */
            break;
        default:
            break;
    }

    if ((b5_init() != E_OK) || (b5_start() != E_OK))
    {
        printf("init failed\n");
        return EXIT_FAILURE;
    }

    (void)b5_post_event(B5_EV_MISSION_START);

    if (setjmp(g_end) == 0)
    {
        g_task_fn(0, NULL);
    }

    printf("scenario %d (%.1f s simulated)\n", scenario, g_now_us / 1e6);

    switch (scenario)
    {
        case 6: {
            int ok=g_releases==1 && g_plans==0 && g_trace[g_trace_len-1]==B5_ST_BLOCKED;
            printf("unknown scan stays stopped: %s\n",ok?"PASS":"FAIL");
            return ok?EXIT_SUCCESS:EXIT_FAILURE;
        }
        case 7: {
            int ok=g_plans==0 && g_fault_reported==B5_FAULT_MOTION;
            printf("missing motion estimate prevents bypass: %s\n",ok?"PASS":"FAIL");
            return ok?EXIT_SUCCESS:EXIT_FAILURE;
        }
        case 9: {
            int ok=g_plans==1 && g_fault_reported==B5_FAULT_MOTION;
            printf("lost bypass odometry stops/faults: %s\n",ok?"PASS":"FAIL");
            return ok?EXIT_SUCCESS:EXIT_FAILURE;
        }
        case 8: {
            int ok=g_stop_injected && g_plans==0 && g_releases==2 && g_trace[g_trace_len-1]==B5_ST_IDLE;
            printf("stop before planning prevents movement: %s\n",ok?"PASS":"FAIL");
            return ok?EXIT_SUCCESS:EXIT_FAILURE;
        }

        case 2:
        case 10:
        {
            static b5_state_t const want[] =
            { B5_ST_IDLE, B5_ST_GUARD, B5_ST_FAULT };
            int r = expect(want, 3, B5_FAULT_ECHO_TIMEOUT, SYS_FAULT_NO_ECHO);
            printf("  plans sent: %d (expected 0)\n", g_plans);
            return ((r == EXIT_SUCCESS) && (g_plans == 0)) ? EXIT_SUCCESS : EXIT_FAILURE;
        }
        case 11:
        case 12:
        {   /* three faulted pings at the first angle: sensor fault, never
             * read as open space, nothing planned */
            static b5_state_t const want[] =
            { B5_ST_IDLE, B5_ST_GUARD, B5_ST_CONFIRM, B5_ST_HALTING,
              B5_ST_COARSE_SWEEP, B5_ST_FAULT };
            int r = expect(want, 6, B5_FAULT_ECHO_TIMEOUT, SYS_FAULT_NO_ECHO);
            printf("  plans sent: %d (expected 0)\n", g_plans);
            return ((r == EXIT_SUCCESS) && (g_plans == 0)) ? EXIT_SUCCESS : EXIT_FAILURE;
        }
        case 3:
        {
            static b5_state_t const want[] =
            { B5_ST_IDLE, B5_ST_GUARD, B5_ST_CONFIRM, B5_ST_HALTING,
              B5_ST_COARSE_SWEEP, B5_ST_EDGE_REFINE, B5_ST_PLANNING,
              B5_ST_BLOCKED, B5_ST_HALTING, B5_ST_COARSE_SWEEP,
              B5_ST_EDGE_REFINE, B5_ST_PLANNING, B5_ST_BLOCKED,
              B5_ST_HALTING, B5_ST_COARSE_SWEEP, B5_ST_EDGE_REFINE,
              B5_ST_PLANNING, B5_ST_BLOCKED, B5_ST_FAULT };
            int r = expect(want, 19, B5_FAULT_BLOCKED, SYS_FAULT_OBSTACLE);
            printf("  reverses: %d (expected %u)\n", g_reverses,
                   B5_BLOCKED_RETRIES);
            return ((r == EXIT_SUCCESS) &&
                    (g_reverses == (int)B5_BLOCKED_RETRIES))
                   ? EXIT_SUCCESS : EXIT_FAILURE;
        }
        case 13:
        {
            static b5_state_t const want[] =
            { B5_ST_IDLE, B5_ST_GUARD, B5_ST_CONFIRM, B5_ST_HALTING,
              B5_ST_COARSE_SWEEP, B5_ST_EDGE_REFINE, B5_ST_PLANNING,
              B5_ST_BLOCKED, B5_ST_HALTING, B5_ST_COARSE_SWEEP,
              B5_ST_EDGE_REFINE, B5_ST_PLANNING, B5_ST_BYPASSING,
              B5_ST_RECOVERING, B5_ST_GUARD, B5_ST_IDLE };
            int r = expect(want, 16, B5_FAULT_NONE, 0U);
            printf("  reverses: %d (expected 1), plans: %d (expected 1)\n",
                   g_reverses, g_plans);
            return ((r == EXIT_SUCCESS) && (g_reverses == 1) && (g_plans == 1))
                   ? EXIT_SUCCESS : EXIT_FAILURE;
        }
        case 4:
        {
            static b5_state_t const want[] =
            { B5_ST_IDLE, B5_ST_GUARD, B5_ST_CONFIRM, B5_ST_HALTING,
              B5_ST_COARSE_SWEEP, B5_ST_EDGE_REFINE, B5_ST_PLANNING,
              B5_ST_BLOCKED, B5_ST_HALTING, B5_ST_COARSE_SWEEP,
              B5_ST_EDGE_REFINE, B5_ST_PLANNING,
              B5_ST_BYPASSING, B5_ST_RECOVERING, B5_ST_FAULT };
            return expect(want, 15, B5_FAULT_LINE_LOST, SYS_FAULT_OBSTACLE);
        }
        default:
        {
            static b5_state_t const want[] =
            { B5_ST_IDLE, B5_ST_GUARD, B5_ST_CONFIRM, B5_ST_HALTING,
              B5_ST_COARSE_SWEEP, B5_ST_EDGE_REFINE, B5_ST_PLANNING,
              B5_ST_BLOCKED, B5_ST_HALTING, B5_ST_COARSE_SWEEP,
              B5_ST_EDGE_REFINE, B5_ST_PLANNING,
              B5_ST_BYPASSING, B5_ST_RECOVERING, B5_ST_GUARD,
              B5_ST_IDLE };
            return expect(want, 16, B5_FAULT_NONE, 0U);
        }
    }
}
