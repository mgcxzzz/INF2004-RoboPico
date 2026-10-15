/**
 * @file    task_obstacle.c
 * @brief   Obstacle Scan and Recovery task: statechart 6.5 of the report.
 *
 *   Idle -> Guard -> Confirm -> Halting -> Scanning{Coarse Sweep,
 *   Edge Refinement} -> Planning -> Bypassing -> Recovering -> Guard
 *   with failure routes Blocked and Fault.
 *
 * One function per state. Each runs the state's entry actions, then its
 * "do" loop, and returns the next state. The task owns the HC-SR04+ and
 * the SG90 and nothing else; every motion is requested from Navigation
 * Coordinator through b5_iface.h.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>

#include "b5_config.h"
#include "../system/sys_hal.h"
#include "b5_iface.h"
#include "obstacle_profile.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"

/* Everything the task remembers between states. Only the task touches it. */
typedef struct
{
    b5_status_t     status;         /* published copy of the state      */
    uint32_t        no_echo_run;    /* consecutive pings with no reply  */
    uint8_t         confirm;        /* consecutive readings < D_stop    */
    bool            b_slow;         /* slow request currently active    */
    bool            b_off_line;     /* car has left the line to bypass  */
    b5_scan_t       scan;
    b5_plan_t       plan;
    b5_motion_est_t est_ref;        /* estimate when the car left line  */
    int32_t         last_dist_mm;
    int32_t         along_mm;       /* dead reckoning in the line frame */
    int32_t         lateral_mm;     /* positive = right of the line     */
    int32_t         guard_dist_mm;  /* distance at Guard entry          */
    uint32_t        last_pub_us;
} b5_ctx_t;

static b5_ctx_t g_ctx;
static ID       g_event_flg = 0;
static ID       g_task_id   = 0;

static char const * const g_state_name[B5_ST_COUNT] =
{
    "Idle", "Guard", "Confirm", "Halting", "Coarse Sweep",
    "Edge Refinement", "Planning", "Bypassing", "Recovering",
    "Blocked", "Fault"
};

/* ---------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

/**
 * @brief  Current state. A single word read, safe from any task.
 */
b5_state_t
b5_get_state (void)
{
    return g_ctx.status.state;
}

char const *
b5_state_name (b5_state_t state)
{
    return (state < B5_ST_COUNT) ? g_state_name[state] : "?";
}

/**
 * @brief  Post events to Buddy 5. Safe from any task or interrupt handler.
 */
ER
b5_post_event (UINT events)
{
    if (g_event_flg <= 0)
    {
        return E_NOEXS;
    }

    return tk_set_flg(g_event_flg, events & B5_EV_ALL);
}

/* Wait for any of the events in mask; consume only those. 0 on timeout. */
static UINT
take_events (UINT mask, TMO tmo)
{
    UINT ptn = 0U;
    ER   er  = tk_wai_flg(g_event_flg, mask, TWF_ORW | TWF_BITCLR, &ptn, tmo);

    return (er == E_OK) ? (ptn & mask) : 0U;
}

/* Drop stale replies before sending a new request that expects them. */
static void
clear_events (UINT mask)
{
    (void)tk_clr_flg(g_event_flg, ~mask);
}

static bool
mission_stopped (void)
{
    return (take_events(B5_EV_MISSION_STOP, TMO_POL) != 0U);
}

static void
publish (bool b_force)
{
    uint32_t now = sys_time_us();

    if (b_force ||
        ((now - g_ctx.last_pub_us) >= (B5_STATUS_PERIOD_MS * 1000U)))
    {
        g_ctx.status.servo_dd = b5_servo_angle();
        g_ctx.last_pub_us     = now;
        b5_telemetry_publish(&g_ctx.status);
    }
}

static b5_state_t
to_fault (b5_fault_t fault)
{
    g_ctx.status.fault = fault;
    return B5_ST_FAULT;
}

/*
 * One ping, with fault counting. Returns false if there was no reading;
 * *p_fault is set once B5_ECHO_FAIL_LIMIT pings in a row got no reply.
 */
static bool
range_once (int32_t * p_mm, bool * p_fault)
{
    ER er = b5_us_ping(p_mm);

    g_ctx.status.pings++;

    if (er == E_OK)
    {
        g_ctx.no_echo_run = 0U;
        return true;
    }

    g_ctx.status.no_echo++;
    g_ctx.no_echo_run++;
    *p_fault = (g_ctx.no_echo_run >= B5_ECHO_FAIL_LIMIT);

    return false;
}

/*
 * Point the servo and take a median of n pings.
 * Returns the fault to raise, or B5_FAULT_NONE.
 */
static b5_fault_t
sample_at (int16_t angle_dd, uint8_t n_pings, b5_sample_t * p_out)
{
    int32_t reading[B5_SAMPLES_PER_ANGLE];
    uint8_t got = 0U;
    uint8_t i;
    bool    b_fault = false;

    if (n_pings > B5_SAMPLES_PER_ANGLE)
    {
        n_pings = B5_SAMPLES_PER_ANGLE;
    }

    if (b5_servo_move(angle_dd) != E_OK)
    {
        return B5_FAULT_SERVO;
    }

    for (i = 0U; i < n_pings; i++)
    {
        int32_t mm;

        if (range_once(&mm, &b_fault))
        {
            reading[got] = mm;
            got++;
        }
        else if (b_fault)
        {
            return B5_FAULT_ECHO_TIMEOUT;
        }
    }

    p_out->angle_dd = angle_dd;
    p_out->b_valid  = (got > 0U) && ((got * 2U) > n_pings);

    if (got >= 3U)
    {
        p_out->range_mm = b5_median3(reading[0], reading[1], reading[2]);
    }
    else if (got == 2U)
    {
        p_out->range_mm = (reading[0] < reading[1]) ? reading[0] : reading[1];
    }
    else
    {
        p_out->range_mm = (got == 1U) ? reading[0] : B5_RANGE_FAR_MM;
    }

    return B5_FAULT_NONE;
}

/* --- dead reckoning in the frame of the line the car left ---------- */

static void
pose_reset (void)
{
    b5_motion_est_t est;

    (void)b5_motion_get_estimate(&est);
    g_ctx.est_ref      = est;
    g_ctx.last_dist_mm = est.distance_mm;
    g_ctx.along_mm     = 0;
    g_ctx.lateral_mm   = 0;
}

static void
pose_update (void)
{
    b5_motion_est_t est;
    int32_t         step;
    int32_t         rel_heading;

    if (!b5_motion_get_estimate(&est))
    {
        return;
    }

    step               = est.distance_mm - g_ctx.last_dist_mm;
    g_ctx.last_dist_mm = est.distance_mm;
    rel_heading        = est.heading_dd - g_ctx.est_ref.heading_dd;

    g_ctx.along_mm   += (step * b5_cos_q15(rel_heading)) / 32768;
    g_ctx.lateral_mm += (step * b5_sin_q15(rel_heading)) / 32768;
}

/* Reversing is safe if the car has just driven that stretch of line. */
static bool
reverse_is_safe (void)
{
    b5_motion_est_t est;

    if (!b5_motion_get_estimate(&est))
    {
        return false;
    }

    return ((est.distance_mm - g_ctx.guard_dist_mm) >=
            (B5_REVERSE_MM + B5_MARGIN_MM));
}

static b5_params_t
profile_params (void)
{
    b5_params_t p;

    p.near_mm           = B5_NEAR_MM;
    p.alpha_dd          = B5_ALPHA_DD;
    p.scan_min_dd       = B5_SCAN_START_DD;
    p.scan_max_dd       = B5_SCAN_END_DD;
    p.car_width_mm      = B5_CAR_WIDTH_MM;
    p.car_length_mm     = B5_CAR_LENGTH_MM;
    p.margin_mm         = B5_MARGIN_MM;
    p.obst_depth_mm     = B5_OBST_DEPTH_MM;
    p.open_clearance_mm = B5_OPEN_CLEARANCE_MM;
    p.b_prefer_right    = (B5_PREFER_RIGHT != 0);

    return p;
}

/* ---------------------------------------------------------------------
 * States
 * ------------------------------------------------------------------ */

/* Idle: entry / servo 90 deg, ranging off. */
static b5_state_t
st_idle (void)
{
    (void)b5_servo_move(B5_AHEAD_DD);

    if (g_ctx.b_slow)
    {
        b5_nav_request_slow(false);
        g_ctx.b_slow = false;
    }

    b5_nav_release_motion();
    g_ctx.b_off_line = false;

    /* Wait for the start; a stop that arrives while idle is consumed
     * here so it cannot end the next run the moment it begins. */
    while ((take_events(B5_EV_MISSION_START | B5_EV_MISSION_STOP, TMO_FEVR)
            & B5_EV_MISSION_START) == 0U)
    {
        /* still idle */
    }

    clear_events(B5_EV_ALL & ~B5_EV_MISSION_STOP);

    return B5_ST_GUARD;
}

/* Guard: entry / park servo at 90 deg; do / ping every 60 ms. */
static b5_state_t
st_guard (void)
{
    b5_motion_est_t est;

    if (b5_servo_move(B5_AHEAD_DD) != E_OK)
    {
        return to_fault(B5_FAULT_SERVO);
    }

    g_ctx.confirm                = 0U;
    g_ctx.status.blocked_tries   = 0U;
    g_ctx.status.recover_tries   = 0U;
    (void)b5_motion_get_estimate(&est);
    g_ctx.guard_dist_mm = est.distance_mm;

    for (;;)
    {
        int32_t mm;
        bool    b_fault = false;

        if (mission_stopped())
        {
            return B5_ST_IDLE;
        }

        if (!range_once(&mm, &b_fault))
        {
            if (b_fault)
            {
                /* No forward sensing: never keep driving blind. */
                return to_fault(B5_FAULT_ECHO_TIMEOUT);
            }
            continue;
        }

        g_ctx.status.range_mm = mm;

        /* FR-USO-02: slow down before stopping. */
        if (!g_ctx.b_slow && (mm < B5_D_SLOW_MM))
        {
            b5_nav_request_slow(true);
            g_ctx.b_slow = true;
        }
        else if (g_ctx.b_slow && (mm > (B5_D_SLOW_MM + B5_SLOW_HYST_MM)))
        {
            b5_nav_request_slow(false);
            g_ctx.b_slow = false;
        }

        if (mm < B5_D_STOP_MM)
        {
            g_ctx.confirm = 1U;
            return B5_ST_CONFIRM;           /* range < D_stop */
        }

        publish(false);
    }
}

/* Confirm: do / count consecutive sub-threshold readings. */
static b5_state_t
st_confirm (void)
{
    for (;;)
    {
        int32_t mm;
        bool    b_fault = false;

        if (mission_stopped())
        {
            return B5_ST_IDLE;
        }

        if (!range_once(&mm, &b_fault))
        {
            if (b_fault)
            {
                return to_fault(B5_FAULT_ECHO_TIMEOUT);
            }
            continue;
        }

        g_ctx.status.range_mm = mm;

        if (mm >= B5_D_STOP_MM)
        {
            return B5_ST_GUARD;             /* reading >= D_stop */
        }

        g_ctx.confirm++;

        if (g_ctx.confirm >= B5_CONFIRM_COUNT)
        {
            return B5_ST_HALTING;           /* 3 consecutive < D_stop */
        }
    }
}

/* Halting: entry / request stop; do / start coarse sweep. */
static b5_state_t
st_halting (void)
{
    UINT ev;

    clear_events(B5_EV_STOP_ACCEPTED);
    b5_nav_request_stop();

    /* Swing to the first sweep angle while the car decelerates. */
    if (b5_servo_move(B5_SCAN_START_DD) != E_OK)
    {
        return to_fault(B5_FAULT_SERVO);
    }

    ev = take_events(B5_EV_STOP_ACCEPTED | B5_EV_MISSION_STOP,
                     B5_STOP_ACCEPT_TMO_MS);

    if ((ev & B5_EV_MISSION_STOP) != 0U)
    {
        return B5_ST_IDLE;
    }

    if ((ev & B5_EV_STOP_ACCEPTED) == 0U)
    {
        return to_fault(B5_FAULT_STOP_TIMEOUT);
    }

    return B5_ST_COARSE_SWEEP;              /* stop accepted */
}

/* Coarse Sweep: do / 30-150 deg in 30 deg steps. */
static b5_state_t
st_coarse_sweep (void)
{
    int16_t angle;

    b5_scan_clear(&g_ctx.scan);

    for (angle = B5_SCAN_START_DD; angle <= B5_SCAN_END_DD;
         angle = (int16_t)(angle + B5_SCAN_STEP_DD))
    {
        b5_sample_t s;
        b5_fault_t  fault;

        if (mission_stopped())
        {
            return B5_ST_IDLE;
        }

        fault = sample_at(angle, B5_SAMPLES_PER_ANGLE, &s);

        if (fault != B5_FAULT_NONE)
        {
            return to_fault(fault);         /* servo or echo timeout */
        }

        (void)b5_scan_add(&g_ctx.scan, s.angle_dd, s.range_mm, s.b_valid);
    }

    return b5_scan_has_near(&g_ctx.scan, B5_NEAR_MM)
           ? B5_ST_EDGE_REFINE              /* near points found */
           : B5_ST_PLANNING;                /* nothing there     */
}

/* Edge Refinement: do / bisect each edge (self-transition while open). */
static b5_state_t
st_edge_refine (void)
{
    uint8_t n;
    int16_t angle;

    for (n = 0U; n < B5_MAX_REFINE_SAMPLES; n++)
    {
        b5_sample_t s;
        b5_fault_t  fault;

        if (mission_stopped())
        {
            return B5_ST_IDLE;
        }

        if (!b5_next_refine_angle(&g_ctx.scan, B5_NEAR_MM,
                                  B5_REFINE_STOP_DD, &angle))
        {
            break;                          /* every bracket closed */
        }

        fault = sample_at(angle, B5_REFINE_PINGS, &s);

        if (fault != B5_FAULT_NONE)
        {
            return to_fault(fault);
        }

        (void)b5_scan_add(&g_ctx.scan, s.angle_dd, s.range_mm, s.b_valid);
    }

    return B5_ST_PLANNING;                  /* scan complete */
}

/* Planning: entry / profile, correct width, score both sides. */
static b5_state_t
st_planning (void)
{
    b5_params_t   par  = profile_params();
    b5_profile_t * p_p = &g_ctx.status.profile;

    b5_build_profile(&g_ctx.scan, &par, p_p);
    publish(true);

    if (!p_p->b_found || (p_p->confidence_pct < B5_MIN_CONFIDENCE_PCT))
    {
        /* No obstacle, or confidence < 50 %: a false stop costs a second,
         * a wrong bypass costs the run. Resume line following. */
        b5_nav_release_motion();
        return B5_ST_GUARD;
    }

    if (p_p->side == B5_SIDE_NONE)
    {
        return B5_ST_BLOCKED;               /* no passable side */
    }

    g_ctx.plan.side       = p_p->side;
    g_ctx.plan.lateral_mm = p_p->lateral_mm;
    g_ctx.plan.forward_mm = p_p->forward_mm;
    g_ctx.plan.closest_mm = p_p->closest_mm;
    g_ctx.plan.width_mm   = p_p->width_mm;

    return B5_ST_BYPASSING;                 /* a side passable */
}

/* Bypassing: entry / record pose, send plan; do / keep watching ahead. */
static b5_state_t
st_bypassing (void)
{
    uint32_t start_us = sys_time_us();

    if (b5_servo_move(B5_AHEAD_DD) != E_OK)
    {
        return to_fault(B5_FAULT_SERVO);
    }

    if (!g_ctx.b_off_line)
    {
        pose_reset();                       /* where the car left the line */
        g_ctx.b_off_line = true;
    }

    g_ctx.confirm = 0U;
    clear_events(B5_EV_PLAN_ACCEPTED | B5_EV_PLAN_REJECTED |
                 B5_EV_MOTION_DONE | B5_EV_MOTION_FAILED);
    b5_nav_send_plan(&g_ctx.plan);

    for (;;)
    {
        UINT    ev;
        int32_t mm;
        bool    b_fault = false;

        ev = take_events(B5_EV_MISSION_STOP | B5_EV_PLAN_ACCEPTED |
                         B5_EV_PLAN_REJECTED | B5_EV_MOTION_DONE |
                         B5_EV_MOTION_FAILED, TMO_POL);

        if ((ev & B5_EV_MISSION_STOP) != 0U)
        {
            return B5_ST_IDLE;
        }

        if ((ev & B5_EV_PLAN_REJECTED) != 0U)
        {
            return B5_ST_BLOCKED;
        }

        if ((ev & B5_EV_MOTION_FAILED) != 0U)
        {
            return to_fault(B5_FAULT_MOTION);
        }

        pose_update();

        if ((ev & B5_EV_MOTION_DONE) != 0U)
        {
            return B5_ST_RECOVERING;        /* passed + clearance margin */
        }

        if ((sys_time_us() - start_us) > (B5_MOTION_TMO_MS * 1000U))
        {
            return to_fault(B5_FAULT_MOTION);
        }

        if (range_once(&mm, &b_fault))
        {
            g_ctx.status.range_mm = mm;
            g_ctx.confirm = (mm < B5_D_STOP_MM) ? (g_ctx.confirm + 1U) : 0U;

            if (g_ctx.confirm >= B5_CONFIRM_COUNT)
            {
                return B5_ST_HALTING;       /* new obstacle confirmed */
            }
        }
        else if (b_fault)
        {
            return to_fault(B5_FAULT_ECHO_TIMEOUT);
        }
        else
        {
            /* single missed echo: keep watching */
        }

        publish(false);
    }
}

static void
request_rejoin (uint8_t attempt)
{
    b5_rejoin_t     req;
    b5_motion_est_t est;

    req.lateral_mm = g_ctx.lateral_mm;
    req.heading_dd = 0;
    req.search_mm  = B5_SEARCH_WIDTH_MM * (int32_t)(attempt + 1U);
    req.attempt    = attempt;
    clear_events(B5_EV_MOTION_DONE | B5_EV_MOTION_FAILED);

    if (b5_motion_get_estimate(&est))
    {
        req.heading_dd = est.heading_dd - g_ctx.est_ref.heading_dd;
    }

    b5_nav_request_rejoin(&req);
}

/* Recovering: entry / steer to stored line; do / poll line sensors. */
static b5_state_t
st_recovering (void)
{
    uint32_t leg_start_us;

    g_ctx.status.recover_tries = 0U;
    request_rejoin(0U);
    leg_start_us = sys_time_us();

    for (;;)
    {
        UINT ev = take_events(B5_EV_MISSION_STOP | B5_EV_MOTION_DONE |
                              B5_EV_MOTION_FAILED, B5_LINE_POLL_MS);
        bool b_leg_over;

        if ((ev & B5_EV_MISSION_STOP) != 0U)
        {
            return B5_ST_IDLE;
        }

        if ((ev & B5_EV_MOTION_FAILED) != 0U)
        {
            return to_fault(B5_FAULT_MOTION);
        }

        pose_update();

        if (b5_line_detected())
        {
            /* line reacquired, line following resumed */
            g_ctx.b_off_line = false;
            b5_nav_release_motion();
            return B5_ST_GUARD;
        }

        b_leg_over = ((ev & B5_EV_MOTION_DONE) != 0U) ||
                     ((sys_time_us() - leg_start_us) >
                      (B5_MOTION_TMO_MS * 1000U));

        if (b_leg_over)
        {
            g_ctx.status.recover_tries++;   /* leg exhausted */

            if (g_ctx.status.recover_tries >= B5_RECOVER_ATTEMPTS)
            {
                return to_fault(B5_FAULT_LINE_LOST);
            }

            request_rejoin(g_ctx.status.recover_tries);
            leg_start_us = sys_time_us();
            publish(true);
        }
    }
}

/* Blocked: entry / publish blocked route (FR-USO-07). */
static b5_state_t
st_blocked (void)
{
    b5_nav_blocked_route(&g_ctx.status.profile);

    if (g_ctx.status.blocked_tries >= B5_BLOCKED_RETRIES)
    {
        return to_fault(B5_FAULT_BLOCKED);  /* attempts exhausted */
    }

    if (reverse_is_safe())
    {
        UINT ev;

        g_ctx.status.blocked_tries++;
        clear_events(B5_EV_MOTION_DONE | B5_EV_MOTION_FAILED);
        b5_nav_request_reverse(B5_REVERSE_MM);

        ev = take_events(B5_EV_MISSION_STOP | B5_EV_MOTION_DONE |
                         B5_EV_MOTION_FAILED, B5_MOTION_TMO_MS);

        if ((ev & B5_EV_MISSION_STOP) != 0U)
        {
            return B5_ST_IDLE;
        }

        if ((ev & B5_EV_MOTION_DONE) == 0U)
        {
            return to_fault(B5_FAULT_MOTION);
        }

        return B5_ST_HALTING;               /* reverse safe, rescan */
    }

    /* Not safe to reverse: stay stopped with the route reported. */
    (void)take_events(B5_EV_MISSION_STOP, TMO_FEVR);
    return B5_ST_IDLE;
}

/* Fault: entry / stop ranging, publish fault, release motion. */
static b5_state_t
st_fault (void)
{
    b5_nav_release_motion();
    b5_report_fault(g_ctx.status.fault);
    (void)b5_servo_move(B5_AHEAD_DD);

    (void)take_events(B5_EV_FAULT_RESET, TMO_FEVR);

    g_ctx.status.fault = B5_FAULT_NONE;
    g_ctx.no_echo_run  = 0U;
    g_ctx.b_off_line   = false;
    clear_events(B5_EV_ALL);                /* drop stale requests */

    return B5_ST_IDLE;                      /* fault acknowledged */
}

typedef b5_state_t (* state_fn_t)(void);

static state_fn_t const g_state_fn[B5_ST_COUNT] =
{
    st_idle, st_guard, st_confirm, st_halting, st_coarse_sweep,
    st_edge_refine, st_planning, st_bypassing, st_recovering,
    st_blocked, st_fault
};

static void
b5_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    g_ctx.status.state = B5_ST_IDLE;
    publish(true);

    for (;;)
    {
        b5_state_t next = g_state_fn[g_ctx.status.state]();

        g_ctx.status.state = next;
        publish(true);
    }
}

/**
 * @brief  Phase 1 (subsystem5_init): drivers and the event flag.
 *         sys_hal_init() must already have run (system_init does it).
 */
ER
b5_init (void)
{
    T_CFLG cflg = { 0 };
    ER     er;

    er = b5_servo_init();

    if (er == E_OK)
    {
        er = b5_us_init();
    }

    if (er != E_OK)
    {
        return er;
    }

    cflg.flgatr  = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0U;
    g_event_flg  = tk_cre_flg(&cflg);

    return (g_event_flg < E_OK) ? g_event_flg : E_OK;
}

/**
 * @brief  Phase 2 (subsystem5_start): create and start the task.
 */
ER
b5_start (void)
{
    T_CTSK ctsk = { 0 };

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = b5_task;
    ctsk.itskpri = B5_TASK_PRIORITY;
    ctsk.stksz   = B5_TASK_STACK_BYTES;
    g_task_id    = tk_cre_tsk(&ctsk);

    if (g_task_id < E_OK)
    {
        return g_task_id;
    }

    return tk_sta_tsk(g_task_id, 0);
}

/**
 * @brief  Copy of the latest status (for tests and telemetry readers).
 *         Not atomic: fields may come from two consecutive updates.
 */
void
b5_get_status (b5_status_t * p_status)
{
    *p_status = g_ctx.status;
}
