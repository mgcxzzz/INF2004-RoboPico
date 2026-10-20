/**
 * @file    nav_coordinator.c
 * @brief   Navigation Coordinator task (team task, Sections 3, 4 and 5).
 *
 * One event-driven task with one inbox (a uT-Kernel message buffer).
 * Every message is handled to completion in this task, so the operating
 * state, the motion owner and the manoeuvre in progress need no locks.
 *
 * Two layers:
 *  1. Operating state (system statechart, Section 5): Ready, Running,
 *     Paused, Safe Stop, Completed, Emergency Stop, Fault.
 *  2. While Running, ONE behaviour owns motion at a time (FR-SYS-02):
 *         Obstacle (Buddy 5)  >  Barcode turn  >  Line search  >  Line
 *     A higher-priority owner pre-empts a lower one; the lower one only
 *     gets control back when the higher one finishes or releases it.
 *
 * Bounded moves (bypass legs, barcode turns, line search sweeps) are run
 * as a short list of Motion Control commands, a "manoeuvre". Each command
 * carries an id; a result with an id that is no longer current is ignored,
 * so a pre-empted manoeuvre can never resume by accident.
 *
 * Minimum-bit data (Tribunal 1):
 *  - the coordinator's records (manoeuvre steps, context, inbox messages)
 *    are bit-fields sized to each value. Only this task writes them.
 *  - state and owner, which other tasks read through nav_get_state() and
 *    nav_get_owner(), share ONE byte (3 + 3 bits) written with a single
 *    store by this task only, so a reader always sees a matching pair.
 *  - the dropped-message count is incremented by OTHER tasks (a poster
 *    whose message did not fit). A plain `count++` from several tasks is
 *    a read-modify-write race, so the count is a kernel semaphore:
 *    tk_sig_sem() adds 1 as one kernel operation and stops at 255.
 *  - telemetry is a binary frame in the team schema (sys_tlm.h): the
 *    3-byte common header and a 6-byte payload.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>

#include "nav_config.h"
#include "nav_iface.h"
#include "nav_internal.h"
#include "../subsystem5/b5_iface.h"           /* events back to Buddy 5 */

#define MAX_STEPS           (6U)        /* must fit in 3 bits      */
#define SPEED_MAX_MM_S      (255)       /* step speed is 8 bits    */

#define MODE_STATE_SHIFT    (0U)        /* g_mode: bits 0..2 state */
#define MODE_OWNER_SHIFT    (3U)        /*         bits 3..5 owner */
#define MODE_FIELD_MASK     (0x7U)

typedef enum
{
    STEP_TURN = 1,
    STEP_DRIVE,
    STEP_STOP
} step_kind_t;

/* One Motion Control step: 4 bytes. */
typedef struct
{
    uint32_t kind  : 2;         /* step_kind_t                        */
    uint32_t speed : 8;         /* mm/s, 0..255                       */
    int32_t  value : 13;        /* dd for a turn, mm for a drive,
                                   +/-NAV_STEP_VALUE_MAX              */
} step_t;

/* What a manoeuvre is for: decides what happens when it ends. */
typedef enum
{
    PURPOSE_NONE = 0,
    PURPOSE_HALT,               /* stop on pause / stop / estop       */
    PURPOSE_B5_STOP,            /* Buddy 5 asked for a stop           */
    PURPOSE_B5_MOVE,            /* Buddy 5 bypass, rejoin or reverse  */
    PURPOSE_BARCODE,            /* barcode turn                       */
    PURPOSE_SEARCH              /* line lost: sweep for it            */
} purpose_t;

typedef struct
{
    step_t   step[MAX_STEPS];
    uint32_t count     : 3;     /* steps in use, 0..MAX_STEPS         */
    uint32_t next      : 3;     /* index of the step to send next     */
    uint32_t purpose   : 3;     /* purpose_t                          */
    uint32_t active_id : 15;    /* 0 = no command outstanding         */
    uint32_t sent_ms;           /* timestamp: stays 32-bit (team rule) */
} manoeuvre_t;

/* Everything else only this task reads and writes. */
typedef struct
{
    uint32_t b_slow        : 1;     /* Buddy 5 asked for reduced speed */
    uint32_t b_cleared     : 1;     /* Safety Supervisor cleared       */
    uint32_t search_step   : 3;     /* line-search progress, 0 = none  */
    uint32_t next_id       : 15;    /* last command id, 1..0x7FFF      */
    uint32_t commands_sent : 16;    /* wrapping                        */
    uint32_t tlm_seq       : 8;     /* telemetry sequence, wrapping    */
} nav_ctx_t;

_Static_assert(sizeof(step_t) == 4U, "step_t must stay 4 bytes");
_Static_assert(sizeof(nav_msg_t) == 8U, "nav_msg_t must stay 8 bytes");
_Static_assert(MAX_STEPS <= 7U, "step counters are 3 bits");

/* State (bits 0..2) and owner (bits 3..5). Written ONLY by this task, as
 * one byte store; read by any task. */
static volatile uint8_t     g_mode = 0U;    /* Ready, no owner          */
static nav_ctx_t            g_ctx;
static manoeuvre_t          g_man;
static ID                   g_drop_sem = 0; /* dropped-message counter  */
static ID                   g_inbox = 0;
static ID                   g_task  = 0;

static UW g_inbox_buf[(NAV_INBOX_MSGS * (sizeof(nav_msg_t) + 4U)) /
                      sizeof(UW)];

static char const * const g_state_names[NAV_ST_COUNT] =
{
    "Ready", "Running", "Paused", "Safe Stop", "Completed",
    "Emergency Stop", "Fault"
};

static char const * const g_owner_names[NAV_OWNER_COUNT] =
{
    "none", "line", "barcode", "search", "obstacle"
};

/* ---------------------------------------------------------------------
 * Public getters and posting
 * ------------------------------------------------------------------ */

nav_state_t
nav_get_state (void)
{
    return (nav_state_t)((g_mode >> MODE_STATE_SHIFT) & MODE_FIELD_MASK);
}

nav_owner_t
nav_get_owner (void)
{
    return (nav_owner_t)((g_mode >> MODE_OWNER_SHIFT) & MODE_FIELD_MASK);
}

/* Coordinator task only: update state and owner with one byte store. */
static void
set_mode (nav_state_t state, nav_owner_t owner)
{
    g_mode = (uint8_t)((((uint32_t)state & MODE_FIELD_MASK) <<
                        MODE_STATE_SHIFT) |
                       (((uint32_t)owner & MODE_FIELD_MASK) <<
                        MODE_OWNER_SHIFT));
}

/* Count one dropped message. Safe from any task: the kernel does the
 * increment, and refuses (E_QOVR) once the count reaches 255. */
static void
count_drop (void)
{
    if (g_drop_sem > 0)
    {
        (void)tk_sig_sem(g_drop_sem, 1);
    }
}

char const *
nav_state_name (nav_state_t state)
{
    return (state < NAV_ST_COUNT) ? g_state_names[state] : "?";
}

char const *
nav_owner_name (nav_owner_t owner)
{
    return (owner < NAV_OWNER_COUNT) ? g_owner_names[owner] : "?";
}

/**
 * @brief  Queue a message for the coordinator. Never blocks.
 */
ER
nav_post_msg (int32_t type, int32_t a, int32_t b, int32_t c)
{
    nav_msg_t msg;
    ER        er;

    if (g_inbox <= 0)
    {
        return E_NOEXS;
    }

    /* Clamp into each field, so nothing wraps on the way in. */
    msg.type = (uint32_t)sys_clamp(type, 0, 15);
    msg.a    = sys_clamp(a, NAV_MSG_A_MIN, NAV_MSG_A_MAX);
    msg.b    = sys_clamp(b, NAV_MSG_A_MIN, NAV_MSG_A_MAX);
    msg.c    = (uint32_t)sys_clamp(c, 0, NAV_MSG_C_MAX);

    er = tk_snd_mbf(g_inbox, &msg, (INT)sizeof(msg), TMO_POL);

    if (er != E_OK)
    {
        count_drop();
    }

    return er;
}

ER nav_post_command (nav_cmd_t cmd)
{ return nav_post_msg(NAV_MSG_COMMAND, (int32_t)cmd, 0, 0); }

/* An id above NAV_CMD_ID_MAX was never issued here: post it as 0
 * ("no command") so it is ignored, never clamped onto a real id. */
ER nav_post_motion_result (uint32_t cmd_id, nav_motion_result_t result)
{
    int32_t id = (cmd_id <= NAV_CMD_ID_MAX) ? (int32_t)cmd_id : 0;

    return nav_post_msg(NAV_MSG_MOTION, id, (int32_t)result, 0);
}

ER nav_post_line_speeds (int16_t left_mm_s, int16_t right_mm_s)
{ return nav_post_msg(NAV_MSG_LINE_SPEEDS, left_mm_s, right_mm_s, 0); }

ER nav_post_line_event (nav_line_event_t event)
{ return nav_post_msg(NAV_MSG_LINE_EVENT, (int32_t)event, 0, 0); }

ER nav_post_barcode (nav_barcode_t command)
{ return nav_post_msg(NAV_MSG_BARCODE, (int32_t)command, 0, 0); }

ER nav_post_safety (nav_safety_t event)
{ return nav_post_msg(NAV_MSG_SAFETY, (int32_t)event, 0, 0); }

/* ---------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

static uint32_t
now_ms (void)
{
    SYSTIM t;

    (void)tk_get_otm(&t);
    return (uint32_t)t.lo;
}

/* Build the telemetry frame (header + 6-byte payload, sys_tlm.h) and
 * hand it to Buddy 1. */
static void
publish (void)
{
    nav_tlm_t t;
    T_RSEM    rsem;
    uint8_t   frame[NAV_TLM_FRAME_BYTES];

    t.seq           = (uint8_t)g_ctx.tlm_seq;
    t.state         = (uint8_t)nav_get_state();
    t.owner         = (uint8_t)nav_get_owner();
    t.b_slow        = (g_ctx.b_slow != 0U);
    t.search_step   = (uint8_t)g_ctx.search_step;
    t.commands_sent = (uint16_t)g_ctx.commands_sent;
    t.dropped       = 0U;

    if ((g_drop_sem > 0) && (tk_ref_sem(g_drop_sem, &rsem) == E_OK))
    {
        t.dropped = (uint8_t)sys_sat_u((uint32_t)rsem.semcnt,
                                       NAV_BITS_DROPPED);
    }

    nav_tlm_pack(&t, frame);
    g_ctx.tlm_seq++;                    /* 8-bit field: wraps */
    nav_telemetry_publish(frame, NAV_TLM_FRAME_BYTES);
}

static void
set_owner (nav_owner_t owner)
{
    if (owner != nav_get_owner())
    {
        set_mode(nav_get_state(), owner);
        line_set_control(owner == NAV_OWNER_LINE);
        publish();
    }
}

/* Build a step, clamping each value to its field. */
static step_t
make_step (step_kind_t kind, int32_t value, int32_t speed_mm_s)
{
    step_t st;

    st.kind  = (uint32_t)kind;
    st.speed = (uint32_t)sys_clamp(speed_mm_s, 0, SPEED_MAX_MM_S);
    st.value = sys_clamp(value, -NAV_STEP_VALUE_MAX, NAV_STEP_VALUE_MAX);

    return st;
}

/* Forget the current manoeuvre; any late result for it is ignored. */
static void
abandon_manoeuvre (void)
{
    g_man.active_id   = 0U;
    g_man.count       = 0U;
    g_man.next        = 0U;
    g_man.purpose     = (uint32_t)PURPOSE_NONE;
    g_ctx.search_step = 0U;
}

static void manoeuvre_failed(void);

/* Send the next step of the manoeuvre to Motion Control. */
static void
send_next_step (void)
{
    step_t const * p_s;
    uint32_t       id;
    ER             er;

    if (g_man.next >= g_man.count)
    {
        return;
    }

    p_s = &g_man.step[g_man.next];
    g_man.next++;

    /* Ids run 1..NAV_CMD_ID_MAX and wrap; 0 means "no command". */
    g_ctx.next_id = (g_ctx.next_id >= NAV_CMD_ID_MAX) ? 1U
                                                      : (g_ctx.next_id + 1U);
    id = g_ctx.next_id;
    g_man.active_id = id;
    g_man.sent_ms   = now_ms();
    g_ctx.commands_sent++;              /* 16-bit field: wraps */

    if (g_man.purpose == (uint32_t)PURPOSE_SEARCH)
    {
        g_ctx.search_step = g_man.next;
    }

    switch ((step_kind_t)p_s->kind)
    {
        case STEP_TURN:
            er = mc_turn(p_s->value, (int16_t)p_s->speed, id);
            break;

        case STEP_DRIVE:
            er = mc_drive_distance(p_s->value, (int16_t)p_s->speed, id);
            break;

        default:
            er = mc_stop(id);
            break;
    }

    if (er != E_OK)
    {
        manoeuvre_failed();
    }
}

static void
start_manoeuvre (purpose_t purpose, step_t const * p_steps, uint8_t count)
{
    uint8_t i;

    abandon_manoeuvre();

    for (i = 0U; (i < count) && (i < MAX_STEPS); i++)
    {
        g_man.step[i] = p_steps[i];
    }

    g_man.count   = i;
    g_man.purpose = (uint32_t)purpose;
    send_next_step();
}

static void enter_state(nav_state_t next);

/* All steps reported done. */
static void
manoeuvre_done (void)
{
    purpose_t purpose = (purpose_t)g_man.purpose;

    abandon_manoeuvre();

    switch (purpose)
    {
        case PURPOSE_B5_STOP:
            (void)b5_post_event(B5_EV_STOP_ACCEPTED);
            break;

        case PURPOSE_B5_MOVE:
            (void)b5_post_event(B5_EV_MOTION_DONE);
            break;

        case PURPOSE_BARCODE:
            set_owner(NAV_OWNER_LINE);      /* back to line following */
            break;

        case PURPOSE_SEARCH:
            /* Every sweep done and the line never reappeared. */
            nav_report_fault(NAV_FAULT_LINE_LOST);
            enter_state(NAV_ST_SAFE_STOP);
            break;

        default:
            break;                          /* HALT: car is stopped */
    }
}

static void
manoeuvre_failed (void)
{
    purpose_t purpose = (purpose_t)g_man.purpose;

    abandon_manoeuvre();

    if ((purpose == PURPOSE_B5_MOVE) || (purpose == PURPOSE_B5_STOP))
    {
        /* Buddy 5 decides; its Fault state then reaches us as a
         * release-with-fault. */
        (void)b5_post_event(B5_EV_MOTION_FAILED);
        return;
    }

    if ((purpose != PURPOSE_HALT) && (nav_get_state() == NAV_ST_RUNNING))
    {
        nav_report_fault(NAV_FAULT_MOTION);
        enter_state(NAV_ST_SAFE_STOP);
    }
}

/* ---------------------------------------------------------------------
 * Operating state (Section 5)
 * ------------------------------------------------------------------ */

static void
stop_everything (void)
{
    step_t halt = make_step(STEP_STOP, 0, 0);

    set_owner(NAV_OWNER_NONE);
    start_manoeuvre(PURPOSE_HALT, &halt, 1U);
    (void)b5_post_event(B5_EV_MISSION_STOP);
    g_ctx.b_slow = 0U;
}

static void
enter_state (nav_state_t next)
{
    nav_state_t prev = nav_get_state();

    if (next == prev)
    {
        return;
    }

    set_mode(next, nav_get_owner());

    if (next == NAV_ST_RUNNING)
    {
        abandon_manoeuvre();
        set_owner(NAV_OWNER_LINE);
        (void)b5_post_event(B5_EV_MISSION_START);
    }
    else if (prev == NAV_ST_RUNNING)
    {
        stop_everything();                  /* motors off on every exit */
    }
    else if ((next == NAV_ST_ESTOP) || (next == NAV_ST_FAULT))
    {
        stop_everything();
    }
    else
    {
        /* Ready, or between stopped states: nothing moves. */
    }

    line_notify_state(next);
    imu_notify_state(next);
    wifi_notify_state(next);
    publish();
}

static void
on_command (nav_cmd_t cmd)
{
    nav_state_t s = nav_get_state();

    switch (cmd)
    {
        case NAV_CMD_START:
            if ((s == NAV_ST_READY) && nav_start_allowed())
            {
                enter_state(NAV_ST_RUNNING);
            }
            break;

        case NAV_CMD_PAUSE:
            if (s == NAV_ST_RUNNING)
            {
                enter_state(NAV_ST_PAUSED);
            }
            break;

        case NAV_CMD_RESUME:
            if (s == NAV_ST_PAUSED)
            {
                enter_state(NAV_ST_RUNNING);
            }
            break;

        case NAV_CMD_STOP:
            if ((s == NAV_ST_RUNNING) || (s == NAV_ST_PAUSED))
            {
                enter_state(NAV_ST_SAFE_STOP);
            }
            break;

        case NAV_CMD_COURSE_FINISHED:
            if (s == NAV_ST_RUNNING)
            {
                enter_state(NAV_ST_COMPLETED);
            }
            break;

        case NAV_CMD_RESET:
            if ((s == NAV_ST_SAFE_STOP) || (s == NAV_ST_COMPLETED) ||
                (((s == NAV_ST_ESTOP) || (s == NAV_ST_FAULT)) &&
                 (g_ctx.b_cleared != 0U)))
            {
                if (s == NAV_ST_FAULT)
                {
                    nav_reinitialise();     /* Fault -> Initialising */
                }

                /* Let Buddy 5 leave its own Fault state too. */
                (void)b5_post_event(B5_EV_FAULT_RESET);
                g_ctx.b_cleared = 0U;
                enter_state(NAV_ST_READY);
            }
            break;

        default:
            break;
    }
}

static void
on_safety (nav_safety_t event)
{
    switch (event)
    {
        case NAV_SAFETY_ESTOP:
            if (nav_get_state() != NAV_ST_FAULT)
            {
                g_ctx.b_cleared = 0U;
                enter_state(NAV_ST_ESTOP);
            }
            break;

        case NAV_SAFETY_FAULT:
            g_ctx.b_cleared = 0U;
            enter_state(NAV_ST_FAULT);
            break;

        case NAV_SAFETY_CLEARED:
            g_ctx.b_cleared = 1U;
            break;

        default:
            break;
    }
}

/* ---------------------------------------------------------------------
 * Behaviours while Running
 * ------------------------------------------------------------------ */

static void
on_line_speeds (int32_t left, int32_t right)
{
    if ((nav_get_state() != NAV_ST_RUNNING) ||
        (nav_get_owner() != NAV_OWNER_LINE))
    {
        return;                         /* proposal not authorised now */
    }

    if (g_ctx.b_slow != 0U)
    {
        left  = (left  * NAV_SLOW_PERCENT) / 100;
        right = (right * NAV_SLOW_PERCENT) / 100;
    }

    g_ctx.commands_sent++;
    (void)mc_set_wheel_speeds((int16_t)left, (int16_t)right);
}

static void
on_line_event (nav_line_event_t event)
{
    if (nav_get_state() != NAV_ST_RUNNING)
    {
        return;
    }

    if ((event == NAV_LINE_LOST) && (nav_get_owner() == NAV_OWNER_LINE))
    {
        /* Alternate sweeps of growing size: +15, -30, +45, -60 deg. */
        step_t  sweep[NAV_SEARCH_STEPS + 1U];
        uint8_t i;
        int32_t angle = NAV_SEARCH_STEP1_DD;

        sweep[0] = make_step(STEP_STOP, 0, 0);

        for (i = 1U; i <= NAV_SEARCH_STEPS; i++)
        {
            sweep[i] = make_step(STEP_TURN, angle, NAV_TURN_MM_S);
            angle = -(angle + ((angle > 0) ? NAV_SEARCH_STEP1_DD
                                           : -NAV_SEARCH_STEP1_DD));
        }

        set_owner(NAV_OWNER_SEARCH);
        start_manoeuvre(PURPOSE_SEARCH, sweep,
                        (uint8_t)(NAV_SEARCH_STEPS + 1U));
    }
    else if ((event == NAV_LINE_FOUND) &&
             (nav_get_owner() == NAV_OWNER_SEARCH))
    {
        /* Hand straight back: the next wheel-speed proposal replaces the
         * sweep in Motion Control. */
        abandon_manoeuvre();
        set_owner(NAV_OWNER_LINE);
    }
    else
    {
        /* Junctions are reported for telemetry; turns come as barcodes. */
    }
}

static void
on_barcode (nav_barcode_t cmd)
{
    int32_t angle;

    if ((nav_get_state() != NAV_ST_RUNNING) ||
        (nav_get_owner() != NAV_OWNER_LINE))
    {
        count_drop();                   /* busy: one event per barcode */
        return;
    }

    switch (cmd)
    {
        case NAV_BARCODE_LEFT:  angle = NAV_TURN_LEFT_DD;  break;
        case NAV_BARCODE_RIGHT: angle = NAV_TURN_RIGHT_DD; break;
        case NAV_BARCODE_UTURN: angle = NAV_TURN_UTURN_DD; break;
        default:                return;     /* straight: keep following */
    }

    {
        step_t turn = make_step(STEP_TURN, angle, NAV_TURN_MM_S);

        set_owner(NAV_OWNER_BARCODE);
        start_manoeuvre(PURPOSE_BARCODE, &turn, 1U);
    }
}

/* --- Buddy 5 (Obstacle Scan and Recovery) ------------------------- */

static bool
obstacle_owns_motion (void)
{
    return (nav_get_state() == NAV_ST_RUNNING) &&
           (nav_get_owner() == NAV_OWNER_OBSTACLE);
}

static int32_t
abs32 (int32_t v)
{
    return (v < 0) ? -v : v;
}

static void
on_b5_stop (void)
{
    step_t halt = make_step(STEP_STOP, 0, 0);

    if (nav_get_state() != NAV_ST_RUNNING)
    {
        return;
    }

    /* Obstacle pre-empts every other behaviour. */
    set_owner(NAV_OWNER_OBSTACLE);
    start_manoeuvre(PURPOSE_B5_STOP, &halt, 1U);
}

/* Leave the line at 45 deg, run parallel past the obstacle. */
static void
on_b5_plan (int32_t side, int32_t lateral_mm, int32_t forward_mm)
{
    step_t  legs[4];
    uint8_t n   = 0U;
    int32_t dir = (side == (int32_t)B5_SIDE_LEFT) ? -1 : 1;
    int32_t along;

    if (!obstacle_owns_motion())
    {
        (void)b5_post_event(B5_EV_PLAN_REJECTED);
        return;
    }

    along = forward_mm;

    if (lateral_mm > 0)
    {
        legs[n++] = make_step(STEP_TURN, dir * NAV_BYPASS_ANGLE_DD,
                              NAV_TURN_MM_S);
        legs[n++] = make_step(STEP_DRIVE,
                              (lateral_mm * NAV_SQRT2_X1000) / 1000,
                              NAV_MANOEUVRE_MM_S);
        legs[n++] = make_step(STEP_TURN, -dir * NAV_BYPASS_ANGLE_DD,
                              NAV_TURN_MM_S);
        along -= lateral_mm;            /* the 45 deg leg went forward too */
    }

    if (along > 0)
    {
        legs[n++] = make_step(STEP_DRIVE, along, NAV_MANOEUVRE_MM_S);
    }

    (void)b5_post_event(B5_EV_PLAN_ACCEPTED);
    start_manoeuvre(PURPOSE_B5_MOVE, legs, n);
}

/* Head back towards the line at 45 deg; Buddy 5 releases when found. */
static void
on_b5_rejoin (int32_t lateral_mm, int32_t heading_dd, int32_t search_mm)
{
    step_t  legs[2];
    int32_t target_dd = 0;
    int32_t leg_mm;

    if (!obstacle_owns_motion())
    {
        return;
    }

    if (abs32(lateral_mm) >= NAV_REJOIN_MIN_MM)
    {
        /* Right of the line -> turn left (negative), and vice versa. */
        target_dd = (lateral_mm > 0) ? -NAV_BYPASS_ANGLE_DD
                                     : NAV_BYPASS_ANGLE_DD;
    }

    leg_mm = ((abs32(lateral_mm) * NAV_SQRT2_X1000) / 1000) + search_mm;

    if (leg_mm > NAV_REJOIN_MAX_MM)
    {
        leg_mm = NAV_REJOIN_MAX_MM;
    }

    legs[0] = make_step(STEP_TURN, target_dd - heading_dd, NAV_TURN_MM_S);
    legs[1] = make_step(STEP_DRIVE, leg_mm, NAV_MANOEUVRE_MM_S);

    start_manoeuvre(PURPOSE_B5_MOVE, legs, 2U);
}

static void
on_b5_reverse (int32_t distance_mm)
{
    step_t back = make_step(STEP_DRIVE, -abs32(distance_mm),
                            NAV_REVERSE_MM_S);

    if (!obstacle_owns_motion())
    {
        return;
    }

    start_manoeuvre(PURPOSE_B5_MOVE, &back, 1U);
}

static void
on_b5_release (bool b_b5_faulted)
{
    if (!obstacle_owns_motion())
    {
        return;
    }

    abandon_manoeuvre();

    if (b_b5_faulted)
    {
        /* No forward sensing: stop here rather than follow the line
         * blind (FR-SYS-03). Safety Supervisor may escalate to Fault. */
        nav_report_fault(NAV_FAULT_OBSTACLE);
        enter_state(NAV_ST_SAFE_STOP);
        return;
    }

    set_owner(NAV_OWNER_LINE);          /* line following resumes */
}

/* ---------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------ */

static void
on_motion_result (uint32_t cmd_id, nav_motion_result_t result)
{
    if ((cmd_id == 0U) || (cmd_id != g_man.active_id))
    {
        return;                         /* stale: manoeuvre was replaced */
    }

    g_man.active_id = 0U;

    if (result == NAV_MOTION_CANCELLED)
    {
        /* Cancelled by Motion Control itself, i.e. Safety Supervisor
         * inhibited drive. Not a motion fault: the Emergency Stop or
         * Fault notice that follows stops everything. */
        abandon_manoeuvre();
    }
    else if (result != NAV_MOTION_DONE)
    {
        manoeuvre_failed();
    }
    else if (g_man.next < g_man.count)
    {
        send_next_step();
    }
    else
    {
        manoeuvre_done();
    }
}

static void
dispatch (nav_msg_t const * p_m)
{
    switch ((nav_msg_type_t)p_m->type)
    {
        case NAV_MSG_COMMAND:     on_command((nav_cmd_t)p_m->a);           break;
        case NAV_MSG_SAFETY:      on_safety((nav_safety_t)p_m->a);         break;
        case NAV_MSG_MOTION:
            on_motion_result((uint32_t)p_m->a, (nav_motion_result_t)p_m->b);
            break;
        case NAV_MSG_LINE_SPEEDS: on_line_speeds(p_m->a, p_m->b);          break;
        case NAV_MSG_LINE_EVENT:  on_line_event((nav_line_event_t)p_m->a); break;
        case NAV_MSG_BARCODE:     on_barcode((nav_barcode_t)p_m->a);       break;
        case NAV_MSG_B5_SLOW:     g_ctx.b_slow = (p_m->a != 0) ? 1U : 0U;  break;
        case NAV_MSG_B5_STOP:     on_b5_stop();                            break;
        case NAV_MSG_B5_PLAN:     on_b5_plan(p_m->a, p_m->b, p_m->c);      break;
        case NAV_MSG_B5_REJOIN:   on_b5_rejoin(p_m->a, p_m->b, p_m->c);    break;
        case NAV_MSG_B5_REVERSE:  on_b5_reverse(p_m->a);                   break;
        case NAV_MSG_B5_RELEASE:  on_b5_release(p_m->a != 0);              break;
        case NAV_MSG_B5_BLOCKED:  publish();                               break;
        default:                                                           break;
    }
}

static void
nav_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    publish();

    for (;;)
    {
        nav_msg_t msg;
        INT       size = tk_rcv_mbf(g_inbox, &msg, NAV_IDLE_WAKE_MS);

        nav_heartbeat();                /* "still alive" to Safety */

        if (size == (INT)sizeof(msg))
        {
            dispatch(&msg);
        }

        /* Watchdog on the outstanding Motion Control command. */
        if ((g_man.active_id != 0U) &&
            ((now_ms() - g_man.sent_ms) > (uint32_t)NAV_MOTION_TMO_MS))
        {
            g_man.active_id = 0U;
            manoeuvre_failed();
        }
    }
}

/**
 * @brief  Phase 1 (system_init): create the inbox. After this, other
 *         tasks may post, even before the coordinator task runs.
 */
ER
nav_init (void)
{
    T_CMBF cmbf = { 0 };
    T_CSEM csem = { 0 };

    /* Dropped-message counter: a semaphore used only as an atomic,
     * saturating counter (0..255). Nobody ever waits on it. */
    csem.sematr  = TA_TFIFO;
    csem.isemcnt = 0;
    csem.maxsem  = (INT)NAV_DROPPED_MAX;
    g_drop_sem   = tk_cre_sem(&csem);

    if (g_drop_sem < E_OK)
    {
        return g_drop_sem;
    }

    cmbf.mbfatr = TA_TFIFO | TA_MFIFO | TA_USERBUF;
    cmbf.bufsz  = (SZ)sizeof(g_inbox_buf);
    cmbf.maxmsz = (INT)sizeof(nav_msg_t);
    cmbf.bufptr = g_inbox_buf;
    g_inbox     = tk_cre_mbf(&cmbf);

    return (g_inbox < E_OK) ? g_inbox : E_OK;
}

/**
 * @brief  Phase 2 (system_start): create and start the task.
 */
ER
nav_start (void)
{
    T_CTSK ctsk = { 0 };

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = nav_task;
    ctsk.itskpri = NAV_TASK_PRIORITY;
    ctsk.stksz   = NAV_TASK_STACK_BYTES;
    g_task       = tk_cre_tsk(&ctsk);

    if (g_task < E_OK)
    {
        return g_task;
    }

    return tk_sta_tsk(g_task, 0);
}
