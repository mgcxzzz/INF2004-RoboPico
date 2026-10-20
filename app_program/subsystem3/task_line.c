/**
 * @file    task_line.c
 * @brief   Line and Barcode task: Week 6 statechart 6.3, one function per
 *          state.
 *
 *   Initialising --checks OK--> Idle / Ready --mission start--> Mission Active
 *   Mission Active (front line sensing runs in every sub-state):
 *     Line Following --first bar [armed]--> Read Barcode
 *     Read Barcode --quiet after the last bar--> Decode and Request
 *     Decode and Request --valid--> (turn point) send ONE event --> Await
 *     Decode and Request --invalid / backwards / timeout--> Line Following
 *     Line Following --line lost--> Await Navigation
 *     Await Navigation --control granted again--> Line Following
 *   stop / pause / mission complete --> Idle / Ready
 *   critical sensor fault or safety inhibit --> Fault / Inhibited
 *   Fault / Inhibited --authorised reset--> Initialising
 *
 * One deviation from the Week 6 drawing, on purpose: an INVALID read goes
 * straight back to Line Following. In the drawing it passes through Await
 * Navigation ("request hold"), but no event is sent for an invalid read
 * (NFR-LINE-02), so the coordinator would never hand control back; the
 * text's "the run continues" is what this code does.
 *
 * Who writes what (minimum bits, no races):
 *   g_ctx           this task only (bit-fields)
 *   g_nav_state     the coordinator only (line_notify_state), one byte
 *   g_granted       the coordinator only (line_set_control), one byte
 *   g_sensors       this task only, one byte (read by Buddy 5)
 *   g_state         this task only, one byte
 *   edges           the GP19 interrupt only (s3_ir.c)
 * The coordinator's two writes also set an event-flag bit to wake the task.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>

#include "../system/nav_iface.h"
#include "../system/safety_iface.h"
#include "../system/sys_config.h"
#include "../system/sys_fault.h"
#include "../subsystem5/b5_iface.h"
#include "s3_barcode.h"
#include "s3_config.h"
#include "s3_iface.h"
#include "s3_line.h"
#include "s3_reader.h"

#define EV_NAV          (1U << 0)       /* state or grant changed */

/* Task record (this task only). Counters wrap at 8 bits: use differences. */
typedef struct
{
    uint32_t b_armed      : 1;      /* a barcode may start               */
    uint32_t b_pending    : 1;      /* valid barcode, waiting for the
                                     * turn point                         */
    uint32_t pending_cmd  : 3;      /* nav_barcode_t                     */
    uint32_t await_lost   : 1;      /* awaiting after LOST (else barcode) */
    uint32_t b_revoked    : 1;      /* control was taken while awaiting  */
    uint32_t b_found_sent : 1;
    uint32_t b_fault_bit  : 1;      /* our shared fault bit is set       */
    uint32_t b_dist_ok    : 1;      /* Motion Control gives a distance   */
    uint32_t b_last_dist  : 1;      /* last read measured by distance    */
    uint32_t side         : 2;      /* s3_side_t now                     */
    uint32_t last_char    : 6;      /* Code 39 index, 63 = none          */
    uint32_t last_result  : 3;
    uint32_t last_cmd     : 3;
    uint32_t rearm_mm     : 12;     /* barcode sensor white this far     */
    uint32_t pending_mm   : 12;     /* travel since the barcode ended    */
    uint32_t barcodes_ok  : 8;
    uint32_t bad_reads    : 8;
    uint32_t backwards    : 8;
    uint32_t junctions    : 8;
    uint32_t lost         : 8;
    uint32_t overflows    : 8;      /* last value seen from the ISR      */
    uint32_t tlm_seq      : 8;
    int16_t  last_dist_mm;          /* Buddy 2's wrapping distance       */
    int32_t  odo_q;                 /* our odometer, mm x 16 (wraps after
                                     * 134 km: only differences used)    */
    uint32_t await_us;              /* timestamps stay 32-bit            */
    uint32_t last_pub_us;
} s3_ctx_t;

#define RESULT_TIMEOUT  (6U)        /* timeout or overflow (3 bits; one
                                     * past the decoder results)         */
#define MM_MAX          (4095U)

static s3_ctx_t        g_ctx;
static s3_line_t       g_line;
static s3_reader_t     g_rd;
static volatile uint8_t g_state     = (uint8_t)S3_ST_INIT;
static volatile uint8_t g_nav_state = (uint8_t)NAV_ST_READY;
static volatile uint8_t g_granted   = 0U;
static volatile uint8_t g_sensors   = 0U;
static ID              g_flg  = 0;
static ID              g_task = 0;
static ER              g_init_er = E_OK;

static char const * const g_state_names[S3_ST_COUNT] =
{
    "Initialising", "Idle / Ready", "Line Following", "Read Barcode",
    "Await Navigation", "Fault / Inhibited"
};

static struct
{
    char          ch;
    nav_barcode_t cmd;
} const g_commands[] = { S3_BC_COMMANDS };

/* =====================================================================
 * Provided to the other subsystems
 * ================================================================== */

s3_state_t
s3_get_state (void)
{
    return (s3_state_t)g_state;
}

char const *
s3_state_name (s3_state_t state)
{
    return (state < S3_ST_COUNT) ? g_state_names[state] : "?";
}

#if !SYS_SIM_LINE

/* Navigation Coordinator (its task): "you may steer now" / "stop". */
void
line_set_control (bool b_granted)
{
    g_granted = b_granted ? 1U : 0U;

    if (g_flg > 0)
    {
        (void)tk_set_flg(g_flg, EV_NAV);
    }
}

/* Navigation Coordinator (its task): the operating state. */
void
line_notify_state (nav_state_t state)
{
    g_nav_state = (uint8_t)state;

    if (g_flg > 0)
    {
        (void)tk_set_flg(g_flg, EV_NAV);
    }
}

/* Buddy 5: either front sensor sees the line right now. */
bool
b5_line_detected (void)
{
    return ((g_sensors & (S3_SENSE_LEFT | S3_SENSE_RIGHT)) != 0U);
}

#endif /* !SYS_SIM_LINE */

/* =====================================================================
 * Helpers
 * ================================================================== */

static uint32_t
add_sat (uint32_t a, uint32_t b)
{
    uint32_t s = a + b;

    return (s > MM_MAX) ? MM_MAX : s;
}

static void
publish (bool b_force)
{
    uint32_t now = s3_time_us();
    s3_tlm_t t;
    uint8_t  frame[S3_TLM_FRAME_BYTES];

    if (!b_force &&
        ((now - g_ctx.last_pub_us) < (S3_STATUS_PERIOD_MS * 1000U)))
    {
        return;
    }

    t.seq         = (uint8_t)g_ctx.tlm_seq;
    t.state       = g_state;
    t.sensors     = g_sensors;
    t.b_granted   = (g_granted != 0U);
    t.side        = (uint8_t)g_ctx.side;
    t.last_char   = (uint8_t)g_ctx.last_char;
    t.last_result = (uint8_t)g_ctx.last_result;
    t.last_cmd    = (uint8_t)g_ctx.last_cmd;
    t.b_dist      = (g_ctx.b_last_dist != 0U);
    t.barcodes_ok = (uint8_t)g_ctx.barcodes_ok;
    t.bad_reads   = (uint8_t)g_ctx.bad_reads;
    t.backwards   = (uint8_t)g_ctx.backwards;
    t.junctions   = (uint8_t)g_ctx.junctions;
    t.lost        = (uint8_t)g_ctx.lost;
    t.overflows   = (uint8_t)g_ctx.overflows;

    s3_tlm_pack(&t, frame);
    g_ctx.tlm_seq++;
    g_ctx.last_pub_us = now;
    line_telemetry_publish(frame, S3_TLM_FRAME_BYTES);
}

static void
enter (s3_state_t next)
{
    if ((s3_state_t)g_state != next)
    {
        g_state = (uint8_t)next;
        publish(true);
    }
}

static nav_barcode_t
command_for (char ch)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(g_commands) / sizeof(g_commands[0])); i++)
    {
        if (g_commands[i].ch == ch)
        {
            return g_commands[i].cmd;
        }
    }

    return (nav_barcode_t)0;
}

/* Travel since the last cycle, and our odometer (mm x 16). */
static uint32_t
measure_travel (bool b_proposing)
{
    int16_t  d;
    int32_t  diff;

    if (s3_motion_get_distance(&d))
    {
        if (g_ctx.b_dist_ok == 0U)
        {
            g_ctx.last_dist_mm = d;     /* first reading: no jump */
            g_ctx.b_dist_ok    = 1U;
        }

        /* 16-bit net distance may wrap: take the difference in 16 bits
         * (agreed with Buddy 2; one cycle is a few mm). */
        diff = (int32_t)(int16_t)(uint16_t)((uint16_t)d -
                                            (uint16_t)g_ctx.last_dist_mm);
        g_ctx.last_dist_mm = d;
    }
    else
    {
        g_ctx.b_dist_ok = 0U;
        diff = b_proposing ? S3_NOMINAL_STEP_MM : 0;
    }

    g_ctx.odo_q += diff * S3_RD_Q;

    return (diff < 0) ? (uint32_t)(-diff) : (uint32_t)diff;
}

static void
barcode_done (s3_bc_result_t res, char const * p_text)
{
    g_ctx.last_result = (uint32_t)res;
    g_ctx.last_char   = s3_c39_index(p_text[0]);
    g_ctx.b_armed     = 0U;             /* re-arm only after a clear zone */
    g_ctx.rearm_mm    = 0U;
    s3_rd_reset(&g_rd);

    if (res == S3_BC_OK)
    {
        nav_barcode_t cmd = command_for(p_text[0]);

        if ((p_text[1] == '\0') && (cmd != (nav_barcode_t)0))
        {
            g_ctx.b_pending   = 1U;
            g_ctx.pending_cmd = (uint32_t)cmd;
            g_ctx.pending_mm  = 0U;
        }
        else
        {
            g_ctx.bad_reads++;          /* valid Code 39, not a command */
        }
    }
    else if (res == S3_BC_BACKWARDS)
    {
        g_ctx.backwards++;
    }
    else
    {
        g_ctx.bad_reads++;
    }

    publish(true);
}

static void
decode_now (void)
{
    bool           b_dist = false;
    char           text[S3_BC_MAX_CHARS + 1U];
    s3_bc_result_t res = s3_rd_decode(&g_rd, text, sizeof(text), &b_dist);

    g_ctx.b_last_dist = b_dist ? 1U : 0U;
    barcode_done(res, text);
}

/* Barcode sensor: keep the reader fed, arm/re-arm, end a read. */
static void
run_barcode (uint32_t sensors, uint32_t moved, bool b_reading_allowed)
{
    uint32_t       t_us;
    bool           b_black;
    uint32_t       ovf = s3_ir_overflows();
    s3_rd_status_t st;

    s3_rd_anchor(&g_rd, s3_time_us(), g_ctx.odo_q, g_ctx.b_dist_ok != 0U);

    while (s3_ir_pop_edge(&t_us, &b_black))
    {
        if (b_reading_allowed &&
            ((g_ctx.b_armed != 0U) || (g_rd.count != 0U)) &&
            (g_ctx.b_pending == 0U))
        {
            s3_rd_edge(&g_rd, t_us, b_black);
        }
    }

    /* Lost edges make the current read worthless. */
    if (ovf != g_ctx.overflows)
    {
        g_ctx.overflows = ovf;
        (void)safety_report_fault(SAFETY_SRC_LINE,
                                  (uint8_t)S3_WARN_EDGE_OVERFLOW, false);

        if (g_rd.count != 0U)
        {
            barcode_done((s3_bc_result_t)RESULT_TIMEOUT, "");
        }
    }

    /* Re-arm once the sensor has stayed white for S3_BC_REARM_MM. */
    if ((sensors & S3_SENSE_BARCODE) != 0U)
    {
        g_ctx.rearm_mm = 0U;
    }
    else if (g_rd.count == 0U)
    {
        g_ctx.rearm_mm = add_sat(g_ctx.rearm_mm, moved);

        if (g_ctx.rearm_mm >= (uint32_t)S3_BC_REARM_MM)
        {
            g_ctx.b_armed = 1U;
        }
    }
    else
    {
        /* reading: the quiet zone is judged by the reader */
    }

    st = s3_rd_poll(&g_rd);

    if (st == S3_RD_READY)
    {
        decode_now();
    }
    else if ((st == S3_RD_TIMEOUT) || (st == S3_RD_OVERFLOW))
    {
        barcode_done((s3_bc_result_t)RESULT_TIMEOUT, "");
    }
    else
    {
        /* idle or still reading */
    }
}

static void
withdraw_and_await (bool b_lost)
{
    g_ctx.await_lost   = b_lost ? 1U : 0U;
    g_ctx.b_revoked    = 0U;
    g_ctx.b_found_sent = 0U;
    g_ctx.await_us     = s3_time_us();
    enter(S3_ST_AWAIT);
}

static void
sensor_fault (uint8_t code)
{
    (void)safety_report_fault(SAFETY_SRC_LINE, code, true);
    (void)sys_fault_set(SYS_FAULT_LINE_SENSOR);
    g_ctx.b_fault_bit = 1U;
    enter(S3_ST_FAULT);
}

/* Common to every Mission Active sub-state. Returns false if the state
 * was left (stop, inhibit or fault), in which case the caller is done. */
static bool
mission_guard (nav_state_t nav)
{
    if ((nav == NAV_ST_ESTOP) || (nav == NAV_ST_FAULT))
    {
        enter(S3_ST_FAULT);             /* safety inhibit: withdraw */
        return false;
    }

    if (nav != NAV_ST_RUNNING)
    {
        g_ctx.b_pending = 0U;           /* stop / pause / complete */
        s3_rd_reset(&g_rd);
        enter(S3_ST_IDLE);
        return false;
    }

    return true;
}

/* =====================================================================
 * One cycle
 * ================================================================== */

void
s3_step (void)
{
    nav_state_t   nav     = (nav_state_t)g_nav_state;
    bool          granted = (g_granted != 0U);
    s3_state_t    state   = (s3_state_t)g_state;
    bool          b_active = ((state == S3_ST_FOLLOW) ||
                              (state == S3_ST_READ));
    uint32_t      sensors;
    uint32_t      moved;
    s3_line_out_t out;

    safety_kick(SAFETY_SRC_LINE);

    moved     = measure_travel(b_active && granted);
    sensors   = s3_ir_read();
    g_sensors = (uint8_t)sensors;

    /* Front sensing runs in every state (Week 6: Mission Active "do"). */
    s3_line_step(&g_line, sensors, moved, &out);
    g_ctx.side = out.side;

    run_barcode(sensors, moved, b_active);

    switch (state)
    {
        case S3_ST_INIT:
            s3_line_reset(&g_line);
            s3_rd_reset(&g_rd);
            g_ctx.b_armed   = 0U;
            g_ctx.rearm_mm  = 0U;
            g_ctx.b_pending = 0U;

            if (g_init_er != E_OK)
            {
                sensor_fault((uint8_t)S3_FAULT_INIT);
            }
            else
            {
                if (g_ctx.b_fault_bit != 0U)
                {
                    /* Latch rule: our bit is cleared only after the cause
                     * is gone and a reset was accepted (we are here). */
                    (void)sys_fault_clear(SYS_FAULT_LINE_SENSOR);
                    g_ctx.b_fault_bit = 0U;
                }

                enter(S3_ST_IDLE);
            }
            break;

        case S3_ST_IDLE:
            if ((nav == NAV_ST_ESTOP) || (nav == NAV_ST_FAULT))
            {
                enter(S3_ST_FAULT);
            }
            else if (nav == NAV_ST_RUNNING)
            {
                s3_line_reset(&g_line);
                enter(S3_ST_FOLLOW);    /* mission start [safety clear] */
            }
            else
            {
                /* wait */
            }
            break;

        case S3_ST_FOLLOW:
        case S3_ST_READ:
            if (!mission_guard(nav))
            {
                break;
            }

            if ((out.events & S3_LEV_STUCK) != 0U)
            {
                sensor_fault((uint8_t)S3_FAULT_STUCK);
                break;
            }

            if ((out.events & S3_LEV_JUNCTION) != 0U)
            {
                g_ctx.junctions++;
                (void)nav_post_line_event(NAV_LINE_JUNCTION);
            }

            if ((out.events & S3_LEV_LOST) != 0U)
            {
                g_ctx.lost++;
                g_ctx.b_pending = 0U;
                s3_rd_reset(&g_rd);
                (void)nav_post_line_event(NAV_LINE_LOST);
                withdraw_and_await(true);
                break;
            }

            /* Propose speeds; the coordinator forwards them only while it
             * has granted line control. */
            if (granted)
            {
                (void)nav_line_drive(out.left_mm_s, out.right_mm_s);
            }

            /* A valid barcode is sent once, at the turn point. */
            if (g_ctx.b_pending != 0U)
            {
                g_ctx.pending_mm = add_sat(g_ctx.pending_mm, moved);

                if ((g_ctx.pending_mm + 1U) > (uint32_t)S3_BC_TURN_DELAY_MM)
                {
                    g_ctx.b_pending = 0U;
                    g_ctx.last_cmd  = g_ctx.pending_cmd;
                    g_ctx.barcodes_ok++;
                    (void)nav_post_barcode((nav_barcode_t)g_ctx.pending_cmd);
                    withdraw_and_await(false);
                    break;
                }
            }

            enter((g_rd.count != 0U) ? S3_ST_READ : S3_ST_FOLLOW);
            break;

        case S3_ST_AWAIT:
            if (!mission_guard(nav))
            {
                break;
            }

            if (!granted)
            {
                g_ctx.b_revoked = 1U;   /* the coordinator took over */
            }

            /* After LOST: tell the coordinator's line search when a
             * front sensor sees the line again (once). */
            if ((g_ctx.await_lost != 0U) && (g_ctx.b_found_sent == 0U) &&
                ((sensors & (S3_SENSE_LEFT | S3_SENSE_RIGHT)) != 0U))
            {
                g_ctx.b_found_sent = 1U;
                (void)nav_post_line_event(NAV_LINE_FOUND);
            }

            /* Resume when control comes back; or if the coordinator never
             * took it (it ignored the event), after S3_AWAIT_MS. */
            if (granted &&
                ((g_ctx.b_revoked != 0U) ||
                 ((s3_time_us() - g_ctx.await_us) >=
                  (S3_AWAIT_MS * 1000U))))
            {
                s3_line_reset(&g_line);
                enter(S3_ST_FOLLOW);
            }
            break;

        case S3_ST_FAULT:
            /* Withdrawn. Back to Initialising only after an authorised
             * reset: the coordinator is Ready again. */
            if (nav == NAV_ST_READY)
            {
                enter(S3_ST_INIT);
            }
            break;

        default:
            enter(S3_ST_INIT);
            break;
    }

    publish(false);
}

/* =====================================================================
 * Task and start-up
 * ================================================================== */

static void
line_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        UINT ptn = 0U;

        /* Wake every S3_PERIOD_MS, or at once when the coordinator
         * changes the state or the grant. */
        (void)tk_wai_flg(g_flg, EV_NAV, TWF_ORW | TWF_CLR, &ptn,
                         S3_PERIOD_MS);
        s3_step();
    }
}

/**
 * @brief  Phase 1: event flag, pins and the GP19 interrupt. A pin or
 *         interrupt failure is remembered and sends the task to Fault.
 */
ER
s3_init (void)
{
    T_CFLG cflg = { 0 };

    g_ctx.last_char   = S3_C39_NONE;
    g_ctx.last_result = 0U;

    cflg.flgatr  = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0U;
    g_flg        = tk_cre_flg(&cflg);

    if (g_flg < E_OK)
    {
        return g_flg;
    }

    g_init_er = s3_ir_init();
    (void)safety_watch(SAFETY_SRC_LINE, S3_WATCH_MS);

    return E_OK;
}

/**
 * @brief  Phase 2: the Line and Barcode task.
 */
ER
s3_start (void)
{
    T_CTSK ctsk = { 0 };

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = line_task;
    ctsk.itskpri = S3_TASK_PRIORITY;
    ctsk.stksz   = S3_TASK_STACK_BYTES;
    g_task       = tk_cre_tsk(&ctsk);

    if (g_task < E_OK)
    {
        return g_task;
    }

    return tk_sta_tsk(g_task, 0);
}
