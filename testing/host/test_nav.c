/**
 * @file    test_nav.c
 * @brief   Host test of nav_coordinator.c (no Pico needed).
 *
 * nav_coordinator.c is compiled unchanged against a mock kernel. The test
 * posts messages the way the other buddies would, runs the coordinator
 * until its inbox is empty, and checks what it sent to Motion Control,
 * to Line and Barcode, and back to Buddy 5. Every telemetry frame it
 * publishes is decoded and checked too (minimum-bit requirement).
 *
 * Build and run:  make -C testing/host
 */

#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tk/tkernel.h>

#include "b5_iface.h"
#include "nav_config.h"
#include "nav_iface.h"
#include "nav_internal.h"

/* ---------------------------------------------------------------------
 * Mock kernel: one inbox, a clock, a task we run by hand
 * ------------------------------------------------------------------ */
#define QSIZE   (64)

static nav_msg_t g_q[QSIZE];
static int       g_qh, g_qt;
static FP        g_task;
static jmp_buf   g_idle;
static uint32_t  g_ms;
static bool      g_inbox_full;          /* make tk_snd_mbf fail       */
static INT       g_sem_cnt, g_sem_max;  /* dropped-message semaphore  */

ID tk_cre_tsk (T_CTSK const * p) { g_task = p->task; return 2; }
ER tk_sta_tsk (ID id, INT s)     { (void)id; (void)s; return E_OK; }
ID tk_cre_mbf (T_CMBF const * p) { (void)p; return 1; }
ER tk_get_otm (SYSTIM * p)       { p->hi = 0; p->lo = g_ms; return E_OK; }

ID tk_cre_sem (T_CSEM const * p)
{ g_sem_cnt = p->isemcnt; g_sem_max = p->maxsem; return 4; }

ER
tk_sig_sem (ID id, INT cnt)
{
    (void)id;

    if ((g_sem_cnt + cnt) > g_sem_max)
    {
        return E_QOVR;                  /* like the kernel: refuse */
    }

    g_sem_cnt += cnt;
    return E_OK;
}

ER tk_ref_sem (ID id, T_RSEM * p)
{ (void)id; p->semcnt = g_sem_cnt; p->wtsk = 0; return E_OK; }

ER
tk_snd_mbf (ID id, void const * msg, INT size, TMO tmo)
{
    (void)id; (void)size; (void)tmo;

    if (g_inbox_full)
    {
        return E_TMOUT;
    }

    memcpy(&g_q[g_qt], msg, sizeof(nav_msg_t));
    g_qt = (g_qt + 1) % QSIZE;
    return E_OK;
}

INT
tk_rcv_mbf (ID id, void * msg, TMO tmo)
{
    (void)id; (void)tmo;

    if (g_qh == g_qt)
    {
        longjmp(g_idle, 1);         /* nothing left: back to the test */
    }

    memcpy(msg, &g_q[g_qh], sizeof(nav_msg_t));
    g_qh = (g_qh + 1) % QSIZE;
    return (INT)sizeof(nav_msg_t);
}

/* Run the coordinator until its inbox is empty. */
static void
run (void)
{
    if (setjmp(g_idle) == 0)
    {
        g_task(0, NULL);
    }
}

/* ---------------------------------------------------------------------
 * Mock buddies: record what the coordinator does
 * ------------------------------------------------------------------ */
typedef struct { char kind; int32_t value; uint32_t id; } mc_cmd_t;

static mc_cmd_t   g_mc[64];
static int        g_mc_n;
static int16_t    g_wheel_l, g_wheel_r;
static int        g_wheel_calls;
static bool       g_line_ctl;
static UINT       g_b5_events;
static uint32_t   g_fault;
static b5_state_t g_b5_state = B5_ST_GUARD;
static int        g_reinit;
static nav_tlm_t  g_tlm;                /* last decoded telemetry frame */
static int        g_frames, g_bad_frames, g_seq_gaps;

ER mc_set_wheel_speeds (int16_t l, int16_t r)
{ g_wheel_l = l; g_wheel_r = r; g_wheel_calls++; return E_OK; }
ER mc_drive_distance (int32_t mm, int16_t v, uint32_t id)
{ (void)v; g_mc[g_mc_n++] = (mc_cmd_t){ 'D', mm, id }; return E_OK; }
ER mc_turn (int32_t dd, int16_t v, uint32_t id)
{ (void)v; g_mc[g_mc_n++] = (mc_cmd_t){ 'T', dd, id }; return E_OK; }
ER mc_stop (uint32_t id)
{ g_mc[g_mc_n++] = (mc_cmd_t){ 'S', 0, id }; return E_OK; }

void line_set_control (bool b)              { g_line_ctl = b; }
void line_notify_state (nav_state_t s)      { (void)s; }
void imu_notify_state (nav_state_t s)       { (void)s; }
void wifi_notify_state (nav_state_t s)      { (void)s; }
void nav_report_fault (uint8_t code)        { g_fault = code; }

void
nav_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    uint8_t prev = g_tlm.seq;

    if (!nav_tlm_unpack(p_frame, len, &g_tlm))
    {
        g_bad_frames++;
        return;
    }

    /* Sequence numbers run 0, 1, 2 ... with no gap (header byte 2). */
    if ((g_frames > 0) && (g_tlm.seq != (uint8_t)(prev + 1U)))
    {
        g_seq_gaps++;
    }

    g_frames++;
}
void nav_reinitialise (void)                { g_reinit++; }
ER   b5_post_event (UINT ev)                { g_b5_events |= ev; return E_OK; }
b5_state_t b5_get_state (void)              { return g_b5_state; }
bool nav_start_allowed (void)                { return true; }
void nav_heartbeat (void)                    { }

/* Answer the most recent Motion Control command. */
static mc_cmd_t
reply_last (nav_motion_result_t result)
{
    mc_cmd_t c = g_mc[g_mc_n - 1];

    (void)nav_post_motion_result(c.id, result);
    run();
    return c;
}

static bool
take_event (UINT ev)
{
    bool b = ((g_b5_events & ev) != 0U);

    g_b5_events &= ~ev;
    return b;
}

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
start_running (void)
{
    (void)nav_post_command(NAV_CMD_START);
    run();
}

static void
test_start_and_line (void)
{
    printf("start, line following, slow request\n");
    start_running();
    CHECK(nav_get_state() == NAV_ST_RUNNING, "running");
    CHECK(nav_get_owner() == NAV_OWNER_LINE, "line owns motion");
    CHECK(g_line_ctl, "line control granted");
    CHECK(take_event(B5_EV_MISSION_START), "Buddy 5 told to start");

    (void)nav_post_line_speeds(150, 140);
    run();
    CHECK((g_wheel_l == 150) && (g_wheel_r == 140), "speeds forwarded");

    b5_nav_request_slow(true);
    (void)nav_post_line_speeds(150, 150);
    run();
    CHECK(g_wheel_l == 75, "slowed to %d", g_wheel_l);
    b5_nav_request_slow(false);
    run();
}

static void
test_obstacle_bypass (void)
{
    mc_cmd_t c;
    int      calls;

    printf("obstacle: stop, bypass right, rejoin, release\n");
    b5_nav_request_stop();
    run();
    CHECK(nav_get_owner() == NAV_OWNER_OBSTACLE, "obstacle owns motion");
    CHECK(!g_line_ctl, "line control revoked");
    CHECK(g_mc[g_mc_n - 1].kind == 'S', "stop sent");
    (void)reply_last(NAV_MOTION_DONE);
    CHECK(take_event(B5_EV_STOP_ACCEPTED), "stop accepted to Buddy 5");

    calls = g_wheel_calls;
    (void)nav_post_line_speeds(150, 150);
    run();
    CHECK(g_wheel_calls == calls, "line speeds ignored during obstacle");

    {
        b5_plan_t plan = { .side = B5_SIDE_RIGHT, .lateral_mm = 200,
                           .forward_mm = 600, .closest_mm = 280,
                           .width_mm = 100 };
        b5_nav_send_plan(&plan);
    }
    run();
    CHECK(take_event(B5_EV_PLAN_ACCEPTED), "plan accepted");
    c = reply_last(NAV_MOTION_DONE);
    CHECK((c.kind == 'T') && (c.value == 450), "turn right 45 (%c %d)",
          c.kind, (int)c.value);
    c = reply_last(NAV_MOTION_DONE);
    CHECK((c.kind == 'D') && (c.value == 282), "diagonal 282 (%d)",
          (int)c.value);
    c = reply_last(NAV_MOTION_DONE);
    CHECK((c.kind == 'T') && (c.value == -450), "turn back");
    CHECK(!take_event(B5_EV_MOTION_DONE), "not done before last leg");
    c = reply_last(NAV_MOTION_DONE);
    CHECK((c.kind == 'D') && (c.value == 400), "parallel 400 (%d)",
          (int)c.value);
    CHECK(take_event(B5_EV_MOTION_DONE), "bypass done to Buddy 5");

    {
        b5_rejoin_t rj = { .lateral_mm = 200, .heading_dd = 0,
                           .attempt = 0, .search_mm = 100 };
        b5_nav_request_rejoin(&rj);
    }
    run();
    c = reply_last(NAV_MOTION_DONE);
    CHECK((c.kind == 'T') && (c.value == -450), "rejoin turns left");
    CHECK((g_mc[g_mc_n - 1].kind == 'D') && (g_mc[g_mc_n - 1].value == 382),
          "rejoin leg 382 (%d)", (int)g_mc[g_mc_n - 1].value);

    g_b5_state = B5_ST_RECOVERING;
    b5_nav_release_motion();            /* line found mid-leg */
    run();
    CHECK(nav_get_owner() == NAV_OWNER_LINE, "line following resumes");
    CHECK(g_line_ctl, "line control granted again");
    (void)reply_last(NAV_MOTION_DONE);  /* late result for the old leg */
    CHECK(!take_event(B5_EV_MOTION_DONE), "stale result ignored");
}

static void
test_barcode (void)
{
    mc_cmd_t c;

    printf("barcode turn, barcode while busy\n");
    (void)nav_post_barcode(NAV_BARCODE_LEFT);
    run();
    CHECK(nav_get_owner() == NAV_OWNER_BARCODE, "barcode owns motion");
    c = reply_last(NAV_MOTION_DONE);
    CHECK((c.kind == 'T') && (c.value == -900), "left 90");
    CHECK(nav_get_owner() == NAV_OWNER_LINE, "back to line");

    b5_nav_request_stop();
    run();
    (void)nav_post_barcode(NAV_BARCODE_RIGHT);
    run();
    CHECK(nav_get_owner() == NAV_OWNER_OBSTACLE, "obstacle not pre-empted");
    (void)reply_last(NAV_MOTION_DONE);
    (void)take_event(B5_EV_STOP_ACCEPTED);
    g_b5_state = B5_ST_PLANNING;
    b5_nav_release_motion();            /* false alarm */
    run();
    CHECK(nav_get_owner() == NAV_OWNER_LINE, "false alarm releases");
}

static void
test_line_search (void)
{
    int i;

    printf("line lost: search found, then search exhausted\n");
    (void)nav_post_line_event(NAV_LINE_LOST);
    run();
    CHECK(nav_get_owner() == NAV_OWNER_SEARCH, "searching");
    (void)reply_last(NAV_MOTION_DONE);  /* the stop */
    CHECK(g_mc[g_mc_n - 1].value == 150, "first sweep +15");
    (void)nav_post_line_event(NAV_LINE_FOUND);
    run();
    CHECK(nav_get_owner() == NAV_OWNER_LINE, "found -> line");

    (void)nav_post_line_event(NAV_LINE_LOST);
    run();
    for (i = 0; i < 5; i++)
    {
        (void)reply_last(NAV_MOTION_DONE);
    }
    CHECK(g_fault == NAV_FAULT_LINE_LOST, "line lost reported");
    CHECK(nav_get_state() == NAV_ST_SAFE_STOP, "safe stop");
    CHECK(take_event(B5_EV_MISSION_STOP), "Buddy 5 stopped");
}

static void
test_states (void)
{
    printf("pause, resume, reset, emergency stop, Buddy 5 fault, timeout\n");
    (void)nav_post_command(NAV_CMD_RESET);
    run();
    CHECK(nav_get_state() == NAV_ST_READY, "reset -> ready");
    start_running();
    (void)nav_post_command(NAV_CMD_PAUSE);
    run();
    CHECK(nav_get_state() == NAV_ST_PAUSED, "paused");
    CHECK(g_mc[g_mc_n - 1].kind == 'S', "motors stopped");
    (void)nav_post_command(NAV_CMD_RESUME);
    run();
    CHECK(nav_get_state() == NAV_ST_RUNNING, "resumed");

    g_fault = 0U;
    (void)nav_post_barcode(NAV_BARCODE_RIGHT);
    run();
    (void)reply_last(NAV_MOTION_CANCELLED);     /* drive inhibited */
    CHECK(g_fault == 0U, "cancel by inhibit is not a motion fault");
    (void)nav_post_safety(NAV_SAFETY_ESTOP);
    run();
    CHECK(nav_get_state() == NAV_ST_ESTOP, "estop");
    (void)nav_post_command(NAV_CMD_RESET);
    run();
    CHECK(nav_get_state() == NAV_ST_ESTOP, "latched until cleared");
    (void)nav_post_safety(NAV_SAFETY_CLEARED);
    (void)nav_post_command(NAV_CMD_RESET);
    run();
    CHECK(nav_get_state() == NAV_ST_READY, "cleared + reset -> ready");
    CHECK(take_event(B5_EV_FAULT_RESET), "Buddy 5 reset too");

    start_running();
    b5_nav_request_stop();
    run();
    (void)reply_last(NAV_MOTION_DONE);
    g_b5_state = B5_ST_FAULT;
    b5_nav_release_motion();            /* Buddy 5 went to Fault */
    run();
    CHECK(nav_get_state() == NAV_ST_SAFE_STOP, "b5 fault -> safe stop");
    CHECK(g_fault == NAV_FAULT_OBSTACLE, "fault reported 0x%x",
          (unsigned)g_fault);

    g_b5_state = B5_ST_GUARD;
    (void)nav_post_command(NAV_CMD_RESET);
    run();
    start_running();
    (void)nav_post_barcode(NAV_BARCODE_UTURN);
    run();
    g_ms += NAV_MOTION_TMO_MS + 100U;   /* Motion Control never answers */
    (void)nav_post_line_speeds(1, 1);   /* wakes the coordinator */
    run();
    CHECK(g_fault == NAV_FAULT_MOTION, "timeout reported");
    CHECK(nav_get_state() == NAV_ST_SAFE_STOP, "timeout -> safe stop");

    (void)nav_post_safety(NAV_SAFETY_FAULT);
    run();
    (void)nav_post_safety(NAV_SAFETY_CLEARED);
    (void)nav_post_command(NAV_CMD_RESET);
    run();
    CHECK((nav_get_state() == NAV_ST_READY) && (g_reinit == 1),
          "fault -> reinitialise -> ready");
}

static void
test_min_bits (void)
{
    int      i;
    uint16_t sent;

    printf("minimum bits: packed inbox, telemetry frame, dropped count\n");
    CHECK(sizeof(nav_msg_t) == 8U, "inbox message %u bytes (was 16)",
          (unsigned)sizeof(nav_msg_t));
    CHECK(g_frames > 0, "frames published");
    CHECK(g_bad_frames == 0, "%d bad frames", g_bad_frames);
    CHECK(NAV_TLM_FRAME_BYTES == 9U, "frame %u bytes (3 + 6)",
          NAV_TLM_FRAME_BYTES);

    (void)nav_post_command(NAV_CMD_RESET);
    run();
    start_running();
    CHECK((g_tlm.state == (uint8_t)NAV_ST_RUNNING) &&
          (g_tlm.owner == (uint8_t)NAV_OWNER_LINE),
          "frame says running / line (%u/%u)", g_tlm.state, g_tlm.owner);

    /* Commands sent: the frame counter moves with each command (16-bit,
     * wrapping, so compare with a uint16_t difference). */
    sent = g_tlm.commands_sent;
    b5_nav_request_slow(true);          /* publishes nothing by itself  */
    b5_nav_request_stop();
    run();
    CHECK(g_tlm.b_slow, "slow flag in frame");
    (void)reply_last(NAV_MOTION_DONE);
    (void)take_event(B5_EV_STOP_ACCEPTED);

    /* Out-of-range values are clamped into their field, never wrapped:
     * a 100 m reverse becomes the longest step the field holds. */
    b5_nav_request_reverse(100000);
    run();
    CHECK((g_mc[g_mc_n - 1].kind == 'D') &&
          (g_mc[g_mc_n - 1].value == -NAV_STEP_VALUE_MAX),
          "reverse clamped to %d (%d)", -NAV_STEP_VALUE_MAX,
          (int)g_mc[g_mc_n - 1].value);

    /* A result whose id we never issued (above 15 bits) is ignored, even
     * if its low bits match the command in progress. */
    (void)nav_post_motion_result(g_mc[g_mc_n - 1].id + 0x8000U,
                                 NAV_MOTION_DONE);
    run();
    CHECK(!take_event(B5_EV_MOTION_DONE), "foreign id ignored");
    (void)reply_last(NAV_MOTION_DONE);
    CHECK(take_event(B5_EV_MOTION_DONE), "real id accepted");
    g_b5_state = B5_ST_GUARD;
    b5_nav_release_motion();            /* owner change publishes */
    run();
    CHECK((uint16_t)(g_tlm.commands_sent - sent) == 2U,
          "commands_sent +2 for stop and reverse (%u -> %u)", sent,
          g_tlm.commands_sent);

    /* Dropped messages are counted by the kernel semaphore and saturate
     * at 255 (8 bits) instead of wrapping to a small number. */
    g_inbox_full = true;

    for (i = 0; i < 300; i++)
    {
        (void)nav_post_line_speeds(100, 100);
    }

    g_inbox_full = false;
    (void)nav_post_command(NAV_CMD_PAUSE);  /* publishes a frame */
    run();
    CHECK(g_tlm.dropped == NAV_DROPPED_MAX, "dropped saturates at 255 (%u)",
          g_tlm.dropped);
    CHECK(g_tlm.state == (uint8_t)NAV_ST_PAUSED, "frame says paused");

    for (i = 0; i < 150; i++)           /* push the 8-bit sequence round */
    {
        (void)nav_post_command(NAV_CMD_RESUME);
        (void)nav_post_command(NAV_CMD_PAUSE);
        run();
    }

    CHECK(g_frames > 256, "%d frames: sequence has wrapped", g_frames);
    CHECK(g_bad_frames == 0, "%d bad frames", g_bad_frames);
    CHECK(g_seq_gaps == 0, "%d sequence gaps (incl. across the wrap)",
          g_seq_gaps);
}

int
main (void)
{
    if ((nav_init() != E_OK) || (nav_start() != E_OK))
    {
        printf("init failed\n");
        return EXIT_FAILURE;
    }

    run();
    test_start_and_line();
    test_obstacle_bypass();
    test_barcode();
    test_line_search();
    test_states();
    test_min_bits();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
