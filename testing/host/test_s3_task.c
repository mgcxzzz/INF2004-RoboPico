/**
 * @file    test_s3_task.c
 * @brief   Host test of Buddy 3's statechart (task_line.c) against a
 *          simulated track: line sensors, a moving car, and the course's
 *          real barcodes passing under the side sensor with their edges
 *          timed like the GP19 interrupt would.
 *
 * task_line.c and the pure modules are compiled unchanged; the IR driver,
 * the coordinator, the Safety Supervisor and Motion Control are mocked.
 *
 * Build and run:  make -C testing/host
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tk/tkernel.h>

#include "nav_iface.h"
#include "safety_iface.h"
#include "sys_fault.h"
#include "b5_iface.h"
#include "s3_barcode.h"
#include "s3_config.h"
#include "s3_iface.h"
#include "s3_line.h"

static int g_pass, g_fail;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; }                                             \
        else { g_fail++; printf("  FAIL line %d: ", __LINE__);              \
               printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

/* ---------------------------------------------------------------------
 * Mock kernel (s3_step is called directly; no task runs)
 * ------------------------------------------------------------------ */
ID tk_cre_flg (T_CFLG const * p) { (void)p; return 1; }
ER tk_set_flg (ID id, UINT ptn)  { (void)id; (void)ptn; return E_OK; }
ER tk_clr_flg (ID id, UINT ptn)  { (void)id; (void)ptn; return E_OK; }
ER tk_wai_flg (ID id, UINT p, UINT m, UINT * r, TMO t)
{ (void)id; (void)p; (void)m; (void)r; (void)t; return E_TMOUT; }
ID tk_cre_tsk (T_CTSK const * p) { (void)p; return 2; }
ER tk_sta_tsk (ID id, INT s)     { (void)id; (void)s; return E_OK; }

/* ---------------------------------------------------------------------
 * Simulated world
 * ------------------------------------------------------------------ */
#define MAX_EDGES   (256)

static uint32_t g_us;               /* time                          */
static double   g_s;                /* car position along the track, mm */
static bool     g_dist_valid = true;
static uint32_t g_front;            /* front sensor bits this step   */
static uint32_t g_edge_t[MAX_EDGES];
static bool     g_edge_b[MAX_EDGES];
static int      g_eh, g_et;
static uint32_t g_overflows;

/* One barcode on the track: element boundaries in mm from its start. */
static double   g_bc_start = -1.0;
static double   g_bc_edge[32];
static int      g_bc_n;             /* boundaries = elements + 1     */
static double   g_blip = -1.0;      /* a stray 1 mm black mark here  */

ER s3_ir_init (void) { return E_OK; }

static bool
barcode_black (double s)
{
    int k;

    if ((g_blip >= 0.0) && (s >= g_blip) && (s < g_blip + 1.0))
    {
        return true;
    }

    if ((g_bc_start < 0.0) || (s < g_bc_start))
    {
        return false;
    }

    for (k = 0; k < (g_bc_n - 1); k++)
    {
        if ((s >= g_bc_start + g_bc_edge[k]) &&
            (s < g_bc_start + g_bc_edge[k + 1]))
        {
            return (k % 2) == 0;        /* even elements are bars */
        }
    }

    return false;
}

uint32_t
s3_ir_read (void)
{
    return g_front | (barcode_black(g_s) ? S3_SENSE_BARCODE : 0U);
}

bool
s3_ir_pop_edge (uint32_t * p_t, bool * p_b)
{
    if (g_et == g_eh)
    {
        return false;
    }

    *p_t = g_edge_t[g_et];
    *p_b = g_edge_b[g_et];
    g_et = (g_et + 1) % MAX_EDGES;
    return true;
}

uint32_t s3_ir_overflows (void) { return g_overflows; }
uint32_t s3_time_us (void)      { return g_us; }

bool
s3_motion_get_distance (int16_t * p_mm)
{
    *p_mm = (int16_t)(uint16_t)(int32_t)g_s;    /* whole mm, wraps */
    return g_dist_valid;
}

/* ---------------------------------------------------------------------
 * Mock coordinator, safety, fault mask, telemetry
 * ------------------------------------------------------------------ */
static int      g_drives;
static int16_t  g_drive_l, g_drive_r;
static int      g_barcodes;
static nav_barcode_t g_last_bc;
static double   g_bc_post_s;
static int      g_lost, g_found, g_junctions;
static int      g_crit_faults, g_warnings;
static uint8_t  g_last_code;
static uint32_t g_mask;
static int      g_frames, g_bad_frames, g_seq_gaps;
static s3_tlm_t g_tlm;

ER
nav_line_drive (int16_t l, int16_t r)
{
    g_drives++;
    g_drive_l = l;
    g_drive_r = r;
    return E_OK;
}

ER
nav_post_barcode (nav_barcode_t cmd)
{
    g_barcodes++;
    g_last_bc   = cmd;
    g_bc_post_s = g_s;
    return E_OK;
}

ER
nav_post_line_event (nav_line_event_t e)
{
    g_lost      += (e == NAV_LINE_LOST);
    g_found     += (e == NAV_LINE_FOUND);
    g_junctions += (e == NAV_LINE_JUNCTION);
    return E_OK;
}

void safety_kick (safety_src_t s) { (void)s; }
ER   safety_watch (safety_src_t s, uint32_t ms) { (void)s; (void)ms; return E_OK; }

ER
safety_report_fault (safety_src_t src, uint8_t code, bool b_critical)
{
    (void)src;
    g_last_code = code;
    g_crit_faults += b_critical ? 1 : 0;
    g_warnings    += b_critical ? 0 : 1;
    return E_OK;
}

ER sys_fault_set (uint32_t b)   { g_mask |= b; return E_OK; }
ER sys_fault_clear (uint32_t b) { g_mask &= ~b; return E_OK; }

void
line_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    uint8_t prev = g_tlm.seq;

    if (!s3_tlm_unpack(p_frame, len, &g_tlm))
    {
        g_bad_frames++;
        return;
    }

    if ((g_frames > 0) && (g_tlm.seq != (uint8_t)(prev + 1U)))
    {
        g_seq_gaps++;
    }

    g_frames++;
}

/* ---------------------------------------------------------------------
 * Driving
 * ------------------------------------------------------------------ */

/* Front sensors: "zig-zag" along the line (one sensor, none, the other),
 * like a bang-bang follower; or a fixed pattern. */
typedef enum { FRONT_ZIGZAG, FRONT_NONE, FRONT_BOTH, FRONT_LEFT } front_t;

static void
set_front (front_t f, int step)
{
    switch (f)
    {
        case FRONT_ZIGZAG:
            g_front = ((step / 6) % 4 == 0) ? S3_SENSE_LEFT :
                      ((step / 6) % 4 == 2) ? S3_SENSE_RIGHT : 0U;
            break;
        case FRONT_BOTH:  g_front = S3_SENSE_LEFT | S3_SENSE_RIGHT; break;
        case FRONT_LEFT:  g_front = S3_SENSE_LEFT; break;
        default:          g_front = 0U; break;
    }
}

/* Drive `mm` at `speed` mm/s (0 = stand still for mm milliseconds). */
static void
drive (double mm, double speed, front_t front)
{
    static int step = 0;
    double     done = 0.0;

    for (;;)
    {
        double ds = speed * 0.010;
        int    sub;

        if ((speed > 0.0) ? (done >= mm) : (done >= mm))
        {
            break;
        }

        /* 10 ms in 50 us slices: record each barcode-sensor change as
         * an interrupt edge with its time. */
        for (sub = 1; sub <= 200; sub++)
        {
            bool before = barcode_black(g_s);

            g_s += ds / 200.0;

            if (barcode_black(g_s) != before)
            {
                int next = (g_eh + 1) % MAX_EDGES;

                if (next != g_et)
                {
                    g_edge_t[g_eh] = g_us + (uint32_t)(sub * 50);
                    g_edge_b[g_eh] = !before;
                    g_eh = next;
                }
            }
        }

        g_us += 10000U;
        done += (speed > 0.0) ? ds : 10.0;
        set_front(front, step++);
        s3_step();
    }
}

/* Put a course barcode on the track at position s0 (element widths in
 * px at 150 dpi; 141 mm over 1395 px). Optionally reversed. */
static uint32_t const g_pdf_a[29] =
{ 41,124,41,42,123,42,124,41,41, 41, 124,42,41,41,41,124,41,42,124, 41,
  41,124,41,41,124,42,123,42,41 };
static uint32_t const g_pdf_z_half[29] =
{ 21,62,21,20,63,20,62,21,21, 21, 20,62,63,20,62,21,21,21,20, 21,
  21,62,21,20,63,20,62,21,21 };

static void
place_px (uint32_t const * p_px, int n, double s0, bool b_reverse)
{
    int k;

    g_bc_start   = s0;
    g_bc_edge[0] = 0.0;

    for (k = 0; k < n; k++)
    {
        uint32_t px = b_reverse ? p_px[n - 1 - k] : p_px[k];

        g_bc_edge[k + 1] = g_bc_edge[k] + (px * 141.0 / 1940.0);
    }

    g_bc_n = n + 1;
}

static void
place_text (char const * p_text, double s0)
{
    uint32_t w[S3_BC_MAX_ELEMENTS];
    uint32_t n = s3_bc_encode(p_text, 30U, 90U, w, S3_BC_MAX_ELEMENTS);
    int      k;

    g_bc_start   = s0;
    g_bc_edge[0] = 0.0;

    for (k = 0; k < (int)n; k++)
    {
        g_bc_edge[k + 1] = g_bc_edge[k] + (w[k] / 10.0);
    }

    g_bc_n = (int)n + 1;
}

static void
nav (nav_state_t st, bool b_grant)
{
    line_notify_state(st);
    line_set_control(b_grant);
}

/* ---------------------------------------------------------------------
 * Scenarios
 * ------------------------------------------------------------------ */
static void
test_start (void)
{
    printf("start-up, mission start, line following\n");
    CHECK(s3_init() == E_OK, "init");
    s3_step();
    CHECK(s3_get_state() == S3_ST_IDLE, "Initialising -> Idle / Ready");
    drive(50, 150, FRONT_ZIGZAG);
    CHECK(g_drives == 0, "no proposals before the mission starts");

    nav(NAV_ST_RUNNING, true);
    drive(300, 150, FRONT_ZIGZAG);
    CHECK(s3_get_state() == S3_ST_FOLLOW, "Line Following");
    CHECK(g_drives >= 190, "a proposal every 10 ms cycle (%d)", g_drives);
    CHECK((g_lost == 0) && (g_junctions == 0), "no line events on a zig-zag");
    g_front = S3_SENSE_LEFT;
    s3_step();
    CHECK(b5_line_detected(), "Buddy 5 sees 'line detected'");
}

static void
test_barcode_left (void)
{
    int drives;

    printf("course barcode *A* -> one LEFT event at the end, then await\n");
    place_px(g_pdf_a, 29, g_s + 100.0, false);
    drive(100 + 141 + 10, 150, FRONT_ZIGZAG);
    CHECK(s3_get_state() == S3_ST_READ, "Read Barcode while over it");
    CHECK(g_barcodes == 0, "nothing sent while reading");
    drive(40, 150, FRONT_ZIGZAG);
    CHECK((g_barcodes == 1) && (g_last_bc == NAV_BARCODE_LEFT),
          "one LEFT event (%d)", g_barcodes);
    CHECK(s3_get_state() == S3_ST_AWAIT, "Await Navigation");
    CHECK(g_bc_post_s > g_bc_start + 141.0, "sent after the barcode ends");

    drives = g_drives;
    drive(10, 150, FRONT_ZIGZAG);       /* well inside S3_AWAIT_MS */
    CHECK(g_drives == drives, "no proposals while awaiting");

    line_set_control(false);            /* coordinator turns the car */
    drive(500, 0, FRONT_NONE);          /* turn in place, 0.5 s       */
    line_set_control(true);             /* turn done, line again      */
    drive(30, 150, FRONT_ZIGZAG);
    CHECK(s3_get_state() == S3_ST_FOLLOW, "back to Line Following");
    CHECK(g_drives > drives, "proposals resume");
    CHECK(g_barcodes == 1, "still exactly one event (FR-LINE-05)");
    CHECK((g_tlm.last_cmd == 1U) && (s3_c39_at(g_tlm.last_char) == 'A') &&
          (g_tlm.barcodes_ok == 1U) && (g_tlm.b_dist),
          "telemetry: A, LEFT, measured by distance");
}

static void
test_one_event (void)
{
    printf("one barcode = one event, even with a stray mark after it\n");
    place_text("B", g_s + 60.0);
    g_blip = g_s + 60.0 + 141.0 + 30.0;     /* 30 mm after the barcode */
    drive(60 + 141 + 80, 150, FRONT_ZIGZAG);
    CHECK((g_barcodes == 2) && (g_last_bc == NAV_BARCODE_RIGHT),
          "B -> RIGHT once (%d events)", g_barcodes);
    line_set_control(false);
    drive(200, 0, FRONT_NONE);
    line_set_control(true);
    drive(150, 150, FRONT_ZIGZAG);
    CHECK(g_barcodes == 2, "the stray mark did not start another read");
    g_blip = -1.0;
}

static void
test_half_size (void)
{
    printf("half-size sample barcode *Z* (1.5 mm bars): read, not a command\n");
    place_px(g_pdf_z_half, 29, g_s + 60.0, false);
    drive(60 + 71 + 60, 150, FRONT_ZIGZAG);
    CHECK((g_tlm.last_result == (uint8_t)S3_BC_OK) &&
          (s3_c39_at(g_tlm.last_char) == 'Z'), "half-size Z decoded");
    CHECK(g_barcodes == 2, "Z is not in the command table: no event (%d)",
          g_barcodes);
    CHECK(s3_get_state() == S3_ST_FOLLOW, "run continues");
    drive(60, 150, FRONT_ZIGZAG);
    g_bc_start = -1.0;
}

static void
test_commands (void)
{
    printf("write-up table: C -> STRAIGHT, D -> U-TURN\n");
    place_text("C", g_s + 60.0);
    drive(60 + 141 + 40, 150, FRONT_ZIGZAG);
    CHECK((g_barcodes == 3) && (g_last_bc == NAV_BARCODE_STRAIGHT),
          "C -> STRAIGHT (%d)", g_barcodes);
    line_set_control(false);
    line_set_control(true);
    drive(60, 150, FRONT_ZIGZAG);

    place_text("D", g_s + 60.0);
    drive(60 + 141 + 40, 150, FRONT_ZIGZAG);
    CHECK((g_barcodes == 4) && (g_last_bc == NAV_BARCODE_UTURN),
          "D -> U-TURN (%d)", g_barcodes);
    line_set_control(false);
    line_set_control(true);
    drive(60, 150, FRONT_ZIGZAG);
    g_bc_start = -1.0;
}

static void
test_bad_reads (void)
{
    uint8_t bad0;
    uint8_t back0;

    printf("bad, backwards and unknown barcodes send nothing\n");
    bad0 = g_tlm.bad_reads;
    back0 = g_tlm.backwards;

    place_px(g_pdf_a, 29, g_s + 60.0, true);    /* backwards */
    drive(60 + 141 + 80, 150, FRONT_ZIGZAG);
    CHECK(g_barcodes == 4, "backwards: no event");
    CHECK((uint8_t)(g_tlm.backwards - back0) == 1U, "counted as backwards");
    CHECK(s3_get_state() == S3_ST_FOLLOW, "run continues (NFR-LINE-02)");

    place_text("Q", g_s + 60.0);                /* valid, not a command */
    drive(60 + 141 + 80, 150, FRONT_ZIGZAG);
    CHECK(g_barcodes == 4, "unknown letter: no event");

    place_px(g_pdf_a, 29, g_s + 60.0, false);
    g_bc_n = 20;                                /* torn: only 19 elements */
    drive(60 + 141 + 80, 150, FRONT_ZIGZAG);
    CHECK(g_barcodes == 4, "torn barcode: no event");
    CHECK((uint8_t)(g_tlm.bad_reads - bad0) == 2U,
          "unknown + torn counted as bad reads (%u)",
          (unsigned)(uint8_t)(g_tlm.bad_reads - bad0));
    CHECK(s3_get_state() == S3_ST_FOLLOW, "still following");
    g_bc_start = -1.0;
}

static void
test_no_distance (void)
{
    printf("no distance from Motion Control: time widths at steady speed\n");
    g_dist_valid = false;
    place_px(g_pdf_a, 29, g_s + 60.0, false);
    drive(60 + 141 + 80, 150, FRONT_ZIGZAG);
    CHECK((g_barcodes == 5) && (g_last_bc == NAV_BARCODE_LEFT),
          "still decodes A (%d)", g_barcodes);
    CHECK(!g_tlm.b_dist, "telemetry says: measured by time");
    line_set_control(false);
    line_set_control(true);
    drive(60, 150, FRONT_ZIGZAG);
    g_dist_valid = true;
    drive(20, 150, FRONT_ZIGZAG);
    g_bc_start = -1.0;
}

static void
test_ignored_event (void)
{
    printf("coordinator ignores an event (keeps control): resume anyway\n");
    place_text("A", g_s + 60.0);
    drive(60 + 141 + 40, 150, FRONT_ZIGZAG);
    CHECK(s3_get_state() == S3_ST_AWAIT, "awaiting");
    drive(S3_AWAIT_MS + 50, 0, FRONT_ZIGZAG);
    CHECK(s3_get_state() == S3_ST_FOLLOW,
          "resumes after %u ms with control still granted", S3_AWAIT_MS);
    g_bc_start = -1.0;
    drive(60, 150, FRONT_ZIGZAG);
}

static void
test_junction_and_lost (void)
{
    int drives;

    printf("junction, then line lost and found again\n");
    drive(20, 150, FRONT_BOTH);
    CHECK(g_junctions == 1, "both black for 20 mm: one junction (%d)",
          g_junctions);
    drive(30, 150, FRONT_ZIGZAG);

    drive(S3_LOST_MM + 20, 150, FRONT_NONE);
    CHECK(g_lost == 1, "no line for 250 mm: one LOST (%d)", g_lost);
    CHECK(s3_get_state() == S3_ST_AWAIT, "awaits the coordinator's search");
    drives = g_drives;
    line_set_control(false);            /* coordinator searches */
    drive(200, 0, FRONT_NONE);
    CHECK(g_drives == drives, "no proposals during the search");
    drive(10, 0, FRONT_LEFT);
    CHECK(g_found == 1, "line seen: FOUND sent once (%d)", g_found);
    drive(30, 0, FRONT_LEFT);
    CHECK(g_found == 1, "not repeated");
    line_set_control(true);
    drive(30, 150, FRONT_ZIGZAG);
    CHECK(s3_get_state() == S3_ST_FOLLOW, "following again");
}

static void
test_states (void)
{
    printf("pause, emergency stop, stuck sensor fault and reset\n");
    nav(NAV_ST_PAUSED, false);
    s3_step();
    CHECK(s3_get_state() == S3_ST_IDLE, "pause -> Idle / Ready");
    nav(NAV_ST_RUNNING, true);
    s3_step();
    CHECK(s3_get_state() == S3_ST_FOLLOW, "resume -> Line Following");

    nav(NAV_ST_ESTOP, false);
    s3_step();
    CHECK(s3_get_state() == S3_ST_FAULT, "e-stop -> Fault / Inhibited");
    nav(NAV_ST_READY, false);
    s3_step();
    s3_step();
    CHECK(s3_get_state() == S3_ST_IDLE, "authorised reset -> Init -> Idle");

    nav(NAV_ST_RUNNING, true);
    drive(S3_STUCK_MM + 20, 150, FRONT_BOTH);
    CHECK(s3_get_state() == S3_ST_FAULT, "both black 400 mm -> Fault");
    CHECK((g_crit_faults == 1) && (g_last_code == S3_FAULT_STUCK),
          "critical fault reported to Safety (code %u)", g_last_code);
    CHECK((g_mask & SYS_FAULT_LINE_SENSOR) != 0U, "shared fault bit set");
    nav(NAV_ST_FAULT, false);
    drive(50, 0, FRONT_NONE);
    CHECK((g_mask & SYS_FAULT_LINE_SENSOR) != 0U, "bit latched until reset");
    nav(NAV_ST_READY, false);
    s3_step();
    s3_step();
    CHECK(s3_get_state() == S3_ST_IDLE, "reset -> Idle");
    CHECK((g_mask & SYS_FAULT_LINE_SENSOR) == 0U, "bit cleared after reset");
}

static void
test_overflow (void)
{
    printf("lost interrupt edges abort the read and warn\n");
    nav(NAV_ST_RUNNING, true);
    drive(60, 150, FRONT_ZIGZAG);
    place_px(g_pdf_a, 29, g_s + 30.0, false);
    drive(30 + 60, 150, FRONT_ZIGZAG);
    g_overflows++;
    drive(81 + 60, 150, FRONT_ZIGZAG);
    CHECK(g_warnings >= 1, "edge overflow reported as a warning");
    CHECK(g_barcodes == 6, "no event from a damaged read (%d)", g_barcodes);
    g_bc_start = -1.0;
}

int
main (void)
{
    test_start();
    test_barcode_left();
    test_one_event();
    test_half_size();
    test_commands();
    test_bad_reads();
    test_no_distance();
    test_ignored_event();
    test_junction_and_lost();
    test_states();
    test_overflow();

    printf("telemetry: %d frames\n", g_frames);
    CHECK((g_bad_frames == 0) && (g_seq_gaps == 0),
          "every frame decodes, no sequence gaps (%d bad, %d gaps)",
          g_bad_frames, g_seq_gaps);

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
