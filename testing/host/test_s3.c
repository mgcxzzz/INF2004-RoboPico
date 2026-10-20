/**
 * @file    test_s3.c
 * @brief   Host test of Buddy 3's pure logic: Code 39 decoding, the
 *          distance-based barcode reader, line following and the
 *          telemetry frame. Compiled unchanged from app_program/subsystem3.
 *
 * The barcode test vectors are the real course barcodes: bar widths
 * measured from Barcode_Sample.pdf (pages 1-4: "*A*", "*A*" half size,
 * "*Z*", "*Z*" half size), in pixels at 150 dpi.
 *
 * Build and run:  make -C testing/host
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "s3_barcode.h"
#include "s3_config.h"
#include "s3_line.h"
#include "s3_reader.h"
#include "../../app_program/system/sys_tlm.h"

static int g_pass, g_fail;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; }                                             \
        else { g_fail++; printf("  FAIL line %d: ", __LINE__);              \
               printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

/* Real course barcodes (Barcode_Sample.pdf, px at 150 dpi). */
static uint32_t const g_pdf[4][29] =
{
    { 41,124,41,42,123,42,124,41,41, 41, 124,42,41,41,41,124,41,42,124, 41,
      41,124,41,41,124,42,123,42,41 },
    { 21,62,21,20,63,20,62,21,21, 21, 62,21,20,21,21,62,21,20,63, 20,
      21,62,21,21,62,20,63,20,21 },
    { 41,124,41,42,123,42,124,41,41, 41, 42,124,123,42,123,42,41,41,42, 41,
      41,124,41,41,124,42,123,42,41 },
    { 21,62,21,20,63,20,62,21,21, 21, 20,62,63,20,62,21,21,21,20, 21,
      21,62,21,20,63,20,62,21,21 }
};
static char const g_pdf_char[4] = { 'A', 'A', 'Z', 'Z' };

/* ---------------------------------------------------------------------
 * Code 39
 * ------------------------------------------------------------------ */
static void
test_table (void)
{
    uint32_t i, j;
    int      bad = 0;

    printf("Code 39 table (checked against a reference encoder)\n");

    for (i = 0U; i < S3_C39_CHARS; i++)
    {
        uint32_t p    = s3_c39_pattern(s3_c39_at(i));
        uint32_t ones = 0U;

        for (j = 0U; j < 9U; j++)
        {
            ones += (p >> j) & 1U;
        }

        for (j = 0U; j < i; j++)
        {
            bad += (s3_c39_pattern(s3_c39_at(j)) == p);
        }

        bad += (ones != 3U) || (s3_c39_char(p) != s3_c39_at(i)) ||
               (s3_c39_index(s3_c39_at(i)) != i);
    }

    CHECK(bad == 0, "44 characters, 3 wide each, all different (%d bad)", bad);
    CHECK(s3_c39_pattern('A') == 0x109U, "A = 100001001");
    CHECK(s3_c39_pattern('Z') == 0x0D0U, "Z = 011010000");
    CHECK(s3_c39_pattern('*') == 0x094U, "* = 010010100");
    CHECK(s3_c39_index('a') == S3_C39_NONE, "lower case is not Code 39");
}

static void
test_course_barcodes (void)
{
    char     text[4];
    uint32_t rev[29];
    int      k;
    uint32_t i;

    printf("the course barcodes from Barcode_Sample.pdf\n");

    for (k = 0; k < 4; k++)
    {
        s3_bc_result_t r = s3_bc_decode(g_pdf[k], 29U, text, sizeof(text));

        CHECK((r == S3_BC_OK) && (text[0] == g_pdf_char[k]) &&
              (text[1] == '\0'), "page %d -> %c (got %d '%s')", k + 1,
              g_pdf_char[k], (int)r, text);

        for (i = 0U; i < 29U; i++)
        {
            rev[i] = g_pdf[k][28U - i];
        }

        r = s3_bc_decode(rev, 29U, text, sizeof(text));
        CHECK((r == S3_BC_BACKWARDS) && (text[0] == '\0'),
              "page %d backwards: no text, reported as backwards", k + 1);
    }
}

static void
test_every_char (void)
{
    uint32_t w[S3_BC_MAX_ELEMENTS];
    char     in[2] = { 0, 0 };
    char     text[4];
    uint32_t i;
    int      bad = 0;

    printf("every character, 3:1 and 2:1 prints\n");

    for (i = 0U; i < (S3_C39_CHARS - 1U); i++)      /* not '*' */
    {
        uint32_t n;

        in[0] = s3_c39_at(i);
        n = s3_bc_encode(in, 30U, 90U, w, S3_BC_MAX_ELEMENTS);
        bad += (s3_bc_decode(w, n, text, sizeof(text)) != S3_BC_OK) ||
               (text[0] != in[0]);
        n = s3_bc_encode(in, 30U, 60U, w, S3_BC_MAX_ELEMENTS);
        bad += (s3_bc_decode(w, n, text, sizeof(text)) != S3_BC_OK) ||
               (text[0] != in[0]);
    }

    CHECK(bad == 0, "43 characters x 2 ratios round trip (%d bad)", bad);
}

static void
test_rejects (void)
{
    uint32_t w[S3_BC_MAX_ELEMENTS];
    char     text[4];
    uint32_t n;

    printf("bad reads send no command\n");
    n = s3_bc_encode("A", 30U, 90U, w, S3_BC_MAX_ELEMENTS);

    CHECK(s3_bc_decode(w, n - 1U, text, sizeof(text)) == S3_BC_BAD_LENGTH,
          "one element missing -> bad length");
    CHECK(s3_bc_decode(w, 9U, text, sizeof(text)) == S3_BC_BAD_LENGTH,
          "a single character -> bad length");

    n = s3_bc_encode("A", 30U, 36U, w, S3_BC_MAX_ELEMENTS);
    CHECK(s3_bc_decode(w, n, text, sizeof(text)) == S3_BC_BAD_RATIO,
          "wide only 1.2 x narrow -> bad ratio (never a guess)");

    n = s3_bc_encode("A", 30U, 90U, w, S3_BC_MAX_ELEMENTS);
    w[0] = 90U; w[1] = 30U;             /* first char no longer '*' */
    CHECK(s3_bc_decode(w, n, text, sizeof(text)) != S3_BC_OK,
          "damaged start character -> rejected");
    CHECK(text[0] == '\0', "no text after a rejected read");

    n = s3_bc_encode("A", 30U, 90U, w, S3_BC_MAX_ELEMENTS);
    w[10] = 90U; w[11] = 90U;           /* 'A' with 4 wide elements */
    CHECK(s3_bc_decode(w, n, text, sizeof(text)) != S3_BC_OK,
          "a character with 4 wide elements -> rejected");
}

/* Random print and sensor noise: a read may fail, but must never come
 * back as a DIFFERENT character (a wrong turn is worse than none). */
static void
test_noise (void)
{
    uint32_t w[S3_BC_MAX_ELEMENTS];
    char     in[2] = { 0, 0 };
    char     text[4];
    int      trial;
    int      ok = 0;
    int      wrong = 0;

    printf("noise: +-25 %% on every element, 2000 random barcodes\n");
    srand(1234U);

    for (trial = 0; trial < 2000; trial++)
    {
        uint32_t n;
        uint32_t i;

        in[0] = s3_c39_at((uint32_t)rand() % (S3_C39_CHARS - 1U));
        n = s3_bc_encode(in, 300U, 900U, w, S3_BC_MAX_ELEMENTS);

        for (i = 0U; i < n; i++)
        {
            int32_t pct = (rand() % 51) - 25;

            w[i] = (uint32_t)((int32_t)w[i] * (100 + pct) / 100);
        }

        if (s3_bc_decode(w, n, text, sizeof(text)) == S3_BC_OK)
        {
            ok    += (text[0] == in[0]);
            wrong += (text[0] != in[0]);
        }
    }

    CHECK(wrong == 0, "never a wrong character (%d wrong)", wrong);
    CHECK(ok == 2000, "all 2000 decoded at +-25 %% on a 3:1 print (%d)", ok);
}

/* ---------------------------------------------------------------------
 * Reader: edges + motion estimate -> widths
 * ------------------------------------------------------------------ */

/* Drive the side sensor over a barcode. Element widths in mm x 100.
 * speed(t) in mm/s is chosen by profile. The motion estimate is whole mm
 * (int16, like Buddy 2's), read every 10 ms; edges get exact us times.
 * Returns the decode result. */
static s3_bc_result_t
drive_over (uint32_t const * p_w_mm100, uint32_t n, int profile,
            bool b_dist_ok, char * p_text, bool * p_b_dist)
{
    s3_reader_t rd;
    uint32_t    edge_pos[S3_RD_MAX_EDGES];   /* mm x 100 from the start */
    uint32_t    k;
    uint32_t    next = 0U;
    double      pos  = -50.0;               /* mm, sensor before barcode */
    double      t    = 0.0;                 /* s */
    int         cycle;

    memset(&rd, 0, sizeof(rd));
    s3_rd_reset(&rd);

    edge_pos[0] = 0U;
    for (k = 0U; k < n; k++)
    {
        edge_pos[k + 1U] = edge_pos[k] + p_w_mm100[k];
    }

    for (cycle = 0; cycle < 2000; cycle++)
    {
        double   dt = 0.010;
        double   v;
        int      sub;

        /* 10 ms of travel in 100 us slices, to time edges exactly. */
        for (sub = 0; sub < 100; sub++)
        {
            double tt = t + (sub * 0.0001);

            if (profile == 0)
            {
                v = 150.0;
            }
            else if (profile == 1)
            {
                v = 150.0 + 60.0 * __builtin_sin(tt * 9.0);    /* 90..210 */
            }
            else if (profile == 2)
            {
                v = ((tt > 0.55) && (tt < 1.55)) ? 0.0 : 150.0;  /* stops */
            }
            else
            {
                /* Stops for 1 s once the sensor is 10 mm in: on the
                 * first WIDE SPACE, the worst place to stop. */
                static double stop_t = -1.0;

                if ((stop_t < 0.0) && (pos >= 10.0))
                {
                    stop_t = tt;
                }

                v = ((stop_t >= 0.0) && (tt < stop_t + 1.0)) ? 0.0 : 150.0;
            }

            pos += v * 0.0001;

            while ((next <= n) && (pos * 100.0 >= (double)edge_pos[next]))
            {
                uint32_t us = (uint32_t)((tt + 0.0001) * 1e6);

                s3_rd_edge(&rd, us, (next % 2U) == 0U);
                next++;
            }
        }

        t += dt;

        /* The task's anchor: whole mm, like the int16 estimate. */
        s3_rd_anchor(&rd, (uint32_t)(t * 1e6),
                     b_dist_ok ? (int32_t)pos * S3_RD_Q : 0, b_dist_ok);

        if (s3_rd_poll(&rd) == S3_RD_READY)
        {
            break;
        }
    }

    return s3_rd_decode(&rd, p_text, 4U, p_b_dist);
}

static void
test_reader (void)
{
    uint32_t       w[S3_BC_MAX_ELEMENTS];
    uint32_t       k;
    char           text[4];
    bool           b_dist;
    s3_bc_result_t r;

    printf("reader: edges timed by interrupt, placed by distance\n");

    /* The real print: px -> mm x 100 (141 mm over 1940 px). */
    for (k = 0U; k < 29U; k++)
    {
        w[k] = (g_pdf[0][k] * 14100U) / 1940U;
    }

    r = drive_over(w, 29U, 0, true, text, &b_dist);
    CHECK((r == S3_BC_OK) && (text[0] == 'A') && b_dist,
          "150 mm/s, distance widths -> A (%d)", (int)r);

    r = drive_over(w, 29U, 1, true, text, &b_dist);
    CHECK((r == S3_BC_OK) && (text[0] == 'A'),
          "speed swinging 90..210 mm/s -> A (%d)", (int)r);

    r = drive_over(w, 29U, 2, true, text, &b_dist);
    CHECK((r == S3_BC_OK) && (text[0] == 'A'),
          "car stops for 1 s in the middle -> still A (%d)", (int)r);

    r = drive_over(w, 29U, 3, true, text, &b_dist);
    CHECK((r == S3_BC_OK) && (text[0] == 'A'),
          "car stops for 1 s ON A SPACE mid-barcode -> still A (%d)", (int)r);

    r = drive_over(w, 29U, 0, false, text, &b_dist);
    CHECK((r == S3_BC_OK) && (text[0] == 'A') && !b_dist,
          "no distance from Motion Control, steady speed: time widths -> A");

    for (k = 0U; k < 29U; k++)
    {
        w[k] = (g_pdf[3][k] * 14100U) / 1940U;          /* half-size Z */
    }

    r = drive_over(w, 29U, 0, true, text, &b_dist);
    CHECK((r == S3_BC_OK) && (text[0] == 'Z'),
          "half-size print (1.5 mm bars), steady speed -> Z (%d)", (int)r);

    /* Known limit: 1 mm distance steps against 1.5 mm bars. With the
     * speed swinging 90..210 mm/s the read may be refused, but it must
     * never come back as another character. */
    r = drive_over(w, 29U, 1, true, text, &b_dist);
    CHECK((r != S3_BC_OK) || (text[0] == 'Z'),
          "half-size, varying speed: refused or Z, never wrong (%d '%s')",
          (int)r, text);
}

static void
test_reader_quiet (void)
{
    s3_reader_t rd;

    printf("reader: the barcode ends only after a quiet zone\n");
    memset(&rd, 0, sizeof(rd));
    s3_rd_reset(&rd);
    s3_rd_anchor(&rd, 0U, 0, true);
    s3_rd_edge(&rd, 1000U, false);
    CHECK(rd.count == 0U, "a white edge cannot start a barcode");
    s3_rd_edge(&rd, 2000U, true);
    s3_rd_edge(&rd, 3000U, true);
    CHECK(rd.count == 1U, "repeated level ignored");
    s3_rd_anchor(&rd, 20000U, 3 * S3_RD_Q, true);
    s3_rd_edge(&rd, 20000U, false);
    s3_rd_anchor(&rd, 30000U, (3 + S3_BC_QUIET_MM - 1) * S3_RD_Q, true);
    CHECK(s3_rd_poll(&rd) == S3_RD_READING, "24 mm of white: not yet");
    s3_rd_anchor(&rd, 40000U, (3 + S3_BC_QUIET_MM) * S3_RD_Q, true);
    CHECK(s3_rd_poll(&rd) == S3_RD_READY, "25 mm of white: ready");
    s3_rd_anchor(&rd, 40000U + (S3_BC_TIMEOUT_MS * 1000U),
                 (3 + S3_BC_QUIET_MM) * S3_RD_Q, true);
    CHECK(s3_rd_poll(&rd) == S3_RD_TIMEOUT, "too long -> timeout");
}

/* ---------------------------------------------------------------------
 * Line following
 * ------------------------------------------------------------------ */
static void
test_line (void)
{
    s3_line_t     ln;
    s3_line_out_t out;
    int           i;
    int           events = 0;

    printf("line: steering, curve vs junction, lost, stuck\n");
    s3_line_reset(&ln);

    s3_line_step(&ln, S3_SENSE_LEFT, 2U, &out);
    CHECK(out.left_mm_s < out.right_mm_s, "line under left sensor: turn left");
    s3_line_step(&ln, 0U, 2U, &out);
    CHECK((out.left_mm_s < out.right_mm_s) &&
          (out.right_mm_s - out.left_mm_s < S3_TURN_OUTER_MM_S - S3_TURN_INNER_MM_S),
          "line between again: lean gently left");
    s3_line_step(&ln, S3_SENSE_RIGHT, 2U, &out);
    CHECK(out.left_mm_s > out.right_mm_s, "line under right sensor: turn right");

    /* A curve: one sensor at a time, never both -> never a junction. */
    s3_line_reset(&ln);
    for (i = 0; i < 300; i++)
    {
        uint32_t s = ((i / 5) % 3 == 0) ? S3_SENSE_LEFT :
                     ((i / 5) % 3 == 1) ? 0U : S3_SENSE_RIGHT;

        s3_line_step(&ln, s, 2U, &out);
        events |= out.events;
    }
    CHECK(events == 0, "zig-zag along a curve: no junction, no loss (0x%x)",
          events);

    /* Junction: both black for >= S3_JUNCTION_MM. */
    s3_line_reset(&ln);
    events = 0;
    for (i = 0; i < 5; i++)                     /* 10 mm: not yet */
    {
        s3_line_step(&ln, S3_SENSE_LEFT | S3_SENSE_RIGHT, 2U, &out);
        events |= out.events;
    }
    CHECK(events == 0, "10 mm both black: not a junction yet");
    s3_line_step(&ln, S3_SENSE_LEFT | S3_SENSE_RIGHT, 2U, &out);
    CHECK((out.events & S3_LEV_JUNCTION) != 0U, "12 mm: junction");
    s3_line_step(&ln, S3_SENSE_LEFT | S3_SENSE_RIGHT, 2U, &out);
    CHECK((out.events & S3_LEV_JUNCTION) == 0U, "reported once");
    s3_line_step(&ln, S3_SENSE_LEFT, 2U, &out);
    for (i = 0; i < 6; i++)
    {
        s3_line_step(&ln, S3_SENSE_LEFT | S3_SENSE_RIGHT, 2U, &out);
    }
    CHECK((out.events & S3_LEV_JUNCTION) != 0U || ln.b_junction,
          "the next junction is reported again");

    /* Lost after S3_LOST_MM of no contact, found again on contact. */
    s3_line_reset(&ln);
    events = 0;
    for (i = 0; i < (S3_LOST_MM / 2) - 1; i++)
    {
        s3_line_step(&ln, 0U, 2U, &out);
        events |= out.events;
    }
    CHECK(events == 0, "248 mm without the line: not lost yet");
    s3_line_step(&ln, 0U, 2U, &out);
    CHECK((out.events & S3_LEV_LOST) != 0U, "250 mm: lost");
    s3_line_step(&ln, 0U, 2U, &out);
    CHECK((out.events & S3_LEV_LOST) == 0U, "lost reported once");
    s3_line_step(&ln, S3_SENSE_RIGHT, 2U, &out);
    CHECK((out.events & S3_LEV_FOUND) != 0U, "found when a sensor sees it");

    /* Standing still never counts as lost (distance, not time). */
    s3_line_reset(&ln);
    events = 0;
    for (i = 0; i < 10000; i++)
    {
        s3_line_step(&ln, 0U, 0U, &out);
        events |= out.events;
    }
    CHECK(events == 0, "standing still for ever: never lost");

    /* Stuck: both black for S3_STUCK_MM. */
    s3_line_reset(&ln);
    events = 0;
    for (i = 0; i < S3_STUCK_MM / 2; i++)
    {
        s3_line_step(&ln, S3_SENSE_LEFT | S3_SENSE_RIGHT, 2U, &out);
        events |= out.events;
    }
    CHECK((events & S3_LEV_STUCK) != 0U, "both black for 400 mm: sensor fault");

    /* 12-bit distance fields saturate, never wrap round to 0. */
    s3_line_reset(&ln);
    for (i = 0; i < 3000; i++)
    {
        s3_line_step(&ln, 0U, 3U, &out);
    }
    CHECK(ln.no_line_mm == 4095U, "distance saturates at 4095 (%u)",
          (unsigned)ln.no_line_mm);
    CHECK(sizeof(s3_line_t) == 4U, "tracker record is one word (%u bytes)",
          (unsigned)sizeof(s3_line_t));
}

/* ---------------------------------------------------------------------
 * Telemetry frame
 * ------------------------------------------------------------------ */
static void
test_frame (void)
{
    s3_tlm_t in;
    s3_tlm_t out;
    uint8_t  f[S3_TLM_FRAME_BYTES];

    printf("telemetry frame (team schema, source 3)\n");
    memset(&in, 0, sizeof(in));
    in.seq = 9U; in.state = 4U; in.sensors = 7U; in.b_granted = true;
    in.side = 3U; in.last_char = (uint8_t)s3_c39_index('Z');
    in.last_result = 5U; in.last_cmd = 4U; in.b_dist = true;
    in.barcodes_ok = 200U; in.bad_reads = 1U; in.backwards = 2U;
    in.junctions = 255U; in.lost = 3U; in.overflows = 4U;
    s3_tlm_pack(&in, f);

    CHECK(S3_TLM_FRAME_BYTES == 15U, "frame 15 bytes (3 + 12)");
    CHECK((f[0] == 0x32U) && (f[1] == 12U) && (f[2] == 9U),
          "header: version 2, source 3, length 12, seq 9");
    CHECK(s3_tlm_unpack(f, sizeof(f), &out), "decodes");
    CHECK((out.state == 4U) && (out.sensors == 7U) && out.b_granted &&
          (out.side == 3U) && (s3_c39_at(out.last_char) == 'Z') &&
          (out.last_result == 5U) && (out.last_cmd == 4U) && out.b_dist,
          "word 0 fields");
    CHECK((out.barcodes_ok == 200U) && (out.bad_reads == 1U) &&
          (out.backwards == 2U) && (out.junctions == 255U) &&
          (out.lost == 3U) && (out.overflows == 4U), "counters");

    in.state = 9U;                              /* out of range */
    s3_tlm_pack(&in, f);
    (void)s3_tlm_unpack(f, sizeof(f), &out);
    CHECK(out.state == 7U, "state saturates in its 3 bits, never wraps");

    CHECK(!s3_tlm_unpack(f, sizeof(f) - 1U, &out), "short frame rejected");
    f[0] = (uint8_t)((TLM_SRC_OBSTACLE << 4) | TLM_VERSION);
    CHECK(!s3_tlm_unpack(f, sizeof(f), &out), "another source's frame rejected");
}

int
main (void)
{
    test_table();
    test_course_barcodes();
    test_every_char();
    test_rejects();
    test_noise();
    test_reader();
    test_reader_quiet();
    test_line();
    test_frame();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
