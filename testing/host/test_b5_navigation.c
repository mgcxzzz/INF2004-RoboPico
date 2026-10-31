#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../app_program/subsystem5/obstacle_profile.h"

#define CHECK(x) do { if (!(x)) { \
    (void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); \
    return 1; } } while (0)

static b5_params_t params(void)
{
    b5_params_t p = { 550, 0, -550, 550, 155, 190, 50, 200, 1000, true };
    return p;
}

static void fill_open(b5_scan_t *scan)
{
    int a;
    b5_scan_clear(scan);
    for (a = -550; a <= 550; a += 55)
    {
        (void)b5_scan_add(scan, (int16_t)a, 4000, true);
    }
}

static void fill_box(b5_scan_t *scan, int half_angle, int range)
{
    int a;
    fill_open(scan);
    for (a = -half_angle; a <= half_angle; a += 55)
    {
        (void)b5_scan_add(scan, (int16_t)a, range, true);
    }
}

int main(void)
{
    b5_scan_t scan;
    b5_profile_t p;
    b5_params_t par = params();
    uint8_t i;

    /* Isolated near return is surfaced as unknown and cannot pass. */
    fill_open(&scan);
    (void)b5_scan_add(&scan, 0, 250, true);
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.b_found != 0U);
    CHECK(p.side == B5_SIDE_NONE);
    CHECK((p.evidence_flags & B5_PROFILE_EVIDENCE_SUPPORTED) == 0U);

    /* Near/far jitter in the hysteresis band cannot establish support. */
    fill_open(&scan);
    (void)b5_scan_add(&scan, -55, 540, true);
    (void)b5_scan_add(&scan, 0, 560, true);
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.side == B5_SIDE_NONE);
    CHECK((p.evidence_flags & B5_PROFILE_EVIDENCE_SUPPORTED) == 0U);

    /* No echoes are not free-space evidence. */
    b5_scan_clear(&scan);
    for (i = 0U; i < 8U; i++)
    {
        (void)b5_scan_add(&scan, (int16_t)(-220 + (i * 55)), 4000, false);
    }
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.b_found == 0U);
    CHECK(p.side == B5_SIDE_NONE);
    CHECK(p.clear_left_mm == 0U && p.clear_right_mm == 0U);

    /* Two near hits with a missing angle cannot prove adjacent support or sides. */
    fill_open(&scan);
    (void)b5_scan_add(&scan, -55, 250, true);
    (void)b5_scan_add(&scan, 0, 250, false);
    (void)b5_scan_add(&scan, 55, 250, true);
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.b_found != 0U);
    CHECK(p.side == B5_SIDE_NONE);

    /* Width never shrinks from observed span; planner keeps its minimum floor. */
    fill_box(&scan, 165, 250); /* simulated broad beam / target */
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.b_found != 0U);
    CHECK(p.width_mm == p.width_raw_mm);
    CHECK(p.planning_width_mm >= p.width_mm);
    CHECK(p.planning_width_mm >= p.width_mm + p.span_uncertainty_mm);
    CHECK(p.planning_width_mm >= 200U);
    CHECK(p.span_uncertainty_mm > 0U);

    /* A complete supported scan can prove a side; a narrow corridor cannot.
     * (300 mm: with the measured car geometry of 2026-11-01 the swept
     * footprint of the 45 deg manoeuvre does not clear the >= 200 mm
     * planning envelope from a 250 mm stop.) */
    fill_box(&scan, 55, 300);
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK((p.evidence_flags & B5_PROFILE_EVIDENCE_SUPPORTED) != 0U);
    CHECK(p.side != B5_SIDE_NONE);
    par.car_width_mm = 900;
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.side == B5_SIDE_NONE);
    par = params();

    /* Broad apparent span at 300 mm with distant background: exercises the
     * ray/plane projection at 300+ mm (width and both clearances measured).
     * With the measured 2026-11-01 car geometry the swept footprint of the
     * 45 deg manoeuvre no longer clears this ~370 mm planning envelope, so
     * no side is required here any more (it was before). */
    b5_scan_clear(&scan);
    for (i = 0U; i < 21U; i++)
    {
        int32_t a = -550 + ((int32_t)i * 55);
        int32_t r = (abs(a) <= 275) ? 300 : 2000;
        (void)b5_scan_add(&scan, (int16_t)a, r, true);
    }
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.width_mm >= 250U && p.width_mm <= 350U);
    CHECK(p.clear_left_mm >= 255U && p.clear_right_mm >= 255U);

    /* Missing side coverage and obstacle at scan boundary must block. */
    fill_box(&scan, 55, 250);
    for (i = 0U; i < scan.count; i++)
    {
        if (scan.sample[i].angle_dd > 220)
        {
            scan.sample[i].b_valid = 0U;
        }
    }
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.side != B5_SIDE_RIGHT);

    /* A valid far pair after a wide missing wedge cannot establish clearance. */
    b5_scan_clear(&scan);
    for (i = 0U; i < 3U; i++)
    {
        (void)b5_scan_add(&scan, (int16_t)(-55 + (i * 55)), 250, true);
    }
    for (i = 0U; i < 4U; i++)
    {
        (void)b5_scan_add(&scan, (int16_t)(385 + (i * 55)), 4000, true);
    }
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.b_found != 0U);
    CHECK(p.clear_right_mm == 0U);
    CHECK(p.side != B5_SIDE_RIGHT);

    fill_open(&scan);
    (void)b5_scan_add(&scan, 495, 250, true);
    (void)b5_scan_add(&scan, 550, 250, true);
    b5_build_navigation_profile(&scan, &par, &p);
    CHECK(p.b_right_at_limit != 0U);
    CHECK(p.side != B5_SIDE_RIGHT);

    /* Unknown side returns and a second near obstacle constrain clearance. */
    fill_box(&scan, 55, 250);
    b5_build_navigation_profile(&scan, &par, &p);
    {
        uint32_t baseline = p.clear_right_mm;
        (void)b5_scan_add(&scan, 220, 300, true);
        b5_build_navigation_profile(&scan, &par, &p);
        CHECK(p.clear_right_mm <= baseline);
        CHECK(p.side != B5_SIDE_RIGHT);
    }

    /* Mirroring the scan swaps lateral edges and preserves the chosen side. */
    {
        b5_profile_t mirrored;
        fill_box(&scan, 55, 250);
        b5_build_navigation_profile(&scan, &par, &p);
        b5_scan_clear(&scan);
        for (i = 0U; i < 21U; i++)
        {
            int32_t a = -550 + ((int32_t)i * 55);
            int32_t r = ((a >= -55) && (a <= 55)) ? 250 : 4000;
            (void)b5_scan_add(&scan, (int16_t)-a, r, true);
        }
        par.b_prefer_right = false;
        b5_build_navigation_profile(&scan, &par, &mirrored);
        CHECK(p.b_found == mirrored.b_found);
        CHECK(p.width_mm == mirrored.width_mm);
        CHECK((p.side == B5_SIDE_RIGHT && mirrored.side == B5_SIDE_LEFT) ||
              (p.side == B5_SIDE_LEFT && mirrored.side == B5_SIDE_RIGHT) ||
              (p.side == B5_SIDE_NONE && mirrored.side == B5_SIDE_NONE));
        CHECK(p.clear_left_mm == mirrored.clear_right_mm);
        CHECK(p.clear_right_mm == mirrored.clear_left_mm);
        par = params();
    }

    (void)printf("navigation profiler checks passed\n");
    return 0;
}
