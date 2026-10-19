/**
 * @file    test_b5_profile.c
 * @brief   Host unit tests for obstacle_profile.c (no Pico needed).
 *
 * A simulated HC-SR04+ with a beam half-angle of ALPHA_TRUE_DEG looks at
 * flat-faced obstacles. The test runs the same Coarse Sweep -> Edge
 * Refinement -> Planning sequence as task_obstacle.c and checks the result.
 *
 * Build and run:  make -C testing/host
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "b5_config.h"
#include "obstacle_profile.h"

#define ALPHA_TRUE_DEG  (7.5)
#define PI              (3.14159265358979)

/* A flat obstacle face parallel to the car: x from x1 (left) to x2
 * (right), at forward distance y. x is positive to the right. */
typedef struct
{
    double x_left;
    double x_right;
    double y;
} face_t;

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; }                                             \
        else { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__);      \
               printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

/* Range the simulated sensor reports at a servo angle (tenths of a deg,
 * signed: -900 left, 0 ahead, +900 right). The geometry below measures
 * angles from the right-hand x axis, so convert first. */
static int32_t
sim_range (face_t const * p_faces, int n_faces, int16_t angle_dd)
{
    double a     = ((900 - angle_dd) / 10.0) * PI / 180.0;
    double half  = ALPHA_TRUE_DEG * PI / 180.0;
    double best  = B5_RANGE_FAR_MM;
    int    f;

    for (f = 0; f < n_faces; f++)
    {
        /* Angles (from the right-hand x axis) of the face end points. */
        double a_right = atan2(p_faces[f].y, p_faces[f].x_right);
        double a_left  = atan2(p_faces[f].y, p_faces[f].x_left);
        double lo      = fmax(a - half, a_right);
        double hi      = fmin(a + half, a_left);
        double t;

        if (lo > hi)
        {
            continue;           /* face outside the beam               */
        }

        /* Closest face point inside the beam window. */
        for (t = lo; t <= hi + 1e-9; t += (hi - lo) / 50.0 + 1e-6)
        {
            double r = p_faces[f].y / sin(t);
            best = fmin(best, r);
        }
    }

    return (int32_t)lround(best);
}

static b5_params_t
default_params (void)
{
    b5_params_t p;

    p.near_mm           = B5_NEAR_MM;
    p.alpha_dd          = (int16_t)lround(ALPHA_TRUE_DEG * 10.0);
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

/* Coarse sweep + edge refinement, exactly as the task does it. */
static int
run_scan (face_t const * p_faces, int n, int16_t refine_stop_dd,
          b5_scan_t * p_scan)
{
    int16_t angle;
    int     extra = 0;

    b5_scan_clear(p_scan);

    for (angle = B5_SCAN_START_DD; angle <= B5_SCAN_END_DD;
         angle = (int16_t)(angle + B5_SCAN_STEP_DD))
    {
        (void)b5_scan_add(p_scan, angle, sim_range(p_faces, n, angle), true);
    }

    while ((extra < (int)B5_MAX_REFINE_SAMPLES) &&
           b5_next_refine_angle(p_scan, B5_NEAR_MM, refine_stop_dd, &angle))
    {
        (void)b5_scan_add(p_scan, angle, sim_range(p_faces, n, angle), true);
        extra++;
    }

    return extra;
}

static void
test_trig (void)
{
    printf("trig and median\n");
    CHECK(b5_cos_q15(0) == 32767, "cos 0");
    CHECK(b5_cos_q15(900) == 0, "cos 90");
    CHECK(b5_cos_q15(1800) == -32767, "cos 180");
    CHECK(abs(b5_cos_q15(600) - 16384) <= 2, "cos 60 = %d", b5_cos_q15(600));
    CHECK(abs(b5_sin_q15(300) - 16384) <= 2, "sin 30 = %d", b5_sin_q15(300));
    CHECK(abs(b5_cos_q15(75) - 32487) <= 4, "cos 7.5 = %d", b5_cos_q15(75));
    CHECK(b5_median3(5, 1, 9) == 5, "median");
    CHECK(b5_median3(9, 9, 1) == 9, "median dup");
}

static void
test_no_obstacle (void)
{
    b5_scan_t    scan;
    b5_profile_t prof;
    b5_params_t  par = default_params();

    printf("no obstacle -> Planning returns to Guard\n");
    (void)run_scan(NULL, 0, B5_ALPHA_DD, &scan);
    b5_build_profile(&scan, &par, &prof);
    CHECK(!b5_scan_has_near(&scan, B5_NEAR_MM), "no near points");
    CHECK(!prof.b_found, "nothing found");
    CHECK(prof.side == B5_SIDE_NONE, "no side");
    CHECK(prof.confidence_pct == 100U, "confidence %u", prof.confidence_pct);
}

/* NFR-USO-02 grid: widths 50-200 mm at 200-400 mm, centred and offset. */
static void
test_width_grid (int16_t refine_stop_dd, double * p_worst_pct,
                 int * p_max_extra)
{
    static double const widths[] = { 50.0, 100.0, 150.0, 200.0 };
    static double const ranges[] = { 200.0, 300.0, 400.0 };
    static double const shifts[] = { -40.0, 0.0, 40.0 };
    size_t w;
    size_t r;
    size_t s;

    *p_worst_pct = 0.0;
    *p_max_extra = 0;

    for (w = 0; w < 4; w++)
    {
        for (r = 0; r < 3; r++)
        {
            for (s = 0; s < 3; s++)
            {
                face_t       face;
                b5_scan_t    scan;
                b5_profile_t prof;
                b5_params_t  par = default_params();
                double       err;
                int          extra;

                face.x_left  = shifts[s] - (widths[w] / 2.0);
                face.x_right = shifts[s] + (widths[w] / 2.0);
                face.y       = ranges[r];

                extra = run_scan(&face, 1, refine_stop_dd, &scan);
                b5_build_profile(&scan, &par, &prof);

                err = 100.0 * fabs(prof.width_mm - widths[w]) / widths[w];
                *p_worst_pct = fmax(*p_worst_pct, err);
                *p_max_extra = (extra > *p_max_extra) ? extra : *p_max_extra;

                CHECK(prof.b_found, "found w=%.0f r=%.0f", widths[w],
                      ranges[r]);
            }
        }
    }
}

static void
test_side_choice (void)
{
    face_t       faces[2];
    b5_scan_t    scan;
    b5_profile_t prof;
    b5_params_t  par = default_params();

    printf("side choice (NFR-USO-03)\n");

    /* Obstacle ahead, a wall 250 mm to its left: must go right. */
    faces[0] = (face_t){ -50.0, 50.0, 300.0 };
    faces[1] = (face_t){ -800.0, -300.0, 350.0 };
    (void)run_scan(faces, 2, B5_REFINE_STOP_DD, &scan);
    b5_build_profile(&scan, &par, &prof);
    CHECK(prof.side == B5_SIDE_RIGHT, "wall left -> right (got %d)",
          prof.side);
    CHECK(prof.clear_right_mm > prof.clear_left_mm, "right %ld > left %ld",
          (long)prof.clear_right_mm, (long)prof.clear_left_mm);

    /* Mirror image: wall on the right, must go left. */
    faces[1] = (face_t){ 300.0, 800.0, 350.0 };
    (void)run_scan(faces, 2, B5_REFINE_STOP_DD, &scan);
    b5_build_profile(&scan, &par, &prof);
    CHECK(prof.side == B5_SIDE_LEFT, "wall right -> left (got %d)",
          prof.side);
    CHECK(prof.lateral_mm > 0, "lateral shift %ld", (long)prof.lateral_mm);
    CHECK(prof.forward_mm > prof.closest_mm, "forward travel past it");

    /* Obstacles beside it on both sides, running past the scan arc:
     * no gap wide enough, so Blocked (FR-USO-07). */
    {
        face_t boxed[3] = { { -50.0, 50.0, 300.0 },
                            { 120.0, 1500.0, 350.0 },
                            { -1500.0, -120.0, 350.0 } };

        (void)run_scan(boxed, 3, B5_REFINE_STOP_DD, &scan);
    }
    b5_build_profile(&scan, &par, &prof);
    CHECK(prof.side == B5_SIDE_NONE, "boxed in -> blocked (got %d)",
          prof.side);
}

static void
test_wide_obstacle (void)
{
    face_t       face = { -900.0, 900.0, 300.0 };
    b5_scan_t    scan;
    b5_profile_t prof;
    b5_params_t  par = default_params();

    printf("obstacle wider than the scan arc -> blocked\n");
    (void)run_scan(&face, 1, B5_REFINE_STOP_DD, &scan);
    b5_build_profile(&scan, &par, &prof);
    CHECK(prof.b_right_at_limit && prof.b_left_at_limit, "both at limit");
    CHECK(prof.side == B5_SIDE_NONE, "blocked");
}

static void
test_invalid_samples (void)
{
    b5_scan_t    scan;
    b5_profile_t prof;
    b5_params_t  par = default_params();

    printf("dead sensor -> low confidence\n");
    b5_scan_clear(&scan);
    (void)b5_scan_add(&scan, -600, 0, false);
    (void)b5_scan_add(&scan, -300, 0, false);
    (void)b5_scan_add(&scan, 0, 250, true);
    (void)b5_scan_add(&scan, 300, 0, false);
    (void)b5_scan_add(&scan, 600, 0, false);
    b5_build_profile(&scan, &par, &prof);
    CHECK(prof.confidence_pct < B5_MIN_CONFIDENCE_PCT, "confidence %u",
          prof.confidence_pct);
}

int
main (void)
{
    double worst;
    int    extra;

    test_trig();
    test_no_obstacle();

    printf("width accuracy, refinement stops at alpha (Week 6 text)\n");
    test_width_grid(B5_ALPHA_DD, &worst, &extra);
    printf("  worst width error %.1f %%, max refinement samples %d"
           " (for comparison only)\n", worst, extra);

    printf("width accuracy, refinement stops at B5_REFINE_STOP_DD\n");
    test_width_grid(B5_REFINE_STOP_DD, &worst, &extra);
    printf("  worst width error %.1f %%, max refinement samples %d\n",
           worst, extra);
    CHECK(worst <= 10.0, "NFR-USO-02 grid worst %.1f %% > 10 %%", worst);
    CHECK(extra <= (int)B5_MAX_REFINE_SAMPLES, "refinement bound %d", extra);

    test_side_choice();
    test_wide_obstacle();
    test_invalid_samples();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
