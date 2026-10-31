/**
 * b5_profile_bench.c v0.7.4 - Buddy 5 rule-B bench (Pico SDK). Wheels never driven.
 * v0.7.4: symmetric sweep +/-46 deg in 4.6 deg steps (21 angles) from b5_config.h;
 * otherwise = v0.7.3.
 * v0.7.3: side clearances compared over the same angles (+/-44 deg,
 * B5_SIDE_COMPARE_SYMMETRIC); otherwise = v0.7.2.
 * v0.7.2: measured car geometry from b5_config.h (2026-11-01: pivot 130 mm behind
 * the front IR bar, sensor 65 mm behind it, width 135 mm). Otherwise = v0.7.1.
 * v0.7.1: sweep -55..+44 deg from b5_config.h (right end stops short of the
 * frame; 2026-10-31 end-stop check). Otherwise identical to v0.7.
 *
 * Runs the REAL planner (obstacle_profile.c: b5_classify_ping,
 * b5_combine_pings, b5_build_navigation_profile_ex with the swept-footprint
 * check; constants from b5_config.h) on real scans. Scan order = firmware:
 * 21 coarse angles (-55..+55 deg, 5.5 deg), 3 pings each, edge refinement,
 * planning.
 *
 * Ping transaction (same sequence as the RTOS driver after rule B):
 *   1. ECHO must read LOW before TRIG (up to B5_PRETRIG_LOW_WAIT_MS),
 *      else FAULT stuck-high and TRIG is not pulsed;
 *   2. TRIG 12 us; wait for the rise (30 ms), else FAULT no-rise;
 *   3. wait for the fall (B5_NORETURN_WAIT_MS; 'w' = 250 ms diagnostic),
 *      else FAULT no-fall; pulses < ECHO_MIN are noise (keep waiting);
 *   4. width <= ECHO_MAX = ECHO (range), longer = completed NO_RETURN.
 * Every ping is printed with its class, width and rise latency, so the
 * sensor's real no-target behaviour can be read off the log.
 *
 * Robo Pico: servo GP12, HC-SR04+ TRIG GP16 / ECHO GP17 (3.3 V). USB serial. h = help.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "b5_config.h"
#include "obstacle_profile.h"

#define PIN_SERVO (12U)
#define PIN_TRIG  (16U)
#define PIN_ECHO  (17U)

#define US_RISE_TIMEOUT_US (30000U)
#define US_MIN_GAP_US      (B5_PING_PERIOD_MS * 1000U)

static int      g_centre_us = 1450;          /* S5-C01; ',' '.' trim */
static int      g_pulse     = 1450;
static int      g_angle_dd  = 0;
static uint32_t g_settle_ms = B5_SERVO_SETTLE_MS;
static uint32_t g_fall_wait_ms = B5_NORETURN_WAIT_MS;   /* 'w' toggles 250 */
static uint64_t g_last_ping;
static unsigned g_scan_no;
static int      g_stop_mm = B5_D_STOP_MM;     /* bench only: 1/5/3 keys */
static int      g_inject;                     /* 'i': 0 none, 1 stuck-high, 2 no-rise (one scan) */

/* per-ping diagnostics of the last ping */
typedef struct { b5_ping_t p; uint32_t pre_us, rise_lat_us; uint8_t n_short; } ping_diag_t;

/* scan-wide counters */
static unsigned g_n_cls[3], g_n_why[5];
static uint32_t g_nr_w_min, g_nr_w_max;

/* ---------------------------------------------------------------- servo */
static int angle_to_pulse(int angle_dd)       /* as drv_servo.c, + = right */
{
    int from_right = 900 - angle_dd;
    return (int)B5_SERVO_MIN_US + ((int)(B5_SERVO_MAX_US - B5_SERVO_MIN_US) * from_right) / 1800
           + (g_centre_us - 1450);
}
static void servo_write_angle(int angle_dd)
{
    if (angle_dd < B5_SCAN_START_DD) angle_dd = B5_SCAN_START_DD;
    if (angle_dd > B5_SCAN_END_DD) angle_dd = B5_SCAN_END_DD;
    g_angle_dd = angle_dd;
    g_pulse = angle_to_pulse(angle_dd);
    pwm_set_gpio_level(PIN_SERVO, (uint16_t)g_pulse);
}
static void servo_move(int angle_dd)
{
    int step = (int)((B5_SERVO_SPEED_DD_S * B5_SERVO_STEP_MS) / 1000U);
    while (g_angle_dd != angle_dd) {
        int d = angle_dd - g_angle_dd;
        servo_write_angle(g_angle_dd + (d > step ? step : (d < -step ? -step : d)));
        sleep_ms(B5_SERVO_STEP_MS);
    }
    sleep_ms(g_settle_ms);
}
static void servo_init(void)
{
    uint slice = pwm_gpio_to_slice_num(PIN_SERVO);
    pwm_config cfg = pwm_get_default_config();
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 19999U);
    pwm_init(slice, &cfg, false);
    g_angle_dd = 0;
    g_pulse = angle_to_pulse(0);
    pwm_set_gpio_level(PIN_SERVO, (uint16_t)g_pulse);
    pwm_set_enabled(slice, true);
}

/* ----------------------------------------------------------- ultrasonic */
static void ping(ping_diag_t *d)
{
    uint64_t t0, trig_end, rise = 0, now, gap = time_us_64() - g_last_ping;
    bool b_rise = false, b_fall = false, b_short = false;
    uint32_t width = 0U;

    d->n_short = 0U; d->rise_lat_us = 0U; d->pre_us = 0U;
    if (gap < US_MIN_GAP_US) sleep_us((uint32_t)(US_MIN_GAP_US - gap));

    if (g_inject == 1) { d->p = b5_classify_ping(false, false, false, false, 0U); g_last_ping = time_us_64(); return; }
    if (g_inject == 2) { d->p = b5_classify_ping(true, false, false, false, 0U); g_last_ping = time_us_64(); return; }

    /* 1. ECHO low before TRIG */
    t0 = time_us_64();
    while (gpio_get(PIN_ECHO)) {
        if (time_us_64() - t0 > (uint64_t)B5_PRETRIG_LOW_WAIT_MS * 1000U) {
            d->pre_us = (uint32_t)(time_us_64() - t0);
            d->p = b5_classify_ping(false, false, false, false, 0U);
            g_last_ping = time_us_64();
            return;
        }
    }
    d->pre_us = (uint32_t)(time_us_64() - t0);

    /* 2. trigger, rise */
    gpio_put(PIN_TRIG, 1); busy_wait_us_32(B5_TRIG_PULSE_US); gpio_put(PIN_TRIG, 0);
    trig_end = time_us_64();
    g_last_ping = trig_end;
    for (;;) {
        while (!gpio_get(PIN_ECHO)) { if (time_us_64() - trig_end > US_RISE_TIMEOUT_US) goto done; }
        rise = time_us_64();
        b_rise = true;
        /* 3. fall */
        for (;;) {
            now = time_us_64();
            if (!gpio_get(PIN_ECHO)) break;
            if (now - rise > (uint64_t)g_fall_wait_ms * 1000U) { b_fall = false; goto done; }
        }
        width = (uint32_t)(time_us_64() - rise);
        if (width >= B5_ECHO_MIN_US) { b_fall = true; break; }
        b_short = true; d->n_short++; b_rise = false;          /* noise: wait for the real echo */
    }
done:
    if (b_rise) d->rise_lat_us = (uint32_t)(rise - trig_end);
    d->p = b5_classify_ping(true, b_rise, b_fall, b_short, width);
}

static void count_ping(b5_ping_t const *p)
{
    g_n_cls[p->cls % 3U]++;
    if (p->cls == B5_PING_FAULT && p->why < 5U) g_n_why[p->why]++;
    if (p->cls == B5_PING_NO_RETURN) {
        if (g_nr_w_min == 0U || p->width_us < g_nr_w_min) g_nr_w_min = p->width_us;
        if (p->width_us > g_nr_w_max) g_nr_w_max = p->width_us;
    }
}

static char const *cls_tag(b5_ping_t const *p)
{
    static char const *why[5] = { "F", "Fhi", "Frise", "Ffall", "Fshort" };
    if (p->cls == B5_PING_ECHO) return "E";
    if (p->cls == B5_PING_NO_RETURN) return "NR";
    return why[p->why < 5U ? p->why : 0U];
}

/* Same rule as task_obstacle.c sample_at() (b5_combine_pings) */
static char g_raw[64];
static void sample_at(int angle_dd, unsigned n_pings, b5_sample_t *p_out)
{
    b5_ping_t p[B5_SAMPLES_PER_ANGLE];
    ping_diag_t d;
    int len = 0;
    if (n_pings > B5_SAMPLES_PER_ANGLE) n_pings = B5_SAMPLES_PER_ANGLE;
    servo_move(angle_dd);
    g_raw[0] = '\0';
    for (unsigned i = 0U; i < n_pings; i++) {
        ping(&d);
        p[i] = d.p;
        count_ping(&d.p);
        if (len < (int)sizeof g_raw - 20)
            len += snprintf(g_raw + len, sizeof g_raw - (size_t)len, "%s%s%u/%u", i ? "," : "",
                            cls_tag(&d.p), (unsigned)(d.p.cls == B5_PING_ECHO ? d.p.range_mm : d.p.width_us),
                            (unsigned)d.rise_lat_us);
    }
    b5_combine_pings((int16_t)angle_dd, p, (uint8_t)n_pings, p_out, NULL);
}

static b5_params_t params(void)               /* = task_obstacle.c profile_params() */
{
    b5_params_t p;
    p.near_mm = B5_NEAR_MM; p.alpha_dd = B5_ALPHA_DD;
    p.scan_min_dd = B5_SCAN_START_DD; p.scan_max_dd = B5_SCAN_END_DD;
    p.car_width_mm = B5_CAR_WIDTH_MM; p.car_length_mm = B5_CAR_LENGTH_MM;
    p.margin_mm = B5_MARGIN_MM; p.obst_depth_mm = B5_OBST_DEPTH_MM;
    p.open_clearance_mm = B5_OPEN_CLEARANCE_MM; p.b_prefer_right = (B5_PREFER_RIGHT != 0);
    return p;
}

static char sfx(b5_sample_t const *s)
{
    if (!s->b_valid) return 'x';
    if (s->b_weak) return 'w';
    if (s->b_block_only) return 'b';
    if (s->b_partial) return 'p';
    return s->b_stable ? ' ' : 'u';
}

static void print_sweep(const char *side, b5_sweep_report_t const *r)
{
    static const char *res[4] = { "not run (gap too small)", "FAIL", "PASS_ASSUMED", "PASS" };
    static const char *why[6] = { "-", "planning envelope", "measured return inside footprint",
                                  "behind a side return", "scanned but unknown", "policy" };
    printf("[b5 sweep] %s: %s; weak %d; outline points %d (assumed %d, weak %d)", side, res[r->result & 3U],
           (int)r->b_weak, (int)r->n_points, (int)r->n_assumed, (int)r->n_weak);
    if (r->result == B5_SWEEP_FAIL)
        printf("; why %s at leg %d (%d, %d) mm", why[r->why < 6U ? r->why : 0U], (int)r->fail_leg,
               (int)r->fail_x_mm, (int)r->fail_y_mm);
    printf("\n");
}

static void scan_and_plan(void)
{
    static b5_scan_t scan;
    static char rawline[48][64]; static int16_t rawang[48]; unsigned nraw = 0U;
    b5_profile_t prof;
    b5_sweep_report_t rl, rr;
    b5_params_t par = params();
    absolute_time_t t0 = get_absolute_time(), t1;
    for (unsigned i = 0; i < 3; i++) g_n_cls[i] = 0;
    for (unsigned i = 0; i < 5; i++) g_n_why[i] = 0;
    g_nr_w_min = g_nr_w_max = 0U;
    b5_scan_clear(&scan);
    printf("[bench] scan %u: v0.7.4 rule B, sweep %d..%d dd; settle %lu ms, servo centre %d us, stop %d mm, fall wait %lu ms%s\n",
           ++g_scan_no, B5_SCAN_START_DD, B5_SCAN_END_DD, (unsigned long)g_settle_ms, g_centre_us, g_stop_mm, (unsigned long)g_fall_wait_ms,
           g_inject == 1 ? ", INJECTED stuck-high" : (g_inject == 2 ? ", INJECTED no-rise" : ""));
    for (int a = B5_SCAN_START_DD; a <= B5_SCAN_END_DD; a += B5_SCAN_STEP_DD) {
        b5_sample_t s; sample_at(a, B5_SAMPLES_PER_ANGLE, &s);
        (void)b5_scan_add_sample(&scan, &s);
        if (nraw < 48U) { rawang[nraw] = (int16_t)a; snprintf(rawline[nraw], sizeof rawline[0], "%s", g_raw); nraw++; }
    }
    if (b5_scan_has_near(&scan, B5_NEAR_MM)) {
        for (unsigned n = 0U; n < B5_MAX_REFINE_SAMPLES; n++) {
            int16_t a;
            if (!b5_next_refine_angle(&scan, B5_NEAR_MM, B5_REFINE_STOP_DD, &a)) break;
            b5_sample_t s; sample_at(a, B5_REFINE_PINGS, &s);
            (void)b5_scan_add_sample(&scan, &s);
            if (nraw < 48U) { rawang[nraw] = a; snprintf(rawline[nraw], sizeof rawline[0], "%s", g_raw); nraw++; }
        }
    }
    servo_move(B5_AHEAD_DD);
    t1 = get_absolute_time();
    b5_build_navigation_profile_ex(&scan, &par, &prof, &rl, &rr);
    int64_t plan_us = absolute_time_diff_us(t1, get_absolute_time());

    printf("[b5 profile] observed span %d mm; model allowance %d mm; planning envelope %d mm; evidence 0x%02x; clearance L/R %d/%d mm\n",
           (int)prof.width_mm, (int)prof.span_uncertainty_mm, (int)prof.planning_width_mm,
           (unsigned)prof.evidence_flags, (int)prof.clear_left_mm, (int)prof.clear_right_mm);
    printf("[b5 scan] %d samples (angle_dd:range_mm; suffix u = unstable echo, p = echo+1 fault, b = echo+faults (block only), w = weak no-return clear, x = unknown)\n", (int)scan.count);
    for (unsigned i = 0U; i < scan.count; i++) {
        char c = sfx(&scan.sample[i]);
        printf("%s%d:%d%s", (i % 8U) == 0U ? "[b5 scan] " : " ", (int)scan.sample[i].angle_dd,
               (int)scan.sample[i].range_mm, c == ' ' ? "" : (char[]){ c, 0 });
        if ((i % 8U) == 7U || i + 1U == scan.count) printf("\n");
    }
    printf("[b5 decision] side %s; found %d; confidence %d%%; closest %d mm at %d dd; edges L/R %d/%d dd; "
           "at arc limit L/R %d/%d; need %d mm; lateral %d mm; forward %d mm\n",
           prof.side == B5_SIDE_RIGHT ? "RIGHT" : (prof.side == B5_SIDE_LEFT ? "LEFT" : "NONE (stay stopped)"),
           (int)prof.b_found, (int)prof.confidence_pct, (int)prof.closest_mm, (int)prof.closest_dd,
           (int)prof.left_edge_dd, (int)prof.right_edge_dd, (int)prof.b_left_at_limit, (int)prof.b_right_at_limit,
           (int)(par.car_width_mm + 2 * par.margin_mm), (int)prof.lateral_mm, (int)prof.forward_mm);
    printf("[b5 evidence] measured %d%%; coverage %d%%; stable-return span %d mm (provisional, not planning width); weak clearance %s\n",
           (int)prof.confidence_pct, (int)prof.coverage_pct, (int)prof.stable_span_mm,
           (prof.evidence_flags & B5_PROFILE_EVIDENCE_WEAK_CLEAR) ? "YES (tentative, no-return rays)" : "no");
    print_sweep("L", &rl);
    print_sweep("R", &rr);
    printf("[b5 sweep] forward watch during the bypass sees ahead only; sides and rear during turns are not monitored\n");
    printf("[b5 sweep] geometry: pivot-to-front %d mm ASSUMED, sensor %d mm behind front, margin %d mm, turn %d dd; planning took %lld ms\n",
           B5_PIVOT_TO_FRONT_MM, B5_SENSOR_BEHIND_FRONT_MM, B5_MARGIN_MM, B5_SWEEP_TURN_DD, (long long)(plan_us / 1000));
    if (prof.b_found == 0U) printf("[b5]  scan: no obstacle found\n");
    else if (prof.coverage_pct < B5_MIN_CONFIDENCE_PCT) printf("[b5]  low coverage: would stay stopped (Blocked)\n");
    else printf("[b5]  obstacle %d mm wide at %d mm, go %s\n", (int)prof.width_mm, (int)prof.closest_mm,
                prof.side == B5_SIDE_RIGHT ? "right" : (prof.side == B5_SIDE_LEFT ? "left" : "nowhere (would stay stopped / reverse)"));
    printf("[b5 pings] echo %u, no-return %u, fault %u (stuck-high %u, no-rise %u, no-fall %u, short %u); no-return widths %lu..%lu us\n",
           g_n_cls[0], g_n_cls[1], g_n_cls[2], g_n_why[1], g_n_why[2], g_n_why[3], g_n_why[4],
           (unsigned long)g_nr_w_min, (unsigned long)g_nr_w_max);
    printf("[b5 raw] per ping: class value/rise-latency-us (E = echo mm, NR = no-return width us, F* = fault width us)\n");
    for (unsigned i = 0U; i < nraw; i++) {
        printf("%s%d:%s", (i % 4U) == 0U ? "[b5 raw] " : "  ", (int)rawang[i], rawline[i]);
        if ((i % 4U) == 3U || i + 1U == nraw) printf("\n");
    }
    printf("[bench] scan took %lld ms\n", (long long)(absolute_time_diff_us(t0, get_absolute_time()) / 1000));
    if (g_inject) { g_inject = 0; printf("[bench] injection cleared\n"); }
}

/* Guard -> Confirm (3 consecutive ECHO pings < stop) -> scan, like the firmware. */
static void auto_mode(void)
{
    unsigned below = 0U, clear = 0U; bool armed = true;
    servo_move(B5_AHEAD_DD);
    printf("[b5]  Guard (auto: stop below %d mm; any key = leave)\n", g_stop_mm);
    while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
        ping_diag_t d; ping(&d);
        int mm = (int)d.p.range_mm;
        if (armed) {
            below = (d.p.cls == B5_PING_ECHO && mm < g_stop_mm) ? below + 1U : 0U;
            if (d.p.cls == B5_PING_FAULT) printf("[b5]  ping fault (%s)\n", cls_tag(&d.p));
            if (below == 1U) printf("[b5]  Confirm (range %d mm)\n", mm);
            if (below >= B5_CONFIRM_COUNT) {
                printf("[b5]  Halting (range %d mm)\n", mm);
                scan_and_plan();
                printf("[bench] remove the obstacle; re-arms after 1 s clear\n");
                armed = false; below = 0U; clear = 0U;
            }
        } else {
            clear = (d.p.cls != B5_PING_FAULT && mm >= g_stop_mm + B5_SLOW_HYST_MM) ? clear + 1U : 0U;
            if (clear * B5_PING_PERIOD_MS >= 1000U) { armed = true; printf("[b5]  Guard (re-armed)\n"); }
        }
    }
    printf("[bench] auto mode off\n");
}

static void help(void)
{
    printf("\n=== Buddy 5 profile bench v0.7.4 (rule B, measured geometry, symmetric sweep; sweep %d..+%d dd) (wheels never driven) ===\n"
           "  a  auto: guard/confirm below %d mm, then scan + plan (any key leaves)\n"
           "  s  one scan + plan now          p  one ping straight ahead (class, width, rise latency)\n"
           "  c  centre servo                 , .  servo centre trim -/+5 us (now %d us)\n"
           "  m  settle time 40 <-> 80 ms (now %lu ms)\n"
           "  w  fall wait %d <-> 250 ms (now %lu ms; 250 = diagnostic only, to measure the no-target pulse)\n"
           "  i  inject for the NEXT scan only: stuck-high -> no-rise -> off (now %s)\n"
           "  [ / ]  hold at the LEFT / RIGHT end of the sweep (check frame clearance)\n"
           "  1 / 5 / 3  auto stop distance 110 / 150 / %d mm (now %d mm; bench only)\n"
           "  h  this help\n"
           "Angles: - = left, + = right. Gap test = car %d + 2 x %d mm; footprint check uses pivot-to-front %d mm (ASSUMED).\n",
           B5_SCAN_START_DD, B5_SCAN_END_DD, g_stop_mm, g_centre_us, (unsigned long)g_settle_ms, B5_NORETURN_WAIT_MS, (unsigned long)g_fall_wait_ms,
           g_inject == 1 ? "stuck-high" : (g_inject == 2 ? "no-rise" : "off"), B5_D_STOP_MM, g_stop_mm,
           B5_CAR_WIDTH_MM, B5_MARGIN_MM, B5_PIVOT_TO_FRONT_MM);
}

int main(void)
{
    stdio_init_all();
    servo_init();
    gpio_init(PIN_TRIG); gpio_set_dir(PIN_TRIG, GPIO_OUT); gpio_put(PIN_TRIG, 0);
    gpio_init(PIN_ECHO); gpio_set_dir(PIN_ECHO, GPIO_IN); gpio_pull_down(PIN_ECHO);
    sleep_ms(500U);
    bool was = false;
    for (;;) {
        bool conn = stdio_usb_connected();
        if (conn && !was) { sleep_ms(300U); help(); }
        was = conn;
        int c = getchar_timeout_us(100000U);
        switch (c) {
        case 'a': auto_mode(); break;
        case 's': scan_and_plan(); break;
        case 'p': { ping_diag_t d; servo_move(B5_AHEAD_DD); ping(&d);
                    printf("ping: %s %s; width %lu us; range %u mm; pre-trigger low wait %lu us; rise latency %lu us; short pulses %u\n",
                           d.p.cls == B5_PING_ECHO ? "ECHO" : (d.p.cls == B5_PING_NO_RETURN ? "NO_RETURN" : "FAULT"),
                           cls_tag(&d.p), (unsigned long)d.p.width_us, (unsigned)d.p.range_mm,
                           (unsigned long)d.pre_us, (unsigned long)d.rise_lat_us, (unsigned)d.n_short);
                    if (g_inject) { g_inject = 0; printf("[bench] injection cleared\n"); } break; }
        case 'c': servo_move(B5_AHEAD_DD); printf("servo centred (%d us)\n", g_centre_us); break;
        case ',': case '.': g_centre_us += (c == ',') ? 5 : -5; servo_write_angle(g_angle_dd);
                  printf("servo centre %d us (lower = sensor turns right)\n", g_centre_us); break;
        case '[': servo_move(B5_SCAN_START_DD); printf("holding at %d.%d deg (LEFT end), pulse %d us\n", B5_SCAN_START_DD / 10, -(B5_SCAN_START_DD % 10), g_pulse); break;
        case ']': servo_move(B5_SCAN_END_DD); printf("holding at +%d.%d deg (RIGHT end), pulse %d us\n", B5_SCAN_END_DD / 10, B5_SCAN_END_DD % 10, g_pulse); break;
        case '1': g_stop_mm = 110; printf("auto stop distance %d mm\n", g_stop_mm); break;
        case '5': g_stop_mm = 150; printf("auto stop distance %d mm\n", g_stop_mm); break;
        case '3': g_stop_mm = B5_D_STOP_MM; printf("auto stop distance %d mm\n", g_stop_mm); break;
        case 'm': g_settle_ms = (g_settle_ms == 40U) ? 80U : 40U; printf("settle %lu ms\n", (unsigned long)g_settle_ms); break;
        case 'w': g_fall_wait_ms = (g_fall_wait_ms == (uint32_t)B5_NORETURN_WAIT_MS) ? 250U : (uint32_t)B5_NORETURN_WAIT_MS;
                  printf("fall wait %lu ms%s\n", (unsigned long)g_fall_wait_ms, g_fall_wait_ms == 250U ? " (diagnostic)" : " (production)"); break;
        case 'i': g_inject = (g_inject + 1) % 3;
                  printf("injection for the next scan/ping: %s\n", g_inject == 1 ? "stuck-high" : (g_inject == 2 ? "no-rise" : "off")); break;
        case 'h': case '?': help(); break;
        case 0x1B: while (getchar_timeout_us(20000U) != PICO_ERROR_TIMEOUT) { } break;
        default: break;
        }
    }
}
