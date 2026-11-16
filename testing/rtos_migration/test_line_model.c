#include <assert.h>
#include <stdio.h>
#define BUDDY_WIFI_LOGGING 1
#define LINE_CAL_MIN_SPAN 50
#define CIRCUIT_SWEEP_SEARCH 1
#include "app_program/line_navigation/line_model.h"

static void same(analog_demand_t a, analog_demand_t b) {
    assert(a.target[0] == b.target[0] && a.target[1] == b.target[1]);
    assert(a.level[0] == b.level[0] && a.level[1] == b.level[1]);
    assert(a.decision == b.decision && a.stop == b.stop);
}

static void test_planner_parity(void) {
    static line_planner_t p;
    static barcode_analog_t a;
    circuit_input_t i = {.now = 1000, .adc = {3400, 3600}, .speed = {45, 45}};
    rl_begin(&p, &i, 180);
    circuit_t ref = p.circuit;
    barcode_nav_t nav = p.nav;
    continuous_steering_t steering = {0};
    scan_smooth_t scan = {0};
    smooth_follow_t legacy = {0};
    a.baseline = a.adc = 180;
    line_calibration_t cal = successful_calibration();
    /* Straight, sustained bend, alternating return, brief white, recovery,
     * scan-activity entry/exit and long white search. No manufactured ADCs
     * are used by the firmware: these samples are host-only stimuli. */
    for (unsigned n = 0; n < 1000; ++n) {
        i.now += 10000;
        i.path += .45;
        i.wheel[0] += .45;
        i.wheel[1] += .45;
        if (n < 100)
            i.adc[0] = 3400, i.adc[1] = 3600;
        else if (n < 200)
            i.adc[0] = 2800, i.adc[1] = 700;
        else if (n < 300)
            i.adc[0] = 900, i.adc[1] = 3000;
        else if (n < 400)
            i.adc[0] = n % 20 < 10 ? 1200 : 2300, i.adc[1] = 1700;
        else if (n < 405 || n > 650)
            i.adc[0] = 160, i.adc[1] = 158;
        else
            i.adc[0] = 3000, i.adc[1] = 2300;
        a.adc = p.side_adc = n >= 420 && n < 460 ? 250 : 180;
        analog_demand_t actual = rl_follow(&p, &cal, &i, .01f);
        analog_demand_t expected =
            continuous_follow_step(&ref, &nav, &cal, &i, 1, &legacy, &a, &scan, &steering, .01f);
        same(actual, expected);
        assert(p.circuit.phase == ref.phase && p.circuit.reason == ref.reason);
        assert(p.steering.gain == steering.gain && p.steering.half == steering.half);
    }
}

static void cross_sample(line_planner_t *p, unsigned adc, uint64_t t, double mm) {
    const double wheel[2] = {mm, mm};
    rl_side(p, (uint16_t)adc, t, wheel);
    rl_side(p, (uint16_t)adc, t + 2000, wheel);
}

static void test_join_parity(void) {
    const char routes[] = {'A', 'F', 'Z'};
    for (unsigned route = 0; route < 3; ++route) {
        static line_planner_t p;
        static barcode_analog_t a;
        memset(&a, 0, sizeof(a));
        circuit_input_t i = {.now = 1000000, .path = 100, .speed = {45, 45}};
        i.wheel[route == 1 ? 0 : 1] = 172.7876;
        rl_begin(&p, &i, 180);
        p.circuit.phase = J_ACQUIRE;
        p.circuit.busy = p.nav.busy = true;
        p.circuit.turn =
            (junction_t){.phase = J_ACQUIRE, .phase_us = i.now, .rolling = true, .exit_origin = 100};
        p.nav.action = routes[route];
        circuit_t ref = p.circuit;
        barcode_nav_t nav = p.nav;
        continuous_steering_t steering = {0};
        scan_smooth_t scan = {0};
        smooth_follow_t legacy = {0};
        a.baseline = a.adc = 180;
        line_calibration_t cal = successful_calibration();
        for (unsigned n = 0; n < 100; ++n) {
            i.now += 10000;
            i.path += .45;
            i.wheel[0] += .45;
            i.wheel[1] += .45;
            i.adc[0] = n < 5 ? 163 : 3000;
            i.adc[1] = n < 5 ? 160 : 2800;
            same(rl_follow(&p, &cal, &i, .01f),
                 continuous_follow_step(&ref, &nav, &cal, &i, 1, &legacy, &a, &scan, &steering,
                                        .01f));
            assert(p.circuit.phase == ref.phase && p.circuit.reason == ref.reason);
            assert(p.circuit.turns == ref.turns && p.circuit.busy == ref.busy);
        }
        assert(p.circuit.turns == 1 && !p.circuit.busy);
    }
}

static void test_cross(void) {
    line_planner_t p;
    circuit_input_t i = {.now = 1000};
    rl_begin(&p, &i, 180);
    p.nav.allow_cross = true;
    assert(barcode_set_pending(&p.nav, 'A'));
    cross_sample(&p, 3000, 10000, 10);
    cross_sample(&p, 180, 500000, 28);
    assert(p.nav.cross_candidate && p.nav.cross_width == 18);
    barcode_cross_confirm(&p.nav, 600000, 31.9, true);
    assert(!p.nav.cross_ready);
    barcode_cross_confirm(&p.nav, 610000, 32, true);
    assert(p.nav.cross_ready && p.nav.pending == 'A');
    /* A narrow bar never qualifies; a second bar inside white confirmation cancels. */
    rl_begin(&p, &i, 180);
    cross_sample(&p, 3000, 10000, 10);
    cross_sample(&p, 180, 100000, 14);
    assert(!p.nav.cross_candidate);
    cross_sample(&p, 3000, 200000, 20);
    cross_sample(&p, 180, 400000, 38);
    assert(p.nav.cross_candidate);
    cross_sample(&p, 3000, 410000, 40);
    assert(!p.nav.cross_candidate && !p.nav.cross_ready);
}

static void test_calibration(void) {
    static barcode_saved_t sdk;
    uint8_t page[256];
    memset(page, 0xff, sizeof(page));
    memset(&sdk, 0, sizeof(sdk));
    uint32_t origin = 99;
    line_calibration_t cal = rl_boot_cal(page, &sdk, &origin);
    assert(origin == 0 && cal.black[1] == 3700);
    sdk.magic = B_MAGIC;
    sdk.version = 28;
    sdk.profile = sdk.recorded_profile = 1;
    sdk.side = (side_calibration_t){0, 1, 3};
    sdk.calibration = (line_calibration_t){{122, 138}, {246, 267}, 7};
    sdk.crc =
        log_crc((const uint8_t *)&sdk + offsetof(barcode_saved_t, count), barcode_record_crc_size(28));
    assert(barcode_record_valid(&sdk));
    cal = rl_boot_cal(page, &sdk, &origin);
    assert(origin == 1 && cal.white[0] == 122 && cal.black[1] == 267);
    line_calibration_t own = {{180, 190}, {900, 1000}, 7};
    rl_cal_encode(page, &own);
    cal = rl_boot_cal(page, &sdk, &origin);
    assert(origin == 2 && cal.white[0] == own.white[0] && cal.white[1] == own.white[1] &&
           cal.black[0] == own.black[0] && cal.black[1] == own.black[1] && cal.mask == own.mask);
    for (unsigned n = 0; n < 256; ++n) {
        page[n] ^= 1;
        line_calibration_t invalid;
        assert(!rl_cal_decode(page, &invalid));
        page[n] ^= 1;
    }
    page[20] ^= 1;
    cal = rl_boot_cal(page, &sdk, &origin);
    assert(origin == 1);
    sdk.calibration.black[0]++; /* Corrupt record must not become a calibration. */
    cal = rl_boot_cal(page, &sdk, &origin);
    assert(origin == 0 && cal.black[0] == 3457);
}

static void test_envelopes(void) {
    line_envelope_t e = {
        .output = {750, -750}, .until = {10000, 10000}, .maneuver = true, .reverse = true};
    for (unsigned w = 0; w < 2; ++w) {
        dual_levels_t before = rl_levels(&e, w, 9999);
        dual_levels_t after = rl_levels(&e, w, 10000);
        assert(before.a + before.b == 750 && after.a + after.b == 600);
        assert(!before.b && !after.b); /* Left forward and right reverse. */
        e.reverse = false;
        dual_levels_t forward = rl_levels(&e, w, 10000);
        assert(w == 0 ? forward.a == 600 : forward.a + forward.b == 0);
        e.reverse = true;
    }
    e = (line_envelope_t){.output = {1000, 1000}, .continuous = true};
    assert(rl_levels(&e, 0, 0).a == 420 && rl_levels(&e, 1, 0).b == 420);
    /* Exercise selected drive mode, U-turn and active search adapters. */
    line_planner_t p;
    circuit_input_t i = {.now = 100000, .adc = {3400, 3600}, .speed = {45, 45}};
    rl_begin(&p, &i, 180);
    line_power_t power = {0};
    analog_demand_t d = {.target = {45, 45}, .level = {900, 900}, .decision = SYM_STRAIGHT};
    e = rl_power(&power, &p, &i, &d, C_FOLLOW, .01f);
    assert(e.continuous && !e.maneuver);
    p.circuit.phase = J_TURN;
    p.circuit.busy = true;
    p.nav.action = 'Z';
    d.target[0] = -45;
    d.target[1] = 0;
    e = rl_power(&power, &p, &i, &d, J_TURN, .01f);
    /* Original U-turn second stage reverses left while right coasts. */
    assert(e.reverse && e.maneuver && !e.continuous);
    assert(rl_levels(&e, 0, i.now).b > 0 && rl_levels(&e, 1, i.now).a == 0);
    p.circuit.phase = C_FOLLOW;
    p.circuit.busy = false;
    p.circuit.follower.core.white = true;
    p.circuit.follower.search.active = true;
    d.target[0] = 0;
    d.target[1] = 45;
    e = rl_power(&power, &p, &i, &d, C_FOLLOW, .01f);
    assert(e.maneuver && !e.continuous && !e.reverse);
    dual_levels_t stopped = rl_reduce((dual_levels_t){0}, (dual_levels_t){750, 0});
    assert(stopped.a == 0 && stopped.b == 0);
    dual_levels_t limit = rl_reduce((dual_levels_t){750, 0}, (dual_levels_t){600, 0});
    assert(limit.a == 600 && limit.b == 0);
    dual_levels_t reverse = rl_reduce((dual_levels_t){420, 0}, (dual_levels_t){0, 600});
    assert(reverse.a == 0 && reverse.b == 0);
}

static void test_log_pack(void) {
    const char routes[] = {0, 'A', 'F', 'Z'};
    for (unsigned n = 0; n < 4; ++n)
        assert(rl_route_ascii(rl_route_bits(routes[n])) == (unsigned)routes[n]);
    for (unsigned side = 0; side <= 4095; ++side) {
        uint16_t raw[3] = {0, 4095, (uint16_t)side};
        analog_demand_t d = {.level = {0, 1000}};
        line_log_row_t r = {0};
        rl_pack_optics(&r, raw, &d, J_ACQUIRE);
        assert((r.optics[0] & 4095) == 0 && ((r.optics[0] >> 12) & 4095) == 4095);
        assert(((r.optics[0] >> 24) | ((r.optics[1] & 15) << 8)) == side);
        assert(((r.optics[1] >> 4) & 1023) == 0 && ((r.optics[1] >> 14) & 1023) == 1000);
        assert(((r.optics[1] >> 24) & 15) == J_ACQUIRE);
        const uint16_t filtered[2] = {(uint16_t)side, (uint16_t)(4095 - side)};
        rl_pack_steering(&r, filtered, side % 1001, (side * 7) % 1001);
        assert((r.aux[0] & 4095U) == side && ((r.aux[0] >> 12) & 4095U) == 4095 - side);
        assert(((r.aux[0] >> 24) | ((r.aux[1] & 3U) << 8)) == side % 1001);
        assert(((r.aux[1] >> 2) & 1023U) == (side * 7) % 1001);
    }
}

int main(void) {
    test_planner_parity();
    test_join_parity();
    test_cross();
    test_calibration();
    test_envelopes();
    test_log_pack();
    puts(
        "PASS Step2: planner parity, cross qualification, calibration CRC/import, power envelopes, log packing.");
    return 0;
}
