#define APP_RTOS_BARCODE 1
#define APP_RTOS_IMU 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
#include "app_program/imu_navigation/ramp_model.h"
#include "src/motion/motion_port.h"

static uint64_t now = 1000000;
static uint64_t clock_now(void *u) {
    (void)u;
    return now;
}
int main(void) {
    RampAssist state;
    ra_reset(&state);
    float speed[2] = {45, 45};
    b4v23_tilt_t tilt = {.valid = true, .tracking = true, .height_mm = 8};
    MotionPort port = {.now_us = clock_now};
    motion_init(&port);
    motion_session_begin(now);
    MotionRampAssist requested = {0}, approved = {0};
    ContinuousDrive drive = {.active = true};
    for (unsigned k = 0; k < 20; ++k) {
        now += 20000;
        tilt.seq++;
        tilt.t_us = now;
        assert(motion_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
        MotionFeedback f = {.now_us = now, .speed_mm_s = {45, 45}, .line_level = {700, 700}};
        (void)motion_update(&f);
        requested = ri_ramp_request(&state, &tilt, now, .02f, true, true, true, speed);
        motion_assist_context(true);
        assert(nav_request_ramp_assist(MOTION_BUDDY4, &requested) == MOTION_OK);
        approved = nav_get_ramp_assist();
    }
    assert(state.assisting && state.activations == 1 && approved.boost_milli == 200);
    assert(approved.hold_milli == 1000 && motion_get_status().owner == MOTION_BUDDY3);
    motion_apply_ramp_profile(&drive, &approved);
    assert(drive.boost_frac == .2f && drive.boost_cap == 40);
    assert(drive.hold_min_speed == 35 && drive.hold_i_max == 300 && drive.hold_ki == 3);
    drive.boost_pwm = 40;
    assert(ri_unboosted(&drive, 200) == 160 && ri_unboosted(&drive, 20) == 0);
    now += 50001; /* The coordinator expires a request independently of the IMU. */
    approved = nav_get_ramp_assist();
    assert(!approved.boost_milli && !approved.hold_milli);
    /* Preserve detector6's 200ms hold-stale budget plus its bounded ramp-out. */
    for (unsigned k = 0; k < 25; ++k) {
        now += 20000; /* stopped IMU sequence/timestamp */
        requested = ri_ramp_request(&state, &tilt, now, .02f, true, true, true, speed);
    }
    assert(!requested.boost_milli && !requested.hold_milli && !requested.push_pwm_x100);
    for (unsigned kind = 0; kind < 3; ++kind) {
        ra_reset(&state);
        for (unsigned k = 0; k < 20; ++k) {
            now += 20000;
            tilt.seq++;
            tilt.t_us = now;
            requested = ri_ramp_request(&state, &tilt, now, .02f, true, true, true, speed);
        }
        assert(state.assisting);
        requested =
            ri_ramp_request(&state, &tilt, now, .01f, kind != 0, kind != 1, kind != 2, speed);
        assert(!requested.boost_milli && !requested.push_pwm_x100 && !requested.hold_milli);
        motion_assist_context(false);
        assert(!nav_get_ramp_assist().boost_milli);
    }
    motion_session_end(0);
    puts(
        "PASS RTOS ramp model: detector6 -> Buddy4 NAV approval; Buddy3 owner retained; bounded profile; TTL/stale; STOP/Pause/junction/search removal; unboosted handover");
    return 0;
}
