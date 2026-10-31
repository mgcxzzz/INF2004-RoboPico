#include <assert.h>
#include <stdio.h>
#include "include/interfaces/nav_api.h"
#include "src/motion/motion_port.h"
#include "src/navigation/nav_codec.h"
static uint64_t clock_us = 1000000;
static uint64_t clock_port(void *u) {
    (void)u;
    return clock_us;
}
int main(void) {
    MotionPort port = {.now_us = clock_port};
    motion_init(&port);
    motion_session_begin(clock_us);
    assert(nav_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
    MotionRampAssist a = {
        .boost_milli = 200, .push_pwm_x100 = 1234, .hold_milli = 1000, .ttl_ms = 50};
    assert(nav_request_ramp_assist(MOTION_BUDDY4, &a) == MOTION_BUSY);
    motion_assist_context(true);
    assert(nav_request_ramp_assist(MOTION_BUDDY4, &a) == MOTION_OK);
    assert(nav_get_status().owner == MOTION_BUDDY3);
    MotionRampAssist b = nav_get_ramp_assist();
    assert(b.boost_milli == 200 && b.push_pwm_x100 == 1234 && b.hold_milli == 1000);
    assert(nav_request_ramp_assist(MOTION_BUDDY5, &a) == MOTION_INVALID);
    NavRequest req = {.kind = NAV_RAMP_ASSIST, .owner = MOTION_BUDDY4, .assist = a}, decoded;
    uint8_t bytes[NAV_PACKET_BYTES];
    unsigned n = nav_packet_encode(bytes, &req);
    assert(n == 6 && nav_packet_decode(&decoded, bytes, n));
    assert(decoded.assist.push_pwm_x100 == 1234 && decoded.assist.ttl_ms == 50);
    req.assist.hold_milli = 1001;
    assert(!nav_packet_encode(bytes, &req));
    clock_us += 50000;
    b = nav_get_ramp_assist();
    assert(b.boost_milli == 0 && b.hold_milli == 0);
    assert(nav_request_ramp_assist(MOTION_BUDDY4, &a) == MOTION_OK);
    motion_assist_context(false);
    assert(nav_get_ramp_assist().hold_milli == 0);
    motion_assist_context(true);
    assert(nav_request_ramp_assist(MOTION_BUDDY4, &a) == MOTION_OK);
    assert(nav_accept_barcode(MOTION_BUDDY3, 'Z') == MOTION_OK);
    assert(nav_accept_barcode(MOTION_BUDDY4, 'Z') == MOTION_INVALID);
    uint32_t lease;
    assert(nav_request_takeover(MOTION_BUDDY5, &lease) == MOTION_OK);
    assert(nav_get_status().owner == MOTION_BUDDY5 && nav_get_ramp_assist().hold_milli == 0);
    assert(nav_request_ramp_assist(MOTION_BUDDY4, &a) == MOTION_BUSY);
    assert(nav_accept_barcode(MOTION_BUDDY3, 'Z') == MOTION_BUSY);
    a = (MotionRampAssist){.ttl_ms = 50};
    assert(nav_request_ramp_assist(MOTION_BUDDY4, &a) == MOTION_OK);
    MotionEvent e;
    bool saw_assist = false, saw_decode = false;
    for (unsigned i = 0; i < motion_log_count(); ++i) {
        assert(motion_log_read(i, &e));
        if (e.kind == NAV_RAMP_ASSIST && e.requester == MOTION_BUDDY4)
            saw_assist = true;
        if (e.kind == NAV_VERIFIED_BARCODE && e.result == MOTION_OK)
            saw_decode = true;
        uint8_t record[NAV_EVENT_BYTES];
        MotionEvent restored;
        bool overflow;
        nav_event_encode(record, &e);
        assert(nav_event_decode(&restored, record, &overflow));
    }
    assert(saw_assist && saw_decode);
    puts(
        "PASS coordinator assist: wire bounds, owner preserved, expiry, search/stop/takeover clearing, barcode approval and audit");
}
