#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include/interfaces/nav_api.h"
#include "src/motion/motion_port.h"
#include "src/navigation/nav_codec.h"
static uint64_t clock_us;
static unsigned depth, off_count, commits;
static MotionFeedback feedback;
static MotionOutput output;
static uintptr_t enter(void *u) {
    (void)u;
    assert(!depth);
    ++depth;
    return 0;
}
static void leave(void *u, uintptr_t n) {
    (void)u;
    (void)n;
    assert(depth == 1);
    --depth;
}
static uint64_t now(void *u) {
    (void)u;
    return clock_us;
}
static void off(void *u) {
    (void)u;
    assert(depth == 1);
    ++off_count;
}
static void apply(void *u) {
    (void)u;
    assert(depth == 1);
    ++commits;
}
static void begin(void) {
    clock_us = 1000;
    depth = off_count = commits = 0;
    MotionPort port = {enter, leave, off, now, NULL};
    motion_init(&port);
    motion_session_begin(clock_us);
    feedback = (MotionFeedback){.now_us = clock_us, .line_level = {700, 700}, .pending_route = 'A'};
    output = motion_update(&feedback);
}
static void tick(bool model) {
    clock_us += 10000;
    feedback.now_us = clock_us;
    if (model)
        for (unsigned w = 0; w < 2; ++w) {
            feedback.speed_mm_s[w] = output.target_mm_s[w];
            feedback.wheel_mm[w] += feedback.speed_mm_s[w] * .01;
        }
    if (nav_get_status().mode == MOTION_AUTO)
        assert(nav_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
    output = motion_update(&feedback);
}
static uint32_t pause_owner(MotionOwner owner) {
    uint32_t lease = 0;
    assert(nav_request_takeover(owner, &lease) == MOTION_OK && lease);
    for (unsigned k = 0; k < 15; ++k)
        tick(true);
    assert(nav_get_status().mode == MOTION_PAUSED && nav_get_status().stopped);
    return lease;
}
static void run_command(uint32_t id) {
    for (unsigned k = 0; k < 7000; ++k) {
        tick(true);
        MotionCommandStatus s;
        assert(nav_get_command(id, &s));
        if (s.state != MOTION_CMD_RUNNING) {
            assert(s.state == MOTION_CMD_DONE);
            return;
        }
    }
    assert(false);
}
static void workflow(void) {
    begin();
    for (int l = -180; l <= 180; ++l) {
        clock_us += 10000;
        feedback.now_us = clock_us;
        assert(nav_submit_navigation(MOTION_BUDDY3, (int16_t)l, (int16_t)-l) == MOTION_OK);
        output = motion_update(&feedback);
        assert(!output.external && output.target_mm_s[0] == l && output.target_mm_s[1] == -l);
    }
    assert(motion_commit(output.generation, apply, NULL) && commits == 1);
    uint32_t old_generation = output.generation, lease = pause_owner(MOTION_BUDDY5), id;
    assert(!motion_commit(old_generation, apply, NULL) && commits == 1 && off_count);
    assert(nav_submit_navigation(MOTION_BUDDY3, 60, 60) == MOTION_BUSY);
    uint32_t other;
    assert(nav_request_takeover(MOTION_BUDDY4, &other) == MOTION_BUSY);
    assert(nav_drive_distance(MOTION_BUDDY4, lease, -30, 40, &id) == MOTION_NOT_OWNER);
    assert(nav_drive_distance(MOTION_BUDDY5, lease, -30, 40, &id) == MOTION_OK);
    uint32_t reverse = id;
    run_command(id);
    MotionCommandStatus result;
    assert(nav_get_command(id, &result) && result.progress_mm <= -29 && result.progress_mm >= -31);
    assert(nav_get_status().mode == MOTION_PAUSED && nav_get_status().pending_route == 'A');
    assert(nav_turn_relative(MOTION_BUDDY5, lease, 45, 45, MOTION_ROLLING, &id) == MOTION_OK);
    run_command(id);
    assert(nav_get_command(id, &result) && result.rotation_deg >= 44);
    assert(nav_turn_relative(MOTION_BUDDY5, lease, -30, 40, MOTION_IN_PLACE, &id) == MOTION_OK);
    run_command(id);
    assert(nav_get_command(id, &result) && result.rotation_deg <= -29);
    assert(nav_get_command(reverse, &result) && result.state == MOTION_CMD_DONE);
    feedback.line_level[0] = feedback.line_level[1] = 0;
    tick(true);
    assert(nav_resume_line(MOTION_BUDDY5, lease, false) == MOTION_NEED_LINE);
    feedback.line_level[0] = 500;
    for (unsigned k = 0; k < 17; ++k)
        tick(true);
    assert(nav_resume_line(MOTION_BUDDY5, lease, false) == MOTION_OK);
    tick(false);
    assert(output.resume_navigation && !output.external && nav_get_status().mode == MOTION_AUTO);
    assert(nav_drive_distance(MOTION_BUDDY5, lease, 30, 40, &id) == MOTION_NOT_OWNER);
    assert(nav_submit_barcode(MOTION_BUDDY3, 'F') == MOTION_OK);
    tick(false);
    assert(output.route == 'F');
    assert(nav_submit_barcode(MOTION_BUDDY3, 'Q') == MOTION_INVALID);
    assert(nav_report_terrain(MOTION_BUDDY4, MOTION_ASCENDING) == MOTION_OK);
    assert(nav_get_status().terrain == MOTION_ASCENDING);
    assert(nav_request_speed_assist(MOTION_BUDDY4, 120) == MOTION_INVALID);
    assert(nav_report_fault(MOTION_BUDDY4, NAV_FAULT_IMU, true) == MOTION_OK);
    assert(nav_report_fault(MOTION_BUDDY5, NAV_FAULT_NO_ECHO, true) == MOTION_OK);
    assert(nav_get_status().fault == (NAV_FAULT_IMU | NAV_FAULT_NO_ECHO));
    assert(nav_report_fault(MOTION_BUDDY4, NAV_FAULT_IMU, false) == MOTION_OK);
    assert(nav_get_status().fault == NAV_FAULT_NO_ECHO);
    motion_session_end(6);
    assert(nav_request_takeover(MOTION_BUDDY5, &lease) == MOTION_INACTIVE);
    assert(!motion_commit(output.generation, apply, NULL));
}
static void interruption_and_failures(void) {
    begin();
    feedback.navigation_busy = true;
    tick(false);
    uint32_t lease = pause_owner(MOTION_BUDDY5);
    assert(nav_resume_line(MOTION_BUDDY5, lease, false) == MOTION_NEED_RECOVERY);
    assert(nav_resume_line(MOTION_BUDDY5, lease, true) == MOTION_OK);
    tick(false);
    assert(output.abandon_navigation);
    begin();
    lease = pause_owner(MOTION_BUDDY4);
    uint32_t newer;
    assert(nav_request_takeover(MOTION_BUDDY5, &newer) == MOTION_OK && newer != lease);
    assert(nav_resume_line(MOTION_BUDDY4, lease, false) == MOTION_NOT_OWNER);
    begin();
    lease = pause_owner(MOTION_BUDDY5);
    uint32_t id;
    assert(nav_drive_distance(MOTION_BUDDY5, lease, 20, 40, &id) == MOTION_OK);
    clock_us += 5000000;
    feedback.now_us = clock_us;
    output = motion_update(&feedback);
    assert(nav_get_status().mode == MOTION_FAULT && (nav_get_status().fault & NAV_FAULT_TIMEOUT));
    MotionCommandStatus s;
    assert(nav_get_command(id, &s) && s.state == MOTION_CMD_FAILED);
    begin();
    lease = pause_owner(MOTION_BUDDY5);
    assert(nav_drive_distance(MOTION_BUDDY5, lease, 20, 300, &id) == MOTION_INVALID);
    assert(nav_turn_relative(MOTION_BUDDY5, lease, NAN, 40, MOTION_ROLLING, &id) == MOTION_INVALID);
    assert(nav_turn_relative(MOTION_BUDDY5, lease, 45, 40, (MotionTurnStyle)2, &id) ==
           MOTION_INVALID);
    assert(nav_drive_distance(MOTION_BUDDY5, lease, 20, 40, &id) == MOTION_OK);
    tick(true);
    assert(nav_request_stop(MOTION_BUDDY4) == MOTION_OK);
    assert(nav_get_command(id, &s) && s.state == MOTION_CMD_CANCELLED);
    assert(nav_get_status().owner == MOTION_BUDDY5);
    begin();
    clock_us += 101000;
    feedback.now_us = clock_us;
    output = motion_update(&feedback);
    assert(nav_get_status().mode == MOTION_FAULT && (nav_get_status().fault & NAV_FAULT_CONTROL));
}
static void codec(void) {
    uint8_t packet[NAV_PACKET_BYTES];
    NavRequest r = {.kind = NAV_DRIVE, .owner = 5, .lease = UINT32_MAX, .speed = 60}, decoded;
    for (int tenth = -10000; tenth <= 10000; ++tenth)
        if (abs(tenth) >= 10) {
            r.value = tenth / 10.0f;
            unsigned n = nav_packet_encode(packet, &r);
            assert(n == 8);
            assert(nav_packet_decode(&decoded, packet, n) && decoded.lease == UINT32_MAX &&
                   fabsf(decoded.value - r.value) < .001f);
        }
    r = (NavRequest){.kind = NAV_TURN, .owner = 5, .value = -180, .speed = 90, .style = 1};
    assert(nav_packet_encode(packet, &r) == 8 && nav_packet_decode(&decoded, packet, 8));
    packet[7] |= 0x80;
    assert(!nav_packet_decode(&decoded, packet, 8));
    r = (NavRequest){.kind = NAV_WHEELS, .owner = 3, .wheels = {-180, 180}};
    assert(nav_packet_encode(packet, &r) == 4 && nav_packet_decode(&decoded, packet, 4));
    assert(decoded.wheels[0] == -180 && decoded.wheels[1] == 180);
    MotionEvent e = {.now_us = UINT64_MAX,
                     .id = UINT32_MAX,
                     .sequence = UINT32_MAX,
                     .owner = 5,
                     .requester = 4,
                     .type = 10,
                     .kind = 3,
                     .state = MOTION_CMD_FAILED,
                     .fault = NAV_FAULT_MASK,
                     .result = MOTION_INVALID,
                     .style = 1,
                     .speed = 90,
                     .target_mm_s = {-180, 180},
                     .goal = -180,
                     .progress_mm = -123.4f,
                     .rotation_deg = 179.9f};
    uint8_t data[NAV_EVENT_BYTES];
    nav_event_encode(data, &e);
    MotionEvent d;
    bool overflow;
    assert(nav_event_decode(&d, data, &overflow) && !overflow && d.fault == NAV_FAULT_MASK &&
           d.requester == 4 && d.owner == 5 && d.id == UINT32_MAX && d.target_mm_s[0] == -180 &&
           d.target_mm_s[1] == 180);
    assert(fabsf(d.rotation_deg - 179.9f) < .01 && fabsf(d.progress_mm + 123.4f) < .01);
    e.progress_mm = 1e20f;
    nav_event_encode(data, &e);
    assert(nav_event_decode(&d, data, &overflow) && overflow);
    begin();
    for (unsigned k = 0; k < 140; ++k)
        assert(nav_report_terrain(MOTION_BUDDY4, MOTION_ASCENDING) == MOTION_OK);
    assert(motion_log_count() == 120 && motion_log_overwritten() == 20);
    assert(motion_log_read(0, &d) && d.sequence == 21 && d.requester == 4 && d.kind == NAV_TERRAIN);
    printf(
        "representation: request<=%u bytes wheels=4 bytes audit=%u bytes status=%zu event_ram=%zu request_working=%zu command_status=%zu\n",
        NAV_PACKET_BYTES, NAV_EVENT_BYTES, sizeof(MotionStatus), sizeof(MotionEvent),
        sizeof(NavRequest), sizeof(MotionCommandStatus));
}
int main(void) {
    workflow();
    interruption_and_failures();
    codec();
    puts(
        "PASS shared coordinator ownership, stop atomicity, finite reverse/turn completion, route/terrain/fault APIs, interrupted junction resume, stale commands/leases, rejection/timeout, compact binary roundtrips and bounded audit.");
}
