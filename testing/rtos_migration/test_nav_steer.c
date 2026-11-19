/* Host test: MOTION_STEERED_DRIVE coordinator extension (v0.2 candidate).
 * NAV_STEER codec round trip/rejections, ownership/mode gating, bounded
 * setpoint effect on the REAL executor heading hold, and the 100 ms lapse.
 * Plant: wheel speed = target. LOGIC EVIDENCE ONLY. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "include/interfaces/motion_api.h"
#include "include/interfaces/nav_api.h"
#include "src/motion/motion_port.h"
#include "src/navigation/nav_codec.h"

static uint64_t g_now = 1000000;
static double g_wheel[2];
static float g_speed[2];
static unsigned g_fail;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++g_fail; } } while (0)
static uint64_t now_fn(void *p) { (void)p; return g_now; }
static motion_output_t step(void) {
    g_now += 10000;
    motion_feedback_t f = {.now_us = g_now};
    f.wheel_mm[0] = g_wheel[0]; f.wheel_mm[1] = g_wheel[1];
    f.speed_mm_s[0] = g_speed[0]; f.speed_mm_s[1] = g_speed[1];
    f.line_level[0] = 500; f.line_level[1] = 40;
    motion_output_t o = motion_update(&f);
    for (int w = 0; w < 2; ++w) { g_speed[w] = o.target_mm_s[w]; g_wheel[w] += g_speed[w] * 0.01; }
    return o;
}
static double yaw_deg(const double origin[2]) {
    return ((g_wheel[1] - origin[1]) - (g_wheel[0] - origin[0])) / 110.0 * 180.0 / M_PI;
}

int main(void) {
    /* 1. codec */
    uint8_t pkt[NAV_PACKET_BYTES];
    for (int d = -80; d <= 80; ++d) {
        nav_request_t in = {.kind = NAV_STEER, .owner = MOTION_BUDDY5, .lease = 0xA5C3u + (unsigned)d,
                            .value = d / 10.0f}, out;
        unsigned n = nav_packet_encode(pkt, &in);
        CHECK(n > 0 && n <= NAV_PACKET_BYTES);
        CHECK(nav_packet_decode(&out, pkt, n));
        CHECK(out.kind == NAV_STEER && out.owner == MOTION_BUDDY5 && out.lease == in.lease);
        CHECK(lroundf(out.value * 10) == d);
        if (n > 1) CHECK(!nav_packet_decode(&out, pkt, n - 1)); /* exact length */
    }
    nav_request_t big = {.kind = NAV_STEER, .owner = MOTION_BUDDY5, .lease = 1, .value = 8.06f};
    CHECK(nav_packet_encode(pkt, &big) == 0);
    big.value = -8.06f; CHECK(nav_packet_encode(pkt, &big) == 0);
    /* kinds above NAV_KIND_LAST are still refused */
    memset(pkt, 0, sizeof pkt); pkt[0] = 14; { nav_request_t o; CHECK(!nav_packet_decode(&o, pkt, NAV_PACKET_BYTES)); }

    /* 2. coordinator gating */
    motion_port_t port = {.now_us = now_fn};
    motion_init(&port);
    motion_session_begin(g_now);
    for (int i = 0; i < 5; ++i) step();
    CHECK(nav_steer_drive(MOTION_BUDDY5, 0, 1.0f) != MOTION_OK); /* AUTO: no lease */
    uint32_t lease = 0;
    CHECK(nav_request_takeover(MOTION_BUDDY5, &lease) == MOTION_OK);
    for (int i = 0; i < 20; ++i) step();
    CHECK(nav_get_status().mode == MOTION_PAUSED);
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease, 1.0f) == MOTION_BUSY);       /* PAUSED */
    CHECK(nav_steer_drive(MOTION_BUDDY3, lease, 1.0f) == MOTION_NOT_OWNER);  /* other owner */
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease + 1, 1.0f) == MOTION_NOT_OWNER); /* stale lease */
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease, 8.2f) == MOTION_INVALID);
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease, NAN) == MOTION_INVALID);
    uint32_t cmd = 0;
    CHECK(nav_turn_relative(MOTION_BUDDY5, lease, 20, 30, MOTION_IN_PLACE, &cmd) == MOTION_OK);
    for (int i = 0; i < 3; ++i) step();
    CHECK(nav_get_status().mode == MOTION_TURNING);
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease, 1.0f) == MOTION_BUSY);       /* turn */
    for (int i = 0; i < 600 && nav_get_status().mode != MOTION_PAUSED; ++i) step();
    CHECK(nav_get_status().mode == MOTION_PAUSED);

    /* 3. bounded effect on a straight drive, refreshed every 10 ms */
    CHECK(nav_drive_distance(MOTION_BUDDY5, lease, 400, 20, &cmd) == MOTION_OK);
    step();
    CHECK(nav_get_status().mode == MOTION_DRIVING);
    double origin[2] = {g_wheel[0], g_wheel[1]};
    double peak = 0;
    for (int i = 0; i < 400; ++i) { /* 4 s at +6 deg */
        CHECK(nav_steer_drive(MOTION_BUDDY5, lease, 6.0f) == MOTION_OK);
        step();
        peak = fmax(peak, fabs(yaw_deg(origin)));
        CHECK(nav_get_status().mode == MOTION_DRIVING);
    }
    double held = yaw_deg(origin);
    printf("steered yaw %.2f deg (setpoint 6.0, peak %.2f)\n", held, peak);
    CHECK(held > 5.0 && held < 7.0); CHECK(peak < 12.0);
    /* 4. silence: setpoint lapses after 100 ms and the hold returns to straight */
    for (int i = 0; i < 400 && nav_get_status().mode == MOTION_DRIVING; ++i) step();
    double after = yaw_deg(origin);
    printf("after lapse yaw %.2f deg, mode %d\n", after, nav_get_status().mode);
    CHECK(fabs(after) < 1.5);
    /* 5. once the drive ends: BUSY (expected, not a fault), lease kept */
    for (int i = 0; i < 2000 && nav_get_status().mode != MOTION_PAUSED; ++i) step();
    CHECK(nav_get_status().mode == MOTION_PAUSED && nav_get_status().owner == MOTION_BUDDY5);
    motion_command_status_t cs; CHECK(nav_get_command(cmd, &cs) && cs.state == MOTION_CMD_DONE);
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease, 2.0f) == MOTION_BUSY);
    /* 6. a new drive starts straight: the previous setpoint does not carry over */
    CHECK(nav_drive_distance(MOTION_BUDDY5, lease, 60, 20, &cmd) == MOTION_OK);
    double o2[2] = {g_wheel[0], g_wheel[1]};
    for (int i = 0; i < 1000 && nav_get_status().mode != MOTION_PAUSED; ++i) step();
    CHECK(nav_get_status().mode == MOTION_PAUSED);
    CHECK(fabs(yaw_deg(o2)) < 0.5);
    /* 7. Stop during a steered drive: stopped, steering refused */
    CHECK(nav_drive_distance(MOTION_BUDDY5, lease, 300, 20, &cmd) == MOTION_OK);
    for (int i = 0; i < 50; ++i) { (void)nav_steer_drive(MOTION_BUDDY5, lease, -8.0f); step(); }
    CHECK(nav_request_stop(MOTION_BUDDY1) == MOTION_OK);
    for (int i = 0; i < 100; ++i) step();
    CHECK(fabsf(g_speed[0]) < 1e-3f && fabsf(g_speed[1]) < 1e-3f);
    CHECK(nav_steer_drive(MOTION_BUDDY5, lease, -8.0f) != MOTION_OK);
    printf("%s: %u failure(s)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
