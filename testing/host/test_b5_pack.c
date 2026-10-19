/**
 * @file    test_b5_pack.c
 * @brief   Host tests for the minimum-bit representation (Tribunal 1):
 *          field sizes, clamping, sign extension, 16-bit wrap and the
 *          binary telemetry frame layout.
 *
 * Build and run:  make -C testing/host
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "b5_pack.h"
#include "obstacle_profile.h"
#include "b5_iface.h"

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; }                                             \
        else { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__);      \
               printf(__VA_ARGS__); printf("\n"); }                         \
    } while (0)

static void
test_sizes (void)
{
    printf("record sizes\n");
    CHECK(sizeof(b5_sample_t) == 4U, "sample %zu bytes", sizeof(b5_sample_t));
    CHECK(sizeof(b5_profile_t) <= 28U, "profile %zu bytes",
          sizeof(b5_profile_t));
    CHECK(sizeof(b5_plan_t) <= 8U, "plan %zu bytes", sizeof(b5_plan_t));
    CHECK(sizeof(b5_rejoin_t) <= 8U, "rejoin %zu bytes", sizeof(b5_rejoin_t));
    CHECK(sizeof(b5_motion_est_t) == 4U, "motion estimate %zu bytes",
          sizeof(b5_motion_est_t));
    printf("  sample %zu, profile %zu, plan %zu, rejoin %zu, status %zu bytes\n",
           sizeof(b5_sample_t), sizeof(b5_profile_t), sizeof(b5_plan_t),
           sizeof(b5_rejoin_t), sizeof(b5_status_t));
}

static void
test_fields (void)
{
    uint32_t w = 0U;

    printf("masks, shifts and sign extension\n");
    w = b5_put_s(w, 17U, B5_BITS_ANGLE, -600);
    w = b5_put_u(w, 4U, B5_BITS_STATE, 9U);
    CHECK(b5_get_s(w, 17U, B5_BITS_ANGLE) == -600, "angle -600 -> %ld",
          (long)b5_get_s(w, 17U, B5_BITS_ANGLE));
    CHECK(b5_get_u(w, 4U, B5_BITS_STATE) == 9U, "state");
    w = b5_put_s(w, 17U, B5_BITS_ANGLE, 900);
    CHECK(b5_get_s(w, 17U, B5_BITS_ANGLE) == 900, "angle +900");
    CHECK(b5_get_u(w, 4U, B5_BITS_STATE) == 9U, "neighbour untouched");
    CHECK(b5_clamp(4396, 0, B5_MM_MAX) == B5_MM_MAX, "clamp, not wrap");
    CHECK((4396U & b5_mask(12U)) == 300U, "(what a wrap would have given)");
}

static void
test_sample_clamp (void)
{
    b5_scan_t scan;

    printf("scan samples clamp instead of wrapping\n");
    b5_scan_clear(&scan);
    (void)b5_scan_add(&scan, -900, 4396, true);
    (void)b5_scan_add(&scan, 1200, 250, true);   /* beyond +900 */
    (void)b5_scan_add(&scan, 0, -5, false);
    CHECK(scan.count == 3U, "count %u", scan.count);
    CHECK(scan.sample[0].angle_dd == -900, "left limit");
    CHECK(scan.sample[0].range_mm == B5_MM_MAX, "4396 -> %u, not 300",
          (unsigned)scan.sample[0].range_mm);
    CHECK(scan.sample[1].range_mm == 0U, "negative -> 0");
    CHECK(scan.sample[2].angle_dd == 900, "1200 -> +900");
}

static void
test_wrap (void)
{
    printf("16-bit net distance wraps safely (agreed with Buddy 2)\n");
    CHECK(b5_wrap_diff16((int16_t)-32700, 32700) == 136, "forward across wrap");
    CHECK(b5_wrap_diff16(32700, (int16_t)-32700) == -136, "reverse across wrap");
    CHECK(b5_wrap_diff16(100, 250) == -150, "reverse 150 mm");
    CHECK(b5_wrap_diff16(2000, 0) == 2000, "one bypass");
}

static void
test_echo_word (void)
{
    uint32_t w = b5_echo_pack(17493U, 201U);

    printf("echo result word (single writer: the ISR)\n");
    CHECK(b5_echo_width_us(w) == 17493U, "width");
    CHECK(b5_echo_seq(w) == 201U, "sequence");
    CHECK((w & B5_ECHO_FLAG_VALID) != 0U, "valid");
    CHECK((w & B5_ECHO_FLAG_SAT) == 0U, "not saturated");
    w = b5_echo_pack(200000U, 7U);
    CHECK(b5_echo_width_us(w) == B5_ECHO_US_MAX, "saturates at 65535");
    CHECK((w & B5_ECHO_FLAG_SAT) != 0U, "saturated flag");
}

static void
test_frame (void)
{
    b5_tlm_t in  = { 0 };
    b5_tlm_t out;
    uint8_t  f[B5_TLM_FRAME_BYTES];

    printf("binary telemetry frame (16 bytes, little-endian)\n");
    in.state = 7U; in.fault = 6U; in.side = 2U; in.blocked_tries = 2U;
    in.recover_tries = 3U; in.servo_dd = -900; in.b_found = true;
    in.range_mm = 4000U; in.closest_mm = 287U; in.confidence_pct = 100U;
    in.width_mm = 104U; in.lateral_mm = 230U; in.pings = 65535U;
    in.no_echo = 12U;
    b5_tlm_pack(&in, f);
    CHECK(b5_tlm_unpack(f, &out), "version accepted");
    CHECK((out.state == 7U) && (out.fault == 6U) && (out.side == 2U),
          "state/fault/side");
    CHECK((out.blocked_tries == 2U) && (out.recover_tries == 3U), "counters");
    CHECK(out.servo_dd == -900, "negative angle %d", out.servo_dd);
    CHECK(out.b_found && (out.range_mm == 4000U) && (out.closest_mm == 287U),
          "ranges");
    CHECK((out.confidence_pct == 100U) && (out.width_mm == 104U) &&
          (out.lateral_mm == 230U), "profile");
    CHECK((out.pings == 65535U) && (out.no_echo == 12U), "ping counters");

    /* Byte 0: version 1 in bits 0..3, state 7 in bits 4..7 -> 0x71. */
    CHECK(f[0] == 0x71U, "byte 0 = 0x%02X", f[0]);

    in.range_mm = 9999U; in.recover_tries = 9U; in.servo_dd = 1500;
    b5_tlm_pack(&in, f);
    (void)b5_tlm_unpack(f, &out);
    CHECK((out.range_mm == B5_MM_MAX) && (out.recover_tries == 3U) &&
          (out.servo_dd == 900), "out-of-range values clamp");

    f[0] = (uint8_t)((f[0] & 0xF0U) | 0x02U);    /* unknown version */
    CHECK(!b5_tlm_unpack(f, &out), "unknown version rejected");
}

int
main (void)
{
    test_sizes();
    test_fields();
    test_sample_clamp();
    test_wrap();
    test_echo_word();
    test_frame();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
