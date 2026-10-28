/**
 * @file    test_b1.c
 * @brief   Host tests for Buddy 1 (WiFi and MQTT): command checking
 *          (FR-WIFI-02/03), acknowledgements, the source-1 status frame and
 *          the reconnect back-off. Compiles app_program/subsystem1/
 *          b1_proto.c and system/sys_tlm.c unchanged; no kernel, no Pico.
 *
 *   make -C testing/subsystem1/host            run the tests
 *   ./test_b1 --vectors > vectors.json         frames for the dashboard check
 *
 * Group IS28, Buddy 1.
 */

#include <stdio.h>
#include <string.h>

#include "b1_proto.h"
#include "b1_mapping.h"
#include "sys_tlm.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(id, cond, ...)                                            \
    do {                                                                \
        if (cond) { g_pass++; printf("PASS  %-8s ", id); }              \
        else      { g_fail++; printf("FAIL  %-8s ", id); }              \
        printf(__VA_ARGS__);                                            \
        printf("\n");                                                   \
    } while (0)

static uint8_t
check_code (uint8_t code, uint16_t arg)
{
    uint8_t  msg[B1_CMD_BYTES];
    b1_cmd_t cmd;

    b1_cmd_pack(code, 7U, arg, msg);
    return b1_cmd_check(msg, B1_CMD_BYTES, &cmd);
}

/* H01: every code gets the right verdict. */
static void
test_commands (void)
{
    static uint8_t const allowed[] =
    {
        B1_CMD_START, B1_CMD_PAUSE, B1_CMD_RESUME, B1_CMD_STOP,
        B1_CMD_RESET, B1_CMD_ESTOP, B1_CMD_PING, B1_CMD_BARCODE_GET
    };
    uint8_t  msg[12];
    b1_cmd_t cmd;
    uint32_t i;
    int      ok = 1;

    for (i = 0U; i < sizeof(allowed); i++)
    {
        ok &= (check_code(allowed[i], 0U) == B1_ACK_ACCEPTED);
    }
    CHECK("H01a", ok, "START, PAUSE, RESUME, STOP, RESET, ESTOP, PING accepted");

    CHECK("H01b", (check_code(B1_CMD_STEER, 0U) == B1_ACK_PROHIBITED) &&
                  (check_code(B1_CMD_DRIVE, 500U) == B1_ACK_PROHIBITED) &&
                  (check_code(B1_CMD_TURN, 900U) == B1_ACK_PROHIBITED),
          "STEER, DRIVE, TURN rejected as prohibited (FR-WIFI-03)");

    ok = 1;
    for (i = 0U; i < 16U; i++)
    {
        if ((i == 0U) || (i == 11U) || (i == 15U))
        {
            ok &= (check_code((uint8_t)i, 0U) == B1_ACK_UNKNOWN);
        }
    }
    CHECK("H01c", ok, "codes 0, 11, 15 rejected as unknown");

    CHECK("H01d", (check_code(B1_CMD_CONFIG, (1U << 12) | 0U) == B1_ACK_ACCEPTED) &&
                  (check_code(B1_CMD_CONFIG, (1U << 12) | 1U) == B1_ACK_ACCEPTED) &&
                  (check_code(B1_CMD_CONFIG, (1U << 12) | 2U) == B1_ACK_FORMAT) &&
                  (check_code(B1_CMD_CONFIG, (2U << 12) | 1U) == B1_ACK_UNSUPPORTED),
          "CONFIG: telemetry on/off accepted, bad value / unknown parameter refused");

    b1_cmd_pack(B1_CMD_START, 9U, 0U, msg);
    CHECK("H01e", (b1_cmd_check(msg, 3U, &cmd) == B1_ACK_FORMAT) &&
                  (cmd.id == 9U) && (cmd.code == B1_CMD_START),
          "3 bytes: bad format, id and code still kept for the ack");
    msg[4] = 0U;
    CHECK("H01f", b1_cmd_check(msg, 5U, &cmd) == B1_ACK_FORMAT,
          "5 bytes: bad format");
    CHECK("H01g", b1_cmd_check(msg, 0U, &cmd) == B1_ACK_FORMAT,
          "empty message: bad format, nothing read");

    b1_cmd_pack(B1_CMD_START, 1U, 0U, msg);
    msg[0] = (uint8_t)((msg[0] & 0xF0U) | 1U);          /* version 1 */
    CHECK("H01h", b1_cmd_check(msg, 4U, &cmd) == B1_ACK_FORMAT,
          "wrong schema version: bad format");

    /* A STEER with a version error is still a format error: the version is
     * checked first, so nothing is trusted from a foreign message. */
    b1_cmd_pack(B1_CMD_STEER, 1U, 0U, msg);
    msg[0] = (uint8_t)((msg[0] & 0xF0U) | 3U);
    CHECK("H01i", b1_cmd_check(msg, 4U, &cmd) == B1_ACK_FORMAT,
          "foreign version checked before the code");

    b1_cmd_pack(B1_CMD_CONFIG, 200U, 0xABCDU, msg);
    (void)b1_cmd_check(msg, 4U, &cmd);
    CHECK("H01j", (cmd.arg == 0xABCDU) && (cmd.id == 200U) &&
                  (msg[0] == (uint8_t)(TLM_VERSION | (B1_CMD_CONFIG << 4))),
          "argument little-endian, id and byte 0 as documented");
}

/* H02: acknowledgement layout. */
static void
test_ack (void)
{
    uint8_t  msg[B1_CMD_BYTES];
    uint8_t  ack[B1_ACK_BYTES];
    b1_cmd_t cmd;

    b1_cmd_pack(B1_CMD_STEER, 42U, 0U, msg);
    (void)b1_cmd_check(msg, 4U, &cmd);
    b1_ack_pack(&cmd, B1_ACK_PROHIBITED, 1U, ack);
    CHECK("H02a", (ack[0] == msg[0]) && (ack[1] == 42U) &&
                  (ack[2] == B1_ACK_PROHIBITED) && (ack[3] == 1U),
          "ack echoes byte 0 and id, carries result and state");
    b1_ack_pack(&cmd, B1_ACK_ACCEPTED, 200U, ack);
    CHECK("H02b", ack[3] == 7U, "state saturates to 3 bits (7 = unknown)");
}

/* H03: status frame round trip, extremes and saturation. */
static void
test_status (void)
{
    b1_tlm_t in;
    b1_tlm_t out;
    uint8_t  f[B1_TLM_FRAME_BYTES];

    memset(&in, 0, sizeof(in));
    in.seq = 255U; in.net = 3U; in.state = 6U; in.last_cmd = 15U;
    in.last_result = 7U; in.uptime_s = 16383U; in.published = 65535U;
    in.lost = 65535U; in.accepted = 255U; in.rejected = 255U;
    in.connects = 255U; in.fill_pct = 100U;
    b1_tlm_pack(&in, f);
    memset(&out, 0, sizeof(out));       /* padding compared too */
    CHECK("H03a", b1_tlm_unpack(f, sizeof(f), &out) &&
                  (memcmp(&in, &out, sizeof(in)) == 0),
          "all fields at their maximum survive a round trip");

    memset(&in, 0, sizeof(in));
    in.net = 2U; in.state = 1U; in.uptime_s = 60000U; in.fill_pct = 250U;
    b1_tlm_pack(&in, f);
    (void)b1_tlm_unpack(f, sizeof(f), &out);
    CHECK("H03b", (out.uptime_s == 16383U) && (out.fill_pct == 100U) &&
                  (out.net == 2U) && (out.state == 1U),
          "uptime and fill saturate instead of wrapping");

    CHECK("H03c", (f[0] == (uint8_t)(TLM_VERSION | (TLM_SRC_WIFI << 4))) &&
                  (f[1] == B1_TLM_PAYLOAD_BYTES) && (sizeof(f) == 15U),
          "header: version 2, source 1, payload 12 bytes");
    CHECK("H03d", (f[3 + 3] & 0xFCU) == 0U,
          "reserved bits 26..31 of word 0 are zero");
    CHECK("H03e", !b1_tlm_unpack(f, sizeof(f) - 1U, &out),
          "wrong length is refused");
    f[0] = (uint8_t)(TLM_VERSION | (TLM_SRC_NAV << 4));
    CHECK("H03f", !b1_tlm_unpack(f, sizeof(f), &out),
          "another source's frame is refused");
}

/* H04: back-off and fill. */
static void
test_helpers (void)
{
    CHECK("H04a", (b1_backoff_ms(1U) == 1000U) && (b1_backoff_ms(2U) == 2000U) &&
                  (b1_backoff_ms(3U) == 4000U) && (b1_backoff_ms(4U) == 8000U) &&
                  (b1_backoff_ms(50U) == 8000U) && (b1_backoff_ms(0U) == 1000U),
          "back-off 1, 2, 4, 8 s, then stays at 8 s");
    CHECK("H04b", (b1_fill_pct(0U, 2048U) == 0U) && (b1_fill_pct(1024U, 2048U) == 50U) &&
                  (b1_fill_pct(4096U, 2048U) == 100U) && (b1_fill_pct(5U, 0U) == 0U),
          "queue fill percent");
    CHECK("H04c", (strcmp(b1_cmd_name(B1_CMD_ESTOP), "ESTOP") == 0) &&
                  (strcmp(b1_net_name(9U), "?") == 0),
          "names");
}

/* Frames for the dashboard decoder check (check_dashboard.mjs). */
static void
hex (char const * name, uint8_t const * p, uint32_t n, int last)
{
    uint32_t i;

    printf("  \"%s\": \"", name);
    for (i = 0U; i < n; i++)
    {
        printf("%02x", p[i]);
    }
    printf("\"%s\n", last ? "" : ",");
}

static int
vectors (void)
{
    b1_tlm_t     b1 = { 0 };
    nav_tlm_t    nav = { 0 };
    safety_tlm_t sf = { 0 };
    uint8_t      f1[B1_TLM_FRAME_BYTES];
    uint8_t      f6[NAV_TLM_FRAME_BYTES];
    uint8_t      f7[SAFETY_TLM_FRAME_BYTES];
    uint8_t      cmd[B1_CMD_BYTES];
    uint8_t      map_cmd[B1_MAP_CMD_BYTES];
    uint8_t      map_frame[B1_MAP_FRAME_BYTES];
    uint8_t      map_empty[B1_MAP_FRAME_BYTES];
    uint8_t const letters[B1_MAP_SLOTS] = { 'Z', 'A', 'M', 'Q' };
    uint32_t     mapping = 0U;

    b1.seq = 17U; b1.net = 2U; b1.state = 1U; b1.last_cmd = 12U;
    b1.last_result = 1U; b1.uptime_s = 1234U; b1.published = 54321U;
    b1.lost = 3U; b1.accepted = 9U; b1.rejected = 4U; b1.connects = 2U;
    b1.fill_pct = 37U;
    b1_tlm_pack(&b1, f1);

    nav.seq = 5U; nav.state = 2U; nav.owner = 4U; nav.b_slow = true;
    nav.search_step = 3U; nav.dropped = 200U; nav.commands_sent = 40000U;
    nav_tlm_pack(&nav, f6);

    sf.seq = 99U; sf.state = 1U; sf.b_inhibit = true; sf.fault_src = 4U;
    sf.warn_src = 7U; sf.fault_code = 0x21U; sf.warn_code = 0x08U;
    sf.estops = 3U; sf.faults = 1U; sf.warnings = 250U; sf.slow_stops = 0U;
    sf.last_stop_ms = 12U; sf.max_stop_ms = 1023U; sf.fault_mask = 0x15U;
    safety_tlm_pack(&sf, f7);

    b1_cmd_pack(B1_CMD_CONFIG, 77U, (1U << 12) | 1U, cmd);
    (void)b1_mapping_encode(letters, &mapping);
    b1_map_cmd_pack(201U, mapping, map_cmd);
    b1_mapping_frame(b1_mapping_replace(0U, mapping), 201U, 255U, map_frame);
    b1_mapping_frame(0U, 7U, 0U, map_empty);

    printf("{\n");
    hex("b1", f1, sizeof(f1), 0);
    hex("nav", f6, sizeof(f6), 0);
    hex("safety", f7, sizeof(f7), 0);
    hex("cmd_config", cmd, sizeof(cmd), 0);
    hex("cmd_mapping", map_cmd, sizeof(map_cmd), 0);
    hex("mapping", map_frame, sizeof(map_frame), 0);
    hex("mapping_empty", map_empty, sizeof(map_empty), 1);
    printf("}\n");
    return 0;
}

int
main (int argc, char ** argv)
{
    if ((argc > 1) && (strcmp(argv[1], "--vectors") == 0))
    {
        return vectors();
    }

    test_commands();
    test_ack();
    test_status();
    test_helpers();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
