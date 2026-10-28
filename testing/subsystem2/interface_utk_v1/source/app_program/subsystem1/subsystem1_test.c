/**
 * @file    subsystem1_test.c
 * @brief   On-target tests for subsystem 1 (WiFi and MQTT). Built only with
 *          make TEST=1 (add WIFI=cyw43 NET=lwip MQTT=1 for the network).
 *
 * Each TEST_CHECK prints a PASS/FAIL line on the console. Save the console
 * output to testing/subsystem1/logs/ and record the result in
 * testing/subsystem1/TEST_LOG.md. IDs as in testing/subsystem1/HOW_TO_TEST.md.
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#include "subsystem1.h"
#include "b1_net.h"
#include "b1_proto.h"
#include "../system/nav_iface.h"
#include "../system/safety_iface.h"
#include "../test_report.h"

#if APP_TEST

void b1_test_counts(uint8_t * p_accepted, uint8_t * p_rejected);

/* Inject a command as if it came from MQTT, then give the task time. */
static void
inject (uint8_t code, uint8_t id, uint16_t arg)
{
    uint8_t msg[B1_CMD_BYTES];

    b1_cmd_pack(code, id, arg, msg);
    b1_net_command(msg, B1_CMD_BYTES);
    (void)tk_dly_tsk(300);
}

EXPORT void
subsystem1_test (void)
{
    b1_cmd_t cmd;
    uint8_t  msg[B1_CMD_BYTES];
    uint8_t  frame[B1_TLM_FRAME_BYTES];
    b1_tlm_t t;
    uint8_t  acc0;
    uint8_t  rej0;
    uint8_t  acc1;
    uint8_t  rej1;

    TEST_INIT();

    /* S1-U01: command checking (FR-WIFI-02/03) */
    b1_cmd_pack(B1_CMD_START, 1U, 0U, msg);
    TEST_CHECK("S1-U01a", b1_cmd_check(msg, 4U, &cmd) == B1_ACK_ACCEPTED,
               "START is accepted");
    b1_cmd_pack(B1_CMD_STEER, 2U, 300U, msg);
    TEST_CHECK("S1-U01b", b1_cmd_check(msg, 4U, &cmd) == B1_ACK_PROHIBITED,
               "STEER is rejected as prohibited");
    TEST_CHECK("S1-U01c", b1_cmd_check(msg, 3U, &cmd) == B1_ACK_FORMAT,
               "a 3-byte command is rejected as bad format");

    /* S1-U02: the heartbeat frame decodes again (team schema, source 1) */
    (void)b1_net_heartbeat(frame);
    TEST_CHECK("S1-U02", b1_tlm_unpack(frame, B1_TLM_FRAME_BYTES, &t) &&
               (t.net < (uint8_t)B1_NET_COUNT),
               "heartbeat frame round trip");

    /* S1-U03: a prohibited command never reaches the coordinator */
    {
        nav_state_t before = nav_get_state();

        b1_test_counts(&acc0, &rej0);
        inject(B1_CMD_DRIVE, 3U, 500U);
        b1_test_counts(&acc1, &rej1);
        TEST_CHECK("S1-U03", ((uint8_t)(rej1 - rej0) == 1U) &&
                   (acc1 == acc0) && (nav_get_state() == before),
                   "DRIVE rejected, operating state unchanged");
    }

    /* S1-U04: emergency stop goes to the Safety Supervisor; reset clears */
    inject(B1_CMD_ESTOP, 4U, 0U);
    TEST_CHECK("S1-U04a", safety_get_state() == SAFETY_ST_ESTOP,
               "ESTOP command latches the emergency stop");
    inject(B1_CMD_RESET, 5U, 0U);
    TEST_CHECK("S1-U04b", safety_get_state() == SAFETY_ST_NORMAL,
               "RESET command clears it");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
