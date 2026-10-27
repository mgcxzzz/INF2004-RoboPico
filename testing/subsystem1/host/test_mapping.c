/* S1-U18: runtime barcode mapping, protocol and all possible letter maps.
 * Compiles the same pure C files used on the Pico. Group IS28, Buddy 1. */
#include <stdio.h>
#include "b1_proto.h"
#include "b1_mapping.h"

static unsigned int g_pass;
static unsigned int g_fail;
#define CHECK(id, cond) do { \
    bool ok = (cond); \
    printf("%s %s\n", ok ? "PASS" : "FAIL", id); \
    if (ok) { g_pass++; } else { g_fail++; } \
} while (0)

int main(void)
{
    uint8_t letters[4] = { 'Z', 'A', 'M', 'Q' };
    uint32_t word = 0U;
    uint32_t state;
    uint32_t readback = 0xFFFFFFFFU;
    uint8_t request = 0U;
    uint8_t cmd[8] = { 0U };
    uint8_t frame[B1_MAP_FRAME_BYTES];
    b1_cmd_t decoded;
    bool all_ok = true;
    uint32_t i;
    uint32_t valid = 0U;

    CHECK("M01 no hard-coded boot mapping",
          b1_mapping_lookup(0U, 'A') == B1_MOVE_NONE);
    CHECK("M02 arbitrary distinct letters accepted",
          b1_mapping_encode(letters, &word));
    state = b1_mapping_replace(0U, word);
    CHECK("M03 movement slots match the specified wire order",
          (b1_mapping_lookup(state, 'Z') == B1_MOVE_FORWARD) &&
          (b1_mapping_lookup(state, 'A') == B1_MOVE_UTURN) &&
          (b1_mapping_lookup(state, 'M') == B1_MOVE_LEFT) &&
          (b1_mapping_lookup(state, 'Q') == B1_MOVE_RIGHT));
    CHECK("M04 unassigned, lowercase, punctuation, invalid state are not movement",
          (b1_mapping_lookup(state, 'B') == B1_MOVE_NONE) &&
          (b1_mapping_lookup(state, 'z') == B1_MOVE_NONE) &&
          (b1_mapping_lookup(state, '*') == B1_MOVE_NONE) &&
          (b1_mapping_lookup(0xFFFFFFFFU, 'Z') == B1_MOVE_NONE));
    letters[1] = 'Z';
    readback = 12345U;
    CHECK("M05 duplicate letters rejected, output untouched",
          !b1_mapping_encode(letters, &readback) && (readback == 12345U));
    letters[1] = 'a';
    CHECK("M06 lowercase rejected", !b1_mapping_encode(letters, &readback));
    letters[1] = '[';
    CHECK("M07 out-of-range ASCII rejected", !b1_mapping_encode(letters, &readback));
    CHECK("M08 invalid replacement leaves active map untouched",
          b1_mapping_replace(state, 0U) == state);
    CHECK("M09 identical retry is idempotent",
          b1_mapping_replace(state, word) == state);
    readback = b1_mapping_replace((state & ~(255UL << B1_MAP_REV_SHIFT)) |
                                  (255UL << B1_MAP_REV_SHIFT), word ^ 1U);
    CHECK("M10 revision wraps without corrupting map",
          ((readback >> B1_MAP_REV_SHIFT) == 0U) &&
          ((readback & B1_MAP_MASK) == (word ^ 1U)) &&
          ((readback & B1_MAP_CONFIGURED) != 0U));

    /* Exhaust every 20-bit value, not just hand-picked examples. */
    for (i = 0U; i <= B1_MAP_MASK; i++)
    {
        if (b1_mapping_check(i))
        {
            uint32_t slot;
            uint32_t snapshot = b1_mapping_replace(0U, i);
            valid++;
            for (slot = 0U; slot < 4U; slot++)
            {
                uint8_t ch = (uint8_t)('A' + ((i >> (slot * 5U)) & 31U));
                all_ok &= b1_mapping_lookup(snapshot, ch) == (b1_movement_t)(slot + 1U);
            }
        }
    }
    CHECK("M11 all 358800 distinct A-Z assignments resolve correctly",
          all_ok && (valid == 26U * 25U * 24U * 23U));

    b1_map_cmd_pack(201U, word, cmd);
    CHECK("M12 5-byte mapping command round trip",
          (b1_cmd_check(cmd, B1_MAP_CMD_BYTES, &decoded) == B1_ACK_ACCEPTED) &&
          (decoded.mapping == word) && (decoded.id == 201U));
    all_ok = true;
    for (i = 0U; i <= sizeof(cmd); i++)
    {
        if (i != B1_MAP_CMD_BYTES)
        {
            all_ok &= b1_cmd_check(cmd, i, &decoded) == B1_ACK_FORMAT;
        }
    }
    CHECK("M13 wrong lengths cannot apply a mapping prefix", all_ok);
    cmd[4] |= 0x10U;
    CHECK("M14 reserved command bits rejected",
          b1_cmd_check(cmd, B1_MAP_CMD_BYTES, &decoded) == B1_ACK_FORMAT);
    b1_map_cmd_pack(9U, 0U, cmd);
    CHECK("M15 duplicate encoded letters rejected",
          b1_cmd_check(cmd, B1_MAP_CMD_BYTES, &decoded) == B1_ACK_FORMAT);
    b1_map_cmd_pack(9U, (word & ~31UL) | 26U, cmd);
    CHECK("M16 letter index above Z rejected",
          b1_cmd_check(cmd, B1_MAP_CMD_BYTES, &decoded) == B1_ACK_FORMAT);
    b1_cmd_pack(B1_CMD_BARCODE_GET, 0U, 0U, cmd);
    CHECK("M17 GET accepted with no argument",
          b1_cmd_check(cmd, B1_CMD_BYTES, &decoded) == B1_ACK_ACCEPTED);
    cmd[2] = 1U;
    CHECK("M18 GET with unexpected argument rejected",
          b1_cmd_check(cmd, B1_CMD_BYTES, &decoded) == B1_ACK_FORMAT);

    b1_mapping_frame(state, 201U, 255U, frame);
    CHECK("M19 active map readback includes matching request id",
          b1_mapping_readback(frame, sizeof(frame), &readback, &request) &&
          (readback == state) && (request == 201U) && (frame[2] == 255U));
    frame[6] |= 0xE0U;
    CHECK("M20 corrupt readback reserved bits rejected",
          !b1_mapping_readback(frame, sizeof(frame), &readback, &request));
    b1_mapping_frame(0U, 7U, 0U, frame);
    CHECK("M21 boot readback explicitly says not configured",
          b1_mapping_readback(frame, sizeof(frame), &readback, &request) &&
          (readback == 0U) && (request == 7U));
    CHECK("M22 truncated readback rejected",
          !b1_mapping_readback(frame, sizeof(frame) - 1U, &readback, &request));
    frame[3] = 1U;
    CHECK("M23 unconfigured status cannot contain an assignment",
          !b1_mapping_readback(frame, sizeof(frame), &readback, &request));
    printf("\n%u passed, %u failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
