#include <assert.h>
#include <stdio.h>
#include "src/navigation/nav_log_storage.h"
int main(void) {
    uint8_t bank[NAV_LOG_FLASH_BYTES];
    memset(bank, 0xff, sizeof(bank));
    MotionEvent e = {.now_us = 12345678,
                     .id = 7,
                     .requester = 5,
                     .owner = 5,
                     .type = MOTION_EVENT_DONE,
                     .kind = NAV_DRIVE,
                     .state = MOTION_CMD_DONE,
                     .goal = -30,
                     .progress_mm = -29.5f,
                     .target_mm_s = {0, 0}};
    for (unsigned n = 0; n < NAV_LOG_ROWS; ++n) {
        e.sequence = n + 1;
        nav_event_encode(bank + NAV_LOG_HEADER + n * NAV_EVENT_BYTES, &e);
    }
    uint32_t data_crc = nav_crc(bank + NAV_LOG_HEADER, NAV_LOG_ROWS * NAV_EVENT_BYTES);
    nav_log_header(bank, NAV_LOG_ROWS, 32, 12300000, 9000000, 6, data_crc);
    assert(nav_log_valid(bank) && nav_rd64(bank + 24) == 12300000 &&
           nav_rd64(bank + 32) == 9000000);
    bank[NAV_LOG_HEADER + 17] ^= 1;
    assert(!nav_log_valid(bank));
    bank[NAV_LOG_HEADER + 17] ^= 1;
    bank[30] ^= 1;
    assert(!nav_log_valid(bank));
    bank[30] ^= 1;
    assert(nav_log_valid(bank));
    uint8_t packet[NAV_EVENT_BYTES];
    nav_event_encode(packet, &e);
    packet[NAV_EVENT_BYTES - 1] |= 0x80;
    assert(!nav_event_decode(&e, packet, NULL));
    memset(bank, 0xff, NAV_LOG_HEADER);
    assert(!nav_log_valid(bank));
    nav_log_header(bank, 0, 0, 0, 0, 0, nav_crc(bank + NAV_LOG_HEADER, 0));
    assert(nav_log_valid(bank));
    puts(
        "PASS separate coordinator flash layout: full/empty bank, data/header corruption, erased/partial header and event padding rejection.");
}
