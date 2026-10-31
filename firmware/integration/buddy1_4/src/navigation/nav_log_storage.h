#ifndef BUDDY_NAV_LOG_STORAGE_H
#define BUDDY_NAV_LOG_STORAGE_H
#include "src/navigation/nav_codec.h"
#include <string.h>
enum { NAV_LOG_ROWS = 120, NAV_LOG_HEADER = 256, NAV_LOG_FLASH_BYTES = 4096 };
_Static_assert(NAV_LOG_HEADER + NAV_LOG_ROWS * NAV_EVENT_BYTES <= NAV_LOG_FLASH_BYTES,
               "Coordinator log bank overflow");
static inline uint32_t nav_crc_update(uint32_t c, const uint8_t *p, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned b = 0; b < 8; ++b)
            c = (c >> 1) ^ ((0u - (c & 1u)) & 0xedb88320u);
    }
    return c;
}
static inline uint32_t nav_crc(const uint8_t *p, unsigned n) {
    return ~nav_crc_update(UINT32_MAX, p, n);
}
static inline void nav_le32(uint8_t *p, uint32_t v) {
    for (unsigned k = 0; k < 4; ++k)
        p[k] = (uint8_t)(v >> (k * 8));
}
static inline uint32_t nav_rd32(const uint8_t *p) {
    uint32_t v = 0;
    for (unsigned k = 0; k < 4; ++k)
        v |= (uint32_t)p[k] << (k * 8);
    return v;
}
static inline void nav_le64(uint8_t *p, uint64_t v) {
    for (unsigned k = 0; k < 8; ++k)
        p[k] = (uint8_t)(v >> (k * 8));
}
static inline uint64_t nav_rd64(const uint8_t *p) {
    uint64_t v = 0;
    for (unsigned k = 0; k < 8; ++k)
        v |= (uint64_t)p[k] << (k * 8);
    return v;
}
static inline void nav_log_header(uint8_t h[NAV_LOG_HEADER], unsigned count, unsigned overwritten,
                                  uint64_t start, uint64_t duration, unsigned reason,
                                  uint32_t data_crc) {
    memset(h, 0, NAV_LOG_HEADER);
    memcpy(h, "NCL1", 4);
    h[4] = 1;
    h[5] = (uint8_t)count;
    h[6] = (uint8_t)(reason & 63);
    h[7] = NAV_EVENT_BYTES;
    nav_le32(h + 8, overwritten);
    nav_le32(h + 12, data_crc);
    nav_le64(h + 24, start);
    nav_le64(h + 32, duration);
    nav_le32(h + 16, nav_crc(h, NAV_LOG_HEADER));
}
static inline bool nav_log_valid(const uint8_t *bank) {
    if (memcmp(bank, "NCL1", 4) || bank[4] != 1 || bank[5] > NAV_LOG_ROWS || bank[6] > 63 ||
        bank[7] != NAV_EVENT_BYTES)
        return false;
    uint8_t header[NAV_LOG_HEADER];
    memcpy(header, bank, sizeof(header));
    uint32_t expected = nav_rd32(header + 16);
    memset(header + 16, 0, 4);
    return nav_crc(header, sizeof(header)) == expected &&
           nav_crc(bank + NAV_LOG_HEADER, bank[5] * NAV_EVENT_BYTES) == nav_rd32(bank + 12);
}
#endif
