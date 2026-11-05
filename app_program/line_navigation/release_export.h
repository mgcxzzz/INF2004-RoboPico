#ifndef RTOS_RELEASE_EXPORT_H
#define RTOS_RELEASE_EXPORT_H
#include <stdbool.h>
#include <stdint.h>
/* Owner-only transfer identity; only the deadline/busy view crosses the
 * existing line-copy lock. Full timestamps/sequence are modulo32 domains. */
typedef struct {
    uint32_t until_us, nav_sequence;
    unsigned total : 12, run : 16, dataset : 2, active : 1;
} RlExportLease;
static inline bool rl_export_busy(const RlExportLease *s, uint32_t now) {
    return s->active && (int32_t)(s->until_us - now) > 0;
}
static inline void rl_export_open(RlExportLease *s, unsigned dataset, unsigned total,
                                  uint16_t run, uint32_t nav_sequence, uint32_t now) {
    *s = (RlExportLease){.until_us = now + 5000000U, .nav_sequence = nav_sequence,
                         .total = total, .run = run, .dataset = dataset, .active = 1};
}
static inline bool rl_export_same(const RlExportLease *s, unsigned dataset, unsigned total,
                                  uint16_t run, uint32_t nav_sequence, uint32_t now) {
    return rl_export_busy(s, now) && s->dataset == dataset && s->total == total &&
           s->run == run && s->nav_sequence == nav_sequence;
}
/* Explicit little endian encoding; never send compiler-layout structs. */
static inline void rl_put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void rl_put32(uint8_t *p, uint32_t v) {
    for (unsigned k = 0; k < 4; ++k) p[k] = v >> (8 * k);
}
#endif
