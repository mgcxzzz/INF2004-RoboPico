/**
 * @file barcode_raw.h
 * @brief Barcode raw.
 * @details Pure barcode/route logic is called by the ADC owner and emits
 * requests. Native records are not wire packets; preserve both decode
 * directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_RAW_H
#define BARCODE_RAW_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "src/motion/log_store.h"

enum
{
#ifdef BUDDY_WIFI_LOGGING
    RAW_CAPACITY = 2048,
    RAW_VERSION = 2,
#else
    RAW_CAPACITY = 4096,
    RAW_VERSION = 1,
#endif
    RAW_BANK_BYTES = 65536
};
#define RAW_MAGIC 0x314f4142u
typedef struct
{
    uint32_t us;
    int32_t path100;
    uint16_t adc, reserved;
} barcode_raw_row_t;
typedef struct
{
    uint32_t magic, version, crc, count, data_crc, capacity;
    uint64_t duration_us;
    int64_t ticks[2];
    uint32_t max_gap_us;
    uint8_t profile, reason, pin, channel, full;
} barcode_raw_meta_t;
typedef union
{
    barcode_raw_meta_t meta;
    uint8_t page[256];
} barcode_raw_header_t;
typedef struct
{
    barcode_raw_header_t header;
    barcode_raw_row_t rows[RAW_CAPACITY];
} barcode_raw_t;
_Static_assert(sizeof(barcode_raw_row_t) == 12 &&
                   sizeof(barcode_raw_header_t) == 256,
               "Raw trace ABI changed");
_Static_assert(sizeof(barcode_raw_t) == 256 + 12 * RAW_CAPACITY &&
                   sizeof(barcode_raw_t) % 256 == 0 &&
                   sizeof(barcode_raw_t) <= RAW_BANK_BYTES,
               "Raw flash layout");
static inline void barcode_raw_begin (barcode_raw_t * r, unsigned profile)
{
    memset(r, 0, sizeof(*r));
    r->header.meta = (barcode_raw_meta_t){.magic = RAW_MAGIC,
                                          .version = RAW_VERSION,
                                          .capacity = RAW_CAPACITY,
                                          .profile = (uint8_t)profile,
                                          .pin = 27,
                                          .channel = 1};
}
static inline void barcode_raw_sample (barcode_raw_t * r, uint64_t us,
                                       int32_t path100, uint16_t adc)
{
    barcode_raw_meta_t * m = &r->header.meta;
    if (m->count == RAW_CAPACITY)
    {
        m->full = 1;
        return;
    }
    if (us > UINT32_MAX)
    {
        m->full = 1;
        return;
    }
    if (m->count && us - r->rows[m->count - 1].us > m->max_gap_us)
    {
        m->max_gap_us = (uint32_t)(us - r->rows[m->count - 1].us);
    }
    r->rows[m->count++] =
        (barcode_raw_row_t){.us = (uint32_t)us, .path100 = path100, .adc = adc};
}
static inline void barcode_raw_finish (barcode_raw_t * r, uint64_t duration,
                                       const int64_t ticks[2], unsigned reason)
{
    barcode_raw_meta_t * m = &r->header.meta;
    m->duration_us = duration;
    m->ticks[0] = ticks[0];
    m->ticks[1] = ticks[1];
    m->reason = (uint8_t)reason;
    m->data_crc = log_crc(r->rows, m->count * sizeof(r->rows[0]));
    m->crc = log_crc((const uint8_t *)r + offsetof(barcode_raw_meta_t, count),
                     sizeof(*r) - offsetof(barcode_raw_meta_t, count));
}
static inline bool barcode_raw_valid (const barcode_raw_t * r)
{
    const barcode_raw_meta_t * m = &r->header.meta;
    if (m->magic != RAW_MAGIC || m->version != RAW_VERSION ||
        m->count > RAW_CAPACITY || m->capacity != RAW_CAPACITY ||
        m->profile < 1 || m->profile > 3 || m->pin != 27 || m->channel != 1 ||
        m->full > 1)
    {
        return false;
    }
    if (m->crc !=
            log_crc((const uint8_t *)r + offsetof(barcode_raw_meta_t, count),
                    sizeof(*r) - offsetof(barcode_raw_meta_t, count)) ||
        m->data_crc != log_crc(r->rows, m->count * sizeof(r->rows[0])))
    {
        return false;
    }
    for (unsigned n = 0; n < m->count; ++n)
    {
        if (r->rows[n].adc > 4095 || r->rows[n].reserved ||
            (n && r->rows[n].us <= r->rows[n - 1].us))
        {
            return false;
        }
    }
    return true;
}
#endif

/* End of file. */
