/**
 * @file log_store.h
 * @brief Log store.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef LOG_STORE_H
#define LOG_STORE_H
#include <stdint.h>
#include <stddef.h>
static inline uint32_t log_crc (const void * p_data, size_t count)
{
    const uint8_t * p_data_value = p_data;
    uint32_t crc = 0xffffffffu;
    while (count--)
    {
        crc ^= *p_data_value++;
        for (unsigned second = 0; second < 8; ++second)
        {
            crc = (crc >> 1) ^ ((0u - (crc & 1u)) & 0xedb88320u);
        }
    }
    return ~crc;
}
#endif

/* End of file. */
