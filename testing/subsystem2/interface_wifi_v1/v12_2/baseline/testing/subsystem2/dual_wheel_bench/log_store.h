#ifndef LOG_STORE_H
#define LOG_STORE_H
#include <stdint.h>
#include <stddef.h>
static inline uint32_t log_crc(const void *data, size_t n) {
    const uint8_t *p=data;
    uint32_t crc=0xffffffffu;
    while(n--) {
        crc^=*p++;
        for(unsigned b=0;b<8;++b) crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
#endif
