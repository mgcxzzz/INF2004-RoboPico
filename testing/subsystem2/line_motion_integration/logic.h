#ifndef INTEGRATION_LOGIC_H
#define INTEGRATION_LOGIC_H
#include <stdint.h>
#include <stdbool.h>
/* Scale ALL S3 speeds by one half, preserving the intended steering ratio. */
static inline bool bench_targets(int16_t l,int16_t r,int16_t out[2]) {
    if(l<0 || r<0 || l>200 || r>200) return false;
    out[0]=l/2; out[1]=r/2;
    return true;
}
static inline bool bench_stale(uint32_t now,uint32_t last) { return now-last>=50000u; }
static inline uint32_t abs_ticks(int32_t n) { return n<0 ? (uint32_t)(-(int64_t)n) : (uint32_t)n; }
#endif
