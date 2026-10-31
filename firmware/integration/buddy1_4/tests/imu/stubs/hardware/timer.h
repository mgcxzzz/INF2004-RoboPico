/* host stub: controllable microsecond clock */
#ifndef HOST_TIMER_H
#define HOST_TIMER_H
#include <stdint.h>
extern uint32_t host_now_us;
static inline uint32_t time_us_32(void) {
    return host_now_us;
}
#endif
