#ifndef RTOS_IMU_ACQUISITION_SCHEDULE_H
#define RTOS_IMU_ACQUISITION_SCHEDULE_H
#include <stdint.h>
#define RI_PERIOD_US 20000U
/* Preserve the reference fixed-rate/rebase policy, but sleep to the next
 * release instead of repeatedly delaying after every poll. */
static inline uint64_t ri_next_release(uint64_t due, uint64_t now) {
    return now - due > RI_PERIOD_US ? now + RI_PERIOD_US : due + RI_PERIOD_US;
}
static inline uint32_t ri_release_delay_ms(uint64_t due, uint64_t now) {
    uint64_t ms = due > now ? (due - now) / 1000U : 0U;
    /* Kernel relative waits add one timer tick (1ms in this profile). */
    return ms > 1U ? (uint32_t)(ms - 1U) : 1U;
}
#endif
