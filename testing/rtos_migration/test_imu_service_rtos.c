/* Same service/processor fixtures as the SDK, with the RTOS TIMER port. */
#include <stdint.h>
#include <stdbool.h>
/* Service fixture uses a synthetic FIFO, not the physical transport. */
void ri_acquisition_record(uint32_t gap_us, bool ok, bool overrun) {
    (void)gap_us;
    (void)ok;
    (void)overrun;
}
extern uint32_t host_now_us;
uint64_t board_time_us(void) {
    return host_now_us;
}
#include "tests/imu/test_buddy4_v23_service.c"
