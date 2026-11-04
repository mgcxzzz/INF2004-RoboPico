/* Same service/processor fixtures as the SDK, with the RTOS TIMER port. */
#include <stdint.h>
extern uint32_t host_now_us;
uint64_t board_time_us(void) {
    return host_now_us;
}
#include "tests/imu/test_buddy4_v23_service.c"
