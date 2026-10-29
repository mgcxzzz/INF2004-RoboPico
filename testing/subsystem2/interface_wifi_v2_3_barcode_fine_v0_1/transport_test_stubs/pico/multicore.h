#ifndef TEST_PICO_MULTICORE_H
#define TEST_PICO_MULTICORE_H
#include <stdbool.h>
#include <stdint.h>
void multicore_lockout_victim_init(void);
bool multicore_lockout_victim_is_initialized(unsigned core);
bool multicore_lockout_start_timeout_us(uint64_t timeout_us);
void multicore_lockout_end_blocking(void);
void multicore_launch_core1(void (*entry)(void));
#endif
