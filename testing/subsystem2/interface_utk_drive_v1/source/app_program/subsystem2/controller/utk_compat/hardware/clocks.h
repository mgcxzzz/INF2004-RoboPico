#ifndef UTK_COMPAT_HARDWARE_CLOCKS_H
#define UTK_COMPAT_HARDWARE_CLOCKS_H
#include <stdint.h>
typedef unsigned int clock_handle_t;
enum { clk_sys = 0u };
uint32_t utk_clock_get_hz(clock_handle_t clock);
#define clock_get_hz utk_clock_get_hz
#endif
