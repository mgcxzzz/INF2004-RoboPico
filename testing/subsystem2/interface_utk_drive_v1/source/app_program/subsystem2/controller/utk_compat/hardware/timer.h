#ifndef UTK_COMPAT_HARDWARE_TIMER_H
#define UTK_COMPAT_HARDWARE_TIMER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct repeating_timer {
    int64_t delay_us;
    bool (*callback)(struct repeating_timer *timer);
    void *user_data;
    bool active;
} repeating_timer;
bool utk_add_repeating_timer_us(int64_t delay_us,
                                bool (*callback)(repeating_timer *timer),
                                void *user_data, repeating_timer *timer);
#define add_repeating_timer_us utk_add_repeating_timer_us
#endif
