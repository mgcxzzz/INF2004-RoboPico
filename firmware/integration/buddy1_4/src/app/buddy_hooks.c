#include "include/interfaces/buddy_hooks.h"
/* Replace these no-ops with Buddy5's non-blocking ultrasonic/state machine,
 * or supply strong definitions in another source added to CMake. */
__attribute__((weak)) void buddy_extension_init(void) {}
__attribute__((weak)) void buddy_extension_poll(uint64_t now_us) {
    (void)now_us;
}
