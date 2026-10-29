#ifndef BUDDY_EXTENSION_HOOKS_H
#define BUDDY_EXTENSION_HOOKS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Called from the existing standalone main loop. Keep non-blocking. */
void buddy_extension_init(void);
void buddy_extension_poll(uint64_t now_us);
#ifdef __cplusplus
}
#endif
#endif
