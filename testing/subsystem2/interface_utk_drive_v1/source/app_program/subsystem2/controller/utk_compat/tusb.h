#ifndef UTK_COMPAT_TUSB_H
#define UTK_COMPAT_TUSB_H
#include <stdbool.h>
bool utk_tud_mounted(void);
#define tud_mounted utk_tud_mounted
#endif
