#ifndef UTK_COMPAT_PICO_STDIO_USB_H
#define UTK_COMPAT_PICO_STDIO_USB_H
#include <stdbool.h>
bool utk_stdio_usb_connected(void);
#define stdio_usb_connected utk_stdio_usb_connected
#endif
