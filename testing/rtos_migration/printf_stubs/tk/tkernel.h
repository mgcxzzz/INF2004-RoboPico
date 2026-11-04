#ifndef PRINTF_TEST_TKERNEL_H
#define PRINTF_TEST_TKERNEL_H
#include <stddef.h>
typedef unsigned char UB;
typedef short H;
typedef unsigned short UH;
typedef unsigned int UW;
typedef unsigned int UINT;
typedef int W;
typedef int INT;
#define LOCAL static
#define EXPORT
#define IMPORT extern
#define USE_TMONITOR 1
#define USE_TM_PRINTF 1
#define TK_SUPPORT_SMP 1
#define TM_CONSOLE_USB_CDC 1
#define TM_CONSOLE_UART 0
#endif
