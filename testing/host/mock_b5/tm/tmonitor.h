#ifndef MOCK_B5_TMONITOR_H
#define MOCK_B5_TMONITOR_H
#include <stdarg.h>
#include <stdio.h>
static inline int tm_printf(const unsigned char *fmt, ...) {
    int n; va_list args; va_start(args,fmt); n=vprintf((const char*)fmt,args); va_end(args); return n;
}
#endif
