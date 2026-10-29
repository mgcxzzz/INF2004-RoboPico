#ifndef TEST_PICO_CYW43_ARCH_H
#define TEST_PICO_CYW43_ARCH_H
#include <stdint.h>
int cyw43_arch_init(void);
void cyw43_arch_enable_sta_mode(void);
void cyw43_arch_poll(void);
int cyw43_arch_wifi_connect_async(const char *, const char *, uint32_t);
#endif
