#ifndef LWIP_TEST_CC_H
#define LWIP_TEST_CC_H
#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS 1
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
typedef unsigned sys_prot_t;
void test_lwip_assert(const char *message);
#define LWIP_PLATFORM_ASSERT(message) test_lwip_assert(message)
#define LWIP_PLATFORM_DIAG(message)                                                                \
    do {                                                                                           \
        printf message;                                                                            \
    } while (0)
#define LWIP_RAND() ((uint32_t)1u)
#endif
