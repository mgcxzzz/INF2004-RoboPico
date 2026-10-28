#ifndef TIMEBASE_TEST_PORT_H
#define TIMEBASE_TEST_PORT_H
#include <stdint.h>
uint32_t test_read(uint32_t addr);
void test_write(uint32_t addr,uint32_t value);
#define UTK_TB_READ(addr) test_read(addr)
#define UTK_TB_WRITE(addr,value) test_write(addr,value)
#endif
