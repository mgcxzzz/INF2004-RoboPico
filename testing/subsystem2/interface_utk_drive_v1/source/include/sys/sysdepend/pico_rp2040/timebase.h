/* RP2040/Pico W: clk_ref is the 12 MHz XOSC; TIMER needs a 1 MHz tick.
 * This can run before .data/.bss initialization: registers and locals only.
 * Host tests replace the two MMIO macros; the production policy is shared. */
#ifndef UTK_RP2040_TIMEBASE_H
#define UTK_RP2040_TIMEBASE_H
#include <stdbool.h>
#include <stdint.h>

#define UTK_TB_TICK_ADDR       0x4005802cu
#define UTK_TB_REF_DIV_ADDR    0x40008034u
#define UTK_TB_REF_SELECTED    0x40008038u
#define UTK_TB_ENABLE          0x200u
#define UTK_TB_RUNNING         0x400u
#define UTK_TB_CYCLES_MASK     0x1ffu
#define UTK_TB_CYCLES          12u
#define UTK_TB_POLL_LIMIT      100000u
#ifndef UTK_TB_READ
#define UTK_TB_READ(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#endif
#ifndef UTK_TB_WRITE
#define UTK_TB_WRITE(addr,value) (*(volatile uint32_t *)(uintptr_t)(addr)=(value))
#endif

static inline bool utk_timebase_registers_ok(uint32_t tick,uint32_t selected,uint32_t divider)
{
    return (tick&(UTK_TB_ENABLE|UTK_TB_RUNNING|UTK_TB_CYCLES_MASK))==
        (UTK_TB_ENABLE|UTK_TB_RUNNING|UTK_TB_CYCLES) &&
        (selected&7u)==4u && divider==0x100u;
}
static inline bool utk_timebase_program(void)
{
    /* Do not assume RUNNING means the divider is correct. The reset value
     * has ENABLE set and CYCLES=0. Always establish the required divider. */
    if((UTK_TB_READ(UTK_TB_REF_SELECTED)&7u)!=4u ||
       UTK_TB_READ(UTK_TB_REF_DIV_ADDR)!=0x100u)return false;
    UTK_TB_WRITE(UTK_TB_TICK_ADDR,UTK_TB_ENABLE|UTK_TB_CYCLES);
    for(uint32_t n=0;n<UTK_TB_POLL_LIMIT;++n) {
        uint32_t tick=UTK_TB_READ(UTK_TB_TICK_ADDR);
        if(utk_timebase_registers_ok(tick,UTK_TB_READ(UTK_TB_REF_SELECTED),
                                    UTK_TB_READ(UTK_TB_REF_DIV_ADDR)))return true;
    }
    return false;
}
static inline bool utk_timebase_elapsed_ok(uint32_t timer_us,uint32_t kernel_ms)
{
    /* 250 ms task delay measured in two independent clock domains. Include
     * 2 ms endpoint allowance plus 2% for tick quantisation/scheduling. */
    if(kernel_ms<200u || kernel_ms>2000u)return false;
    uint32_t expected=kernel_ms*1000u;
    uint32_t difference=timer_us>expected?timer_us-expected:expected-timer_us;
    return difference<=2000u+expected/50u;
}
#endif
