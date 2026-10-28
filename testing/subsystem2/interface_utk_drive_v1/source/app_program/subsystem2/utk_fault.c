#include <stdint.h>
/* libgcc's default divide-by-zero path uses hosted signals/stdio. A fatal
 * arithmetic fault in this freestanding controller must remove motor PWM. */
static void __attribute__((noreturn)) arithmetic_halt(void) {
    *(volatile uint32_t *)0x4005005cu=0; /* GP8/9 slice4 compare */
    *(volatile uint32_t *)0x40050070u=0; /* GP10/11 slice5 compare */
    __asm__ volatile("cpsid i":::"memory");
    for(;;)__asm__ volatile("wfi");
}
int __aeabi_idiv0(int value){(void)value;arithmetic_halt();}
long long __aeabi_ldiv0(long long value){(void)value;arithmetic_halt();}
