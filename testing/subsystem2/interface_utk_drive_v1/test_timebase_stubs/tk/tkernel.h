#ifndef TIMEBASE_TEST_KERNEL_H
#define TIMEBASE_TEST_KERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int ER;
typedef int INT;
typedef unsigned int UINT;
typedef unsigned char UB;
typedef int BOOL;
#define FALSE 0
#define TRUE 1
typedef struct { int32_t hi;uint32_t lo; } SYSTIM;
typedef struct { uint32_t intatr;void (*inthdr)(UINT); } T_DINT;
#define E_OK 0
#define E_SYS (-5)
#define E_PAR (-17)
#define TA_HLNG 1
#define RESETS_RESET 0x4000c000u
#define RESETS_RESET_DONE 0x4000c008u
#define RESETS_RESET_TIMER 0x200000u
#define TIMER_TIMERAWL 0x40054028u
#define IO_BANK0_BASE 0x40014000u
uint32_t in_w(uint32_t addr);
void out_w(uint32_t addr,uint32_t value);
ER tk_get_otm(SYSTIM *time);
ER tk_dly_tsk(uint32_t ms);
ER tk_def_int(UINT irq,const T_DINT *definition);
void ClearInt(UINT irq);
void EnableInt(UINT irq,INT level);
#endif
