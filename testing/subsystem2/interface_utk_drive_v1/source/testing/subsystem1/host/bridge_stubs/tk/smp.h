#ifndef DRIVE_BRIDGE_TEST_SMP_H
#define DRIVE_BRIDGE_TEST_SMP_H
#include <stdint.h>
#include <tk/tkernel.h>
typedef volatile uint32_t T_SPLOCK;
ER InitSpinLock(T_SPLOCK *lock);
ER ISpinLock(T_SPLOCK *lock,UINT *intsts);
ER ISpinUnlock(T_SPLOCK *lock,UINT intsts);
#endif
