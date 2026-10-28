#ifndef UTK_COMPAT_HARDWARE_SYNC_H
#define UTK_COMPAT_HARDWARE_SYNC_H
#include <stdbool.h>
#include <stdint.h>
typedef unsigned int uint;
/* Opaque kernel lock handle.  The public layout stays tiny and carries only
 * the index; T_SPLOCK storage lives in the T-Kernel-only adapter. */
typedef struct { uint32_t number; } spin_lock_t;
uint32_t utk_spin_lock_claim_unused(bool required);
spin_lock_t *utk_spin_lock_instance(uint lock_num);
uint32_t utk_spin_lock_blocking(spin_lock_t *lock);
void utk_spin_unlock(spin_lock_t *lock, uint32_t irq_state);
uint32_t utk_save_and_disable_interrupts(void);
void utk_restore_interrupts(uint32_t state);
#define spin_lock_claim_unused utk_spin_lock_claim_unused
#define spin_lock_instance utk_spin_lock_instance
#define spin_lock_blocking utk_spin_lock_blocking
#define spin_unlock utk_spin_unlock
#define save_and_disable_interrupts utk_save_and_disable_interrupts
#define restore_interrupts utk_restore_interrupts
#endif
