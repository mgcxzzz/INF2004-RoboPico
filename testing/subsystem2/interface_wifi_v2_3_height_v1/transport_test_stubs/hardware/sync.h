#ifndef TEST_HARDWARE_SYNC_H
#define TEST_HARDWARE_SYNC_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { int unused; } spin_lock_t;
unsigned spin_lock_claim_unused(bool required);
spin_lock_t *spin_lock_instance(unsigned lock_num);
uint32_t spin_lock_blocking(spin_lock_t *lock);
void spin_unlock(spin_lock_t *lock, uint32_t irq_state);
#endif
