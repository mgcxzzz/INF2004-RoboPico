/**
 * @file b1_mapping.h
 * @brief Runtime barcode configuration and wire format. Pure C, no RTOS.
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */
#ifndef B1_MAPPING_H
#define B1_MAPPING_H

#include "../system/sys_tlm.h"

#define B1_MAP_CONFIGURED       (1UL << 20)
#define B1_MAP_REV_SHIFT        (21U)
#define B1_MAP_STATUS_MASK      (0x1FFFFFFFUL)

/* Configuration slots, in wire order. Not motor commands. */
typedef enum
{
    B1_MOVE_NONE = 0,
    B1_MOVE_FORWARD,
    B1_MOVE_UTURN,
    B1_MOVE_LEFT,
    B1_MOVE_RIGHT
} b1_movement_t;

/* Four distinct uppercase ASCII letters, in FORWARD, UTURN, LEFT, RIGHT
 * order. Invalid input leaves *p_word unchanged. */
bool b1_mapping_encode(uint8_t const p_letters[B1_MAP_SLOTS], uint32_t * p_word);
bool b1_mapping_check(uint32_t word);

/* Whole-map replacement. Identical retries keep the same revision. */
uint32_t b1_mapping_replace(uint32_t snapshot, uint32_t word);
b1_movement_t b1_mapping_lookup(uint32_t snapshot, uint8_t letter);

void b1_mapping_frame(uint32_t snapshot, uint8_t request_id, uint8_t seq,
                      uint8_t p_frame[B1_MAP_FRAME_BYTES]);
bool b1_mapping_readback(uint8_t const * p_frame, uint32_t len,
                        uint32_t * p_snapshot, uint8_t * p_request_id);

#endif /* B1_MAPPING_H */
