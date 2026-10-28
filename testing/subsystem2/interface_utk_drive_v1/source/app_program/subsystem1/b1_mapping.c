/**
 * @file b1_mapping.c
 * @brief Validated, compact barcode configuration. No default assignments.
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */
#include "b1_mapping.h"

bool
b1_mapping_check (uint32_t word)
{
    uint32_t seen = 0U;
    uint32_t i;

    if ((word & ~B1_MAP_MASK) != 0U)
    {
        return false;
    }
    for (i = 0U; i < B1_MAP_SLOTS; i++)
    {
        uint32_t index = (word >> (5U * i)) & 31U;

        if ((index >= 26U) || ((seen & (1UL << index)) != 0U))
        {
            return false;
        }
        seen |= 1UL << index;
    }
    return true;
}

bool
b1_mapping_encode (uint8_t const p_letters[B1_MAP_SLOTS], uint32_t * p_word)
{
    uint32_t word = 0U;
    uint32_t i;

    for (i = 0U; i < B1_MAP_SLOTS; i++)
    {
        if ((p_letters[i] < (uint8_t)'A') || (p_letters[i] > (uint8_t)'Z'))
        {
            return false;
        }
        word |= (uint32_t)(p_letters[i] - (uint8_t)'A') << (5U * i);
    }
    if (!b1_mapping_check(word))
    {
        return false;
    }
    *p_word = word;
    return true;
}

uint32_t
b1_mapping_replace (uint32_t snapshot, uint32_t word)
{
    uint32_t revision;

    if (!b1_mapping_check(word))
    {
        return snapshot;
    }
    if (((snapshot & B1_MAP_CONFIGURED) != 0U) &&
        ((snapshot & B1_MAP_MASK) == word))
    {
        return snapshot;
    }
    revision = ((snapshot >> B1_MAP_REV_SHIFT) + 1U) & 0xFFU;
    return word | B1_MAP_CONFIGURED | (revision << B1_MAP_REV_SHIFT);
}

b1_movement_t
b1_mapping_lookup (uint32_t snapshot, uint8_t letter)
{
    uint32_t i;
    uint32_t word = snapshot & B1_MAP_MASK;

    if (((snapshot & B1_MAP_CONFIGURED) == 0U) || !b1_mapping_check(word) ||
        ((snapshot & ~B1_MAP_STATUS_MASK) != 0U) ||
        (letter < (uint8_t)'A') || (letter > (uint8_t)'Z'))
    {
        return B1_MOVE_NONE;
    }
    for (i = 0U; i < B1_MAP_SLOTS; i++)
    {
        if (((word >> (5U * i)) & 31U) == (uint32_t)(letter - (uint8_t)'A'))
        {
            return (b1_movement_t)(i + 1U);
        }
    }
    return B1_MOVE_NONE;
}

void
b1_mapping_frame (uint32_t snapshot, uint8_t request_id, uint8_t seq,
                   uint8_t p_frame[B1_MAP_FRAME_BYTES])
{
    tlm_put_header(p_frame, TLM_SRC_BARCODE_MAP, B1_MAP_PAYLOAD_BYTES, seq);
    sys_put_le32(&p_frame[TLM_HDR_BYTES], snapshot & B1_MAP_STATUS_MASK);
    p_frame[TLM_HDR_BYTES + 4U] = request_id;
}

bool
b1_mapping_readback (uint8_t const * p_frame, uint32_t len,
                     uint32_t * p_snapshot, uint8_t * p_request_id)
{
    uint8_t seq;
    uint32_t snapshot;

    if (!tlm_check_header(p_frame, len, TLM_SRC_BARCODE_MAP,
                          B1_MAP_PAYLOAD_BYTES, &seq))
    {
        return false;
    }
    snapshot = sys_get_le32(&p_frame[TLM_HDR_BYTES]);
    if (((snapshot & ~B1_MAP_STATUS_MASK) != 0U) ||
        (((snapshot & B1_MAP_CONFIGURED) != 0U) ?
          !b1_mapping_check(snapshot & B1_MAP_MASK) : (snapshot != 0U)))
    {
        return false;
    }
    *p_snapshot = snapshot;
    *p_request_id = p_frame[TLM_HDR_BYTES + 4U];
    return true;
}
