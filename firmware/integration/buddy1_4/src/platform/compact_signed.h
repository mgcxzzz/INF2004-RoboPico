/**
 * @file compact_signed.h
 * @brief Explicit signed interpretation of compact unsigned storage.
 * @details Native field layout is never a wire format. Callers validate or
 * clamp mathematical values before packing; the conversion deliberately retains
 * the low width bits, including two's-complement negative encodings.
 */
#ifndef ROBOPICO_COMPACT_SIGNED_H
#define ROBOPICO_COMPACT_SIGNED_H

#include <stdint.h>

/**
 * @brief Retain the low width bits of a signed value, without signed shifts.
 * @param value Mathematical signed value; out-of-domain values wrap modulo
 * 2^width.
 * @param width Compact field width, 1 through 31; invalid widths return zero.
 * @return Canonical unsigned bit pattern, independent of compiler bit-field
 * order.
 */
static inline uint32_t compact_signed_pack (int32_t value, unsigned width)
{
    if ((0U == width) || (width > 31U))
    {
        return 0U;
    }
    /* C defines signed-to-unsigned conversion modulo 2^32. Masking then keeps
     * exactly the caller's field width; no signed representation is inspected.
     */
    return ((uint32_t)value) & ((UINT32_C(1) << width) - 1U);
}

/**
 * @brief Interpret a compact unsigned field as a mathematical signed value.
 * @param bits Unsigned field; bits above width are ignored.
 * @param width Field width, 1 through 31; invalid widths return zero.
 * @return Signed result in [-2^(width-1), 2^(width-1)-1].
 */
static inline int32_t compact_signed_unpack (uint32_t bits, unsigned width)
{
    if ((0U == width) || (width > 31U))
    {
        return 0;
    }
    const uint32_t sign = UINT32_C(1) << (width - 1U);
    /* Each masked operand is <= 2^30 and fits int32_t. Their difference is in
     * [-2^30, 2^30-1], so both casts and the subtraction are representable. */
    return (int32_t)(bits & (sign - 1U)) - (int32_t)(bits & sign);
}

#endif /* ROBOPICO_COMPACT_SIGNED_H */

/* End of file. */
