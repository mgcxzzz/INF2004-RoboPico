/**
 * @file    s3_barcode.c
 * @brief   Code 39 decoding from measured bar and space widths.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include "s3_barcode.h"
#include "s3_config.h"

/* Code 39: characters and their 9-element patterns (bit 8 = first bar,
 * 1 = wide). Each has exactly three wide elements. */
static char const g_c39_chars[S3_C39_CHARS] =
{
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z', '-', '.', ' ', '$',
    '/', '+', '%', '*'
};

static uint16_t const g_c39_patterns[S3_C39_CHARS] =
{
    0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U, 0x070U, 0x025U,
    0x124U, 0x064U, 0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U,
    0x00DU, 0x10CU, 0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U,
    0x112U, 0x052U, 0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U,
    0x1C0U, 0x091U, 0x190U, 0x0D0U, 0x085U, 0x184U, 0x0C4U, 0x0A8U,
    0x0A2U, 0x08AU, 0x02AU, 0x094U
};

uint32_t
s3_c39_index (char c)
{
    uint32_t i;

    for (i = 0U; i < S3_C39_CHARS; i++)
    {
        if (g_c39_chars[i] == c)
        {
            return i;
        }
    }

    return S3_C39_NONE;
}

char
s3_c39_at (uint32_t index)
{
    return (index < S3_C39_CHARS) ? g_c39_chars[index] : '?';
}

uint32_t
s3_c39_pattern (char c)
{
    uint32_t i = s3_c39_index(c);

    return (i < S3_C39_CHARS) ? (uint32_t)g_c39_patterns[i] : 0U;
}

char
s3_c39_char (uint32_t pattern)
{
    uint32_t i;

    for (i = 0U; i < S3_C39_CHARS; i++)
    {
        if ((uint32_t)g_c39_patterns[i] == pattern)
        {
            return g_c39_chars[i];
        }
    }

    return '\0';
}

/*
 * Classify one 9-element group: the three widest are wide. Reject the
 * group unless the narrowest wide is clearly wider than the widest narrow
 * (ratio >= min_ratio_x10 / 10), so a smudged print never guesses.
 * Returns the 9-bit pattern, or 0 on a bad ratio.
 */
static uint32_t
classify (uint32_t const * p_w, uint32_t min_ratio_x10)
{
    uint32_t wide_mask = 0U;
    uint32_t min_wide  = 0xFFFFFFFFU;
    uint32_t max_narrow = 0U;
    uint32_t k;
    uint32_t i;

    for (k = 0U; k < 3U; k++)
    {
        uint32_t best = S3_C39_ELEMENTS;

        for (i = 0U; i < S3_C39_ELEMENTS; i++)
        {
            if (((wide_mask & (1UL << i)) == 0U) &&
                ((best == S3_C39_ELEMENTS) || (p_w[i] > p_w[best])))
            {
                best = i;
            }
        }

        wide_mask |= (1UL << best);
        min_wide   = (p_w[best] < min_wide) ? p_w[best] : min_wide;
    }

    for (i = 0U; i < S3_C39_ELEMENTS; i++)
    {
        if (((wide_mask & (1UL << i)) == 0U) && (p_w[i] > max_narrow))
        {
            max_narrow = p_w[i];
        }
    }

    if ((uint64_t)min_wide * 10U < (uint64_t)max_narrow * min_ratio_x10)
    {
        return 0U;
    }

    /* Element 0 (the first bar) is bit 8. */
    {
        uint32_t pattern = 0U;

        for (i = 0U; i < S3_C39_ELEMENTS; i++)
        {
            if ((wide_mask & (1UL << i)) != 0U)
            {
                pattern |= 1UL << (S3_C39_ELEMENTS - 1U - i);
            }
        }

        return pattern;
    }
}

/* Decode in one direction. p_w may be read forwards or backwards. */
static s3_bc_result_t
decode_dir (uint32_t const * p_w, uint32_t n, bool b_reverse,
            char * p_text, uint32_t text_max)
{
    uint32_t chars = (n + 1U) / 10U;
    uint32_t c;
    uint32_t out = 0U;
    uint32_t group[S3_C39_ELEMENTS];

    for (c = 0U; c < chars; c++)
    {
        uint32_t i;
        uint32_t pattern;
        char     ch;

        for (i = 0U; i < S3_C39_ELEMENTS; i++)
        {
            uint32_t at = (10U * c) + i;

            group[i] = b_reverse ? p_w[n - 1U - at] : p_w[at];
        }

        pattern = classify(group, S3_BC_MIN_RATIO_X10);

        if (pattern == 0U)
        {
            return S3_BC_BAD_RATIO;
        }

        ch = s3_c39_char(pattern);

        if (ch == '\0')
        {
            return S3_BC_BAD_CHAR;
        }

        if ((c == 0U) || (c == (chars - 1U)))
        {
            if (ch != '*')
            {
                return S3_BC_NO_START_STOP;
            }
        }
        else if (ch == '*')
        {
            return S3_BC_NO_START_STOP;     /* '*' only at the ends */
        }
        else if (out < (text_max - 1U))
        {
            p_text[out++] = ch;
        }
        else
        {
            return S3_BC_BAD_LENGTH;
        }
    }

    p_text[out] = '\0';
    return S3_BC_OK;
}

s3_bc_result_t
s3_bc_decode (uint32_t const * p_w, uint32_t n, char * p_text,
              uint32_t text_max)
{
    s3_bc_result_t res;
    char           scratch[8];

    if ((text_max == 0U) || (p_text == (char *)0))
    {
        return S3_BC_BAD_LENGTH;
    }

    p_text[0] = '\0';

    /* 10 x chars - 1 elements, at least "*X*" (3 chars). */
    if ((n < 29U) || (((n + 1U) % 10U) != 0U))
    {
        return S3_BC_BAD_LENGTH;
    }

    res = decode_dir(p_w, n, false, p_text, text_max);

    if (res == S3_BC_OK)
    {
        return S3_BC_OK;
    }

    /* Read backwards? Code 39 is not symmetric, so a reversed barcode
     * fails forwards but decodes in reverse. Report it, give no text. */
    if (decode_dir(p_w, n, true, scratch, sizeof(scratch)) == S3_BC_OK)
    {
        p_text[0] = '\0';
        return S3_BC_BACKWARDS;
    }

    p_text[0] = '\0';
    return res;
}

uint32_t
s3_bc_encode (char const * p_text, uint32_t narrow, uint32_t wide,
              uint32_t * p_w, uint32_t w_max)
{
    uint32_t n = 0U;
    uint32_t len = 0U;
    uint32_t c;

    while (p_text[len] != '\0')
    {
        len++;
    }

    if (((10U * (len + 2U)) - 1U) > w_max)
    {
        return 0U;
    }

    for (c = 0U; c < (len + 2U); c++)
    {
        char     ch = ((c == 0U) || (c == (len + 1U))) ? '*' : p_text[c - 1U];
        uint32_t pattern = s3_c39_pattern(ch);
        uint32_t i;

        if (pattern == 0U)
        {
            return 0U;
        }

        if (c != 0U)
        {
            p_w[n++] = narrow;              /* inter-character gap */
        }

        for (i = 0U; i < S3_C39_ELEMENTS; i++)
        {
            p_w[n++] = ((pattern & (1UL << (8U - i))) != 0U) ? wide : narrow;
        }
    }

    return n;
}
