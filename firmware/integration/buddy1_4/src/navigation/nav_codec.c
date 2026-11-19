/**
 * @file nav_codec.c
 * @brief Nav codec.
 * @details The shared facade/coordinator arbitrates ownership and generations.
 * Acceptance is not completion; external callers never write motor registers.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "src/navigation/nav_codec.h"
#include <math.h>
#include <string.h>
static void put (uint8_t * p_data, unsigned * p_at_value, uint64_t value,
                 unsigned bits)
{
    for (unsigned item_index = 0; item_index < bits;
         ++item_index, ++*p_at_value)
    {
        if ((value >> item_index) & 1)
        {
            p_data[*p_at_value / 8] |= (uint8_t)(1u << (*p_at_value % 8));
        }
    }
}
static uint64_t get (const uint8_t * p_data, unsigned * p_at_value,
                     unsigned bits)
{
    uint64_t value = 0;
    for (unsigned item_index = 0; item_index < bits;
         ++item_index, ++*p_at_value)
    {
        if ((p_data[*p_at_value / 8] >> (*p_at_value % 8)) & 1)
        {
            value |= UINT64_C(1) << item_index;
        }
    }
    return value;
}
static unsigned payload_bits (unsigned kind)
{
    static const unsigned bits[] = {0, 0,  53, 52, 33,
#ifdef BUDDY_RUNTIME_MAPPING
                                    5,
#else
                                    2,
#endif
                                    2, 18, 5,  11, 5,  39, 2,
#ifdef MOTION_STEERED_DRIVE
                                    40
#endif
    };
    return kind <= NAV_KIND_LAST ? bits[kind] : UINT32_MAX;
}
unsigned nav_packet_encode (uint8_t out[NAV_PACKET_BYTES],
                            const nav_request_t * p_result)
{
    if (((((!p_result) || (p_result->kind > NAV_KIND_LAST)) ||
          (p_result->owner < 1)) ||
         (p_result->owner > 5)) ||
        (!isfinite(p_result->value)))
    {
        return 0;
    }
    memset(out, 0, NAV_PACKET_BYTES);
    unsigned at_value = 0;
    put(out, &at_value, p_result->kind, 4);
    put(out, &at_value, p_result->owner, 3);
    if (NAV_DRIVE == p_result->kind || NAV_TURN == p_result->kind ||
        NAV_RESUME == p_result->kind
#ifdef MOTION_STEERED_DRIVE
        || NAV_STEER == p_result->kind
#endif
    )
    {
        put(out, &at_value, p_result->lease, 32);
    }
    switch (p_result->kind)
    {
    case NAV_DRIVE:
        if (((fabsf(p_result->value) < 1) || (fabsf(p_result->value) > 1000)) ||
            ((p_result->speed) &&
             ((p_result->speed < 20) || (p_result->speed > 60))))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)(lroundf(p_result->value * 10) + 10000),
            15);
        put(out, &at_value, p_result->speed, 6);
        break;
    case NAV_TURN:
        if (((fabsf(p_result->value) < 1) || (fabsf(p_result->value) > 180)) ||
            ((p_result->speed) &&
             ((p_result->speed < 20) || (p_result->speed > 90))))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)(lroundf(p_result->value * 10) + 1800),
            12);
        put(out, &at_value, p_result->speed, 7);
        put(out, &at_value, p_result->style, 1);
        break;
    case NAV_RESUME:
        put(out, &at_value, p_result->flag, 1);
        break;
    case NAV_ROUTE:
#ifdef BUDDY_RUNTIME_MAPPING
        if (((p_result->value < 0) || (p_result->value > 25)) ||
            (floorf(p_result->value) != p_result->value))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)p_result->value, 5);
        break;
#else
        if (p_result->value != 0 && p_result->value != 1 &&
            p_result->value != 2)
        {
            return 0;
        }
        put(out, &at_value, (unsigned)p_result->value, 2);
        break;
#endif
    case NAV_TERRAIN:
        if (((p_result->value < 0) || (p_result->value > 3)) ||
            (floorf(p_result->value) != p_result->value))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)p_result->value, 2);
        break;
    case NAV_WHEELS:
        if ((((p_result->wheels[0] < -180) || (p_result->wheels[0] > 180)) ||
             (p_result->wheels[1] < -180)) ||
            (p_result->wheels[1] > 180))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)(p_result->wheels[0] + 180), 9);
        put(out, &at_value, (unsigned)(p_result->wheels[1] + 180), 9);
        break;
    case NAV_VERIFIED_BARCODE:
        if (((p_result->value < 0) || (p_result->value > 25)) ||
            (floorf(p_result->value) != p_result->value))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)p_result->value, 5);
        break;
    case NAV_RAMP_ASSIST:
        if (((((p_result->assist.boost_milli > 200) ||
               (p_result->assist.push_pwm_x100 > 6000)) ||
              (p_result->assist.hold_milli > 1000)) ||
             (!p_result->assist.ttl_ms)) ||
            (p_result->assist.ttl_ms > 200))
        {
            return 0;
        }
        put(out, &at_value, p_result->assist.boost_milli, 8);
        put(out, &at_value, p_result->assist.push_pwm_x100, 13);
        put(out, &at_value, p_result->assist.hold_milli, 10);
        put(out, &at_value, p_result->assist.ttl_ms, 8);
        break;
    case NAV_ASSIST:
        if (((p_result->value < 0) || (p_result->value > 20)) ||
            (floorf(p_result->value) != p_result->value))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)p_result->value, 5);
        break;
    case NAV_FAULT_REPORT:
        put(out, &at_value, p_result->bitmap, 10);
        put(out, &at_value, p_result->flag, 1);
        break;
    case NAV_SPEED_MODE:
        if (p_result->speed > DRIVE_MODE_NONE)
        {
            return 0;
        }
        put(out, &at_value, p_result->speed, 2);
        break;
#ifdef MOTION_STEERED_DRIVE
    case NAV_STEER:
    {
        long decideg = lroundf(p_result->value * 10);
        if ((decideg < -NAV_STEER_MAX_DECIDEG) ||
            (decideg > NAV_STEER_MAX_DECIDEG))
        {
            return 0;
        }
        put(out, &at_value, (unsigned)(decideg + 128), 8);
        break;
    }
#endif
    default:
        break;
    }
    return (at_value + 7) / 8;
}
bool nav_packet_decode (nav_request_t * p_result, const uint8_t * p_data,
                        unsigned bytes)
{
    if ((((!p_result) || (!p_data)) || (!bytes)) || (bytes > NAV_PACKET_BYTES))
    {
        return false;
    }
    memset(p_result, 0, sizeof(*p_result));
    unsigned at_value = 0;
    p_result->kind = (unsigned)get(p_data, &at_value, 4);
    p_result->owner = (unsigned)get(p_data, &at_value, 3);
    if (((p_result->kind > NAV_KIND_LAST) || (p_result->owner < 1)) ||
        (p_result->owner > 5))
    {
        return false;
    }
    unsigned bits = 7 + payload_bits(p_result->kind);
    if (bytes != (bits + 7) / 8)
    {
        return false;
    }
    for (unsigned bit = bits; bit < bytes * 8; ++bit)
    {
        if ((p_data[bit / 8] >> (bit % 8)) & 1)
        {
            return false;
        }
    }
    if (NAV_DRIVE == p_result->kind || NAV_TURN == p_result->kind ||
        NAV_RESUME == p_result->kind
#ifdef MOTION_STEERED_DRIVE
        || NAV_STEER == p_result->kind
#endif
    )
    {
        p_result->lease = (uint32_t)get(p_data, &at_value, 32);
    }
    switch (p_result->kind)
    {
    case NAV_DRIVE:
        p_result->value = ((int)get(p_data, &at_value, 15) - 10000) / 10.0f;
        p_result->speed = (unsigned)get(p_data, &at_value, 6);
        break;
    case NAV_TURN:
        p_result->value = ((int)get(p_data, &at_value, 12) - 1800) / 10.0f;
        p_result->speed = (unsigned)get(p_data, &at_value, 7);
        p_result->style = (unsigned)get(p_data, &at_value, 1);
        break;
    case NAV_RESUME:
        p_result->flag = (unsigned)get(p_data, &at_value, 1);
        break;
    case NAV_ROUTE:
#ifdef BUDDY_RUNTIME_MAPPING
        p_result->value = (float)get(p_data, &at_value, 5);
        break;
#else
        p_result->value = (float)get(p_data, &at_value, 2);
        break;
#endif
    case NAV_TERRAIN:
        p_result->value = (float)get(p_data, &at_value, 2);
        break;
    case NAV_WHEELS:
        p_result->wheels[0] = (int16_t)((int)get(p_data, &at_value, 9) - 180);
        p_result->wheels[1] = (int16_t)((int)get(p_data, &at_value, 9) - 180);
        break;
    case NAV_VERIFIED_BARCODE:
        p_result->value = (float)get(p_data, &at_value, 5);
        break;
    case NAV_RAMP_ASSIST:
        p_result->assist.boost_milli = (uint16_t)get(p_data, &at_value, 8);
        p_result->assist.push_pwm_x100 = (uint16_t)get(p_data, &at_value, 13);
        p_result->assist.hold_milli = (uint16_t)get(p_data, &at_value, 10);
        p_result->assist.ttl_ms = (uint8_t)get(p_data, &at_value, 8);
        break;
    case NAV_ASSIST:
        p_result->value = (float)get(p_data, &at_value, 5);
        break;
    case NAV_FAULT_REPORT:
        p_result->bitmap = (unsigned)get(p_data, &at_value, 10);
        p_result->flag = (unsigned)get(p_data, &at_value, 1);
        break;
    case NAV_SPEED_MODE:
        p_result->speed = (unsigned)get(p_data, &at_value, 2);
        break;
#ifdef MOTION_STEERED_DRIVE
    case NAV_STEER:
        p_result->value = ((int)get(p_data, &at_value, 8) - 128) / 10.0f;
        break;
#endif
    default:
        break;
    }
    uint8_t canonical[NAV_PACKET_BYTES];
    unsigned count = nav_packet_encode(canonical, p_result);
    return (count == bytes) && (!memcmp(p_data, canonical, count));
}
static int32_t quant (float value, float scale, int32_t low, int32_t high,
                      bool * p_overflow)
{
    double value_x = (double)value * scale;
    if (!isfinite(value_x))
    {
        *p_overflow = true;
        return 0;
    }
    if (value_x < low)
    {
        *p_overflow = true;
        return low;
    }
    if (value_x > high)
    {
        *p_overflow = true;
        return high;
    }
    return (int32_t)llround(value_x);
}
void nav_event_encode (uint8_t out[NAV_EVENT_BYTES],
                       const motion_event_t * p_event_value)
{
    memset(out, 0, NAV_EVENT_BYTES);
    unsigned at_value = 0;
    bool overflow = false;
    put(out, &at_value, p_event_value->now_us, 64);
    put(out, &at_value, p_event_value->sequence, 32);
    put(out, &at_value, p_event_value->id, 32);
    put(out, &at_value, p_event_value->requester, 3);
    put(out, &at_value, p_event_value->owner, 3);
    put(out, &at_value, p_event_value->type, 4);
    put(out, &at_value, p_event_value->kind, 4);
    put(out, &at_value, p_event_value->state, 3);
    put(out, &at_value, p_event_value->fault, 10);
    put(out, &at_value, p_event_value->result, 4);
    put(out, &at_value, p_event_value->style, 1);
    put(out, &at_value, p_event_value->speed, 7);
    put(out, &at_value,
        (uint32_t)(quant(p_event_value->goal, 10, -10000, 10230, &overflow) +
                   10000),
        15);
    put(out, &at_value,
        (uint32_t)(quant(p_event_value->progress_mm, 10, -8388608, 8388607,
                         &overflow) +
                   8388608),
        24);
    put(out, &at_value,
        (uint32_t)(quant(p_event_value->rotation_deg, 10, -32768, 32767,
                         &overflow) +
                   32768),
        16);
    put(out, &at_value, overflow, 1);
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        put(out, &at_value,
            (uint32_t)(quant(p_event_value->target_mm_s[wheel], 1, -180, 180,
                             &overflow) +
                       180),
            9);
    }
    if (overflow)
    {
        out[222 / 8] |= (uint8_t)(1u << (222 % 8));
    }
}
bool nav_event_decode (motion_event_t * p_event_value,
                       const uint8_t in_value_value[NAV_EVENT_BYTES],
                       bool * p_overflow_value)
{
    if ((!p_event_value) || (!in_value_value))
    {
        return false;
    }
    memset(p_event_value, 0, sizeof(*p_event_value));
    unsigned at_value = 0;
    p_event_value->now_us = get(in_value_value, &at_value, 64);
    p_event_value->sequence = (uint32_t)get(in_value_value, &at_value, 32);
    p_event_value->id = (uint32_t)get(in_value_value, &at_value, 32);
    p_event_value->requester = (unsigned)get(in_value_value, &at_value, 3);
    p_event_value->owner = (unsigned)get(in_value_value, &at_value, 3);
    p_event_value->type = (unsigned)get(in_value_value, &at_value, 4);
    p_event_value->kind = (unsigned)get(in_value_value, &at_value, 4);
    p_event_value->state = (unsigned)get(in_value_value, &at_value, 3);
    p_event_value->fault = (unsigned)get(in_value_value, &at_value, 10);
    p_event_value->result = (unsigned)get(in_value_value, &at_value, 4);
    p_event_value->style = (unsigned)get(in_value_value, &at_value, 1);
    p_event_value->speed = (unsigned)get(in_value_value, &at_value, 7);
    p_event_value->goal =
        ((int)get(in_value_value, &at_value, 15) - 10000) / 10.0f;
    p_event_value->progress_mm =
        ((int)get(in_value_value, &at_value, 24) - 8388608) / 10.0f;
    p_event_value->rotation_deg =
        ((int)get(in_value_value, &at_value, 16) - 32768) / 10.0f;
    bool clipped = get(in_value_value, &at_value, 1) != 0;
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        p_event_value->target_mm_s[wheel] =
            (int16_t)((int)get(in_value_value, &at_value, 9) - 180);
    }
    if (p_overflow_value)
    {
        *p_overflow_value = clipped;
    }
    for (unsigned bit = at_value; bit < NAV_EVENT_BYTES * 8; ++bit)
    {
        if ((in_value_value[bit / 8] >> (bit % 8)) & 1)
        {
            return false;
        }
    }
    return (((((((((p_event_value->requester <= 5) &&
                   (p_event_value->owner <= 5)) &&
                  (p_event_value->type <= MOTION_EVENT_SESSION_END)) &&
                 (p_event_value->kind <= 11)) &&
                (p_event_value->state <= MOTION_CMD_FAILED)) &&
               (p_event_value->result <= MOTION_NEED_RECOVERY)) &&
              (p_event_value->target_mm_s[0] >= -180)) &&
             (p_event_value->target_mm_s[0] <= 180)) &&
            (p_event_value->target_mm_s[1] >= -180)) &&
           (p_event_value->target_mm_s[1] <= 180);
}

/* End of file. */
