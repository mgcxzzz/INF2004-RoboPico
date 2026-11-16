/**
 * @file nav_codec.h
 * @brief Nav codec.
 * @details The shared facade/coordinator arbitrates ownership and generations.
 * Acceptance is not completion; external callers never write motor registers.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_NAV_CODEC_H
#define BUDDY_NAV_CODEC_H
#include "src/motion/motion_port.h"
enum
{
    NAV_PACKET_BYTES = 8,
    NAV_EVENT_BYTES = 31
};
typedef enum
{
    NAV_PAUSE = 0,
    NAV_STOP,
    NAV_DRIVE,
    NAV_TURN,
    NAV_RESUME,
    NAV_ROUTE,
    NAV_TERRAIN,
    NAV_WHEELS,
    NAV_ASSIST,
    NAV_FAULT_REPORT,
    NAV_VERIFIED_BARCODE,
    NAV_RAMP_ASSIST,
    NAV_SPEED_MODE
} nav_kind_t;
typedef struct
{
    unsigned kind : 4;
    unsigned owner : 3;
    unsigned style : 1;
    unsigned flag : 1;
    unsigned speed : 7;
    unsigned bitmap : 10;
    uint32_t lease;
    float value;
    int16_t wheels[2];
    motion_ramp_assist_t assist;
} nav_request_t;
/* Canonical little-bit-order binary, independent of compiler bitfield layout.
 * Request length is exact per kind; unused final-byte bits must be zero. */
unsigned nav_packet_encode (uint8_t out[NAV_PACKET_BYTES],
                            const nav_request_t * p_request);
bool nav_packet_decode (nav_request_t * p_request, const uint8_t * p_packet,
                        unsigned bytes);
/* Log quantisation only: goal0.1unit, progress0.1mm signed24, yaw0.1deg signed16.
 * Overflow is explicit; control/command result float calculations are untouched. */
void nav_event_encode (uint8_t out[NAV_EVENT_BYTES],
                       const motion_event_t * p_event);
bool nav_event_decode (motion_event_t * p_event,
                       const uint8_t in_value[NAV_EVENT_BYTES],
                       bool * p_overflow);
#endif

/* End of file. */
