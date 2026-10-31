#ifndef BUDDY_NAV_CODEC_H
#define BUDDY_NAV_CODEC_H
#include "src/motion/motion_port.h"
enum { NAV_PACKET_BYTES = 8, NAV_EVENT_BYTES = 31 };
typedef enum {
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
    NAV_RAMP_ASSIST
} NavKind;
typedef struct {
    unsigned kind : 4, owner : 3, style : 1, flag : 1, speed : 7, bitmap : 10;
    uint32_t lease;
    float value;
    int16_t wheels[2];
    MotionRampAssist assist;
} NavRequest;
/* Canonical little-bit-order binary, independent of compiler bitfield layout.
 * Request length is exact per kind; unused final-byte bits must be zero. */
unsigned nav_packet_encode(uint8_t out[NAV_PACKET_BYTES], const NavRequest *request);
bool nav_packet_decode(NavRequest *request, const uint8_t *packet, unsigned bytes);
/* Log quantisation only: goal0.1unit, progress0.1mm signed24, yaw0.1deg signed16.
 * Overflow is explicit; control/command result float calculations are untouched. */
void nav_event_encode(uint8_t out[NAV_EVENT_BYTES], const MotionEvent *event);
bool nav_event_decode(MotionEvent *event, const uint8_t in[NAV_EVENT_BYTES], bool *overflow);
#endif
