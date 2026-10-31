#ifndef BARCODE_START_H
#define BARCODE_START_H
#include "src/line_following/line_logic.h"

/* Explicit-start checks for this test only. Optical placement is interpreted
 * by the running follower, not used as permission to begin a test. Keep the
 * same non-optical checks and priority as profile_start_block(profile5). */
static inline unsigned barcode_start_block(bool active, bool settling, bool valid, bool stop,
                                           bool ready, bool hold, unsigned mode) {
    if (active)
        return START_ACTIVE;
    if (settling)
        return START_SETTLING;
    if (!valid)
        return START_CAL;
    if (stop)
        return START_GP20;
    if (!ready)
        return START_REARM;
    if (mode == LINE_USB_HOLD && !hold)
        return START_HOLD_GP21;
    if (mode != LINE_USB_HOLD && hold)
        return START_RELEASE_GP21;
    return START_OK;
}
#endif
