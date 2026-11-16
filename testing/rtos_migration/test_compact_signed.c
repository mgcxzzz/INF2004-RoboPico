/* Boundary/ABI regression for the Barr cleanup's representation changes. */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "src/platform/compact_signed.h"
#include "src/imu/imu_lift_guard.h"
#include "src/line_following/line_search.h"
#include "app_program/imu_navigation/imu_lift_packet.h"
#include "app_program/line_navigation/drive_diagnostics.h"
#include "app_program/network_bridge/network_stop.h"
#include "reference/native_layout_v026.h"

#define SAME_LAYOUT(CURRENT, BASELINE) \
    _Static_assert(sizeof(CURRENT) == sizeof(BASELINE), "native size changed"); \
    _Static_assert(_Alignof(CURRENT) == _Alignof(BASELINE), "native alignment changed")
SAME_LAYOUT(ri_lift_packet_t, baseline_lift_packet_t);
SAME_LAYOUT(imu_lift_guard_t, baseline_lift_guard_t);
SAME_LAYOUT(rl_drive_diagnostic_t, baseline_drive_diag_t);
SAME_LAYOUT(rn_network_stop_snapshot_t, baseline_network_stop_t);
SAME_LAYOUT(line_search_t, baseline_line_search_t);

static void check_width(unsigned width) {
    const int32_t half = (int32_t)(UINT32_C(1) << (width - 1U));
    const uint32_t mask = (UINT32_C(1) << width) - 1U;
    for (int32_t value = -half; value < half; ++value) {
        const uint32_t bits = compact_signed_pack(value, width);
        assert(bits <= mask);
        assert(compact_signed_unpack(bits, width) == value);
        assert(compact_signed_unpack(bits | ~mask, width) == value);
    }
}

int main(void) {
    check_width(2U);
    check_width(3U);
    check_width(9U);
    check_width(13U);
    assert(compact_signed_unpack(compact_signed_pack(-(INT32_C(1) << 30), 31U), 31U)
           == -(INT32_C(1) << 30));
    assert(compact_signed_unpack(compact_signed_pack((INT32_C(1) << 30) - 1, 31U), 31U)
           == (INT32_C(1) << 30) - 1);
    assert(compact_signed_pack(INT32_MIN, 0U) == 0U);
    assert(compact_signed_unpack(UINT32_MAX, 32U) == 0);

    for (int32_t value = -4096; value <= 4095; ++value) {
        imu_lift_guard_t guard = {0};
        guard.up_projection_bits = compact_signed_pack(value, 13U);
        assert(imu_lift_projection(&guard) == value);
        ri_lift_packet_t packet = {.up_bits = compact_signed_pack(value, 13U)};
        uint8_t bytes[RI_LIFT_PACKET_BYTES];
        ri_lift_packet_encode(&packet, 0, 0, 0, bytes);
        assert(((uint32_t)bytes[19] | ((uint32_t)bytes[20] << 8U))
               == compact_signed_pack(value, 13U));
    }
    line_search_t search = {0};
    assert(line_search_direction(&search) == 0);
    search.first_bits = compact_signed_pack(-1, 2U);
    assert(line_search_direction(&search) == -1);
    search.first_bits = compact_signed_pack(-line_search_direction(&search), 2U);
    assert(line_search_direction(&search) == 1);
    printf("PASS compact signed ranges and five native layouts: %zu/%zu/%zu/%zu/%zu bytes\n",
           sizeof(ri_lift_packet_t), sizeof(imu_lift_guard_t), sizeof(rl_drive_diagnostic_t),
           sizeof(rn_network_stop_snapshot_t), sizeof(line_search_t));
    return 0;
}
