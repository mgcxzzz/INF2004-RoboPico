#include <assert.h>
#include <stdio.h>

#include "app_program/network_bridge/connected_policy.h"
#include "app_program/network_bridge/network_stop.h"

static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

int main(void) {
    const uint32_t now = 5000U;
    RnStatus online = {.polls = 1U,
                       .last_poll_ms = now - 250U,
                       .connects = 17U,
                       .last_complete_ms = now - 3000U,
                       .link = 3,
                       .flags = RN_CONFIGURED | RN_CONNECTED | RN_SUBSCRIBED};
    assert(rn_link_usable(&online, now));
    assert(rn_delivery_fresh(&online, now, online.last_complete_ms));
    assert(rn_network_stop_failure_mask(&online, now, now - 250U,
                                        online.last_complete_ms) == 0U);
    RnStatus wrapped = online;
    wrapped.last_poll_ms = UINT32_MAX - 239U;
    wrapped.last_complete_ms = UINT32_MAX - 2989U;
    assert(rn_link_usable(&wrapped, 10U));
    assert(rn_delivery_fresh(&wrapped, 10U, wrapped.last_complete_ms));
    assert(rn_network_stop_failure_mask(&wrapped, 10U, UINT32_MAX - 239U,
                                        wrapped.last_complete_ms) == 0U);

    RnStatus bad = online;
    bad.link = 2;
    assert(rn_network_stop_failure_mask(&bad, now, now - 250U,
                                        bad.last_complete_ms) == RN_STOP_FAIL_LINK_DOWN);
    bad = online;
    bad.flags &= (uint8_t)~RN_CONNECTED;
    assert(rn_network_stop_failure_mask(&bad, now, now - 250U,
                                        bad.last_complete_ms) == RN_STOP_FAIL_NOT_CONNECTED);
    bad = online;
    bad.flags &= (uint8_t)~RN_SUBSCRIBED;
    assert(rn_network_stop_failure_mask(&bad, now, now - 250U,
                                        bad.last_complete_ms) == RN_STOP_FAIL_NOT_SUBSCRIBED);
    bad = online;
    bad.last_poll_ms = now - 251U;
    assert(!rn_link_usable(&bad, now));
    assert(rn_network_stop_failure_mask(&bad, now, now - 250U,
                                        bad.last_complete_ms) == RN_STOP_FAIL_POLL_STALE);
    bad = online;
    bad.polls = 0U;
    bad.last_poll_ms = now;
    assert(!rn_link_usable(&bad, now));
    assert(rn_network_stop_failure_mask(&bad, now, now - 250U,
                                        bad.last_complete_ms) == RN_STOP_FAIL_POLL_STALE);
    assert(rn_network_stop_failure_mask(&online, now, now - 251U,
                                        online.last_complete_ms) == RN_STOP_FAIL_APP_STALE);
    bad = online;
    bad.last_complete_ms = now - 3001U;
    assert(!rn_delivery_fresh(&bad, now, bad.last_complete_ms));
    assert((rn_network_stop_failure_mask(&bad, now, now - 250U,
                                         bad.last_complete_ms) & RN_STOP_FAIL_ACK_STALE) != 0U);
    assert((rn_network_stop_failure_mask(&online, now, now - 250U, now) &
            RN_STOP_FAIL_ACK_BEFORE_CONNECT) != 0U);

    bad = (RnStatus){.polls = 0U,
                     .last_poll_ms = now - 251U,
                     .connects = 0x12345678U,
                     .last_complete_ms = now - 3001U,
                     .link = -2,
                     .flags = 0U};
    uint8_t expected_failures = RN_STOP_FAIL_LINK_DOWN | RN_STOP_FAIL_NOT_CONNECTED |
                                RN_STOP_FAIL_NOT_SUBSCRIBED | RN_STOP_FAIL_POLL_STALE |
                                RN_STOP_FAIL_APP_STALE | RN_STOP_FAIL_ACK_STALE |
                                RN_STOP_FAIL_ACK_BEFORE_CONNECT;
    assert(rn_network_stop_failure_mask(&bad, now, now - 251U, now) == expected_failures);

    RnNetworkStopSnapshot stored = {0};
    RnNetworkStopSnapshot first = rn_network_stop_candidate(
        &online, now, now - 250U, online.last_complete_ms);
    assert(first.valid && first.failmask == 0U && first.poll_age_ms == 250U);
    assert(first.app_age_ms == 250U && first.ack_age_ms == 3000U);
    assert(rn_network_stop_capture_once(&stored, &first));
    RnNetworkStopSnapshot later = rn_network_stop_candidate(&bad, now, now - 251U, now);
    assert(!rn_network_stop_capture_once(&stored, &later));
    assert(stored.stop_ms == now && stored.connects == 17U && stored.failmask == 0U);

    uint8_t packet[RN_NETWORK_STOP_PACKET_BYTES];
    rn_network_stop_encode(&stored, 0x5aU, packet);
    assert(sizeof(packet) == 32U && packet[0] == 0xf2U && packet[1] == 29U);
    assert(packet[2] == 0x5aU && packet[3] == 0xc6U && packet[4] == 0x21U && packet[5] == 0U);
    assert(read32(packet + 6U) == now && read32(packet + 10U) == 250U);
    assert(read32(packet + 14U) == 250U && read32(packet + 18U) == 3000U);
    assert(read32(packet + 22U) == 17U && read32(packet + 26U) == now - 3000U);
    uint16_t metadata = (uint16_t)packet[30] | ((uint16_t)packet[31] << 8U);
    assert(metadata == 0x201fU); /* flags=7, signed3 link=+3, valid=1 */

    RnNetworkStopSnapshot signed_link = rn_network_stop_candidate(&bad, now, now, now);
    signed_link.valid = 1U;
    rn_network_stop_encode(&signed_link, 0U, packet);
    metadata = (uint16_t)packet[30] | ((uint16_t)packet[31] << 8U);
    assert(((metadata >> 3U) & 7U) == 6U); /* signed3 -2 */
    assert((metadata & 0xc000U) == 0U);

    FILE *fixture = fopen("testing/rtos_migration/build-host/network_stop.bin", "wb");
    assert(fixture && fwrite(packet,1,sizeof(packet),fixture)==sizeof(packet)); fclose(fixture);
    rn_network_stop_clear(&stored);
    assert(!stored.valid && stored.stop_ms == 0U);
    rn_network_stop_encode(&stored, 0xa5U, packet);
    assert(packet[0] == 0xf2U && packet[1] == 29U && packet[2] == 0xa5U &&
           packet[3] == 0xc6U && packet[4] == 0x21U);
    for (unsigned i = 5U; i < sizeof(packet); ++i)
        assert(packet[i] == 0U);
    assert(rn_network_stop_capture_once(&stored, &later));
    assert(stored.valid);

    RnStatus no_polls = online;
    no_polls.polls = 0U;
    RnNetworkStopSnapshot no_poll = rn_network_stop_candidate(&no_polls, now, now, now);
    assert(no_poll.poll_age_ms == UINT32_MAX);

    puts("PASS network stop snapshot: guard boundaries, failure mask, latch, C6 codec and clear");
    return 0;
}
