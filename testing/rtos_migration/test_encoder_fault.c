#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "app_program/rtos_bringup/encoder_fault.h"

static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

static uint32_t read24(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U);
}

static uint64_t read40(const uint8_t *p) {
    uint64_t value = 0U;
    for (unsigned i = 0U; i < 5U; ++i)
        value |= (uint64_t)p[i] << (8U * i);
    return value;
}

static void save_fixture(const char *path, const uint8_t packet[BR_ENCODER_FAULT_PACKET_BYTES]) {
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        perror(path);
        abort();
    }
    if (fwrite(packet, 1U, BR_ENCODER_FAULT_PACKET_BYTES, file) !=
        BR_ENCODER_FAULT_PACKET_BYTES) {
        perror(path);
        (void)fclose(file);
        abort();
    }
    if (fclose(file) != 0) {
        perror(path);
        abort();
    }
}

int main(void) {
    BringupEncoderFaultSnapshot latched = {0};
    BringupEncoderFaultSnapshot first = {
        .kind = BR_ENCODER_FAULT_KIND_DENSITY,
        .wheel = BR_ENCODER_FAULT_WHEEL_RIGHT,
        .control_timer_us = 0x12345678U,
        .kernel_ms = 0x90abcdefU,
        .direction_duration_us = 0U,
        .recent_window_bad = 0x54321U,
        .recent_window_changes = 0xabcdeU,
        .lifetime_edges = {0x01020304U, 0xfedcba98U},
        .lifetime_invalid = {0x11223344U, 0xa1b2c3d4U},
        .optical_target_mm_s = {-45, 70},
        .previous_pwm = {-33, 400},
        .measured_speed_x10 = {-1234, 4321}};
    assert(br_encoder_fault_capture_once(&latched, &first));
    assert(latched.valid && latched.kind == BR_ENCODER_FAULT_KIND_DENSITY);

    BringupEncoderFaultSnapshot later = first;
    later.kind = BR_ENCODER_FAULT_KIND_BURST;
    later.control_timer_us++;
    assert(!br_encoder_fault_capture_once(&latched, &later));
    assert(latched.control_timer_us == first.control_timer_us &&
           latched.kind == BR_ENCODER_FAULT_KIND_DENSITY);

    uint8_t packet[BR_ENCODER_FAULT_PACKET_BYTES];
    br_encoder_fault_encode(&latched, 0x5aU, packet);
    assert(sizeof(packet) == 48U && packet[0] == 0xf2U && packet[1] == 45U);
    assert(packet[2] == 0x5aU && packet[3] == 0xc4U && packet[4] == 0x21U);
    assert(packet[5] == 0x15U); /* valid, density, right wheel; no clamp */
    assert(read32(packet + 6U) == first.control_timer_us);
    assert(read32(packet + 10U) == first.kernel_ms);
    assert(packet[14] == 0U && packet[15] == 0U && packet[16] == 0U);
    assert(read40(packet + 17U) == (uint64_t)0xabcde54321ULL);
    assert(read32(packet + 22U) == first.lifetime_edges[0]);
    assert(read32(packet + 26U) == first.lifetime_invalid[0]);
    assert(read32(packet + 30U) == first.lifetime_edges[1]);
    assert(read32(packet + 34U) == first.lifetime_invalid[1]);
    assert(read24(packet + 38U) == 0x8dd3U); /* signed9: -45, +70 */
    assert(read24(packet + 41U) == 0x0c87dfU); /* signed11: -33, +400 */
    assert((int16_t)((uint16_t)packet[44U] | ((uint16_t)packet[45U] << 8U)) == -1234);
    assert((int16_t)((uint16_t)packet[46U] | ((uint16_t)packet[47U] << 8U)) == 4321);
    save_fixture("testing/rtos_migration/build-host/encoder_fault.bin", packet);

    BringupEncoderFaultSnapshot saturated = first;
    saturated.valid = 1U;
    saturated.kind = BR_ENCODER_FAULT_KIND_DIRECTION;
    saturated.wheel = BR_ENCODER_FAULT_WHEEL_LEFT;
    saturated.direction_duration_us = 200000U;
    saturated.recent_window_bad = 0x100000U;
    saturated.recent_window_changes = 0x200000U;
    saturated.optical_target_mm_s[0] = -181;
    saturated.previous_pwm[1] = 1001;
    br_encoder_fault_encode(&saturated, 0U, packet);
    assert((packet[5] & 0x3fU) == 0x2fU); /* valid, direction, left, clamped */
    assert((packet[16] & 0xfeU) == 0U);   /* duration high7 reserved */
    assert((read40(packet + 17U) & 0xfffffU) == 0xfffffU);
    assert((read24(packet + 38U) & 0x1ffU) == 0x14cU); /* -180 signed9 */
    assert(((read24(packet + 41U) >> 11U) & 0x7ffU) == 1000U);
    assert((packet[40] & 0xfcU) == 0U && (packet[43] & 0xc0U) == 0U);

    br_encoder_fault_clear(&latched);
    assert(!latched.valid && latched.kind == 0U && latched.control_timer_us == 0U);
    br_encoder_fault_encode(&latched, 0xa5U, packet);
    assert(packet[0] == 0xf2U && packet[1] == 45U && packet[2] == 0xa5U &&
           packet[3] == 0xc4U && packet[4] == 0x21U && packet[5] == 0U);
    for (unsigned i = 6U; i < sizeof(packet); ++i)
        assert(packet[i] == 0U);
    save_fixture("testing/rtos_migration/build-host/encoder_fault_clear.bin", packet);
    assert(br_encoder_fault_capture_once(&latched, &later));

    puts("PASS encoder fault snapshot latch, compact C4 codec, saturation and clear frame");
    return 0;
}
