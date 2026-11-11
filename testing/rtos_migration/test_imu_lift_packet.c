#include <assert.h>
#include <stdio.h>
#include "app_program/imu_navigation/imu_lift_packet.h"
int main(void) {
    RiLiftPacket s = {.trip_ms=123400,.sample_ms=123390,.events=7,.up_mg=-1536,
        .norm_mg=1024,.reason=1,.latched=1,.armed=1,.fresh=1,.can_reset=1,
        .up_ms=25,.tilt_ms=42,.upright_ms=500};
    uint8_t frame[RI_LIFT_PACKET_BYTES];
    ri_lift_packet_encode(&s,9,123456,0x1234,frame);
    assert(frame[0]==0xf2 && frame[1]==27 && frame[2]==9);
    assert(frame[3]==0xc9 && frame[4]==0x23);
    assert(!(frame[20]&0xe0) && !(frame[22]&0xf0) && !(frame[23]&0x80));
    assert(frame[28]==0x34 && frame[29]==0x12);
    FILE *f=fopen("testing/rtos_migration/build-host/imu_lift.bin","wb");
    assert(f && fwrite(frame,1,sizeof(frame),f)==sizeof(frame));fclose(f);
    puts("PASS: actual compact IMU lift C9 frame fixture, signed projection and reserved bits");
    return 0;
}
