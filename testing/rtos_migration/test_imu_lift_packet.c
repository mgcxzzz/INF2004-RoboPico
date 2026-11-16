#include <assert.h>
#include <stdio.h>
#include "src/platform/compact_signed.h"
#include "app_program/imu_navigation/imu_lift_packet.h"
int main(void) {
    ri_lift_packet_t s = {.trip_ms=123400,.sample_ms=123390,.events=7,.up_bits=compact_signed_pack(-1536,13U),
        .norm_mg=1024,.reason=1,.latched=1,.armed=1,.fresh=1,.can_reset=1,
        .up_ms=25,.tilt_ms=100,.upright_ms=500};
    uint8_t frame[RI_LIFT_PACKET_BYTES];
    ri_lift_packet_encode(&s,9,123456,0x1234,frame);
    assert(frame[0]==0xf2 && frame[1]==27 && frame[2]==9);
    assert(frame[3]==0xc9 && frame[4]==0x25);
    assert(frame[23]==0x3d && frame[24]==25 && frame[25]==100);
    assert(!(frame[25]&0x80));
    assert(frame[28]==0x34 && frame[29]==0x12);
    FILE *f=fopen("testing/rtos_migration/build-host/imu_lift.bin","wb");
    assert(f && fwrite(frame,1,sizeof(frame),f)==sizeof(frame));fclose(f);
    ri_lift_peak_packet_t peak = {.report_ms=123456,.peak_sample_ms=123390,.window_start_ms=120000,
        .min_cos_q15=-12345,.norm_mg=1000,.up_mg=-1536,.events=7,
        .tilt_ms=100,.up_ms=30,.last_candidate=2,.last_rejection=1,
        .armed=true,.current_valid=true,.latched=true,.saturated=false};
    uint8_t peak_frame[RI_LIFT_PEAK_PACKET_BYTES];
    ri_lift_peak_packet_encode(&peak,10,peak_frame);
    assert(peak_frame[0]==0xf2 && peak_frame[1]==25 && peak_frame[2]==10);
    assert(peak_frame[3]==0xca && peak_frame[4]==1 && peak_frame[19]==100 && peak_frame[20]==30);
    assert(peak_frame[21]==0x76 && peak_frame[26]==7 && peak_frame[27]==0);
    assert(!(peak_frame[16]&0xf0) && !(peak_frame[18]&0xe0));
    FILE *pf=fopen("testing/rtos_migration/build-host/imu_lift_peak.bin","wb");
    assert(pf && fwrite(peak_frame,1,sizeof(peak_frame),pf)==sizeof(peak_frame));fclose(pf);
    puts("PASS: actual C9 0x25 and CA 0x01 frame fixtures, peak metrics, signed values and reserved bits");
    return 0;
}
