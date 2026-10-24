#include <assert.h>
#include <stdio.h>
#include "probe_record.h"
int main(void) {
    Calibration c={.white=226.5,.black=3824.4,.white_sd=9.1,.black_sd=5.1,.mask=3};
    assert(probe_cal_valid(&c));assert(probe_level(&c,226.5)==0 && probe_level(&c,3824.4)==1000);
    assert(probe_level(&c,(c.white+c.black)*.5)==500);
    assert(probe_level(&c,0)==0 && probe_level(&c,4095)==1000);
    Calibration reversed={.white=3824.4,.black=226.5,.white_sd=5,.black_sd=9,.mask=3};
    assert(probe_cal_valid(&reversed));assert(probe_level(&reversed,3824.4)==0 && probe_level(&reversed,226.5)==1000);
    Calibration bad=c;bad.black=bad.white;assert(!probe_cal_valid(&bad) && probe_level(&bad,500)==-1);
    bad=c;bad.mask=1;assert(!probe_cal_valid(&bad));
    bad=c;bad.black=NAN;assert(!probe_cal_valid(&bad));
    bad=c;bad.white_sd=1000;assert(!probe_cal_valid(&bad));
    ProbeRecord a,b;probe_record_make(&a,&c,1);probe_record_make(&b,&c,2);
    assert(probe_record_valid(&a) && probe_record_valid(&b) && probe_sequence_newer(b.sequence,a.sequence));
    ((unsigned char *)&b)[30]^=1;assert(!probe_record_valid(&b) && probe_record_valid(&a));
    probe_record_make(&b,&c,2);b.pin=26;b.crc=probe_crc(&b,offsetof(ProbeRecord,crc));assert(!probe_record_valid(&b));
    probe_record_make(&b,&c,2);b.version=99;b.crc=probe_crc(&b,offsetof(ProbeRecord,crc));assert(!probe_record_valid(&b));
    memset(&b,0xff,sizeof(b));assert(!probe_record_valid(&b));
    assert(probe_sequence_newer(0,UINT32_MAX) && !probe_sequence_newer(UINT32_MAX,0));
    assert(probe_crc("123456789",9)==0xcbf43926u);
    uint32_t stream=probe_crc_update(0xffffffffu,"1234",4);stream=probe_crc_update(stream,"56789",5);
    assert(~stream==0xcbf43926u);
    puts("PASS: calibration normalization both polarities; invalid references; flash CRC/version/pin/erased/corrupt record rejection; old slot validity; sequence wrap; incremental CRC golden vector.");
}
