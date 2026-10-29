#include <assert.h>
#include <stdio.h>
#include "barcode_raw.h"
static BarcodeRaw r;
int main(void) {
    int64_t ticks[2]={-1234,2345};
    barcode_raw_begin(&r,1);barcode_raw_sample(&r,2000,-25,181);
    barcode_raw_sample(&r,4000,0,218);barcode_raw_sample(&r,6000,4,205);
    barcode_raw_finish(&r,7000,ticks,0);
    assert(r.header.meta.data_crc==0x32c011ebu && barcode_raw_valid(&r));
    barcode_raw_begin(&r,1);
    for(unsigned n=0;n<RAW_CAPACITY+1;++n) barcode_raw_sample(&r,(n+1)*2000,n*4,(uint16_t)(180+n%5));
    barcode_raw_finish(&r,9000000,ticks,9);
    assert(barcode_raw_valid(&r) && r.header.meta.count==4096 && r.header.meta.full);
    assert(r.rows[0].us==2000 && r.rows[4095].us==8192000 && r.header.meta.max_gap_us==2000);
    assert(r.header.meta.ticks[0]==-1234 && r.header.meta.ticks[1]==2345);
    r.rows[2].adc++;assert(!barcode_raw_valid(&r));r.rows[2].adc--;
    r.header.meta.pin=26;assert(!barcode_raw_valid(&r));r.header.meta.pin=27;
    r.header.meta.version=2;assert(!barcode_raw_valid(&r));r.header.meta.version=1;
    assert(barcode_raw_valid(&r));
    r.rows[2].us=r.rows[1].us;barcode_raw_finish(&r,9000000,ticks,9);
    assert(!barcode_raw_valid(&r));
    memset(&r,0xff,sizeof(r));assert(!barcode_raw_valid(&r));
    barcode_raw_begin(&r,2);barcode_raw_sample(&r,2000,-25,4095);barcode_raw_finish(&r,3000,ticks,0);
    assert(barcode_raw_valid(&r) && !r.header.meta.full);
    r.rows[0].reserved=1;barcode_raw_finish(&r,3000,ticks,0);assert(!barcode_raw_valid(&r));
    printf("PASS: raw bytes=%zu; bounded/no-overwrite capture; CRC/header/torn-data rejection; timestamp order; signed path; full flag; max gap; metadata.\n",sizeof(r));
}
