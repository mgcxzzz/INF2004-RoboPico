#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h"
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_raw.h"
int main(void) {
    uint8_t *bank=calloc(1,65536);assert(bank);BarcodeSaved *s=(BarcodeSaved *)bank;
    s->magic=B_MAGIC;s->version=26;s->profile=s->recorded_profile=1;
    s->calibration=(LineCalibration){{163,160},{3457,3700},7};s->side=(SideCalibration){0,1,3};
    s->count=B_ROWS;s->decoded_total=27;s->crc=log_crc(bank+offsetof(BarcodeSaved,count),barcode_record_crc_size(26));
    assert(barcode_record_valid(s));s->rows[B_ROWS-1].motion.level[0]^=1;assert(!barcode_record_valid(s));
    s->version=25;s->count=1000;s->profile=3;
    s->crc=log_crc(bank+offsetof(BarcodeSaved,count),64848u - offsetof(BarcodeSaved,count));
    LineCalibration cal={0};uint8_t profile=0;
    assert(!barcode_record_valid(s));assert(barcode_import_v25(bank,&cal,&profile));assert(profile==3 && cal.white[0]==163);
    bank[64800]^=1;assert(!barcode_import_v25(bank,&cal,&profile));
    BarcodeRaw *raw=calloc(1,sizeof(*raw));assert(raw);barcode_raw_begin(raw,1);
    for(unsigned k=0;k<RAW_CAPACITY+2;++k)barcode_raw_sample(raw,2000u*k,(int32_t)k,200);
    int64_t ticks[2]={1,2};barcode_raw_finish(raw,5000000,ticks,11);
    assert(barcode_raw_valid(raw) && raw->header.meta.count==2048 && raw->header.meta.full);
    raw->rows[1024].adc^=1;assert(!barcode_raw_valid(raw));
    printf("PASS record26 bytes=%zu, latest384 rows, CRC, V25 calibration-only import, RAW2 capacity2048\n",sizeof(*s));
    free(raw);free(bank);return 0;
}
