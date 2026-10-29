#ifndef BARCODE_RAW_H
#define BARCODE_RAW_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "../dual_wheel_bench/log_store.h"

enum {
#ifdef BUDDY_WIFI_LOGGING
 RAW_CAPACITY=2048,RAW_VERSION=2,
#else
 RAW_CAPACITY=4096,RAW_VERSION=1,
#endif
 RAW_BANK_BYTES=65536 };
#define RAW_MAGIC 0x314f4142u
typedef struct { uint32_t us;int32_t path100;uint16_t adc,reserved; } BarcodeRawRow;
typedef struct {
    uint32_t magic,version,crc,count,data_crc,capacity;
    uint64_t duration_us;int64_t ticks[2];uint32_t max_gap_us;
    uint8_t profile,reason,pin,channel,full;
} BarcodeRawMeta;
typedef union { BarcodeRawMeta meta;uint8_t page[256]; } BarcodeRawHeader;
typedef struct { BarcodeRawHeader header;BarcodeRawRow rows[RAW_CAPACITY]; } BarcodeRaw;
_Static_assert(sizeof(BarcodeRawRow)==12 && sizeof(BarcodeRawHeader)==256,"Raw trace ABI changed");
_Static_assert(sizeof(BarcodeRaw)==256+12*RAW_CAPACITY && sizeof(BarcodeRaw)%256==0 && sizeof(BarcodeRaw)<=RAW_BANK_BYTES,"Raw flash layout");
static inline void barcode_raw_begin(BarcodeRaw *r,unsigned profile) {
    memset(r,0,sizeof(*r));r->header.meta=(BarcodeRawMeta){.magic=RAW_MAGIC,.version=RAW_VERSION,
        .capacity=RAW_CAPACITY,.profile=(uint8_t)profile,.pin=27,.channel=1};
}
static inline void barcode_raw_sample(BarcodeRaw *r,uint64_t us,int32_t path100,uint16_t adc) {
    BarcodeRawMeta *m=&r->header.meta;
    if(m->count==RAW_CAPACITY) { m->full=1;return; }
    if(us>UINT32_MAX) { m->full=1;return; }
    if(m->count && us-r->rows[m->count-1].us>m->max_gap_us)
        m->max_gap_us=(uint32_t)(us-r->rows[m->count-1].us);
    r->rows[m->count++]=(BarcodeRawRow){.us=(uint32_t)us,.path100=path100,.adc=adc};
}
static inline void barcode_raw_finish(BarcodeRaw *r,uint64_t duration,const int64_t ticks[2],unsigned reason) {
    BarcodeRawMeta *m=&r->header.meta;m->duration_us=duration;
    m->ticks[0]=ticks[0];m->ticks[1]=ticks[1];m->reason=(uint8_t)reason;
    m->data_crc=log_crc(r->rows,m->count*sizeof(r->rows[0]));
    m->crc=log_crc((const uint8_t *)r+offsetof(BarcodeRawMeta,count),sizeof(*r)-offsetof(BarcodeRawMeta,count));
}
static inline bool barcode_raw_valid(const BarcodeRaw *r) {
    const BarcodeRawMeta *m=&r->header.meta;
    if(m->magic!=RAW_MAGIC || m->version!=RAW_VERSION || m->count>RAW_CAPACITY ||
       m->capacity!=RAW_CAPACITY || m->profile<1 || m->profile>3 || m->pin!=27 || m->channel!=1 || m->full>1) return false;
    if(m->crc!=log_crc((const uint8_t *)r+offsetof(BarcodeRawMeta,count),sizeof(*r)-offsetof(BarcodeRawMeta,count)) ||
       m->data_crc!=log_crc(r->rows,m->count*sizeof(r->rows[0]))) return false;
    for(unsigned n=0;n<m->count;++n)
        if(r->rows[n].adc>4095 || r->rows[n].reserved ||
           (n && r->rows[n].us<=r->rows[n-1].us)) return false;
    return true;
}
#endif
