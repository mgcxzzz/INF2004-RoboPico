#ifndef BARCODE_PROBE_RECORD_H
#define BARCODE_PROBE_RECORD_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
typedef struct { double white,black,white_sd,black_sd;unsigned mask; } Calibration;
static inline bool probe_cal_valid(const Calibration *c) {
    if(c->mask!=3 || !isfinite(c->white) || !isfinite(c->black) ||
       !isfinite(c->white_sd) || !isfinite(c->black_sd) ||
       c->white<0 || c->white>4095 || c->black<0 || c->black>4095 ||
       c->white_sd<0 || c->black_sd<0) return false;
    double span=fabs(c->black-c->white);
    return span>=100 && span>=6*fmax(c->white_sd,c->black_sd);
}
static inline int probe_level(const Calibration *c,double a) {
    if(!probe_cal_valid(c) || !isfinite(a)) return -1;
    double v=1000*(a-c->white)/(c->black-c->white);
    return v<0?0:v>1000?1000:(int)(v+.5);
}
typedef struct {
    uint32_t magic,version,bytes,sequence,pin,channel;
    Calibration calibration;
    uint32_t crc;
} ProbeRecord;
_Static_assert(sizeof(Calibration)==40 && sizeof(ProbeRecord)==72,"Probe calibration flash ABI changed");
#define PROBE_MAGIC 0x34415042u
static inline uint32_t probe_crc_update(uint32_t crc,const void *data,size_t length) {
    const uint8_t *p=data;
    for(size_t k=0;k<length;++k) {
        crc^=p[k];
        for(unsigned b=0;b<8;++b) crc=(crc>>1)^((crc&1)?0xedb88320u:0);
    }
    return crc;
}
static inline uint32_t probe_crc(const void *data,size_t length) { return ~probe_crc_update(0xffffffffu,data,length); }
static inline bool probe_record_valid(const ProbeRecord *p) {
    return p->magic==PROBE_MAGIC && p->version==1 && p->bytes==sizeof(*p) &&
           p->pin==27 && p->channel==1 && probe_cal_valid(&p->calibration) &&
           p->crc==probe_crc(p,offsetof(ProbeRecord,crc));
}
static inline void probe_record_make(ProbeRecord *p,const Calibration *c,uint32_t seq) {
    memset(p,0,sizeof(*p));p->magic=PROBE_MAGIC;p->version=1;p->bytes=sizeof(*p);
    p->sequence=seq;p->pin=27;p->channel=1;p->calibration=*c;
    p->crc=probe_crc(p,offsetof(ProbeRecord,crc));
}
static inline bool probe_sequence_newer(uint32_t a,uint32_t b) { return (int32_t)(a-b)>0; }
#endif
