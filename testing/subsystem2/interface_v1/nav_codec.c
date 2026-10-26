#include "nav_codec.h"
#include <math.h>
#include <string.h>
static void put(uint8_t *p,unsigned *at,uint64_t value,unsigned bits) {
    for(unsigned k=0;k<bits;++k,++*at)if((value>>k)&1)p[*at/8]|=(uint8_t)(1u<<(*at%8));
}
static uint64_t get(const uint8_t *p,unsigned *at,unsigned bits) {
    uint64_t v=0;for(unsigned k=0;k<bits;++k,++*at)if((p[*at/8]>>(*at%8))&1)v|=UINT64_C(1)<<k;return v;
}
static unsigned payload_bits(unsigned kind) {
    static const unsigned bits[]={0,0,53,52,33,2,2,18,8,11};
    return kind<10?bits[kind]:UINT32_MAX;
}
unsigned nav_packet_encode(uint8_t out[NAV_PACKET_BYTES],const NavRequest *r) {
    if(!r || r->kind>9 || r->owner<1 || r->owner>5 || !isfinite(r->value))return 0;
    memset(out,0,NAV_PACKET_BYTES);unsigned at=0;put(out,&at,r->kind,4);put(out,&at,r->owner,3);
    if(r->kind==NAV_DRIVE || r->kind==NAV_TURN || r->kind==NAV_RESUME)put(out,&at,r->lease,32);
    switch(r->kind) {
    case NAV_DRIVE:
        if(fabsf(r->value)<1 || fabsf(r->value)>1000 || (r->speed && (r->speed<20 || r->speed>60)))return 0;
        put(out,&at,(unsigned)(lroundf(r->value*10)+10000),15);put(out,&at,r->speed,6);break;
    case NAV_TURN:
        if(fabsf(r->value)<1 || fabsf(r->value)>180 || (r->speed && (r->speed<20 || r->speed>90)))return 0;
        put(out,&at,(unsigned)(lroundf(r->value*10)+1800),12);put(out,&at,r->speed,7);put(out,&at,r->style,1);break;
    case NAV_RESUME:put(out,&at,r->flag,1);break;
    case NAV_ROUTE:
        if(r->value!=0 && r->value!=1 && r->value!=2)return 0;
        put(out,&at,(unsigned)r->value,2);break;
    case NAV_TERRAIN:
        if(r->value<0 || r->value>3 || floorf(r->value)!=r->value)return 0;
        put(out,&at,(unsigned)r->value,2);break;
    case NAV_WHEELS:
        if(r->wheels[0]<-180 || r->wheels[0]>180 || r->wheels[1]<-180 || r->wheels[1]>180)return 0;
        put(out,&at,(unsigned)(r->wheels[0]+180),9);put(out,&at,(unsigned)(r->wheels[1]+180),9);break;
    case NAV_ASSIST:put(out,&at,(unsigned)r->value,8);break;
    case NAV_FAULT_REPORT:put(out,&at,r->bitmap,10);put(out,&at,r->flag,1);break;
    default:break;
    }
    return (at+7)/8;
}
bool nav_packet_decode(NavRequest *r,const uint8_t *p,unsigned bytes) {
    if(!r || !p || !bytes || bytes>NAV_PACKET_BYTES)return false;
    memset(r,0,sizeof(*r));unsigned at=0;r->kind=(unsigned)get(p,&at,4);r->owner=(unsigned)get(p,&at,3);
    if(r->kind>9 || r->owner<1 || r->owner>5)return false;
    unsigned bits=7+payload_bits(r->kind);if(bytes!=(bits+7)/8)return false;
    for(unsigned bit=bits;bit<bytes*8;++bit)if((p[bit/8]>>(bit%8))&1)return false;
    if(r->kind==NAV_DRIVE || r->kind==NAV_TURN || r->kind==NAV_RESUME)r->lease=(uint32_t)get(p,&at,32);
    switch(r->kind) {
    case NAV_DRIVE:r->value=((int)get(p,&at,15)-10000)/10.0f;r->speed=(unsigned)get(p,&at,6);break;
    case NAV_TURN:r->value=((int)get(p,&at,12)-1800)/10.0f;r->speed=(unsigned)get(p,&at,7);r->style=(unsigned)get(p,&at,1);break;
    case NAV_RESUME:r->flag=(unsigned)get(p,&at,1);break;
    case NAV_ROUTE:case NAV_TERRAIN:r->value=(float)get(p,&at,2);break;
    case NAV_WHEELS:r->wheels[0]=(int16_t)((int)get(p,&at,9)-180);r->wheels[1]=(int16_t)((int)get(p,&at,9)-180);break;
    case NAV_ASSIST:r->value=(float)get(p,&at,8);break;
    case NAV_FAULT_REPORT:r->bitmap=(unsigned)get(p,&at,10);r->flag=(unsigned)get(p,&at,1);break;
    default:break;
    }
    uint8_t canonical[NAV_PACKET_BYTES];unsigned n=nav_packet_encode(canonical,r);
    return n==bytes && !memcmp(p,canonical,n);
}
static int32_t quant(float value,float scale,int32_t lo,int32_t hi,bool *overflow) {
    double x=(double)value*scale;
    if(!isfinite(x)) {*overflow=true;return 0;}
    if(x<lo) {*overflow=true;return lo;}if(x>hi) {*overflow=true;return hi;}
    return (int32_t)llround(x);
}
void nav_event_encode(uint8_t out[NAV_EVENT_BYTES],const MotionEvent *e) {
    memset(out,0,NAV_EVENT_BYTES);unsigned at=0;bool overflow=false;
    put(out,&at,e->now_us,64);put(out,&at,e->sequence,32);put(out,&at,e->id,32);
    put(out,&at,e->requester,3);put(out,&at,e->owner,3);put(out,&at,e->type,4);put(out,&at,e->kind,4);
    put(out,&at,e->state,3);put(out,&at,e->fault,10);put(out,&at,e->result,4);put(out,&at,e->style,1);put(out,&at,e->speed,7);
    put(out,&at,(uint32_t)(quant(e->goal,10,-10000,10230,&overflow)+10000),15);
    put(out,&at,(uint32_t)(quant(e->progress_mm,10,-8388608,8388607,&overflow)+8388608),24);
    put(out,&at,(uint32_t)(quant(e->rotation_deg,10,-32768,32767,&overflow)+32768),16);
    put(out,&at,overflow,1);
    for(unsigned w=0;w<2;++w)put(out,&at,(uint32_t)(quant(e->target_mm_s[w],1,-180,180,&overflow)+180),9);
    if(overflow)out[222/8]|=(uint8_t)(1u<<(222%8));
}
bool nav_event_decode(MotionEvent *e,const uint8_t in[NAV_EVENT_BYTES],bool *overflow) {
    if(!e || !in)return false;
    memset(e,0,sizeof(*e));unsigned at=0;
    e->now_us=get(in,&at,64);e->sequence=(uint32_t)get(in,&at,32);e->id=(uint32_t)get(in,&at,32);
    e->requester=(unsigned)get(in,&at,3);e->owner=(unsigned)get(in,&at,3);e->type=(unsigned)get(in,&at,4);e->kind=(unsigned)get(in,&at,4);
    e->state=(unsigned)get(in,&at,3);e->fault=(unsigned)get(in,&at,10);e->result=(unsigned)get(in,&at,4);e->style=(unsigned)get(in,&at,1);e->speed=(unsigned)get(in,&at,7);
    e->goal=((int)get(in,&at,15)-10000)/10.0f;e->progress_mm=((int)get(in,&at,24)-8388608)/10.0f;
    e->rotation_deg=((int)get(in,&at,16)-32768)/10.0f;bool clipped=get(in,&at,1)!=0;
    for(unsigned w=0;w<2;++w)e->target_mm_s[w]=(int16_t)((int)get(in,&at,9)-180);
    if(overflow)*overflow=clipped;
    for(unsigned bit=at;bit<NAV_EVENT_BYTES*8;++bit)if((in[bit/8]>>(bit%8))&1)return false;
    return e->requester<=5 && e->owner<=5 && e->type<=MOTION_EVENT_SESSION_END && e->kind<=9 && e->state<=MOTION_CMD_FAILED && e->result<=MOTION_NEED_RECOVERY &&
        e->target_mm_s[0]>=-180 && e->target_mm_s[0]<=180 && e->target_mm_s[1]>=-180 && e->target_mm_s[1]<=180;
}
