#ifndef BUDDY_DRIVE_DIAG_STORAGE_H
#define BUDDY_DRIVE_DIAG_STORAGE_H
#include "drive_diag.h"
#include "nav_log_storage.h"
static inline void diag_header(uint8_t h[DIAG_HEADER],const DriveDiag *d) {
 memset(h,0,DIAG_HEADER);memcpy(h,"DGD1",4);h[4]=1;h[5]=DIAG_ROW_BYTES;
 uint32_t flags=d->count|((uint32_t)d->head<<9)|((uint32_t)d->reason<<17)|((uint32_t)d->profile<<23)|
  ((uint32_t)d->trigger<<25)|((uint32_t)d->frozen<<27)|((uint32_t)d->triggered<<28)|((uint32_t)d->begun<<29);
 nav_le32(h+8,flags);nav_le32(h+12,nav_crc((const uint8_t *)d->rows,sizeof(d->rows)));
 nav_le32(h+20,d->total);nav_le32(h+24,d->max_gap);nav_le32(h+28,d->max_cost);
 nav_le32(h+32,d->late20);nav_le32(h+36,d->trigger_us);nav_le32(h+40,d->overwritten);nav_le32(h+44,d->max_pack);
 nav_le64(h+48,d->start_us);nav_le64(h+56,d->duration_us);nav_le64(h+64,(uint64_t)d->ticks[0]);nav_le64(h+72,(uint64_t)d->ticks[1]);
 uint64_t cal=d->white[0]|((uint64_t)d->white[1]<<12)|((uint64_t)d->black[0]<<24)|((uint64_t)d->black[1]<<36);
 for(unsigned k=0;k<6;++k)h[80+k]=(uint8_t)(cal>>(k*8));
 h[87]=(uint8_t)d->post;
 nav_le32(h+16,nav_crc(h,DIAG_HEADER));
}
static inline bool diag_bank_valid(const uint8_t *bank) {
 if(memcmp(bank,"DGD1",4)||bank[4]!=1||bank[5]!=DIAG_ROW_BYTES)return false;
 uint8_t h[DIAG_HEADER];memcpy(h,bank,sizeof(h));uint32_t crc=nav_rd32(h+16);memset(h+16,0,4);
 if(nav_crc(h,sizeof(h))!=crc)return false;
 uint32_t bits=nav_rd32(h+8);unsigned count=bits&511,head=(bits>>9)&255,profile=(bits>>23)&3;
 if(count>DIAG_ROWS || (!profile) || (bits>>30) || !(bits&(1u<<29)) || h[87]>DIAG_POST)return false;
 if(count<DIAG_ROWS && head!=count)return false;
 if(nav_crc(bank+DIAG_HEADER,DIAG_ROWS*DIAG_ROW_BYTES)!=nav_rd32(h+12))return false;
 const DiagRow *rows=(const DiagRow *)(bank+DIAG_HEADER);uint32_t previous=0;
 for(unsigned k=0;k<count;++k) {
  const DiagRow *r=&rows[(head+DIAG_ROWS-count+k)%DIAG_ROWS];DiagValues v;diag_unpack(r,&v);
  if((k && v.v[D_US]<=previous)||v.v[D_PROFILE]<1 || v.v[D_PROFILE]>3)return false;
  previous=(uint32_t)v.v[D_US];
  unsigned used=diag_bits()%8;if(used && r->bytes[DIAG_ROW_BYTES-1]>>used)return false;
 }
 return true;
}
static inline void diag_restore(DriveDiag *d,const uint8_t *bank) {
 memset(d,0,sizeof(*d));uint32_t bits=nav_rd32(bank+8);
 d->count=(uint16_t)(bits&511);d->head=(uint16_t)((bits>>9)&255);d->reason=(uint8_t)((bits>>17)&63);
 d->profile=(uint8_t)((bits>>23)&3);d->trigger=(uint8_t)((bits>>25)&3);
 d->frozen=(bits>>27)&1;d->triggered=(bits>>28)&1;d->begun=(bits>>29)&1;
 d->total=nav_rd32(bank+20);d->max_gap=nav_rd32(bank+24);d->max_cost=nav_rd32(bank+28);
 d->late20=nav_rd32(bank+32);d->trigger_us=nav_rd32(bank+36);d->overwritten=nav_rd32(bank+40);d->max_pack=nav_rd32(bank+44);
 d->start_us=nav_rd64(bank+48);d->duration_us=nav_rd64(bank+56);d->ticks[0]=(int64_t)nav_rd64(bank+64);d->ticks[1]=(int64_t)nav_rd64(bank+72);
 uint64_t c=0;for(unsigned k=0;k<6;++k)c|=(uint64_t)bank[80+k]<<(k*8);
 d->white[0]=(uint16_t)(c&4095);d->white[1]=(uint16_t)((c>>12)&4095);
 d->black[0]=(uint16_t)((c>>24)&4095);d->black[1]=(uint16_t)((c>>36)&4095);d->post=bank[87];
 memcpy(d->rows,bank+DIAG_HEADER,sizeof(d->rows));
}
#endif
