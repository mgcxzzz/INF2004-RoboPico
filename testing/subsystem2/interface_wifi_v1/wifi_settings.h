#ifndef BUDDY_WIFI_SETTINGS_H
#define BUDDY_WIFI_SETTINGS_H
#include "wifi_transport.h"
#include <string.h>
#include "nav_log_storage.h"
enum { WIFI_SETTINGS_PAGE=256 };
static inline bool wifi_text(const char *p,size_t cap,bool empty) {
    size_t n=0;while(n<cap && p[n]) {if((unsigned char)p[n]<32 || p[n]=='|')return false;++n;}
    return n<cap && (empty || n>0);
}
static inline bool wifi_ipv4(const char *p) {
    unsigned groups=0,value=0,digits=0;
    for(size_t k=0;k<16;++k) {
        char c=p[k];
        if(c>='0' && c<='9') {value=value*10+(unsigned)(c-'0');if(++digits>3 || value>255)return false;}
        else if(c=='.' || !c) {if(!digits)return false;++groups;if(!c)return groups==4;value=digits=0;}
        else return false;
    }return false;
}
static inline bool wifi_settings_valid(const WifiTransportConfig *c) {
    return c && wifi_text(c->ssid,sizeof(c->ssid),false) && wifi_text(c->password,sizeof(c->password),true) &&
        wifi_text(c->broker,sizeof(c->broker),false) && wifi_ipv4(c->broker) &&
        strcmp(c->broker,"0.0.0.0") && wifi_text(c->client_id,sizeof(c->client_id),false);
}
static inline bool wifi_settings_encode(uint8_t page[WIFI_SETTINGS_PAGE],const WifiTransportConfig *c) {
    if(!wifi_settings_valid(c))return false;
    memset(page,0xff,WIFI_SETTINGS_PAGE);memcpy(page,"WCF1",4);memcpy(page+8,c,sizeof(*c));
    nav_le32(page+4,nav_crc(page+8,WIFI_SETTINGS_PAGE-8));return true;
}
static inline bool wifi_settings_decode(WifiTransportConfig *c,const uint8_t page[WIFI_SETTINGS_PAGE]) {
    if(memcmp(page,"WCF1",4) || nav_rd32(page+4)!=nav_crc(page+8,WIFI_SETTINGS_PAGE-8))return false;
    WifiTransportConfig candidate;memcpy(&candidate,page+8,sizeof(candidate));
    if(!wifi_settings_valid(&candidate))return false;
    *c=candidate;return true;
}
#endif
