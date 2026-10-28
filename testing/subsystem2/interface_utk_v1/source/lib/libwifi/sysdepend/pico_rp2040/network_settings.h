/* Portable decoder and runtime accessors for the WCF1 USB-provisioned page.
 * This header intentionally has no Pico SDK or micro T-Kernel dependencies. */
#ifndef UTK_NETWORK_SETTINGS_H
#define UTK_NETWORK_SETTINGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum { UTK_NETWORK_SETTINGS_PAGE_BYTES=256u,
       UTK_NETWORK_SETTINGS_DATA_OFFSET=8u,
       UTK_NETWORK_SETTINGS_DATA_BYTES=248u };

typedef struct {
    char ssid[33];
    char password[65];
    char broker[16];
    char client_id[32];
} NetworkSettings;

_Static_assert(sizeof(NetworkSettings)==146u,"WCF1 settings ABI changed");

static inline uint32_t utk_network_settings_crc32(const uint8_t *data,size_t length)
{
    uint32_t crc=UINT32_MAX;
    if(data==NULL && length!=0u)return 0u;
    for(size_t i=0u;i<length;++i) {
        crc^=data[i];
        for(unsigned bit=0u;bit<8u;++bit)
            crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}

static inline uint32_t utk_network_settings_read_le32(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}

static inline bool utk_network_settings_text_valid(const char *p,size_t capacity,bool allow_empty)
{
    size_t n=0u;
    if(p==NULL)return false;
    while(n<capacity && p[n]!='\0') {
        unsigned char c=(unsigned char)p[n];
        if(c<32u || c=='|')return false;
        ++n;
    }
    return n<capacity && (allow_empty || n>0u);
}

static inline bool utk_network_settings_ipv4_valid(const char *p)
{
    unsigned groups=0u,value=0u,digits=0u;
    if(p==NULL)return false;
    for(size_t i=0u;i<16u;++i) {
        char c=p[i];
        if(c>='0' && c<='9') {
            value=value*10u+(unsigned)(c-'0');
            if(++digits>3u || value>255u)return false;
        } else if(c=='.' || c=='\0') {
            if(digits==0u)return false;
            ++groups;
            if(c=='\0')return groups==4u;
            value=0u;digits=0u;
        } else return false;
    }
    return false;
}

static inline bool utk_network_settings_valid(const NetworkSettings *settings)
{
    return settings!=NULL &&
        utk_network_settings_text_valid(settings->ssid,sizeof(settings->ssid),false) &&
        utk_network_settings_text_valid(settings->password,sizeof(settings->password),true) &&
        utk_network_settings_text_valid(settings->broker,sizeof(settings->broker),false) &&
        utk_network_settings_ipv4_valid(settings->broker) &&
        strcmp(settings->broker,"0.0.0.0")!=0 &&
        utk_network_settings_text_valid(settings->client_id,sizeof(settings->client_id),false);
}

/* Layout matches interface_wifi_v1/wifi_settings.h: magic@0, CRC32@4,
 * NetworkSettings@8, remaining page bytes included in the CRC. On failure,
 * output is not modified. */
static inline bool utk_network_settings_decode(NetworkSettings *out,const uint8_t page[UTK_NETWORK_SETTINGS_PAGE_BYTES])
{
    NetworkSettings candidate;
    if(out==NULL || page==NULL || memcmp(page,"WCF1",4u)!=0 ||
       utk_network_settings_read_le32(page+4u)!=
           utk_network_settings_crc32(page+UTK_NETWORK_SETTINGS_DATA_OFFSET,UTK_NETWORK_SETTINGS_DATA_BYTES))return false;
    memcpy(&candidate,page+UTK_NETWORK_SETTINGS_DATA_OFFSET,sizeof(candidate));
    if(!utk_network_settings_valid(&candidate))return false;
    *out=candidate;
    return true;
}

/* Defined by cyw43_utk.c after its core0-owned WCF1 read. MQTT uses these
 * getters so its network-owner context consumes the exact same loaded page. */
bool utk_network_settings_load(void);
bool utk_network_settings_available(void);
const char *utk_network_ssid(void);
const char *utk_network_password(void);
const char *utk_network_broker(void);
const char *utk_network_client_id(void);

#endif
