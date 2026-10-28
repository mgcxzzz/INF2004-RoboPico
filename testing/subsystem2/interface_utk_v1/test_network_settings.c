#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "source/lib/libwifi/sysdepend/pico_rp2040/network_settings.h"

static void put_le32(uint8_t *p,uint32_t value)
{
    p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8);p[2]=(uint8_t)(value>>16);p[3]=(uint8_t)(value>>24);
}

static void seal(uint8_t page[UTK_NETWORK_SETTINGS_PAGE_BYTES],const NetworkSettings *settings)
{
    memset(page,0xff,UTK_NETWORK_SETTINGS_PAGE_BYTES);
    memcpy(page,"WCF1",4u);memcpy(page+UTK_NETWORK_SETTINGS_DATA_OFFSET,settings,sizeof(*settings));
    put_le32(page+4u,utk_network_settings_crc32(page+UTK_NETWORK_SETTINGS_DATA_OFFSET,
        UTK_NETWORK_SETTINGS_DATA_BYTES));
}

int main(int argc,char **argv)
{
    if(argc==2) {
        uint8_t wire[256];NetworkSettings decoded;FILE *file=fopen(argv[1],"rb");
        assert(file && fread(wire,1,sizeof(wire),file)==sizeof(wire));fclose(file);
        assert(utk_network_settings_decode(&decoded,wire));
        assert(!strcmp(decoded.ssid,"UTK-TEST") && !strcmp(decoded.broker,"192.168.1.2"));
        assert(!strcmp(decoded.client_id,"is28-utk-network-v1"));
        puts("PASS Python BOOTSEL page consumed by production C WCF1 decoder");return 0;
    }
    uint8_t page[UTK_NETWORK_SETTINGS_PAGE_BYTES];
    NetworkSettings source={0},decoded={0};
    strcpy(source.ssid,"RoboPico-Test");strcpy(source.password,"test-pass");
    strcpy(source.broker,"192.168.1.42");strcpy(source.client_id,"is28-buddy123");
    seal(page,&source);
    assert(utk_network_settings_decode(&decoded,page));
    assert(strcmp(decoded.ssid,source.ssid)==0 && strcmp(decoded.password,source.password)==0);
    assert(strcmp(decoded.broker,source.broker)==0 && strcmp(decoded.client_id,source.client_id)==0);

    /* The settings transport permits an empty password for open APs and
     * includes the full FF-padded remainder in its CRC. */
    source.password[0]='\0';seal(page,&source);assert(utk_network_settings_decode(&decoded,page));
    assert(decoded.password[0]=='\0');

    NetworkSettings unchanged=decoded;
    page[0]='X';assert(!utk_network_settings_decode(&decoded,page));assert(memcmp(&decoded,&unchanged,sizeof(decoded))==0);
    seal(page,&source);page[4]^=1u;assert(!utk_network_settings_decode(&decoded,page));
    seal(page,&source);page[UTK_NETWORK_SETTINGS_PAGE_BYTES-1u]^=1u;
    assert(!utk_network_settings_decode(&decoded,page));
    seal(page,&source);source.broker[0]='\0';seal(page,&source);assert(!utk_network_settings_decode(&decoded,page));
    strcpy(source.broker,"256.1.1.1");seal(page,&source);assert(!utk_network_settings_decode(&decoded,page));
    strcpy(source.broker,"192.168.1.42");strcpy(source.ssid,"bad|ssid");seal(page,&source);
    assert(!utk_network_settings_decode(&decoded,page));
    memset(source.ssid,'S',sizeof(source.ssid));seal(page,&source);assert(!utk_network_settings_decode(&decoded,page));
    memset(source.ssid,0,sizeof(source.ssid));strcpy(source.ssid,"ok");
    strcpy(source.password,"bad\npassword");seal(page,&source);assert(!utk_network_settings_decode(&decoded,page));
    puts("network settings WCF1 tests passed");
    return 0;
}
