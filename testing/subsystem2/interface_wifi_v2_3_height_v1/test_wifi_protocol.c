#include "wifi_protocol.h"
#include <assert.h>
#include <stdio.h>

static uint32_t pack_letters(char a, char b, char c, char d)
{
    return (uint32_t)(a-'A') | ((uint32_t)(b-'A')<<5) |
           ((uint32_t)(c-'A')<<10) | ((uint32_t)(d-'A')<<15);
}
static void map_command(uint32_t word, uint8_t id, uint8_t out[5])
{
    out[0]=(uint8_t)((WP_CMD_MAP<<4)|WP_VERSION); out[1]=id;
    out[2]=(uint8_t)word; out[3]=(uint8_t)(word>>8); out[4]=(uint8_t)(word>>16);
}
int main(void)
{
    uint8_t p[5]={0}, ack[4], frame[8]; WifiCommand c; uint32_t map, snap;
    unsigned i, j, k;
    /* Every A-Z letter is exercised in each of four slots; all four remain distinct. */
    for (i=0;i<26;i++) {
        char x=(char)('A'+i); char a[4]={'A','B','C','D'};
        for (j=0;j<4;j++) a[j]=(char)('A'+((i+j)%26));
        /* Replace any accidental duplicates at the wrap boundary. */
        for (j=1;j<4;j++) for (k=0;k<j;k++) if (a[j]==a[k]) a[j]=(char)('A'+((i+j+4)%26));
        map=pack_letters(a[0],a[1],a[2],a[3]); assert(wp_map_valid(map));
        map_command(map,(uint8_t)i,p); assert(wp_parse(p,5,&c)==WP_ACK_ACCEPTED);
        assert(c.id==(uint8_t)i && c.mapping==map);
        assert(wp_map_action(map|WP_MAP_CONFIGURED,x)=='-' || x==a[0] || x==a[1] || x==a[2] || x==a[3]);
        for (j=0;j<4;j++) assert(wp_map_action(map|WP_MAP_CONFIGURED,a[j])=="SULR"[j]);
    }
    map=pack_letters('S','U','L','R'); assert(wp_map_valid(map));
    assert(!wp_map_valid(pack_letters('A','A','C','D')));
    assert(!wp_map_valid(map | (1u<<20)));
    /* All length/version failures, including oversized received MQTT buffers. */
    for (i=0;i<7;i++) { p[0]=(uint8_t)((WP_CMD_START<<4)|WP_VERSION); assert(wp_parse(p,i,&c)==(i==4u ? WP_ACK_ACCEPTED : WP_ACK_FORMAT)); }
    assert(wp_parse(NULL,4,&c)==WP_ACK_FORMAT); assert(wp_parse(p,4,NULL)==WP_ACK_FORMAT);
    p[0]=(uint8_t)((WP_CMD_START<<4)|1u); assert(wp_parse(p,4,&c)==WP_ACK_FORMAT);
    p[0]=(uint8_t)((WP_CMD_START<<4)|WP_VERSION);
    for (i=1;i<=8;i++) { if (i==7u) continue; p[0]=(uint8_t)((i<<4)|WP_VERSION); assert(wp_parse(p,4,&c)==WP_ACK_ACCEPTED); }
    p[0]=(uint8_t)((WP_CMD_CONFIG<<4)|WP_VERSION); p[2]=0; p[3]=0x10;
    assert(wp_parse(p,4,&c)==WP_ACK_ACCEPTED);
    p[0]=(uint8_t)((WP_CMD_GET<<4)|WP_VERSION); p[2]=p[3]=0; assert(wp_parse(p,4,&c)==WP_ACK_ACCEPTED);
    for (i=12;i<=14;i++) { p[0]=(uint8_t)((i<<4)|WP_VERSION); assert(wp_parse(p,4,&c)==WP_ACK_PROHIBITED); }
    p[0]=(uint8_t)((15u<<4)|WP_VERSION); assert(wp_parse(p,4,&c)==WP_ACK_UNKNOWN);
    p[0]=(uint8_t)((WP_CMD_CONFIG<<4)|WP_VERSION); p[2]=0; p[3]=0x20;
    assert(wp_parse(p,4,&c)==WP_ACK_UNSUPPORTED);
    p[3]=0x10; p[2]=2; assert(wp_parse(p,4,&c)==WP_ACK_FORMAT);
    p[0]=(uint8_t)((WP_CMD_GET<<4)|WP_VERSION); p[2]=1; assert(wp_parse(p,4,&c)==WP_ACK_FORMAT);
    p[0]=(uint8_t)((WP_CMD_GET_LOG<<4)|WP_VERSION); p[2]=0; p[3]=0x40;
    assert(wp_parse(p,4,&c)==WP_ACK_UNSUPPORTED);
    p[3]=0x30; assert(wp_parse(p,4,&c)==WP_ACK_ACCEPTED);
    /* MAP invalid words never leak a usable partial mapping. */
    map_command(pack_letters('A','A','C','D'),77,p);
    assert(wp_parse(p,5,&c)==WP_ACK_FORMAT && c.mapping==0u && c.id==77u);
    p[4]|=0x10u; assert(wp_parse(p,5,&c)==WP_ACK_FORMAT);
    snap=wp_map_replace(0u,map); assert((snap&WP_MAP_CONFIGURED)!=0u);
    assert((snap>>WP_MAP_REV_SHIFT)==1u);
    assert(wp_map_replace(snap,map)==snap); /* retries are idempotent */
    assert(wp_map_replace(snap,pack_letters('A','B','C','D'))!=snap);
    assert(wp_map_replace(snap,pack_letters('A','A','C','D'))==snap);
    assert(wp_map_action(snap,'S')=='S' && wp_map_action(snap,'R')=='R');
    wp_map_frame(snap,42u,9u,frame);
    assert(frame[0]==0x82u && frame[1]==5u && frame[2]==9u && frame[7]==42u);
    { uint32_t echoed=(uint32_t)frame[3]|((uint32_t)frame[4]<<8)|((uint32_t)frame[5]<<16)|((uint32_t)frame[6]<<24); assert(echoed==snap); }
    p[0]=(uint8_t)((WP_CMD_START<<4)|WP_VERSION); p[1]=0xab; p[2]=p[3]=0;
    assert(wp_parse(p,4,&c)==WP_ACK_ACCEPTED); wp_ack(&c,WP_ACK_ACCEPTED,15,ack);
    assert(ack[0]==p[0] && ack[1]==0xabu && ack[2]==0u && ack[3]==7u);
    puts("wifi protocol tests passed"); return 0;
}
