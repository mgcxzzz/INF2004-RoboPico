#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <tk/tkernel.h>
#include <tk/smp.h>
#include "../../../app_program/subsystem1/drive_bridge.h"

enum { MAX_MSGS=64, MAX_MSG_BYTES=65 };
typedef struct { int len; uint8_t data[MAX_MSG_BYTES]; } Message;
typedef struct { int capacity,used,max_message,count; Message messages[MAX_MSGS]; } FakeMbf;
static FakeMbf mbfs[3];
static int next_mbf=1;
static bool mutex_busy;
static bool counter_lock_ready;

ER InitSpinLock(T_SPLOCK *lock){*lock=0u;counter_lock_ready=true;return E_OK;}
ER ISpinLock(T_SPLOCK *lock,UINT *intsts){(void)lock;assert(counter_lock_ready);*intsts=0u;return E_OK;}
ER ISpinUnlock(T_SPLOCK *lock,UINT intsts){(void)lock;(void)intsts;return E_OK;}

ID tk_cre_mbf(const T_CMBF *spec)
{
    assert(spec!=NULL && next_mbf<3);
    ID id=next_mbf++;
    mbfs[id].capacity=spec->bufsz;mbfs[id].max_message=spec->maxmsz;
    return id;
}
ER tk_del_mbf(ID id){(void)id;return E_OK;}
ER tk_snd_mbf(ID id,const void *data,INT size,TMO timeout)
{
    FakeMbf *mbf=&mbfs[id];assert(timeout==TMO_POL);
    if(size<=0 || size>mbf->max_message || mbf->count==MAX_MSGS || mbf->used+size>mbf->capacity)return E_TMOUT;
    Message *msg=&mbf->messages[mbf->count++];msg->len=size;memcpy(msg->data,data,(size_t)size);mbf->used+=size;
    return E_OK;
}
INT tk_rcv_mbf(ID id,void *data,TMO timeout)
{
    FakeMbf *mbf=&mbfs[id];assert(timeout==TMO_POL);
    if(mbf->count==0)return E_TMOUT;
    Message msg=mbf->messages[0];memmove(mbf->messages,mbf->messages+1,(size_t)(--mbf->count)*sizeof(Message));
    mbf->used-=msg.len;memcpy(data,msg.data,(size_t)msg.len);return msg.len;
}
ER tk_ref_mbf(ID id,T_RMBF *status)
{status->frbufsz=mbfs[id].capacity-mbfs[id].used;return E_OK;}
ID tk_cre_mtx(const T_CMTX *spec){(void)spec;return 3;}
ER tk_loc_mtx(ID id,TMO timeout){(void)id;assert(timeout==TMO_POL);return mutex_busy?E_TMOUT:E_OK;}
ER tk_unl_mtx(ID id){(void)id;return E_OK;}

int main(void)
{
    uint8_t packet[DRIVE_BRIDGE_PACKET_MAX],out[DRIVE_BRIDGE_PACKET_MAX],topic=0u;
    uint32_t length=0u,original=0u;
    WifiTransportStatus in={.configured=1u,.ready=1u,.connected=1u,.subscribed=1u,.published=7u};
    WifiTransportStatus status;

    assert(!drive_bridge_ready());
    assert(drive_bridge_init()==E_OK && drive_bridge_ready());
    assert(!drive_bridge_tx_take(&topic,out,&length));

    for(unsigned i=0;i<sizeof(packet);++i)packet[i]=(uint8_t)(i^0x5au);
    assert(drive_bridge_tx_enqueue(WIFI_TOPIC_HB,packet,sizeof(packet)));
    memset(out,0,sizeof(out));
    assert(drive_bridge_tx_take(&topic,out,&length));
    assert(topic==WIFI_TOPIC_HB && length==sizeof(packet) && memcmp(packet,out,sizeof(packet))==0);
    assert(!drive_bridge_tx_enqueue(3u,packet,1u));
    assert(!drive_bridge_tx_enqueue(WIFI_TOPIC_TLM,packet,65u));

    assert(drive_bridge_rx_enqueue(packet,8u,1024u));
    memset(out,0,sizeof(out));
    assert(drive_bridge_rx_take(out,&original));
    assert(original==1024u && memcmp(packet,out,8u)==0);
    assert(!drive_bridge_rx_take(out,&original));

    for(unsigned n=0;n<7u;++n)drive_bridge_publish_result(true);
    assert(drive_bridge_status_update(&in));
    assert(drive_bridge_status_snapshot(&status));
    assert(status.configured && status.ready && status.connected && status.subscribed);
    assert(status.published==7u && status.tx_dropped>=2u);
    assert(status.tx_fill_pct==0u);
    mutex_busy=true;
    in.connected=0u;
    assert(!drive_bridge_status_update(&in));
    assert(!drive_bridge_status_snapshot(&status));
    mutex_busy=false;
    assert(drive_bridge_status_snapshot(&status) && status.connected);

    /* Fill the bounded TX MBF until it reports a nonblocking drop. */
    unsigned accepted=0u;
    while(drive_bridge_tx_enqueue(WIFI_TOPIC_ACK,packet,sizeof(packet)))++accepted;
    assert(accepted>0u && accepted<40u);
    assert(drive_bridge_status_snapshot(&status) && status.tx_fill_pct>0u && status.tx_fill_pct<=100u);
    assert(status.tx_dropped>=3u);
    puts("PASS drive bridge: 64-byte topic packets, RX original length, TMO_POL busy path, bounded queue/drop telemetry");
    return 0;
}
