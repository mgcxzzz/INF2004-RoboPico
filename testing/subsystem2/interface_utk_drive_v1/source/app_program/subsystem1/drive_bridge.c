/* Kernel-owned bounded bridge for the single core1 controller task and the
 * core0 CYW43/MQTT service. Every MBF/mutex operation is TMO_POL. */
#include "drive_bridge.h"
#include <tk/tkernel.h>
#include <tk/smp.h>

enum {
    DRIVE_TX_BUFFER_BYTES=2048,
    DRIVE_TX_MESSAGE_BYTES=DRIVE_BRIDGE_PACKET_MAX+1,
    DRIVE_RX_QUEUE_DEPTH=8,
    DRIVE_RX_MESSAGE_BYTES=4+1+DRIVE_BRIDGE_RX_COPY_MAX,
    DRIVE_RX_BUFFER_BYTES=DRIVE_RX_QUEUE_DEPTH*DRIVE_RX_MESSAGE_BYTES,
};

static ID g_tx_mbf;
static ID g_rx_mbf;
static ID g_status_mtx;
static volatile uint32_t g_ready;
static T_SPLOCK g_counter_lock;
static WifiTransportStatus g_status;
static volatile uint32_t g_tx_dropped;
static volatile uint32_t g_rx_dropped;
static volatile uint32_t g_published;

static void copy_bytes(uint8_t *dst,const uint8_t *src,uint32_t n)
{
    for(uint32_t i=0u;i<n;++i)dst[i]=src[i];
}

static void zero_bytes(uint8_t *dst,uint32_t n)
{
    for(uint32_t i=0u;i<n;++i)dst[i]=0u;
}

static void counter_increment(volatile uint32_t *counter)
{
    UINT intsts;
    if(ISpinLock(&g_counter_lock,&intsts)!=E_OK)return;
    *counter=*counter+1u;
    (void)ISpinUnlock(&g_counter_lock,intsts);
}

static uint32_t counter_read(volatile uint32_t *counter)
{
    UINT intsts;
    uint32_t value=0u;
    if(ISpinLock(&g_counter_lock,&intsts)!=E_OK)return 0u;
    value=*counter;
    (void)ISpinUnlock(&g_counter_lock,intsts);
    return value;
}

int32_t drive_bridge_init(void)
{
    T_CMBF mbf={0};
    T_CMTX mtx={0};
    ID made_tx=0,made_rx=0,made_mtx=0;
    if(__atomic_load_n(&g_ready,__ATOMIC_ACQUIRE)!=0u)return E_OK;
    ER lock_result=InitSpinLock(&g_counter_lock);
    if(lock_result<E_OK)return lock_result;
    zero_bytes((uint8_t *)&g_status,(uint32_t)sizeof(g_status));
    g_tx_dropped=0u;g_rx_dropped=0u;g_published=0u;

    mbf.mbfatr=TA_TFIFO;mbf.bufsz=DRIVE_TX_BUFFER_BYTES;mbf.maxmsz=DRIVE_TX_MESSAGE_BYTES;
    made_tx=tk_cre_mbf(&mbf);
    if(made_tx<E_OK)return made_tx;

    mbf.bufsz=DRIVE_RX_BUFFER_BYTES;mbf.maxmsz=DRIVE_RX_MESSAGE_BYTES;
    made_rx=tk_cre_mbf(&mbf);
    if(made_rx<E_OK){(void)tk_del_mbf(made_tx);return made_rx;}

    mtx.mtxatr=TA_INHERIT;
    made_mtx=tk_cre_mtx(&mtx);
    if(made_mtx<E_OK){(void)tk_del_mbf(made_rx);(void)tk_del_mbf(made_tx);return made_mtx;}

    g_tx_mbf=made_tx;g_rx_mbf=made_rx;g_status_mtx=made_mtx;
    __atomic_store_n(&g_ready,1u,__ATOMIC_RELEASE);
    return E_OK;
}

bool drive_bridge_ready(void)
{
    return __atomic_load_n(&g_ready,__ATOMIC_ACQUIRE)!=0u;
}

bool drive_bridge_tx_enqueue(uint8_t topic,const uint8_t *data,uint32_t length)
{
    uint8_t message[DRIVE_TX_MESSAGE_BYTES];
    if(!drive_bridge_ready())return false;
    if(data==NULL || length==0u || length>DRIVE_BRIDGE_PACKET_MAX || topic>2u) {
        counter_increment(&g_tx_dropped);return false;
    }
    message[0]=topic;copy_bytes(message+1u,data,length);
    if(tk_snd_mbf(g_tx_mbf,message,(INT)(length+1u),TMO_POL)!=E_OK) {
        counter_increment(&g_tx_dropped);return false;
    }
    return true;
}

bool drive_bridge_tx_take(uint8_t *topic,uint8_t data[DRIVE_BRIDGE_PACKET_MAX],uint32_t *length)
{
    uint8_t message[DRIVE_TX_MESSAGE_BYTES];
    INT received;
    if(!drive_bridge_ready() || topic==NULL || data==NULL || length==NULL)return false;
    received=tk_rcv_mbf(g_tx_mbf,message,TMO_POL);
    if(received<2 || received>(INT)DRIVE_TX_MESSAGE_BYTES)return false;
    *topic=message[0];*length=(uint32_t)received-1u;
    copy_bytes(data,message+1u,*length);
    return true;
}

bool drive_bridge_rx_enqueue(const uint8_t copied[DRIVE_BRIDGE_RX_COPY_MAX],
                             uint8_t copied_length,uint32_t original_length)
{
    uint8_t message[DRIVE_RX_MESSAGE_BYTES];
    if(!drive_bridge_ready())return false;
    if(copied_length>DRIVE_BRIDGE_RX_COPY_MAX || (copied_length!=0u && copied==NULL)) {
        counter_increment(&g_rx_dropped);return false;
    }
    message[0]=(uint8_t)original_length;message[1]=(uint8_t)(original_length>>8);
    message[2]=(uint8_t)(original_length>>16);message[3]=(uint8_t)(original_length>>24);
    message[4]=copied_length;zero_bytes(message+5u,DRIVE_BRIDGE_RX_COPY_MAX);
    if(copied_length!=0u)copy_bytes(message+5u,copied,copied_length);
    if(tk_snd_mbf(g_rx_mbf,message,DRIVE_RX_MESSAGE_BYTES,TMO_POL)!=E_OK) {
        counter_increment(&g_rx_dropped);return false;
    }
    return true;
}

bool drive_bridge_rx_take(uint8_t data[DRIVE_BRIDGE_RX_COPY_MAX],uint32_t *original_length)
{
    uint8_t message[DRIVE_RX_MESSAGE_BYTES];
    INT received;
    if(!drive_bridge_ready() || data==NULL || original_length==NULL)return false;
    received=tk_rcv_mbf(g_rx_mbf,message,TMO_POL);
    if(received!=(INT)DRIVE_RX_MESSAGE_BYTES)return false;
    *original_length=(uint32_t)message[0]|((uint32_t)message[1]<<8)|
        ((uint32_t)message[2]<<16)|((uint32_t)message[3]<<24);
    if(message[4]>DRIVE_BRIDGE_RX_COPY_MAX)return false;
    zero_bytes(data,DRIVE_BRIDGE_RX_COPY_MAX);copy_bytes(data,message+5u,message[4]);
    return true;
}

bool drive_bridge_status_update(const WifiTransportStatus *status)
{
    if(!drive_bridge_ready() || status==NULL || tk_loc_mtx(g_status_mtx,TMO_POL)!=E_OK)return false;
    g_status=*status;
    g_status.tx_dropped=counter_read(&g_tx_dropped);
    g_status.rx_dropped=counter_read(&g_rx_dropped);
    g_status.published=counter_read(&g_published);
    (void)tk_unl_mtx(g_status_mtx);
    return true;
}

bool drive_bridge_status_snapshot(WifiTransportStatus *status)
{
    T_RMBF ref={0};
    if(!drive_bridge_ready() || status==NULL || tk_loc_mtx(g_status_mtx,TMO_POL)!=E_OK)return false;
    *status=g_status;
    status->tx_dropped=counter_read(&g_tx_dropped);
    status->rx_dropped=counter_read(&g_rx_dropped);
    status->published=counter_read(&g_published);
    if(tk_ref_mbf(g_tx_mbf,&ref)==E_OK) {
        uint32_t used=DRIVE_TX_BUFFER_BYTES>(uint32_t)ref.frbufsz?
            DRIVE_TX_BUFFER_BYTES-(uint32_t)ref.frbufsz:0u;
        uint32_t fill=(used*100u)/DRIVE_TX_BUFFER_BYTES;
        status->tx_fill_pct=(uint8_t)(fill>100u?100u:fill);
    }
    (void)tk_unl_mtx(g_status_mtx);
    return true;
}

void drive_bridge_publish_result(bool published)
{
    if(!drive_bridge_ready())return;
    if(published)counter_increment(&g_published);
    else counter_increment(&g_tx_dropped);
}
