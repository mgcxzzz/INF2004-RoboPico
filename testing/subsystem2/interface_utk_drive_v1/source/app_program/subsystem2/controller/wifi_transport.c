/* Controller-side transport adapter. MQTT and lwIP remain in the existing
 * kernel-owned CYW43 service; this file only copies bounded packets through
 * the T-Kernel bridge created during phase-1 initialization. */
#include "wifi_transport.h"
#include "../../subsystem1/drive_bridge.h"
#include "../../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#include "../../../lib/libwifi/sysdepend/pico_rp2040/network_settings.h"
#include "wifi_mqtt_diag.h"

#include <string.h>

static bool g_initialized;
static WifiTransportStatus g_last_status;

static uint32_t ipv4_host_order(const char *text)
{
    uint32_t octets[4]={0u,0u,0u,0u};unsigned part=0u,digits=0u;
    if(text==NULL)return 0u;
    for(unsigned i=0u;i<16u;++i) {
        char c=text[i];
        if(c>='0' && c<='9') {
            if(++digits>3u)return 0u;
            octets[part]=octets[part]*10u+(uint32_t)(c-'0');
            if(octets[part]>255u)return 0u;
        } else if(c=='.' && digits!=0u && part<3u) {++part;digits=0u;}
        else if(c=='\0' && digits!=0u && part==3u)
            return (octets[0]<<24)|(octets[1]<<16)|(octets[2]<<8)|octets[3];
        else return 0u;
    }
    return 0u;
}

bool wifi_transport_configure(const WifiTransportConfig *config)
{
    /* Credentials are installed by the BOOTSEL settings workflow and loaded
     * by the CYW43 service. Runtime reconfiguration is deliberately absent. */
    (void)config;
    return false;
}

void wifi_transport_init(void)
{
    if(g_initialized)return;
    memset(&g_last_status,0,sizeof(g_last_status));
    g_initialized=true;
}

bool wifi_transport_send(unsigned topic,const uint8_t *data,size_t len)
{
    if(!g_initialized || topic>WIFI_TOPIC_HB || len==0u || len>WIFI_PACKET_MAX)return false;
    return drive_bridge_tx_enqueue((uint8_t)topic,data,(uint32_t)len);
}

bool wifi_transport_receive(uint8_t data[8],size_t *original_len)
{
    uint32_t length;
    if(!g_initialized || data==NULL || original_len==NULL ||
       !drive_bridge_rx_take(data,&length))return false;
    *original_len=(size_t)length;
    return true;
}

WifiTransportStatus wifi_transport_status(void)
{
    WifiTransportStatus current;
    if(g_initialized && drive_bridge_status_snapshot(&current))g_last_status=current;
    /* This function is called by the sole controller task. If the shared
     * status mutex is busy, return its previous coherent snapshot. */
    current=g_last_status;
    {
        T_CYW43_UTK_STATUS radio;
        cyw43_utk_get_status(&radio);
        current.configured=(unsigned)utk_network_settings_available();
        current.ready=radio.ready!=0u;
        current.link_status=(int8_t)radio.link_status;
        current.ipv4=((radio.ip_addr&0x000000ffu)<<24)|((radio.ip_addr&0x0000ff00u)<<8)|
                     ((radio.ip_addr&0x00ff0000u)>>8)|((radio.ip_addr&0xff000000u)>>24);
        current.broker_ipv4=ipv4_host_order(utk_network_broker());
        if(radio.link_status!=3) {
            current.connected=current.subscribed=0u;
            current.mqtt_stage=radio.ready?WIFI_STAGE_WIFI:
                (radio.init_complete?WIFI_STAGE_INIT_FAILED:WIFI_STAGE_INIT);
        }
    }
    return current;
}

bool wifi_transport_flash_begin(void)
{
    /* Implemented by the HAL's stopped-only USB-service/core0 park handshake.
     * Never report a fake success if the real rendezvous is unavailable. */
    extern bool utk_flash_begin(void);
    return utk_flash_begin();
}

void wifi_transport_flash_end(void)
{
    extern void utk_flash_end(void);
    utk_flash_end();
}
