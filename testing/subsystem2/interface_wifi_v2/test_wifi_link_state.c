#include <assert.h>
#include <stdio.h>
enum { CYW43_ITF_STA=0, CYW43_LINK_DOWN=0, CYW43_LINK_JOIN=1,
       CYW43_LINK_NOIP=2, CYW43_LINK_UP=3, CYW43_LINK_BADAUTH=-3 };
static int cyw43_state;
static int tcpip_state,queries;
static int cyw43_tcpip_link_status(int *state,int itf)
{
    assert(state==&cyw43_state && itf==CYW43_ITF_STA);
    ++queries;return tcpip_state;
}
#include "wifi_link_state.h"
int main(void)
{
    /* A station can have joined the AP while DHCP is still outstanding. */
    const int states[]={CYW43_LINK_DOWN,CYW43_LINK_JOIN,CYW43_LINK_NOIP,
                       CYW43_LINK_UP,CYW43_LINK_BADAUTH};
    for(unsigned i=0;i<sizeof(states)/sizeof(states[0]);++i) {
        tcpip_state=states[i];int observed=wifi_network_link_status();
        assert(observed==states[i]);
        assert((observed==CYW43_LINK_UP)==(i==3));
    }
    assert(queries==5);
    puts("PASS TCP/IP readiness query: joined/no-IP do not authorize MQTT; DHCP-ready does; auth failure preserved.");
    return 0;
}
