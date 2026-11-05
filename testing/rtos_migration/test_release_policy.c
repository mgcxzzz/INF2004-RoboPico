#define APP_RTOS_RELEASE 1
#include <assert.h>
#include <stdio.h>
#include "app_program/network_bridge/connected_policy.h"
int main(void) {
    unsigned ready=BR_READY|BR_CLOCK_OK;
    assert(br_can_start(ready,2000000,0,0,true));
    assert(br_outputs_allowed(ready|BR_ACTIVE,false,0,true));
    assert(!br_can_start(ready,2000000,0,0,false));
    assert(!br_can_start(ready|BR_ESTOP,2000000,0,0,true));
    assert(!br_outputs_allowed(ready|BR_ACTIVE,true,0,true));
    assert(!br_outputs_allowed(ready|BR_ACTIVE|BR_ENCODER,false,0,true));
    assert(!br_outputs_allowed(BR_ACTIVE,false,0,true));
    RnStatus s={.link=3,.flags=RN_CONNECTED|RN_SUBSCRIBED,.polls=1,.last_poll_ms=1000};
    assert(rn_link_usable(&s,1250));assert(!rn_link_usable(&s,1251));
    s.last_poll_ms=4500;s.last_complete_ms=1500;
    assert(rn_delivery_fresh(&s,4500,1000));
    s.last_poll_ms=4501;assert(!rn_delivery_fresh(&s,4501,1000));
    s.last_complete_ms=4501;assert(!rn_delivery_fresh(&s,4501,4502));
    assert(rn_delivery_fresh(&s,4501,4500));
    s.flags=RN_CONNECTED;assert(!rn_link_usable(&s,1001));
    assert(!rn_request_current(1600,1000,1,1,1,1));
    assert(!rn_request_current(1100,1000,1,2,1,1));
    puts("PASS powerbank policy: no USB requirement; clock, buttons, latches and fresh network/command guards retained.");
}
