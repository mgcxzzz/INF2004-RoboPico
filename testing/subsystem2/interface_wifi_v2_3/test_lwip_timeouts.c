/* Real SDK lwIP timeout list + fixed-pool allocator. Only platform/clock and
 * unrelated protocol timer bodies are stubbed; no network/radio is simulated. */
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lwip/opt.h"
#include "netif/ppp/ppp_opts.h"
#include "lwip/timeouts.h"
#include "lwip/memp.h"
#include "lwip/priv/tcp_priv.h"
#include "lwip/sys.h"
static jmp_buf panic_jump;
static bool expect_exhaustion;
static unsigned app_calls;
static u32_t clock_ms;
struct tcp_pcb *tcp_active_pcbs;
struct tcp_pcb *tcp_tw_pcbs;
volatile u8_t pbuf_free_ooseq_pending;
void pbuf_free_ooseq(void) {}
void tcp_tmr(void) {}
void ip_reass_tmr(void) {}
void etharp_tmr(void) {}
void dhcp_coarse_tmr(void) {}
void dhcp_fine_tmr(void) {}
void acd_tmr(void) {}
void igmp_tmr(void) {}
void dns_tmr(void) {}
u32_t sys_now(void) { return clock_ms; }
sys_prot_t sys_arch_protect(void) { return 0; }
void sys_arch_unprotect(sys_prot_t previous) { (void)previous; }
void test_lwip_assert(const char *message) {
    if (expect_exhaustion && strstr(message,"MEMP_SYS_TIMEOUT is empty")) longjmp(panic_jump,1);
    fprintf(stderr,"Unexpected lwIP assertion: %s\n",message);abort();
}
static void app_timer(void *arg) {
    app_calls++;
    sys_timeout(1000u,app_timer,arg); /* Same lifetime as MQTT cyclic timer. */
}
int main(int argc,char **argv) {
    expect_exhaustion=argc==2 && strcmp(argv[1],"--expect-exhaustion")==0;
    memp_init();
    sys_timeouts_init();
    /* Trigger the core TCP timer without constructing a real TCP connection. */
    tcp_active_pcbs=(struct tcp_pcb *)(void *)&clock_ms;
    tcp_timer_needed();
    if (setjmp(panic_jump)) {
        puts("PASS baseline reproduces exact MEMP_SYS_TIMEOUT-empty panic when app timer starts");
        return 0;
    }
    sys_timeout(1000u,app_timer,NULL);
    if (expect_exhaustion) { fputs("Baseline unexpectedly had spare timeout capacity\n",stderr);return 1; }
    assert(MEMP_NUM_SYS_TIMEOUT>=LWIP_NUM_SYS_TIMEOUT_INTERNAL+4);
    for (unsigned i=0;i<1000;i++) {
        clock_ms+=100u;
        sys_check_timeouts();
    }
    assert(app_calls==100u);
    sys_untimeout(app_timer,NULL);
    for (unsigned i=0;i<1000;i++) {
        sys_timeout(1000u,app_timer,NULL);
        sys_untimeout(app_timer,NULL);
    }
    printf("PASS real lwIP timeout pool: core=%u capacity=%u, 100 app callbacks, 1000 add/remove cycles\n",
           (unsigned)LWIP_NUM_SYS_TIMEOUT_INTERNAL,(unsigned)MEMP_NUM_SYS_TIMEOUT);
    return 0;
}
