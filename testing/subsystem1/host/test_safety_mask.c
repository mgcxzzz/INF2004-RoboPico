/* Exercise the production Safety event/periodic handlers with host-only
 * kernel/navigation stubs. This does not simulate network delivery. */
#include <stdio.h>
#include <string.h>
#include "../../../app_program/system/safety_supervisor.c"

static uint32_t mask;
static unsigned frames;
static unsigned failures;
static safety_tlm_t latest;

#define CHECK(name, cond) do { \
    printf("%s %s\n", (cond) ? "PASS" : "FAIL", name); \
    if (!(cond)) { failures++; } \
} while (0)

ER tk_get_otm(SYSTIM * t) { t->hi = 0U; t->lo = 100U; return E_OK; }
ID tk_cre_flg(T_CFLG const * c) { (void)c; return 1; }
ID tk_cre_tsk(T_CTSK const * c) { (void)c; return 2; }
ER tk_set_flg(ID id, UINT bits) { (void)id; (void)bits; return E_OK; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT * out, INT timeout)
{
    (void)id; (void)bits; (void)mode; (void)out; (void)timeout;
    return E_TMOUT;
}
uint32_t sys_fault_get(void) { return mask; }
nav_state_t nav_get_state(void) { return NAV_ST_READY; }
ER nav_post_command(nav_cmd_t cmd) { (void)cmd; return E_OK; }
ER nav_post_safety(nav_safety_t event) { (void)event; return E_OK; }
void mc_inhibit(bool inhibit) { (void)inhibit; }
void safety_telemetry_publish(uint8_t const * data, uint32_t len)
{
    if (!safety_tlm_unpack(data, len, &latest)) { failures++; }
    frames++;
}

int main(void)
{
    unsigned before;
    (void)safety_init();
    publish();
    mask = SYS_FAULT_WIFI_LOSS;
    note_warning(SAFETY_SRC_WIFI, 1U);
    CHECK("link-loss warning carries latched mask", latest.fault_mask == mask);

    /* Real ordering: Safety can process RESET before Buddy 1 clears its
     * bit. Safety is already Normal, so no state transition publishes. */
    handle_events(EV_RESET);
    mask = 0U;
    periodic_checks();
    CHECK("Normal reset then owner clear publishes zero mask",
          latest.fault_mask == 0U && latest.state == SAFETY_ST_NORMAL);
    before = frames;
    periodic_checks();
    CHECK("unchanged mask does not flood telemetry", frames == before);

    mask = SYS_FAULT_WIFI_LOSS | SYS_FAULT_IMU;
    periodic_checks();
    CHECK("mask change without Safety transition is published",
          latest.fault_mask == mask);
    handle_events(EV_RESET);
    mask &= ~SYS_FAULT_WIFI_LOSS;
    periodic_checks();
    CHECK("clearing WiFi preserves other owners' bits",
          latest.fault_mask == SYS_FAULT_IMU && mask == SYS_FAULT_IMU);

    mask = SYS_FAULT_WIFI_LOSS;
    enter_estop();
    handle_events(EV_RESET);
    CHECK("ESTOP reset still releases inhibit",
          latest.state == SAFETY_ST_NORMAL && !latest.b_inhibit);
    mask = 0U;
    periodic_checks();
    CHECK("late owner clear after ESTOP reset publishes zero mask",
          latest.fault_mask == 0U);
    printf("Safety mask regression: %u failures\n", failures);
    return failures ? 1 : 0;
}
