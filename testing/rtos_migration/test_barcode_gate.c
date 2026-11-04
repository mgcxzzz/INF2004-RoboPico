#define APP_RTOS_BARCODE 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
#include "app_program/barcode_navigation/barcode_model.h"

static const LineCalibration cal = {{122, 138}, {246, 267}, 7};
static void prepare(Circuit *c, BarcodeNav *n, CircuitInput *i, char action) {
    *i = (CircuitInput){
        .now = 1000000, .wheel = {100, 100}, .path = 100, .speed = {0, 8}, .adc = {246, 267}};
    circuit_begin(c, i);
    *n = (BarcodeNav){.start_us = i->now,
                      .pending = 'F',
                      .pending_action = action,
                      .pending_revision = 2,
                      .cross_ready = true,
                      .cross_gate = false,
                      .cross_width = 12.9,
                      .cross_us = i->now,
                      .cross_wheel = {80, 80},
                      .external_reader = true,
                      .external_cross = true};
}
int main(void) {
    const char actions[] = "LRUS";
    for (unsigned k = 0; k < 4; ++k) {
        Circuit c;
        BarcodeNav n;
        CircuitInput i;
        prepare(&c, &n, &i, actions[k]);
        /* Gate history from earlier correction is intentionally ineligible;
         * first tick has no300ms dwell and one measured wheel below10mm/s. */
        AnalogDemand d = barcode_circuit_step(&c, &n, &cal, &i, 1);
        assert(!d.stop);
        if (BARCODE_CONFIRM_GATE) {
            assert(!c.ignored && !n.pending);
            if (actions[k] == 'S')
                assert(!n.busy && n.straight_crosses == 1);
            else
                assert(n.busy && n.executing_action == actions[k] && n.executing_revision == 2);
        } else {
            assert(c.ignored == 1 && !n.busy && n.pending == 'F');
            assert(n.events[n.event_count - 1].type == B_GATE_REJECT);
            assert(n.events[n.event_count - 1].flags == 0); /* SDK event parity. */
        }
    }
    Circuit c;
    BarcodeNav n;
    CircuitInput i;
    prepare(&c, &n, &i, 'L');
    n.cross_gate = true;
    c.clear_path = 101; /* 150mm post-turn lockout is retained. */
    (void)barcode_circuit_step(&c, &n, &cal, &i, 1);
    assert(c.ignored == 1 && n.pending == 'F' && !n.busy);
    assert(n.events[n.event_count - 1].type == B_GATE_REJECT);
    assert(n.events[n.event_count - 1].flags == (BARCODE_CONFIRM_GATE ? B_GATE_CLEARANCE : 0));
    prepare(&c, &n, &i, 'L');
    n.cross_ready = false;
    (void)barcode_circuit_step(&c, &n, &cal, &i, 1);
    assert(!n.busy && n.pending == 'F'); /* A decode alone never turns. */
    AnalogDemand d = {.level = {0, 0}};
    c.clear_path = i.path + 1;
    c.follower.core.white = true;
    c.follower.search.active = true;
    unsigned flags = barcode_cross_rejection(&c, &n, &i, &d);
    assert(flags == (BARCODE_CONFIRM_GATE ? 30U : 7U));
    c.clear_path = 0;
    c.follower.core.white = false;
    c.follower.search.active = false;
    d.level[0] = 400;
    n.cross_gate = false;
    assert(barcode_cross_rejection(&c, &n, &i, &d) ==
           (BARCODE_CONFIRM_GATE ? 0U : B_GATE_APPROACH));
    printf(
        "PASS barcode gate policy=%s: all actions, stale leading veto, low speed/no dwell, clearance, visibility/search/lost reason bits, decode-alone rejection\n",
        BARCODE_CONFIRM_GATE ? "confirmation" : "legacy");
    return 0;
}
