#include <assert.h>
#include <stdio.h>
#include "app_program/line_navigation/adc_reader.h"

static volatile uint32_t cs, result;
static unsigned scenario, time_reads;
static uint64_t now_mock(void) {
    if (time_reads++ == 0) {
        if (scenario == 0 || scenario == 3)
            cs |= RL_ADC_READY | (scenario == 3 ? RL_ADC_ERR : 0);
        return UINT64_C(0xfffffff0);
    }
    /* Conversion completes while the task is off CPU between READY and its
     * deadline check; cross a 32-bit timer boundary as well. */
    if (scenario == 1 || scenario == 4)
        cs |= RL_ADC_READY | (scenario == 4 ? RL_ADC_ERR : 0);
    return UINT64_C(0xfffffff0) + 1000;
}

static void conversions(void) {
    for (scenario = 0; scenario < 5; ++scenario) {
        cs = 1;
        result = 1234;
        time_reads = 0;
        uint16_t value = 4321;
        uint32_t elapsed = 0, observed = 0, avoided = 0;
        RlAdcPort p = {.cs = &cs, .result = &result, .now = now_mock};
        RlAdcReason reason = rl_adc_conversion(&p, &value, &elapsed, &observed, &avoided);
        if (scenario == 0 || scenario == 1) {
            assert(reason == RL_ADC_NONE && value == 1234 && (observed & RL_ADC_READY));
            assert(avoided == (scenario == 1 ? 1U : 0U));
        } else {
            assert(value == 4321); /* Failure never publishes an invalid value. */
            assert(reason == (scenario == 2 ? RL_ADC_TIMEOUT : RL_ADC_CONVERSION_ERR));
            assert(elapsed == 1000);
            assert((observed & RL_ADC_ERR) == (scenario == 2 ? 0U : RL_ADC_ERR));
        }
    }
}

static void recovery(void) {
    RlAdcHealth h = {.state = RL_ADC_QUALIFYING};
    assert(!rl_adc_good_frame(&h) && h.good == 1);
    assert(!rl_adc_good_frame(&h) && h.good == 2);
    assert(rl_adc_good_frame(&h) && h.state == RL_ADC_HEALTHY && h.good == 3);
    rl_adc_failed(&h, 1000);
    assert(h.good == 0 && h.state == RL_ADC_OFFLINE);
    assert(!rl_adc_can_retry(&h, 100999, false));
    assert(!rl_adc_can_retry(&h, 101000, true)); /* Never reset the ADC while moving. */
    assert(rl_adc_can_retry(&h, 101000, false));
    assert(!rl_adc_good_frame(&h));
    h = (RlAdcHealth){.state = RL_ADC_QUALIFYING};
    assert(!rl_adc_good_frame(&h));
    rl_adc_failed(&h, 200000);
    assert(h.good == 0 && !rl_adc_good_frame(&h)); /* Invalid frame resets qualification. */
    h = (RlAdcHealth){.state = RL_ADC_QUALIFYING};
    assert(!rl_adc_good_frame(&h));
    assert(!rl_adc_good_frame(&h));
    assert(rl_adc_good_frame(&h));
}

int main(void) {
    conversions();
    recovery();
    puts(
        "PASS ADC: normal/preempted conversions, true timeout, ERR including after preemption, unchanged failed output, timer wrap, stopped-only retry and three-frame qualification.");
    return 0;
}
