/* Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 * Adapted from pico_async_context/async_context_base.c execute_once in Pico SDK
 * a1438dff1d38bd9c65dbd693f0e5db4b9ae91779. See LICENSE.pico-sdk.txt.
 * Same timed-then-pending execution and timeout refresh; adds callback markers.
 */
#ifndef WIFI_OBSERVED_POLL_H
#define WIFI_OBSERVED_POLL_H
#include <assert.h>
#include <stdint.h>
#ifndef WIFI_POLL_HOST_TEST
#include "pico/async_context.h"
#include "pico/async_context_base.h"
#endif
static inline void wifi_observed_poll(async_context_t *context,
                                     void (*observe)(unsigned,uintptr_t))
{
    assert(context->flags & ASYNC_CONTEXT_FLAG_POLLED);
    async_at_time_worker_t *timed;
    while((timed=async_context_base_remove_ready_at_time_worker(context))!=NULL) {
        observe(1u,(uintptr_t)timed->do_work);
        timed->do_work(context,timed);
        observe(0u,0u);
    }
    for(async_when_pending_worker_t *pending=context->when_pending_list;pending;pending=pending->next) {
        if(pending->work_pending) {
            pending->work_pending=false;
            observe(2u,(uintptr_t)pending->do_work);
            pending->do_work(context,pending);
            observe(0u,0u);
        }
    }
    observe(3u,0u);
    async_context_base_refresh_next_timeout(context);
    observe(0u,0u);
}
#endif
