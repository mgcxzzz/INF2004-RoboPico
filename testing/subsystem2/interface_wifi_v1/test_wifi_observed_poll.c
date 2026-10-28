#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#define WIFI_POLL_HOST_TEST 1
enum { ASYNC_CONTEXT_FLAG_POLLED=1 };
typedef struct async_context async_context_t;
typedef struct async_at_time_worker async_at_time_worker_t;
typedef struct async_when_pending_worker async_when_pending_worker_t;
struct async_at_time_worker {void (*do_work)(async_context_t *,async_at_time_worker_t *);};
struct async_when_pending_worker {async_when_pending_worker_t *next;bool work_pending;void (*do_work)(async_context_t *,async_when_pending_worker_t *);};
struct async_context {unsigned flags;async_when_pending_worker_t *when_pending_list;};
static unsigned timed_ready,kind,refreshes,trace[8],trace_n;
static uintptr_t callback;
static async_at_time_worker_t timer;
static async_when_pending_worker_t first,second;
static async_at_time_worker_t *async_context_base_remove_ready_at_time_worker(async_context_t *c)
{(void)c;if(timed_ready){--timed_ready;return &timer;}return NULL;}
static void async_context_base_refresh_next_timeout(async_context_t *c)
{(void)c;assert(kind==3 && callback==0);++refreshes;}
#include "wifi_observed_poll.h"
static void observe(unsigned k,uintptr_t pc){kind=k;callback=pc;}
static void timed_work(async_context_t *c,async_at_time_worker_t *w)
{assert(c && w==&timer && kind==1 && callback==(uintptr_t)timed_work);trace[trace_n++]=1;first.work_pending=true;}
static void pending_first(async_context_t *c,async_when_pending_worker_t *w)
{assert(c && w==&first && !w->work_pending && kind==2 && callback==(uintptr_t)pending_first);trace[trace_n++]=2;second.work_pending=true;}
static void pending_second(async_context_t *c,async_when_pending_worker_t *w)
{assert(c && w==&second && !w->work_pending && kind==2 && callback==(uintptr_t)pending_second);trace[trace_n++]=3;}
int main(void)
{
    timer.do_work=timed_work;first=(async_when_pending_worker_t){&second,false,pending_first};
    second=(async_when_pending_worker_t){NULL,false,pending_second};
    async_context_t context={ASYNC_CONTEXT_FLAG_POLLED,&first};timed_ready=2;
    wifi_observed_poll(&context,observe);
    assert(trace_n==4 && trace[0]==1 && trace[1]==1 && trace[2]==2 && trace[3]==3);
    assert(refreshes==1 && kind==0 && callback==0);
    wifi_observed_poll(&context,observe);assert(trace_n==4 && refreshes==2);
    puts("PASS observed SDK poll ordering, timed catchup, pending flag clearing/new pending work, callback attribution and idle refresh.");
    return 0;
}
