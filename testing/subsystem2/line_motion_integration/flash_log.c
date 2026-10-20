/* RP2040-only, single-core bench log. All code executed with XIP disabled
   lives in .data.flash_write and calls ROM or an SRAM copy of boot2 only.
   Boot2 copy/restore follows Raspberry Pi Pico SDK hardware_flash/flash.c
   (Copyright 2020 Raspberry Pi (Trading) Ltd, BSD-3-Clause).
   Never call while motor outputs are enabled or from interrupt context. */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <string.h>
#include <stddef.h>
#include "integration.h"
#include "log_store.h"
#define OFFSET 0x1ff000u
#define XIP 0x10000000u
_Static_assert(sizeof(BenchLog)<=4096,"flash log overflow");
static union { BenchLog log; uint8_t sector[4096]; } image;
static uint32_t boot2[64];
typedef void (*VoidFn)(void);
typedef void (*EraseFn)(uint32_t,uint32_t,uint32_t,uint8_t);
typedef void (*ProgramFn)(uint32_t,const uint8_t *,uint32_t);
typedef void *(*LookupFn)(uint16_t *,uint32_t);
typedef struct { VoidFn connect,exit_xip,flush,boot; EraseFn erase; ProgramFn program; } Rom;
static uint16_t rom_half(uint32_t address) {
    uint32_t value;
    __asm volatile("ldrh %0, [%1]" : "=r"(value) : "r"(address) : "memory");
    return (uint16_t)value;
}
static void *lookup(char a,char b) {
    LookupFn f=(LookupFn)(uintptr_t)rom_half(0x18);
    uint16_t *table=(uint16_t *)(uintptr_t)rom_half(0x14);
    return f(table,(uint32_t)(uint8_t)a|((uint32_t)(uint8_t)b<<8));
}
__attribute__((section(".data.flash_write"),noinline))
static void write_sector(const Rom *r,const uint8_t *bytes) {
    r->connect(); r->exit_xip();
    r->erase(OFFSET,4096,4096,0x20);
    r->program(OFFSET,bytes,4096);
    r->flush(); r->boot();
}
static bool valid(const BenchLog *l) {
    return l->magic==BENCH_MAGIC && l->version==BENCH_VERSION && l->count>0 && l->count<=BENCH_ROWS && l->arm_count<=32
        && l->crc==log_crc((const uint8_t *)l+offsetof(BenchLog,count),sizeof(*l)-offsetof(BenchLog,count));
}
bool bench_flash_load(BenchLog *l) {
    const BenchLog *p=(const BenchLog *)(XIP+OFFSET);
    if(!valid(p)) return false;
    memcpy(l,p,sizeof(*l)); return true;
}
bool bench_flash_save(BenchLog *l) {
    Rom r={(VoidFn)lookup('I','F'),(VoidFn)lookup('E','X'),(VoidFn)lookup('F','C'),
           (VoidFn)((uintptr_t)boot2|1u),(EraseFn)lookup('R','E'),(ProgramFn)lookup('R','P')};
    if(!r.connect || !r.exit_xip || !r.flush || !r.erase || !r.program) return false;
    for(unsigned i=0;i<64;++i) boot2[i]=((volatile uint32_t *)XIP)[i];
    memset(&image,0,sizeof(image));
    l->magic=BENCH_MAGIC; l->version=BENCH_VERSION;
    l->crc=log_crc((const uint8_t *)l+offsetof(BenchLog,count),sizeof(*l)-offsetof(BenchLog,count));
    memcpy(&image.log,l,sizeof(*l));
    UINT imask;
    DI(imask);
    __asm volatile("dmb" ::: "memory");
    write_sector(&r,image.sector);
    __asm volatile("dmb" ::: "memory");
    EI(imask);
    return valid((const BenchLog *)(XIP+OFFSET)) && memcmp((const void *)(XIP+OFFSET),l,sizeof(*l))==0;
}
