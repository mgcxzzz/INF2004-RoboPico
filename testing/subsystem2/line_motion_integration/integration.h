#ifndef INTEGRATION_H
#define INTEGRATION_H
#include <stdint.h>
#include <stdbool.h>
#define BENCH_ROWS 80
#define BENCH_MAGIC 0x31493332u
#define BENCH_VERSION 2
/* This format is deliberately distinct from the earlier standalone logs. */
typedef struct {
    uint32_t ms;
    uint8_t state,buttons,usb,reject;
} BenchArmEvent;
typedef struct {
    int32_t ticks[2];
    int16_t speed10[2],pwm[2],requested[2],target[2];
    uint16_t ms;
    uint8_t raw,sensors,nav,owner,line,active;
} BenchRow;
typedef struct {
    uint32_t magic,version,crc,count,dropped,duration_us,reason;
    int32_t cutoff[2];
    uint32_t invalid[2], accepted;
    uint32_t arm_count;
    BenchArmEvent arm[32];
    BenchRow row[BENCH_ROWS];
} BenchLog;
bool bench_flash_load(BenchLog *log);
bool bench_flash_save(BenchLog *log);
#endif
