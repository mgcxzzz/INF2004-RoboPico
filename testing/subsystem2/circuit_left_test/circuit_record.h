#ifndef CIRCUIT_RECORD_H
#define CIRCUIT_RECORD_H
#include <stddef.h>
#include "circuit_logic.h"
#include "../dual_wheel_bench/log_store.h"
enum { C_ROWS=1200,C_SUMMARIES=64,C_STORE_BYTES=65536 };
#define C_MAGIC 0x31524343u
typedef struct {
    uint64_t ms;uint32_t path10;int32_t wheel10[2];
    int16_t speed10[2],pwm[2],target[2];uint16_t level[2],adc[2];
    uint32_t turns;uint8_t phase,side,decision,active;
} CircuitTrace;
typedef struct {
    uint64_t ms;uint32_t path10,number,follow_ms;
    uint16_t centre_ms,turn_ms,align_ms;int16_t angle10;
} CircuitSummary;
typedef struct {
    uint32_t magic,version,crc,count,summary_count,overwritten;
    uint64_t duration_us,path10;int64_t ticks[2];uint32_t invalid[2],turns,ignored;
    uint8_t reason,key,offline,profile,recorded_profile,imported;
    LineCalibration calibration;SideCalibration side;
    CircuitTrace rows[C_ROWS];CircuitSummary summaries[C_SUMMARIES];
} CircuitSaved;
_Static_assert(sizeof(CircuitTrace)==48 && sizeof(CircuitSummary)==32,"Circuit trace ABI changed");
_Static_assert(sizeof(CircuitSaved)<=C_STORE_BYTES,"Circuit saved log exceeds64KiB");
static inline bool circuit_record_valid(const CircuitSaved *s) {
    return s->magic==C_MAGIC && s->version>=1 && s->version<=3 && s->count<=C_ROWS && s->summary_count<=C_SUMMARIES &&
        s->profile>=1 && s->profile<=3 && s->recorded_profile>=1 && s->recorded_profile<=3 &&
        calibration_valid(&s->calibration) && side_valid(&s->side) &&
        s->crc==log_crc((const uint8_t *)s+offsetof(CircuitSaved,count),sizeof(*s)-offsetof(CircuitSaved,count));
}
typedef struct {
    uint32_t magic,version,crc,count,dropped;uint64_t duration;
    int32_t cutoff[2];uint32_t invalid[2];uint8_t reason,key,offline,mode;
    LineCalibration calibration;SideCalibration side;uint8_t selected_mode,firmware;
    Junction junction;uint8_t rows[3600];
} CircuitLegacy;
_Static_assert(sizeof(CircuitLegacy)==3792 && offsetof(CircuitLegacy,rows)==192,"V10 import ABI changed");
static inline bool circuit_import(const CircuitLegacy *s,LineCalibration *cal,SideCalibration *side) {
    if(s->magic!=0x314e434au || s->version!=1 || s->firmware>10 || s->count>100 ||
       s->mode<1 || s->mode>3 || s->selected_mode<1 || s->selected_mode>3 ||
       !calibration_valid(&s->calibration) || !side_valid(&s->side) ||
       s->crc!=log_crc((const uint8_t *)s+offsetof(CircuitLegacy,count),sizeof(*s)-offsetof(CircuitLegacy,count))) return false;
    *cal=s->calibration;*side=s->side;return true;
}
#endif
