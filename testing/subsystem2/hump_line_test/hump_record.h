#ifndef HUMP_RECORD_H
#define HUMP_RECORD_H
#include <stddef.h>
#include "hump_control.h"
#include "../dual_wheel_bench/log_store.h"
#include "../junction_left_test/junction_logic.h" /* Legacy record TYPE only. */

enum { HUMP_ROWS=100 };
#define HUMP_MAGIC 0x31504d48u
typedef struct {
    uint16_t ms;
    int16_t velocity10[2],pwm[2],target[2];
    uint16_t adc[2],level[2],extra[2];
    uint8_t active,decision;
    int32_t ticks[2];
} HumpTrace;
typedef struct {
    uint32_t magic,version,crc,count,dropped;
    uint32_t firmware; /* Uses V1's zero-filled alignment padding. */
    uint64_t duration;
    int32_t cutoff[2];uint32_t invalid[2];
    uint8_t reason,key,offline,profile,recorded_profile,imported;
    LineCalibration calibration;
    HumpTrace rows[HUMP_ROWS];
} HumpSavedLog;
_Static_assert(sizeof(HumpTrace)==36 && sizeof(HumpSavedLog)==3664,"Unexpected hump flash ABI");
_Static_assert(offsetof(HumpSavedLog,rows)==64,"Unexpected hump row offset");
_Static_assert(offsetof(HumpSavedLog,firmware)==20 && offsetof(HumpSavedLog,duration)==24,
               "Hump V1 compatibility changed");
static inline unsigned hump_record_firmware(const HumpSavedLog *v) {
    return v->firmware ? v->firmware : 1;
}
static inline bool hump_record_valid(const HumpSavedLog *v) {
    return v->magic==HUMP_MAGIC && v->version==1 && v->firmware<=5 && v->count<=HUMP_ROWS &&
        v->profile<=HUMP_ASSIST && v->recorded_profile<=HUMP_ASSIST && calibration_valid(&v->calibration) &&
        v->crc==log_crc((const uint8_t *)v+offsetof(HumpSavedLog,count),sizeof(*v)-offsetof(HumpSavedLog,count));
}
/* Preserve the FINAL flash sector (V9 junction data) and import endpoints only.
 * Hump records live in the PENULTIMATE sector. Never write this legacy record. */
typedef struct {
    uint32_t magic,version,crc,count,dropped;
    uint64_t duration;
    int32_t cutoff[2];uint32_t invalid[2];
    uint8_t reason,key,offline,mode;
    LineCalibration calibration;
    SideCalibration side;
    uint8_t selected_mode,firmware;
    Junction junction;
    uint8_t rows[3600];
} HumpLegacyJunction;
_Static_assert(sizeof(HumpLegacyJunction)==3792 && offsetof(HumpLegacyJunction,rows)==192,
               "Legacy V1..V9 junction layout changed");
static inline bool hump_import_junction(const HumpLegacyJunction *v,LineCalibration *c) {
    if(v->magic!=0x314e434au || v->version!=1 || v->count>100 || v->firmware>10 ||
       v->selected_mode<1 || v->selected_mode>3 || v->mode<1 || v->mode>3 ||
       !calibration_valid(&v->calibration) || !side_valid(&v->side) ||
       v->crc!=log_crc((const uint8_t *)v+offsetof(HumpLegacyJunction,count),sizeof(*v)-offsetof(HumpLegacyJunction,count))) return false;
    *c=v->calibration;return true;
}
typedef struct {
    uint16_t ms;int16_t velocity10[2],pwm[2];int32_t ticks[2];
    uint16_t adc[2],level[2];int16_t target[2];uint8_t active,decision;
} HumpLegacyLineTrace;
typedef struct {
    uint32_t magic,version,crc,count,dropped;uint64_t duration;
    int32_t cutoff[2];uint32_t invalid[2];uint8_t reason,key,offline,reserved;
    LineCalibration calibration;uint8_t profile;HumpLegacyLineTrace rows[100];
} HumpLegacyLine;
static inline bool hump_import_line(const HumpLegacyLine *v,LineCalibration *c) {
    if(v->magic!=0x31464c41u || v->version!=1 || v->count>100 || v->profile>5 || v->reserved>6 ||
       !calibration_valid(&v->calibration) ||
       v->crc!=log_crc((const uint8_t *)v+offsetof(HumpLegacyLine,count),sizeof(*v)-offsetof(HumpLegacyLine,count))) return false;
    *c=v->calibration;return true;
}
#endif
