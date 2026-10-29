#ifndef BARCODE_RECORD_H
#define BARCODE_RECORD_H
#include "../circuit_left_test/circuit_record.h"
#include "barcode_nav.h"
#define B_MAGIC 0x31424343u
typedef struct {
    CircuitTrace motion;
    union {
        struct { uint8_t pending,decoded,action,scan; }; /* V1-V3 */
        struct { uint16_t side_adc;uint8_t ao_black,ao_bank; }; /* V4-V12 */
    };
} BarcodeTrace;
typedef struct { CircuitSummary motion;uint8_t code,action; } BarcodeSummary;
typedef struct {
    uint32_t magic,version,crc,count,summary_count,overwritten;
    uint64_t duration_us,path10;int64_t ticks[2];uint32_t invalid[2],turns,ignored;
    uint8_t reason,key,offline,profile,recorded_profile,imported;
    LineCalibration calibration;SideCalibration side;
    uint32_t event_count,event_overwritten,decoded_a,decoded_f,unsupported,rejected,lefts,rights,straight_crosses;
    uint8_t last_code;
    BarcodeTrace rows[B_ROWS];BarcodeSummary summaries[C_SUMMARIES];BarcodeEvent events[B_EVENTS];
    uint32_t decoded_z,uturns; /* V11 extension; V1..V10 prefix remains byte-compatible. */
#ifdef BUDDY_WIFI_LOGGING
    uint32_t decoded_total; /* Record26 counts all verified letters, assigned or not. */
#endif
} BarcodeSaved;
_Static_assert(sizeof(BarcodeTrace)==56 && sizeof(BarcodeSummary)==40,"Barcode trace ABI changed");
#ifdef BUDDY_WIFI_LOGGING
_Static_assert(sizeof(BarcodeSaved)<=32768,"WiFi motion history must fit32KiB");
#else
_Static_assert(offsetof(BarcodeSaved,decoded_z)==64840 && sizeof(BarcodeSaved)==64848,"V11 record extension ABI");
#endif
_Static_assert(sizeof(BarcodeSaved)<=C_STORE_BYTES,"Barcode record exceeds64KiB");
static inline size_t barcode_record_crc_size(unsigned version) {
    size_t bytes=version>=11?sizeof(BarcodeSaved):offsetof(BarcodeSaved,decoded_z);
    return bytes-offsetof(BarcodeSaved,count);
}
/* Old rows used1000 slots; only the common front-calibration prefix is imported.
 * Caller supplies the complete64KiB flash bank, never a shorter RAM struct. */
static inline bool barcode_import_v25(const uint8_t *bank,LineCalibration *cal,uint8_t *profile) {
    const BarcodeSaved *s=(const BarcodeSaved *)bank;
    if(s->magic!=B_MAGIC || s->version<1 || s->version>25 || s->count>1000 ||
       s->summary_count>C_SUMMARIES || s->event_count>B_EVENTS ||
       s->profile<1 || s->profile>3 || !calibration_valid(&s->calibration) || !side_valid(&s->side) ||
       s->crc!=log_crc(bank+offsetof(BarcodeSaved,count),
           (s->version>=11?64848u:64840u)-offsetof(BarcodeSaved,count)))return false;
    *cal=s->calibration;*profile=s->profile;return true;
}
static inline bool barcode_record_valid(const BarcodeSaved *s) {
    return s->magic==B_MAGIC &&
#ifdef BUDDY_WIFI_LOGGING
        ((s->version>=26 && s->version<=28) || s->version==30)
#else
        s->version>=1 && s->version<=25
#endif
        && s->count<=B_ROWS && s->summary_count<=C_SUMMARIES &&
        s->event_count<=B_EVENTS && s->profile>=1 && s->profile<=3 && s->recorded_profile>=1 && s->recorded_profile<=3 &&
        calibration_valid(&s->calibration) && side_valid(&s->side) &&
        s->crc==log_crc((const uint8_t *)s+offsetof(BarcodeSaved,count),barcode_record_crc_size(s->version));
}
#endif
