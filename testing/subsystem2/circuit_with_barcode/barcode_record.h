#ifndef BARCODE_RECORD_H
#define BARCODE_RECORD_H
#include "../circuit_left_test/circuit_record.h"
#include "barcode_nav.h"
#define B_MAGIC 0x31424343u
typedef struct {
    CircuitTrace motion;
    union {
        struct { uint8_t pending,decoded,action,scan; }; /* V1-V3 */
        struct { uint16_t side_adc;uint8_t ao_black,ao_bank; }; /* V4-V5 */
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
} BarcodeSaved;
_Static_assert(sizeof(BarcodeTrace)==56 && sizeof(BarcodeSummary)==40,"Barcode trace ABI changed");
_Static_assert(sizeof(BarcodeSaved)<=C_STORE_BYTES,"Barcode record exceeds64KiB");
static inline bool barcode_record_valid(const BarcodeSaved *s) {
    return s->magic==B_MAGIC && s->version>=1 && s->version<=5 && s->count<=B_ROWS && s->summary_count<=C_SUMMARIES &&
        s->event_count<=B_EVENTS && s->profile>=1 && s->profile<=3 && s->recorded_profile>=1 && s->recorded_profile<=3 &&
        calibration_valid(&s->calibration) && side_valid(&s->side) &&
        s->crc==log_crc((const uint8_t *)s+offsetof(BarcodeSaved,count),sizeof(*s)-offsetof(BarcodeSaved,count));
}
#endif
