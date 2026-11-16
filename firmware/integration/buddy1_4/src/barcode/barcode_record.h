/**
 * @file barcode_record.h
 * @brief Barcode record.
 * @details Pure barcode/route logic is called by the ADC owner and emits requests.
 * Native records are not wire packets; preserve both decode directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_RECORD_H
#define BARCODE_RECORD_H
#include "src/line_following/circuit_record.h"
#include "src/barcode/barcode_nav.h"
#define B_MAGIC 0x31424343u
typedef struct
{
    circuit_trace_t motion;
    union
    {
        struct
        {
            uint8_t pending;
            uint8_t decoded;
            uint8_t action;
            uint8_t scan;
        }; /* V1-V3 */
        struct
        {
            uint16_t side_adc;
            uint8_t ao_black;
            uint8_t ao_bank;
        }; /* V4-V12 */
    };
} BarcodeTrace;
typedef struct
{
    circuit_summary_t motion;
    uint8_t code;
    uint8_t action;
} barcode_summary_t;
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t crc;
    uint32_t count;
    uint32_t summary_count;
    uint32_t overwritten;
    uint64_t duration_us;
    uint64_t path10;
    int64_t ticks[2];
    uint32_t invalid[2];
    uint32_t turns;
    uint32_t ignored;
    uint8_t reason;
    uint8_t key;
    uint8_t offline;
    uint8_t profile;
    uint8_t recorded_profile;
    uint8_t imported;
    line_calibration_t calibration;
    side_calibration_t side;
    uint32_t event_count;
    uint32_t event_overwritten;
    uint32_t decoded_a;
    uint32_t decoded_f;
    uint32_t unsupported;
    uint32_t rejected;
    uint32_t lefts;
    uint32_t rights;
    uint32_t straight_crosses;
    uint8_t last_code;
    BarcodeTrace rows[B_ROWS];
    barcode_summary_t summaries[C_SUMMARIES];
    barcode_event_t events[B_EVENTS];
    uint32_t decoded_z;
    uint32_t
        uturns; /* V11 extension; V1..V10 prefix remains byte-compatible. */
#ifdef BUDDY_WIFI_LOGGING
    uint32_t
        decoded_total; /* Record26 counts all verified letters, assigned or not. */
#endif
} barcode_saved_t;
_Static_assert(sizeof(BarcodeTrace) == 56 && sizeof(barcode_summary_t) == 40,
               "Barcode trace ABI changed");
#ifdef BUDDY_WIFI_LOGGING
_Static_assert(sizeof(barcode_saved_t) <= 32768,
               "WiFi motion history must fit32KiB");
#else
_Static_assert(offsetof(barcode_saved_t, decoded_z) == 64840 &&
                   sizeof(barcode_saved_t) == 64848,
               "V11 record extension ABI");
#endif
_Static_assert(sizeof(barcode_saved_t) <= C_STORE_BYTES,
               "Barcode record exceeds64KiB");
static inline size_t barcode_record_crc_size (unsigned version)
{
    size_t bytes = version >= 11 ? sizeof(barcode_saved_t)
                                 : offsetof(barcode_saved_t, decoded_z);
    return bytes - offsetof(barcode_saved_t, count);
}
/* Old rows used1000 slots; only the common front-calibration prefix is imported.
 * Caller supplies the complete64KiB flash bank, never a shorter RAM struct. */
static inline bool barcode_import_v25 (const uint8_t * p_bank,
                                       line_calibration_t * p_cal,
                                       uint8_t * p_profile)
{
    const barcode_saved_t * p_state = (const barcode_saved_t *)p_bank;
    if (p_state->magic != B_MAGIC || p_state->version < 1 ||
        p_state->version > 25 || p_state->count > 1000 ||
        p_state->summary_count > C_SUMMARIES ||
        p_state->event_count > B_EVENTS || p_state->profile < 1 ||
        p_state->profile > 3 || !calibration_valid(&p_state->calibration) ||
        !side_valid(&p_state->side) ||
        p_state->crc != log_crc(p_bank + offsetof(barcode_saved_t, count),
                                (p_state->version >= 11 ? 64848u : 64840u) -
                                    offsetof(barcode_saved_t, count)))
    {
        return false;
    }
    *p_cal = p_state->calibration;
    *p_profile = p_state->profile;
    return true;
}
static inline bool barcode_record_valid (const barcode_saved_t * p_state)
{
    return p_state->magic == B_MAGIC &&
#ifdef BUDDY_WIFI_LOGGING
           (p_state->version >= 26 && p_state->version <= 28)
#else
           p_state->version >= 1 && p_state->version <= 25
#endif
           && p_state->count <= B_ROWS &&
           p_state->summary_count <= C_SUMMARIES &&
           p_state->event_count <= B_EVENTS && p_state->profile >= 1 &&
           p_state->profile <= 3 && p_state->recorded_profile >= 1 &&
           p_state->recorded_profile <= 3 &&
           calibration_valid(&p_state->calibration) &&
           side_valid(&p_state->side) &&
           p_state->crc == log_crc((const uint8_t *)p_state +
                                       offsetof(barcode_saved_t, count),
                                   barcode_record_crc_size(p_state->version));
}
#endif

/* End of file. */
