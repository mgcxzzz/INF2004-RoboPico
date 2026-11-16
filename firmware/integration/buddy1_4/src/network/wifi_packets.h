/**
 * @file wifi_packets.h
 * @brief Wifi packets.
 * @details Network helpers validate and explicitly encode bounded application records.
 * Transport calls belong to the configured SDK or RTOS owner, not these codecs.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef WIFI_PACKETS_H
#define WIFI_PACKETS_H

#include <stdbool.h>
#include <stdint.h>

#define WIFI_PACKET_MAX 64u
#define WPK_HEADER_BYTES 3u
#define WPK_MOTION_PAYLOAD_BYTES 45u
#define WPK_LINE_PAYLOAD_BYTES 21u
#define WPK_NAV_PAYLOAD_BYTES 23u
#define WPK_SUMMARY_PAYLOAD_BYTES 22u
#define WPK_HEARTBEAT_PAYLOAD_BYTES 12u
#define WPK_DIAGNOSTICS_PAYLOAD_BYTES 60u
#define WPK_LOG_CHUNK_HEADER_BYTES 12u
#define WPK_LOG_CHUNK_DATA_MAX 49u
#define WPK_MOTION_BYTES (WPK_HEADER_BYTES + WPK_MOTION_PAYLOAD_BYTES)
#define WPK_LINE_BYTES (WPK_HEADER_BYTES + WPK_LINE_PAYLOAD_BYTES)
#define WPK_NAV_BYTES (WPK_HEADER_BYTES + WPK_NAV_PAYLOAD_BYTES)
#define WPK_SUMMARY_BYTES (WPK_HEADER_BYTES + WPK_SUMMARY_PAYLOAD_BYTES)
#define WPK_HEARTBEAT_BYTES (WPK_HEADER_BYTES + WPK_HEARTBEAT_PAYLOAD_BYTES)
#define WPK_DIAGNOSTICS_BYTES (WPK_HEADER_BYTES + WPK_DIAGNOSTICS_PAYLOAD_BYTES)

enum
{
    WPK_SRC_HEARTBEAT = 1u,
    WPK_SRC_MAP = 8u,
    WPK_SRC_MOTION = 9u,
    WPK_SRC_LINE = 10u,
    WPK_SRC_NAV = 11u,
    WPK_SRC_SUMMARY = 12u,
    WPK_SRC_LOG_CHUNK = 13u,
    WPK_SRC_DIAGNOSTICS = 14u
};
enum
{
    WPK_EVENT_STATUS = 0u,
    WPK_EVENT_RUN_BEGIN = 1u,
    WPK_EVENT_DECODED = 2u,
    WPK_EVENT_QUEUED = 3u,
    WPK_EVENT_JUNCTION_CONFIRMED = 4u,
    WPK_EVENT_DISPATCH = 5u,
    WPK_EVENT_COMPLETED = 6u,
    WPK_EVENT_RECOVERY_WAIT = 7u,
    WPK_EVENT_RECOVERY_SWEEP = 8u,
    WPK_EVENT_RECOVERY_FOUND = 9u,
    WPK_EVENT_STOP = 10u,
    WPK_EVENT_MAP_APPLIED = 11u,
    WPK_EVENT_COMMAND_REQUEST = 12u,
    WPK_EVENT_COMMAND_RESULT = 13u,
    WPK_EVENT_FAULT = 14u,
    WPK_EVENT_UNASSIGNED = 15u
};
enum
{
    WPK_LOG_STATUS_OK = 0u,
    WPK_LOG_STATUS_BUSY = 1u,
    WPK_LOG_STATUS_RANGE = 2u,
    WPK_LOG_STATUS_UNAVAILABLE = 3u
};

/* Source 1 payload is Buddy 1 v2, preserved for existing dashboard clients. */
typedef struct
{
    uint8_t seq;
    uint8_t net;
    uint8_t state;
    uint8_t last_cmd;
    uint8_t last_result;
    uint16_t uptime_s;
    uint16_t published;
    uint16_t lost;
    uint8_t accepted;
    uint8_t rejected;
    uint8_t connects;
    uint8_t fill_pct;
} wifi_heartbeat_t;

/* Source 9, payload offsets: uptime_ms u32@0; elapsed_ms u32@4; run_id
 * u16@8; phase/profile/flags/recovery_state@10..13; measured speed L/R
 * i16 in 0.1 mm/s @14/@16; target L/R i16 mm/s @18/@20; PWM L/R i16
 * @22/@24; ADC L/R/barcode u16 @26/@28/@30; calibrated line levels L/R
 * u16 0..1000 or 0xffff unavailable @32/@34; barcode calibration is
 * explicitly unavailable (0xffff) @36; line bitmask@38; distance_mm u32@39; heading centidegrees
 * i16 wrapped to +/-18000 @43. */
typedef struct
{
    uint8_t seq;
    uint8_t phase;
    uint8_t profile;
    uint8_t flags;
    uint8_t recovery_state;
    uint8_t line_levels;
    uint32_t uptime_ms;
    uint32_t elapsed_ms;
    uint32_t distance_mm;
    uint16_t run_id;
    uint16_t adc_left;
    uint16_t adc_right;
    uint16_t adc_barcode;
    uint16_t calibrated_level_left;
    uint16_t calibrated_level_right;
    int16_t speed_left_0p1mm_s;
    int16_t speed_right_0p1mm_s;
    int16_t target_left_mm_s;
    int16_t target_right_mm_s;
    int16_t pwm_left;
    int16_t pwm_right;
    int16_t heading_cdeg;
} wifi_motion_packet_t;

/* Source 10, payload offsets: uptime_ms@0,event@4, ADC left/right/barcode
 * u16 @5/@7/@9, calibrated line levels L/R u16 0..1000 or 0xffff unavailable @11/@13, last decoded
 * letter@15, pending letter/action/revision@16..18, executing
 * action/revision@19..20. */
typedef struct
{
    uint8_t seq;
    uint8_t event;
    uint8_t last_decoded_letter;
    uint8_t pending_letter;
    uint8_t pending_action;
    uint8_t pending_revision;
    uint8_t executing_action;
    uint8_t executing_revision;
    uint32_t uptime_ms;
    uint16_t adc_left;
    uint16_t adc_right;
    uint16_t adc_barcode;
    uint16_t calibrated_level_left;
    uint16_t calibrated_level_right;
} wifi_line_packet_t;

/* Source 11 event/status, payload offsets: uptime_ms@0,event@4,letter@5,
 * action@6,map_revision@7,command_id u32@8,generation u32@12,
 * fault_bitmap u16@16,owner/mode/command_state@18..20,detail u16@21. */
typedef struct
{
    uint8_t seq;
    uint8_t event;
    uint8_t letter;
    uint8_t action;
    uint8_t map_revision;
    uint8_t owner;
    uint8_t mode;
    uint8_t command_state;
    uint32_t uptime_ms;
    uint32_t command_id;
    uint32_t generation;
    uint16_t fault_bitmap;
    uint16_t detail;
} wifi_nav_packet_t;

typedef struct
{
    uint8_t seq;
    uint8_t result;
    uint8_t flags;
    uint32_t uptime_ms;
    uint32_t elapsed_ms;
    uint32_t distance_mm;
    uint16_t run_id;
    uint16_t line_events;
    uint16_t nav_events;
    uint16_t recoveries;
} wifi_summary_packet_t;

/* Source 14 diagnostics/status, payload offsets: uptime_ms u32@0,
 * threshold_adc u16@4,bank@6,flags@7; decoded_total/turns/lefts/rights/
 * uturns/bad_frames/unsupported/max_adc_gap_us/max_control_gap_us/tx_dropped/
 * rx_dropped/invalid_left/invalid_right are u32 at @8,@12,@16,@20,@24,@28,
 * @32,@36,@40,@44,@48,@52,@56. Flags: scanning,pending,front-cal-valid,
 * run-active in bits0..3; upper bits reserved. Threshold<=4095, bank<=62. */
typedef struct
{
    uint8_t seq;
    uint8_t bank;
    uint8_t flags;
    uint32_t uptime_ms;
    uint16_t threshold_adc;
    uint32_t decoded_total;
    uint32_t turns;
    uint32_t lefts;
    uint32_t rights;
    uint32_t uturns;
    uint32_t bad_frames;
    uint32_t unsupported;
    uint32_t max_adc_gap_us;
    uint32_t max_control_gap_us;
    uint32_t tx_dropped;
    uint32_t rx_dropped;
    uint32_t invalid_left;
    uint32_t invalid_right;
} wifi_diagnostics_packet_t;

/* All packers return false for a null pointer or an invalid semantic field.
 * Integers are encoded little-endian; each frame starts with v2 common header. */
bool wpk_heartbeat (const wifi_heartbeat_t * p_in_value,
                    uint8_t out[WPK_HEARTBEAT_BYTES]);
bool wpk_motion (const wifi_motion_packet_t * p_in_value,
                 uint8_t out[WPK_MOTION_BYTES]);
bool wpk_line (const wifi_line_packet_t * p_in_value,
               uint8_t out[WPK_LINE_BYTES]);
bool wpk_nav (const wifi_nav_packet_t * p_in_value, uint8_t out[WPK_NAV_BYTES]);
bool wpk_summary (const wifi_summary_packet_t * p_in_value,
                  uint8_t out[WPK_SUMMARY_BYTES]);
bool wpk_diagnostics (const wifi_diagnostics_packet_t * p_in_value,
                      uint8_t out[WPK_DIAGNOSTICS_BYTES]);
/* Source 13 row-based chunk response; payload offsets are dataset@0,
 * request_id@1,row_index u16@2,total_rows u16@4,part_index@6,part_count@7,
 * status@8,data_len@9,reserved zero u16@10,data@12. */
bool wpk_log_chunk (uint8_t seq, uint8_t dataset, uint8_t request_id,
                    uint16_t row_index, uint16_t total_rows, uint8_t part_index,
                    uint8_t part_count, uint8_t status, const uint8_t * p_data,
                    uint8_t length, uint8_t out[WIFI_PACKET_MAX],
                    uint8_t * p_out_len);

#endif

/* End of file. */
