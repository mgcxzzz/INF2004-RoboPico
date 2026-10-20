/**
 * @file    imu_iface.h
 * @brief   IMU Terrain (Buddy 4): the ONE shared header for everything the
 *          IMU publishes: imu_event_t, motion classes, hump results and
 *          the IMU telemetry frame (team schema, source 4).
 *
 * Tribunal 1 (IS28_ImmTri1):
 *  - R3: "publish the struct in one shared header used by all buddies
 *    instead of duplicated copies": Navigation, WiFi/MQTT and the
 *    dashboard include THIS file; nobody re-declares imu_event_t.
 *  - Buddy 4 stays the single source of imu_event_t and reports all six
 *    motion classes. Climbing, descending and suspected impact come from
 *    the IMU's own tilt and acceleration; stationary, accelerating and
 *    turning come from Buddy 2's motion estimate (imu_motion_get() below),
 *    packed into the same motion field. Nothing is computed twice.
 *  - Minimum bits: every field below is sized to its valid range; values
 *    are clamped before they are stored or packed (R4/R5).
 *
 * This file needs no kernel header, so the host tests include it as is.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#ifndef IMU_IFACE_H
#define IMU_IFACE_H

#include <stdbool.h>
#include <stdint.h>

#include "../system/sys_tlm.h"     /* shared helpers and frame header */

/* ------------------------------------------------------------------------
 * IMU Terrain task state (statechart 6.4). 3 bits.
 * --------------------------------------------------------------------- */
typedef enum
{
    IMU_ST_INIT = 0,        /* Initialising: configure the GY-511       */
    IMU_ST_CALIBRATING,     /* stationary gravity baseline              */
    IMU_ST_IDLE,            /* Sampling: Idle / Standby                 */
    IMU_ST_TRACKING,        /* Sampling: Motion Tracking                */
    IMU_ST_HUMP,            /* Sampling: Hump Processing                */
    IMU_ST_IMPACT,          /* Sampling: Impact / Shock                 */
    IMU_ST_FAULT,           /* Fault / Recovery: data invalid           */
    IMU_ST_COUNT
} imu_state_t;

/* ------------------------------------------------------------------------
 * Motion class (FR-IMU-03). 3 bits. Order = priority when several apply.
 * --------------------------------------------------------------------- */
typedef enum
{
    IMU_MOTION_UNKNOWN = 0,     /* not calibrated, or no data to decide */
    IMU_MOTION_STATIONARY,      /* Buddy 2: |speed| < 10 mm/s           */
    IMU_MOTION_STEADY,          /* moving, none of the classes below    */
    IMU_MOTION_ACCELERATING,    /* Buddy 2: |dv/dt| high (incl. braking) */
    IMU_MOTION_TURNING,         /* Buddy 2: heading rate high           */
    IMU_MOTION_CLIMBING,        /* IMU: nose up                         */
    IMU_MOTION_DESCENDING,      /* IMU: nose down                       */
    IMU_MOTION_IMPACT,          /* IMU: suspected impact (shock)        */
    IMU_MOTION_COUNT
} imu_motion_t;

/* ------------------------------------------------------------------------
 * Event type. 3 bits. A telemetry frame carries the event that caused it,
 * or IMU_EVT_STATUS for the periodic frame.
 * --------------------------------------------------------------------- */
typedef enum
{
    IMU_EVT_STATUS = 0,     /* periodic, nothing new                    */
    IMU_EVT_MOTION,         /* the motion class changed                 */
    IMU_EVT_HUMP_START,     /* FR-IMU-01: a hump was detected           */
    IMU_EVT_HUMP_PEAK,      /* FR-IMU-02: valid hump, height = its peak */
    IMU_EVT_HUMP_REJECTED,  /* profiled but not valid, detail = reason  */
    IMU_EVT_IMPACT,         /* NFR-IMU-03: shock detected               */
    IMU_EVT_FAULT,          /* detail = IMU_FAULT_* code                */
    IMU_EVT_RUN_SUMMARY,    /* run ended: height = highest peak of run  */
    IMU_EVT_COUNT
} imu_evt_type_t;

/* Why a hump estimate was rejected (a false peak is never kept). 3 bits. */
typedef enum
{
    IMU_REJ_NONE = 0,
    IMU_REJ_TIMEOUT,        /* profile took too long or too far         */
    IMU_REJ_NO_DESCENT,     /* went up and stayed up: a ramp            */
    IMU_REJ_RANGE,          /* peak below or above the plausible range  */
    IMU_REJ_NOT_CLOSED,     /* height did not come back to the ground   */
    IMU_REJ_NO_ODOMETRY,    /* no fresh Buddy 2 distance during profile */
    IMU_REJ_SHOCK,          /* a shock invalidated the partial hump     */
    IMU_REJ_STOPPED         /* mission stopped or paused mid-hump       */
} imu_reject_t;

/* Fault and warning codes: 8 bits to safety_report_fault(SAFETY_SRC_IMU,
 * ...), 4 bits in telemetry. */
#define IMU_FAULT_NONE          (0x00U)
#define IMU_FAULT_INIT          (0x01U) /* GY-511 not answering / wrong ID */
#define IMU_FAULT_CAL           (0x02U) /* baseline |g| or axis implausible */
#define IMU_FAULT_CAL_TIMEOUT   (0x03U) /* never still enough to calibrate */
#define IMU_FAULT_I2C           (0x04U) /* I2C reads failing (timeout)     */
#define IMU_FAULT_STALE         (0x05U) /* sensor stopped producing data   */
#define IMU_FAULT_INVALID       (0x06U) /* data stuck (identical for 1 s)  */
#define IMU_WARN_SHOCK          (0x08U) /* warning: shock (NFR-IMU-03)     */

/* Field limits (minimum bits, each sized to its range). */
#define IMU_HEIGHT_MAX_MM       (255)   /* 8 bits, 1 mm: humps the car can
                                         * climb are far below this      */
#define IMU_PITCH_MAX_DD        (900)   /* 11 bits signed, +-90.0 deg   */

/* ------------------------------------------------------------------------
 * imu_event_t: one IMU event, minimum bits (40 bits of data).
 * Used on the chip (event hook below). It never leaves the chip as a C
 * struct: telemetry packs it with masks and shifts (imu_tlm_pack()).
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t type      : 3;     /* imu_evt_type_t                       */
    uint32_t motion    : 3;     /* imu_motion_t at the time             */
    uint32_t detail    : 4;     /* imu_reject_t or IMU_FAULT_* code     */
    uint32_t height_mm : 8;     /* hump peak (or highest peak), mm      */
    int32_t  pitch_dd  : 11;    /* body pitch, 0.1 deg, + = nose up     */
    uint32_t b_valid   : 1;     /* IMU data valid (calibrated, no fault) */
    uint32_t b_odo     : 1;     /* Buddy 2 estimate fresh               */
    uint32_t reserved  : 1;
    uint16_t t_ms;              /* sys_time_us() / 1000, wrapping: the
                                 * shared 1 us clock is the team's
                                 * timestamp base (integration test
                                 * IMU + Motion Control, +-100 ms)      */
} imu_event_t;

/* ------------------------------------------------------------------------
 * Telemetry, source 4 (TLM_SRC_IMU). Payload 16 bytes.
 * Rules as in system/sys_tlm.h: little-endian words, masks and shifts,
 * clamp before packing, reserved bits 0. Packed by imu_tlm_pack()
 * (b4_pack.c). To be moved into sys_tlm.h's "Sources 1, 2 and 4" slot
 * when the branches are merged (see subsystem4/README.md).
 *
 *   word 0  bits  0..2   task state (imu_state_t, 7 values)
 *           bits  3..5   motion class (imu_motion_t, 8 values)
 *           bits  6..8   event that caused this frame (imu_evt_type_t)
 *           bits  9..12  event detail (imu_reject_t or IMU_FAULT_*)
 *           bit  13      IMU data valid
 *           bit  14      Buddy 2 motion estimate fresh
 *           bits 15..25  pitch, SIGNED, dd, -900..+900 (+ = nose up)
 *           bits 26..29  last fault code (IMU_FAULT_*, 0 = none)
 *           bits 30..31  reserved
 *   word 1  bits  0..7   height of the event's hump (or, in a run
 *                        summary, the highest peak), mm, 0..255
 *           bits  8..15  highest valid peak this run, mm, 0..255
 *           bits 16..23  valid humps this run (wrapping)
 *           bits 24..31  rejected humps this run (wrapping)
 *   word 2  bits  0..11  last shock peak, mg from gravity, 0..4095 (sat.)
 *           bits 12..17  last shock detect-to-notify time, ms, 0..63
 *                        (saturates; NFR-IMU-03 limit 50)
 *           bits 18..25  shocks since boot (wrapping)
 *           bits 26..31  I2C errors since boot (wrapping, 6 bits)
 *   word 3  bits  0..15  50 Hz frames missed since boot (wrapping,
 *                        NFR-IMU-01 evidence)
 *           bits 16..31  event time, ms (imu_event_t.t_ms, wrapping)
 * --------------------------------------------------------------------- */
#define IMU_TLM_PAYLOAD_BYTES   (16U)
#define IMU_TLM_FRAME_BYTES     (TLM_HDR_BYTES + IMU_TLM_PAYLOAD_BYTES)

#define IMU_BITS_STATE          (3U)
#define IMU_BITS_MOTION         (3U)
#define IMU_BITS_EVT            (3U)
#define IMU_BITS_DETAIL         (4U)
#define IMU_BITS_PITCH          (11U)
#define IMU_BITS_FAULT          (4U)
#define IMU_BITS_HEIGHT         (8U)
#define IMU_BITS_COUNT8         (8U)
#define IMU_BITS_SHOCK_MG       (12U)
#define IMU_BITS_LATENCY        (6U)
#define IMU_BITS_I2C_ERR        (6U)
#define IMU_BITS_MISSED         (16U)
#define IMU_BITS_TIME           (16U)

/* Decoded view (what the receiver gets back). */
typedef struct
{
    uint8_t  seq;               /* frame sequence (common header)       */
    uint8_t  state;             /* imu_state_t                          */
    uint8_t  motion;            /* imu_motion_t                         */
    uint8_t  evt;               /* imu_evt_type_t                       */
    uint8_t  detail;
    bool     b_valid;
    bool     b_odo;
    int16_t  pitch_dd;
    uint8_t  fault;
    uint8_t  height_mm;
    uint8_t  max_height_mm;
    uint8_t  humps;
    uint8_t  rejected;
    uint16_t shock_mg;
    uint8_t  shock_ms;
    uint8_t  shocks;
    uint8_t  i2c_errors;
    uint16_t missed;
    uint16_t t_ms;
} imu_tlm_t;

void imu_tlm_pack(imu_tlm_t const * p_in,
                  uint8_t p_frame[IMU_TLM_FRAME_BYTES]);
bool imu_tlm_unpack(uint8_t const * p_frame, uint32_t len,
                    imu_tlm_t * p_out);

/* Names for consoles and logs. */
char const * imu_state_name(imu_state_t state);
char const * imu_motion_name(imu_motion_t motion);
char const * imu_evt_name(imu_evt_type_t evt);
char const * imu_reject_name(imu_reject_t reason);

/* ------------------------------------------------------------------------
 * Provided by IMU Terrain (task context only)
 * --------------------------------------------------------------------- */
imu_state_t  imu_get_state(void);
imu_motion_t imu_get_motion(void);
uint8_t      imu_get_highest_peak_mm(void);    /* this run, 0 = none    */
bool         imu_data_valid(void);

/* imu_notify_state(nav_state_t) is declared in system/nav_iface.h: the
 * Navigation Coordinator calls it on every operating-state change. */

/* ------------------------------------------------------------------------
 * Provided by other buddies (weak defaults in b4_stubs.c)
 * --------------------------------------------------------------------- */

/* Motion Control (Buddy 2): the shared motion estimate (mutex-protected,
 * copied out). Same units and signs as IF-16: distance_mm is the signed
 * NET distance, heading_dd + = turned right, both int16 and may WRAP
 * (Buddy 4 only uses differences). age_ms = how old the estimate is;
 * Buddy 4 ignores it above B4_MOTION_FRESH_MS (the team's 50 ms rule).
 * Returns false if there is no estimate at all. */
typedef struct
{
    int16_t  distance_mm;
    int16_t  heading_dd;
    uint16_t age_ms;
} imu_motion_in_t;

bool imu_motion_get(imu_motion_in_t * p_est);

/* WiFi and MQTT (Buddy 1): queue the IMU_TLM_FRAME_BYTES-byte frame for
 * MQTT. Called from the IMU task: copy the bytes and return, never block. */
void imu_telemetry_publish(uint8_t const * p_frame, uint32_t len);

/* Anyone on the chip who wants IMU events as they happen (optional).
 * Called from the IMU task; copy the event and return, never block. */
void imu_event_notify(imu_event_t const * p_evt);

#endif /* IMU_IFACE_H */
