/* Frozen native layouts from tested main 7b18596. Comparison only; never serialized. */
#ifndef BASELINE_NATIVE_LAYOUT_V026_H
#define BASELINE_NATIVE_LAYOUT_V026_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t trip_ms, sample_ms;
    uint16_t events;
    signed up_mg : 13;
    unsigned norm_mg : 12;
    unsigned reason : 2, latched : 1, armed : 1, fresh : 1, can_reset : 1, saturated : 1;
    unsigned up_ms : 5, tilt_ms : 7, upright_ms : 9;
} baseline_lift_packet_t;
typedef struct {
    signed up_projection_mg : 13; /* current projection on calibrated gravity */
    unsigned norm_mg : 12;         /* current acceleration magnitude */
    int16_t min_cos_q15;           /* minimum dot/(gravity*norm) among gravity-like samples */
    uint16_t min_cos_norm_mg;      /* magnitude at that valid minimum-cosine sample */
    int16_t peak_up_projection_mg; /* lifetime maximum calibrated-up projection */
    uint16_t event_count;      /* saturating lifetime event count */
    uint32_t event_time_ms;    /* first sample that qualified, wraps naturally */
    uint32_t min_cos_sample_ms; /* hardware timer epoch; wraps naturally */
    uint32_t peak_window_start_ms; /* first valid sample since reset */
    uint32_t peak_up_samples : 4; /* greatest upward dwell seen since reset */
    uint32_t peak_tilt_samples : 6; /* greatest tilt dwell seen since reset */
    uint32_t up_samples : 4;      /* consecutive upward-acceleration samples, <= 12 */
    uint32_t tilt_samples : 6;    /* consecutive >=45-degree samples, <= 40 */
    uint32_t upright_samples : 8; /* consecutive reset-ready samples, <= 200 */
    uint32_t reason : 2;          /* 1 = up accel, 2 = tilt; 3 reserved */
    uint32_t last_candidate : 2;  /* latest nonzero candidate since Reset */
    uint32_t last_rejection : 2;  /* latest norm/invalid rejection; 3 is initial */
    uint32_t armed : 1;           /* calibrated baseline has been observed */
    uint32_t current_valid : 1;   /* projection/norm describe a fresh sample */
    uint32_t latched : 1;         /* retained until authorized upright reset */
    uint32_t saturated : 1;       /* current metric exceeded its packed range */
    uint32_t window_set : 1;      /* lifetime peak window initialized */
    uint32_t min_cos_set : 1;     /* at least one gravity-like sample seen */
    uint32_t peak_up_set : 1;     /* upward projection high-water initialized */
    uint32_t peak_saturated : 1;  /* any sample exceeded the packed metric range */
} baseline_lift_guard_t;
typedef struct {
    unsigned bend_phase : 2, bend_side : 2, bend_percent : 7, recovery_phase : 3;
    unsigned sweep_stage : 3;
    signed yaw_x10 : 13, target_left : 9, target_right : 9;
    unsigned saturated : 1;
} baseline_drive_diag_t;
typedef struct {
    unsigned valid : 1;
    unsigned flags : 3;
    signed int link : 3;
    unsigned failmask : 7;
    uint32_t stop_ms;
    uint32_t poll_age_ms;
    uint32_t app_age_ms;
    uint32_t ack_age_ms;
    uint32_t connects;
    uint32_t last_complete_ms;
} baseline_network_stop_t;
typedef struct {
    uint64_t since, wait_since;
    uint64_t settle_since, progress_since, still_since;
    uint32_t mm;
    signed first : 2;
    unsigned phase : 3, stage : 3;
    double origin_wheel[2], last_wheel[2], progress_wheel[2];
    float relative_yaw_deg, target_yaw_deg;
    unsigned active : 1, waiting : 1, still_timing : 1, wheel_valid : 1, progress_valid : 1,
        found_candidate : 1;
} baseline_line_search_t;
typedef struct {
    float mean, half;
    int16_t emitted[2];
    float previous_error, return_confidence, same_side_duration;
    float gain, damp_weight;
    /* Opt-in bend assist state. Durations and scales are finite, bounded
     * values; bitfields make the state visible to compact diagnostics. */
    float bend_demand_duration, bend_yaw_duration, bend_exit_duration;
    float bend_speed_scale, bend_authority_scale;
    uint8_t side : 2;
    uint8_t active : 1;
    uint8_t bend_candidate : 1;
    uint8_t bend_confirmed : 1;
    uint8_t bend_exiting : 1;
    int8_t bend_direction : 2;
} baseline_continuous_steering_t;
typedef struct {
    uint32_t type : 3;      /* imu_evt_type_t                       */
    uint32_t motion : 3;    /* imu_motion_t at the time             */
    uint32_t detail : 4;    /* imu_reject_t or IMU_FAULT_* code     */
    uint32_t height_mm : 8; /* hump peak (or highest peak), mm      */
    int32_t pitch_dd : 11;  /* body pitch, 0.1 deg, + = nose up     */
    uint32_t b_valid : 1;   /* IMU data valid (calibrated, no fault) */
    uint32_t b_odo : 1;     /* Buddy 2 estimate fresh               */
    uint32_t reserved : 1;
    uint16_t t_ms; /* sys_time_us() / 1000, wrapping: the
                                 * shared 1 us clock is the team's
                                 * timestamp base (integration test
                                 * IMU + Motion Control, +-100 ms)      */
} baseline_imu_event_t;
#endif
