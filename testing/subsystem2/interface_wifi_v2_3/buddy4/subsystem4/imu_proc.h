/**
 * @file    imu_proc.h
 * @brief   Buddy 4: statechart 6.4 and all IMU processing, as pure C.
 *
 * No kernel, no hardware: task_imu.c reads the sensor and Buddy 2's
 * estimate, fills a b4_in_t every 20 ms and acts on the b4_out_t that
 * comes back. The same code runs in the host tests
 * (testing/subsystem4/host) and in the bench tool.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#ifndef IMU_PROC_H
#define IMU_PROC_H

#include <stdbool.h>
#include <stdint.h>

#include "b4_config.h"
#include "imu_iface.h"

/* Observability only: exclusions seen by the existing side-ramp guards. */
enum {
    B4_Q_SPREAD = 1U, B4_Q_GRAVITY = 2U, B4_Q_ACCEL = 4U,
    B4_Q_TURN = 8U, B4_Q_REVERSE = 16U, B4_Q_IMPACT = 32U,
    B4_Q_TIMING = 64U, B4_Q_UNAVAILABLE = 128U
};

/* One accelerometer sample in the CAR's frame (after the mounting axes
 * and signs of b4_config.h), mg. */
typedef struct
{
    int16_t fwd_mg;
    int16_t lat_mg;
    int16_t up_mg;
} b4_vec_t;

/* Everything the task hands over every period. */
typedef struct
{
    bool     b_read_ok;             /* the I2C reads all worked          */
    uint8_t  n;                     /* samples taken from the FIFO       */
    bool     b_overrun;             /* the FIFO overflowed (data lost)   */
    b4_vec_t s[B4_FIFO_DEPTH];      /* oldest first                      */
    uint32_t t_us;                  /* sys_time_us() when read           */
    uint32_t gap_us;                /* since the previous period         */
    bool     b_motion;              /* Buddy 2 estimate present + fresh  */
    int16_t  distance_mm;           /* net distance, wraps               */
    int16_t  heading_dd;            /* + = right, wraps                  */
    bool     b_fine_motion;         /* optional sub-mm displacement */
    int32_t  distance_um;           /* net micrometres from same snapshot */
} b4_in_t;

/* What the task must do after a call. */
#define B4_OUT_EVENTS_MAX   (4U)

typedef struct
{
    uint8_t     n_events;
    imu_event_t events[B4_OUT_EVENTS_MAX];
    uint8_t     fault;              /* IMU_FAULT_* to report now, 0 none */
    bool        b_shock;            /* shock warning to report now       */
    uint32_t    shock_age_us;       /* sample age when it was detected   */
    bool        b_init;             /* (re)configure the GY-511 now      */
    bool        b_clear_fault;      /* clear SYS_FAULT_IMU (after reset) */
} b4_out_t;

/* Operating-state changes from the Navigation Coordinator, mapped. */
typedef enum
{
    B4_MISSION_START = 0,   /* Ready -> Running: new run, reset maximum */
    B4_MISSION_RESUME,      /* Paused -> Running                        */
    B4_MISSION_STOP,        /* paused, stopped, e-stop, fault           */
    B4_MISSION_DONE,        /* course completed: publish run summary    */
    B4_MISSION_RESET        /* back to Ready: an authorised reset       */
} b4_mission_t;

/* Hump profile (front-contact height rebuilt along the distance). */
typedef struct
{
    int32_t  bins[B4_HUMP_BINS];    /* height, 0.1 mm, ring by distance  */
    uint32_t head;                  /* newest bin                        */
    int32_t  d_mm;                  /* distance travelled (accumulated)  */
    int32_t  next_bin_mm;           /* distance of the next bin          */
    int32_t  level_mm;              /* level this far (in Tracking)      */
    uint32_t level_us;              /* and this long                     */
    bool     b_zeroed;              /* ground reference set this stretch */
    bool     b_ref_ok;              /* ground reference trusted (false
                                     * after a shock until level again)  */
    bool     b_tainted;             /* this hump began without it        */
    bool     b_active;              /* a hump is being profiled          */
    bool     b_descent;
    bool     b_odo_ok;              /* fresh distance all through it     */
    uint32_t t_start_us;
    int32_t  d_start_mm;
    int32_t  peak;                  /* 0.1 mm                            */
    int32_t  after_mm;              /* level after the descent           */
    uint32_t after_us;
    uint32_t up_us;                 /* nose up this long (start guard)   */
} b4_hump_t;

#define B4_RATE_N   ((B4_RATE_WINDOW_MS / B4_PERIOD_MS) + 1U)

/* The whole subsystem state. */
typedef struct
{
    imu_state_t  state;
    bool         b_mission;         /* Running                           */
    uint8_t      fault;             /* last IMU_FAULT_* (latched)        */

    /* Calibrating */
    uint32_t     cal_start_us;
    uint32_t     cal_n;
    int32_t      cal_sum[3];
    int16_t      cal_min[3];
    int16_t      cal_max[3];

    /* Baseline (gravity at rest, car frame) */
    int16_t      base[3];           /* fwd, lat, up, mg                  */
    int32_t      g_mg;              /* |g| measured                      */

    /* Filters and derived values */
    int32_t      lp[3];             /* low-passed sample, mg << 8        */
    int16_t      pitch_dd;
    int8_t       tilt;              /* +1 climbing, -1 descending, 0     */

    /* Motion estimate history (Buddy 2) */
    uint32_t     hist_t[B4_RATE_N];
    int32_t      hist_d[B4_RATE_N];
    int32_t      hist_fine_d[B4_RATE_N];
    bool         hist_fine;
    int32_t      hist_h[B4_RATE_N];
    uint32_t     hist_n;            /* consecutive fresh periods         */
    bool         b_have_last;
    int16_t      last_dist;
    int16_t      last_head;
    int32_t      head_dd;           /* heading, accumulated              */
    int32_t      speed_mm_s;
    int32_t      accel_mm_s2;
    int32_t      turn_dd_s;
    bool         b_rates;           /* the three above are valid         */

    /* Classification */
    imu_motion_t motion;            /* published class                   */
    imu_motion_t cand;              /* candidate waiting out the hold    */
    uint32_t     cand_us;

    /* Shock */
    uint32_t     clear_us;          /* below the clear level this long   */
    uint16_t     shock_mg;          /* last shock peak                   */
    uint8_t      shock_ms;          /* last detect-to-notify time        */
    uint8_t      shocks;

    /* Hump */
    b4_hump_t    hump;
    uint8_t      last_height_mm;
    uint8_t      max_height_mm;     /* highest valid peak this run       */
    uint8_t      humps;
    uint8_t      rejected;

    /* One-sided ramp view (right wheel relative to left, signed). */
    int16_t      side_height_mm;    /* instantaneous right-minus-left */
    int16_t      side_roll_dd;      /* roll derived from gravity       */
    bool         b_side_valid;      /* current side measurement usable */
    uint8_t      side_quality_reason; /* this tick's exclusion bitmask */
    uint16_t     side_spread_mg;    /* current FIFO maximum axis range */
    uint16_t     side_mad_mg;       /* maximum axis median absolute deviation */
    uint16_t     side_axis_mad_mg[3]; /* per-axis MAD, fwd/lat/up (diagnostic) */
    uint8_t      side_fifo_samples;
    bool         side_rates_valid; /* rates refreshed by this tick */
    bool         side_candidate;    /* candidate lift is being checked */
    bool         side_rising;       /* sustained right-up threshold met */
    bool         side_returning;    /* return-to-ground timer active   */
    bool         side_level_ready;  /* level observed before candidate */
    int16_t      side_peak_mm;      /* candidate maximum, signed       */
    int16_t      side_start_dist;   /* odometry at candidate start     */
    int16_t      side_last_dist;
    int16_t      side_last_head;
    int32_t      side_travel_mm;    /* forward travel while elevated   */
    uint32_t     side_start_us;
    uint32_t     side_up_us;
    uint32_t     side_return_us;
    uint32_t     side_level_us;
    uint32_t     side_skip_run_us;  /* v0.8: consecutive skipped ticks  */
    uint32_t     side_skip_us;      /* v0.8: skipped time in candidate  */
    int16_t      side_last_h_mm;    /* v0.8: last valid candidate height */
    bool         side_have_odo;

    /* Data health */
    uint8_t      i2c_run;           /* failed reads in a row             */
    uint8_t      stale_run;
    uint8_t      stuck_run;
    b4_vec_t     last_mean;
    uint8_t      i2c_errors;
    uint16_t     missed;            /* 50 Hz frames missed               */
    bool         b_odo_now;         /* this period had fresh distance    */
} b4_ctx_t;

/* Statechart. */
void b4_reset(b4_ctx_t * p_ctx);
void b4_init_done(b4_ctx_t * p_ctx, bool b_ok, uint32_t t_us,
                  b4_out_t * p_out);
void b4_mission(b4_ctx_t * p_ctx, b4_mission_t event, uint32_t t_us,
                b4_out_t * p_out);
void b4_tick(b4_ctx_t * p_ctx, b4_in_t const * p_in, b4_out_t * p_out);

/* Fill a telemetry view from the context and the frame's event. */
void b4_fill_tlm(b4_ctx_t const * p_ctx, imu_event_t const * p_evt,
                 imu_tlm_t * p_tlm);

/* Helpers, exported for the tests and the bench tool. */
int16_t  b4_asin_dd(int32_t sin_q15);
uint32_t b4_isqrt(uint32_t value);
bool     b4_nfr02_ok(int32_t est_mm, int32_t ref_mm);
bool     b4_side_geometry(int16_t const baseline[3], int16_t const sample[3],
                          int32_t g_mg, int16_t *height_mm,
                          int16_t *roll_dd);

#endif /* IMU_PROC_H */
