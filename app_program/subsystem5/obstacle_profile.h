/**
 * @file    obstacle_profile.h
 * @brief   Obstacle profiling and bypass planning (pure logic, no RTOS).
 *
 * This module turns a set of (angle, range) samples into an obstacle
 * profile and a bypass plan. It has no hardware or kernel dependency, so
 * it is unit-tested on the host (see test/).
 *
 * Coordinate frame, seen from the sensor looking forward:
 *   angle  0 dd = right, 900 dd = straight ahead, 1800 dd = left
 *   x = r * cos(angle)  (positive to the RIGHT)
 *   y = r * sin(angle)  (positive AHEAD)
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#ifndef B5_PROFILE_H
#define B5_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define B5_MAX_SAMPLES      (24U)   /* coarse (5) + refinement + spare */

/** One range sample taken at one servo angle. */
typedef struct
{
    int16_t angle_dd;   /**< servo angle, tenths of a degree           */
    int32_t range_mm;   /**< median range; B5_RANGE_FAR_MM if no target */
    bool    b_valid;    /**< false if the sensor did not respond        */
} b5_sample_t;

/** All samples of one scan, kept sorted by angle. */
typedef struct
{
    b5_sample_t sample[B5_MAX_SAMPLES];
    uint8_t     count;
} b5_scan_t;

/** Tunables the profiler needs (copied from b5_config.h by the caller). */
typedef struct
{
    int32_t near_mm;            /**< range below which a sample is "near" */
    int16_t alpha_dd;           /**< beam half-angle                     */
    int16_t scan_min_dd;        /**< lowest angle the sweep reaches      */
    int16_t scan_max_dd;        /**< highest angle the sweep reaches     */
    int32_t car_width_mm;
    int32_t car_length_mm;
    int32_t margin_mm;
    int32_t obst_depth_mm;
    int32_t open_clearance_mm;
    bool    b_prefer_right;
} b5_params_t;

/** Which way to go round the obstacle. */
typedef enum
{
    B5_SIDE_NONE = 0,
    B5_SIDE_RIGHT,
    B5_SIDE_LEFT
} b5_side_t;

/** Result of Planning: what the obstacle looks like and what to do. */
typedef struct
{
    bool      b_found;          /**< at least one near sample           */
    int16_t   right_edge_dd;    /**< raw right edge (bracket middle)    */
    int16_t   left_edge_dd;     /**< raw left edge (bracket middle)     */
    bool      b_right_at_limit; /**< obstacle runs past the scan arc    */
    bool      b_left_at_limit;
    int32_t   closest_mm;       /**< closest range in the obstacle run  */
    int16_t   closest_dd;
    int32_t   width_raw_mm;     /**< width before beam-spread correction */
    int32_t   width_mm;         /**< width after correction (NFR-USO-02) */
    int32_t   x_right_mm;       /**< corrected right edge, lateral      */
    int32_t   x_left_mm;        /**< corrected left edge, lateral       */
    int32_t   clear_right_mm;   /**< free lateral space right of edge   */
    int32_t   clear_left_mm;    /**< free lateral space left of edge    */
    uint8_t   confidence_pct;   /**< share of samples that responded    */
    b5_side_t side;             /**< chosen side, NONE = blocked        */
    int32_t   lateral_mm;       /**< sideways shift needed for bypass   */
    int32_t   forward_mm;       /**< forward travel to pass + margin    */
} b5_profile_t;

/* Trigonometry in Q15 fixed point (32768 = 1.0), angle in tenths of deg. */
int32_t b5_cos_q15(int32_t angle_dd);
int32_t b5_sin_q15(int32_t angle_dd);

/* Scan bookkeeping. */
void    b5_scan_clear(b5_scan_t * p_scan);
bool    b5_scan_add(b5_scan_t * p_scan, int16_t angle_dd,
                    int32_t range_mm, bool b_valid);
int32_t b5_median3(int32_t a, int32_t b, int32_t c);

/* Coarse Sweep -> Edge Refinement decision. */
bool    b5_scan_has_near(b5_scan_t const * p_scan, int32_t near_mm);
bool    b5_next_refine_angle(b5_scan_t const * p_scan, int32_t near_mm,
                             int16_t alpha_dd, int16_t * p_angle_dd);

/* Planning. */
void    b5_build_profile(b5_scan_t const * p_scan,
                         b5_params_t const * p_param,
                         b5_profile_t * p_prof);

#endif /* B5_PROFILE_H */
