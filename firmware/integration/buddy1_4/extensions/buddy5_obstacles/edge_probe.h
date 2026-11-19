#ifndef B5_EDGE_PROBE_H
#define B5_EDGE_PROBE_H

/* Bounded edge-crossing diagnostic.  This module observes and reports only;
 * its result never resumes navigation or commands an obstacle bypass. */
#include <stdbool.h>
#include <stdint.h>
#include "extensions/buddy5_obstacles/automatic_bypass.h" /* B5SensorPort */
#include "include/interfaces/nav_api.h"

/* rtos_buddy1_5: the ONLY change from edge_probe v0.1.1 is the edge_scan v0.1.1
 * near association allowance (25 mm, was 15): a clean echo counts IN for the
 * face if expected-25 <= echo <= expected+60. Motion, poses, triplet/fault
 * rules, FAR +60, trims and the decision are unchanged. BE/BF carry wire
 * version 3 (= v2 layout) so a decoder applies the 25 mm semantics. */
#ifndef B5EP_DIR_WIRE            /* host checks may build the exact v0.1.1 rule */
#define B5EP_DIR_WIRE       3u
#endif
#define B5EP_SERVO_LIMIT_DD 460
#define B5EP_PROBE_MM       50u
#define B5EP_QUEUE_CAP      16u
#define B5EP_RECORD_COUNT   8u
#define B5EP_D0_MIN_MM      120u
#define B5EP_D0_MAX_MM      260u

enum {
    B5EP_IDLE, B5EP_CLAIM, B5EP_REF,
    B5EP_TURN_L, B5EP_PRE_L, B5EP_DRIVE_L, B5EP_POST_L, B5EP_RETRACE_L, B5EP_BACK_L,
    B5EP_TURN_R, B5EP_PRE_R, B5EP_DRIVE_R, B5EP_POST_R, B5EP_RETRACE_R, B5EP_BACK_R,
    B5EP_TURN_BACK, B5EP_FINAL, B5EP_DONE, B5EP_HOLD, B5EP_TRIM
};
enum {
    B5EP_END_NONE, B5EP_END_COMPLETE, B5EP_END_CANCELLED, B5EP_END_MOVE,
    B5EP_END_SENSOR, B5EP_END_LEASE, B5EP_END_TIMEOUT, B5EP_END_REFERENCE,
    B5EP_END_ALIGNMENT
};
enum {
    B5EP_PH_REF, B5EP_PH_PRE_L, B5EP_PH_POST_L, B5EP_PH_BACK_L,
    B5EP_PH_PRE_R, B5EP_PH_POST_R, B5EP_PH_BACK_R, B5EP_PH_FINAL
};
enum { B5EP_UNKNOWN, B5EP_IN, B5EP_FAR };
enum { B5EP_SIDE_UNKNOWN, B5EP_SIDE_OPEN, B5EP_SIDE_CROSSED, B5EP_SIDE_REMAINS };
enum { B5EP_DECISION_HOLD, B5EP_DECISION_LEFT, B5EP_DECISION_RIGHT };
enum {
    B5EP_FLAG_REF_OK = 1u << 0,
    B5EP_FLAG_FINAL_REF_OK = 1u << 1,
    B5EP_FLAG_LEFT_BACK_OK = 1u << 2,
    B5EP_FLAG_RIGHT_BACK_OK = 1u << 3,
    B5EP_FLAG_SERVO_IN_RANGE = 1u << 4
};

typedef struct {
    uint32_t ms;
    uint16_t run, index;
    uint8_t phase, n;
    int16_t heading_cdeg, servo_dd, applied_dd, travel_mm10;
    uint16_t expected_mm, anchor_mm;
    uint8_t cls[3], evidence;
    uint16_t value[3];
    int16_t net_x_mm10, net_y_mm10, correction_cdeg;
} B5EPRec;

typedef struct {
    uint32_t ms;
    uint16_t run;
    uint8_t end, decision, state_l, state_r;
    uint16_t records, lost, d0;
    int16_t advance_l_mm10, reverse_l_mm10, advance_r_mm10, reverse_r_mm10;
    int16_t turn_l_cdeg, turn_r_cdeg, turn_back_cdeg, final_heading_cdeg;
    int16_t net_x_mm10, net_y_mm10;
    uint16_t flags, correction_count;
    int16_t correction_total_cdeg;
} B5EPSummary;

typedef struct {
    B5SensorPort sensor;
    int (*applied)(void); /* Optional applied servo command, never pose feedback. */
    unsigned state, sub, end;
    uint32_t lease, command;
    uint64_t began, since, ping_since, sample_since;
    float heading_deg, move_start_heading;
    uint16_t d0, run;
    b5_ping_t ping[3];
    unsigned npings;
    bool command_active, move_committed, pose_timing, ping_active;
    unsigned trim_return_state, correction_count;
    int16_t correction_total_cdeg;
    float trim_target_deg;
    bool pose_trimmed;
    int servo_dd, last_track_dd;
    uint8_t sample_phase;
    uint32_t sample_ms;
    int16_t sample_heading_cdeg, sample_servo_dd, sample_applied_dd;
    int16_t pre_expected_mm[2];
    uint16_t pre_median_mm[2];
    int16_t pre_heading_cdeg[2], pre_net_y_mm10[2];
    float net_x_mm, net_y_mm, travel_mm;
    float advance_l_mm, reverse_l_mm, advance_r_mm, reverse_r_mm;
    float turn_l_deg, turn_r_deg, turn_back_deg;
    uint8_t verdict[ B5EP_RECORD_COUNT ];
    bool ref_ok, final_ref_ok, left_back_ok, right_back_ok, servo_in_range;
    B5EPRec rec[B5EP_QUEUE_CAP];
    uint16_t rec_head, rec_count, rec_index, lost;
    B5EPSummary summary;
    bool summary_pending;
} B5EdgeProbe;

void b5_ep_init(B5EdgeProbe *t, const B5SensorPort *port);
bool b5_ep_ready(const B5EdgeProbe *t);
bool b5_ep_begin(B5EdgeProbe *t, uint64_t now);
void b5_ep_tick(B5EdgeProbe *t, uint64_t now);
void b5_ep_cancel(B5EdgeProbe *t, uint64_t now);
void b5_ep_reset(B5EdgeProbe *t);
bool b5_ep_peek(const B5EdgeProbe *t, B5EPRec *out);
void b5_ep_drop(B5EdgeProbe *t);
const char *b5_ep_state_name(unsigned state);
const char *b5_ep_end_name(unsigned end);

#endif
