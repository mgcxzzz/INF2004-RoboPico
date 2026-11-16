/**
 * @file nav_coordinator.h
 * @brief Nav coordinator.
 * @details The shared facade/coordinator arbitrates ownership and generations.
 * Acceptance is not completion; external callers never write motor registers.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_NAV_COORDINATOR_H
#define BUDDY_NAV_COORDINATOR_H
#include "src/motion/motion_port.h"
/* Public motion_api.h is the request facade. These coordinator entry points
 * are its implementation: ownership and policy live here, not in the driver. */
void nav_coordinator_init (const motion_port_t * p_port);
void nav_coordinator_begin (uint64_t now);
void nav_coordinator_end (uint8_t reason);
motion_result_t nav_coordinator_pause (motion_owner_t owner,
                                       uint32_t * p_lease);
motion_result_t nav_coordinator_stop (motion_owner_t owner);
motion_result_t nav_coordinator_move (motion_owner_t owner, uint32_t lease,
                                      float goal, unsigned speed, bool turn,
                                      motion_turn_style_t style,
                                      uint32_t * p_identifier);
motion_result_t nav_coordinator_resume (motion_owner_t owner, uint32_t lease,
                                        bool abandon);
motion_result_t nav_coordinator_route (motion_owner_t owner, char code);
motion_result_t nav_coordinator_terrain (motion_owner_t owner,
                                         motion_terrain_t terrain);
motion_result_t nav_coordinator_navigation (motion_owner_t owner, int16_t left,
                                            int16_t right);
motion_result_t nav_coordinator_fault (motion_owner_t owner, unsigned bitmap,
                                       bool active);
motion_result_t nav_coordinator_speed_mode (motion_owner_t owner,
                                            unsigned mode);
motion_speed_mode_status_t nav_coordinator_speed_mode_status (void);
motion_result_t
nav_coordinator_ramp_assist (motion_owner_t owner,
                             const motion_ramp_assist_t * p_request);
motion_ramp_assist_t nav_coordinator_get_assist (void);
motion_result_t nav_coordinator_verified_barcode (motion_owner_t owner,
                                                  char code);
/* Application adapter only: motion-selection eligibility, not buddy policy. */
void nav_coordinator_assist_context (bool eligible);
void nav_coordinator_note (unsigned phase, char decoded);
void nav_coordinator_reject (motion_owner_t owner, unsigned kind, float goal,
                             unsigned speed, motion_result_t result);
motion_status_t nav_coordinator_status (void);
bool nav_coordinator_command (uint32_t identifier,
                              motion_command_status_t * p_status);
motion_output_t nav_coordinator_update (const motion_feedback_t * p_feedback);
bool nav_coordinator_commit (uint32_t epoch, void (*p_apply)(void *),
                             void * p_user);
bool nav_coordinator_pop (motion_event_t * p_event);
unsigned nav_coordinator_log_count (void);
unsigned nav_coordinator_log_overwritten (void);
bool nav_coordinator_log_read (unsigned index, motion_event_t * p_event);
#endif

/* End of file. */
