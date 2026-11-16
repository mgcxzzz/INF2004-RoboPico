/**
 * @file motion_api.c
 * @brief Motion api.
 * @details The shared facade/coordinator arbitrates ownership and generations.
 * Acceptance is not completion; external callers never write motor registers.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "include/interfaces/motion_api.h"
#include "src/navigation/nav_coordinator.h"
#include "src/navigation/nav_codec.h"
#include <math.h>
#include <stddef.h>
/* All buddy requests cross the canonical compact binary boundary before
 * coordinator policy. Convenience C arguments are not a second motor path. */
static bool valid_owner (motion_owner_t o_value)
{
    return o_value >= MOTION_BUDDY1 && o_value <= MOTION_BUDDY5;
}
static motion_result_t reject (motion_owner_t o_value, unsigned kind,
                               float value, unsigned speed,
                               motion_result_t result)
{
    nav_coordinator_reject(o_value, kind, value, speed, result);
    return result;
}
static motion_result_t submit (const nav_request_t * p_request,
                               uint32_t * p_token)
{
    uint8_t packet[NAV_PACKET_BYTES];
    nav_request_t result_value;
    unsigned count = nav_packet_encode(packet, p_request);
    if (!count || !nav_packet_decode(&result_value, packet, count))
    {
        return reject((motion_owner_t)p_request->owner, p_request->kind,
                      p_request->value, p_request->speed, MOTION_INVALID);
    }
    motion_owner_t o_value = (motion_owner_t)result_value.owner;
    motion_result_t result;
    switch (result_value.kind)
    {
    case NAV_PAUSE:
        result = nav_coordinator_pause(o_value, p_token);
        break;
    case NAV_STOP:
        result = nav_coordinator_stop(o_value);
        break;
    case NAV_DRIVE:
    case NAV_TURN:
        result = nav_coordinator_move(
            o_value, result_value.lease, result_value.value, result_value.speed,
            result_value.kind == NAV_TURN,
            (motion_turn_style_t)result_value.style, p_token);
        break;
    case NAV_RESUME:
        result = nav_coordinator_resume(o_value, result_value.lease,
                                        result_value.flag);
        break;
    case NAV_ROUTE:
#ifdef BUDDY_RUNTIME_MAPPING
        result = nav_coordinator_route(
            o_value, (char)('A' + (unsigned)result_value.value));
        break;
#else
        result =
            nav_coordinator_route(o_value, result_value.value == 0   ? 'A'
                                           : result_value.value == 1 ? 'F'
                                                                     : 'Z');
        break;
#endif
    case NAV_VERIFIED_BARCODE:
        result = nav_coordinator_verified_barcode(
            o_value, (char)('A' + (unsigned)result_value.value));
        break;
    case NAV_RAMP_ASSIST:
        result = nav_coordinator_ramp_assist(o_value, &result_value.assist);
        break;
    case NAV_ASSIST:
    {
        motion_ramp_assist_t first = {
            .boost_milli = (uint16_t)((unsigned)result_value.value * 10),
            .ttl_ms = 100};
        result = nav_coordinator_ramp_assist(o_value, &first);
        break;
    }
    case NAV_TERRAIN:
        result = nav_coordinator_terrain(
            o_value, (motion_terrain_t)(unsigned)result_value.value);
        break;
    case NAV_WHEELS:
        result = nav_coordinator_navigation(o_value, result_value.wheels[0],
                                            result_value.wheels[1]);
        break;
    case NAV_FAULT_REPORT:
        result = nav_coordinator_fault(o_value, result_value.bitmap,
                                       result_value.flag);
        break;
    case NAV_SPEED_MODE:
        result = nav_coordinator_speed_mode(o_value, result_value.speed);
        break;
    default:
        result = MOTION_UNSUPPORTED;
        break;
    }
    /* Per-tick Buddy3 BUSY simply means it yielded to an avoidance lease;
     * do not flood the audit log with repeated control targets. */
    if (result != MOTION_OK &&
        !(result_value.kind == NAV_WHEELS && result == MOTION_BUSY))
    {
        nav_coordinator_reject(o_value, result_value.kind, result_value.value,
                               result_value.speed, result);
    }
    return result;
}
motion_result_t motion_pause (motion_owner_t o_value, uint32_t * p_lease_value)
{
    if (!valid_owner(o_value) || !p_lease_value)
    {
        return reject(o_value, NAV_PAUSE, 0, 0, MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_PAUSE, .owner = o_value};
    return submit(&result, p_lease_value);
}
motion_result_t motion_stop (motion_owner_t o_value)
{
    if (!valid_owner(o_value))
    {
        return reject(o_value, NAV_STOP, 0, 0, MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_STOP, .owner = o_value};
    return submit(&result, NULL);
}
motion_result_t motion_drive_distance (motion_owner_t o_value, uint32_t lease,
                                       float mm_value_value, unsigned speed,
                                       uint32_t * p_identifier)
{
    if (p_identifier)
    {
        *p_identifier = 0;
    }
    if (!valid_owner(o_value) || !p_identifier || speed > 60)
    {
        return reject(o_value, NAV_DRIVE, mm_value_value, speed,
                      MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_DRIVE,
                            .owner = o_value,
                            .lease = lease,
                            .value = mm_value_value,
                            .speed = speed};
    return submit(&result, p_identifier);
}
motion_result_t motion_turn_relative (motion_owner_t o_value, uint32_t lease,
                                      float degrees, unsigned speed,
                                      motion_turn_style_t style,
                                      uint32_t * p_identifier)
{
    if (p_identifier)
    {
        *p_identifier = 0;
    }
    if (!valid_owner(o_value) || !p_identifier || speed > 90 ||
        (style != MOTION_ROLLING && style != MOTION_IN_PLACE))
    {
        return reject(o_value, NAV_TURN, degrees, speed, MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_TURN,
                            .owner = o_value,
                            .lease = lease,
                            .value = degrees,
                            .speed = speed,
                            .style = style};
    return submit(&result, p_identifier);
}
motion_result_t motion_resume_line (motion_owner_t o_value, uint32_t lease,
                                    bool abandon)
{
    if (!valid_owner(o_value))
    {
        return reject(o_value, NAV_RESUME, 0, 0, MOTION_INVALID);
    }
    nav_request_t result = {
        .kind = NAV_RESUME, .owner = o_value, .lease = lease, .flag = abandon};
    return submit(&result, NULL);
}
motion_status_t motion_get_status (void)
{
    return nav_coordinator_status();
}
motion_result_t motion_request_speed_mode (motion_owner_t o_value,
                                           unsigned mode)
{
    if (!valid_owner(o_value))
    {
        return nav_coordinator_speed_mode(o_value, mode);
    }
    nav_request_t result = {.kind = NAV_SPEED_MODE,
                            .owner = o_value,
                            .speed = mode <= DRIVE_MODE_NONE ? mode
                                                             : DRIVE_MODE_NONE};
    return submit(&result, NULL);
}
motion_speed_mode_status_t motion_get_speed_mode_status (void)
{
    return nav_coordinator_speed_mode_status();
}
bool motion_get_command (uint32_t identifier_value,
                         motion_command_status_t * p_state)
{
    return nav_coordinator_command(identifier_value, p_state);
}
motion_result_t motion_submit_barcode (motion_owner_t o_value, char character)
{
#ifdef BUDDY_RUNTIME_MAPPING
    if (!valid_owner(o_value) || character < 'A' || character > 'Z')
    {
        return reject(o_value, NAV_ROUTE, character, 0, MOTION_INVALID);
    }
    nav_request_t result = {
        .kind = NAV_ROUTE, .owner = o_value, .value = (float)(character - 'A')};
    return submit(&result, NULL);
#else
    if (!valid_owner(o_value) ||
        (character != 'A' && character != 'F' && character != 'Z'))
    {
        return reject(o_value, NAV_ROUTE, character, 0, MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_ROUTE,
                            .owner = o_value,
                            .value = character == 'A'   ? 0
                                     : character == 'F' ? 1
                                                        : 2};
    return submit(&result, NULL);
#endif
}
motion_result_t motion_submit_navigation (motion_owner_t o_value, int16_t left,
                                          int16_t right)
{
    if (!valid_owner(o_value))
    {
        return reject(o_value, NAV_WHEELS, 0, 0, MOTION_INVALID);
    }
    nav_request_t result = {
        .kind = NAV_WHEELS, .owner = o_value, .wheels = {left, right}};
    return submit(&result, NULL);
}
motion_result_t motion_report_terrain (motion_owner_t o_value,
                                       motion_terrain_t sample)
{
    if (!valid_owner(o_value) || (unsigned)sample > (unsigned)MOTION_DESCENDING)
    {
        return reject(o_value, NAV_TERRAIN, sample, 0, MOTION_INVALID);
    }
    nav_request_t result = {
        .kind = NAV_TERRAIN, .owner = o_value, .value = (float)sample};
    return submit(&result, NULL);
}
motion_result_t motion_request_speed_assist (motion_owner_t o_value,
                                             unsigned data)
{
    if (!valid_owner(o_value) || data > 20)
    {
        return reject(o_value, NAV_ASSIST, (float)data, 0, MOTION_INVALID);
    }
    nav_request_t result = {
        .kind = NAV_ASSIST, .owner = o_value, .value = (float)data};
    return submit(&result, NULL);
}
motion_result_t motion_report_fault (motion_owner_t o_value, unsigned mask,
                                     bool active)
{
    if (!valid_owner(o_value) || !mask || (mask & ~NAV_FAULT_MASK))
    {
        return reject(o_value, NAV_FAULT_REPORT, (float)mask, 0,
                      MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_FAULT_REPORT,
                            .owner = o_value,
                            .bitmap = mask,
                            .flag = active};
    return submit(&result, NULL);
}
const char * motion_result_name (motion_result_t result_value)
{
    static const char * const names[] = {
        "OK",          "INACTIVE",    "BUSY",      "NOT_OWNER",    "INVALID",
        "NOT_STOPPED", "UNSUPPORTED", "NEED_LINE", "NEED_RECOVERY"};
    return (unsigned)result_value < sizeof(names) / sizeof(names[0])
               ? names[result_value]
               : "UNKNOWN";
}
void motion_init (const motion_port_t * p_data)
{
    nav_coordinator_init(p_data);
}
void motion_session_begin (uint64_t sample)
{
    nav_coordinator_begin(sample);
}
void motion_session_end (uint8_t result)
{
    nav_coordinator_end(result);
}
motion_output_t motion_update (const motion_feedback_t * p_flags)
{
    return nav_coordinator_update(p_flags);
}
bool motion_commit (uint32_t event, void (*p_flags)(void *), void * p_u_value)
{
    return nav_coordinator_commit(event, p_flags, p_u_value);
}
bool motion_pop_event (motion_event_t * p_event_value)
{
    return nav_coordinator_pop(p_event_value);
}
void motion_note_navigation (unsigned phase, char decoded)
{
    nav_coordinator_note(phase, decoded);
}
unsigned motion_log_count (void)
{
    return nav_coordinator_log_count();
}
unsigned motion_log_overwritten (void)
{
    return nav_coordinator_log_overwritten();
}
bool motion_log_read (unsigned index_value, motion_event_t * p_event_value)
{
    return nav_coordinator_log_read(index_value, p_event_value);
}

motion_result_t
motion_request_ramp_assist (motion_owner_t o_value,
                            const motion_ramp_assist_t * p_first)
{
    if (!valid_owner(o_value) || !p_first)
    {
        return reject(o_value, NAV_RAMP_ASSIST, 0, 0, MOTION_INVALID);
    }
    nav_request_t result = {
        .kind = NAV_RAMP_ASSIST, .owner = o_value, .assist = *p_first};
    return submit(&result, NULL);
}
motion_ramp_assist_t motion_get_ramp_assist (void)
{
    return nav_coordinator_get_assist();
}
void motion_assist_context (bool eligible)
{
    nav_coordinator_assist_context(eligible);
}
motion_result_t motion_accept_barcode (motion_owner_t o_value, char code)
{
    if (!valid_owner(o_value) || code < 'A' || code > 'Z')
    {
        return reject(o_value, NAV_VERIFIED_BARCODE, code, 0, MOTION_INVALID);
    }
    nav_request_t result = {.kind = NAV_VERIFIED_BARCODE,
                            .owner = o_value,
                            .value = (float)(code - 'A')};
    return submit(&result, NULL);
}

/* End of file. */
