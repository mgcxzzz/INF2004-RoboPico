/* Opt-in adapter example, NOT an obstacle planner. Default weak inputs never
 * trigger movement. Replace them with non-blocking Buddy5 measurements/plans. */
#include "extensions/buddy5_obstacles/buddy5_extension.h"
#include "include/interfaces/buddy_hooks.h"
__attribute__((weak)) bool buddy5_obstacle_detected(void) {
    return false;
}
__attribute__((weak)) bool buddy5_obstacle_clear(void) {
    return false;
}
__attribute__((weak)) bool buddy5_next_step(Buddy5Step *step) {
    (void)step;
    return false;
}
__attribute__((weak)) void buddy5_step_completed(const MotionCommandStatus *r) {
    (void)r;
}
enum { B5_WATCH = 0, B5_STOPPING, B5_PLANNING, B5_MOVING, B5_RELEASING, B5_FAILED };
static struct {
    unsigned state : 3;
    uint32_t lease, id;
    uint64_t retry_us;
    Buddy5Step step;
} job;
void buddy_extension_init(void) {
    job.state = B5_WATCH;
    job.lease = job.id = 0;
    job.retry_us = 0;
}
void buddy_extension_poll(uint64_t now_us) {
    MotionStatus status = nav_get_status();
    if (!status.session_active) {
        buddy_extension_init();
        return;
    }
    if (job.state == B5_WATCH) {
        if (buddy5_obstacle_detected()) {
            MotionResult r = nav_request_takeover(MOTION_BUDDY5, &job.lease);
            if (r == MOTION_OK)
                job.state = B5_STOPPING;
        }
        return;
    }
    if (status.mode == MOTION_FAULT) {
        job.state = B5_FAILED;
        return;
    }
    if (job.state == B5_STOPPING) {
        if (status.mode == MOTION_PAUSED && status.stopped)
            job.state = B5_PLANNING;
        return;
    }
    if (job.state == B5_PLANNING) {
        if (!buddy5_next_step(&job.step))
            return;
        MotionResult r;
        if (job.step.kind == B5_RESUME) {
            job.state = B5_RELEASING;
            job.retry_us = 0;
            return;
        }
        if (job.step.kind == B5_DRIVE)
            r = nav_drive_distance(MOTION_BUDDY5, job.lease, job.step.goal, job.step.speed_mm_s,
                                   &job.id);
        else if (job.step.kind == B5_TURN)
            r = nav_turn_relative(MOTION_BUDDY5, job.lease, job.step.goal, job.step.speed_mm_s,
                                  (MotionTurnStyle)job.step.style, &job.id);
        else
            r = MOTION_INVALID;
        if (r == MOTION_OK)
            job.state = B5_MOVING;
        else {
            (void)nav_request_stop(MOTION_BUDDY5);
            job.state = B5_FAILED;
        }
        return;
    }
    if (job.state == B5_MOVING) {
        MotionCommandStatus result;
        if (!nav_get_command(job.id, &result)) {
            job.state = B5_FAILED;
            (void)nav_request_stop(MOTION_BUDDY5);
            return;
        }
        if (result.state == MOTION_CMD_DONE) {
            buddy5_step_completed(&result);
            job.state = B5_PLANNING;
        } else if (result.state != MOTION_CMD_RUNNING)
            job.state = B5_FAILED;
        return;
    }
    if (job.state == B5_RELEASING && buddy5_obstacle_clear() && now_us >= job.retry_us) {
        MotionResult r =
            nav_resume_line(MOTION_BUDDY5, job.lease, job.step.abandon_interrupted_navigation);
        job.retry_us = now_us + 100000;
        if (r == MOTION_OK)
            job.state = B5_WATCH;
        else if (r != MOTION_NEED_LINE && r != MOTION_NOT_STOPPED && r != MOTION_NEED_RECOVERY)
            job.state = B5_FAILED;
    }
    /* FAILED holds motion ownership until the operator stops/restarts the test.
     * A real planner can provide its own explicit recovery policy instead. */
}
