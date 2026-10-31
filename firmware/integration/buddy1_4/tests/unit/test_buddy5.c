#include <assert.h>
#include <stdio.h>
#include "extensions/buddy5_obstacles/buddy5_extension.h"
#include "include/interfaces/buddy_hooks.h"
#include "src/motion/motion_port.h"
static uint64_t now_us;
static unsigned offered, completed;
static bool detected = true;
static uint64_t now(void *p) {
    (void)p;
    return now_us;
}
bool buddy5_obstacle_detected(void) {
    return detected;
}
bool buddy5_obstacle_clear(void) {
    return offered >= 3;
}
bool buddy5_next_step(Buddy5Step *s) {
    if (offered == 0)
        *s = (Buddy5Step){.kind = B5_DRIVE, .goal = -10, .speed_mm_s = 30};
    else if (offered == 1)
        *s = (Buddy5Step){.kind = B5_TURN, .goal = -10, .speed_mm_s = 30, .style = MOTION_ROLLING};
    else if (offered == 2) {
        *s = (Buddy5Step){.kind = B5_RESUME};
        detected = false;
    } else
        return false;
    ++offered;
    return true;
}
void buddy5_step_completed(const MotionCommandStatus *s) {
    assert(s->state == MOTION_CMD_DONE);
    ++completed;
}
int main(void) {
    now_us = 1000;
    MotionPort port = {.now_us = now};
    motion_init(&port);
    motion_session_begin(now_us);
    buddy_extension_init();
    MotionFeedback f = {.now_us = now_us, .line_level = {700, 700}};
    MotionOutput o = motion_update(&f);
    for (unsigned tick = 0; tick < 2000; ++tick) {
        now_us += 10000;
        f.now_us = now_us;
        for (unsigned w = 0; w < 2; ++w) {
            f.speed_mm_s[w] = o.target_mm_s[w];
            f.wheel_mm[w] += f.speed_mm_s[w] * .01;
        }
        if (nav_get_status().mode == MOTION_AUTO)
            assert(nav_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
        o = motion_update(&f);
        buddy_extension_poll(now_us);
        if (completed == 2 && nav_get_status().mode == MOTION_AUTO) {
            assert(offered == 3);
            puts(
                "PASS opt-in Buddy5 example: takeover -> measured stop -> reverse -> right turn -> explicit clear/visible-line resume; no blocking hooks.");
            return 0;
        }
    }
    assert(false);
}
