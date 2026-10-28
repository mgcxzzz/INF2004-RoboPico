#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "continuous_follow.h"
#include "nav_api.h"
#include "motion_port.h"

static uint64_t clock_us;
static unsigned lock_depth, off_calls, apply_calls;
static int32_t pwm[2];
static MotionFeedback feedback;
static MotionOutput motion_output;
static ContinuousDrive wheel[2];
static SignedWheel legacy[2];
static ContinuousSteering steering;

static uintptr_t enter(void *user) {
    (void)user;
    assert(lock_depth == 0);
    ++lock_depth;
    return 0;
}

static void leave(void *user, uintptr_t state) {
    (void)user;
    (void)state;
    assert(lock_depth == 1);
    --lock_depth;
}

static uint64_t now_us(void *user) {
    (void)user;
    return clock_us;
}

static void outputs_off(void *user) {
    (void)user;
    assert(lock_depth == 1);
    ++off_calls;
    pwm[0] = pwm[1] = 0;
}

typedef struct {
    int32_t next[2];
    bool selected;
} Commit;

static void commit_outputs(void *argument) {
    const Commit *commit = argument;
    assert(lock_depth == 1);
    ++apply_calls;
    for (unsigned w = 0; w < 2; ++w) {
        DualLevels levels = commit->selected
            ? continuous_output_levels(w, commit->next[w])
            : (DualLevels){0};
        pwm[w] = levels.a ? (int32_t)levels.a : (int32_t)levels.b;
    }
}

static void reset_controllers(void) {
    memset(wheel, 0, sizeof(wheel));
    memset(legacy, 0, sizeof(legacy));
    continuous_steering_reset(&steering);
}

static void begin_session(void) {
    clock_us = 100000;
    lock_depth = off_calls = apply_calls = 0;
    pwm[0] = pwm[1] = 0;
    reset_controllers();
    MotionPort port = {enter, leave, outputs_off, now_us, NULL};
    motion_init(&port);
    motion_session_begin(clock_us);
    feedback = (MotionFeedback){.now_us=clock_us,.line_level={700,700},.pending_route='A'};
    motion_output = motion_update(&feedback);
}

static bool selection(unsigned profile, unsigned phase, bool busy, bool external,
                      bool ramp, bool maneuver, bool white, const AnalogDemand *d) {
    return continuous_drive_selected(profile, phase, busy, external, ramp,
                                     maneuver, white, d);
}

static void auto_tick(unsigned profile, AnalogDemand *d, uint64_t elapsed_us) {
    clock_us += elapsed_us;
    feedback.now_us = clock_us;
    assert(motion_get_status().mode == MOTION_AUTO);
    assert(nav_submit_navigation(MOTION_BUDDY3, d->target[0], d->target[1]) == MOTION_OK);
    motion_output = motion_update(&feedback);
    assert(!motion_output.external);

    bool selected = selection(profile, C_FOLLOW, false, motion_output.external,
                              false, false, false, d);
    assert(selected);
    Commit commit = {0};
    for (unsigned w = 0; w < 2; ++w) {
        commit.next[w] = continuous_drive_step(&wheel[w], &legacy[w], w,
            motion_output.target_mm_s[w], feedback.speed_mm_s[w],
            (float)elapsed_us / 1000000.0f, clock_us, pwm[w]);
    }
    commit.selected = selected;
    assert(motion_commit(motion_output.generation, commit_outputs, &commit));
}

static void handover_and_resume(void) {
    begin_session();
    AnalogDemand demand = {.target={45,45},.level={700,700},.decision=25};

    assert(selection(1, C_FOLLOW, false, false, false, false, false, &demand));
    assert(selection(3, C_FOLLOW, false, false, false, false, false, &demand));
    assert(!selection(2, C_FOLLOW, false, false, false, false, false, &demand));
    assert(!selection(1, J_ACQUIRE, false, false, false, false, false, &demand));
    assert(!selection(1, C_FOLLOW, false, true, false, false, false, &demand));
    assert(!selection(1, C_FOLLOW, false, false, true, false, false, &demand));
    assert(!selection(1, C_FOLLOW, false, false, false, true, false, &demand));
    assert(selection(1, C_FOLLOW, false, false, false, false, true, &demand));
    demand.decision = SYM_STRAIGHT;
    assert(selection(1, C_FOLLOW, false, false, false, false, false, &demand));
    demand.decision = 25;

    for (unsigned k = 0; k < 8; ++k) {
        feedback.speed_mm_s[0] = feedback.speed_mm_s[1] = 35.0f;
        auto_tick(1, &demand, 10000);
    }
    assert(wheel[0].active && wheel[1].active);
    assert(pwm[0] > 0 && pwm[1] > 0);
    assert(selection(3, C_FOLLOW, false, false, false, false, false, &demand));
    uint32_t auto_generation = motion_output.generation;
    unsigned prior_apply_calls = apply_calls;

    uint32_t lease = 0;
    assert(nav_request_takeover(MOTION_BUDDY5, &lease) == MOTION_OK && lease != 0);
    assert(off_calls > 0 && pwm[0] == 0 && pwm[1] == 0);

    /* The accepted takeover invalidates an in-flight AUTO motor commit. */
    Commit stale = {{250,250},true};
    assert(!motion_commit(auto_generation, commit_outputs, &stale));
    assert(apply_calls == prior_apply_calls && pwm[0] == 0 && pwm[1] == 0);

    /* Adapter reset is driven by the coordinator's dirty output while it settles. */
    for (unsigned k = 0; k < 16; ++k) {
        clock_us += 10000;
        feedback.now_us = clock_us;
        feedback.speed_mm_s[0] = feedback.speed_mm_s[1] = 0.0f;
        motion_output = motion_update(&feedback);
        if (motion_output.reset_controllers) reset_controllers();
        assert(motion_output.external);
        assert(!selection(1, C_FOLLOW, false, motion_output.external,
                          false, false, false, &demand));
        Commit stopped = {{0,0},false};
        assert(motion_commit(motion_output.generation, commit_outputs, &stopped));
        assert(pwm[0] == 0 && pwm[1] == 0);
    }
    assert(motion_get_status().mode == MOTION_PAUSED && motion_get_status().stopped);
    assert(!wheel[0].active && !wheel[1].active && !steering.active);

    assert(nav_resume_line(MOTION_BUDDY5, lease, false) == MOTION_OK);
    clock_us += 10000;
    feedback.now_us = clock_us;
    feedback.line_level[0] = feedback.line_level[1] = 700;
    motion_output = motion_update(&feedback);
    assert(motion_output.resume_navigation && !motion_output.external);
    if (motion_output.reset_controllers || motion_output.resume_navigation)
        reset_controllers();
    assert(!wheel[0].active && !wheel[1].active && !steering.active);

    uint32_t id = 0;
    assert(nav_drive_distance(MOTION_BUDDY5, lease, 30, 40, &id) == MOTION_NOT_OWNER);
    for (unsigned k = 0; k < 3; ++k) {
        feedback.speed_mm_s[0] = feedback.speed_mm_s[1] = 0.0f;
        auto_tick(1, &demand, 10000);
    }
    assert(wheel[0].active && wheel[1].active);
    assert(motion_get_status().owner == MOTION_BUDDY3);
    assert(pwm[0] > 0 && pwm[1] > 0);
}

static void presearch_gap_keeps_continuous_driver(void) {
    const unsigned forward_decisions[]={SYM_STRAIGHT,SYM_LEFT,SYM_RIGHT,
        SYM_RECOVER_LEFT,SYM_RECOVER_RIGHT,C_PROBE_LEFT,C_PROBE_RIGHT,24,25};
    AnalogDemand d={.target={0,35},.decision=SYM_RECOVER_LEFT};
    for(unsigned profile_index=0;profile_index<2;++profile_index) {
        unsigned profile=profile_index?3:1;
        for(unsigned k=0;k<sizeof(forward_decisions)/sizeof(forward_decisions[0]);++k) {
            d.decision=(uint8_t)forward_decisions[k];
            assert(continuous_drive_selected(profile,C_FOLLOW,false,false,false,
                                             false,true,&d));
        }
    }

    /* Do not admit unknown/C_FAST raw decisions or line-search sweep stages. */
    d.decision=C_FAST;
    assert(!continuous_drive_selected(1,C_FOLLOW,false,false,false,false,true,&d));
    for(unsigned decision=19;decision<=23;++decision) {
        d.decision=(uint8_t)decision;
        assert(!continuous_drive_selected(1,C_FOLLOW,false,false,false,false,true,&d));
    }
    d.decision=SYM_RECOVER_LEFT;
    assert(!continuous_drive_selected(2,C_FOLLOW,false,false,false,false,true,&d));
    assert(!continuous_drive_selected(1,J_ACQUIRE,false,false,false,false,true,&d));
    assert(!continuous_drive_selected(1,C_FOLLOW,true,false,false,false,true,&d));
    assert(!continuous_drive_selected(1,C_FOLLOW,false,true,false,false,true,&d));
    assert(!continuous_drive_selected(1,C_FOLLOW,false,false,true,false,true,&d));
    assert(!continuous_drive_selected(1,C_FOLLOW,false,false,false,true,true,&d));
    d.stop=true;
    assert(!continuous_drive_selected(1,C_FOLLOW,false,false,false,false,true,&d));
    d.stop=false;

    /* The captured short-gap case entered the white bridge at 252 PWM with
     * 94 physical integral. Selection keeps the same motor state and limits
     * the next actuator change to the normal 12-per-10ms slew. */
    ContinuousDrive s={.duty=252,.reference=35,.integral=94,.active=1,
                       .start_used=1};
    SignedWheel p={.sign=1,.pi={.ramp=35,.integral=94.0f/1.2f}};
    d=(AnalogDemand){.target={35,35},.decision=SYM_STRAIGHT};
    assert(continuous_drive_selected(1,C_FOLLOW,false,false,false,false,true,&d));
    const int32_t prior_pwm=252;
    int32_t next=continuous_drive_step(&s,&p,1,d.target[1],35.0f,.01f,
                                       clock_us+10000,prior_pwm);
    int32_t delta=next-prior_pwm;
    assert(delta>=-12 && delta<=12);
    assert(s.active && s.start_used && s.reference==35.0f);
    assert(s.integral>=94.0f && s.integral<=100.0f);
    assert(p.pi.ramp==s.reference);

    /* The raw both-white coast demand is also eligible, and its explicit zero
     * target remains zero so the ordinary drive API turns that wheel off. */
    d=(AnalogDemand){.target={0,0},.decision=SYM_STRAIGHT};
    int16_t preserved[2]={d.target[0],d.target[1]};
    assert(continuous_drive_selected(3,C_FOLLOW,false,false,false,false,true,&d));
    assert(d.target[0]==preserved[0] && d.target[1]==preserved[1]);
    assert(continuous_drive_step(&s,&p,0,d.target[0],0.0f,.01f,
                                 clock_us+20000,next)==0);
    assert(continuous_drive_step(&s,&p,1,d.target[1],0.0f,.01f,
                                 clock_us+20000,prior_pwm)==0);
    assert(!s.active && s.integral==0.0f);
}

int main(void) {
    handover_and_resume();
    presearch_gap_keeps_continuous_driver();
    puts("continuous integration test passed (coordinator ownership, STOP/off, stale commit rejection, controller reset, and profile routing)");
    return 0;
}
