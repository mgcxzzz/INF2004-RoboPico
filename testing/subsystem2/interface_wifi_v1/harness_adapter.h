#include "cruise_handover.h"
#include "continuous_follow.h"
static ContinuousDrive continuous_wheel[2];
static ContinuousSteering continuous_steering;
static bool continuous_active;
/* Included by the frozen V12.2 main after globals and log helpers. Hardware
 * stays in that one owner; buddy code only sees nav_api.h. */
static MotionOutput interface_motion;
static unsigned interface_last_phase,interface_a,interface_f,interface_z;
static uint32_t interface_session_generation;
static void interface_reset_controllers(void) {
    continuous_active=false;continuous_steering_reset(&continuous_steering);
    memset(continuous_wheel,0,sizeof(continuous_wheel));
    for(unsigned w=0;w<2;++w) {
        controller[w]=(SignedWheel){0};maneuver_power[w]=(ManeuverPower){0};
        ramp_wheel[w]=(RampWheel){0};feedback[w]=(CircuitFeedback){0};bad_direction_us[w]=0;
    }
    capture=(CircuitCapture){0};load=(CircuitLoad){0};capture_outer=-1;last_fast=false;
}
static void interface_resume_navigation(const CircuitInput *i) {
    /* Preserve completed navigation counters and a fully decoded pending route.
     * Partial barcode frames and interrupted manoeuvre goals are not reusable. */
    circuit.busy=nav.busy=false;circuit.phase=C_FOLLOW;circuit.completed=false;
    circuit.turn=(Junction){0};circuit.follower=(CircuitTracker){0};circuit.joining=(CircuitTracker){0};
    circuit.gate_timing=circuit.white_timing=circuit.cruise_timing=circuit.join_timing=circuit.fast=false;
    circuit.clear_path=circuit.follow_path=i->path;circuit.leg_since=i->now;
    nav.action=0;nav.uturn=(BarcodeUTurn){0};nav.cross_ready=nav.cross_candidate=nav.allow_cross=false;
    nav.leading_gate=nav.cross_gate=false;nav.leading_us=0;
    barcode_reader_reset(&nav.reader,side_analog.black,i->now,(i->wheel[0]+i->wheel[1])*.5);
    barcode_analog_begin(&side_analog,&nav,side_adc,i->now,i->wheel);
    steering=(SmoothFollow){0};gentle=(ScanSmooth){0};interface_reset_controllers();
}
static void interface_drain_events(void) {
    MotionEvent e;
    while(motion_pop_event(&e)) { wifi_nav_event(&e); /* z retains the independent audit ring. */ }
}
static AnalogDemand interface_navigation_step(const CircuitInput *i,float dt) {
    MotionStatus status=nav_get_status();AnalogDemand d={0};
    if(status.session_active && status.mode==MOTION_AUTO) {
        d=continuous_follow_step(&circuit,&nav,&calibration,i,recorded_mode,&steering,&side_analog,&gentle,&continuous_steering,dt);
        if(!d.stop)(void)nav_submit_navigation(MOTION_BUDDY3,d.target[0],d.target[1]);
        if(interface_session_generation!=guard.generation) {
            interface_last_phase=UINT32_MAX;interface_a=interface_f=interface_z=0;
            interface_session_generation=guard.generation;
        }
        if(nav.decoded_total!=interface_a)motion_note_navigation(circuit.phase,nav.last_code);
        interface_a=nav.decoded_total;interface_f=nav.decoded_f;interface_z=nav.decoded_z;
        if(circuit.phase!=interface_last_phase) {motion_note_navigation(circuit.phase,0);interface_last_phase=circuit.phase;}
    } else {
        circuit.completed=false;
        for(unsigned w=0;w<2;++w)d.level[w]=line_level(i->adc[w],calibration.white[w],calibration.black[w]);
    }
    MotionFeedback f={.now_us=i->now,.wheel_mm={i->wheel[0],i->wheel[1]},
        .speed_mm_s={i->speed[0],i->speed[1]},.line_level={d.level[0],d.level[1]},
        .navigation_busy=circuit.busy,.pending_route=nav.pending};
    interface_motion=motion_update(&f);
    if(interface_motion.reset_controllers)interface_reset_controllers();
    if(interface_motion.resume_navigation)interface_resume_navigation(i);
    if(interface_motion.route)(void)barcode_set_pending(&nav,interface_motion.route);
    if(!d.stop) {
        d.target[0]=interface_motion.target_mm_s[0];d.target[1]=interface_motion.target_mm_s[1];
        if(interface_motion.external)d.decision=interface_motion.turning?27:
            (d.target[0] || d.target[1])?26:28;
    }
    interface_drain_events();return d;
}
typedef struct {int32_t next[2];bool ramp,maneuver,continuous,turning,external,loaded[2];unsigned profile;uint64_t now;} InterfaceCommit;
static void interface_commit_outputs(void *argument) {
    InterfaceCommit *p=argument;
    for(unsigned w=0;w<2;++w) {
        ramp_output[w]=p->ramp;maneuver_output[w]=p->maneuver;continuous_output[w]=p->continuous;
        signed_output[w]=p->external || (barcode_nav_action(&nav,nav.action)=='U' && circuit.phase==J_TURN);
        peak_until[w]=p->maneuver?(controller[w].peak_active?controller[w].peak_end_us:0):
            p->ramp?ramp_wheel[w].boost_until:p->turning?controller[w].peak_end_us:0;
        output[w]=p->continuous?continuous_output_cap(p->next[w]):p->maneuver?maneuver_output_cap(p->next[w],peak_until[w],time_us_64()):
            barcode_output_cap(p->next[w],p->ramp,peak_until[w],time_us_64());
        DualLevels v=p->continuous?continuous_output_levels(w,output[w]):p->maneuver?maneuver_levels(w,output[w],signed_output[w]):
            signed_output[w]?boosted_junction_levels(w,output[w],p->external?p->turning:true):
            barcode_output_levels(w,output[w],p->ramp,p->loaded[w],p->profile);
        pwm_set_both_levels(pwm_gpio_to_slice_num(ma[w]),v.a,v.b);
    }
}
