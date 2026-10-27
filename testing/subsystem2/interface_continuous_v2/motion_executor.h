#ifndef BUDDY_MOTION_EXECUTOR_H
#define BUDDY_MOTION_EXECUTOR_H
#include "motion_port.h"
typedef struct {
    bool active,turning,settling,still_timing;
    MotionTurnStyle style;float goal,speed,progress_mm,rotation_deg;
    double origin[2];uint64_t since,deadline,settle_since,still_since;
    int16_t target[2];unsigned fault:10;
} MotionExecutor;
bool motion_executor_start(MotionExecutor *e,const MotionFeedback *f,bool turn,float goal,unsigned speed,MotionTurnStyle style);
MotionCommandState motion_executor_step(MotionExecutor *e,const MotionFeedback *f);
#endif
