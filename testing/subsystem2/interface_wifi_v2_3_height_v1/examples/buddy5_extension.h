#ifndef BUDDY5_EXTENSION_EXAMPLE_H
#define BUDDY5_EXTENSION_EXAMPLE_H
#include "../nav_api.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { B5_DRIVE=0,B5_TURN,B5_RESUME } Buddy5StepKind;
typedef struct {
    unsigned kind:2,speed_mm_s:7,style:1,abandon_interrupted_navigation:1;
    float goal; /* mm for drive, +LEFT/-RIGHT degrees for turn */
} Buddy5Step;
/* Implement using Buddy5's own sensor and route planner. Defaults do nothing. */
bool buddy5_obstacle_detected(void);
bool buddy5_obstacle_clear(void);
bool buddy5_next_step(Buddy5Step *step);
void buddy5_step_completed(const MotionCommandStatus *result);
#ifdef __cplusplus
}
#endif
#endif
