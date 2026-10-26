/* Separate diagnostic build. The shared adapter retains its existing pins,
 * encoders, controller, independent stop ISR and saved motion-log format. */
#define B_STRAIGHT_DRIVE_PROBE 1
#define B_RECORD_VERSION 15
#include "../circuit_with_barcode/main.c"
