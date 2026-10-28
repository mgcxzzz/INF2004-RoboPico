/* The tested Continuous V2 loop runs once, in one kernel task. Hardware
 * ownership is supplied by utk_hal, not a second Pico SDK runtime. */
#define main utk_drive_main
#include "controller/main.c"
#undef main

void buddy_drive_task(int stacd, void *exinf)
{
    (void)stacd;(void)exinf;
    (void)utk_drive_main();
    for(;;)sleep_ms(1000);
}
