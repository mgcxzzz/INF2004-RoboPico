/**
 * @file    subsystem1.c
 * @brief   Subsystem 1 -- WiFi and MQTT (Buddy 1): creates its objects and
 *          starts its task (the network side runs inside the kernel's WiFi
 *          service task, see b1_net.h).
 *
 * Group IS28, Buddy 1. Coding standard: Barr-C:2018.
 */

#include "subsystem1.h"

EXPORT ER
subsystem1_init (void)
{
    return b1_create_objects();
}

EXPORT ER
subsystem1_start (void)
{
    ID tid = app_start_task(b1_task, S1_PRI_WIFI, 0, "wifi_task");

    if (tid < E_OK)
    {
        return tid;
    }
    return b1_glue_start();
}
