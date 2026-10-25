/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Called by cyw43_utk_task() on every poll (about every 2 ms) while the
 * link is CYW43_LINK_UP (see cyw43_utk.c). IS28 Buddy 1: connects to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h), reconnects with
 * back-off, publishes the heartbeat and the queued telemetry, and receives
 * supervisory commands (app_program/subsystem1/b1_net.h). Each call does a
 * bounded amount of work and returns.
 */
void mqtt_utk_start(void);

#endif /* MQTT_UTK_H */
