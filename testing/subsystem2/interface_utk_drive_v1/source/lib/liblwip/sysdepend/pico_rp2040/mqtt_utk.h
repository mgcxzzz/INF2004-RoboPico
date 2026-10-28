/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Called by cyw43_utk_task() on every poll (about every 2 ms) while the
 * link is CYW43_LINK_UP (see cyw43_utk.c). Connects to the validated WCF1
 * runtime broker, subscribes to supervisory commands, and transfers copied
 * packets through the bounded drive bridge. The controller is the sole
 * heartbeat producer. Each call does bounded work and returns.
 */
void mqtt_utk_start(void);

#endif /* MQTT_UTK_H */
