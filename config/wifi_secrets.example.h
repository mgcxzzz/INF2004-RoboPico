/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip).
 * MQTT_BROKER_IP: the computer running Mosquitto, on the same network
 * (WiFi is 2.4 GHz only). IS28 Buddy 1 uses the is28/... topics
 * (app_program/subsystem1/b1_net.h); MQTT_TOPIC is no longer used. */
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mtk3smp-pico"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"

/* Optional WiFi timing experiment, applied only by the WiFi service task.
 * Leave undefined to keep the driver's normal power management.
 * Uncomment for the Buddy 1 heartbeat-gap diagnostic; verify the boot
 * log says set=0, read=0 and active mode's low nibble is 0 before testing.
 * #define WIFI_PM_MODE CYW43_NONE_PM
 */

#endif /* WIFI_SECRETS_H */
