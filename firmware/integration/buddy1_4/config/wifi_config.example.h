/**
 * @file wifi_config.example.h
 * @brief Wifi config.example.
 * @details Build-time configuration for the tested application.
 * Coding cleanup does not change calibration, timing, motor or sensor constants.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef WIFI_CONFIG_EXAMPLE_H
#define WIFI_CONFIG_EXAMPLE_H

/* Copy to wifi_config.local.h and set values for your own network. */
#define WIFI_SSID "replace-with-laptop-hotspot-ssid"
#define WIFI_PASSWORD "replace-with-wifi-password"
#define MQTT_BROKER_IP "192.168.1.2"
#define MQTT_CLIENT_ID "buddy23"

#endif

/* End of file. */
