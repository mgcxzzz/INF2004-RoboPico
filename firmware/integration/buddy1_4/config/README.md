# Configuration and calibration

The combined build loads a valid RTOS calibration first, then a valid SDK
record 28, then the built-in preset. Use **Diagnostics → Front IR calibration**
while stopped to capture and apply a new set in RAM or save it to flash. See
the [wireless calibration guide](../../../../docs/wireless_calibration_v0_2_8.md).
The built-in low-height preset has white 163/160 and black 3457/3700.

The teammate's later tilted-front capture is white122/138, black246/267 (spans124/129), recorded in testing/evidence/combined_baseline. It is evidence for that mounting, not a new universally validated default. This build deliberately preserves the working saved calibration on an existing car. A blank Pico starts with the original preset and may need the mounting-appropriate capture. Reflashing normally preserves saved settings.

`wifi_config.example.h` is documentation only. Public packages have no local credentials header; use runtime provisioning. If building a private image, config/wifi_config.local.h is gitignored. `lwipopts.h` retains the MQTT timeout-pool headroom. Control constants remain beside their algorithms and are explained in the feature READMEs.
