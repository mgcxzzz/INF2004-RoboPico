# Configuration and calibration

`calibration_presets.h` preserves the existing record28 rules and built-in successful low-height preset: white163/160, black3457/3700. A valid own record28 overrides it. `w/l/r/k` remain available; `c` restores that original preset, then `k` saves it. Automatic stopped-run saving also persists current calibration.

The teammate's later tilted-front capture is white122/138, black246/267 (spans124/129), recorded in testing/evidence/combined_baseline. It is evidence for that mounting, not a new universally validated default. This build deliberately preserves the working saved calibration on an existing car. A blank Pico starts with the original preset and may need the mounting-appropriate capture. Reflashing normally preserves saved settings.

`wifi_config.example.h` is documentation only. Public packages have no local credentials header; use runtime provisioning. If building a private image, config/wifi_config.local.h is gitignored. `lwipopts.h` retains the MQTT timeout-pool headroom. Control constants remain beside their algorithms and are explained in the feature READMEs.
