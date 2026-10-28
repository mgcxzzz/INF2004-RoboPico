# Buddy 1/2/3 Wi-Fi V2.3 - working SDK base with calibration preset

This release starts from **interface_wifi_v2 V2.2** at commit `adcfb6c91326f1636aecd90d7b64e03c810e1411`, which you and your teammate report worked on the robot. It includes the successful Continuous V2 front calibration. V2.2 and the UTK releases remain in their original folders.

Download [buddy123_interface_wifi_v2_3.uf2](firmware/buddy123_interface_wifi_v2_3.uf2), or the [complete package](firmware/buddy123_interface_wifi_v2_3_bundle.zip). Follow [SETUP.md](SETUP.md).

## What is preserved

Continuous V2 steering, per-wheel PI, PWM limits, profiles, ADC filtering, barcode decoding, junction handling and line recovery are unchanged from V2.2. All three continuous-control headers are also byte-identical to `interface_continuous_v2`. Motion APIs and the nav coordinator remain in use; Buddy 5's default extension is empty. Network code, dashboard and binary packet formats are byte-identical to V2.2.

This is the **standalone Pico SDK architecture** that worked for your teammate: core 0 controls motion; core 1 owns Wi-Fi/lwIP/MQTT through bounded copied queues. It is not the professor's micro T-Kernel service architecture. The failing UTK V1.1 boot timebase comparison is absent because this release uses SDK startup and clocks. The observed 251 ms / 382865 us mismatch does not establish the exact UTK cause; it is not explained by the original divider fault alone.

The working V2.2 fixes are retained: `LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4` (11 timer slots), and separate 8 KiB stacks with the heap below both. The official SDK commit matches the V2.2 release; the compiler differs and is recorded in BUILD_METRICS.json.

## Calibration

| Front sensor | White ADC | Black ADC |
| --- | ---: | ---: |
| Left | 163 | 3457 |
| Right | 160 | 3700 |

These are reference endpoints from the successful `idk_v2.txt` run, not required live readings at the middle of a straight. First boot uses this preset and profile 1 even if older firmware saved another calibration. A valid calibration/profile saved by V2.3 uses record version 28 and is restored after subsequent boots. Records 26/27 remain readable, but cannot override the initial preset. The saved record layout and CRC remain unchanged.

`w`, `l`, `r`, `k` remain available for manual calibration. While stopped, after saving finishes and with both buttons released, **`c` restores the successful preset**; **`k` saves it**. Restoring clears the previous motion rows in RAM and leaves the selected profile unchanged. Export a run before restoring or recalibrating.

## Validation

ARM Release build, 35 C test executables, 10 JavaScript schema tests, 20 Python helper tests, C-to-dashboard packet vectors, actual SDK lwIP allocator regression and real ELF stack/heap checks pass. [TEST_LOG.md](TEST_LOG.md) records evidence and limits. Physical V2.3 driving and heartbeat timing still need your teammate's test; the reported success applies to its V2.2 base.

[BUILD.md](BUILD.md) gives reproduction instructions. `WORKING_V2_2_BASELINE.json` records the copied base, `CALIBRATION_CHANGES.patch` shows the firmware changes, and `SOURCE_SHA256.json` records the release sources. Other copied V1/V2 provenance documents are historical.
