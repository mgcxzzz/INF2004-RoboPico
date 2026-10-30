# Combined build history (v0.1–v0.4)

Notes, patches, build manifests and host-test logs of the intermediate combined builds (v2.3 + Buddy 4 + ramp assist + barcode peak decoder). The current build is combined v0.5 in `testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5/`. The intermediate source folders are kept locally, not in Git; each patch is against the previous version (v0.1 against `interface_wifi_v2_3/`), so any version can be rebuilt by applying them in order.

| Combined | Ramp assist | Change | Why (hardware) |
|---|---|---|---|
| v0.1 | v0.2 | Peak decoder integrated into the Buddy 1–4 build; source-15 coexistence; dashboard v0.3 | Barcode working on the standalone peak build |
| v0.2 | v0.3 | Hold through ACCEL/SPREAD-only tilt exclusions (150 ms) | Assist cut out at the crest |
| v0.3 | v0.4 | Stall push +60 PWM | Stuck at the crest at 295/165 PWM |
| v0.4 | v0.5 | Fixed 300 PWM floor per wheel until flat | Ramp wheel starved by steering; replaced in v0.5 (too fast, single use) |
| **v0.5** | **v0.6** | Speed hold (min 35 mm/s, integral limit 300) | Current — see `B4_PEAK_V0_5.md` |

The host-test logs and manifests were produced at the time; UF2 hashes refer to local, credential-bearing images that are not published.
