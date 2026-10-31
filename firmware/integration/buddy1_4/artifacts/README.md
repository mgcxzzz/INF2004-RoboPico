# Provisional firmware and complete package

**Buddy1–4 integration v0.1. Buddy5 NOT integrated. New physical acceptance pending.**

- `robopico_buddy1_4_integration_v0_1.uf2`: public image with no embedded Wi-Fi credentials.
- `robopico_buddy1_4_integration_v0_1_bundle.zip`: complete SDK integration source, tests, dashboard, setup tools/docs, selected evidence and the same UF2.
- `BUILD_MANIFEST.json`: actual build identity/options, calibration rules, memory layout and validation limits.
- `SOURCE_SHA256.json` and `SHA256SUMS.txt`: package/source/artifact integrity.

Start with the bundle's BUNDLE_README.md and docs/setup.md. Existing saved Wi-Fi/calibration normally survive reflashing. A blank Pico still starts with the original low-height preset; the later tilted-front capture is archived evidence, not silently installed. The separate original RTOS source tree is outside the standalone bundle.
