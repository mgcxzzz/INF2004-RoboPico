# Provisional SDK integration bundle

This package contains the complete Buddy1–4 SDK integration source, tests, UF2, dashboard, setup helpers, documentation and selected historical evidence. Buddy5 is not integrated. The original repository's separate micro T-Kernel source tree is not included in this standalone package; its historical guide is retained for context only.

Start with docs/setup.md. The firmware is under firmware/integration/buddy1_4/artifacts/. Its BUILD_MANIFEST.json states the source/dependency baseline, calibration behavior, checks and limits. This restructured image still requires physical acceptance. No Wi-Fi credentials are embedded; runtime provisioning reuses/saves device settings. Use the included root Windows.cmd files or their documented Python equivalents.
