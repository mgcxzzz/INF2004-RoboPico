# Build the provisional integration

The public bundle contains source and a ready-to-flash UF2. Building requires the Pico SDK and ARM GNU toolchain, CMake, Ninja and Python. Use SDK commit98a542c1a62fb549ffb5d66a3e5892b06276b670 to reproduce this build's dependency baseline. Normal SDK submodules for USB/Wi-Fi/lwIP are required; Bluetooth is not used.

From repository/package root:

```text
cmake -S firmware/integration/buddy1_4 -B build-buddy1_4 -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w -DPICO_SDK_PATH=<your-sdk-path>
cmake --build build-buddy1_4
```

The output is robopico_buddy1_4_integration_v0_1.uf2 when a compatible SDK picotool is available. Ramp assistance and the peak decoder default ON. For a shadow experiment set RAMP_ASSIST_ACTIVE=OFF and/or BARCODE_PEAK_ACTIVE=OFF and identify those options in the build manifest; do not overwrite the published active artifact.

If the matching picotool is unavailable, configure with `-DPICO_NO_PICOTOOL=1`, build the ELF/BIN and use `python tools/firmware/package_release.py --build-dir <build-buddy1_4>`. The release tool verifies the standard RP2040 UF2 blocks, flash limits, linker stack/heap layout, no-local-credentials build configuration and complete bundle contents. See its help. It never flashes a device.

Public builds use no config/wifi_config.local.h. The runtime provisioner loads saved Wi-Fi from its reserved flash sector. Build products, local credentials and captured dashboard sessions are gitignored. Run [offline tests](testing.md) before packaging. Build artifacts and checksums are under firmware/integration/buddy1_4/artifacts/. The release manifest records actual compile options, SDK/compiler identity, checks and provisional status; hardware acceptance remains pending.
