# Interface WiFi V1.3 diagnostic build evidence - 2026-10-28

Continuous V2 base: 1263390b069ae9617c412e47212177e77807c505. Buddy1 input: b09403074939f45cf4d7c11b7606148f6c2a4e73. New independent folder; old V2 artifact remains unchanged. Motion/steering/follow parameters and motor executor are byte-checked against V2. Buddy5 default hooks stay empty. No WiFi credentials are embedded.

V1.1 corrects the SDK readiness query from radio-only cyw43_wifi_link_status to cyw43_tcpip_link_status. The former returns JOIN, not IP-ready UP; comparing it with UP prevented all MQTT attempts in V1. The network-owner adapter regression checks down/join/no-IP/UP/auth failure. New n diagnostics report link state, Pico IP and connection attempts/results. V1 binaries remain archived for comparison.

V1.2 adds exact settings-save rejection codes, retained setup status, stored broker readout and completed network-loop count/time/phase. Guards and control algorithms are unchanged. Existing tests now exercise active/settling/button/flash-overlap rejection attribution, and the helper retains full reason lines without echoing unexpected configuration text. These diagnostics are not a claim that a physical worker stall is fixed.

V1.3 DIAG adds a dedicated8192-byte network stack,100ms estimated watermark/canary sampling and per-callback poll kind/PC markers. The observed poll preserves the pinned SDK execution order; a new regression checks timed catchup, pending flag clearing/new pending work and refresh. This is a diagnostic experiment; physical stack exhaustion is not established. The legacy StackOne linker reservation is unused by this custom-stack launch.

## Executed software checks

- 34 C test executables pass with C11, -Wall -Wextra -Werror. Tests cover existing motion/coordinator/lease/clock/stop behavior, controller models, A-Z in both scan directions across all four map slots, stable queued revisions, straight consumption, unassigned letters, CRC/settings/import and actual WiFi harness supervisory commands/downloads.
- C-produced literal packets decode in the JavaScript consumer. Ten schema tests and eleven Python provisioning/broker-helper tests pass. Malformed lengths, numeric bounds, reserved bits and incomplete downloads are tested.
- Headless Chrome desktop/mobile smoke passes with simulated MQTT: mapping readback, active/paused download exclusion, fragmented diagnostics, bounded retry/rejection, unknown calibration, long history export, local vendor loading and no page errors/overflow. Screenshots/logs are local ignored build artifacts.
- Three Pico W Release builds pass -Wall -Wextra -Werror. The network-disabled and unpacked variants are measurement/reference builds; the delivered UF2 is network-enabled, packed, with empty extension source.
- verify_firmware.py validates provenance, unchanged V2 controls, source bundle, UF2 structure/markers/flash bounds and tracked publication artifacts. It runs after Git staging.

| ARM variant | text | data | BSS |
|---|---:|---:|---:|
| build-wifi-on | 508704 | 0 | 201348 |
| build-wifi-off | 210764 | 0 | 148080 |
| build-wifi-unpacked | 507384 | 0 | 206308 |

Packed versus identical unpacked WiFi features saves4960B shared-state BSS. Network transport adds53268B versus network-off. Smaller histories reduce total BSS relative to V2; that reduction is not a like-for-like packing result. DATA_REPRESENTATION.md records the partial minimum-bit compliance boundary.

Symbols: flash end 0x1007b324; BSS end 0x20032114; StackLimit 0x20040000. The 57068B linker gap is not measured worst-case heap/stack margin. Core1 stack symbols are 0x20040800..0x20041000. Padded UF2 image ends0x1007b400, below settings bank0x101aa000. Existing diagnostic/nav/raw/motion banks remain0x1ab000/0x1af000/0x1b0000/0x1d0000; new settings sector0x1aa000.

Artifact: firmware/buddy123_interface_wifi_v1_3_diag.uf2, **1009664 bytes**, **1972 RP2040 UF2 blocks**. SHA256: b8577871b61d19278f1a48d961743d42657dd501a22c2f3befef1734c117a9db.

WIFI_BASELINE.json hashes the V2 C/H snapshot. WIFI_PATCHES.json lists explicit routing/storage/harness/API changes. SOURCE_SHA256.json covers shipped source/docs/scripts. Historical V12.2/V1/Continuous manifests describe their earlier components, not this artifact.

## Rebuild and verify

Use Pico SDK commit a1438dff1d38bd9c65dbd693f0e5db4b9ae91779 (including CYW43/lwIP/TinyUSB submodules), ARM GCC14.2.1, CMake and Ninja. From this directory, with PICO_SDK_PATH set:

    cmake -S . -B build-wifi-on -G Ninja -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release -DBUDDY_WIFI_ENABLED=ON
    cmake --build build-wifi-on

For network-off use a separate build directory with -DBUDDY_WIFI_ENABLED=OFF. For the identical uncompressed reference use -DINTERFACE_UNPACKED_REFERENCE=ON with WiFi ON. The default BUDDY_EXTENSION_SOURCE is empty. wifi_config.local.h is optional/ignored; the release must build without it.

Run run_tests.py with Python3 and ZIG pointing to a Zig compiler. Node is required for schema/vector tests. Browser smoke: node dashboard/ui_smoke.mjs, with PLAYWRIGHT_NODE_MODULE_PATH and CHROME_PATH if needed. Run python verify_firmware.py on the published bundle; rebuilding with another toolchain may change byte hashes.

**Not yet validated:** a physical WiFi/MQTT connection, runtime core1/control timing, integrated barcode/junction/bend behavior, heap/stack high-water usage or obstacle avoidance. Existing V2 physical straight-line evidence does not prove this integration. Use profile1 and README.md's staged hardware checks.
