# Build and verify Wi-Fi V2

Requires Raspberry Pi Pico SDK 2.2.0 (tested checkout `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`), CMake, Arm GNU 14.2.Rel1 and Python 3. Host tests also need a native C compiler and Node.js. The supplied UF2 targets **Pico W / RP2040**, with networking ON and no optional Buddy 5 extension or embedded credentials.

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build-wifi-on -DCMAKE_BUILD_TYPE=Release
cmake --build build-wifi-on -j8
python3 run_tests.py
python3 run_lwip_timeout_test.py
python3 verify_stack_layout.py --build build-wifi-on
python3 verify_firmware.py
```

CMake normally generates `build-wifi-on/buddy123_interface_wifi_v2.uf2` via the SDK extra-output tools. The recorded build used `-DPICO_NO_PICOTOOL=1`, then the Pico SDK `elf2uf2` converter on the ELF. The network-OFF build was also compiled with `-DBUDDY_WIFI_ENABLED=OFF` in a separate directory. Neither build is evidence of physical connectivity or driving.

To reproduce a credential-free release, do not place `wifi_config.local.h` in this folder. Device USB provisioning is supported and keeps normal reflashing independent of credentials. Source/binary checksums, build measurements and flash/RAM bounds are in `BUILD_METRICS.json`. Firmware ends before the settings sector at `0x101aa000`.

`verify_firmware.py` checks all current ledger entries, preserved controller sources against V1, UF2 block structure/family/flash bounds/markers, RAM bounds, no local credential header, and packaged file integrity. It does not require staging or committing files. It verifies the recorded artifact; rebuilding may alter timestamps/paths and thus its hash. Repackaging after any edits requires regenerating source/artifact metadata.

V2.1 reserves four application timeout slots above `LWIP_NUM_SYS_TIMEOUT_INTERNAL`. `run_lwip_timeout_test.py` uses the configured SDK source, verifies the old core-only configuration panics when the application timer is added, then tests the fixed pool. It is separate from the SDK-independent host suite.

V2.2 uses RP2040-only SDK memory-region symbol overrides: top 16 KiB of main SRAM reserved for separate 8 KiB stacks; heap ends below both. main.c is compiled with `-fstack-usage`; the linked ELF plus its `.su` report are required for `verify_stack_layout.py`. SDK 2.2.0 and its symbol-based default linker scripts are required for the recorded layout.
