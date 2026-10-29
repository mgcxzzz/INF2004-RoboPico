# Build and verify Wi-Fi V2.3_height_v1

Pico W / RP2040 standalone SDK application. Pin **Pico SDK `98a542c1a62fb549ffb5d66a3e5892b06276b670`**, including its `lib/tinyusb`, `lib/cyw43-driver` and `lib/lwip` submodules. This is the official SDK commit used by the working V2.3 base (and its V2.2 predecessor). Older SDK linker scripts may ignore the region-symbol overrides and fail to fit the 8 KiB stacks; do not reduce stack sizes to work around that.

The packaged release uses Arm GNU **14.2.Rel1 / 14.2.1 20241119**, CMake/Ninja and Release optimization, matching the packaged V2.3 toolchain. No local credential header or Buddy 5 extension was present.

```sh
export PICO_SDK_PATH=/path/to/pinned-pico-sdk
cmake -S . -B build-wifi-on -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w
cmake --build build-wifi-on -j8
python3 run_tests.py
python3 run_lwip_timeout_test.py
python3 verify_stack_layout.py --build build-wifi-on
python3 package_release.py --build build-wifi-on
python3 verify_firmware.py
```

Host tests need a native C compiler and Node. `CC` overrides the compiler; `CFLAGS` adds host-only flags. On this Windows machine, the passing host suite used MinGW GCC 9.2 with `-msse2 -mfpmath=sse -Wno-misleading-indentation`, `-O0 -UNDEBUG`; target warnings-as-errors remain enabled. SSE avoids x87 extended-precision changes to inherited exact float boundary tests. The style-warning suppression is for existing host test formatting only.

Windows native builds used short source/build paths and `CMAKE_OBJECT_PATH_MAX=200` for path lengths. `PICO_NO_PICOTOOL=1` was used, then the SDK `elf2uf2` converter generated `buddy123_interface_wifi_v2_3_height_v1.uf2` from the linked ELF. A normal SDK setup with picotool can generate UF2 directly.

Real ELF memory bounds and compiler frame measurements are in `BUILD_METRICS.json` and `logs/stack_layout.txt`. Each core has 8192 stack bytes, and BSS/heap ends below both stacks. These are static checks, not a complete worst-case runtime stack bound. Firmware ends below the unchanged settings sector at `0x101aa000`.

`HEIGHT_BASELINE.json` records every copied source hash. `verify_firmware.py` compares unchanged files to that base, checks the approved height patch, source ledger, UF2 blocks/markers/flash bounds, stack bounds and package contents. `SOURCE_SHA256.json` excludes itself, build metrics and generated firmware/ZIP files to avoid circular hashes. After rebuilding or editing, regenerate release metadata and the package before verifying. Rebuilding can change the artifact hash.

Diagnostic capture format5 preserves58-byte rows but changes GAIN/DAMP interpretation; see README. Current output filename includes v2_3_height_v1. Host suite now has37 C executables. Exact original SDK/network/stack fixes are retained. HEIGHT_PATCH.json can reverse every approved production-source change to the hashed V2.3 base.

The packaging helper requires this Git checkout with the recorded base commit available, ARM tools on PATH and the host/lwIP/stack evidence saved under logs/. It packages and verifies sources; it does not build or publish.
