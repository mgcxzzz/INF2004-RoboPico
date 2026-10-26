# V12.2 build evidence —2026-10-26

Original V12 baseline `f0e3b35`, isolated45 transitive source/header/test files. Five intentional snapshot edits: main integration/help/record labels, event IDs, saved-record accepted versions, two record-version fixtures. All other files match BASELINE.json exactly, including barcode_nav.h, optical follower/recovery, smooth_follow.h legacy helper, decoder, junction/U-turn and all motor controllers. scan_smooth.h is the new active visible-output adapter; no V12.1 scan_hold or synthetic/frozen optical input is compiled.

## Executed validation

-12 host executables passed under Zig0.13 with -std=c11 -Wall -Wextra -Werror, LINE_CAL_MIN_SPAN=50 and CIRCUIT_SWEEP_SEARCH=1.
-New adapter: gradual nominal60mm/s clear-straight cruise,45mm/s scan transition; 1mm/s target hysteresis; mirrored transient steering; sustained moderate error converges; strong departure in either direction returns original targets immediately after scan entry; true both-white advances actual V12 loss state. Full140mm A/F/Z ADC fixtures decode in both directions and dispatch once at the separate cross, with planned manoeuvres taking priority.
-Synthetic10ms alternating small-error fixture: summed integer wheel-target variation V12=774, V12.2=0; the same test subsequently applies a persistent moderate offset and verifies a meaningful turning differential. This proves the computational response for this fixture, not zero physical jitter.
-Retained V12 tests: generic Code39/both sizes and directions, measured historical large-A replay,63-bank low-amplitude narrow bars, incomplete/missing bars and ambiguity rejection, CRC/raw capture/layout, front calibration,192 start cases,12 alternating A/F junctions per profile, outgoing mirroring, Z original-line return model, signed output and bounded power, rolling search, ramp bypass, legacy smoothing. All passed.
-Pico SDK2.2.0/GCC14.2.1, Pico W/RP2040 Release, -Wall -Wextra -Werror build passed. Optional unavailable WiFi/Bluetooth submodule warnings are expected for this USB-only image.
-UF2 RP2040 family/block/contiguous address checks, original V12 UF2 SHA256, frozen baseline manifest, required markers and image-before-data-bank limit passed. Packaged artifact matches build output.
-Record layout64848 bytes unchanged; imports1–18, unknown19 rejects, new run18 displays v12.2. Record17 displays v12.1. Raw49408-byte capture format unchanged.

## Artifact and memory

firmware/buddy23_circuit_with_barcode_v12_2.uf2
bytes246272;481 blocks; padded end0x1001e100
SHA25676941390c3779295ff750596a76d84e575a92c2b26bc78a3203771efa052641c
linker end0x1001e09c; first data bank0x101b0000
ELF text127128/data0/BSS215408
BSS end0x20035d10; StackLimit0x20040000; gap41712 bytes

## Physical test pending

Use profile1, first straight/bend then A barcode/junction. Actual achieved speed/smoothness/decoding require a physical run. Front sensors always steer; no heading lock. Strong departures retain an abrupt response where required. Possible barcode noise may keep the slower scan mode active; decoder still requires complete agreement. Nominal speeds can settle within1mm/s due target hysteresis. Motor power caps/PI unchanged. Sensor92mm axle geometry still needs updating after any remount. V12 rolling2s recovery remains, not V13+ stationary recovery. Profile2 remains the paused ramp experiment. Existing front calibration imports; no new centre/barcode calibration is introduced.
