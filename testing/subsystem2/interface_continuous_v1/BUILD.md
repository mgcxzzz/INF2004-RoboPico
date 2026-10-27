# Interface Continuous v1 build evidence — 2026-10-28

Base: Interface V1.1 HANDOVER1 at`de9bacd792ba514c8340070c0789f17941777c02`. Separate folder and artifact. The default extension source is empty; no Buddy5 obstacle policy is added or enabled.

## Executed validation

- **26 host executables passed**, compiled with Zig C11 and`-Wall -Wextra -Werror`. Retained tests cover nav ownership/stop/clock/finite commands, compact codecs and flash CRC, optical tracking, complete A/F/Z frames both sizes/directions and queued cross dispatch, junction turns and white-loss/search.
- New motor checks cover continuous output/rate bounds, falling reference slew, integral recovery, one-shot/early-ending startup aid, explicit zero/invalid input, and illustrative stiction/load models. These models verify software properties, not physical torque or smoothness.
- New steering checks cover recorded-style47/13 command transitions, bounded mirrored corrections, eventual stopped-inner-wheel authority, profile/scan transitions and immediate loss/stop/planned-phase bypass. New wrapper tests compare the original optical demand and navigation state under identical input traces, exact profile2 legacy adapter parity, scan activity begin/end and mirrored A/F/Z dispatch. Actual trajectory will differ and is not proven by identical-input replay.
- New coordinator integration test routes continuous outputs through the generation-checked commit, verifies takeover/off, rejects a stale commit, resets private state, returns ownership to Buddy3 after line resume and rejects the expired lease. This is a host mock of the port, not an ultrasonic integration.
- Earlier parity tests remain checks of retained baseline modules; their old success messages about unchanged steering refer to those baseline modules, not this new ordinary output adapter.
- Diagnostic codec459bits/58bytes, current format3 CRC/readback restore and rejection of old-format attribution passed. The new scheduled continuous window at3s is tested alongside old jump/reversal triggers.
- **Three Pico W Release builds passed** with`-Wall -Wextra -Werror`: default, unpacked shared-state reference, and the unchanged optional Buddy5 example. The example is only a build compatibility check and is **not linked into the delivered default UF2**. Pico SDK2.2.0, ARM GCC14.2.1, existing local CMake/Ninja. No new dependencies.

| ARM build | text | data | BSS |
|---|---:|---:|---:|
| HANDOVER1 default |163696|0|237372|
| Continuous default |170320|0|237472|
| Continuous unpacked shared state |169248|0|242432|
| Continuous optional existing example |173048|0|237504|

The new private controller/steering/output state adds100B BSS relative to HANDOVER1. Like-for-like shared-state packing still saves4960B. Diagnostic payload remains14848B. This is partial data-representation evidence, not a project-wide minimum-bit compliance claim.

Default symbols:`__flash_binary_end=0x10028954`,`__bss_end__=0x2003b450`,`__StackLimit=0x20040000`. The19376B between BSS end and StackLimit is not a measured worst-case heap/stack margin. Padded UF2 ends at0x10028a00, below diagnostic flash bank0x101ab000. Diagnostic/nav/raw/motion offsets remain0x1ab000/0x1af000/0x1b0000/0x1d0000. No diagnostic flash/UART work occurs while moving.

## Artifact and provenance

`firmware/buddy23_interface_continuous_v1.uf2`: **332800bytes**, **650contiguous RP2040 UF2 blocks**. SHA256:`8f5ec682c7d1e530892eadc272c21a4d55e4de8268da9275d54059e06c720843`.

`CONTINUOUS_BASELINE.json` records HANDOVER1 source hashes. `CONTINUOUS_PATCHES.json` accounts for every new/changed C/H source. Public APIs, nav coordinator, executor, codec, hooks and optional example sources are byte-identical. Original optical/calibration and barcode/junction/manoeuvre headers are protected. Explicit changes are the three new ordinary-control adapters, hardware/harness dispatch and limits, diagnostic attribution/window scheduling, normal record24, and their tests. Older baseline manifests/build files are inherited historical records, not claims about this artifact.

Run`C:/Python310/python.exe testing/subsystem2/interface_continuous_v1/run_tests.py` from repository root for host tests. Configure/build the root CMake project as in the inherited build instructions (target name remains`buddy23_interface_v1` for integration compatibility). Default `BUDDY_EXTENSION_SOURCE` is empty. Run`verify_firmware.py` to verify packaged identity, source scope, old artifact preservation, default empty hooks and flash bounds.

First physical test uses profile1, existing valid calibration, same mounting/mat and normal startup. The full autonomous harness remains enabled: line following, bends, A/F/Z scanning, queued junction turns and continued following. The recommended short straight-and-bend comparison is an initial test, not a firmware restriction. Stop/wait5s and collect p+d. Physical smoothness, tighter-bend capture and barcode consistency are **not yet validated**.
