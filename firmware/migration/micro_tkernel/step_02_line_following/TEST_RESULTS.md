# Step 2 v0.1 — software verification

Date: 2026-11-02. **Hardware acceptance pending.** This is an experimental
checkpoint, not a final integrated release. Step 1 hardware acceptance is
recorded separately; it cannot certify the new ADC/line/flash boundary.

## Passed software checks

- **11 strict C regression programs** using production algorithms. Existing
  ContinuousDrive, Continuous V2 steering/full wrapper, shared coordinator
  ownership, stale generation rejection, clock/request age ordering, coordinated
  assist, line search and U-turn tests remain passing.
- New production Step 2 adapter tests compare its emitted demands with the
  unchanged full profile 1 wrapper for straight tracking, sustained bends,
  alternating corrections, scan-activity transitions, white gaps and search.
  Added outgoing-line acquisition comparisons cover left, mirrored right and
  U-turn routes through completion. These are host input/geometry tests.
- Bare-cross qualification tests cover 18 mm black plus 4 mm white, rejected
  narrow bars and cancellation by a second bar before white confirmation.
- Calibration tests cover empty/corrupt sectors, SDK record 28 CRC import,
  own-page priority, all 256 single-byte page corruptions and fallback. Own
  references are packed to 12 bits each with explicit little-endian encoding.
- Power-envelope tests cover ordinary cap, U-turn reverse, active search,
  bounded peak expiry and a safety reduction that cannot re-enable an already
  stopped output or switch its powered channel. Full ADC/steering/route packing
  boundaries round-trip without altering the shared controller types.
- Start policy rejects stale commands and queued commands from before STOP,
  including timer wraparound and long idle. A separate cancellation generation
  protects the hands-free countdown; export cancels it too.
- **24 Python checks**: 15 idle timing/readiness analyzer tests and 9 stopped-log
  export tests for complete, missing/reordered/malformed/out-of-range/interrupted
  data, duplicate dumps, interleaved status and an empty history. A complete
  export is explicitly distinguished from driving acceptance.
- Both `APP=rtos_bringup` and `APP=rtos_line` build. New application code uses
  `-Wall -Wextra -Werror`. Reference main feature source/config/interfaces are
  verified byte-for-byte by the packaging tool.

The Windows host compiler uses SSE2 float32 arithmetic for these checks;
its default x87 extended-precision constants made the untouched steering test's
exact `.35f` boundary compare incorrectly. No production gain or threshold was
changed to accommodate the host compiler.

## Linked image checks

- Arm GCC 14.2.1 build: `text=104516`, `bss=123532` bytes as reported by `size`.
  The `.data` region includes executable RAM routines, so this tool classifies
  those bytes as text rather than ordinary initialized data.
- Static RAM end `0x2001e340`; **138432 bytes** remain below the main 256 KiB RAM
  ceiling before dynamic kernel/task allocations. Application tasks request
  4 KiB stacks; static compiler frame checks are not a measured high-water mark.
- Control frame 568 bytes, line-follow wrapper 296, power adapter 208 and
  export frame 368. All application static frames are below 2048 bytes.
- Flash parking routine at `0x200000e4`, writer at `0x2000011c`. Both are in
  copied RAM `.data`; disassembly has no direct call into XIP from those routines.
  Flash operations use prepared ROM pointers and the RAM boot2 copy.
- The linker reports an RWX load segment because `.data` intentionally holds
  RAM executable functions on RP2040. This is expected for this layout.
- Packaging validates UF2 family/indices/addresses, the code ceiling below
  saved flash sectors, required/forbidden linked symbols, ZIP integrity and
  all source hashes. Manifest records exact binary hashes and memory checks.

Evidence: `logs/host_tests.txt`, `logs/arm_build.txt`,
`logs/step1_regression_build.txt`, `logs/linked_checks.json`, and the
`firmware/BUILD_MANIFEST.json` generated with this package.

## Pending car checks

| Check | Status |
|---|---|
| Motors off at boot; original stop behavior on this image | Pending |
| Current car's calibration correctly imported/read back | Pending |
| ADC ranges and control timing at current sensor mounting | Pending |
| Optional stopped `k` save, read-back and reboot persistence | Pending |
| Short straight, then gentle and sharp bends | Pending |
| Bare-cross left/right/U-turn plus outgoing-line reacquisition | Pending |
| Complete log export, terminal stop sample and video correlation | Pending |

The known SDK sharp-bend and junction-reacquisition problems have not been
declared fixed or retuned in this port. The purpose of these logs is to identify
whether the same symptoms reproduce with correct RTOS timing and calibration.
