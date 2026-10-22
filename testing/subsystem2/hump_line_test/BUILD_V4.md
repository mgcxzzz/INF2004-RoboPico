# Hump line V4 build, 2026-10-22

Artifact: `firmware/buddy23_hump_line_v4.uf2`

- SHA-256: `9c1931907cf62b9d8ad146c0d7462013636f7c0c855f6c7f4f0b164b63b2ea3b`
- 115712 bytes;226 sequential RP2040 UF2 blocks,256-byte payloads.
- Payload `[0x10000000,0x1000e200)`; ELF end `0x1000e174`.
- Reserved hump0x101fe000 and junction0x101ff000 sectors excluded from writes.
- Arm size:text61808,data0,BSS12152 bytes. USB stdio present;UART stdio absent.
- Pico SDK2.2.0, Arm GCC14.2.1, Pico W Release,C11,warnings as errors.

Profile2 adds bounded straight-hump carry while BOTH normalized levels<400.
Requires150ms both>=400/difference<=150 with measured forward speed>=15mm/s
on both wheels; the last trusted heading must be no older than1s. Carry targets
40/40 with heading correction capped8 (targets32..48). Stop at1s/25mm/8deg;
both levels>=400/difference<=350 for80ms returns normal optics. Flash ABI is
unchanged and existing V1..V3 records retain their recorded revision label.

Same30% ordinary/42% assisted limits,1.20x scaling,500ms stall,10s/500mm whole
run, direction/invalid-edge/button/USB/late-loop guards. No IMU integration.
Normal optical tracking and profile1 behaviour retain V3 policy. The bounded
carry explicitly keeps load assistance eligible without a current visible line.

Passed host suites `test_hump.c` and `test_navigation.c`:

- Qualification only from recent, moving, near-centred tracking; no carry
  from parked/all-white startup, stale observations or out-of-bounds heading.
- Positive and negative heading correction,8deg bound, exact1s/25mm boundaries.
-80ms both-sensor reacquisition; one-sensor hits/flickering black cannot
  extend budgets; immediate repeated blind carry cannot renew itself.
- Sampled V3 orange failure sequence: weak signal enters carry near4948ms;
 5406ms levels39/132 no longer immediately stop/revoke assistance.
- Sustained underspeed during carry can exceed300 PWM before the existing
 500ms stall window; output remains<=420. Optical replay is not a physical
 motor/traction simulation and does not prove the hump will be cleared.
- Previous wheel PI/zero/overspeed/effort/CRC/import/stop checks remain passing.
- UF2 identity/sequence/address/banners/build equality/hash verified.

Physical V4 testing is pending. This is a bounded temporary fallback; encoder
heading cannot establish lateral position or compensate unknown wheel slip.
No junction source or firmware is changed by this V4 task.
