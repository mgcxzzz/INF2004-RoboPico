# Hump line V5 build, 2026-10-22

Artifact: `firmware/buddy23_hump_line_v5.uf2`

- SHA-256: `556847ffaa607f8c8c3df039a73746f117dbe1b6908e8764137538933dbbc260`
- 116224 bytes;227 sequential RP2040 UF2 blocks,256-byte payloads.
- Payload `[0x10000000,0x1000e300)`; ELF end `0x1000e2cc`.
- No image writes to hump-record0x101fe000 or junction-record0x101ff000 sectors.
- Arm size:text62152,data0,BSS12152 bytes. USB stdio present;UART stdio absent.
- Pico SDK2.2.0, Arm GCC14.2.1, Pico W Release,C11,warnings as errors.

Final user request: six seconds of no-line allowance, including continuous
BOTH WHITE. V5 changes armed carry1s ->6s and distance25 ->240mm (6s at40mm/s).
Whole-run10 ->16s; path500mm retained. Trace125 ->200ms fits16s plus settling
in the same100-row saved ABI. Existing records1..4 remain readable; old STOP
time/distance messages retain their recorded values, new records label V5.

Retained:150ms recent centred moving qualification,1s recency, both levels<400
entry (including all-white<150), heading reference/correction/8deg bound,
80ms both-sensor reacquisition, no flicker deadline renewal, normal45mm/s
centred targets/carry40 with targets32..48, PWM30%/42%,500ms blocked-wheel guard,
button/USB/late-loop/direction/invalid-edge checks. Unarmed optical policy
remains650ms/25mm with recent direction or immediate stop without it.
The6s clock starts at carry entry, not again when readings fall below150.

Passed `test_hump.c` and `test_navigation.c`:

- Continuous both-white remains in carry at old1s/2s deadlines; active just
  before6s and stops at6s;240mm distance boundary and8deg heading checks.
- Successful80ms reacquisition after5s; one-sensor/flicker cannot renew6s.
- Qualification failures, zero/overspeed/assistance limits, legacy record imports,
  V1..V5 CRC/revision handling and existing optical/button/late-loop checks.
- Independent whole-run guard active before16s, stops at16s; trace capacity.
- V3 orange sampled optical failure replay still uses carry at the old stop.
- UF2 family/sequence/address/banners/build equality/hash verified.

Physical V5 outcome is pending. This is a bounded encoder fallback, not a
sensor-height fix or proof of line position/traction. Other guards can stop
earlier than6s. No IMU, junction source, or prior UF2 artifact is changed.
