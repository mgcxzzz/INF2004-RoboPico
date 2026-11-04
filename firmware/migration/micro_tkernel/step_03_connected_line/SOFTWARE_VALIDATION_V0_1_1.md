# Step3B v0.1.1 software validation — 2026-11-04

Hardware acceptance pending. No robot was flashed or driven during these
software checks. The exact source commit and binary hashes are recorded in
the adjacent v0_1_1 artifact BUILD_MANIFEST/SHA256SUMS.

- 22 strict production-C regression executables passed, including the new
encoder-quality checks, copied latest-stream/priority tests and the production
kernel MQTT pump with deterministic lwIP/radio doubles. Existing steering,
drive, coordinator/API, stop/stale-generation, ADC, junction and wire-codec
regressions passed. Network doubles do not establish real RF timing.
- 51 Python timing/capture/export/provisioning checks passed. Idle clock tests
remain strict; connected captures may include motion without falsely marking
healthy timing FAIL. Actual motor latches remain readiness blockers.
- 27 dashboard protocol/compatibility tests and50 simulated browser assertions
passed. No broker/Pico/car was used for the browser simulation.
- ARM Step2, Step3A and Step3B profiles built. Final connected image has1626
UF2 blocks, XIP0x10000000..0x10065a00, below the0x10100000 ceiling and saved
settings/calibration sectors. Known RP2040 RAM-execution linker RWX warning
remains; no new compiler warnings/errors were observed.
- 101 shared main-reference files match byte-for-byte. The same two reviewed,
opt-in Step2 junction-departure edits remain; no new shared steering, motor,
barcode, IMU, navigation API or calibration changes were introduced.
- Static RAM ends at0x2002d3cc, leaving76852 bytes before dynamic kernel/task
allocations (224 additional static bytes versus v0.1). The larger MQTT client
also consumes more of the existing16KiB lwIP arena. Largest individual
application stack frame is880 bytes, below the2048-byte packaging check;
this is not a measured task/call-chain high-water mark.
- Flash park/writer symbols remain in RAM and contain no direct XIP calls.
UF2 family/block/bounds and ZIP/source-byte verification are packaging gates.

Required hardware evidence is the stationary Telemetry off/on baseline,
short straight Pause/Resume/Stop and full stopped log, a longer tracking run,
local emergency stops and reconnect/no-automatic-restart tests described in
[UPDATE_V0_1_1.md](UPDATE_V0_1_1.md). Extended-mat L/R and sharp-bend acceptance
are still outstanding Step2 items. Barcode/IMU remain later migration steps.
