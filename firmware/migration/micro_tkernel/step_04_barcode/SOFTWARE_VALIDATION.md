# Step4 software evidence — 2026-11-04

Hardware acceptance of this package is pending. No physical car was flashed
or driven during software verification. Exact source commit, compiler/SDK,
UF2 hashes, memory addresses and individual stack frames are recorded in each
artifact BUILD_MANIFEST. Source and generated ZIP contents are byte-verified.

- 25 strict C regression programs passed. This includes the real coordinator,
API/commit/stop/stale-generation protections, continuous steering/drive,
ADC recovery, both junction departure policies, packet codecs and the kernel
MQTT pump with deterministic transport doubles.
- The migrated barcode adapter passes real captured Z/Y replays, synthetic A–Z
in both print directions, coordinator-approved decoding, no route for unmapped
letters, frozen pending action/revision across map updates/Resume, partial
frame rejection after interruption and raw-ring integrity. A separate cross
dispatches each mapped S/L/R/U action; the stop bar alone does not dispatch.
These are software trajectories, not evidence of physical turn alignment.
- The unchanged reference combined-peak integration passes28 checks covering
duplicate suppression, both conflict orders, unmapped letters, busy/reset,
junction separation and mixed source15 frames. Pure peak decoding passes47
checks including four real captured car failures, synthetic labels, negatives,
harsh optics/noise and telemetry. Captured replay success is not a new car test.
- 55 Python capture/export/provisioning checks pass. New barcode export checks
reject missing markers, wrong row counts, malformed fields, invalid ADC/
patterns and reordered data; stationary empty exports remain valid.
- The pinned upstream TCP allocator is compiled under old/new options. Fourteen
60-byte writes reserve20440 versus840 payload bytes (headers/metadata extra).
Source inspection verifies CYW43's existing pbuf-chain copy path. New heap
diagnostics are needed to confirm the car's actual network improvement.
- 27 dashboard protocol/compatibility tests and53 browser assertions pass using
simulated MQTT. Step4 identification, Busy semantics, disabled unsupported
stored downloads and a reboot to the comparison image are covered. No Pico,
RF link or real broker is involved in that browser simulation.
- ARM regression builds cover Step2 and Step3A. Published Step3B/Step4 images
are built from the packaged source, with matching per-profile stack metadata.
The known RAM-execution RWX linker warning remains; new application code uses
strict compiler warnings as errors. RAM park/write code stays in RAM with no
direct XIP calls. Static RAM must end below0x20030000; individual application
stack frames must remain below2048 bytes. These checks do not measure the
whole task/call-chain high-water mark.
- 101 shared reference source files remain byte-identical. The same two earlier
opt-in junction-departure edits remain; decoder thresholds, line calibration
presets, steering, motor controller and navigation API sources are unchanged.

This validates software integration and packaging, not the RF cause of every
heartbeat gap, real2ms sampling cadence, optics, junction rejoining or ramps.
The Step4 README specifies the physical network/scan/mapped-action tests.
