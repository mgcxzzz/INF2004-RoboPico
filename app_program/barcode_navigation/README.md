# Buddy3 RTOS barcode adapter

Experimental Step4. This folder adapts the SDK reference to the already tested
Step3B motion foundation. IMU/ramp and Buddy5 are not active in this image.

`barcode_model.h` is hardware-free. `line_service.c` owns ADC0/1/2 and calls it
on the existing 2ms GP27 four-reading average. Between100Hz encoder updates,
the existing velocity interpolation estimates edge distance. Both decoders
receive exactly the same ADC/time/wheel inputs, original first, peak second.

The reference63-bank decoder needs neighbouring thresholds to agree on a
complete Code39 frame. The peak decoder requires the complete15-bar `*X*`
frame, classifies the relative peaks/spaces and supports either direction.
The shared decoder headers and their thresholds are unchanged. Replay
fixtures live in `firmware/integration/buddy1_4/tests/barcode/`.

When a decoder accepts a letter, `barcode_queue_code` calls the existing
Buddy3 `nav_accept_barcode` API. The coordinator checks session/owner state.
The planner then captures that letter's current mapped action AND revision.
Changing the dashboard map affects subsequent decodes; it does not alter a
previously verified pending turn. Unassigned letters are logged and continue
following without a turn. There are no boot-time A/F/Z assignments.

If both decoders read the same physical frame, they queue it once. A different
letter on the same frame cancels the pending command and records a conflict.
The selecting stop bar is never reused as the next junction. The existing
width11–30mm,4mm white confirmation, straight gate and Step2 turn/acquisition
policy still apply. Motor targets continue through the shared coordinator
and the single motion writer; this folder never writes PWM.

During Pause, no new barcode is consumed. Qualified Resume preserves completed
events and an already verified pending action/revision, clears partial reader
and crossing state, and reacquires from the current position. New Start resets
the run. Turns suppress barcode reading as in the reference. Safety loss ends
the session; there is no automatic restart.

`barcode_runtime.inc` serializes copied dashboard packets: source10 line state,
source11 decoded/queued/cross/dispatch/completed events and source15 peak
diagnostics. All send calls use non-blocking copied application buffers.
Only the kernel Wi-Fi service calls lwIP/MQTT. Queue timestamps use the kernel
epoch; raw history uses elapsed run time. Source11 events stay FIFO; only its
periodic STATUS frame may be coalesced. All source11 producers share a sequence
allocator, so mixed status/event traffic cannot create artificial sequence gaps.

The raw history is2048 explicitly packed10-byte rows: elapsed ms32 bits,
signed distance in0.01mm32 bits, ADC12 bits with a reserved upper nibble. It
holds the newest~4.1s of2ms samples and freezes at the first accepted decode.
Without a decode, it keeps rolling until Stop. Export with USB `b` only while
stopped. Reference barcode events (192 rows) retain letter/action/map revision.
To retain all63 banks plus this raw trace, Step4's100Hz line ring is640 rows
(6.4s). The comparison Step3B image keeps1536 rows. Neither image writes these
run logs to flash. Start is inhibited throughout a stopped export.

Tests exercise the new adapter with real captured failed Z/Y inputs, synthetic
A–Z in both directions, live map revision freezing, unmapped letters, partial
frame rejection after Resume and raw-ring integrity. The reference peak
integration tests separately cover both conflict orders, duplicate suppression
and junction separation. These replays are software evidence, not proof of
physical sampling/optics on the migrated car. See the Step4 test guide.
