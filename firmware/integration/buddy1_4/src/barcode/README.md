# Barcode decoding and junction routing — Buddy 3

**PROVISIONAL Buddy1–4 integration. Buddy5 is not integrated.**

## Two readers, one verified-result path

The GP27 analog sensor is averaged at roughly2ms intervals. Both readers consume the same ADC sample and mean encoder path. They are independent of the front `w/l/r/k` calibration.

The original reader (`barcode_analog.h`, `barcode_scan.h`) runs63 threshold banks with debounce and hysteresis. Each bank converts edges into black/white run widths, validates Code39 start/data/stop patterns in either direction, and votes on complete frames. Agreeing adjacent complete-frame banks select a result; disagreeing votes suppress a route. It remains available for sharp, high-contrast labels.

The peak reader (`barcode_peak.h`) addresses blurred signals whose narrow bars are only small bumps and cannot share one global threshold with all gaps:

1. Smooth ADC over about0.5mm of encoder travel and at least5 samples.
2. Track white baseline over twelve5mm bins. Find local maxima with minimum prominence/rise and merge peaks less than3mm apart.
3. Accumulate a group; after sufficient trailing quiet, try15-bar windows representing a complete `*X*` frame.
4. Classify bar widths from rise relative to the tallest bar: narrow below15%, wide at least50%, ambiguous heights rejected. Infer spaces from peak-centre spacing with an estimated2–4.5mm narrow unit and a3:1 width model.
5. Require six wide black bars, valid narrow character gaps, unambiguous spaces and valid sentinels/data in exactly one direction.
6. If a16–18-peak group has no valid raw window, retry after dropping one or two of the weakest candidate narrow noise peaks. Successful interpretations must agree on one character.

These are optical/label assumptions, not a universal decoder. A sharp sensor can give equal peak heights for both widths, leaving this reader unable to classify them; the original reader then remains useful.

## Acceptance, mapping and junctions

The barcode adapter deduplicates overlapping matching reads. A conflicting letter cancels the pending action and records a conflict. Both physical readers use `barcode_queue_code`, whose integrated-build hook calls `nav_accept_barcode(MOTION_BUDDY3, code)` for coordinator approval before updating physical decode counters and pending navigation. Unmapped A–Z letters are recorded but do not queue movement. The public API for an injected route is separate and does not claim a physical decode.

The pending letter captures its action and mapping revision at decode time. Dashboard changes affect later reads, not the action already attached to a queued letter. A confirmed separate cross, not a barcode bar, triggers the manoeuvre. Existing cross width11–30mm and4mm white confirmation remain. The side sensor is92mm ahead; turn initiation uses the existing55mm-before-centre geometry. Left/right rolling turns and U-turn acquisition remain unchanged.

At a new run or a navigation Resume, reset both partial readers. A complete pending route may survive the existing handback policy; half a label may not. The new regression feeds half a real Z capture, simulates a30s interruption, resets through the production hook and verifies the remainder cannot queue a route. A fresh full label still decodes afterward.

## Evidence and outputs

Four committed captured failures (three Z, one Y) decode in the peak replay suite. The teammate recorded reverse Y on combinedv0.5, but that Y was unmapped; host tests cover a mapped peak result followed by a separate cross. This does not substitute for a new physical mapped-turn test. Original event logs are preserved in the evidence archive.

`j` prints detailed barcode events when stopped; raw sensor export contains only the first2048 samples, so it may miss a later failed scan. Source15 subtype0xB1 reports peak decisions. Send raw-sensor, motion, navigation and barcode events together when diagnosing a scan.

[Recorded evidence and provenance](../../../../../testing/evidence/README.md).
