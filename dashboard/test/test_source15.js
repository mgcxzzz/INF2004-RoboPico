'use strict';
/* Source-15 coexistence + combined reporting path (node --test test/test_source15.js).
 * Fixture sdk_b4_peak_c_frames.txt is produced by the REAL C code of the combined build
 * (testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_1/barcode_peak_tests/gen_dashboard_frames.c):
 * original decoder + peak harness on a real failed Y capture, ramp-assist packer interleaved,
 * wp_map_frame readback and wpk_line/wpk_nav packers. No broker, Pico or car. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const M = require('../is28_schema.js');
const frames = fs.readFileSync(path.join(__dirname, 'fixtures', 'sdk_b4_peak_c_frames.txt'), 'utf8')
  .split('\n').filter((l) => l.startsWith('TLM ')).map((l) => Uint8Array.from(Buffer.from(l.slice(4), 'hex')));
const decoded = frames.map((b) => M.decodeFrame(b));

test('every C-packed frame of the combined build decodes without error', () => {
  assert.ok(frames.length > 100);
  for (const [i, f] of decoded.entries()) assert.ok(!f.error, `frame ${i}: ${f.error}`);
});

test('source 15: ramp-assist and barcode-peak layouts interleave and dispatch by length/subtype', () => {
  const s15 = decoded.filter((f) => f.src === 15);
  const ramp = s15.filter((f) => f.data.kind === 'ramp'), peak = s15.filter((f) => f.data.kind === 'peak');
  assert.equal(ramp.length + peak.length, s15.length);
  assert.ok(ramp.length >= 10 && peak.length === 2);
  assert.ok(ramp.every((f) => f.len === 28 && f.data.detector === 2 && f.data.estimator === 2 && f.data.active));
  assert.ok(peak.every((f) => f.len === 24 && f.data.version === 1 && f.data.combinedBuild && f.data.active));
  const firstPeak = decoded.findIndex((f) => f.src === 15 && f.data.kind === 'peak');
  const lastPeak = decoded.map((f) => f.src === 15 && f.data.kind === 'peak').lastIndexOf(true);
  assert.ok(decoded.slice(firstPeak, lastPeak).some((f) => f.src === 15 && f.data.kind === 'ramp'), 'ramp frames between peak frames');
  const [announce, label] = peak.map((f) => f.data);
  assert.equal(announce.reason, 10); assert.equal(announce.reasonName, 'run start'); assert.equal(announce.bars, 0);
  assert.equal(label.code, 'Y'); assert.ok(label.decoded && label.applied && !label.originalConflict && !label.originalSameFrame);
  assert.equal(label.bars, 15); assert.equal(label.reasonName, 'ok');
  /* independent sequence counters per layout */
  const rs = ramp.map((f) => f.seq); assert.deepEqual(rs, rs.map((_, i) => (rs[0] + i) & 255));
  assert.deepEqual(peak.map((f) => f.seq), [0, 1]);
});

test('real SDK decode reaches source 10/11 and the mapping gives the movement', () => {
  const map = decoded.find((f) => f.src === 8).data;
  assert.ok(map.configured); assert.deepEqual(map.letters, ['Y', 'Z', 'A', 'F']);
  const navDecoded = decoded.filter((f) => f.src === 11 && f.data.event === 2);
  assert.equal(navDecoded.length, 1); assert.equal(navDecoded[0].data.letter, 'Y'); assert.equal(navDecoded[0].data.action, 'S');
  const queued = decoded.filter((f) => f.src === 11 && f.data.event === 3);
  assert.equal(queued.length, 1);
  const lines = decoded.filter((f) => f.src === 10).map((f) => f.data);
  assert.equal(lines[0].letter, '-');
  assert.equal(lines[lines.length - 1].letter, 'Y'); assert.equal(lines[lines.length - 1].pendingLetter, 'Y');
  assert.equal(M.mappedMovement(map, 'Y'), 'Forward');
});

test('malformed and foreign source-15 frames are rejected, never guessed', () => {
  const fr = (p) => Uint8Array.from([0xF2, p.length, 1, ...p]);
  const peakPayload = Array.from(frames.find((b) => b[0] === 0xF2 && b[1] === 24).slice(3));
  const rampPayload = Array.from(frames.find((b) => b[0] === 0xF2 && b[1] === 28).slice(3));
  assert.match(M.decodeFrame(fr(peakPayload.slice(0, 23))).error, /unknown source-15 layout/);
  assert.match(M.decodeFrame(fr([...peakPayload.slice(0, 4), 0xB2, ...peakPayload.slice(5)])).error, /unknown source-15 layout/);
  assert.match(M.decodeFrame(fr([...rampPayload, 0])).error, /unknown source-15 layout/);
  assert.match(M.decodeFrame(fr([])).error, /unknown source-15 layout/);
  assert.match(M.decodeFrame(Uint8Array.from([0xF2, 24, 1, ...peakPayload.slice(0, 20)])).error, /length/);
  assert.match(M.decodeFrame(Uint8Array.from([0xF1, 24, 1, ...peakPayload])).error, /schema version/);
  assert.equal(M.decodeFrame(fr(rampPayload)).data.kind, 'ramp');
  assert.equal(M.decodeFrame(fr(peakPayload)).data.kind, 'peak');
});

test('profile detection: source 15 is SDK-only', () => {
  assert.equal(M.detectProfile([1, 15]), 'sdk');
  assert.equal(M.detectProfile([1, 3, 15]), 'mixed');
  assert.equal(M.detectProfile([1, 2, 4]), 'unknown');
});
