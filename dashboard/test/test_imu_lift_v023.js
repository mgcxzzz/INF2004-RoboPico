'use strict';
/* v0.2.3 source-15 C9 IMU lift/overturn safety codec. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const IS28 = require('../is28_schema.js');

function putBits(bytes, bitOffset, width, value) {
  const n = value < 0 ? value + 2 ** width : value;
  for (let bit = 0; bit < width; bit++) {
    if (Math.floor(n / 2 ** bit) % 2)
      bytes[(bitOffset + bit) >>> 3] |= 1 << ((bitOffset + bit) & 7);
  }
}
function putU16(bytes, offset, value) { putBits(bytes, offset * 8, 16, value); }
function putU32(bytes, offset, value) { putBits(bytes, offset * 8, 32, value >>> 0); }
function frame(payload, seq = 9) {
  return Uint8Array.from([0xf2, payload.length, seq, ...payload]);
}
function imuLiftSafety({ reason = 1, latched = true, armed = true, fresh = true,
                         canReset = true, saturated = false, upProjection = -1536,
                         normMg = 1024, upDwellMs = 25, tiltDwellMs = 42,
                         uprightMs = 500 } = {}) {
  const p = new Uint8Array(27);
  p[0] = 0xc9; p[1] = 0x23;
  putU32(p, 2, 123456); putU32(p, 6, 123400); putU32(p, 10, 123390);
  putU16(p, 14, 7); putBits(p, 16 * 8, 13, upProjection);
  putBits(p, 18 * 8, 12, normMg);
  p[20] = reason | (latched ? 0x04 : 0) | (armed ? 0x08 : 0) |
    (fresh ? 0x10 : 0) | (canReset ? 0x20 : 0) | (saturated ? 0x40 : 0);
  p[21] = upDwellMs; p[22] = tiltDwellMs;
  putU16(p, 23, uprightMs); putU16(p, 25, 0x1234);
  return p;
}

test('C9 decodes the firmware-generated lift-stop snapshot', () => {
  const fixture = path.join(__dirname, '..', '..', 'testing', 'rtos_migration', 'build-host', 'imu_lift.bin');
  assert.ok(fs.existsSync(fixture), `missing pure-C C9 fixture: ${fixture}`);
  const f = IS28.decodeFrame(new Uint8Array(fs.readFileSync(fixture)));
  assert.equal(f.error, undefined, f.error);
  assert.equal(f.src, IS28.SRC.SOURCE15); assert.equal(f.len, 27);
  assert.deepEqual(f.data, {
    kind: 'imu-lift-safety', version: '0.2.3', uptimeMs: 123456, tripMs: 123400,
    lastSampleMs: 123390, eventCount: 7, upProjection: -1536, normMg: 1024,
    reason: 1, reasonName: 'lifted', latched: true, armed: true, fresh: true,
    canReset: true, saturated: false, upDwellMs: 25, tiltDwellMs: 42,
    uprightMs: 500, runId: 0x1234
  });
});

test('C9 reports overturn separately and preserves the signed projection and run identity', () => {
  const d = IS28.decodeFrame(frame(imuLiftSafety({ reason: 2, upProjection: -4096, normMg: 4095 }))).data;
  assert.equal(d.kind, 'imu-lift-safety'); assert.equal(d.reasonName, 'overturn');
  assert.equal(d.upProjection, -4096); assert.equal(d.normMg, 4095); assert.equal(d.runId, 0x1234);
});

test('C9 rejects versions, truncation, reserved bits, and out-of-range dwell values', () => {
  const mutate = (offset, mask, value) => {
    const p = imuLiftSafety(); p[offset] = (p[offset] & ~mask) | value;
    return IS28.decodeFrame(frame(p));
  };
  assert.match(mutate(1, 0xff, 0x22).error, /version/);
  assert.match(mutate(17, 0xe0, 0x20).error, /reserved/);
  assert.match(mutate(19, 0xf0, 0x10).error, /reserved/);
  assert.match(mutate(20, 0x80, 0x80).error, /reserved/);
  assert.match(mutate(21, 0xe0, 0x20).error, /reserved/);
  assert.match(mutate(22, 0xc0, 0x40).error, /reserved/);
  assert.match(mutate(24, 0xfe, 0x02).error, /reserved/);
  assert.match(IS28.decodeFrame(frame(imuLiftSafety({ upDwellMs: 31 }))).error, /range/);
  assert.match(IS28.decodeFrame(frame(imuLiftSafety({ tiltDwellMs: 51 }))).error, /range/);
  assert.match(IS28.decodeFrame(frame(imuLiftSafety({ uprightMs: 501 }))).error, /range/);
  assert.match(IS28.decodeFrame(frame(imuLiftSafety().slice(0, 26))).error, /unknown.*layout/);
  assert.match(IS28.decodeFrame(frame(imuLiftSafety()).slice(0, 29)).error, /length/);
});

test('C3 accepts the v0.2.3 BR_IMU bit in its 8-bit supervisor flags', () => {
  const p = new Uint8Array(60); p[0] = 0xc3; p[59] = 0x80;
  const d = IS28.decodeFrame(frame(p)).data;
  assert.equal(d.kind, 'runtime'); assert.equal(d.controlFlags, 0x80); assert.equal(d.imuStop, true);
});
