const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const s = require('../is28_schema.js');

const le = (value, bytes) => Array.from({length:bytes}, (_, i) => (value >>> (8 * i)) & 255);
const frame = (payload, seq = 1) => Uint8Array.from([(15 << 4) | 2, payload.length, seq, ...payload]);

assert.equal(s.driveModeArg(0), 0x0002);
assert.equal(s.driveModeArg(1), 0x0102);
assert.equal(s.driveModeArg(2), 0x0202);
assert.throws(() => s.driveModeArg(3), /mode must be 0, 1, or 2/);

const addedPwm = 11 | (23 << 7);
const cb = [0xcb, 0x25, ...le(4567, 4), ...le(9, 4), 0x8e, 0x07,
  addedPwm & 255, (addedPwm >>> 8) & 255];
const mode = s.decodeFrame(frame(cb));
assert.equal(mode.error, undefined);
assert.equal(mode.data.kind, 'drive-mode-status');
assert.equal(mode.data.mode, 2);
assert.equal(mode.data.requestedMode, 3);
assert.equal(mode.data.canChange, true);
assert.equal(mode.data.sessionActive, true);
assert.equal(mode.data.loadLeft, true);
assert.equal(mode.data.loadRight, true);
assert.equal(mode.data.addedPwmLeft, 11);
assert.equal(mode.data.addedPwmRight, 23);
const nativeModePath = path.join(__dirname, '..', '..', 'testing', 'rtos_migration', 'build-host', 'drive_mode.bin');
assert.ok(fs.existsSync(nativeModePath), `missing firmware-generated CB fixture: ${nativeModePath}`);
const nativeMode = s.decodeFrame(new Uint8Array(fs.readFileSync(nativeModePath)));
assert.equal(nativeMode.error, undefined, nativeMode.error);
assert.equal(nativeMode.data.uptimeMs, 0x01020304);
assert.equal(nativeMode.data.revision, 0x11223344);
assert.equal(nativeMode.data.mode, 2);
assert.equal(nativeMode.data.requestedMode, 2);
assert.equal(nativeMode.data.canChange, true);
assert.equal(nativeMode.data.sessionActive, false);
assert.equal(nativeMode.data.addedPwmLeft, 23);
assert.equal(nativeMode.data.addedPwmRight, 37);
const cbReserved = cb.slice(); cbReserved[11] |= 0x80;
assert.match(s.decodeFrame(frame(cbReserved)).error, /drive-mode status/);

const c9 = [0xc9, 0x25, ...le(123456, 4), ...le(123400, 4), ...le(123390, 4),
  ...le(7, 2), 0, 0, 0xe8, 0x03, 0x1d, 30, 100, ...le(250, 2), ...le(7, 2)];
const safety = s.decodeFrame(frame(c9, 2));
assert.equal(safety.error, undefined);
assert.equal(safety.data.kind, 'imu-lift-safety');
assert.equal(safety.data.version, '0.2.5');
assert.equal(safety.data.tiltDwellMs, 100);
assert.equal(safety.data.runId, 7);
const legacy = c9.slice(); legacy[1] = 0x23; legacy[22] = 42;
assert.equal(s.decodeFrame(frame(legacy, 3)).data.tiltDwellMs, 42);
const invalidTilt = c9.slice(); invalidTilt[22] = 101;
assert.match(s.decodeFrame(frame(invalidTilt, 4)).error, /IMU lift-safety/);

const ca = [0xca, 1, ...le(123456, 4), ...le(123390, 4), ...le(-12345, 2),
  ...le(1000, 2), ...le(6656, 2), 100, 30, 0x72, ...le(120000, 4), ...le(7, 2)];
const peak = s.decodeFrame(frame(ca, 5));
assert.equal(peak.error, undefined);
assert.equal(peak.data.kind, 'imu-lift-peak');
assert.equal(peak.data.minCosQ15, -12345);
assert.equal(peak.data.normMg, 1000);
assert.equal(peak.data.peakUpMg, -1536);
assert.equal(peak.data.maxTiltDwellMs, 100);
assert.equal(peak.data.maxUpDwellMs, 30);
assert.equal(peak.data.candidate, 2);
assert.equal(peak.data.armed && peak.data.currentValid && peak.data.latched, true);
assert.equal(peak.data.windowStartMs, 120000);
assert.equal(peak.data.eventCount, 7);
const nativePeakPath = path.join(__dirname, '..', '..', 'testing', 'rtos_migration', 'build-host', 'imu_lift_peak.bin');
assert.ok(fs.existsSync(nativePeakPath), `missing firmware-generated CA fixture: ${nativePeakPath}`);
const nativePeak = s.decodeFrame(new Uint8Array(fs.readFileSync(nativePeakPath)));
assert.equal(nativePeak.error, undefined, nativePeak.error);
assert.equal(nativePeak.data.kind, 'imu-lift-peak');
assert.equal(nativePeak.data.reportMs, 123456);
assert.equal(nativePeak.data.peakSampleMs, 123390);
assert.equal(nativePeak.data.minCosQ15, -12345);
assert.equal(nativePeak.data.normMg, 1000);
assert.equal(nativePeak.data.peakUpMg, -1536);
assert.equal(nativePeak.data.maxTiltDwellMs, 100);
assert.equal(nativePeak.data.maxUpDwellMs, 30);
assert.equal(nativePeak.data.lastCandidate, 2);
assert.equal(nativePeak.data.lastRejection, 1);
assert.equal(nativePeak.data.armed && nativePeak.data.currentValid && nativePeak.data.latched, true);
assert.equal(nativePeak.data.windowStartMs, 120000);
assert.equal(nativePeak.data.eventCount, 7);
const noPeak = s.decodeFrame(frame([0xca, 1, ...Array(23).fill(0)], 6));
assert.equal(noPeak.data.hasPeak, false, 'an empty retained CA snapshot must not invent a zero-degree peak');

console.log('PASS v0.2.5 overview wire contracts: drive mode CONFIG/status, C9 extended dwell, and retained CA peak evidence.');
