'use strict';
/* v0.2.1 source-15 diagnostic layouts: packed encoder snapshots (C4) and
 * barcode raw-capture metadata (C5). No broker, Pico or car is used. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const IS28 = require('../is28_schema.js');

function frame(src, payload, seq = 1) {
  return Uint8Array.from([(src << 4) | 2, payload.length, seq & 255, ...payload]);
}
function putBits(bytes, bitOffset, width, value) {
  let n = value < 0 ? value + 2 ** width : value;
  for (let bit = 0; bit < width; bit++) {
    if (Math.floor(n / 2 ** bit) % 2)
      bytes[(bitOffset + bit) >>> 3] |= 1 << ((bitOffset + bit) & 7);
  }
}
function putU32(bytes, offset, value) { putBits(bytes, offset * 8, 32, value >>> 0); }
function putI16(bytes, offset, value) { putBits(bytes, offset * 8, 16, value); }
function encoderFault({ kind = 1, wheel = 1, duration = 0, targetL = -45, targetR = 135,
                        pwmL = -333, pwmR = 777, speedL = -120, speedR = 45 } = {}) {
  const p = new Uint8Array(45);
  p[0] = 0xc4; p[1] = 0x21; p[2] = 1 | (kind << 1) | (wheel << 3) | 0x20;
  putU32(p, 3, 0x12345678); putU32(p, 7, 9123); putBits(p, 11 * 8, 17, duration);
  putBits(p, 14 * 8, 40, 123 | (456 * 2 ** 20));
  putU32(p, 19, 10000); putU32(p, 23, 9); putU32(p, 27, 12000); putU32(p, 31, 12);
  putBits(p, 35 * 8, 9, targetL); putBits(p, 35 * 8 + 9, 9, targetR);
  putBits(p, 38 * 8, 11, pwmL); putBits(p, 38 * 8 + 11, 11, pwmR);
  putI16(p, 41, speedL); putI16(p, 43, speedR);
  return p;
}
function barcodeCapture({ flags = 1 | 2 | 4 | 16 | 32, code = 25, reason = 0, bars = 14, count = 144 } = {}) {
  const p = new Uint8Array(27); p[0] = 0xc5;
  putU32(p, 1, 9000); putU32(p, 5, 22); putU32(p, 9, 8700); putU32(p, 13, 8950);
  putU32(p, 17, 3); putBits(p, 21 * 8, 16, 7);
  const meta = flags | (code << 6) | (reason << 11) | (bars << 15) | (count << 20);
  putU32(p, 23, meta);
  return p;
}
function networkStop({ flags = 7, link = 2, failMask = 0x69, valid = true, pollAgeMs = 410 } = {}) {
  const p = new Uint8Array(29); p[0] = 0xc6; p[1] = 0x21;
  putU32(p, 3, 9000); putU32(p, 7, pollAgeMs); putU32(p, 11, 30);
  putU32(p, 15, 5000); putU32(p, 19, 2); putU32(p, 23, 500);
  putBits(p, 27 * 8, 16, flags | ((link & 7) << 3) | (failMask << 6) | (valid ? 1 << 13 : 0));
  return p;
}

test('C4 decodes packed fault cause, wheel, frozen speeds/PWM and error-window counts', () => {
  const d = IS28.decodeFrame(frame(15, encoderFault())).data;
  assert.equal(d.kind, 'encoder-fault'); assert.equal(d.valid, true);
  assert.equal(d.version, '0.2.1'); assert.equal(d.fault, 'error burst'); assert.equal(d.wheel, 'left');
  assert.equal(d.saturated, true); assert.equal(d.controlTimerUs, 0x12345678); assert.equal(d.kernelMs, 9123);
  assert.equal(d.windowBad, 123); assert.equal(d.windowChanges, 456);
  assert.deepEqual([d.leftEdges, d.leftInvalid, d.rightEdges, d.rightInvalid], [10000, 9, 12000, 12]);
  assert.deepEqual([d.targetLeft, d.targetRight, d.pwmLeft, d.pwmRight, d.speedLeft, d.speedRight],
    [-45, 135, -333, 777, -12, 4.5]);
});

test('C4 direction snapshot requires its bounded duration; explicit all-zero body is the clear marker', () => {
  const d = IS28.decodeFrame(frame(15, encoderFault({ kind: 3, wheel: 2, duration: 131071 }))).data;
  assert.equal(d.fault, 'wrong direction'); assert.equal(d.wheel, 'right'); assert.equal(d.directionDurationUs, 131071);
  const clear = new Uint8Array(45); clear[0] = 0xc4; clear[1] = 0x21;
  assert.deepEqual(IS28.decodeFrame(frame(15, clear)).data, { kind: 'encoder-fault', version: '0.2.1', valid: false });
});

test('C4 rejects reserved bits, impossible enums/ranges and nonzero invalid snapshots', () => {
  const mutate = (offset, mask, value) => { const p = encoderFault(); p[offset] = (p[offset] & ~mask) | value; return IS28.decodeFrame(frame(15, p)); };
  assert.match(mutate(2, 0xc0, 0x40).error, /reserved flag/);
  assert.match(mutate(13, 0xfe, 0x80).error, /duration reserved/);
  assert.match(mutate(37, 0xfc, 0x04).error, /target reserved/);
  assert.match(mutate(40, 0xc0, 0x40).error, /PWM reserved/);
  const badKind = encoderFault(); badKind[2] = (badKind[2] & ~6) | 0x01;
  assert.match(IS28.decodeFrame(frame(15, badKind)).error, /kind/);
  const badWheel = encoderFault(); badWheel[2] = (badWheel[2] & ~0x18) | 0x18;
  assert.match(IS28.decodeFrame(frame(15, badWheel)).error, /wheel/);
  assert.match(IS28.decodeFrame(frame(15, encoderFault({ targetL: 181 }))).error, /target speed/);
  assert.match(IS28.decodeFrame(frame(15, encoderFault({ pwmR: 1001 }))).error, /PWM/);
  assert.match(IS28.decodeFrame(frame(15, encoderFault({ kind: 3, duration: 49999 }))).error, /below 50000/);
  const badClear = new Uint8Array(45); badClear[0] = 0xc4; badClear[1] = 0x21; badClear[11] = 1;
  assert.match(IS28.decodeFrame(frame(15, badClear)).error, /clear snapshot/);
  const badVersion = encoderFault(); badVersion[1] = 0x20;
  assert.match(IS28.decodeFrame(frame(15, badVersion)).error, /version/);
  assert.match(IS28.decodeFrame(frame(15, encoderFault().slice(0, 44))).error, /unknown source-15 layout/);
});

test('C4 decodes the firmware-generated latched and explicit-clear fixtures', () => {
  const dir = path.join(__dirname, '..', '..', 'testing', 'rtos_migration', 'build-host');
  const latched = IS28.decodeFrame(new Uint8Array(fs.readFileSync(path.join(dir, 'encoder_fault.bin'))));
  const cleared = IS28.decodeFrame(new Uint8Array(fs.readFileSync(path.join(dir, 'encoder_fault_clear.bin'))));
  assert.equal(latched.error, undefined); assert.equal(latched.data.fault, 'error density');
  assert.equal(latched.data.wheel, 'right'); assert.deepEqual([latched.data.targetLeft, latched.data.targetRight], [-45, 70]);
  assert.deepEqual([latched.data.speedLeft, latched.data.speedRight], [-123.4, 432.1]);
  assert.deepEqual(cleared.data, { kind: 'encoder-fault', version: '0.2.1', valid: false });
});

test('C5 preserves unknown code for a 14-bar group and decodes the frozen attempt metadata', () => {
  const d = IS28.decodeFrame(frame(15, barcodeCapture({ flags:1 | 16, code:0, bars:14, reason:10 }))).data;
  assert.equal(d.kind, 'barcode-capture'); assert.equal(d.uptimeMs, 9000);
  assert.equal(d.attemptSequence, 22); assert.equal(d.runId, 7);
  assert.equal(d.code, '?'); assert.equal(d.bars, 14); assert.equal(d.reasonName, 'in progress');
  assert.equal(d.decoded, false); assert.equal(d.valid, true); assert.equal(d.finalized, false);
  assert.equal(d.startValid, true); assert.equal(d.originalAccepted, false); assert.equal(d.rawCount, 144);
  assert.equal(d.rawOverwritten, 3); assert.equal(d.firstRawMs, 8700); assert.equal(d.lastRawMs, 8950);
});

test('C5 rejects out-of-range packed code, result, bar and raw-row values', () => {
  for (const [field, value, pattern] of [['code', 27, /code index/], ['reason', 11, /result/],
                                         ['bars', 25, /bar count/], ['count', 2049, /raw count/]]) {
    const p = barcodeCapture({ [field]: value });
    assert.match(IS28.decodeFrame(frame(15, p)).error, pattern, field);
  }
  assert.match(IS28.decodeFrame(frame(15, barcodeCapture().slice(0, 26))).error, /unknown source-15 layout/);
  assert.match(IS28.decodeFrame(frame(15, barcodeCapture({ flags:1|4,code:6 }))).error, /known code/);
  assert.match(IS28.decodeFrame(frame(15, barcodeCapture({ flags:1|2|4,code:0 }))).error, /known code/);
  assert.match(IS28.decodeFrame(frame(15, barcodeCapture({ flags:0,code:0 }))).error, /clear snapshot/);
});

test('C6 decodes a latched network Start-guard failure and explicit clear', () => {
  const d = IS28.decodeFrame(frame(15, networkStop())).data;
  assert.equal(d.kind, 'network-stop'); assert.equal(d.valid, true); assert.equal(d.version, '0.2.1');
  assert.deepEqual([d.stopMs,d.pollAgeMs,d.appAgeMs,d.ackAgeMs,d.connects,d.lastCompleteMs], [9000,410,30,5000,2,500]);
  assert.deepEqual([d.configured,d.connected,d.subscribed,d.link,d.failMask], [true,true,true,2,0x69]);
  assert.equal(IS28.decodeFrame(frame(15, networkStop({ pollAgeMs:0xffffffff }))).data.pollAgeMs, 0xffffffff,
    'no-poll sentinel is preserved in the decoded export data');
  const clear = new Uint8Array(29); clear[0] = 0xc6; clear[1] = 0x21;
  assert.deepEqual(IS28.decodeFrame(frame(15, clear)).data, { kind:'network-stop', version:'0.2.1', valid:false });
});

test('C6 rejects reserved bits/bytes, invalid link values, versions and malformed clears', () => {
  const p = networkStop(); p[2] = 1;
  assert.match(IS28.decodeFrame(frame(15, p)).error, /reserved byte/);
  const metaReserved = networkStop(); metaReserved[28] |= 0x40;
  assert.match(IS28.decodeFrame(frame(15, metaReserved)).error, /reserved metadata/);
  assert.match(IS28.decodeFrame(frame(15, networkStop({ link:-4 }))).error, /link code/);
  const version = networkStop(); version[1] = 0x20;
  assert.match(IS28.decodeFrame(frame(15, version)).error, /version/);
  const clear = new Uint8Array(29); clear[0] = 0xc6; clear[1] = 0x21; clear[3] = 1;
  assert.match(IS28.decodeFrame(frame(15, clear)).error, /clear snapshot/);
  assert.ok(IS28.decodeFrame(frame(15, networkStop().slice(0, 28))).error, 'a truncated C6 frame is not guessed as a valid layout');
});

test('C5/C6 decode real firmware snapshots without inferring a barcode from bars or losing age sentinels', () => {
  const dir = path.join(__dirname, '..', '..', 'testing', 'rtos_migration', 'build-host');
  const barcode = IS28.decodeFrame(new Uint8Array(fs.readFileSync(path.join(dir, 'barcode_attempt.bin'))));
  assert.equal(barcode.error, undefined); assert.equal(barcode.data.attemptSequence, 3);
  assert.equal(barcode.data.code, 'F', 'F comes from the actual current-attempt code field');
  assert.equal(barcode.data.bars, 14); assert.equal(barcode.data.reasonName, 'too few bars');
  assert.equal(barcode.data.rawCount, 2048); assert.equal(barcode.data.truncated, true);
  const stop = IS28.decodeFrame(new Uint8Array(fs.readFileSync(path.join(dir, 'network_stop.bin'))));
  assert.equal(stop.error, undefined); assert.equal(stop.data.link, -2); assert.equal(stop.data.failMask, 111);
  assert.equal(stop.data.pollAgeMs, 0xffffffff, 'no-poll sentinel is retained exactly');
});

test('v0.2.0 C2 release metadata and C3 runtime frames remain compatible', () => {
  const c2 = new Uint8Array(12); c2.set([0xc2, 0x20, 0x70, 1]);
  const rel = IS28.decodeFrame(frame(15, c2)).data;
  assert.equal(rel.kind, 'release'); assert.equal(rel.version, '0.2.0'); assert.equal(rel.storedLogs, true);
  const c3 = new Uint8Array(60); c3[0] = 0xc3;
  const runtime = IS28.decodeFrame(frame(15, c3)).data;
  assert.equal(runtime.kind, 'runtime'); assert.equal(runtime.uptimeMs, 0); assert.equal(runtime.controlFlags, 0);
});
