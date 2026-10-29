'use strict';
/* Merged dashboard v0.1 decoder checks (node --test test/test_merged_schema.js).
 * 1. Differential: the merged decoder must give exactly what the original
 *    family decoder gives (RTOS for 1-8, SDK v2.3 for 1, 8, 9-14), on
 *    random and boundary payloads.
 * 2. Units, signs, unavailable markers, variable-length/64-byte frames.
 * 3. Firmware-profile detection.
 * 4. Replay of real native captures (SDK v2.3 + Buddy4 add-on, RTOS v0.9).
 * No broker, browser or hardware is used. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const M = require('../is28_schema.js');
const RTOS = require('./reference/rtos_is28_schema.js');
const V23 = require('./reference/v23_is28_schema.js');

function frame(src, payload, seq = 19) {
  return Uint8Array.from([(src << 4) | 2, payload.length, seq & 255, ...payload]);
}
let seed = 0x1234567;
function rnd() { seed = (Math.imul(seed, 1103515245) + 12345) >>> 0; return seed >>> 8; }
function bytes(n) { return Array.from({ length: n }, () => rnd() & 255); }
function strip(f) { if (!f || f.error) return { error: f && f.error }; const { srcName, ...rest } = f; return rest; }
/* Only known difference: command code 11 is labelled 'GET_LOG' (SDK v2.3
 * name) instead of the RTOS placeholder 'code11'. Values are identical. */
function normaliseRtosLabel(o) {
  if (o && o.data && o.data.lastCmdName === 'code11') o.data = { ...o.data, lastCmdName: 'GET_LOG' };
  return o;
}
function sameAsReference(ref, b, label) {
  const want = strip(ref.decodeFrame(b));
  // Fine barcode candidate extends the SDK diagnostic bank bound from 62 to 63.
  // Compare original-domain rejection semantics without requiring the old bound text.
  if(ref===V23 && want.error==='barcode bank exceeds 62')want.error='barcode bank exceeds 63';
  assert.deepEqual(strip(M.decodeFrame(b)), ref === RTOS ? normaliseRtosLabel(want) : want, label);
}

test('differential: RTOS sources 1-8 decode exactly as the RTOS dashboard decoder', () => {
  let n = 0;
  for (const [src, lens] of [[1, [12]], [2, [20, 24, 32, 36, 37]], [3, [12]], [4, [16]], [5, [16]], [6, [6]], [7, [12]], [8, [5]]]) {
    for (const len of lens) {
      for (let i = 0; i < 400; i++) {
        const p = bytes(len);
        if (src === 2 && i % 2) {          /* half constrained to valid source-2 shapes */
          p[0] &= 0x0f; p[1] = 0;
          if (len >= 24) { p[21] &= 0x0f; p[22] = p[22] % 26; p[23] = Math.max(p[22], p[23] % 26); }
          if (len >= 32) { p[30] = p[30] % (p[23] + 1); if (len < 37) p[31] = 0; }
          if (len >= 36) { p[34] = p[34] % 33; if (len === 36) p[35] = 0; }
          if (len === 37) { const mad = p[32] | (p[33] << 8); const top = Math.min(255, Math.ceil((mad & 0xffff) / 4));
                            p[31] = top; p[35] = rnd() % (top + 1); p[36] = rnd() % (top + 1); }
        }
        if (src === 8 && i % 2) { const w = (rnd() % 26) | ((rnd() % 26) << 5) | ((rnd() % 26) << 10) | ((rnd() % 26) << 15) | (1 << 20) | ((rnd() & 255) << 21);
                                  p[0] = w & 255; p[1] = (w >>> 8) & 255; p[2] = (w >>> 16) & 255; p[3] = (w >>> 24) & 255; }
        sameAsReference(RTOS, frame(src, p, i), `src ${src} len ${len} case ${i}`); n++;
      }
      sameAsReference(RTOS, frame(src, bytes(len + 1)), `src ${src} oversize`);
    }
  }
  assert.ok(n > 4000);
});

test('differential: SDK sources 1, 8, 9-14 decode exactly as the v2.3 dashboard decoder', () => {
  for (const [src, len] of [[1, 12], [8, 5], [9, 45], [10, 21], [11, 23], [12, 22], [14, 60]]) {
    for (let i = 0; i < 400; i++) {
      const p = bytes(len);
      if(src===14 && p[6]===63)p[6]=64; // new valid bank covered separately
      if (src === 14 && i % 2) { p[5] &= 0x0f; p[6] %= 63; p[7] &= 0x0f; }
      sameAsReference(V23, frame(src, p, i), `src ${src} case ${i}`);
    }
    sameAsReference(V23, frame(src, bytes(len - 1)), `src ${src} short`);
  }
  for (let i = 0; i < 400; i++) {                       /* source 13, every data length 0..49 */
    const dataLen = i % 50, p = [rnd() & 3, rnd() & 255, rnd() & 255, rnd() & 15, 4, 0, 0, 1, 0, dataLen, 0, 0, ...bytes(dataLen)];
    if (i % 7 === 0) p[10] = 1;                          /* reserved byte error path */
    sameAsReference(V23, frame(13, p, i), `src 13 case ${i}`);
  }
});

test('SDK frames reach 64 bytes and are not truncated; RTOS 40-byte frame accepted', () => {
  const p = [2, 9, 5, 0, 10, 0, 1, 2, 0, 49, 0, 0, ...bytes(49)];
  const big = frame(13, p);
  assert.equal(big.length, 64);
  const f = M.decodeFrame(big);
  assert.equal(f.error, undefined); assert.equal(f.data.bytes.length, 49); assert.equal(f.len, 61);
  const tooBig = frame(13, [...p.slice(0, 9), 50, 0, 0, ...bytes(50)]);
  assert.match(M.decodeFrame(tooBig).error, /length mismatch/);
  const diag = frame(14, new Array(60).fill(0));
  assert.equal(diag.length, 63); assert.equal(M.decodeFrame(diag).error, undefined);
  const b2 = new Array(37).fill(0);
  assert.equal(frame(2, b2).length, 40); assert.equal(M.decodeFrame(frame(2, b2)).error, undefined);
  assert.equal(M.MAX_FRAME.sdk, 64); assert.equal(M.MAX_FRAME.rtos, 40);
});

test('units, signs and unavailable markers are preserved', () => {
  const p = new Array(45).fill(0);
  const v = new DataView(Uint8Array.from(p).buffer);
  const b = new Uint8Array(45); const dv = new DataView(b.buffer);
  dv.setUint32(0, 123456, true); dv.setUint32(4, 9876, true); dv.setUint16(8, 7, true);
  b[12] = 0x80 | 0x01; dv.setInt16(14, -1234, true); dv.setInt16(16, 250, true); dv.setInt16(18, -45, true);
  dv.setInt16(22, -300, true); dv.setUint16(32, 0xffff, true); dv.setUint16(34, 1000, true); dv.setUint16(36, 0xffff, true);
  dv.setUint32(39, 4000000000, true); dv.setInt16(43, -17999, true);
  const d = M.decodeFrame(frame(9, Array.from(b))).data;
  assert.equal(d.speedL, -123.4); assert.equal(d.speedR, 25); assert.equal(d.targetL, -45); assert.equal(d.pwmL, -300);
  assert.deepEqual(d.levels, [65535, 1000, 65535]); assert.equal(d.calibrationValid, false); assert.equal(d.runRelativeUptime, true);
  assert.equal(d.distanceMm, 4000000000); assert.equal(d.headingCdeg, -17999);
  assert.equal(M.picoUptimeMs({ src: 9, data: d }), 123456); assert.equal(M.frameRunId({ src: 9, data: d }), 7);
  void v;
  const q = new Uint8Array(20); const qv = new DataView(q.buffer);
  q[0] = 0x03; qv.setInt16(2, -40, true); qv.setInt16(10, -9, true); qv.setInt32(12, -123456, true); qv.setInt16(8, -35, true);
  const m = M.decodeFrame(frame(2, Array.from(q))).data;
  assert.equal(m.leftSpeedMmS, -40); assert.equal(m.rightMinusLeftHeightMm, -9); assert.equal(m.netDistanceMm, -123456); assert.equal(m.rollDeg, -3.5);
  assert.equal(M.picoUptimeMs({ src: 2, data: m }), null);
  const hb = M.decodeFrame(frame(1, [2 | (1 << 2) | (3600 << 12) & 255, ((3600 << 12) >>> 8) & 255, ((3600 << 12) >>> 16) & 255, ((3600 << 12) >>> 24) & 255, 0, 0, 0, 0, 0, 0, 0, 0]));
  assert.equal(hb.data.uptimeS, 3600); assert.equal(M.picoUptimeMs(hb), 3600000);
  const w0 = 3 | (1 << 13) | ((-87 & 0x7ff) << 15);
  const imu = M.decodeFrame(frame(4, [w0 & 255, (w0 >>> 8) & 255, (w0 >>> 16) & 255, (w0 >>> 24) & 255, 10, 12, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])).data;
  assert.equal(imu.pitchDeg, -8.7); assert.equal(imu.heightMm, 10); assert.equal(imu.highestMm, 12); assert.equal(imu.humps, 1);
});

test('profile detection: observed packet types, inference and manual override', () => {
  assert.equal(M.detectProfile([]), 'unknown');
  assert.equal(M.detectProfile([1, 8]), 'unknown');
  assert.equal(M.detectProfile([1, 2, 4, 8]), 'unknown');
  assert.equal(M.detectProfile([1, 9, 10]), 'sdk');
  assert.equal(M.detectProfile([1, 2, 4, 11, 14]), 'sdk');
  assert.equal(M.detectProfile([6]), 'rtos');
  assert.equal(M.detectProfile([7, 1]), 'rtos');
  assert.equal(M.detectProfile([6, 9]), 'mixed');
  assert.equal(M.detectProfile([1, 2, 4], new Map([[2, 1000], [4, 1100]]), 3999), 'unknown');
  assert.equal(M.detectProfile([1, 2, 4], new Map([[2, 1000], [4, 1100]]), 4000), 'rtos-inferred');
  assert.equal(M.detectProfile([2, 9], new Map([[2, 0]]), 10000), 'sdk');
  assert.equal(M.effectiveProfile('rtos-inferred', 'auto'), 'rtos');
  assert.equal(M.effectiveProfile('sdk', 'rtos'), 'rtos');
  assert.equal(M.effectiveProfile('unknown', 'sdk'), 'sdk');
  assert.equal(M.effectiveProfile('mixed', 'auto'), 'mixed');
  for (const s of [3, 5, 6, 7]) assert.equal(M.SRC_FAMILY[s], 'rtos');
  for (const s of [9, 10, 11, 12, 13, 14]) assert.equal(M.SRC_FAMILY[s], 'sdk');
  for (const s of [1, 2, 4, 8]) assert.equal(M.SRC_FAMILY[s], 'shared');
});

test('existing name contracts kept for both dashboards', () => {
  assert.equal(M.SRC.MOTION, 2); assert.equal(M.SRC.BARCODE_MAP, 8); assert.equal(M.SRC.LOG_CHUNK, 13);
  assert.equal(M.SRC.SDK_MOTION, 9); assert.equal(M.CMD.GET_LOG, 11); assert.equal(M.CMD_NAME[11], 'GET_LOG');
  assert.deepEqual(M.LOG_DATASETS.map((d) => d.rowBytes), [48, 58, 10, 31]);
  assert.deepEqual(M.encodeCommand(M.CMD.GET_LOG, 9, M.getLogArg(3, 4095)), Uint8Array.from([0xb2, 9, 0xff, 0x3f]));
  assert.deepEqual(M.encodeCommand(M.CMD.CONFIG, 77, M.configArg(M.CFG_TELEMETRY, 1)), RTOS.encodeCommand(RTOS.CMD.CONFIG, 77, RTOS.configArg(RTOS.CFG_TELEMETRY, 1)));
  assert.deepEqual(M.encodeMapping(5, ['Z', 'A', 'M', 'Q']), V23.encodeMapping(5, ['Z', 'A', 'M', 'Q']));
  assert.deepEqual(M.decodeAck([0x52, 4, 0, 6]), V23.decodeAck([0x52, 4, 0, 6]));
});

function replay(file) {
  const lines = fs.readFileSync(path.join(__dirname, 'fixtures', file), 'utf8').trim().split('\n');
  const out = { total: lines.length, decoded: 0, errors: [], perSrc: {}, topics: {}, seen: new Set(), firstSeen: new Map(), lastMs: 0, mismatches: [] };
  for (const line of lines) {
    const [iso, topic, hexs] = line.split(' ');
    out.topics[topic] = (out.topics[topic] || 0) + 1;
    if (topic !== 'is28/tlm' && topic !== 'is28/hb') continue;
    const b = Uint8Array.from(Buffer.from(hexs || '', 'hex'));
    const f = M.decodeFrame(b);
    if (f.error) { out.errors.push(`${iso} ${f.error}`); continue; }
    out.decoded++;
    out.perSrc[f.src] = (out.perSrc[f.src] || 0) + 1;
    const ms = Date.parse(iso.replace(/(\.\d{3})\d+/, '$1').replace(/([+-]\d\d)(\d\d)$/, '$1:$2'));
    out.seen.add(f.src); if (!out.firstSeen.has(f.src)) out.firstSeen.set(f.src, ms); out.lastMs = ms;
    const ref = M.SRC_FAMILY[f.src] === 'sdk' || f.src === 1 || f.src === 8 ? V23 : RTOS;
    const refOut = ref === RTOS ? normaliseRtosLabel(strip(ref.decodeFrame(b))) : strip(ref.decodeFrame(b)), mine = strip(f);
    if ((f.src === 2 || f.src === 4) || M.SRC_FAMILY[f.src] === 'sdk' || f.src === 1 || f.src === 8) {
      if (JSON.stringify(refOut) !== JSON.stringify(mine)) out.mismatches.push(`${iso} src ${f.src}`);
    }
  }
  return out;
}

test('real capture: SDK v2.3 + Buddy4 add-on segment decodes completely and is detected as SDK', () => {
  const r = replay('sdk_v23_b4_segment.txt');
  assert.deepEqual(r.errors, []); assert.deepEqual(r.mismatches, []);
  for (const s of [1, 2, 4, 8, 9, 10, 11, 12, 14]) assert.ok(r.perSrc[s] > 0, `source ${s} present`);
  assert.equal(M.detectProfile(r.seen, r.firstSeen, r.lastMs), 'sdk');
  assert.ok(r.topics['is28/cmd'] > 0 && r.topics['is28/ack'] > 0);
  console.log('SDK segment', JSON.stringify({ total: r.total, decoded: r.decoded, perSrc: r.perSrc, topics: r.topics }));
});

test('real capture: RTOS Buddy4 v0.9 segment decodes completely and is inferred as RTOS', () => {
  const r = replay('rtos_v0_9_head1500.txt');
  assert.deepEqual(r.errors, []); assert.deepEqual(r.mismatches, []);
  assert.deepEqual(Object.keys(r.perSrc).map(Number).sort((a, b) => a - b), [1, 2, 4, 8]);
  assert.equal(M.detectProfile(r.seen), 'unknown', 'shared sources alone are not proof');
  assert.equal(M.detectProfile(r.seen, r.firstSeen, r.lastMs), 'rtos-inferred');
  console.log('RTOS segment', JSON.stringify({ total: r.total, decoded: r.decoded, perSrc: r.perSrc }));
});

test('real capture: last-will segment contains broker offline messages', () => {
  const r = replay('sdk_v23_b4_lwt_segment.txt');
  assert.deepEqual(r.errors, []);
  assert.equal(r.topics['is28/lwt'], 2);
});

// Explicit extension; frozen reference decoder remains untouched.
test('fine barcode bank63 is valid, bank64 and reserved flags are rejected',()=>{
  const p=new Array(60).fill(0);p[4]=204;p[6]=63;
  assert.equal(M.decodeFrame(frame(14,p)).data.bank,63);
  p[6]=64;assert.match(M.decodeFrame(frame(14,p)).error,/bank/);
  p[6]=63;p[7]=0x80;assert.match(M.decodeFrame(frame(14,p)).error,/reserved/);
});
