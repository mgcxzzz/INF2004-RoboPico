// Compat copy for the merged dashboard v0.1: original testing/integration/host/check_speed_dashboard.mjs (sha256 72313c7267ecae05), changed only where marked COMPAT (paths/mock route/screenshots).
/* INT-07: source-2 wire decoder and production panel handler host checks. */
'use strict';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const schema = require('../../is28_schema.js'); // COMPAT path
const html = fs.readFileSync(new URL('../../index.html', import.meta.url), 'utf8'); // COMPAT path

function frame(src, seq, payload) {
  return Uint8Array.from([0x20 | src, payload.length, seq, ...payload]);
}
function motionPayload({ flags = 0x0f, reserved = 0, left = 0, right = 0, centre = 0,
                         rollTenths = 0, height = 0, distance = 0, invalidLeft = 0,
                         invalidRight = 0 } = {}) {
  const p = new Uint8Array(20), v = new DataView(p.buffer);
  p[0] = flags; p[1] = reserved;
  v.setInt16(2, left, true); v.setInt16(4, right, true); v.setInt16(6, centre, true);
  v.setInt16(8, rollTenths, true); v.setInt16(10, height, true);
  v.setInt32(12, distance, true); v.setUint16(16, invalidLeft, true);
  v.setUint16(18, invalidRight, true);
  return p;
}

let assertions = 0;
const check = (condition, message) => { assert.ok(condition, message); assertions++; };
const decoded = schema.decodeFrame(frame(2, 44, motionPayload({ left: -1234, right: 2345,
  centre: 555, rollTenths: -127, height: -31, distance: -123456789,
  invalidLeft: 0x8123, invalidRight: 0xfedc })));
check(!decoded.error, 'valid motion frame');
assert.deepEqual({ ...decoded.data }, {
  freshOdometry: true, sideEstimateValid: true, oneSidedMode: true, passive: true,
  leftSpeedMmS: -1234, rightSpeedMmS: 2345, centreSpeedMmS: 555, rollDeg: -12.7,
  rightMinusLeftHeightMm: -31, netDistanceMm: -123456789,
  invalidTransitionsLeft: 0x8123, invalidTransitionsRight: 0xfedc
}); assertions += 13;
for (const p of [motionPayload({ flags: 0x10 }), motionPayload({ flags: 0x80 }),
                 motionPayload({ reserved: 1 })]) {
  check(!!schema.decodeFrame(frame(2, 1, p)).error, 'reserved flag/byte rejected');
}
check(!!schema.decodeFrame(frame(2, 1, new Uint8Array(19))).error, 'short payload rejected');
check(!!schema.decodeFrame(frame(2, 1, new Uint8Array(21))).error, 'oversize payload rejected');
check(!!schema.decodeFrame(frame(2, 1, motionPayload()).subarray(0, 22)).error, 'header size mismatch rejected');
const validZero = schema.decodeFrame(frame(2, 2, motionPayload({ flags: 0x03 })));
check(validZero.data.freshOdometry && validZero.data.centreSpeedMmS === 0 &&
      validZero.data.sideEstimateValid && validZero.data.rightMinusLeftHeightMm === 0,
      'valid zero values remain valid');
const speedOnly = schema.decodeFrame(frame(2, 3, motionPayload({ flags: 0x01, centre: -1, height: 82 })));
check(speedOnly.data.freshOdometry && !speedOnly.data.sideEstimateValid &&
      speedOnly.data.centreSpeedMmS === -1 && speedOnly.data.rightMinusLeftHeightMm === 82,
      'speed validity independent of height validity');
const heightOnly = schema.decodeFrame(frame(2, 4, motionPayload({ flags: 0x02, centre: 999, height: -9 })));
check(!heightOnly.data.freshOdometry && heightOnly.data.sideEstimateValid &&
      heightOnly.data.centreSpeedMmS === 999 && heightOnly.data.rightMinusLeftHeightMm === -9,
      'height validity independent of speed validity');

// Execute the dashboard's actual show/summary/timer code with a small DOM stub.
const nodes = new Map(), timers = []; let clock = 100;
class Element {
  constructor() { this.textContent = ''; this.innerHTML = ''; this.value = ''; this.dataset = {};
    this.childNodes = []; this.style = {}; this.checked = false; }
  set id(x) { this._id = x; nodes.set(x, this); } get id() { return this._id; }
  setAttribute() {} add() {} appendChild(x) { this.childNodes.push(x); return x; }
  append(...xs) { this.childNodes.push(...xs); } click() {} remove() {}
}
for (const m of html.matchAll(/id="([^"]+)"/g)) { const n = new Element(); n.id = m[1]; }
for (const m of html.matchAll(/id="([^"]+)"[^>]*>([^<]*)/g)) nodes.get(m[1]).textContent = m[2].trim();
const document = { getElementById: id => nodes.get(id), createElement: () => new Element(),
  querySelectorAll: q => q === 'button[data-cmd]' ? [] : q === '[data-map-active]' ?
    [...nodes.values()].filter(n => 'mapActive' in n.dataset) : [] };
const context = vm.createContext({ document, IS28: schema, location: { hostname: '127.0.0.1' },
  localStorage: { getItem: () => null }, performance: { now: () => clock },
  setInterval: fn => { timers.push(fn); return timers.length; }, clearInterval: () => {},
  setTimeout: () => 0, Option: function (text, value) { this.text = text; this.value = value; },
  console, Uint8Array, Date, URL, Blob: function () {} });
for (const m of html.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g)) {
  if (m[1].trim()) vm.runInContext(m[1], context);
}
const show = f => { context.__frame = f; vm.runInContext('show(__frame)', context); };
const get = id => nodes.get(id).textContent;
show(decoded);
check(get('imuSpeed') === '555 mm/s', 'centre speed shown');
check(get('motionWheels') === '-1234 / 2345 mm/s', 'left and right speeds shown');
check(get('motionDistance') === '-123456789 mm', 'signed distance shown');
check(get('motionRoll') === '-12.7°' && get('motionHeight') === '-31 mm', 'signed geometry shown');
check(get('motionQuality').includes('passive') && get('motionQuality').includes('one-sided mode'), 'quality shown');
check(get('s2').includes('555 mm/s') && get('s2').includes('-31 mm'), 'compact source summary');
show(speedOnly);
check(get('imuSpeed') === '-1 mm/s' && get('motionWheels') === '0 / 0 mm/s' &&
      get('motionHeight').includes('unavailable'), 'panel hides height when side estimate invalid');
show(heightOnly);
check(get('imuSpeed').includes('unavailable') && get('motionWheels').includes('unavailable') &&
      get('motionHeight') === '-9 mm', 'panel hides speeds when odometry invalid');

clock = 2201; for (const tick of timers) tick();
check(get('imuSpeed').includes('stale') && get('motionHeight').includes('stale'), 'motion fields stale and hidden');
show(heightOnly); // Duplicate delivery must not make an old sample look fresh.
check(get('motionQuality').includes('stale'), 'duplicate does not refresh motion age');
const validZeroFrame = schema.decodeFrame(frame(2, 5, motionPayload({ flags: 0x03 })));
show(validZeroFrame);
check(get('imuSpeed') === '0 mm/s' && get('motionHeight') === '0 mm', 'fresh valid zeros displayed');

const bootData = { netName: 'Operational', uptimeS: 50, state: 1, stateName: 'Running',
  published: 0, lost: 0, accepted: 0, rejected: 0, connects: 0, fillPct: 0 };
show({ src: 1, seq: 1, data: bootData });
bootData.uptimeS = 1; show({ src: 1, seq: 2, data: bootData });
check(get('motionQuality').includes('waiting for fresh'), 'powercycle clears motion display');
show(validZeroFrame); // Retained/repeated sequence from the previous boot is ignored.
check(get('motionQuality').includes('waiting for fresh'), 'old sequence ignored after restart');
const afterBoot = schema.decodeFrame(frame(2, 6, motionPayload({ flags: 0x03, centre: 25, height: 4 })));
show(afterBoot);
check(get('imuSpeed') === '25 mm/s' && get('motionHeight') === '4 mm', 'new source frame accepted after restart');
check(schema.SRC.MOTION === 2, 'motion source ID');

const extendedPayload = new Uint8Array(24);
extendedPayload.set(motionPayload());
extendedPayload.set([0x0c, 3, 9, 25], 20);
const diagnosticFrame = schema.decodeFrame(frame(2, 7, extendedPayload));
check(!diagnosticFrame.error && diagnosticFrame.data.qualityReasons === 0x0c &&
      diagnosticFrame.data.rampLevelReady && diagnosticFrame.data.rampCandidate &&
      !diagnosticFrame.data.rampRising && !diagnosticFrame.data.rampReturning &&
      diagnosticFrame.data.qualityBadSamples === 9 && diagnosticFrame.data.qualityTotalSamples === 25,
      'extended diagnostics decoded');
show(diagnosticFrame);
check(get('motionGuards').includes('acceleration') && get('motionGuards').includes('turn') &&
      get('motionGuards').includes('9/25'), 'recent exclusions and interval counts shown');
check(get('motionRampStage').includes('level reference armed') &&
      get('motionRampStage').includes('checking rise'), 'candidate state displayed');
extendedPayload[21]=0x80;
check(!!schema.decodeFrame(frame(2, 8, extendedPayload)).error, 'reserved diagnostic state rejected');
extendedPayload[21]=3;extendedPayload[22]=26;
check(!!schema.decodeFrame(frame(2, 8, extendedPayload)).error, 'bad diagnostic sample counts rejected');
clock=5000;for(const tick of timers)tick();
check(get('motionGuards').includes('stale') && get('motionRampStage').includes('stale'),
      'diagnostic fields become stale');
const numericPayload = new Uint8Array(32);
extendedPayload[22]=9;
numericPayload.set(extendedPayload);
const nv = new DataView(numericPayload.buffer);
nv.setUint16(24, 432, true); nv.setUint16(26, 65535, true);
nv.setUint16(28, 234, true); numericPayload[30]=24;
const numericFrame = schema.decodeFrame(frame(2, 9, numericPayload));
check(!numericFrame.error && numericFrame.data.maxSpreadMg===432 &&
      numericFrame.data.maxAbsAccelMmS2===65535 && numericFrame.data.maxAbsTurnDegS===23.4 &&
      numericFrame.data.rateSamples===24, 'numeric interval maxima decoded');
show(numericFrame);
check(get('motionGuardValues').includes('432 mg') && get('motionGuardValues').includes('65535 mm/s²') &&
      get('motionGuardValues').includes('23.4°/s'), 'numeric diagnostics displayed');
numericPayload[31]=1;
check(!!schema.decodeFrame(frame(2, 10, numericPayload)).error,'numeric reserved byte rejected');
numericPayload[31]=0;numericPayload[30]=26;
check(!!schema.decodeFrame(frame(2, 10, numericPayload)).error,'excess rate count rejected');
numericPayload[30]=0;
show(schema.decodeFrame(frame(2, 10, numericPayload)));
check(get('motionGuardValues').includes('rates unavailable'), 'missing fresh rates not shown as zero');
const robustPayload = new Uint8Array(36);
robustPayload.set(numericPayload);
robustPayload[32]=37;robustPayload[34]=7;
const robustFrame = schema.decodeFrame(frame(2, 11, robustPayload));
check(!robustFrame.error && robustFrame.data.maxMadMg===37 && robustFrame.data.minFifoSamples===7,
      'robust diagnostics decoded');
show(robustFrame);
check(get('motionGuardValues').includes('37 mg robust') && get('motionGuardValues').includes('min 7 FIFO samples'),
      'robust diagnostics displayed');
robustPayload[35]=1;
check(!!schema.decodeFrame(frame(2,12,robustPayload)).error,'robust reserved byte rejected');
robustPayload[35]=0;robustPayload[34]=33;
check(!!schema.decodeFrame(frame(2,12,robustPayload)).error,'impossible FIFO count rejected');
robustPayload[34]=7;
/* v0.6: 37-byte payload, per-axis MAD bytes fwd/lat/up at 31/35/36. */
const axisPayload = new Uint8Array(37);
axisPayload.set(robustPayload);
axisPayload[32]=67;axisPayload[33]=0;axisPayload[31]=17;axisPayload[35]=3;axisPayload[36]=10;
const axisFrame = schema.decodeFrame(frame(2, 13, axisPayload));
check(!axisFrame.error && axisFrame.len===37 && axisFrame.data.maxMadMg===67 &&
      axisFrame.data.maxMadFwdMg===68 && axisFrame.data.maxMadLatMg===12 &&
      axisFrame.data.maxMadUpMg===40, 'v0.6 per-axis MAD decoded as 4 mg upper bounds');
check(frame(2,13,axisPayload).length===40, 'v0.6 frame is exactly the 40-byte message limit');
show(axisFrame);
check(get('motionGuardValues').includes('fwd ≤68 / lat ≤12 / up ≤40'), 'v0.6 per-axis MAD displayed');
axisPayload[36]=18;
check(!!schema.decodeFrame(frame(2,14,axisPayload)).error,'per-axis byte above max MAD rejected');
axisPayload[36]=10;axisPayload[31]=16;
check(!!schema.decodeFrame(frame(2,14,axisPayload)).error,'per-axis maximum below max MAD rejected');
axisPayload[31]=17;
check(!schema.decodeFrame(frame(2,14,axisPayload)).error,'restored v0.6 frame accepted');
check(schema.decodeFrame(frame(2,15,robustPayload)).data.maxMadFwdMg===undefined,
      'v0.5 frame has no per-axis fields');
check(!!schema.decodeFrame(frame(2,16,new Uint8Array(38))).error,'38-byte payload rejected');
clock=8000;for(const tick of timers)tick();
check(get('motionGuardValues').includes('stale'), 'numeric diagnostics become stale');
console.log(`INT-07 PASS: ${assertions} protocol and production panel assertions.`);
