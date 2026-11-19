'use strict';
/* usage: node dashboard/test/smoke_b5_panel.js dashboard
 * Headless smoke (no browser, no broker): run index.html's inline script against a permissive DOM stub,
   feed the C-packed fixture frames, and check the Buddy5 panel text. */
const assert = require('node:assert/strict');
const fs = require('fs'), vm = require('vm'), path = require('path');
const dir = path.resolve(process.argv[2] || path.join(__dirname, '..'));
const any = new Proxy(function () {}, { get: (t, k) => k === Symbol.toPrimitive ? () => '' : any, apply: () => any, construct: () => any });
const els = new Map();
function el(id) {
  if (!els.has(id)) {
    const e = new Proxy({ id, textContent: '', className: '', style: {}, classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } }, firstElementChild: null, addEventListener() {}, appendChild() {}, prepend() {}, replaceChildren() {}, insertBefore() {}, setAttribute() {},
      dataset: {}, children: [], value: '', checked: false },
      { get(t, k) { if (k in t) return t[k]; return typeof k === 'string' ? any : undefined; },
        set(t, k, v) { t[k] = v; return true; } });
    els.set(id, e);
  }
  return els.get(id);
}
const document = new Proxy({ getElementById: el, querySelector: (q) => el(q), querySelectorAll: () => [], createElement: (t) => el('new-' + t + Math.random()),
  addEventListener() {}, body: el('body'), documentElement: el('html') }, { get(t, k) { return k in t ? t[k] : (() => el('doc.' + String(k))); } });
const ctx = { addEventListener() {}, removeEventListener() {}, RoboPicoCalibration: any, mqtt: any, Option: function () { return el('opt' + Math.random()); }, Event: function () {}, CustomEvent: function () {}, WebSocket: function () {}, getComputedStyle: () => new Proxy({}, { get: () => '' }), matchMedia: () => ({ matches: false, addEventListener() {} }), Chart: function () { return new Proxy({}, { get: () => () => {} }); }, document, window: {}, console, setInterval() {}, setTimeout() {}, clearTimeout() {}, clearInterval() {}, performance: { now: () => Date.now() },
  localStorage: { getItem() { return null; }, setItem() {} }, navigator: {}, location: { search: '', hash: '' }, URLSearchParams, TextDecoder, TextEncoder,
  requestAnimationFrame() {}, fetch: () => Promise.reject(new Error('no net')), Uint8Array, Date, Math, JSON, Map, Set, Number, String, Array, Object, Promise };
ctx.window = ctx; ctx.self = ctx; ctx.globalThis = ctx;
ctx.IS28 = require(path.join(dir, 'is28_schema.js'));
vm.createContext(ctx);
const html = fs.readFileSync(path.join(dir, 'index.html'), 'utf8');
const code = [...html.matchAll(/<script(?![^>]*src)[^>]*>([\s\S]*?)<\/script>/g)].map((m) => m[1]).join('\n');
try { vm.runInContext('globalThis.__rx = receiveB5;\n' + code, ctx); } catch (e) { throw e; }
const lines = fs.readFileSync(path.join(dir, 'test/fixtures/b5_rtos_mission_c_frames.txt'), 'utf8').split('\n');
let cur = null; const out = {};
for (const l of lines) {
  if (l.startsWith('SCENARIO ')) {   /* each simulated mission restarts at width run 1: fresh panel state */
    cur = l.slice(9); vm.runInContext('b5.tracker = IS28.B5Tracker(); b5.events = []; b5.status = null; b5.lastStage = null;', ctx); continue;
  }
  if (!l.startsWith('TLM ')) continue;
  const f = ctx.IS28.decodeFrame(Uint8Array.from(Buffer.from(l.slice(4), 'hex')));
  if (!f.data || !f.data.kind || !f.data.kind.startsWith('b5-')) continue;
  ctx.__rx(f.data, f);
  if (f.data.kind === 'b5-status')
    out[cur] = ['b5WidthPolicy', 'b5Width', 'b5Steer', 'b5WidthRaw', 'b5WidthProbe', 'b5WidthLimits', 'b5WidthVerify']
      .map((id) => el(id).textContent);
}
for (const k of ['185 offset RIGHT 60 -> LEFT', '100 offset RIGHT 35 -> LEFT', 'sonar faults in direction probe',
                 '185 centred -> default (labelled)', 'REF rejected, then clean sonar: HOLD', 'start 250 mm: clean skip',
                 'approach push 3.0deg -2mm steered'])
  console.log(k, '\n   ', out[k].join('\n    '));
/* long probe, 185 box: right side clearance-limited before its edge -> labelled assumption */
const [mp, mw, , mraw, , mlim, mver] = out['185 offset RIGHT 60 -> LEFT'];
assert.match(mp, /^measured/); assert.match(mw, /^150\.00 mm — ASSUMED project midpoint/);
assert.match(mraw, /^n\/a \/ n\/a \/ 150\.00 mm/);
assert.match(mlim, /^run 1: L FAR .* \| R CLEARANCE .* ceiling 220 mm, margin 50 mm$/);
assert.match(mver, /^run 1: Pico-reported · capture complete \(replay offline to verify\) · long probe/);
const [, ew, , eraw] = out['100 offset RIGHT 35 -> LEFT'];
assert.match(ew, /^156\.15 mm — Pico estimate/); assert.match(eraw, /^166\.15 \/ 156\.15 \/ 156\.15 mm/);
assert.doesNotMatch(eraw, /LIMITED/);
const [, cw, , craw] = out['sonar faults in direction probe'];
assert.match(cw, /^200\.00 mm — Pico estimate/); assert.match(craw, /^246\.25 \/ 236\.25 \/ 200\.00 mm.*LIMITED/);
assert.match(out['185 centred -> default (labelled)'][6], /NOT independently verifiable/);
const [fp, fw] = out['REF rejected, then clean sonar: HOLD'];
assert.match(fp, /^failed/); assert.match(fw, /^150\.00 mm — ASSUMED.*FAILED: mission HELD/);
assert.match(out['REF rejected, then clean sonar: HOLD'][4], /^HOLD · last sample REF/);
const [sp, sw] = out['start 250 mm: clean skip'];
assert.match(sp, /^skipped/); assert.match(sw, /ASSUMED.*deliberately skipped/); assert.doesNotMatch(sw, /FAILED/);
assert.match(out['start 250 mm: clean skip'][5], /^no width probe in this mission/);
assert.match(out['start 250 mm: clean skip'][6], /^no width probe in this mission/);
assert.match(out['approach push 3.0deg -2mm steered'][2], /largest 8\.0° of ±8\.0° · at bound/);
console.log('PASS: Buddy5 panel smoke');
