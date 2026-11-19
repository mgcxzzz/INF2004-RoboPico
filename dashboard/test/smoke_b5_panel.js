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
  if (l.startsWith('SCENARIO ')) { cur = l.slice(9); continue; }
  if (!l.startsWith('TLM ')) continue;
  const f = ctx.IS28.decodeFrame(Uint8Array.from(Buffer.from(l.slice(4), 'hex')));
  if (f.data && f.data.kind === 'b5-status') { ctx.__rx(f.data, f); out[cur] = [el('b5WidthPolicy').textContent, el('b5Width').textContent, el('b5Steer').textContent]; }
}
for (const k of ['185 offset RIGHT 60 -> LEFT', 'REF rejected, then clean sonar: HOLD', 'start 250 mm: clean skip', 'approach push 3.0deg -2mm steered'])
  console.log(k, '\n   ', out[k].join('\n    '));
const [mp, mw] = out['185 offset RIGHT 60 -> LEFT'];
assert.match(mp, /^measured/); assert.doesNotMatch(mw, /ASSUMED/);
const [fp, fw] = out['REF rejected, then clean sonar: HOLD'];
assert.match(fp, /^failed/); assert.match(fw, /^150\.00 mm — ASSUMED.*FAILED: mission HELD/);
const [sp, sw] = out['start 250 mm: clean skip'];
assert.match(sp, /^skipped/); assert.match(sw, /ASSUMED.*deliberately skipped/); assert.doesNotMatch(sw, /FAILED/);
assert.match(out['approach push 3.0deg -2mm steered'][2], /largest 8\.0° of ±8\.0° · at bound/);
console.log('PASS: Buddy5 panel smoke');
