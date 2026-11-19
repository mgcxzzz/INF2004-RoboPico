'use strict';
/* Buddy5 rtos_buddy1_5 telemetry (node --test test/test_b5_rtos.js).
 * Fixture b5_rtos_mission_c_frames.txt is written by the REAL C packers driven
 * by the host mission simulation (testing/rtos_migration/test_b5_mission.c):
 * E5 status v3, long-probe width D1/D2 v10, D3 v3, D4 v3, D5 v2, BE/BF
 * direction (v3, 25 mm) and B5 bypass events (v0.2.12 long-width candidate).
 * b5_rtos_mission_c_frames_v0_2_legacy.txt is the unchanged v0.2 candidate
 * fixture (E5 v2, short-probe D1/D2 v4, D3 v1): older captures must stay
 * readable under their original rules.
 * In the "185 centred" scenario D1 index 5 is deliberately never written, to
 * check that the dashboard reports the hole instead of filling it.
 * Simulated data only: no broker, Pico or car. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const M = require('../is28_schema.js');
function loadFixture(file) {
  const out = new Map();
  let current = null;
  for (const l of fs.readFileSync(path.join(__dirname, 'fixtures', file), 'utf8').split('\n')) {
    if (l.startsWith('SCENARIO ')) { current = l.slice(9); out.set(current, []); }
    else if (l.startsWith('TLM ') && current) out.get(current).push(Uint8Array.from(Buffer.from(l.slice(4), 'hex')));
  }
  return out;
}
const scenarios = loadFixture('b5_rtos_mission_c_frames.txt');
const legacy = loadFixture('b5_rtos_mission_c_frames_v0_2_legacy.txt');
const decodeAll = (name, set = scenarios) => set.get(name).map((b) => M.decodeFrame(b));
const of = (d, kind) => d.filter((f) => f.data.kind === kind).map((f) => f.data);

test('every C-packed Buddy5 frame decodes without error, with exact layouts', () => {
  let n = 0;
  for (const [name, frames] of scenarios) for (const b of frames) {
    const f = M.decodeFrame(b); n++;
    assert.ok(!f.error, `${name}: ${f.error}`);
    assert.equal(f.src, 15);
    assert.ok(b.length <= 64, 'WIFI_PACKET_MAX');
    assert.ok(f.data && f.data.kind.startsWith('b5-'), f.data && f.data.kind);
  }
  assert.ok(n > 1000);
});

test('evidence bypass: direction LEFT from probe evidence, width with basis, complete records', () => {
  const d = decodeAll('185 offset RIGHT 60 -> LEFT');
  const bf = d.filter((f) => f.data.kind === 'b5-direction-summary').map((f) => f.data);
  assert.equal(bf.length, 1);
  assert.equal(bf[0].version, 3); assert.equal(bf[0].nearMm, 25);
  assert.equal(bf[0].decisionName, 'LEFT'); assert.equal(bf[0].endName, 'COMPLETE');
  const st = d.filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  const last = st[st.length - 1];
  assert.equal(last.sideName, 'left'); assert.equal(last.basis, 1); assert.ok(!last.defaultSide);
  assert.ok(st.some((s) => s.stageName === 'width probe') && st.some((s) => s.stageName === 'bypass'));
  const t = M.B5Tracker(); d.forEach((f) => t.add(f.data));
  const w = t.width(), e = t.direction();
  assert.ok(w.complete, JSON.stringify(w)); assert.deepEqual(w.missing, []);
  assert.ok(e.complete); assert.equal(e.claimed, 8);
  const res = t.widthRun().result;
  assert.ok(res.displayMm >= 100 && res.displayMm <= 200);
  assert.ok(['CALIBRATED_RANGE_RISE', 'EMPIRICAL_REACH_COMPLETION', 'PROJECT_RANGE_ASSUMPTION',
             'CORROBORATED_EDGE_ESTIMATE'].includes(res.basisName));
  assert.equal(res.protocol, 'long'); assert.equal(res.version, 3); assert.equal(res.ruleId, 2);
  assert.equal(w.protocol, 'long'); assert.ok(w.limits);
  assert.match(w.verification, /Pico-reported · capture complete/);
  const ev = d.filter((f) => f.data.kind === 'b5-event').map((f) => f.data.codeName);
  assert.ok(ev.includes('PLAN') && ev.includes('ACQUIRED') && ev.includes('HANDBACK'), ev.join(','));
});

test('healthy no preference uses the DEFAULT side and says so; dropped D1 is reported, not filled', () => {
  const d = decodeAll('185 centred -> default (labelled)');
  const st = d.filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  const last = st[st.length - 1];
  assert.equal(last.basis, 2); assert.ok(last.defaultSide); assert.equal(last.sideName, 'right');
  assert.match(last.basisName, /DEFAULT/);
  const t = M.B5Tracker(); d.forEach((f) => t.add(f.data));
  const w = t.width();
  assert.ok(!w.complete); assert.deepEqual(w.missing, [5]);
  assert.equal(w.received, w.claimed - 1);
  assert.match(w.verification, /NOT independently verifiable/);
});

test('failed direction probe is HELD, never a side; width assumption is labelled', () => {
  const d = decodeAll('sonar faults in direction probe');
  const st = d.filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  const last = st[st.length - 1];
  assert.equal(last.stageName, 'hold'); assert.equal(last.basis, 3); assert.equal(last.sideName, 'none');
  assert.equal(last.reasonName, 'direction probe aborted');
  });

test('v0.2: attempted width probe whose REF is rejected is HELD; its 150 mm assumption stays visible', () => {
  const d = decodeAll('REF rejected, then clean sonar: HOLD');
  const r = d.filter((f) => f.data.kind === 'b5-width-result').map((f) => f.data);
  assert.equal(r.length, 1); assert.ok(r[0].assumed); assert.equal(r[0].displayMm, 150);
  assert.equal(r[0].basisName, 'PROJECT_RANGE_ASSUMPTION'); assert.equal(r[0].endName, 'REFERENCE');
  const st = d.filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  const last = st[st.length - 1];
  assert.equal(last.version, 3);
  assert.equal(last.stageName, 'hold'); assert.equal(last.reasonName, 'width probe aborted');
  assert.equal(last.widthPolicy, 3); assert.ok(last.widthFailed && !last.widthSkipped);
  assert.equal(last.widthDisplayMm, 150); assert.equal(last.widthBasis, 3);
  assert.ok(last.widthAttempted && !last.directionAttempted);
  assert.ok(last.widthGateMm >= 270 && last.widthGateMm <= 330);
  assert.ok(!st.some((s) => s.stageName === 'approach' || s.stageName === 'direction probe'));
});

test('v0.2: clean out-of-window stop is a labelled SKIP (no probe), distinct from a failure', () => {
  const d = decodeAll('start 250 mm: clean skip');
  assert.equal(d.filter((f) => f.data.kind.startsWith('b5-width')).length, 0, 'no D1-D4: probe never ran');
  const st = d.filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  const skip = st.filter((s) => s.widthPolicy === 2);
  assert.ok(skip.length > 0);
  for (const s of skip) {
    assert.ok(s.widthSkipped && !s.widthFailed && !s.widthAttempted);
    assert.equal(s.widthBasisName, 'PROJECT_RANGE_ASSUMPTION'); assert.equal(s.widthStatusName, 'INVALID');
    assert.equal(s.widthDisplayMm, 150); assert.ok(s.widthGateMm < 270 || s.widthGateMm > 330);
    assert.match(s.widthPolicyName, /skipped/);
  }
  assert.ok(st.some((s) => s.stageName === 'bypass'));
  for (const name of ['faults in width-policy range', 'noisy width-policy range']) {
    const h = decodeAll(name).filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
    const l = h[h.length - 1];
    assert.equal(l.stageName, 'hold'); assert.equal(l.reasonName, 'fresh range unusable');
    assert.equal(l.widthPolicy, 0, name + ': a faulty/noisy range is never a skip');
  }
});

test('v0.2: approach steering setpoint is reported and bounded to +/-8 deg', () => {
  let seen = 0;
  for (const [name] of scenarios) for (const f of decodeAll(name)) {
    if (f.data.kind !== 'b5-status') continue;
    assert.equal(f.data.version, 3);
    assert.ok(Math.abs(f.data.steerDeg) <= 8 && f.data.steerMaxDeg <= 8, name);
    if (f.data.steerMaxDeg > 0) seen++;
  }
  assert.ok(seen > 0);
  const on = decodeAll('approach push 3.0deg -2mm steered').filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  const off = decodeAll('approach push 3.0deg -2mm straight').filter((f) => f.data.kind === 'b5-status').map((f) => f.data);
  assert.ok(on[on.length - 1].steerMaxDeg > 0 && on[on.length - 1].steerSaturated > 0);
  assert.equal(off[off.length - 1].steerMaxDeg, 0);
});

test('existing source-15 layouts keep their meaning (dispatch unaffected)', () => {
  const ramp = new Uint8Array(3 + 28); ramp[0] = 0xF2; ramp[1] = 28;
  assert.equal(M.decodeFrame(ramp).data.kind, 'ramp');
  const unknown = new Uint8Array(3 + 54); unknown[0] = 0xF2; unknown[1] = 54; unknown[3 + 4] = 0xE6;
  assert.ok(M.decodeFrame(unknown).error);
  const badVersion = new Uint8Array(3 + 54); badVersion[0] = 0xF2; badVersion[1] = 54; badVersion[3 + 4] = 0xE5; badVersion[3 + 5] = 2;
  assert.match(M.decodeFrame(badVersion).error, /version/);
  const v1In59 = new Uint8Array(3 + 59); v1In59[0] = 0xF2; v1In59[1] = 59; v1In59[3 + 4] = 0xE5; v1In59[3 + 5] = 1;
  assert.match(M.decodeFrame(v1In59).error, /version/);
  /* a v0.1 (54-byte, version 1) status still decodes, without policy/steering */
  const v1 = new Uint8Array(3 + 54); v1[0] = 0xF2; v1[1] = 54; v1[3 + 4] = 0xE5; v1[3 + 5] = 1;
  const d1 = M.decodeFrame(v1).data;
  assert.equal(d1.version, 1); assert.equal(d1.widthPolicy, null); assert.equal(d1.steerDeg, null);
  const bad = new Uint8Array(3 + 59); bad[0] = 0xF2; bad[1] = 59; bad[3 + 4] = 0xE5; bad[3 + 5] = 2; bad[3 + 19] = 9;
  assert.match(M.decodeFrame(bad).error, /range/);
});

test('long probe: D1/D2 v10, D3 v3 rule 2, D4 v3 and one D5 v2 per run with the tested constants', () => {
  let runs = 0;
  for (const [name] of scenarios) {
    const d = decodeAll(name);
    for (const r of of(d, 'b5-width-record')) { assert.equal(r.version, 10); assert.equal(r.protocol, 'long'); }
    for (const r of of(d, 'b5-width-retry')) assert.equal(r.version, 3);
    const sums = of(d, 'b5-width-summary'), lims = of(d, 'b5-width-limits'), res = of(d, 'b5-width-result');
    assert.equal(lims.length, sums.length, name + ': one D5 per D2');
    for (const s of sums) {
      assert.equal(s.version, 10); assert.ok(s.records <= 61, name + ' records ' + s.records);
      const l = lims.find((x) => x.run === s.run);
      assert.ok(l, name + ': D5 for run ' + s.run);
      assert.equal(l.ceilingMm, 220); assert.equal(l.overshootMm, 5); assert.equal(l.headingReserveDeg, 5);
      assert.equal(l.marginMm, 50); assert.equal(l.d0, s.d0);
      assert.equal(l.left.steps, s.stepsL); assert.equal(l.right.steps, s.stepsR);
      for (const side of [l.left, l.right]) {
        assert.ok(side.outMm <= 220.05, name); assert.ok(side.steps <= 12, name);
        if (s.endName === 'COMPLETE') assert.ok(['FAR', 'CEILING', 'CLEARANCE', 'STEPS'].includes(side.limitName), name);
        if (side.minTurnGapMm !== null) assert.ok(side.minTurnGapMm >= 49.85, name + ' turn gap ' + side.minTurnGapMm);
      }
      runs++;
    }
    for (const r of res) { assert.equal(r.version, 3); assert.equal(r.ruleId, 2); assert.equal(r.offsetMm, 10);
      assert.equal(r.rangeLoMm, 100); assert.equal(r.rangeHiMm, 200); }
  }
  assert.ok(runs >= 15, 'runs ' + runs);
});

test('width display shows raw, corrected-before-clamp and the final value with its label', () => {
  /* clamped: raw - 10 > 200 */
  const clamp = of(decodeAll('sonar faults in direction probe'), 'b5-width-result')[0];
  assert.equal(clamp.statusName, 'ESTIMATE'); assert.ok(clamp.measured && !clamp.assumed);
  assert.equal(Math.round(clamp.correctedMm * 100), Math.round(clamp.rawMm * 100) - 1000);
  assert.ok(clamp.correctedMm > 200); assert.equal(clamp.displayMm, 200);
  assert.match(clamp.label, /^estimate/);
  /* in range: display == corrected */
  const ok = of(decodeAll('100 offset RIGHT 35 -> LEFT'), 'b5-width-result')[0];
  assert.equal(ok.displayMm, ok.correctedMm); assert.equal(Math.round(ok.correctedMm * 100), Math.round(ok.rawMm * 100) - 1000);
  /* unresolved edge under clearance: labelled assumption, no raw/corrected */
  const ass = of(decodeAll('185 offset RIGHT 60 -> LEFT'), 'b5-width-result')[0];
  assert.equal(ass.basisName, 'PROJECT_RANGE_ASSUMPTION'); assert.ok(ass.assumed); assert.equal(ass.displayMm, 150);
  assert.equal(ass.rawMm, null); assert.equal(ass.correctedMm, null); assert.match(ass.label, /ASSUMED/);
  const st = of(decodeAll('185 offset RIGHT 60 -> LEFT'), 'b5-status');
  assert.match(st[st.length - 1].widthLabel, /ASSUMED project midpoint/);
  const l = of(decodeAll('185 offset RIGHT 60 -> LEFT'), 'b5-width-limits')[0];
  assert.equal(l.left.limitName, 'FAR'); assert.equal(l.right.limitName, 'CLEARANCE');
});

test('E5 v3 reports the live width-probe state, side, sample phase and step', () => {
  const st = of(decodeAll('185 offset RIGHT 60 -> LEFT'), 'b5-status');
  const during = st.filter((s) => s.stageName === 'width probe');
  assert.ok(during.length > 10);
  assert.ok(during.every((s) => s.version === 3 && s.probeStateName !== null && s.samplePhaseName !== null));
  const states = new Set(during.map((s) => s.probeStateName)), sides = new Set(during.map((s) => s.probeSide));
  assert.ok(states.has('STEP') || states.has('OUT'), [...states].join(','));
  assert.ok(sides.has('L') && sides.has('R'), [...sides].join(','));
  assert.ok(during.some((s) => s.sampleStep > 0));
  const bad = new Uint8Array(3 + 61); bad[0] = 0xF2; bad[1] = 61; bad[3 + 4] = 0xE5; bad[3 + 5] = 3; bad[3 + 59] = 0x30;
  assert.match(M.decodeFrame(bad).error, /range/);
  const v3In59 = new Uint8Array(3 + 59); v3In59[0] = 0xF2; v3In59[1] = 59; v3In59[3 + 4] = 0xE5; v3In59[3 + 5] = 3;
  assert.match(M.decodeFrame(v3In59).error, /version/);
});

test('legacy v0.2 captures stay readable under their original rules', () => {
  let n = 0;
  for (const [name, frames] of legacy) for (const b of frames) {
    const f = M.decodeFrame(b); n++;
    assert.ok(!f.error, `${name}: ${f.error}`);
  }
  assert.ok(n > 1000);
  const d = decodeAll('185 offset RIGHT 60 -> LEFT', legacy);
  const st = of(d, 'b5-status');
  assert.ok(st.every((s) => s.version === 2 && s.probeState === null));
  const r = of(d, 'b5-width-result')[0];
  assert.equal(r.version, 1); assert.equal(r.protocol, 'legacy');
  assert.equal(of(d, 'b5-width-limits').length, 0);
  const t = M.B5Tracker(); d.forEach((f) => t.add(f.data));
  assert.equal(t.width().protocol, 'legacy'); assert.ok(t.width().complete);
  /* basis 4 does not exist in the legacy D3 v1 protocol */
  const raw = legacy.get('185 offset RIGHT 60 -> LEFT').find((b) => b[1] === 58 && b[3 + 4] === 0xD3);
  const b4 = Uint8Array.from(raw); b4[3 + 9] = 4;
  assert.match(M.decodeFrame(b4).error, /range/);
});

test('a run mixing legacy and long width protocols is rejected, never merged', () => {
  const longD = decodeAll('185 offset RIGHT 60 -> LEFT');
  const oldRec = legacy.get('185 offset RIGHT 60 -> LEFT').find((b) => b[1] === 56 && b[3 + 4] === 0xD1);
  const t = M.B5Tracker(); longD.forEach((f) => t.add(f.data));
  const intruder = M.decodeFrame(Uint8Array.from(oldRec)).data;
  intruder.run = t.width().run; intruder.index = 60; t.add(intruder);
  assert.equal(t.width().protocol, 'mixed'); assert.ok(!t.width().complete);
  assert.match(t.width().verification, /REJECTED/);
  const d5bad = longD.find((f) => f.data.kind === 'b5-width-limits');
  const bytes = Uint8Array.from(scenarios.get('185 offset RIGHT 60 -> LEFT').find((b) => b[1] === 36 && b[3 + 4] === 0xD5));
  bytes[3 + 5] = 1; assert.match(M.decodeFrame(bytes).error, /D5/);
  assert.ok(d5bad);
});
