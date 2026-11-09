'use strict';
const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const schema=require('../is28_schema.js');
const base=path.resolve(__dirname,'../../testing/rtos_migration/build-host');
function decode(bytes){return schema.decodeFrame(Uint8Array.from(bytes));}
test('actual production-C C8 fixture preserves signed targets, angle and bounded bend state',()=>{
  const f=decode(fs.readFileSync(path.join(base,'drive_diagnostics.bin')));assert(!f.error,f.error);
  const d=f.data;assert.equal(d.kind,'drive-state');assert.equal(d.version,'0.2.2');
  assert.equal(d.runId,7);assert.equal(d.uptimeMs,123456);assert.equal(d.bendPhase,2);
  assert.equal(d.bendSide,1);assert.equal(d.bendPercent,100);assert.equal(d.recoveryPhase,3);
  assert.equal(d.sweepStage,4);assert.equal(d.recoveryYawDeg,-12.3);
  assert.equal(d.targetLeft,-35);assert.equal(d.targetRight,35);assert(!d.saturated);
});
test('C8 rejects reserved bits, invalid direction and malformed length',()=>{
  const original=fs.readFileSync(path.join(base,'drive_diagnostics.bin'));
  let f=Buffer.from(original);f[12]|=0xc0;assert(decode(f).error);
  f=Buffer.from(original);f[11]|=12;assert(decode(f).error);
  f=Buffer.from(original);f[17]|=0xf8;assert(decode(f).error);
  assert(decode(original.subarray(0,-1)).error);
});
test('controller rows retain the full-rate two-bit bend phase without changing other fields',()=>{
  const row=new Uint8Array(56);row[39]=0x80;row[51]=0x80;
  const d=schema.decodeStoredLine(row);assert.equal(d.bend_phase,3);
  assert.equal(d.control_flags,0);assert.equal(d.turn_degrees_x10,0);
});
test('actual production-C C7 fixture exposes measured junction width and explicit result',()=>{
  const f=decode(fs.readFileSync(path.join(base,'junction_decision.bin')));assert(!f.error,f.error);
  assert.equal(f.data.kind,'junction-decision');assert.equal(f.data.resultName,'white confirmed');
  assert.equal(f.data.widthMm,10.4);assert.equal(f.data.threshold,1100);assert.equal(f.data.hysteresis,90);
  assert.equal(f.data.blackEntry,1190);assert.equal(f.data.whiteReturn,1010);
});
