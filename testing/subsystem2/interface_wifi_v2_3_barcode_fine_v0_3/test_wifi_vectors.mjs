import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import IS28 from './dashboard/is28_schema.js';

const input = process.argv[2];
if (!input) throw new Error('usage: node test_wifi_vectors.mjs <compiled-wifi-vectors-executable|C-vector-json>');
const v = input.toLowerCase().endsWith('.json')
  ? JSON.parse(readFileSync(input, 'utf8'))
  : JSON.parse(execFileSync(input, { encoding: 'utf8' }));
const decode = (bytes) => IS28.decodeFrame(Uint8Array.from(bytes));

const hb=decode(v.heartbeat); assert.equal(hb.error,undefined);
assert.equal(hb.data.uptimeS,16383); assert.equal(hb.data.state,1); assert.equal(hb.data.lost,4);
const map=decode(v.map); assert.equal(map.error,undefined);
assert.deepEqual(map.data.letters,['S','A','M','Q']); assert.equal(map.data.requestId,0x4a);
const motion=decode(v.motion); assert.equal(motion.error,undefined);
assert.equal(motion.data.speedL,-2.5); assert.equal(motion.data.speedR,7.5);
assert.deepEqual(motion.data.levels,[400,800,65535]); assert.equal(motion.data.elapsedMs,70000);
const line=decode(v.line); assert.equal(line.error,undefined);
assert.equal(line.data.event,IS28.EVENT_NAME.indexOf('queued'));
assert.equal(line.data.pendingLetter,'A'); assert.equal(line.data.action,'S');
assert.equal(line.data.executingAction,'R'); assert.equal(line.data.executingRevision,3);
const nav=decode(v.nav); assert.equal(nav.error,undefined);
assert.equal(nav.data.commandId,0x12345678); assert.equal(nav.data.generation,0x10203040);
assert.equal(nav.data.action,'S'); assert.equal(nav.data.detail,99);
const summary=decode(v.summary); assert.equal(summary.error,undefined);
assert.equal(summary.data.distanceMm,654321); assert.equal(summary.data.recoveries,2);
const diagnostics=decode(v.diagnostics); assert.equal(diagnostics.error,undefined);
assert.equal(diagnostics.data.uptimeMs,0x78563412); assert.equal(diagnostics.data.thresholdAdc,2048);
assert.equal(diagnostics.data.bank,31); assert.equal(diagnostics.data.flags,0x0b);
assert.equal(diagnostics.data.decodedTotal,0x01020304); assert.equal(diagnostics.data.turns,0x11121314);
assert.equal(diagnostics.data.lefts,0x21222324); assert.equal(diagnostics.data.rights,0x31323334);
assert.equal(diagnostics.data.uturns,0x41424344); assert.equal(diagnostics.data.badFrames,0x51525354);
assert.equal(diagnostics.data.unsupported,0x61626364); assert.equal(diagnostics.data.maxAdcGapUs,0x71727374);
assert.equal(diagnostics.data.maxControlGapUs,0x81828384); assert.equal(diagnostics.data.txDropped,0x91929394);
assert.equal(diagnostics.data.rxDropped,0xa1a2a3a4); assert.equal(diagnostics.data.invalidLeft,0xb1b2b3b4);
assert.equal(diagnostics.data.invalidRight,0xc1c2c3c4);
const log=decode(v.log); assert.equal(log.error,undefined);
assert.deepEqual(log.data.bytes,[0x11,0x22,0x33]); assert.equal(log.data.partCount,2);
const ack=IS28.decodeAck(Uint8Array.from(v.ack)); assert.equal(ack.error,undefined);
assert.equal(ack.code,IS28.CMD.GET_LOG); assert.equal(ack.id,0x4a); assert.equal(ack.accepted,true);
console.log('C-generated telemetry vectors decode in the dashboard schema');
