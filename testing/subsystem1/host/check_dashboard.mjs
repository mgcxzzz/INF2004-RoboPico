// Checks that the dashboard decoder (tools/dashboard/is28_schema.js) reads
// the frames packed by the C code exactly as sys_tlm.h defines them.
//   make -C testing/subsystem1/host vectors && node testing/subsystem1/host/check_dashboard.mjs
// Group IS28, Buddy 1.
import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const IS28 = createRequire(import.meta.url)(join(here, '../../../tools/dashboard/is28_schema.js'));
const v = JSON.parse(readFileSync(join(here, 'vectors.json'), 'utf8'));
const hex = (h) => Uint8Array.from(h.match(/../g).map((x) => parseInt(x, 16)));

let pass = 0, fail = 0;
function check(id, cond, what) {
  if (cond) pass++; else fail++;
  console.log(`${cond ? 'PASS' : 'FAIL'}  ${id.padEnd(6)} ${what}`);
}

// Values set in test_b1.c, vectors().
const b1 = IS28.decodeFrame(hex(v.b1));
const d = b1.data;
check('D01', b1.src === 1 && b1.seq === 17 && d.net === 2 && d.state === 1 &&
  d.lastCmd === 12 && d.lastResult === 1 && d.uptimeS === 1234 && d.published === 54321 &&
  d.lost === 3 && d.accepted === 9 && d.rejected === 4 && d.connects === 2 && d.fillPct === 37,
  'Buddy 1 status frame');

const nav = IS28.decodeFrame(hex(v.nav)).data;
check('D02', nav.state === 2 && nav.owner === 4 && nav.slow === true && nav.searchStep === 3 &&
  nav.dropped === 200 && nav.commandsSent === 40000, 'Navigation frame');

const sf = IS28.decodeFrame(hex(v.safety)).data;
check('D03', sf.state === 1 && sf.inhibit && sf.faultSrc === 'Obstacle' && sf.warnSrc === 'none' &&
  sf.faultCode === 0x21 && sf.warnCode === 0x08 && sf.estops === 3 && sf.faults === 1 &&
  sf.warnings === 250 && sf.lastStopMs === 12 && sf.maxStopMs === 1023 && sf.faultMask === 0x15,
  'Safety frame');

const cmd = IS28.encodeCommand(IS28.CMD.CONFIG, 77, IS28.configArg(IS28.CFG_TELEMETRY, 1));
check('D04', Buffer.from(cmd).toString('hex') === v.cmd_config, 'command bytes match the C packer');

const bad = hex(v.b1).slice(0, 10);
check('D05', IS28.decodeFrame(bad).error !== undefined, 'short frame refused');

const t = new IS28.SeqTracker();
[1, 2, 3, 5, 6, 255, 0, 1].forEach((s) => t.add(1, s));
check('D06', t.per[1].missing === 1 && t.per[1].received === 8,
  'sequence gaps counted with 8-bit wrap; a jump of 128+ is a restart, not losses');

const letters = ['Z', 'A', 'M', 'Q'];
check('D07', Buffer.from(IS28.encodeMapping(201, letters)).toString('hex') === v.cmd_mapping,
  'four-letter command matches C bytes, including A/Z boundaries');
const mapping = IS28.decodeFrame(hex(v.mapping));
check('D08', mapping.src === IS28.SRC.BARCODE_MAP && mapping.seq === 255 &&
  mapping.data.configured && mapping.data.requestId === 201 && mapping.data.revision === 1 &&
  mapping.data.letters.join('') === letters.join(''), 'active mapping readback matches C');
const empty = IS28.decodeFrame(hex(v.mapping_empty)).data;
check('D09', !empty.configured && empty.letters.length === 0 && empty.requestId === 7 &&
  IS28.mappedMovement(empty, 'A') === 'not configured', 'boot has no fabricated mapping');
check('D10', IS28.mappedMovement(mapping.data, 'Z') === 'Forward' &&
  IS28.mappedMovement(mapping.data, 'A') === 'U-turn' &&
  IS28.mappedMovement(mapping.data, 'M') === 'Left' &&
  IS28.mappedMovement(mapping.data, 'Q') === 'Right' &&
  IS28.mappedMovement(mapping.data, 'B') === 'unassigned', 'all slots and unassigned letters');
let refused = 0;
for (const bad of [['A', 'A', 'B', 'C'], ['a', 'B', 'C', 'D'], ['', 'B', 'C', 'D'],
                   ['*', 'B', 'C', 'D'], ['AB', 'C', 'D', 'E'], ['A', 'B', 'C'],
                   [0, 'B', 'C', 'D']]) {
  try { IS28.encodeMapping(1, bad); } catch { refused++; }
}
check('D11', refused === 7, 'invalid, missing and duplicate UI assignments refused');
const corrupt = hex(v.mapping); corrupt[6] |= 0xe0;
check('D12', !!IS28.decodeFrame(corrupt).error &&
  !!IS28.decodeFrame(hex(v.mapping).slice(0, -1)).error, 'corrupt or short readback refused');
check('D13', !!IS28.decodeAck([0x91, 1, 0, 0]).error &&
  IS28.decodeAck([0x92, 201, 0, 0]).accepted, 'foreign-version acknowledgements refused');
const get = IS28.encodeCommand(IS28.CMD.BARCODE_GET, 0, 0);
check('D14', Buffer.from(get).toString('hex') === 'a2000000', 'GET keeps the 4-byte command layout');
let allLetters = true;
for (let slot = 0; slot < 4; slot++) {
  for (let letter = 0; letter < 26; letter++) {
    const assignment = [0, 1, 2, 3].map((i) => String.fromCharCode(65 + (letter + i - slot + 26) % 26));
    const bytes = IS28.encodeMapping(0, assignment);
    allLetters &&= ((bytes[2] | (bytes[3] << 8) | (bytes[4] << 16)) >>> (slot * 5) & 31) === letter;
  }
}
check('D15', allLetters, 'every A-Z letter encodes in every movement slot');

const per = { 1: { received: 300, missing: 0 }, 4: { received: 100, missing: 0 } };
check('D16', IS28.connectivityVerdict(per, 300, 1.1, 300, true).heartbeatPass &&
  IS28.connectivityVerdict(per, 300, 1.1, 300, true).deliveryPass, 'complete connectivity run can pass');
check('D17', !IS28.connectivityVerdict(per, 10, 1, 10, true).complete &&
  !IS28.connectivityVerdict(per, 300, 1, 300, false).complete, 'short and running tests cannot pass');
check('D18', !IS28.connectivityVerdict(per, 300, 200, 2, true).heartbeatPass,
  'silent tail fails heartbeat even after two good heartbeats');
check('D19', !IS28.connectivityVerdict({ 1: per[1], 4: { received: 10, missing: 10 } },
  300, 1, 300, true).deliveryPass, 'one bad source cannot be hidden by good aggregate delivery');
check('D20', !IS28.connectivityVerdict({ 8: { received: 1000, missing: 0 } },
  300, 0, 0, true).deliveryPass, 'configuration readbacks alone do not pass connectivity');

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
