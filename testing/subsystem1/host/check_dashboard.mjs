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

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
