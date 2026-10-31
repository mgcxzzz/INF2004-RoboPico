/* Combined Buddy1-4 + barcode peak build: dashboard browser checks with a SIMULATED MQTT transport.
 * Frames come from the REAL C packers (fixtures/sdk_b4_peak_c_frames.txt, see test_source15.js).
 * No broker, Pico or car is used and no screenshots are taken; results are
 * read from the DOM, the published-command log and downloaded files.
 *   node test/ui_merged.mjs   (needs Playwright; PLAYWRIGHT_NODE_MODULE_PATH / CHROME_PATH optional)
 */
import { createRequire } from 'node:module';
import { createServer } from 'node:http';
import { readFile, mkdtemp } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_NODE_MODULE_PATH || 'playwright');
const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const types = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8' };
const server = createServer(async (req, res) => {
  const p = new URL(req.url, 'http://x').pathname;
  const file = p === '/' ? 'index.html' : p.slice(1);
  if (!['index.html', 'is28_schema.js', 'vendor/mqtt.min.js'].includes(file)) { res.writeHead(404).end(); return; }
  const body = await readFile(join(root, file));
  res.writeHead(200, { 'Content-Type': types[file.slice(file.lastIndexOf('.'))] }); res.end(body);
});

/* ---------- frame builders (layouts from wifi_packets.c and sys_tlm.h) ---------- */
const le = (n, bytes) => Array.from({ length: bytes }, (_, i) => (n >>> (8 * i)) & 255);
const fr = (src, p, seq) => [(src << 4) | 2, p.length, seq & 255, ...p];
let seq = 1;
const hbF = (uptimeS, state = 0) => fr(1, [...le(2 | (state << 2) | (7 << 9) | (uptimeS << 12), 4), ...le(10, 4), ...le(0, 4)], seq++);
const sdkNav = (mode, fault = 0, event = 0, up = 5000) => fr(11, [...le(up, 4), event, 0, 0, 3, ...le(0, 4), ...le(1, 4), ...le(fault, 2), 1, mode, 0, ...le(0, 2)], seq++);
const sdkMotion = (up = 6000, runId = 4) => fr(9, [...le(up, 4), ...le(1500, 4), ...le(runId, 2), 4, 1, 0x07, 0, ...le(120, 2), ...le(130, 2), ...le(45, 2), ...le(45, 2), ...le(30, 2), ...le(32, 2),
  ...le(400, 2), ...le(410, 2), ...le(420, 2), ...le(321, 2), ...le(654, 2), ...le(0xffff, 2), 5, ...le(678, 4), ...le(0xff38, 2)], seq++);
const sdkDiag = () => fr(14, [...le(7000, 4), ...le(1700, 2), 17, 0x04, ...new Array(52).fill(0)], seq++);
const b2 = (height, flags = 0x07) => fr(2, [flags, 0, ...le(40, 2), ...le(42, 2), ...le(41, 2), ...le(-12 & 0xffff, 2), ...le(height & 0xffff, 2), ...le(250, 4), ...le(0, 4)], seq++);
const imu = (eventIdx, height, humps, highest, pitchTenths = 25, state = 3) => {
  const w0 = state | (2 << 3) | (eventIdx << 6) | (1 << 13) | (1 << 14) | ((pitchTenths & 0x7ff) << 15);
  return fr(4, [...le(w0, 4), height, highest, humps, 0, ...le(0, 4), ...le(0, 4)], seq++);
};
const safety = (state = 0) => fr(7, [...le(state, 4), ...le(0, 4), ...le(0, 4)], seq++);
const nav6 = (state = 1) => fr(6, [...le(state | (1 << 3), 4), 0, 0], seq++);
const chunk64 = () => fr(13, [1, 99, 0, 0, 1, 0, 0, 1, 0, 49, 0, 0, ...Array.from({ length: 49 }, (_, i) => i)], seq++);
const ack = (code, id, result = 0, state = 0) => [(code << 4) | 2, id, result, state];

const mock = `
window.__m = { handlers:{}, sent:[], connected:false,
  emit(topic, b, packet){ const h=this.handlers.message; if(h) h(topic, new Uint8Array(b), packet); } };
window.mqtt = { connect(){ const c = { get connected(){return window.__m.connected;},
  on(n,fn){ window.__m.handlers[n]=fn; return this; },
  subscribe(t,cb){ setTimeout(()=>cb&&cb(null),0); return this; },
  publish(t,p,o){ window.__m.sent.push({topic:t, bytes:Array.from(p), options:o}); },
  end(){ window.__m.connected=false; } };
  setTimeout(()=>{ window.__m.connected=true; window.__m.handlers.connect && window.__m.handlers.connect(); },0); return c; } };`;

let count = 0;
const check = (label, ok) => { assert.ok(ok, 'FAIL ' + label); console.log('PASS ' + label); count++; };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
const url = `http://127.0.0.1:${server.address().port}/`;
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME_PATH || undefined });
const dl = await mkdtemp(join(tmpdir(), 'is28-merged-'));
const errors = [];
async function openPage(viewport = { width: 1400, height: 1000 }) {
  const ctx = await browser.newContext({ acceptDownloads: true, viewport });
  const page = await ctx.newPage();
  page.on('pageerror', (e) => errors.push(String(e)));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  await page.route('**/vendor/mqtt.min.js', (r) => r.fulfill({ status: 200, contentType: 'text/javascript', body: mock }));
  await page.goto(url);
  await page.locator('#connect').click();
  await page.waitForFunction(() => window.__m.connected && window.__m.sent.length > 0);
  return page;
}
const emit = (page, topic, b, packet) => page.evaluate(([t, x, p]) => window.__m.emit(t, x, p), [topic, b, packet || null]);
const text = (page, id) => page.locator('#' + id).textContent();
const sentCodes = (page) => page.evaluate(() => window.__m.sent.map((s) => s.bytes[0] >> 4));
async function saveDownload(page, clickId, n = 1) {
  const got = [];
  const handler = (d) => got.push(d);
  page.on('download', handler);
  await page.locator('#' + clickId).click();
  for (let i = 0; i < 80 && got.length < n; i++) await sleep(100);
  page.off('download', handler);
  assert.equal(got.length, n, `${clickId} produced ${got.length} downloads, expected ${n}`);
  const out = [];
  for (const d of got) { const p = join(dl, d.suggestedFilename()); await d.saveAs(p); out.push({ name: d.suggestedFilename(), path: p, text: await readFile(p, 'utf8') }); }
  return out;
}

const cframes = readFileSync(join(root, 'test', 'fixtures', 'sdk_b4_peak_c_frames.txt'), 'utf8')
  .split('\n').filter((l) => l.startsWith('TLM ')).map((l) => Array.from(Buffer.from(l.slice(4), 'hex')));
try {
  const p = await openPage();
  await emit(p, 'is28/hb', hbF(200));
  check('P01 before any decode the B3 letter shows no frames', (await text(p, 'barcodeLetter')) === 'no frames');
  for (const b of cframes) await emit(p, 'is28/tlm', b);
  check('P02 Last decoded letter (B3) updates from the real SDK decode (sources 10/11)', (await text(p, 'barcodeLetter')) === 'Y');
  check('P03 mapped movement from the confirmed mapping readback', (await text(p, 'barcodeMovement')) === 'Forward');
  check('P04 progress card: latest letter and pending action', (await text(p, 'eventLetter')) === 'Y' && (await text(p, 'pendingAction')).startsWith('Y'));
  check('P05 peak decoder card shows the queued letter', (await text(p, 'peakResult')).startsWith('read Y → queued'));
  check('P06 ramp-assist card decodes the interleaved ramp frames', (await text(p, 'rampAssist')).startsWith('idle'));
  const build = await text(p, 'buildId');
  check('P07 combined build identified (peak bit7 + ramp detector/estimator)', build.includes('combined Buddy1-4 + peak') && build.includes('peak v0.1 active') && build.includes('detector 2') && build.includes('estimator 2'));
  check('P08 SDK profile selected', (await text(p, 'profilePill')).includes('SDK'));
  const log = await p.evaluate(() => document.getElementById('eventTimeline').textContent);
  check('P09 timeline has the peak read and the run-start announce', log.includes('Barcode peak read Y') && log.includes('run start'));
  /* connectivity counter must not mix the two source-15 sequences */
  await p.locator('#testStart').click();
  for (const b of cframes) if (b[0] === 0xF2) await emit(p, 'is28/tlm', b);
  await p.locator('#testStop').click();
  const rows = await p.evaluate(() => Array.from(document.querySelectorAll('#testRows tr')).map((tr) => Array.from(tr.cells).map((c) => c.textContent)));
  const r15 = rows.find((r) => r[0] === 'Ramp assist (src 15)'), r16 = rows.find((r) => r[0] === 'Barcode peak (src 15)');
  check('P10 connectivity test counts ramp and peak source-15 streams separately, 0 missing each',
    r15 && r16 && Number(r15[1]) >= 10 && r15[2] === '0' && Number(r16[1]) === 2 && r16[2] === '0');
  const constraint = Buffer.alloc(21);
  constraint.writeUInt32LE(220000,0); constraint[4]=0xC1; constraint[5]=9;
  constraint[6]=200; constraint.writeUInt16LE(1000,9);
  constraint.writeInt16LE(0,11); constraint.writeInt16LE(450,13);
  constraint.writeInt16LE(350,15); constraint.writeInt16LE(450,17);
  constraint[19]=3; constraint[20]=50;
  await emit(p,'is28/tlm',fr(15,Array.from(constraint),1));
  check('P14 ramp requested/effective targets are visible', (await text(p,'rampTargets')).includes('0/45 → 35/45'));
  check('P15 provisional status and missing Buddy5 are explicit', (await p.locator('h1').textContent()).includes('PROVISIONAL') && (await p.locator('h1').textContent()).includes('Buddy 5 NOT integrated'));
  /* unknown source-15 layout is logged as bad, not decoded */
  await emit(p, 'is28/tlm', [0xF2, 3, 9, 1, 2, 3]);
  check('P11 unknown source-15 layout is rejected', (await p.evaluate(() => document.body.textContent)).includes('unknown source-15 layout'));
  /* car restart: mapping needs readback; staleness handling preserved */
  await emit(p, 'is28/hb', hbF(5));
  check('P12 car restart: mapping unknown again, build/peak cards cleared', (await text(p, 'barcodeMovement')) === 'unknown' && (await text(p, 'peakResult')) === '—' && (await text(p, 'buildId')) === '—');
  check('P16 restart clears previous ramp target data', (await text(p,'rampTargets')) === '—');
  check('P13 no page or console errors', errors.length === 0);
  console.log(`\n${count} passed, 0 failed (simulated MQTT; frames from the real C packers)`);
} catch (e) {
  console.error(e.message || e); if (errors.length) console.error('page errors:', errors); process.exitCode = 1;
} finally { await browser.close(); server.close(); }
