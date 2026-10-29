/* Merged dashboard v0.1 browser checks with a SIMULATED MQTT transport.
 * No broker, Pico or car is used and no screenshots are taken; results are
 * read from the DOM, the published-command log and downloaded files.
 *   node test/ui_merged.mjs   (needs Playwright; PLAYWRIGHT_NODE_MODULE_PATH / CHROME_PATH optional)
 */
import { createRequire } from 'node:module';
import { createServer } from 'node:http';
import { readFile, mkdtemp } from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { tmpdir } from 'node:os';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

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

try {
  /* ================= SDK v2.3 + Buddy4 add-on ================= */
  const p = await openPage();
  check('S01 first automatic publish is the read-only mapping query', (await sentCodes(p))[0] === 10);
  await emit(p, 'is28/hb', hbF(100), { retain: true });
  check('S02 retained heartbeat does not make the car online', !(await text(p, 'carlink')).includes('online'));
  await emit(p, 'is28/hb', hbF(101));
  check('S03 fresh heartbeat shows car: online', (await text(p, 'carlink')) === 'car: online');
  await emit(p, 'is28/tlm', sdkNav(7)); await emit(p, 'is28/tlm', sdkMotion()); await emit(p, 'is28/tlm', sdkDiag());
  check('S04 SDK packets select the SDK profile', (await text(p, 'profilePill')).includes('SDK v2.3'));
  const safetySdk = await text(p, 'safety');
  check('S05 RTOS Safety shown as not provided on SDK firmware, never Normal', safetySdk.includes('not provided by SDK') && !safetySdk.includes('Normal'));
  check('S06 SDK measured/target speed, PWM, ADC, levels with unavailable marker',
    (await text(p, 'speedMeasured')) === '12 / 13 mm/s' && (await text(p, 'speedTarget')) === '45 / 45 mm/s' &&
    (await text(p, 'pwm')) === '30 / 32' && (await text(p, 'adc')) === '400 / 410 / 420' &&
    (await text(p, 'levels')) === '321 / 654 / unavailable · mask 0x5' && (await text(p, 'heading')) === '-2.00°');
  await emit(p, 'is28/tlm', b2(-7));
  check('S07 live right-minus-left height shown with sign', (await text(p, 'motionHeight')) === '-7 mm');
  check('S08 profile notes the Buddy4 add-on', (await text(p, 'profilePill')).includes('Buddy4 add-on'));
  await emit(p, 'is28/tlm', imu(3, 10, 1, 10));
  check('S09 accepted hump peak shown separately', (await text(p, 'imuHeight')) === '10 mm' && (await text(p, 'imuHighest')) === '10 mm');
  await emit(p, 'is28/tlm', b2(3));
  check('S10 later live height does not replace the accepted ramp height',
    (await text(p, 'motionHeight')) === '3 mm' && (await text(p, 'imuHeight')) === '10 mm');
  check('S11 pitch and roll shown', (await text(p, 'imuPitch')) === '2.5°' && (await text(p, 'motionRoll')) === '-1.2°');

  /* command acceptance vs confirmed state */
  await p.locator('button[data-cmd=START]').click();
  const startId = await p.evaluate(() => window.__m.sent.at(-1).bytes[1]);
  await emit(p, 'is28/ack', ack(1, startId, 0, 1));
  check('S12 accepted START reported as acceptance, state still from telemetry',
    (await text(p, 'cmdNote')).includes('Acceptance is not') && (await text(p, 'opstate')) === 'Off');
  /* original v2.3: per-run Fault survives an accepted Reset */
  await emit(p, 'is28/hb', hbF(102, 6)); await emit(p, 'is28/tlm', sdkNav(6, 2));
  check('S13 SDK fault displayed', (await text(p, 'opstate')) === 'Fault' && (await text(p, 'sdkFault')) === '0x2');
  await p.locator('button[data-cmd=RESET]').click();
  const resetId = await p.evaluate(() => window.__m.sent.at(-1).bytes[1]);
  await emit(p, 'is28/ack', ack(5, resetId, 0, 6));
  await sleep(300);
  check('S14 accepted Reset with lingering v2.3 per-run Fault is explained, not hidden',
    (await text(p, 'opNote')).includes("previous run's Fault") && (await text(p, 'opstate')) === 'Fault');
  await emit(p, 'is28/hb', hbF(103, 3)); await emit(p, 'is28/tlm', sdkNav(7));

  /* mixed firmware frames must not produce a Normal Safety state */
  await emit(p, 'is28/tlm', safety(0));
  check('S15 RTOS Safety frame alongside SDK frames is flagged, not shown as Normal',
    (await text(p, 'profileDetected')).includes('MIXED') && !(await text(p, 'safety')).includes('Normal'));

  /* car restart invalidates live readings but keeps the recording */
  const framesBefore = await p.evaluate(() => frames.length);
  await emit(p, 'is28/hb', hbF(5));
  check('S16 restart clears SDK live values', (await text(p, 'speedMeasured')) === '—' && (await text(p, 'diagCounts')) === '—');
  check('S17 restart clears Buddy2/Buddy4 live values with a reason',
    (await text(p, 'motionHeight')).includes('waiting for fresh Buddy 2 telemetry after car restart') &&
    (await text(p, 'imuHeight')).includes('waiting for fresh Buddy 4 telemetry'));
  check('S18 restart keeps recorded evidence', (await p.evaluate(() => frames.length)) === framesBefore + 1 &&
    (await text(p, 'eventTimeline')).includes('Live readings cleared'));
  check('S19 restart resets observed profile (mixed state cleared)', !(await text(p, 'profileDetected')).includes('MIXED'));
  await emit(p, 'is28/tlm', sdkNav(7)); await emit(p, 'is28/tlm', sdkMotion(1000, 5));

  /* recording, label, markers */
  await p.locator('#runLabel').fill('run A/1');
  await p.locator('#runLabelSet').click();
  check('S20 run label sanitised and applied', (await p.locator('#runLabel').inputValue()) === 'run-A-1');
  for (let i = 0; i < 5; i++) { await emit(p, 'is28/tlm', sdkMotion(2000 + i * 50, 5)); await emit(p, 'is28/tlm', b2(i)); }
  await emit(p, 'is28/tlm', imu(3, 11, 2, 11));
  await p.locator('#markNote').fill('ramp');
  await p.locator('#markBtn').click();
  check('S21 marker recorded with note and Pico uptime estimate',
    (await text(p, 'markStatus')).startsWith('M1') && /Marker M1 · ramp · Pico uptime ≈ \d+ ms/.test(await text(p, 'eventTimeline')));
  await emit(p, 'is28/tlm', chunk64());
  const last = await p.evaluate(() => frames.filter((f) => f.src === 13).at(-1));
  check('S22 64-byte SDK frame recorded whole (no 40-byte truncation)', last.len === 64 && last.error === null && last.data.bytes.length === 49);
  await emit(p, 'is28/cmd', [0x12, 200, 0, 0]);
  check('S23 command from another client is surfaced', (await text(p, 'otherClients')).includes('Another client sent START') &&
    (await p.locator('#otherClients').getAttribute('class')).includes('warn'));

  /* clearing the visible log does not erase recordings */
  const counts = await p.evaluate(() => [logLines.length, frames.length, eventRows.length]);
  await p.locator('#logClear').click();
  const after = await p.evaluate(() => [logLines.length, frames.length, eventRows.length]);
  check('S24 Clear visible keeps message log, frames and events', after[0] >= counts[0] && after[1] === counts[1] && after[2] === counts[2] &&
    (await text(p, 'logStatus')).includes('still contains'));

  /* exports */
  const [tcsv] = await saveDownload(p, 'csvSave');
  const tl = tcsv.text.trim().split(/\r\n/);
  check('S25 telemetry CSV keeps v2.3 columns and adds source/seq/label', tl[0].startsWith('received_at,uptime_basis,uptime_ms,run_id') &&
    tl[0].endsWith('src,seq,laptop_ms,run_label,firmware_profile') && tl.length === 6 && tl.slice(1).every((l) => l.includes(',run-A-1,')));
  const [ejson] = await saveDownload(p, 'eventJsonSave');
  const ev = JSON.parse(ejson.text);
  check('S26 event JSON includes markers and capture limits', ev.markers.length === 1 && ev.markers[0].note === 'ramp' && ev.capture.limits.events === 20000);
  const [fcsv] = await saveDownload(p, 'framesCsvSave');
  const fl = fcsv.text.trim().split(/\r\n/);
  check('S27 all-frames CSV has laptop ISO time, Pico uptime, source/sequence, run ID and decoded data',
    fl[0] === 'received_at,laptop_ms,topic,retained,src,src_name,family,seq,len,pico_uptime_ms,run_id,latest_run_id,run_label,firmware_profile,error,hex,decoded_json' &&
    fl.some((l) => l.includes('is28/tlm') && l.includes(',9,Motion telemetry,sdk,') && l.includes(',2000,5,5,run-A-1,')));
  const [bcsv] = await saveDownload(p, 'b4CsvSave');
  const bl = bcsv.text.trim().split(/\r\n/), bh = bl[0].split(',');
  const liveCol = bh.indexOf('live_right_minus_left_height_mm'), accCol = bh.indexOf('accepted_peak_mm');
  const imuRow = bl.slice(1).find((l) => l.split(',')[2] === '4').split(',');
  check('S28 Buddy4 CSV separates live height and accepted peak', liveCol > 0 && accCol > 0 && imuRow[accCol] === '11' && imuRow[liveCol] === '' &&
    bl.slice(1).filter((l) => l.split(',')[2] === '2').map((l) => l.split(',')[liveCol]).join('|') === '0|1|2|3|4');
  const ev2 = await saveDownload(p, 'evidenceSave', 7);
  const manifest = JSON.parse(ev2.find((f) => f.name.endsWith('_manifest.json')).text);
  let hashesOk = manifest.files.length === 6;
  for (const f of manifest.files) {
    const got = ev2.find((x) => x.name === f.name);
    hashesOk = hashesOk && !!got && createHash('sha256').update(await readFile(got.path)).digest('hex') === f.sha256;
  }
  check('S29 evidence set: 7 files, manifest SHA-256 matches every file', hashesOk);
  const fj = JSON.parse(ev2.find((f) => f.name.endsWith('_frames.json')).text);
  check('S30 evidence set scoped to the run label', manifest.scopeLabel === 'run-A-1' && fj.frames.length > 0 &&
    fj.frames.every((r) => r.runLabel === 'run-A-1') && manifest.rowCounts.markers === 1);

  /* discard needs two clicks */
  const n1 = await p.evaluate(() => frames.length);
  await p.locator('#discardRec').click();
  check('S31 first discard click only arms', (await p.evaluate(() => frames.length)) === n1 && (await text(p, 'discardRec')).includes('Click again'));
  await p.locator('#discardRec').click();
  check('S32 second click discards and reports what was removed', (await p.evaluate(() => frames.length)) === 0 && (await text(p, 'exportStatus')).includes('Discarded'));

  /* staleness: coordinator status stops (telemetry off) */
  await emit(p, 'is28/tlm', sdkNav(7));
  await sleep(4000);
  check('S33 stale coordinator status disables stored downloads with a reason',
    (await p.locator('#logDownload').isDisabled()) && (await text(p, 'logDownloadStatus')).includes('stale') &&
    (await text(p, 'navDetails')).includes('stale'));
  check('S34 stale SDK motion marked', (await text(p, 'sdkMotionStatus')).startsWith('STALE'));

  /* no automatic motion commands */
  const codes = await sentCodes(p);
  check('S35 only GET was sent automatically; START and RESET only when clicked',
    codes.every((c) => [10, 1, 5].includes(c)) && codes.filter((c) => c === 1).length === 1 && codes.filter((c) => c === 5).length === 1);
  await p.locator('#connect').click();
  check('S36 disconnect: car link unknown and live values cleared', (await text(p, 'carlink')).includes('unknown') && (await text(p, 'speedMeasured')) === '—');

  /* ================= RTOS ================= */
  const r = await openPage();
  await emit(r, 'is28/hb', hbF(50, 1)); await emit(r, 'is28/tlm', nav6(1)); await emit(r, 'is28/tlm', safety(0));
  check('R01 RTOS profile and Safety Normal from a fresh frame', (await text(r, 'profilePill')).includes('RTOS') &&
    (await text(r, 'safety')) === 'Normal' && (await text(r, 'opstate')) === 'Running');
  check('R02 stored downloads explained as SDK-only; STEER test enabled on RTOS',
    (await r.locator('#logDownload').isDisabled()) && (await text(r, 'logDownloadStatus')).includes('SDK v2.3 feature') &&
    !(await r.locator('#steerBtn').isDisabled()));
  for (let i = 0; i < 4; i++) { await sleep(900); await emit(r, 'is28/hb', hbF(51 + i, 1)); }
  const st = await text(r, 'safety');
  check('R03 stale Safety never shown as Normal', st.includes('stale') && !st.includes('Normal'));
  check('R04 stale navigation falls back to heartbeat state', (await text(r, 'opstateSource')).includes('heartbeat'));
  await emit(r, 'is28/lwt', [0]);
  check('R05 last will: car offline and live values cleared', (await text(r, 'carlink')) === 'car offline (broker last will)' &&
    (await text(r, 'safety')).includes('no Safety frame'));

  const i3 = await openPage({ width: 390, height: 844 });
  await emit(i3, 'is28/hb', hbF(20));
  for (let i = 0; i < 8; i++) { await emit(i3, 'is28/tlm', b2(1)); await emit(i3, 'is28/tlm', imu(0, 0, 0, 0)); await sleep(450); }
  check('R06 RTOS Buddy4 build (shared sources only) inferred as RTOS', (await text(i3, 'profileDetected')).includes('inferred'));
  check('R07 mobile layout has no horizontal overflow', await i3.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
  check('R08 no page or console errors', errors.length === 0);
  console.log(`\n${count} passed, 0 failed (simulated MQTT; no broker, Pico or car)`);
} catch (e) {
  console.error(e.message || e); if (errors.length) console.error('page errors:', errors); process.exitCode = 1;
} finally { await browser.close(); server.close(); }
