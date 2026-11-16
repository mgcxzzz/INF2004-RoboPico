/* Focused v0.2.5 demo overview checks with simulated MQTT; no Pico or broker. */
import { createRequire } from 'node:module';
import { readFile } from 'node:fs/promises';
import { createServer } from 'node:http';
import { tmpdir } from 'node:os';
import { join, resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_NODE_MODULE_PATH || 'playwright');
const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const releaseDir = resolve(root, '..', 'testing', 'rtos_migration', 'build-host');
const mockSource = await readFile(join(root, 'test', 'ui_merged.mjs'), 'utf8');
const mock = mockSource.split('const mock = `')[1].split('`;')[0];
const server = createServer(async (req, res) => {
  const path = new URL(req.url, 'http://local').pathname;
  const file = path === '/' ? 'index.html' : path.slice(1);
  if (!['index.html', 'is28_schema.js', 'vendor/mqtt.min.js'].includes(file)) { res.writeHead(404).end(); return; }
  const type = file.endsWith('.html') ? 'text/html; charset=utf-8' : 'text/javascript; charset=utf-8';
  res.writeHead(200, { 'Content-Type': type }); res.end(await readFile(join(root, file)));
});
const le = (v, n) => Array.from({ length: n }, (_, i) => (v >>> (8 * i)) & 255);
const frame = (src, p, seq) => [(src << 4) | 2, p.length, seq & 255, ...p];
const ack = (code, id, result = 0, state = 0) => [(code << 4) | 2, id, result, state];
function driveStatus({ mode = 0, requested = 3, result = 0, canChange = true, active = false,
  loadL = true, loadR = true, addedL = 15, addedR = 18, revision = 1 } = {}) {
  const packedPwm = addedL | (addedR << 7);
  return frame(15, [0xcb, 0x25, ...le(12000, 4), ...le(revision, 4),
    mode | (requested << 2) | (result << 4) | (canChange ? 0x80 : 0),
    (active ? 1 : 0) | (loadL ? 2 : 0) | (loadR ? 4 : 0),
    packedPwm & 255, (packedPwm >>> 8) & 255], revision + 1);
}
const b1 = frame(1, [...le(0, 4), ...le(2, 4), ...le(0, 4)], 1);
const lineWord = (2 | (4 << 3) | (63 << 9)) >>> 0;
const b3 = frame(3, [...le(lineWord, 4), ...le(0, 4), ...le(0, 4)], 1);
const b2 = frame(2, [1, 0, ...le(40, 2), ...le(42, 2), ...le(41, 2), ...le(0, 2), ...le(0, 2), ...le(0, 4), ...le(0, 4)], 1);
const sdkDrive = (() => { const p = Array(45).fill(0); p[8] = 7; return frame(9, p, 2); })();
const b4Word = 3 | (1 << 13) | (1 << 14);
const b4 = frame(4, [...le(b4Word, 4), 12, 14, 1, 0, ...le(0, 4), ...le(0, 4)], 1);
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
const browser = await chromium.launch({ headless: true, executablePath: process.env.CHROME_PATH || undefined });
const page = await browser.newPage({ acceptDownloads: true, viewport: { width: 1440, height: 1000 } });
const errors = [];
page.on('pageerror', (error) => {errors.push(error.message);console.error('PAGE ERROR:',error.message);});
await page.route('**/vendor/mqtt.min.js', (route) => route.fulfill({ contentType: 'text/javascript', body: mock }));
const emit = (bytes, topic = 'is28/tlm', packet = null) => page.evaluate(([b, t, p]) => window.__m.emit(t, b, p), [Array.from(bytes), topic, packet]);
const buildFrame = async (name) => new Uint8Array(await readFile(join(releaseDir, name)));
async function waitForModeEnabled() {
  try { await page.waitForFunction(() => document.getElementById('driveMode')?.disabled === false, null, { timeout: 3000 }); }
  catch (error) {
    const state = await page.evaluate(() => ({
      driveModeDisabled: document.getElementById('driveMode')?.disabled,
      driveModeStatus: document.getElementById('driveModeStatus')?.textContent,
      profilePill: document.getElementById('profilePill')?.textContent,
      currentProfile: typeof currentProfile === 'function' ? currentProfile() : 'unavailable',
      releaseLabel: document.getElementById('releaseLabel')?.textContent
    }));
    throw new Error(`Mode selector did not enable: ${JSON.stringify(state)}; page errors: ${errors.join(' | ') || 'none'}; ${error.message}`);
  }
}

try {
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  assert.equal(await page.locator('#dashboardOverview .overview-card').count(), 6);
  assert.equal(await page.locator('#driveMode').isDisabled(), true, 'mode waits for supported car readback');
  assert.match(await page.locator('#overviewRunState').textContent(), /Unknown/);
  assert.match(await page.locator('#overviewBarcodeDetected').textContent(), /Waiting/);
  assert.match(await page.locator('#overviewImuPeak').textContent(), /No retained evidence/);
  await page.locator('#connect').click();
  await page.waitForFunction(() => window.__m.connected);

  const metadata = await buildFrame('release_metadata.bin');
  const oldMetadata = Buffer.from(metadata); oldMetadata[4] = 0x20; oldMetadata[5] = (oldMetadata[5] & 0xf0) | 4;
  await emit(oldMetadata, 'is28/tlm'); await emit(b1, 'is28/hb'); await emit(b3); await emit(b2);
  assert.equal(await page.locator('#driveMode').isDisabled(), true, 'older firmware without CB remains unsupported');
  await emit(metadata);
  await page.locator('#dashboardTabs [data-view="logs"]').click();
  await page.locator('#tlmOff').click();
  await page.locator('#dashboardTabs [data-view="telemetry"]').click();
  await emit(driveStatus({ active: true, canChange: false }));
  assert.equal(await page.locator('#driveMode').isDisabled(), true, 'run-active status blocks mode change');
  await emit(driveStatus({ active: false, canChange: true, revision: 2 }));
  await waitForModeEnabled();
  assert.equal(await page.locator('#overviewMeasured').textContent(), '41');
  assert.match(await page.locator('#overviewWheels').textContent(), /40 \/ 42 mm\/s/);
  assert.match(await page.locator('#overviewLoad').textContent(), /added PWM 15 \/ 18/);
  assert.match(await page.locator('#overviewBarcodeDetected').textContent(), /black bar/);
  assert.match(await page.locator('#overviewLineState').textContent(), /Line following/);

  await page.locator('#driveMode').selectOption('1');
  const sent = await page.evaluate(() => window.__m.sent.filter((item) => (item.bytes[0] >>> 4) === 7).at(-1).bytes);
  assert.equal(sent[2] | (sent[3] << 8), 0x0102, 'CONFIG uses mode in the high byte and key 2 in the low byte');
  assert.match(await page.locator('#driveModeStatus').textContent(), /waiting for ACK and fresh car readback/i);
  await emit(ack(7, sent[1]), 'is28/ack');
  assert.match(await page.locator('#driveModeStatus').textContent(), /CONFIG accepted; waiting/i);
  assert.doesNotMatch(await page.locator('#driveModeStatus').textContent(), /Confirmed by car.*Normal/);
  await emit(driveStatus({ mode: 1, requested: 1, canChange: true, active: false, revision: 3 }));
  await page.waitForFunction(() => document.getElementById('driveModeStatus')?.textContent.includes('Confirmed by car · Normal'));
  assert.equal(await page.locator('#driveMode').inputValue(), '1');

  await page.locator('#driveMode').selectOption('2');
  const rejected = await page.evaluate(() => window.__m.sent.filter((item) => (item.bytes[0] >>> 4) === 7).at(-1).bytes);
  await emit(ack(7, rejected[1], 4), 'is28/ack');
  await page.waitForFunction(() => document.getElementById('driveModeStatus')?.textContent.startsWith('Rejected by car:'));
  assert.equal(await page.locator('#driveMode').inputValue(), '1', 'rejected mode leaves the last car-reported mode selected');

  const c9 = await buildFrame('imu_lift.bin');
  const ca = await buildFrame('imu_lift_peak.bin');
  await emit(b4); await emit(c9); await emit(ca, 'is28/tlm', { retain: true });
  assert.match(await page.locator('#overviewImuArmed').textContent(), /Armed · fresh C9/);
  assert.match(await page.locator('#overviewImuTrip').textContent(), /lifted trip latched/i);
  assert.match(await page.locator('#overviewMotorOutput').textContent(), /No fresh same-run/);
  assert.match(await page.locator('#overviewImuPeak').textContent(), /Greatest valid tilt/);
  assert.match(await page.locator('#overviewImuEvidence').textContent(), /not a current-run label/i);

  await page.locator('#editMapping').click();
  assert.equal(await page.locator('#dashboardTabs [data-view="diagnostics"]').getAttribute('aria-selected'), 'true');
  assert.equal(await page.locator('#mapRead').isVisible(), true, 'mapping read/edit controls remain available');
  await page.locator('#dashboardTabs [data-view="logs"]').click();
  assert.equal(await page.locator('#csvSave').isVisible(), true);
  assert.equal(await page.locator('#logDownload').isVisible(), true);
  await page.locator('#dashboardTabs [data-view="telemetry"]').click();

  const desktopShot = join(tmpdir(), 'robopico-demo-overview-v025-desktop.png');
  await page.screenshot({ path: desktopShot, fullPage: false });
  await page.setViewportSize({ width: 390, height: 844 });
  assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'narrow view has no horizontal overflow');
  const narrowShot = join(tmpdir(), 'robopico-demo-overview-v025-narrow.png');
  await page.screenshot({ path: narrowShot, fullPage: true });

  await page.waitForTimeout(2750);
  assert.equal(await page.locator('#driveMode').isDisabled(), true, 'stale CB readback blocks changes');
  assert.match(await page.locator('#overviewTelemetry').textContent(), /Stale/);
  assert.match(await page.locator('#overviewCar').textContent(), /Stale/);
  assert.match(await page.locator('#overviewMotorOutput').textContent(), /No fresh/);
  await page.locator('#connect').click();
  assert.equal(await page.locator('#driveMode').isDisabled(), true, 'offline broker blocks mode changes');
  assert.match(await page.locator('#overviewBroker').textContent(), /Disconnected/);
  assert.match(await page.locator('#overviewTelemetry').textContent(), /No fresh motion frame/);
  assert.match(await page.locator('#overviewImuArmed').textContent(), /unknown/);
  await page.locator('#connect').click();
  await page.waitForFunction(() => window.__m.connected);
  await emit(metadata); await emit(sdkDrive); await emit(driveStatus({ mode: 1, requested: 3, revision: 8 }));
  assert.match(await page.locator('#profilePill').textContent(), /MIXED/);
  assert.equal(await page.locator('#driveMode').isDisabled(), true, 'genuine SDK telemetry plus RTOS C2/CB remains mixed and blocks changes');
  assert.deepEqual(errors, []);
  console.log(`PASS v0.2.5 demo overview: six live cards, old/run/stale mode gating, CONFIG ACK+CB readback, IMU evidence, tabs and mapping. Screenshots: ${desktopShot} · ${narrowShot}`);
} finally {
  await browser.close();
  await new Promise((resolve) => server.close(resolve));
}
