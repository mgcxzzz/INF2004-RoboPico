// Compat copy for the merged dashboard v0.1: original testing/subsystem1/host/check_dashboard_ui.mjs (sha256 80f983c0720351e8), changed only where marked COMPAT (paths/mock route/screenshots).
// S1-U19: exercise the actual dashboard UI with a simulated MQTT transport.
// No broker/Pico/hardware results are claimed. Requires Playwright + Chromium.
// IS28_PLAYWRIGHT_MODULE may point to a bundled Playwright package directory.
import { createRequire } from 'node:module';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const require = createRequire(import.meta.url);
const { chromium } = require(process.env.IS28_PLAYWRIGHT_MODULE || 'playwright');
const here = dirname(fileURLToPath(import.meta.url));
const vectors = JSON.parse(readFileSync((process.env.IS28_VECTORS || join(here, '../../../../testing/subsystem1/host/vectors.json')) /* COMPAT */, 'utf8'));
const bytes = (h) => Array.from(Buffer.from(h, 'hex'));
const mock = `
window.testMqtt = {
  handlers: {}, sent: [], connected: true,
  on(name, fn) { this.handlers[name] = fn; return this; },
  subscribe(topic, cb) { cb(null); },
  publish(topic, payload, options) { this.sent.push({topic, bytes: Array.from(payload), options}); },
  end() { this.connected = false; },
  emit(name, ...args) { if (this.handlers[name]) this.handlers[name](...args); }
};
window.mqtt = { connect() { window.testMqtt.connected = true;
  setTimeout(() => window.testMqtt.emit('connect'), 0); return window.testMqtt; } };
`;
let count = 0;
function check(label, ok) {
  if (!ok) throw new Error('FAIL ' + label);
  console.log('PASS ' + label); count++;
}
const browser = await chromium.launch({ headless: true, channel: process.env.IS28_BROWSER_CHANNEL || undefined });
try {
  const page = await browser.newPage({ viewport: { width: 1280, height: 1000 } });
  const errors = [];
  page.on('pageerror', (e) => errors.push(e.message));
  await page.route('**/vendor/mqtt.min.js' /* COMPAT: vendored MQTT.js */, (route) => route.fulfill({ contentType: 'application/javascript', body: mock }));
  await page.goto('file://' + join(here, '../../index.html') /* COMPAT */);
  check('UI01 four empty selectors with every A-Z option', await page.locator('[id^=mapLetter]').count() === 4 &&
    await page.locator('#mapLetter0 option').count() === 27 && await page.locator('#mapLetter0').inputValue() === '');
  await page.locator('#mapApply').click();
  check('UI02 missing selections rejected', (await page.locator('#mapStatus').innerText()).includes('four different'));
  await page.locator('#connect').click();
  await page.waitForFunction(() => window.testMqtt.sent.length > 0);
  check('UI03 subscribes then queries active map', await page.evaluate(() => testMqtt.sent[0].bytes[0] === 0xa2));
  const emitFrame = async (hex) => page.evaluate((b) => testMqtt.emit('message', 'is28/tlm', new Uint8Array(b)), bytes(hex));
  const empty = bytes(vectors.mapping_empty);
  empty[7] = await page.evaluate(() => testMqtt.sent[0].bytes[1]);
  await page.evaluate((b) => testMqtt.emit('message', 'is28/tlm', new Uint8Array(b)), empty);
  check('UI04 boot readback has no hard-coded letters', (await page.locator('#mapStatus').innerText()).includes('no mapping'));
  const assign = async (letters) => {
    for (let i = 0; i < 4; i++) await page.locator('#mapLetter' + i).selectOption(letters[i]);
  };
  await assign(['A', 'A', 'B', 'C']);
  const sentBefore = await page.evaluate(() => testMqtt.sent.length);
  await page.locator('#mapApply').click();
  check('UI05 duplicate letters never published', await page.evaluate(() => testMqtt.sent.length) === sentBefore);
  await assign(['Z', 'A', 'M', 'Q']);
  await page.locator('#mapApply').click();
  const sent = await page.evaluate(() => testMqtt.sent.at(-1));
  check('UI06 whole-map command uses five bytes and retain=false', sent.bytes.length === 5 &&
    sent.bytes[0] === 0x92 && sent.options.retain === false);
  await page.evaluate((id) => testMqtt.emit('message', 'is28/ack', new Uint8Array([0x92, id, 0, 0])), sent.bytes[1]);
  check('UI07 acceptance alone does not claim confirmed mapping',
    (await page.locator('#mapStatus').innerText()).includes('waiting for the car'));
  const active = bytes(vectors.mapping); active[7] = sent.bytes[1];
  await page.evaluate((b) => testMqtt.emit('message', 'is28/tlm', new Uint8Array(b)), active);
  check('UI08 matching readback confirms exactly the applied letters',
    (await page.locator('#mapStatus').innerText()).includes('Confirmed by car') &&
    (await page.locator('[data-map-active]').allTextContents()).join('') === 'ZAMQ');
  await page.locator('#tlmOff').click();
  await page.locator('#mapRead').click();
  const getId = await page.evaluate(() => testMqtt.sent.at(-1).bytes[1]);
  active[7] = getId;
  await page.evaluate((b) => testMqtt.emit('message', 'is28/tlm', new Uint8Array(b)), active);
  check('UI09 mapping readback still handled after telemetry-off command',
    (await page.locator('#mapStatus').innerText()).includes('Confirmed by car'));
  await emitFrame(vectors.mapping.slice(0, -2));
  check('UI10 malformed readback logged without replacing active letters',
    (await page.locator('#log').innerText()).includes('bad frame') &&
    (await page.locator('[data-map-active]').allTextContents()).join('') === 'ZAMQ');
  if (process.env.IS28_UI_SCREENSHOTS) {
    await page.screenshot({ path: join(process.env.IS28_UI_SCREENSHOTS, '2026-10-27_S1-U19_dashboard_desktop.png'), fullPage: true });
  }
  await page.setViewportSize({ width: 390, height: 844 });
  check('UI11 mobile page has no horizontal overflow', await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
  if (process.env.IS28_UI_SCREENSHOTS) {
    await page.screenshot({ path: join(process.env.IS28_UI_SCREENSHOTS, '2026-10-27_S1-U19_dashboard_mobile.png'), fullPage: true });
  }
  await page.locator('#mapApply').click();
  await page.waitForFunction(() => document.getElementById('mapStatus').textContent.includes('No matching readback'), null, { timeout: 7000 });
  check('UI12 lost readback produces unknown status, not false success',
    (await page.locator('#mapStatus').innerText()).includes('Apply status is unknown'));
  await page.locator('#connect').click();
  check('UI13 disconnect invalidates previously confirmed mapping',
    (await page.locator('[data-map-active]').allTextContents()).every((x) => x === 'unknown'));
  check('UI14 no page JavaScript errors', errors.length === 0);
  await page.locator('#testStart').click();
  await page.locator('#testStop').click();
  check('UI15 early stop is incomplete rather than PASS',
    (await page.locator('#testVerdict').innerText()).includes('INCOMPLETE'));
  await page.evaluate(() => {
    test.running = false; test.end = now(); test.start = test.end - 300001;
    test.firstHb = test.start + 100; test.lastHb = test.start + 1100;
    test.gaps = [1]; test.seq.add(1, 1); test.seq.add(1, 2);
    updateTest();
  });
  check('UI16 silent test tail fails heartbeat in the actual UI',
    /NFR-WIFI-02\s+FAIL/.test(await page.locator('#testVerdict').innerText()));
  console.log('\n' + count + ' passed, 0 failed (simulated MQTT; no Pico)');
} finally { await browser.close(); }
