/* Production dashboard with simulated MQTT, no real broker/robot commands. */
import {createRequire} from 'node:module';
import {readFile,mkdir} from 'node:fs/promises';
import {createServer} from 'node:http';
import {resolve,join,dirname} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const require=createRequire(import.meta.url);
const {chromium}=require(process.env.PLAYWRIGHT_NODE_MODULE_PATH||'playwright');
const root=resolve(dirname(fileURLToPath(import.meta.url)),'..');
const prior=await readFile(join(root,'test/ui_merged.mjs'),'utf8');
const mock=prior.split('const mock = `')[1].split('`;')[0];
const server=createServer(async(req,res)=>{
 const file=req.url==='/'?'index.html':req.url.slice(1);
 if(!['index.html','is28_schema.js','calibration_ui.js','vendor/mqtt.min.js'].includes(file)){res.writeHead(404).end();return;}
 res.writeHead(200,{'Content-Type':file.endsWith('.html')?'text/html; charset=utf-8':'text/javascript'});
 res.end(await readFile(join(root,file)));
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_PATH||undefined});
const page=await browser.newPage({viewport:{width:1400,height:1000}});
const errors=[];page.on('pageerror',e=>errors.push(e.message));
await page.route('**/vendor/mqtt.min.js',r=>r.fulfill({contentType:'text/javascript',body:mock}));
const emit=(b,t='is28/tlm')=>page.evaluate(([b,t])=>window.__m.emit(t,b),[Array.from(b),t]);
const fixture=await readFile(resolve(root,'../testing/rtos_migration/build-host/front_calibration.bin'));
let seq=0;
const packet=({op=0,id=0,result=0,flags=12,captured=0,capturing=0,samples=0}={})=>{
 const b=Buffer.from(fixture);b[2]=seq++;b[11]=id;b[12]=op|(result<<4);b[13]=flags;b[14]=captured|(capturing<<3);b[15]=samples;return b;
};
const button=op=>page.locator(`[data-cal-op="${op}"]`);
const click=async op=>{await button(op).click();return page.evaluate(()=>window.__m.sent.filter(x=>(x.bytes[0]>>4)===7&&x.bytes[2]===3).at(-1).bytes[1]);};
try {
 await page.goto(`http://127.0.0.1:${server.address().port}/`);
 await page.locator('#connect').click();await page.waitForFunction(()=>window.__m.connected);
 await page.locator('#dashboardTabs [data-view="diagnostics"]').click();
 assert.equal(await button(1).isDisabled(),true);
 assert.equal(await page.evaluate(()=>window.__m.sent.some(x=>x.bytes[2]===3)),false,'no automatic calibration command');
 await emit(packet({flags:4}));assert.equal(await button(1).isDisabled(),true,'motion disables changes');
 await emit(packet());assert.equal(await button(1).isDisabled(),false);
 let id=await click(1);await emit([0x72,id,0,0],'is28/ack');
 assert.match(await page.locator('#calNotice').textContent(),/Acceptance alone/);
 assert.equal(await button(2).isDisabled(),true,'ACK alone cannot open the session');
 await emit(packet({op:1,id,result:1,flags:13}));
 assert.equal(await button(2).isDisabled(),false);assert.equal(await button(3).isDisabled(),true);
 id=await click(2);await emit(packet({op:2,id,result:2,flags:13,capturing:1,samples:12}));
 assert.equal(await button(6).isDisabled(),true);assert.equal(await button(7).isDisabled(),false);
 await emit(packet({op:2,id,result:3,flags:13,captured:1,samples:32}));
 assert.equal(await button(3).isDisabled(),false);assert.equal(await button(6).isDisabled(),true);
 await emit(packet({op:4,id,result:3,flags:15,captured:7,samples:32}));
 assert.equal(await button(6).isDisabled(),false);
 id=await click(6);await emit([0x72,id,0,0],'is28/ack');
 assert.doesNotMatch(await page.locator('#calStatus').textContent(),/Saved/);
 await emit(packet({op:6,id,result:12,flags:15,captured:7,samples:32}));
 assert.match(await page.locator('#calStatus').textContent(),/verification failed/);
 id=await click(6);await emit(packet({op:6,id,result:5,flags:62,captured:7,samples:32}));
 assert.match(await page.locator('#calStatus').textContent(),/Saved and verified/);
 assert.match(await page.locator('#calActive').textContent(),/120 \/ 180/);
 /* Reopen staging for the visual check, no real commands are sent. */
 await emit(packet({op:4,result:3,flags:15,captured:7,samples:32}));
 const metadata=await readFile(resolve(root,'../testing/rtos_migration/build-host/release_metadata.bin'));
 await emit(metadata);
 await page.evaluate(()=>{
   const tag=document.createElement('span');tag.className='pill';
   tag.textContent='PREVIEW - SIMULATED DATA';document.querySelector('header').append(tag);
 });
 const output=resolve(root,'../testing/rtos_migration/build-host');await mkdir(output,{recursive:true});
 await page.evaluate(()=>window.scrollTo(0,0));await page.waitForTimeout(150);
 await page.screenshot({path:join(output,'wireless_calibration_full_page.png'),fullPage:true});
 await page.locator('#frontCalibration').scrollIntoViewIfNeeded();
 const out=resolve(root,'../testing/rtos_migration/build-host');await mkdir(out,{recursive:true});
 await page.locator('#frontCalibration').screenshot({path:join(out,'wireless_calibration_desktop.png')});
 await page.setViewportSize({width:390,height:900});
 await page.locator('#frontCalibration').screenshot({path:join(out,'wireless_calibration_mobile.png')});
 assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>window.innerWidth),false,'no horizontal page overflow');
 await page.waitForTimeout(2700);assert.equal(await button(6).isDisabled(),true,'stale status disables changes');
 await emit(packet({op:4,result:3,flags:15,captured:7}));
 await page.locator('#connect').click();assert.equal(await button(6).isDisabled(),true,'disconnect invalidates staging view');
 assert.deepEqual(errors,[]);
 console.log('PASS browser calibration: stopped/fresh gates, staging, ACK/readback, capture, verified save/failure, stale/disconnect and responsive layout');
} finally {await browser.close();await new Promise(r=>server.close(r));}
