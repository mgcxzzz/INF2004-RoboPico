/* Local browser with simulated MQTT. No broker/car commands are sent. */
import {createRequire} from 'node:module';
import {readFile,mkdtemp} from 'node:fs/promises';
import {createServer} from 'node:http';
import {tmpdir} from 'node:os';
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
 if(file==='favicon.ico'){res.writeHead(204).end();return;}
 if(!['index.html','is28_schema.js','vendor/mqtt.min.js'].includes(file)){res.writeHead(404).end();return;}
 res.writeHead(200,{'Content-Type':file.endsWith('.html')?'text/html; charset=utf-8':'text/javascript; charset=utf-8'});
 res.end(await readFile(join(root,file)));
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));
const browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_PATH||undefined});
const page=await browser.newPage({acceptDownloads:true,viewport:{width:1400,height:1000}});
const errors=[];page.on('pageerror',e=>errors.push(e.message));
await page.route('**/vendor/mqtt.min.js',r=>r.fulfill({contentType:'text/javascript',body:mock}));
const emit=(bytes,topic='is28/tlm')=>page.evaluate(([b,t])=>window.__m.emit(t,b),[Array.from(bytes),topic]);
const base=resolve(root,'../testing/rtos_migration/build-host');
const metadata=await readFile(join(base,'release_metadata.bin'));
const runtime=await readFile(join(base,'release_runtime.bin'));
const row=await readFile(join(base,'release_line.bin'));
const le=(v,n)=>Array.from({length:n},(_,i)=>(v>>>(8*i))&255);
const frame=(src,p,seq)=>[(src<<4)|2,p.length,seq,...p];
const motion=frame(9,Array(45).fill(0),1);motion[14]=1;motion[15]=0x76;
const nav=frame(11,[...le(5000,4),0,0,0,0,...le(0,4),...le(1,4),0,0,0,7,0,0,0],1);
try {
 await page.goto(`http://127.0.0.1:${server.address().port}/`);
 await page.locator('#connect').click();await page.waitForFunction(()=>window.__m.connected);
 await emit(motion);await emit(nav);await emit(metadata);await emit(runtime);
 assert.match(await page.locator('#profilePill').textContent(),/v0.2.0/);
 assert.equal(await page.locator('#rtosRuntimeCard').isVisible(),true);
 assert.match(await page.locator('#releaseCal').textContent(),/122\/138/);
 for(let k=0;k<11;k++){await page.waitForTimeout(500);await emit(metadata);nav[2]++;await emit(nav);}
 await page.locator('#logDataset').selectOption('1');
 await page.waitForFunction(()=>!document.getElementById('logDownload').disabled);
 const downloads=[];page.on('download',d=>downloads.push(d));
 await page.locator('#logDownload').click();
 const sent=await page.evaluate(()=>window.__m.sent.filter(x=>(x.bytes[0]>>4)===11).at(-1).bytes);
 const id=sent[1];
 for(let part=0;part<2;part++) {
  const bytes=Array.from(row.subarray(49*part,49*(part+1)));
  await emit(frame(13,[1,id,0,0,1,0,part,2,0,bytes.length,0,0,...bytes],part+1));
 }
 await page.waitForFunction(()=>document.getElementById('logDownloadStatus').textContent.startsWith('Complete:'));
 assert.equal(downloads.length,3);
 const out=await mkdtemp(join(tmpdir(),'rtos-release-ui-'));
 const got={};for(const d of downloads){const file=join(out,d.suggestedFilename());await d.saveAs(file);got[d.suggestedFilename().split('.').at(-1)]=await readFile(file);}
 const meta=JSON.parse(got.json.toString());assert.equal(meta.rowBytes,56);assert.equal(meta.format,'rtos-controller-v1');assert.equal(meta.release.version,'0.2.0');
 assert.match(got.csv.toString(),/sequence,ms,ticks_l,ticks_r/);assert.match(got.csv.toString(),/42,123,-100,100/);
 assert.deepEqual(got.bin,row);
 assert.deepEqual(errors,[]);
 console.log('PASS production browser: release/profile/calibration/runtime display, stopped wireless two-part download, binary/CSV/JSON formats; no page errors.');
} finally {await browser.close();await new Promise(r=>server.close(r));}
