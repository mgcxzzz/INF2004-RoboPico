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
const fault=await readFile(join(base,'encoder_fault.bin'));
const faultClear=await readFile(join(base,'encoder_fault_clear.bin'));
const barcode=await readFile(join(base,'barcode_attempt.bin'));
const networkStop=await readFile(join(base,'network_stop.bin'));
const le=(v,n)=>Array.from({length:n},(_,i)=>(v>>>(8*i))&255);
const frame=(src,p,seq)=>[(src<<4)|2,p.length,seq,...p];
const c5=(attempt,code,bars,seq,flags=1|4|16,reason=9)=>{
 const meta=(flags|(code<<6)|(reason<<11)|(bars<<15)|(144<<20))>>>0;
 return frame(15,[0xc5,...le(9000,4),...le(attempt,4),...le(8700,4),...le(8950,4),...le(3,4),...le(7,2),...le(meta,4)],seq);
};
const c6Clear=(seq)=>frame(15,[0xc6,0x21,...Array(27).fill(0)],seq);
const ack=(code,id,result=0,state=0)=>[(code<<4)|2,id,result,state];
const motion=frame(9,Array(45).fill(0),1);motion[14]=1;motion[15]=0x76;
const nav=frame(11,[...le(5000,4),0,0,0,0,...le(0,4),...le(1,4),0,0,0,7,0,0,0],1);
try {
 await page.goto(`http://127.0.0.1:${server.address().port}/`);
 await page.locator('#connect').click();await page.waitForFunction(()=>window.__m.connected);
 await emit(motion);await emit(nav);await emit(metadata);await emit(runtime);
 const releaseVersion=`${metadata[4]&15}.${metadata[4]>>>4}.${metadata[5]&15}`;
 assert.match(await page.locator('#profilePill').textContent(),new RegExp(`v${releaseVersion.replaceAll('.','\\.')}`));
 assert.equal(await page.locator('#rtosRuntimeCard').isVisible(),true);
 assert.match(await page.locator('#releaseCal').textContent(),/122\/138/);
 await emit(fault);
 assert.equal(await page.locator('#encoderFault').textContent(),'Right encoder: error density · latched');
 assert.match(await page.locator('#encoderFaultDetails').textContent(),/Frozen target L\/R -45\/70 mm\/s/);
 await emit(networkStop);
 assert.match(await page.locator('#networkStop').textContent(),/Last recorded network stop/);
 assert.match(await page.locator('#networkStopDetails').textContent(),/no service poll recorded/);
 assert.match(await page.locator('#networkStopDetails').textContent(),/TCP ACK age 3001 ms/);
 await emit(barcode);
 assert.match(await page.locator('#barcodeCapture').textContent(),/code F.*14 bars/);
 assert.match(await page.locator('#barcodeCapture').textContent(),/truncated/);
 await page.locator('button[data-cmd="START"]').click();
 const start=await page.evaluate(()=>window.__m.sent.filter(x=>(x.bytes[0]>>4)===1).at(-1).bytes);
 await emit(ack(1,start[1],0,1),'is28/ack');
 assert.match(await page.locator('#encoderFault').textContent(),/latched/);
 assert.match(await page.locator('#networkStop').textContent(),/latched/);
 await emit(faultClear);await emit(c6Clear(92));
 assert.match(await page.locator('#encoderFault').textContent(),/explicit firmware clear/);
 assert.match(await page.locator('#networkStop').textContent(),/explicit firmware clear/);
 await emit(fault);await emit(networkStop);
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
 const meta=JSON.parse(got.json.toString());assert.equal(meta.rowBytes,56);assert.equal(meta.format,'rtos-controller-v1');assert.equal(meta.release.version,releaseVersion);
 assert.match(got.csv.toString(),/sequence,ms,ticks_l,ticks_r/);assert.match(got.csv.toString(),/42,123,-100,100/);
 assert.deepEqual(got.bin,row);
 /* Raw export freezes the latest C5 metadata at request time, even if a new
    attempt is reported while the row chunks are still arriving. */
 const release021=Buffer.from(metadata);release021[4]=0x20;release021[5]=(release021[5]&0xf0)|1;
 await emit(release021);
 await page.locator('button[data-cmd="START"]').click();
 const rejectedStart=await page.evaluate(()=>window.__m.sent.filter(x=>(x.bytes[0]>>4)===1).at(-1).bytes);
 await emit(ack(1,rejectedStart[1],4,3),'is28/ack');
 assert.match(await page.locator('#lastack').textContent(),/check fault, clocks, network supervision and transfer readiness/);
 assert.match(await page.locator('#cmdNote').textContent(),/Check the latched encoder fault, clock status, Wi-Fi\/network connection, and transfer readiness/);
 await emit(c5(55,0,14,94,1|4|16,9));
 assert.match(await page.locator('#barcodeCapture').textContent(),/no known code.*14 bars/);
 assert.doesNotMatch(await page.locator('#barcodeCapture').textContent(),/code F/);
 await page.locator('#logDataset').selectOption('2');
 downloads.length=0;
 await page.locator('#logDownload').click();
 const rawSent=await page.evaluate(()=>window.__m.sent.filter(x=>(x.bytes[0]>>4)===11).at(-1).bytes);
 const rawId=rawSent[1];
 await emit(c5(56,6,10,95,1|2|4|16,0));
 await emit(ack(11,rawId,0,0),'is28/ack');
 const ackStatus=await page.locator('#logDownloadStatus').textContent();
 assert.match(ackStatus,/GET_LOG accepted.*row 1/);assert.doesNotMatch(ackStatus,/row 0/);
 const rawRow=Buffer.from([1,2,3,4,5,6,7,8,9,10]);
 await emit(frame(13,[2,rawId,0,0,1,0,0,1,0,10,0,0,...rawRow],96));
 await page.waitForFunction(()=>document.getElementById('logDownloadStatus').textContent.startsWith('Complete:'));
 assert.equal(downloads.length,3);
 const rawGot={};for(const d of downloads){const file=join(out,d.suggestedFilename());await d.saveAs(file);rawGot[d.suggestedFilename().split('.').at(-1)]=await readFile(file);}
 const rawMeta=JSON.parse(rawGot.json.toString());
 assert.equal(rawMeta.rowBytes,10);assert.equal(rawMeta.release.version,'0.2.1');
 assert.equal(rawMeta.barcodeCapture.attemptSequence,55);assert.equal(rawMeta.barcodeCapture.code,'?');
 assert.equal(rawMeta.barcodeCapture.bars,14);assert.match(rawMeta.barcodeCaptureNote,/last reported attempt/);
 assert.equal(rawMeta.barcodeCaptureMatchesRows,false,'stale metadata must not be presented as a verified row association');
 assert.match(rawMeta.barcodeCaptureNote,/association is unverified/);
 assert.equal(await page.evaluate(b=>barcodeCaptureMatchesRows({valid:true,runId:7,rawCount:1,firstRawMs:0x04030201,lastRawMs:0x04030201},{runId:7},[Uint8Array.from(b)]),Array.from(rawRow)),true);
 assert.equal(await page.locator('#barcodeCapture').textContent().then(s=>s.includes('Attempt 56')),true);
 assert.deepEqual(rawGot.bin,rawRow);
 await page.evaluate(()=>invalidateLive('simulated stale telemetry'));
 assert.match(await page.locator('#encoderFault').textContent(),/latched/);
 assert.match(await page.locator('#networkStop').textContent(),/latched/);
 assert.deepEqual(errors,[]);
 console.log('PASS production browser: release/profile/calibration/runtime display, stopped wireless two-part download, binary/CSV/JSON formats; no page errors.');
} finally {await browser.close();await new Promise(r=>server.close(r));}
