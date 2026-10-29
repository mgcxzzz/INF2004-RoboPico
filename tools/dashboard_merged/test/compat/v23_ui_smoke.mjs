// Compat copy for the merged dashboard v0.1: original interface_wifi_v2_3/dashboard/ui_smoke.mjs (sha256 977029ba24c8ede3), changed only where marked COMPAT (paths/mock route/screenshots).
import { createRequire } from 'node:module';
import { createServer } from 'node:http';
import { readFile, mkdir, writeFile } from 'node:fs/promises';
import { resolve, extname, join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const require = createRequire(import.meta.url);
const playwrightPath=process.env.PLAYWRIGHT_NODE_MODULE_PATH || 'playwright' /* COMPAT default */;
const { chromium } = require(playwrightPath);
const browserArg=process.argv.find((arg)=>arg.startsWith('--browser='));
const chromePath=browserArg ? browserArg.slice('--browser='.length) :
  (process.env.CHROME_PATH || undefined) /* COMPAT default */;
const root = resolve(dirname(fileURLToPath(import.meta.url)), '../..'); // COMPAT: serve the merged dashboard
const reviewDir = resolve((await import('node:os')).tmpdir(), 'is28-merged-v23-smoke'); // COMPAT: temp dir
const assets = new Map([
  ['/', ['index.html', 'text/html; charset=utf-8']],
  ['/index.html', ['index.html', 'text/html; charset=utf-8']],
  ['/is28_schema.js', ['is28_schema.js', 'text/javascript; charset=utf-8']],
  ['/vendor/mqtt.min.js', ['vendor/mqtt.min.js', 'text/javascript; charset=utf-8']]
]);
const server = createServer(async (req, res) => {
  try {
    const path = new URL(req.url, 'http://localhost').pathname;
    if (path === '/favicon.ico') { res.writeHead(204).end(); return; }
    const item = assets.get(path);
    if (!item) { res.writeHead(404).end('not found'); return; }
    const body = await readFile(join(root, item[0]));
    res.writeHead(200, { 'Content-Type': item[1], 'Content-Length': body.length }); res.end(body);
  } catch (error) { res.writeHead(500).end(String(error)); }
});
let browserRef=null, contextRef=null;

function mockMqttSource() {
  return `(() => {
    const state = window.__mqttMock = { client:null, sequence:0, currentMap:['A','B','C','D'],
      logMode:'retry-success', logRequests:[], attempts:{}, emitted:0 };
    const le16=(b,i,n)=>{b[i]=n&255;b[i+1]=(n>>>8)&255};
    const le32=(b,i,n)=>{b[i]=n&255;b[i+1]=(n>>>8)&255;b[i+2]=(n>>>16)&255;b[i+3]=(n>>>24)&255};
    function frame(src,p){return Uint8Array.from([(src<<4)|2,p.length,(state.sequence++)&255,...p]);}
    function emit(topic,b){state.emitted++; const c=state.client; if(c&&c.handlers.message)c.handlers.message(topic,b);}
    function ack(code,id,result=0){emit('is28/ack',Uint8Array.from([(code<<4)|2,id,result,0]));}
    function mapFrame(id){let w=(1<<20)|(73<<21); for(let i=0;i<4;i++) w|=(state.currentMap[i].charCodeAt(0)-65)<<(5*i);
      const p=new Array(5).fill(0); le32(p,0,w);p[4]=id; emit('is28/tlm',frame(8,p)); }
    state.mode=7;
    function navStatus(){const p=new Array(23).fill(0);le32(p,0,6000);p[4]=0;p[18]=1;p[19]=state.mode;p[20]=0;emit('is28/tlm',frame(11,p));}
    function chunk(ds,id,row,total,part,count,status,bytes){const p=new Array(12+bytes.length).fill(0);p[0]=ds;p[1]=id;le16(p,2,row);le16(p,4,total);p[6]=part;p[7]=count;p[8]=status;p[9]=bytes.length;bytes.forEach((v,i)=>p[12+i]=v);emit('is28/tlm',frame(13,p));}
    function source1(){const p=new Array(12).fill(0);let w0=2|(1<<5)|(7<<9)|(6000<<12);le32(p,0,w0);le32(p,4,20);le32(p,8,0);emit('is28/hb',frame(1,p));}
    function handleCommand(bytes){const code=bytes[0]>>4,id=bytes[1];
      if(code===10){ack(code,id);setTimeout(()=>mapFrame(id),5);return;}
      if(code===9){ack(code,id);setTimeout(()=>mapFrame(id),5);return;}
      if(code===11){const arg=bytes[2]|(bytes[3]<<8),ds=(arg>>12)&15,row=arg&4095;state.logRequests.push({dataset:ds,row,id});
        if(state.logMode==='reject'){ack(code,id);setTimeout(()=>chunk(ds,id,row,1,0,1,3,[]),5);return;}
        ack(code,id);const key=ds+':'+row+':'+id;state.attempts[key]=(state.attempts[key]||0)+1;
        const bytesOut=ds===0?state.motionRow():Array.from({length:ds===1?58:ds===2?10:ds===3?31:48},(_,i)=>(i*17+9)&255);
        if(state.logMode==='retry-success' && state.attempts[key]<=3){setTimeout(()=>chunk(ds,id,row,1,0,2,0,bytesOut.slice(0,49)),8);return;}
        const split=bytesOut.length>49?49:bytesOut.length;
        setTimeout(()=>{chunk(ds,id,row,1,0,bytesOut.length>49?2:1,0,bytesOut.slice(0,split));if(bytesOut.length>49)setTimeout(()=>chunk(ds,id,row,1,1,2,0,bytesOut.slice(split)),8);},5);return;
      }
      ack(code,id,0);
    }
    window.mqtt={connect(){const c={connected:false,handlers:{},on(n,fn){this.handlers[n]=fn;return this;},
      subscribe(_topic,cb){setTimeout(()=>cb&&cb(null),0);return this;},publish(_topic,payload,_options){handleCommand(Array.from(payload));},
      end(){this.connected=false;this.handlers.close&&this.handlers.close();}};state.client=c;
      setTimeout(()=>{c.connected=true;c.handlers.connect&&c.handlers.connect();source1();navStatus();state.statusTimer=setInterval(()=>{source1();navStatus();},250);},0);return c;}};
    state.emit=emit;state.frame=frame;state.navStatus=navStatus;state.setMode=function(mode){state.mode=mode;navStatus();};state.source1=source1;
    state.motion=function(flags=7,uptime=123456){const p=new Array(45).fill(0);le32(p,0,uptime);le32(p,4,6500);le16(p,8,12);p[10]=4;p[11]=1;p[12]=flags;p[13]=0;
      le16(p,14,120);le16(p,16,130);le16(p,18,140);le16(p,20,150);le16(p,22,30);le16(p,24,32);
      le16(p,26,400);le16(p,28,410);le16(p,30,420);le16(p,32,321);le16(p,34,654);le16(p,36,65535);p[38]=5;le32(p,39,54321);le16(p,43,1234);emit('is28/tlm',frame(9,p));};
    state.motionRow=function(){const p=new Array(45).fill(0);le32(p,0,1234);le32(p,4,456);le16(p,8,3);p[10]=4;p[11]=1;p[12]=128;p[13]=0;
      le16(p,14,120);le16(p,16,130);le16(p,18,140);le16(p,20,150);le16(p,22,30);le16(p,24,32);
      le16(p,26,400);le16(p,28,410);le16(p,30,420);le16(p,32,0);le16(p,34,65535);le16(p,36,65535);p[38]=7;le32(p,39,678);le16(p,43,1234);
      return Array.from(frame(9,p));};
    state.diagnostics=function(){const p=new Array(60).fill(0);le32(p,0,1200);le16(p,4,1700);p[6]=17;p[7]=15;
      for(let i=0;i<13;i++)le32(p,8+i*4,100+i);emit('is28/tlm',frame(14,p));};
    state.line=function(){const p=new Array(21).fill(0);le32(p,0,123456);p[4]=2;le16(p,5,410);le16(p,7,420);le16(p,9,430);le16(p,11,65535);le16(p,13,200);p[15]=75;p[16]=65;p[17]=82;p[18]=73;p[19]=85;p[20]=72;emit('is28/tlm',frame(10,p));};
  })();`;
}

async function main() {
  await mkdir(reviewDir, { recursive:true });
  await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
  const address=server.address(), url=`http://127.0.0.1:${address.port}/index.html`;
  const browser=browserRef=await chromium.launch({headless:true,executablePath:chromePath});
  const context=contextRef=await browser.newContext({acceptDownloads:true,viewport:{width:1440,height:1000}});
  const page=await context.newPage(); const errors=[]; page.on('pageerror',(e)=>errors.push(String(e))); page.on('console',(m)=>{if(m.type()==='error')errors.push(m.text());});
  await page.route('**/vendor/mqtt.min.js',route=>route.fulfill({status:200,contentType:'text/javascript',body:mockMqttSource()}));
  const files=[]; page.on('download',d=>files.push(d));
  await page.goto(url); await page.locator('#connect').click();
  await page.waitForFunction(()=>window.__mqttMock?.client?.connected);
  await page.waitForFunction(()=>document.querySelector('#carlink')?.textContent==='car: online');
  await page.waitForFunction(()=>document.querySelector('#mapStatus')?.textContent.includes('Confirmed by car'));
  await page.evaluate(()=>{window.__mqttMock.motion(5);window.__mqttMock.diagnostics();});
  await page.waitForFunction(()=>document.querySelector('#diagCounts')?.textContent==='100 / 101 / 102 / 103 / 104');
  assert.equal(await page.locator('#diagGaps').textContent(),'107 / 108 µs');
  await page.waitForFunction(()=>document.querySelector('#levels')?.textContent.startsWith('unavailable / unavailable / unavailable'));
  await page.evaluate(()=>{window.__mqttMock.motion(7);window.__mqttMock.line();});
  await page.waitForFunction(()=>document.querySelector('#speedMeasured')?.textContent==='12 / 13 mm/s');
  assert.equal(await page.locator('#levels').textContent(),'321 / 654 / unavailable · mask 0x5');
  assert.match(await page.locator('#eventTimeline').textContent(),/calibrated L\/R unavailable\/200/);
  assert.equal(await page.locator('#barcodeMovement').textContent(),'unassigned');
  assert.equal(await page.locator('#pendingAction').textContent(),'A → Right');
  await page.evaluate(()=>{for(let i=0;i<260;i++)window.__mqttMock.motion(7,200000+i*50);});
  await page.waitForFunction(()=>document.querySelector('#chartSampleCount')?.textContent==='240 / 240 chart samples');
  assert.match(await page.locator('#archiveStatus').textContent(),/262\/72000 motion samples/);
  await page.locator('#csvSave').click();
  await page.waitForFunction(()=>window.__mqttMock.emitted>=264);
  const captureDownload=files.at(-1),capturePath=join(reviewDir,'capture-over-240.csv');
  await captureDownload.saveAs(capturePath);
  const captureCsv=await readFile(capturePath,'utf8');
  assert.ok(captureCsv.split(/\r\n/).length-2>240,'live CSV contains the full retained archive, not just the chart window');
  assert.match(captureCsv,/adc2_right_gp28/);
  const desktopPath=null; // COMPAT: no screenshot

  for (const [i,ch] of ['Q','R','S','T'].entries()) await page.locator(`#mapLetter${i}`).selectOption(ch);
  await page.locator('#mapApply').click();
  await page.waitForFunction(()=>document.querySelector('#mapStatus')?.textContent.includes('does not match'));
  await page.locator('#mapRead').click();
  await page.waitForFunction(()=>document.querySelector('#mapStatus')?.textContent.includes('Confirmed by car'));

  await page.evaluate(()=>window.__mqttMock.setMode(2));
  assert.equal(await page.locator('#logDownload').isDisabled(),true,'Paused mode must not allow stored-log access');
  await page.evaluate(()=>window.__mqttMock.setMode(7));
  await page.waitForFunction(()=>document.querySelector('#logDownload')?.disabled===false,null,{timeout:10000});
  await page.locator('#logDataset').selectOption('1');
  await page.locator('#logDownload').click();
  await page.waitForFunction(()=>{const s=document.querySelector('#logDownloadStatus')?.textContent||'';return /^(Complete:|Firmware rejected|Row \d+ incomplete|Coordinator left|Broker disconnected)/.test(s);},null,{timeout:20000});
  const downloadStatus=await page.locator('#logDownloadStatus').textContent();
  assert.ok(downloadStatus.startsWith('Complete:'),downloadStatus);
  assert.equal(await page.evaluate(()=>window.__mqttMock.logRequests.length),4);
  const reqs=await page.evaluate(()=>window.__mqttMock.logRequests);
  assert.ok(reqs.every(r=>r.id===reqs[0].id),'all retry chunks match the same request id');
  assert.equal(files.length,4,'stored-log success downloads .bin, .csv, and .json in addition to live capture CSV');
  const saved=[]; for (const [i,d] of files.entries()) {const path=join(reviewDir,`download-${i}-${d.suggestedFilename()}`);await d.saveAs(path);saved.push(path);}
  const raw=await readFile(saved.find(f=>f.endsWith('.bin'))); assert.equal(raw.length,58);
  assert.deepEqual([...raw],Array.from({length:58},(_,i)=>(i*17+9)&255));
  const json=JSON.parse(await readFile(saved.find(f=>f.endsWith('.json')),'utf8')); assert.equal(json.rowCount,1); assert.equal(json.rowBytes,58);
  await page.evaluate(()=>{window.__mqttMock.logMode='reject';});
  await page.locator('#logDataset').selectOption('2'); await page.locator('#logDownload').click();
  await page.waitForFunction(()=>document.querySelector('#logDownloadStatus')?.textContent.includes('rejected'),{timeout:3000});
  assert.equal(files.length,4,'rejected response must not create partial export files');
  await page.evaluate(()=>{window.__mqttMock.logMode='retry-success';});
  await page.locator('#logDataset').selectOption('0'); await page.locator('#logDownload').click();
  await page.waitForFunction(()=>{const s=document.querySelector('#logDownloadStatus')?.textContent||'';return /^(Complete:|Firmware rejected|Row \d+ incomplete|Coordinator left|Broker disconnected)/.test(s);},null,{timeout:15000});
  const motionDownloadStatus=await page.locator('#logDownloadStatus').textContent();
  assert.ok(motionDownloadStatus.startsWith('Complete:'),motionDownloadStatus);
  assert.equal(files.length,7,'motion download yields complete binary, CSV, and manifest');
  const motionCsvDownload=files.find(d=>d.suggestedFilename().endsWith('.csv')&&d.suggestedFilename().includes('_motion_'));
  const motionCsvPath=join(reviewDir,'download-motion.csv'); await motionCsvDownload.saveAs(motionCsvPath);
  const motionCsv=await readFile(motionCsvPath,'utf8');
  assert.match(motionCsv,/run-relative/); assert.match(motionCsv,/unavailable/); assert.match(motionCsv,/calibration_valid/);

  const mobile=await context.newPage(); await mobile.setViewportSize({width:390,height:844});
  await mobile.route('**/vendor/mqtt.min.js',route=>route.fulfill({status:200,contentType:'text/javascript',body:mockMqttSource()}));
  mobile.on('pageerror',(e)=>errors.push(String(e))); mobile.on('console',(m)=>{if(m.type()==='error')errors.push(m.text());});
  await mobile.goto(url); await mobile.locator('#connect').click();
  await mobile.waitForFunction(()=>window.__mqttMock?.client?.connected);
  await mobile.waitForFunction(()=>document.querySelector('#carlink')?.textContent==='car: online');
  await mobile.evaluate(()=>window.__mqttMock.motion(7));
  await mobile.waitForFunction(()=>document.querySelector('#speedMeasured')?.textContent==='12 / 13 mm/s');
  const mobilePath=null; // COMPAT: no screenshot
  assert.equal(await mobile.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth),true,'mobile layout has no horizontal page overflow');
  const bundlePage=await context.newPage();
  bundlePage.on('pageerror',(e)=>errors.push(String(e))); bundlePage.on('console',(m)=>{if(m.type()==='error')errors.push(m.text());});
  await bundlePage.goto(url); await bundlePage.waitForFunction(()=>typeof window.mqtt?.connect==='function');
  await bundlePage.close();
  assert.deepEqual(errors,[],'no browser console or page errors');
  console.log(JSON.stringify({desktopPath,mobilePath,downloads:saved,motionCsvPath,logRetries:reqs.length,errors},null,2));
  await context.close(); await browser.close();
  await new Promise((ok)=>server.close(ok));
}

main().catch(async (error)=>{console.error(error);try{await contextRef?.close();await browserRef?.close();}catch{}try{server.close();}catch{}process.exitCode=1;});
