import { createRequire } from 'node:module';
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { join, dirname } from 'node:path';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const require=createRequire(import.meta.url);
const playwrightPath=process.env.PLAYWRIGHT_NODE_MODULE_PATH||'C:/Users/nguic/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright';
const {chromium}=require(playwrightPath);
const browserArg=process.argv.find(a=>a.startsWith('--browser='));
const chromePath=browserArg?browserArg.slice(10):(process.env.CHROME_PATH||'C:/Program Files/Google/Chrome/Application/chrome.exe');
const root=dirname(fileURLToPath(import.meta.url));
const review=join(process.env.DASHBOARD_SMOKE_OUT||tmpdir(),'interface-utk-v1-dashboard-smoke');
const assets=new Map([['/',['index.html','text/html']],['/index.html',['index.html','text/html']],['/is28_schema.js',['is28_schema.js','text/javascript']],['/vendor/mqtt.min.js',['vendor/mqtt.min.js','text/javascript']]]);
const server=createServer(async(req,res)=>{try{const path=new URL(req.url,'http://localhost').pathname;if(path==='/favicon.ico'){res.writeHead(204).end();return;}const item=assets.get(path);if(!item){res.writeHead(404).end();return;}const body=await readFile(join(root,item[0]));res.writeHead(200,{'Content-Type':item[1]});res.end(body);}catch(e){res.writeHead(500).end(String(e));}});
function mockSource(){return `(()=>{
 const s=window.__mqttMock={client:null,seq:{},published:[],map:['A','B','C','D'],mismatchNext:true};
 const le32=(p,i,n)=>{p[i]=n&255;p[i+1]=(n>>>8)&255;p[i+2]=(n>>>16)&255;p[i+3]=(n>>>24)&255;};
 function frame(src,p){const seq=s.seq[src]||0;s.seq[src]=(seq+1)&255;return Uint8Array.from([(src<<4)|2,p.length,seq,...p]);}
 function emit(topic,b){s.client?.handlers.message?.(topic,b);}
 function ack(code,id,result=0,state=1){setTimeout(()=>emit('is28/ack',Uint8Array.from([(code<<4)|2,id,result,state])),0);}
 function hb(){const p=new Array(12).fill(0);le32(p,0,2|(1<<2)|(1<<5)|(1<<9)|(75<<12));le32(p,4,29|(2<<16));le32(p,8,3|(1<<8)|(2<<16)|(17<<24));emit('is28/hb',frame(1,p));}
 function mapFrame(id){let w=(1<<20)|(7<<21);for(let i=0;i<4;i++)w|=(s.map[i].charCodeAt(0)-65)<<(i*5);const p=[w&255,(w>>>8)&255,(w>>>16)&255,(w>>>24)&255,id];emit('is28/tlm',frame(8,p));}
 function publish(bytes){const b=Array.from(bytes);s.published.push(b);const code=b[0]>>4,id=b[1];if(code===10){ack(code,id);setTimeout(()=>mapFrame(id),2);return;}if(code===9){ack(code,id);if(s.mismatchNext){s.mismatchNext=false;setTimeout(()=>{mapFrame(id);s.map=['Q','R','S','T'];},2);}else{s.map=['Q','R','S','T'];setTimeout(()=>mapFrame(id),2);}return;}ack(code,id);}
 window.mqtt={connect(){const c={connected:false,handlers:{},on(n,fn){this.handlers[n]=fn;return this;},subscribe(_t,cb){cb?.(null);return this;},publish(_t,b){publish(b);},end(){this.connected=false;this.handlers.close?.();}};s.client=c;setTimeout(()=>{c.connected=true;c.handlers.connect?.();hb();},1);return c;}};
 s.hb=hb;s.frame=frame;s.mapFrame=mapFrame;
})();`;}
async function main(){await mkdir(review,{recursive:true});await new Promise(ok=>server.listen(0,'127.0.0.1',ok));let browser,context;try{
 const url=`http://127.0.0.1:${server.address().port}/index.html`;
 browser=await chromium.launch({headless:true,executablePath:chromePath});context=await browser.newContext({viewport:{width:1360,height:950}});const page=await context.newPage();const errors=[];page.on('pageerror',e=>errors.push(String(e)));page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
 await page.route('**/vendor/mqtt.min.js',r=>r.fulfill({status:200,contentType:'text/javascript',body:mockSource()}));await page.goto(url);await page.locator('#connect').click();
 await page.waitForFunction(()=>document.querySelector('#car')?.textContent==='car: online');await page.waitForFunction(()=>document.querySelector('#mapStatus')?.textContent.includes('Confirmed by source-8'));
 assert.match(await page.locator('#net').textContent(),/Operational/);assert.match(await page.locator('#navState').textContent(),/Running/);assert.equal(await page.locator('#published').textContent(),'29 / 2');assert.equal(await page.locator('#commandCounts').textContent(),'3 / 1');assert.equal(await page.locator('#queue').textContent(),'17% full');
 assert.equal(await page.locator('#mapRows [data-active]').allTextContents().then(x=>x.join('/')),'A/B/C/D');
 assert.equal(await page.locator('canvas').count(),0,'the RTOS build must not show unimplemented motion charts');assert.equal(await page.locator('#speedMeasured').count(),0,'the RTOS build must not fabricate motion telemetry');assert.match(await page.locator('main').textContent(),/motors are disabled/);
 for(const [i,ch] of ['Q','R','S','T'].entries())await page.locator(`#letter${i}`).selectOption(ch);await page.locator('#applyMap').click();await page.waitForFunction(()=>document.querySelector('#mapStatus')?.textContent.includes('differs from requested'));assert.equal(await page.locator('#mapRows [data-active]').allTextContents().then(x=>x.join('/')),'A/B/C/D');
 await page.locator('#readMap').click();await page.waitForFunction(()=>document.querySelector('#mapStatus')?.textContent.includes('Confirmed by source-8'));assert.equal(await page.locator('#mapRows [data-active]').allTextContents().then(x=>x.join('/')),'Q/R/S/T');
 await page.locator('[data-cmd="START"]').click();await page.waitForFunction(()=>document.querySelector('#lastAck')?.textContent.includes('START request'));const published=await page.evaluate(()=>window.__mqttMock.published);assert.ok(published.some(b=>(b[0]>>4)===1),'START was sent through Buddy 1 command topic');assert.ok(published.every(b=>![12,13,14].includes(b[0]>>4)),'no direct steering/drive/turn command is offered');
  await page.evaluate(()=>window.__mqttMock.hb());await page.waitForFunction(()=>document.querySelector('#delivery')?.textContent.startsWith('2 received'));
 const desktop=join(review,'desktop.png');await page.screenshot({path:desktop,fullPage:true});
 const mobile=await context.newPage();await mobile.setViewportSize({width:390,height:844});await mobile.route('**/vendor/mqtt.min.js',r=>r.fulfill({status:200,contentType:'text/javascript',body:mockSource()}));mobile.on('pageerror',e=>errors.push(String(e)));await mobile.goto(url);await mobile.locator('#connect').click();await mobile.waitForFunction(()=>document.querySelector('#car')?.textContent==='car: online');const mobilePath=join(review,'mobile.png');await mobile.screenshot({path:mobilePath,fullPage:true});assert.equal(await mobile.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true,'mobile dashboard has no horizontal overflow');
 const bundled=await context.newPage();bundled.on('pageerror',e=>errors.push(String(e)));bundled.on('console',m=>{if(m.type()==='error')errors.push(m.text());});await bundled.goto(url);await bundled.waitForFunction(()=>typeof window.mqtt?.connect==='function');await bundled.close();assert.deepEqual(errors,[],'no browser errors');
 console.log(JSON.stringify({desktop,mobile:mobilePath,heartbeat:'source-1 decoded',mapReadback:'source-8 mismatch then confirmation',command:'Buddy 1 supervisory route',errors},null,2));
 }finally{await context?.close();await browser?.close();await new Promise(ok=>server.close(ok));}}
main().catch(e=>{console.error(e);process.exitCode=1;});
