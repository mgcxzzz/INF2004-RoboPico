/* Check actual production harness/packer output from run_tests.py, not synthetic frames.
 * node test_fine_reporting.js <test-log> [additional-dashboard-schema.js] */
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const lines=fs.readFileSync(process.argv[2],'utf8').split(/\r?\n/);
const frames=lines.filter(s=>s.startsWith('FINE_REPORT_VECTOR ')).map(s=>Uint8Array.from(Buffer.from(s.split(' ')[1],'hex')));
assert.equal(frames.length,4,'both idle and fine-bank reporting must be captured');
for(const file of [path.join(__dirname,'dashboard/is28_schema.js'),...process.argv.slice(3)]){
 const schema=require(path.resolve(file));const results=frames.map(b=>schema.decodeFrame(b));
 for(const r of results){assert.equal(r.error,undefined);assert.equal(r.data.error,undefined);}
 assert.equal(results[0].data.bank,11);assert.equal(results[0].data.thresholdAdc,213);
 assert.equal(results[2].data.bank,63);assert.equal(results[2].data.thresholdAdc,204);
 for(const i of [0,2])assert.equal(results[i].data.flags&0xf0,0);
 for(const i of [1,3])assert.equal(results[i].data.detail,0xbc03);
 const bad=frames[2].slice();bad[9]=64;assert.match(schema.decodeFrame(bad).error,/bank/);
 const reserved=frames[0].slice();reserved[10]|=0x80;assert.match(schema.decodeFrame(reserved).error,/reserved/);
 console.log('PASS actual periodic producer -> C packer -> JS decoder, bank11/bank63/version and reserved-bit guards: '+file);
}
