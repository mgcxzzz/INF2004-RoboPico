/* Decode bytes emitted by the production C owner/codec test. */
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const S=require('../is28_schema.js');
const f=fs.readFileSync(path.resolve(__dirname,'../../testing/rtos_migration/build-host/front_calibration.bin'));
const d=S.decodeFrame(f).data;
assert.equal(d.kind,'front-calibration');assert.equal(d.result,5);
assert.equal(d.saved,true);assert.equal(d.applied,true);assert.equal(d.session,false);
assert.deepEqual(d.activeWhite,[120,180]);assert.deepEqual(d.activeBlack,[450,650]);
assert.deepEqual(d.stagedWhite,[120,180]);assert.deepEqual(d.stagedBlack,[450,650]);
assert.deepEqual(d.raw,[450,180]);assert.equal(d.requestId,251);assert.equal(d.samples,32);
for(let op=0;op<=7;op++) assert.equal(S.calibrationArg(op),3|(op<<8));
for(const op of [-1,8,255,1.5,NaN])assert.throws(()=>S.calibrationArg(op),RangeError);
for(const [offset,mask] of [[4,1],[12,8],[13,128],[14,128]]) {
 const bad=Buffer.from(f);bad[offset]^=mask;assert(S.decodeFrame(bad).data?.error||S.decodeFrame(bad).error);
}
for(const [offset,value] of [[15,33],[16,4],[39,6]]) {
 const bad=Buffer.from(f);bad[offset]=value;assert(S.decodeFrame(bad).data?.error||S.decodeFrame(bad).error);
}
assert(S.decodeCalibrationStatus(f.subarray(3,39)).error);
console.log('PASS calibration schema: real C packet, raw/active/staged values, commands and invalid layouts');
