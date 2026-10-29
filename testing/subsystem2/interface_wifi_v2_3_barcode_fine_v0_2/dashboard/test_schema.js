'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const IS28 = require('./is28_schema.js');

function frame(src, payload, seq = 19) {
  return Uint8Array.from([(src << 4) | IS28.VERSION, payload.length, seq, ...payload]);
}
function put16(a, i, n) { a[i] = n & 255; a[i + 1] = (n >>> 8) & 255; }
function put32(a, i, n) { a[i] = n & 255; a[i+1] = (n>>>8)&255; a[i+2]=(n>>>16)&255; a[i+3]=(n>>>24)&255; }

test('decoder accepts exact current source payload lengths and rejects malformed frames', () => {
  for (const [src, len] of [[1,12],[8,5],[9,45],[10,21],[11,23],[12,22],[14,60]]) {
    const out = IS28.decodeFrame(frame(src, new Array(len).fill(0)));
    assert.equal(out.error, undefined, `src ${src}`);
  }
  assert.match(IS28.decodeFrame(new Uint8Array([2, 0])).error, /too short/);
  assert.match(IS28.decodeFrame(new Uint8Array([0x92, 45, 1])).error, /length/);
  assert.match(IS28.decodeFrame(frame(9, new Array(44).fill(0))).error, /expected 45/);
  assert.match(IS28.decodeFrame(new Uint8Array([0x91, 0, 0])).error, /schema version/);
});

test('motion frame decodes signed speed, calibrated levels, distance, and timing', () => {
  const p = new Array(45).fill(0);
  put32(p,0,123456); put32(p,4,9876); put16(p,8,0x3412);
  p[10]=4; p[11]=3; p[12]=5; p[13]=2;
  put16(p,14,0xff9c); put16(p,16,250); put16(p,18,120); put16(p,20,125);
  put16(p,22,30); put16(p,24,31); put16(p,26,10); put16(p,28,20); put16(p,30,30);
  put16(p,32,100); put16(p,34,500); put16(p,36,0xffff); p[38]=5;
  put32(p,39,654321); put16(p,43,0xff38);
  const { data:d } = IS28.decodeFrame(frame(9,p));
  assert.equal(d.uptimeMs,123456); assert.equal(d.elapsedMs,9876); assert.equal(d.runId,0x3412);
  assert.equal(d.speedL,-10); assert.equal(d.speedR,25); assert.deepEqual(d.levels,[100,500,65535]);
  assert.equal(d.motionActive,true); assert.equal(d.calibrationValid,false); assert.equal(d.lineMaskAvailable,false);
  assert.equal(d.automatic,true); assert.equal(d.emergencyStop,false); assert.equal(d.runRelativeUptime,false);
  assert.equal(d.distanceMm,654321); assert.equal(d.headingCdeg,-200);
});

test('line event preserves decoded, pending, executing actions and their revisions', () => {
  const p = new Array(21).fill(0); put32(p,0,42); p[4]=7; put16(p,5,1); put16(p,7,2); put16(p,9,3);
  put16(p,11,0xffff); put16(p,13,900); p[15]=90; p[16]=76; p[17]=82; p[18]=8; p[19]=85; p[20]=7;
  const { data:d } = IS28.decodeFrame(frame(10,p));
  assert.equal(d.letter,'Z'); assert.equal(d.pendingLetter,'L'); assert.equal(d.action,'R');
  assert.deepEqual(d.levels,[65535,900]); assert.equal(d.revision,8); assert.equal(d.executingAction,'U'); assert.equal(d.executingRevision,7);
});

test('coordinator event distinguishes dispatch and completion identifiers', () => {
  const p=new Array(23).fill(0); put32(p,0,777); p[4]=6; p[5]=90; p[6]=82; p[7]=23;
  put32(p,8,0x12345678); put32(p,12,0x10203040); put16(p,16,2);
  p[18]=3; p[19]=4; p[20]=5; put16(p,21,99);
  const { data:d }=IS28.decodeFrame(frame(11,p));
  assert.equal(IS28.EVENT_NAME[d.event],'completed'); assert.equal(d.letter,'Z'); assert.equal(d.action,'R');
  assert.equal(d.revision,23); assert.equal(d.commandId,0x12345678); assert.equal(d.generation,0x10203040);
  assert.equal(d.fault,2); assert.equal(d.owner,3); assert.equal(d.mode,4); assert.equal(d.commandState,5); assert.equal(d.detail,99);
});

test('barcode map builder covers every A-Z letter in each distinct slot', () => {
  for (let i=0;i<26;i++) {
    const ch=String.fromCharCode(65+i);
    for (let slot=0;slot<4;slot++) {
      const others=Array.from({length:26},(_,n)=>String.fromCharCode(65+n)).filter((x)=>x!==ch).slice(0,3);
      const letters=Array(4); letters[slot]=ch;
      let n=0; for(let k=0;k<4;k++) if(k!==slot) letters[k]=others[n++];
      const packed=IS28.mappingWord(letters);
      assert.equal((packed >>> (slot*5)) & 31,i);
    }
  }
  assert.throws(()=>IS28.mappingWord(['A','A','B','C']), /four different/);
  assert.throws(()=>IS28.mappingWord(['A','B','C','?']), /four different/);
});

test('map readback validates configured bits and exposes revision and letters', () => {
  const word = IS28.mappingWord(['Z','A','Q','C']) | (1<<20) | (73<<21);
  const p=[word&255,(word>>>8)&255,(word>>>16)&255,(word>>>24)&255,31];
  assert.deepEqual(IS28.decodeFrame(frame(8,p)).data, {configured:true,revision:73,requestId:31,letters:['Z','A','Q','C']});
});

test('map status distinguishes confirmed, empty, mismatch, and rejected readback data', () => {
  const configured={configured:true,revision:9,requestId:4,letters:['A','B','C','D']};
  assert.equal(IS28.mappingReadbackStatus(configured,['A','B','C','D']),'confirmed');
  assert.equal(IS28.mappingReadbackStatus(configured,['A','B','C','E']),'mismatch');
  assert.equal(IS28.mappingReadbackStatus({configured:false,revision:0,requestId:4,letters:[]},null),'not_configured');
  assert.equal(IS28.mappingReadbackStatus({error:'invalid mapping status'},null),'invalid');
  const rejected=IS28.decodeAck(new Uint8Array([0xb2,4,1,0]));
  assert.equal(rejected.codeName,'GET_LOG'); assert.equal(rejected.accepted,false);
  assert.equal(rejected.resultName,IS28.ACK_NAME[1]);
  const mapRejected=IS28.decodeAck(new Uint8Array([0x92,8,3,0]));
  assert.equal(mapRejected.codeName,'BARCODE_MAP'); assert.equal(mapRejected.accepted,false);
  assert.equal(mapRejected.resultName,IS28.ACK_NAME[3]);
  const invalidReserved=[0,0,0,0x20,9];
  assert.match(IS28.decodeFrame(frame(8,invalidReserved)).error,/invalid mapping status/);
  const duplicateWord=(1<<20)|(0<<0)|(0<<5)|(2<<10)|(3<<15);
  const duplicate=[duplicateWord&255,(duplicateWord>>>8)&255,(duplicateWord>>>16)&255,(duplicateWord>>>24)&255,9];
  assert.match(IS28.decodeFrame(frame(8,duplicate)).error,/invalid mapping letters/);
});

test('map confirmation times out at its explicit five-second deadline', () => {
  const pending={time:1000};
  assert.equal(IS28.mappingTimedOut(pending,5999),false);
  assert.equal(IS28.mappingTimedOut(pending,6000),true);
  assert.equal(IS28.mappingTimedOut(null,9000),false);
});

test('source-13 log chunks validate identity, reserved bytes, status, and chunk lengths', () => {
  const p=new Array(15).fill(0); p[0]=2; p[1]=77; put16(p,2,3); put16(p,4,12);
  p[6]=1; p[7]=2; p[8]=0; p[9]=3; p[12]=0xaa; p[13]=0xbb; p[14]=0xcc;
  const f=IS28.decodeFrame(frame(13,p));
  assert.equal(f.error,undefined); assert.deepEqual(f.data.bytes,[0xaa,0xbb,0xcc]);
  assert.equal(f.data.dataset,2); assert.equal(f.data.requestId,77); assert.equal(f.data.rowIndex,3);
  const bad=p.slice(); bad[10]=1;
  assert.match(IS28.decodeFrame(frame(13,bad)).error,/reserved/);
  const mismatch=p.slice(); mismatch[9]=4;
  assert.match(IS28.decodeFrame(frame(13,mismatch)).error,/length mismatch/);
  const rejected=new Array(12).fill(0); rejected[0]=2; rejected[1]=77; rejected[7]=1; rejected[8]=3;
  assert.equal(IS28.decodeFrame(frame(13,rejected)).data.status,3);
});

test('source-14 diagnostics decodes counters, flags, and gaps, and rejects invalid status values', () => {
  const p=new Array(60).fill(0); put32(p,0,1200); put16(p,4,1700); p[6]=17; p[7]=15;
  for(let i=0;i<13;i++) put32(p,8+i*4,100+i);
  const d=IS28.decodeFrame(frame(14,p)).data;
  assert.equal(d.thresholdAdc,1700); assert.equal(d.bank,17); assert.equal(d.scanning,true);
  assert.equal(d.pending,true); assert.equal(d.frontCalibrationValid,true); assert.equal(d.runActive,true);
  assert.equal(d.decodedTotal,100); assert.equal(d.turns,101); assert.equal(d.lefts,102); assert.equal(d.rights,103);
  assert.equal(d.uturns,104); assert.equal(d.badFrames,105); assert.equal(d.unsupported,106);
  assert.equal(d.maxAdcGapUs,107); assert.equal(d.maxControlGapUs,108); assert.equal(d.txDropped,109);
  assert.equal(d.rxDropped,110); assert.equal(d.invalidLeft,111); assert.equal(d.invalidRight,112);
  const badFlags=p.slice(); badFlags[7]=0x80;
  assert.match(IS28.decodeFrame(frame(14,badFlags)).error,/reserved bits/);
  const badThreshold=p.slice(); put16(badThreshold,4,4096);
  assert.match(IS28.decodeFrame(frame(14,badThreshold)).error,/threshold/);
  const fineBank=p.slice(); fineBank[6]=63;
  assert.equal(IS28.decodeFrame(frame(14,fineBank)).data.bank,63);
  const badBank=p.slice(); badBank[6]=64;
  assert.match(IS28.decodeFrame(frame(14,badBank)).error,/bank/);
});
