const schema = require(process.argv[2]); const fs = require('fs');
const lines = fs.readFileSync(process.argv[3], 'utf8').trim().split('\n');
let n = {2:0,4:0}, err = 0, humps = 0, maxH = 0;
for (const l of lines) { const f = schema.decodeFrame(Uint8Array.from(Buffer.from(l, 'hex')));
  if (f.error || !f.data) { err++; console.log('ERR', l, f.error); continue; } n[f.src]++;
  if (f.src === 4) { humps = Math.max(humps, f.data.humps ?? 0); maxH = Math.max(maxH, f.data.highestMm ?? 0); } }
console.log(JSON.stringify({frames: lines.length, src2: n[2], src4: n[4], errors: err, humps, maxH}));
const last4 = lines.map(l=>schema.decodeFrame(Uint8Array.from(Buffer.from(l,'hex')))).filter(f=>f.src===4).slice(-1)[0];
console.log('last src4 keys:', Object.keys(last4.data).join(','));
process.exit(err ? 1 : 0);
