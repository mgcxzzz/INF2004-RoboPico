#!/usr/bin/env python3
# Compat copy for the merged dashboard v0.1: original testing/integration/host/check_imu_dashboard.py (sha256 9275e6125261844e), changed only where marked COMPAT (paths/mock route/screenshots).
"""INT-03: compile real C packer, compare real JS decoder to known fields.

NODE may select the local Node executable. No broker or hardware is used.
"""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]  # COMPAT: repo root from tools/dashboard_merged/test/compat
host = root / 'testing/integration/host'  # COMPAT
with tempfile.TemporaryDirectory(prefix='is28-int03-') as temp:
    executable = str(Path(temp) / 'imu_vectors')
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2',
                    '-I' + str(root / 'app_program/subsystem4'),
                    str(host / 'imu_vectors.c'),
                    str(root / 'app_program/subsystem4/b4_pack.c'),
                    '-o', executable], check=True)
    frames = subprocess.run([executable], check=True, capture_output=True, text=True).stdout
    javascript = r'''
const assert = require('node:assert/strict');
const fs = require('node:fs');
const schema = require(process.argv[1]);
const frames = fs.readFileSync(0, 'utf8').trim().split('\n');
const decoded = frames.map(x => schema.decodeFrame(Buffer.from(x, 'hex')));
const fields = [
  {src:4,seq:17,state:3,valid:true,odometry:false,pitchDeg:12.5,motion:'climbing',event:'status'},
  {src:4,seq:18,state:4,valid:true,odometry:true,pitchDeg:-8.7,heightMm:28,highestMm:31,humps:2,rejected:1,framesMissed:3,eventMs:12345,event:'hump peak'},
  {src:4,seq:255,state:6,valid:false,odometry:false,pitchDeg:-90,lastFault:9,shockMg:4095,shockNotifyMs:63,shocks:255,i2cErrors:63,framesMissed:65535,eventMs:65535,event:'fault'}
];
let count = 0;
for(let i=0;i<fields.length;i++) {
  assert.equal(decoded[i].error, undefined);
  const actual = {...decoded[i],...decoded[i].data};
  for(const [key,value] of Object.entries(fields[i])) {assert.equal(actual[key],value,`vector ${i}, ${key}`);count++;}
}
const short = Buffer.from(frames[0],'hex').subarray(0,18);
assert.ok(schema.decodeFrame(short).error);count++;
console.log(`INT-03 PASS: ${count} field/malformed-frame assertions; production C packer and JS decoder.`);
// Exercise the production panel handler with a minimal host DOM. This is
// display-logic regression coverage, not a browser rendering/hardware test.
const vm = require('node:vm');
const html = fs.readFileSync(process.argv[2],'utf8');
const nodes = new Map(); const timers = []; let clock = 100;
class Element {
  constructor(){this.textContent='';this.innerHTML='';this.value='';this.dataset={};this.childNodes=[];this.style={};}
  set id(value){this._id=value;nodes.set(value,this);} get id(){return this._id;}
  setAttribute(){} add(){} appendChild(value){this.childNodes.push(value);return value;}
  append(...values){this.childNodes.push(...values);} click(){} remove(){}
}
for(const match of html.matchAll(/id="([^"]+)"/g)){const node=new Element();node.id=match[1];}
for(const match of html.matchAll(/id="([^"]+)"[^>]*>([^<]*)/g))nodes.get(match[1]).textContent=match[2].trim();
const document={getElementById:id=>nodes.get(id),createElement:()=>new Element(),
 querySelectorAll:selector=>selector==='[data-map-active]'?[...nodes.values()].filter(n=>'mapActive' in n.dataset):[]};
const context=vm.createContext({document,IS28:schema,location:{hostname:'127.0.0.1'},
 localStorage:{getItem:()=>null},performance:{now:()=>clock},
 setInterval:fn=>{timers.push(fn);return timers.length;},clearInterval:()=>{},
 setTimeout:()=>0,Option:function(label,value){this.value=value;this.text=label;},
 console,Uint8Array,Date});
const inline=[...html.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g)].map(m=>m[1]).filter(x=>x.trim());
for(const code of inline)vm.runInContext(code,context);
const get=id=>nodes.get(id).textContent;
vm.runInContext('show('+JSON.stringify(decoded[0])+')',context);
assert.equal(get('imuPitch'),'12.5°');
assert.equal(get('imuValid'),'valid (calibrated)');
assert.equal(get('imuOdometry'),'unavailable');
assert.ok(get('imuSpeed').includes('unavailable'));
assert.equal(get('imuHeight'),'no accepted peak received');
vm.runInContext('show('+JSON.stringify(decoded[1])+')',context);
assert.equal(get('imuPitch'),'-8.7°');assert.equal(get('imuHeight'),'28 mm');
vm.runInContext('show('+JSON.stringify(decoded[2])+')',context);
assert.ok(get('imuPitch').includes('not valid'));
assert.equal(get('imuHeight'),'28 mm'); // Fault/rejection event must not replace accepted peak.
clock=2600;for(const tick of timers)tick();
assert.ok(get('imuValid').includes('stale'));
console.log('INT-03 PASS: 10 production panel assertions with host DOM stubs; no rendering/hardware claim.');
'''
    subprocess.run([os.environ.get('NODE', 'node'), '-e', javascript,
                    str(root / 'tools/dashboard_merged/is28_schema.js'),  # COMPAT
                    str(root / 'tools/dashboard_merged/index.html')],  # COMPAT
                   input=frames, text=True, check=True)
