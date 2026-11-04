/* Exercise the actual profile functions/packet marker without browser I/O. */
const fs = require('node:fs'), vm = require('node:vm'), assert = require('node:assert/strict');
const html = fs.readFileSync('dashboard/index.html', 'utf8');
const declaration = html.match(/^let connectedDriveSeen =.*$/m)[0];
const text = html.match(/function profileText\(p\) \{[\s\S]*?\n\}/)[0];
const clear = html.match(/^function clearSeen\(\).*$/m)[0];
const marker = html.match(/if \(d.flags & 0x40\) \{[^\n]+else sdkDriveSeen = true;/)[0];
const ctx = vm.createContext({seenSources:new Map(),firstSeen:new Map()});
vm.runInContext(declaration+'\n'+text+'\n'+clear, ctx);
function feed(flags) { vm.runInContext('var d={flags:'+flags+'};'+marker, ctx); }
feed(0x60); assert.match(vm.runInContext("profileText('rtos-connected')",ctx),/Step4/);
feed(0x70); assert.match(vm.runInContext("profileText('rtos-connected')",ctx),/Step5/);
vm.runInContext('clearSeen()',ctx); feed(0x60);
assert.match(vm.runInContext("profileText('rtos-connected')",ctx),/Step4/);
vm.runInContext('clearSeen()',ctx); feed(0x40);
assert.match(vm.runInContext("profileText('rtos-connected')",ctx),/Step3B/);
console.log('PASS Step5 dashboard marker, Step4/Step3B compatibility and reset');
