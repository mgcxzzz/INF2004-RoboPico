"""Check real ARM stack frames, call edges and linker stack/heap separation."""
from pathlib import Path
import argparse,json,re,subprocess
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--build',type=Path)
p.add_argument('--elf',type=Path)
p.add_argument('--usage',type=Path)
p.add_argument('--expect-unsafe',action='store_true')
a=p.parse_args()
release=json.loads((Path(__file__).resolve().parent/'BUILD_METRICS.json').read_text())
elf=a.elf or a.build/Path(release['artifact']).with_suffix('.elf').name
usage=a.usage or a.build/'CMakeFiles/buddy123_interface_wifi_v2.dir/main.c.su'
frames={}
for line in usage.read_text().splitlines():
    f=line.split('\t')
    if len(f)>=3:frames[f[0].rsplit(':',1)[-1]]=int(f[1])
chain=['main','control_step','interface_navigation_step','barcode_circuit_step.constprop']
frame_names=[next(n for n in frames if n==f or n.startswith(f+'.')) for f in chain]
text=subprocess.check_output(['arm-none-eabi-nm','-n',str(elf)],text=True)
symbols={};function_names=[]
for line in text.splitlines():
    f=line.split()
    if len(f)==3:
        symbols[f[2]]=int(f[0],16)
        if f[1] in 'tT':function_names.append(f[2])
real_names=[next(n for n in function_names if n==f or n.startswith(f+'.')) for f in chain]
for caller,callee in zip(real_names,real_names[1:]):
    asm=subprocess.check_output(['arm-none-eabi-objdump','-d','--disassemble='+caller,str(elf)],text=True)
    assert re.search(r'\bbl\s+[^\n]*<'+re.escape(callee)+r'>',asm),f'Missing call edge {caller} -> {callee}'
known=sum(frames[n] for n in frame_names)
print('Compiler frames on verified driving call chain:',', '.join(f'{n}={frames[n]}' for n in frame_names))
print('Known chain total:',known,'bytes; excludes deeper callees, interrupt frames and runtime watermark.')
core0top=symbols['__StackTop'];core0bottom=symbols['__StackBottom']
core1top=symbols['__StackOneTop'];core1bottom=symbols['__StackOneBottom']
if a.expect_unsafe:
    assert core0top-known<core1top and core0top>core1bottom,'Old overlap was not reproduced'
    print(f'PASS old-layout defect reproduced: projected SP={core0top-known:#x} enters network stack [{core1bottom:#x},{core1top:#x}).')
else:
    assert core0top-core0bottom==8192
    assert core1top-core1bottom==8192
    assert symbols['__bss_end__']<=symbols['__HeapLimit']<=core1bottom<core1top<=core0bottom<core0top<=0x20040000
    assert symbols['core1_stack']==core1bottom
    assert symbols['__StackLimit']==symbols['__HeapLimit']
    assert known+1024<core0top-core0bottom,'Insufficient margin above measured chain lower bound'
    print(f'PASS core0 stack [{core0bottom:#x},{core0top:#x}), core1 [{core1bottom:#x},{core1top:#x}); heap ends {symbols["__HeapLimit"]:#x}.')
    print('Static checks pass; physical Start/heartbeat test and runtime maximum stack use remain unverified.')
