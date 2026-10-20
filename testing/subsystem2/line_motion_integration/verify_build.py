"""Verify the real linked integration and the ISR/stack constraints (no hardware claims)."""
from pathlib import Path
import subprocess, re, struct, hashlib, sys
R=Path(__file__).resolve().parent
ELF=R/"build/build_make/mtk3pico_smp0_usb_cdc.elf"
prefix=Path(sys.argv[1]) if len(sys.argv)>1 else None
def tool(name): return str(prefix/("arm-none-eabi-"+name+".exe")) if prefix else "arm-none-eabi-"+name
symbols=subprocess.check_output([tool("nm"),"-S",str(ELF)],text=True)
for name in ["mc_set_wheel_speeds","mc_inhibit","nav_line_drive","line_set_control","usermain"]:
 assert re.search(r"^[0-9a-f]+ [0-9a-f]+ T "+name+r"$",symbols,re.M),name
for name in ["knl_idle_stack","knl_tmp_stack"]:
 assert re.search(r"^[0-9a-f]+ 00000800 [bB] "+name+r"$",symbols,re.M),name
for name,allowed in [("watchdog_irq",{"sys_time_us","stop_locked"}),("stop_locked",set())]:
 assembly=subprocess.check_output([tool("objdump"),"-d","--disassemble="+name,str(ELF)],text=True)
 assert "<"+name+">:" in assembly
 calls=set(re.findall(r"\bbl\s+[0-9a-f]+ <([^>]+)>",assembly))
 assert calls<=allowed,(name,calls)
 assert "__aeabi" not in assembly
ram=re.search(r"^([0-9a-f]+) [0-9a-f]+ [tT] write_sector(?:\.constprop\.0)?$",symbols,re.M)
assert ram and 0x20000000<=int(ram[1],16)<0x20040000
base=R.parents[2]
ref="bc950be50755d3a67cc8de82716c5b1582b56ff1"
norm=lambda x:x.replace(b"\r\n",b"\n")
for path in ["system/nav_coordinator.c","subsystem3/task_line.c","subsystem3/s3_ir.c","subsystem3/s3_line.c"]:
 original=subprocess.check_output(["git","-C",str(base),"show",ref+":app_program/"+path])
 assert norm(original)==norm((R/"build/app_program"/path).read_bytes()),path
for name in ["integration.c","integration.h","logic.h","flash_log.c"]:
 assert norm((R/name).read_bytes())==norm((R/"build/app_program/integration"/name).read_bytes()),name
uf2=R/"firmware/buddy23_line_motion_v3.uf2"
data=uf2.read_bytes()
for i in range(0,len(data),512):
 a,b,flags,address,n,block,total,family=struct.unpack_from("<8I",data,i)
 assert a==0x0a324655 and b==0x9e5d5157
 assert 0x10000000<=address and address+n<=0x101ff000
print("PASS strong real interfaces; idle/tmp 2048 bytes; raw cutoff has no kernel/FP calls; SRAM flash writer")
print("PASS unchanged coordinator/S3 sources; adapter matches build; UF2 below log sector")
print("UF2",len(data),"bytes SHA256",hashlib.sha256(data).hexdigest())
