"""Prepare a pinned buddy3 uT-Kernel build without modifying the main application."""
from pathlib import Path
import subprocess, zipfile, shutil, hashlib
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[2]
COMMIT="bc950be50755d3a67cc8de82716c5b1582b56ff1"
BUILD=HERE/"build"
BUILD.mkdir(exist_ok=True)
archive=BUILD/"source.zip"
subprocess.run(["git","-C",str(REPO),"archive","--format=zip","-o",str(archive),COMMIT,
 "app_program","build_make","config","device","etc","include","kernel","lib","tools","testing/host"],check=True)
with zipfile.ZipFile(archive) as z:
 for name in z.namelist():
  target=(BUILD/name).resolve()
  if not target.is_relative_to(BUILD.resolve()): raise RuntimeError(name)
 z.extractall(BUILD)
app=BUILD/"app_program"/"integration"
app.mkdir(exist_ok=True)
for name in ("integration.c","flash_log.c","integration.h","logic.h"):
 shutil.copyfile(HERE/name,app/name)
MOTION_COMMIT="deac4940395b6cb9da23cfa29003e95f75a7f0fa"
for source,name in (("testing/subsystem2/dual_wheel_bench/dual_logic.h","dual_logic.h"),
 ("testing/subsystem2/dual_wheel_bench/log_store.h","log_store.h"),
 ("testing/subsystem2/left_motor_bench/bench_logic.h","bench_logic.h")):
 data=subprocess.check_output(["git","-C",str(REPO),"show",MOTION_COMMIT+":"+source])
 (app/name).write_bytes(data)
# All real coordinator + S3 + safety code stays unchanged; no simulation linked.
(BUILD/"app_program/system/sys_config.h").write_text("""#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H
#define SYS_SIM_CAR 0
#define SYS_SIM_LINE 0
#define SYS_CONSOLE_STATUS 0
#define SYS_BENCH_BUTTONS 0
#define SYS_SIM_AUTOSTART_MS 0
#endif
""")
files=["system/nav_coordinator.c","system/nav_stubs.c","system/sys_hal.c",
 "system/sys_fault.c","system/sys_tlm.c","system/safety_supervisor.c",
 "system/safety_glue.c","system/safety_stubs.c",
 "subsystem3/task_line.c","subsystem3/s3_ir.c","subsystem3/s3_line.c",
 "subsystem3/s3_reader.c","subsystem3/s3_barcode.c","subsystem3/s3_pack.c",
 "subsystem3/s3_stubs.c","integration/integration.c","integration/flash_log.c"]
mk="TEMP_SRCS := "+" ".join("../app_program/"+x for x in files)+"\n"
mk+="TEMP_OBJS = $(TEMP_SRCS:.c=.o)\nTEMP_DEPS = $(TEMP_SRCS:.c=.d)\n"
mk+="OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))\nC_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))\n"
mk+="mtkernel_3/app_program/%.o: ../app_program/%.c\n\t@mkdir -p $(dir $@)\n\t$(GCC) $(CFLAGS) -Wall -Wextra -Werror -DPROHIBIT_DEF_SIZE_T -include stddef.h -D$(TARGET) $(INCPATH) -MF\"$(@:%.o=%.d)\" -MT\"$(@)\" -c -o \"$@\" \"$<\"\n"
(BUILD/"build_make/mtkernel_3/app_program/subdir.mk").write_text(mk)
# Reserve log sector at link time, including the .data load image.
ld=BUILD/"etc/linker/pico_rp2040/tkernel_map.ld"
s=ld.read_text().replace("LENGTH = 2048K","LENGTH = 2044K")
s+='\nASSERT(__data_org + SIZEOF(.data) <= 0x101ff000, "firmware overlaps log sector")\n'
ld.write_text(s)
print("Prepared",BUILD,"from buddy3",COMMIT)

# The course USB console discards input. Add a tiny RX hook in its existing
# owner task; the adapter consumes a bounded ring, never calls TinyUSB itself.
glue=BUILD/"lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c"
s=glue.read_text().replace("while (tud_cdc_available()) (void)tud_cdc_read_char();",
 "extern void bench_rx_char(int ch);\n    while (tud_cdc_available()) bench_rx_char((int)tud_cdc_read_char());")
glue.write_text(s)

# Regression gate: this kernel shuts down if usermain returns. Check the
# adapter lifecycle against the actual pinned initial-task implementation.
entry=(app/"integration.c").read_text().split("EXPORT INT usermain(void)",1)[1]
assert "return 0;" not in entry
assert entry.count("for(;;) (void)tk_slp_tsk(TMO_FEVR);")==2
initial=(BUILD/"kernel/inittask/inittask.c").read_text()
assert "fin = usermain();" in initial and "shutdown_system(fin);" in initial
print("PASS startup lifecycle: success/failure keep USB task alive, no return to kernel shutdown")
