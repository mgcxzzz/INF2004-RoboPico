# Host tests: run on a PC with gcc, no Pico needed.
#   make -C testing/host
#
# The code under test is compiled unchanged from app_program/; only the
# kernel is replaced by a small mock (mock_*/tk/tkernel.h).
#
#   test_b5_pack     S5 minimum-bit representation: sizes, clamping,
#                    sign extension, 16-bit wrap, telemetry frame
#   test_b5_profile  S5 obstacle profiling: width accuracy, side choice
#   test_b5_task     S5 statechart against a simulated car (4 scenarios)
#   test_nav         Navigation Coordinator, incl. its packed inbox and
#                    telemetry frames (team schema, sys_tlm.h)
#   test_safety      Safety Supervisor, incl. its telemetry frames
#   test_s3          S3 Code 39 decoding (the course's real barcodes), the
#                    distance-based reader, line logic, telemetry frame
#   test_s3_task     S3 statechart on a simulated track with barcodes
CC      ?= gcc
APP     := ../../app_program
S3      := $(APP)/subsystem3
S5      := $(APP)/subsystem5
SYS     := $(APP)/system
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2
HDRS    := $(wildcard $(S3)/*.h $(S5)/*.h $(SYS)/*.h)
S3_LOGIC := $(S3)/s3_barcode.c $(S3)/s3_line.c $(S3)/s3_reader.c $(S3)/s3_pack.c

.PHONY: all run clean
all: run

test_b5_pack: test_b5_pack.c $(S5)/b5_pack.c $(S5)/obstacle_profile.c $(HDRS)
	$(CC) $(CFLAGS) -Imock_b5 -I$(S5) -o $@ test_b5_pack.c $(S5)/b5_pack.c \
	    $(S5)/obstacle_profile.c

test_b5_profile: test_b5_profile.c $(S5)/obstacle_profile.c $(HDRS)
	$(CC) $(CFLAGS) -I$(S5) -o $@ test_b5_profile.c $(S5)/obstacle_profile.c -lm

test_b5_task: test_b5_task.c $(S5)/task_obstacle.c $(S5)/obstacle_profile.c $(S5)/b5_pack.c $(HDRS)
	$(CC) $(CFLAGS) -Imock_b5 -I$(S5) -I$(SYS) -o $@ \
	    test_b5_task.c $(S5)/task_obstacle.c $(S5)/obstacle_profile.c \
	    $(S5)/b5_pack.c -lm

test_s3: test_s3.c $(S3_LOGIC) $(HDRS)
	$(CC) $(CFLAGS) -Imock_b5 -I$(S3) -I$(SYS) -o $@ test_s3.c $(S3_LOGIC) -lm

test_s3_task: test_s3_task.c $(S3)/task_line.c $(S3_LOGIC) $(HDRS)
	$(CC) $(CFLAGS) -Imock_sys -I$(S3) -I$(SYS) -I$(S5) -o $@ \
	    test_s3_task.c $(S3)/task_line.c $(S3_LOGIC) -lm

test_nav: test_nav.c $(SYS)/nav_coordinator.c $(SYS)/nav_b5.c $(SYS)/sys_tlm.c $(HDRS)
	$(CC) $(CFLAGS) -Imock_sys -I$(SYS) -I$(S5) -o $@ \
	    test_nav.c $(SYS)/nav_coordinator.c $(SYS)/nav_b5.c $(SYS)/sys_tlm.c

test_safety: test_safety.c $(SYS)/safety_supervisor.c $(SYS)/safety_glue.c $(SYS)/sys_tlm.c $(HDRS)
	$(CC) $(CFLAGS) -Imock_sys -I$(SYS) -I$(S5) -o $@ \
	    test_safety.c $(SYS)/safety_supervisor.c $(SYS)/safety_glue.c \
	    $(SYS)/sys_tlm.c

run: test_s3 test_s3_task test_b5_pack test_b5_profile test_b5_task test_nav test_safety
	./test_s3
	@echo
	./test_s3_task
	@echo
	./test_b5_pack
	@echo
	./test_b5_profile
	@echo
	@echo "statechart scenarios"
	@for s in 1 2 3 4; do ./test_b5_task $$s || exit 1; done
	@echo
	./test_nav
	@echo
	./test_safety

clean:
	rm -f test_s3 test_s3_task test_b5_pack test_b5_profile test_b5_task \
	    test_nav test_safety
