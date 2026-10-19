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
#   test_nav         Navigation Coordinator
#   test_safety      Safety Supervisor
CC      ?= gcc
APP     := ../../app_program
S5      := $(APP)/subsystem5
SYS     := $(APP)/system
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2
HDRS    := $(wildcard $(S5)/*.h $(SYS)/*.h)

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

test_nav: test_nav.c $(SYS)/nav_coordinator.c $(SYS)/nav_b5.c $(HDRS)
	$(CC) $(CFLAGS) -Imock_sys -I$(SYS) -I$(S5) -o $@ \
	    test_nav.c $(SYS)/nav_coordinator.c $(SYS)/nav_b5.c

test_safety: test_safety.c $(SYS)/safety_supervisor.c $(SYS)/safety_glue.c $(HDRS)
	$(CC) $(CFLAGS) -Imock_sys -I$(SYS) -I$(S5) -o $@ \
	    test_safety.c $(SYS)/safety_supervisor.c $(SYS)/safety_glue.c

run: test_b5_pack test_b5_profile test_b5_task test_nav test_safety
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
	rm -f test_b5_pack test_b5_profile test_b5_task test_nav test_safety
