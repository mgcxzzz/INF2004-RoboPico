# Buddy 4 host tests: run on a PC with gcc, no Pico needed.
#   make -C testing/subsystem4/host
#
# imu_proc.c (statechart 6.4 and all processing) and b4_pack.c are compiled
# unchanged from app_program/subsystem4; they need no kernel.

CC      ?= gcc
S4      := ../../../app_program/subsystem4
CFLAGS  := -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O2 -I$(S4)
SRCS    := $(S4)/imu_proc.c $(S4)/b4_pack.c

.PHONY: all run clean
all: run

test_imu: test_imu.c $(SRCS) $(wildcard $(S4)/*.h)
	$(CC) $(CFLAGS) -o $@ test_imu.c $(SRCS) -lm

run: test_imu
	./test_imu

clean:
	rm -f test_imu
