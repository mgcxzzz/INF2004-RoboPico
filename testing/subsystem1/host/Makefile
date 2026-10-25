# Buddy 1 host tests: run on a PC with gcc, no Pico needed.
#   make -C testing/subsystem1/host
#
# b1_proto.c (command checking, acknowledgements, status frame) and the
# shared sys_tlm.c are compiled unchanged; they need no kernel.
# "make vectors" prints sample frames for the dashboard decoder check
# (node check_dashboard.mjs).

CC      ?= gcc
S1      := ../../../app_program/subsystem1
SYS     := ../../../app_program/system
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 -I$(S1) -I$(SYS)
SRCS    := $(S1)/b1_proto.c $(SYS)/sys_tlm.c

.PHONY: all run vectors clean
all: run

test_b1: test_b1.c $(SRCS) $(wildcard $(S1)/*.h) $(SYS)/sys_tlm.h
	$(CC) $(CFLAGS) -o $@ test_b1.c $(SRCS)

run: test_b1
	./test_b1

vectors: test_b1
	./test_b1 --vectors > vectors.json

clean:
	rm -f test_b1 vectors.json
