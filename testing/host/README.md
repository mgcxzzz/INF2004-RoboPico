# Host tests (run on a PC, no Pico needed)

These tests compile the real code from `app_program/` **unchanged** with
the PC's gcc. Only the kernel is swapped for a small fake
(`mock_b5/tk/tkernel.h`, `mock_sys/tk/tkernel.h`) with a fake clock, so a
whole scenario runs in a fraction of a second.

They complement the on-target tests (`make TEST=1`), which check the
same code on the real Pico with the real sensor.

## Run

```sh
make -C testing/host          # build and run everything
make -C testing/host clean
```

Needs `gcc` and `make` (Linux, macOS, or WSL on Windows). The build uses
`-Wall -Wextra -Werror`, so a warning fails the test.

## What is tested

| Program | Code under test | What it checks | Last result |
|---|---|---|---|
| `test_b5_profile` | `subsystem5/obstacle_profile.c` | width accuracy over many obstacle sizes, distances and offsets; the side choice; the trig and median helpers | 95 passed |
| `test_b5_task 1..4` | `subsystem5/task_obstacle.c` | S5 statechart against a simulated car: bypass left, bypass right, blocked both sides, line not found | 4 scenarios PASS |
| `test_nav` | `system/nav_coordinator.c`, `nav_b5.c` | Navigation Coordinator (see `app_program/system/README.md` section 7) | 48 passed |
| `test_safety` | `system/safety_supervisor.c`, `safety_glue.c` | Safety Supervisor (same README) | 25 passed |

## Recording results

When a host test is used as evidence, save the output in the subsystem's
`logs/` folder and add a row to its `TEST_LOG.md`, the same as an
on-target test:

```sh
make -C testing/host 2>&1 | tee testing/subsystem5/logs/2026-10-15_host.txt
```

## Adding your own

Copy the pattern of `test_nav.c`: include your module's header, write the
kernel calls it uses in the mock (or add them to a mock header), and add a
rule to the `Makefile`.
