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
| `test_s3` | `subsystem3/s3_barcode.c`, `s3_reader.c`, `s3_line.c`, `s3_pack.c` | Code 39 (all four course barcodes measured from Barcode_Sample.pdf, every character, rejects, 2000 noisy reads never misread), the reader placing interrupt-timed edges by distance (steady, varying speed, stopping on a space), line steering / junction / lost / stuck, telemetry frame | 58 passed |
| `test_s3_task` | `subsystem3/task_line.c` | statechart 6.3 on a simulated track with the course barcodes: barcode turns at the next junction with the axle over it, leading-sensor line search, U-turn / straight, no junction = dropped, one event per barcode, commands A/B/C/D, stray marks, bad / backwards / unknown reads, no distance, ignored event, junction, lost and found, pause, e-stop, stuck sensor and fault-bit latch, lost edges, every telemetry frame | 74 passed |
| `test_b5_pack` | `subsystem5/b5_pack.c`, `obstacle_profile.c` | minimum-bit representation: record sizes, clamping (never wrapping), sign extension, 16-bit wrap, echo result word, binary telemetry frame layout with the common header (wrong version, source or length rejected) | 43 passed |
| `test_b5_profile` | `subsystem5/obstacle_profile.c` | width accuracy over many obstacle sizes, distances and offsets; the side choice; the trig and median helpers | 95 passed |
| `test_b5_task 1..4` | `subsystem5/task_obstacle.c` | S5 statechart against a simulated car: bypass, sensor dead, blocked both sides, line not found; also checks the shared fault-mask bit, decodes every telemetry frame, and runs the 16-bit odometer across its wrap | 4 scenarios PASS |
| `test_nav` | `system/nav_coordinator.c`, `nav_b5.c`, `sys_tlm.c` | Navigation Coordinator (see `app_program/system/README.md` section 9), plus its 8-byte inbox messages, value clamping, 15-bit command ids, the saturating dropped-message count every telemetry frame (header sequence never skips), Buddy 3's direct steering path, and barcode turns by line search | 84 passed |
| `test_safety` | `system/safety_supervisor.c`, `safety_glue.c`, `sys_tlm.c` | Safety Supervisor (same README), plus every telemetry frame: stop time saturating at 1023 ms, 8-bit counters wrapping, the shared fault mask carried, header sequence never skips | 36 passed |

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
