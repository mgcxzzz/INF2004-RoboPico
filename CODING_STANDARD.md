# RoboPico application C standard

This project adopts **RoboPico C Standard 1.0, based on BARR-C:2018**. It is an
adapted standard, not a claim of unmodified Barr-C or MISRA certification.
The project owner accepted the exception-based approach on 17 November 2026.

## Scope

Application C, headers and implementation includes in:

- `app_program/rtos_bringup`, `line_navigation`, `barcode_navigation`,
  `network_bridge` and `imu_navigation`;
- `firmware/integration/buddy1_4/src`, `include` and `config`.

Historical migration images, recorded evidence and generated artifacts retain
their original contents. Upstream kernel/BSP, SDK and networking libraries are
tracked dependencies; their code is not reformatted as application code. Local
changes to their ports require focused review. Host tests are validation clients,
not firmware; update them when application names change. JavaScript and Python
are outside this C standard.

## Required practices

- GNU C11 with the pinned Arm GCC toolchain; retain compile-time layout checks.
- Four-space indentation, no tabs, 80-column layout, Allman braces and braces for
  every control body. Long URLs, include paths and indivisible string contents
  may exceed the limit; wrap adjacent string literals without changing bytes.
- Lowercase underscore-separated application types ending in `_t`; public types
  and functions use a feature/module prefix. Prefer descriptive names; pointer
  variables use `p_`, file/global objects use `g_`, and loop counters have at least
  three characters. Kernel-defined types and externally required names remain.
- One declarator and one executable statement per line. Use explicit grouping
  for compound Boolean expressions and put constants first in equality tests.
- Fixed-width integers where width matters. Unsigned bit-fields only. Pack
  negative bounded values through unsigned storage with explicit signed access;
  never change their mathematical range or serialize native bit-field memory.
- Explicit byte/bit encoding for network and persisted data, with reserved bits
  zero and range validation. Retain packet sizes, signed semantics, calibration
  records and source/subtype identifiers unless a separately reviewed protocol
  change is intended.
- Static private helpers; documented public contracts and non-obvious numeric
  casts. Comments describe assumptions and ownership, not obvious assignments.
- Task functions end in `_task`; application interrupt callbacks end in `_isr`.
  Keep the kernel's interrupt registration and calling convention.
- Protect shared state with the existing synchronization and copied snapshots.
  `volatile` is for hardware/interrupt visibility, not an SMP locking substitute.
- Avoid magic loop bounds, undefined shifts, mixed-sign surprises, unchecked
  floating exceptional values and side effects in macros. Use typed helpers for
  repeated computation. Review casts against the actual source/destination range.
- Make reasonable effort to keep functions within 100 lines. Split by coherent
  responsibility without changing deadline, lock, acquisition or output order.

Barr's explicitly preferred practices remain recommendations rather than hard
errors. Early error returns are permitted where they simplify validation and
ensure locks/resources are released. Details and approved exceptions are in
[the exception register](docs/coding_exceptions.md).

## Change and validation discipline

Keep formatting, naming/structure, representation and release packaging in
separate commits. Compare against the hardware-tested v0.2.6 baseline. Preserve
RTOS task topology, priorities, timing budgets, coordinator ownership, motor-off
paths, both barcode decoders, 7 mm junction qualification, speed modes and saved
calibration priority. Coding cleanup does not authorize retuning those values.

Run `python tools/coding_standard/check.py` for the enforced subset and the
pinned clang-format check when available. Mechanical checks do not certify all
semantic Barr rules. Conversion/range, concurrency and module-contract review
remains required. The checker's report must identify its scope and limitations.

Changes to signed storage require boundary/codec parity checks and a release ARM
build with RAM/stack bounds. A final hardware smoke test is required before
promoting the cleanup release to the tested main baseline. Until then, the
existing main firmware remains available and the cleanup package is provisional.
