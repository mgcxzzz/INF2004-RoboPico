# Application C checks

The policy and exceptions are in the repository-root `CODING_STANDARD.md` and
`docs/coding_exceptions.md`. `scope.py` owns the application boundary; it does
not select upstream libraries or historical artifacts.

```text
python tools/coding_standard/check.py
python tools/coding_standard/format.py --check
```

Use clang-format **18.1.8** (`pip install clang-format==18.1.8`), or set
`CLANG_FORMAT` to that executable. Without `--check`, the formatter edits only
the owned application files. The mechanical checker uses Python's standard
library and supports `--json <report-path>`.

The checker rejects tabs/trailing whitespace, missing module contracts/guards/
EOF markers, signed bit-fields, nonconforming typedef names and C-source
inclusion. It does not claim to parse all C semantics. Formatting is a separate
pinned check. Comments/strings are masked before token searches.

Clang's source parser was used for the initial rename/declaration/expression
cleanup; it is not a runtime dependency or a requirement for running the robot.
The X-macro diagnostic schema is kept one row per field with a documented local
formatting exclusion because clang-format otherwise repeatedly rearranges it.

Existing production C regression programs are run through
`tools/rtos_migration/run_tests.py`. `test_compact_signed` adds range, signed
interpretation and baseline layout checks. GitHub Actions enforces the mechanical
and formatting gates plus focused affected-feature checks; it does not access
hardware or certify whole-standard conformance.
