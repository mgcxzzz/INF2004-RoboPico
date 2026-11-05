# Testing and evidence

Run offline regressions from the repository root: `python firmware/integration/buddy1_4/tests/run_tests.py` (requires a C compiler, Python and Node). Set CC/CFLAGS/NODE for your installed tools. Windows GCC9 may need `CFLAGS=-msse2 -mfpmath=sse -Wno-misleading-indentation`. Assertions stay enabled.

The suite covers inherited tracker/controller/parity and Wi-Fi interfaces, captured peak signals, dual-reader integration, ramp state/budgets/plant models, IMU service/packers, and new coordinator/handback regressions. No test contacts the robot or broker; provisioning and launcher tests mock their I/O.

[Evidence index](../testing/evidence/README.md) distinguishes hardware records, historical failures and software/build tests. Each imported file has a pinned original branch/commit/path and hash. Current build checks are stored under testing/evidence/current. Hardware acceptance of this restructured image remains pending; see STATUS.md.
