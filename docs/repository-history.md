# Development branch archive

On 21 November 2026, the team retired the 14 development branches from GitHub after confirming that work on them had finished. The combined build, CI and test evidence are available from `main`. The [testing index](../testing/README.md) links to the standalone records and selected logs preserved in the repository.

Before retirement, the full history of all 15 branch tips was saved in `RoboPico-before-branch-retirement-2026-11-21.bundle`. The team verified the bundle by fetching each tip into a fresh bare repository and checking its Git objects. Its SHA-256 is `5da97af6c0804047a4257e1c8b366db938f0f0fd1449bfde02eb50696a9b71fb`.

The commit counts show what was reachable from each former tip outside `main` immediately before retirement. Counts can overlap across branches.

| Retired branch | Tip | Commits outside main | History after retirement |
| --- | --- | ---: | --- |
| `barr-c-cleanup` | `81a994f` | 0 | Reachable from main and archived |
| `barr-c-combined` | `10d05a2` | 0 | Reachable from main and archived |
| `buddy1` | `b094030` | 3 | Preserved in archive |
| `buddy3` | `74e0c5c` | 10 | Preserved in archive |
| `buddy4` | `e6b19b0` | 2 | Preserved in archive |
| `buddy5` | `1a0b559` | 14 | Preserved in archive |
| `buddy5-rtos-integration` | `573689b` | 1 | Preserved in archive |
| `buddy5-rtos-v0.2.12-review` | `89e7fc6` | 0 | Reachable from main and archived |
| `buddy5-single-scan-alignment` | `4ca468c` | 0 | Reachable from main and archived |
| `buddy5-test` | `128d254` | 15 | Preserved in archive |
| `codex/buddy2-left-motor-test` | `99d1f50` | 100 | Preserved in archive |
| `codex/micro-tkernel-migration` | `75325f2` | 0 | Reachable from main and archived |
| `halt-testing` | `424d8ad` | 0 | Reachable from main and archived |
| `integration/v23-buddy4-review` | `ac161d3` | 101 | Preserved in archive |
