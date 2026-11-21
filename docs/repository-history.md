# Development branch audit

Checked against the remote heads on 21 November 2026. The combined build, CI and current test evidence are available from `main`. Test ledgers use repository-relative evidence paths. Historical manifests retain the branch and commit names under which their artifacts were made.

The table counts commits reachable from each branch tip that are outside `main`. A zero count means its full Git history is reachable from `main`. A nonzero count means retain that history under a permanent ref or a verified archive before removing the branch. The counts overlap where development branches share commits.

| Remote branch | Tip | Commits outside main | Retirement assessment |
| --- | --- | ---: | --- |
| `origin` | `9ce68f3` | 0 | History is on main |
| `barr-c-cleanup` | `81a994f` | 0 | History is on main |
| `barr-c-combined` | `10d05a2` | 0 | History is on main |
| `buddy1` | `b094030` | 3 | Archive unique history first |
| `buddy3` | `74e0c5c` | 10 | Archive unique history first |
| `buddy4` | `e6b19b0` | 2 | Archive unique history first |
| `buddy5` | `1a0b559` | 14 | Archive unique history first |
| `buddy5-rtos-integration` | `573689b` | 1 | Archive unique history first |
| `buddy5-rtos-v0.2.12-review` | `89e7fc6` | 0 | History is on main |
| `buddy5-single-scan-alignment` | `4ca468c` | 0 | History is on main |
| `buddy5-test` | `128d254` | 15 | Archive unique history first |
| `codex/buddy2-left-motor-test` | `99d1f50` | 100 | Archive unique history first |
| `codex/micro-tkernel-migration` | `75325f2` | 0 | History is on main |
| `halt-testing` | `424d8ad` | 0 | History is on main |
| `integration/v23-buddy4-review` | `ac161d3` | 101 | Archive unique history first |

Confirm with the owners that the branches are no longer active before retiring them. The [evidence index](../testing/README.md) points to the standalone documents and selected logs preserved on main.
