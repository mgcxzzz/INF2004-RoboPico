# INF2004 RoboPico

## Experimental micro T-Kernel migration branch

`codex/micro-tkernel-migration` starts from the working main baseline and follows
the [staged RTOS migration plan](firmware/migration/micro_tkernel/README.md).
The current checkpoint is [Step1 motor/encoder/stop bring-up](firmware/migration/micro_tkernel/step_01_motor_bringup/README.md).
Its hardware acceptance is pending. It does not run line following or Wi-Fi yet.
The SDK integration below remains the behaviour reference; it does not fulfil
the project's micro T-Kernel requirement by itself.

## Current development build: PROVISIONAL Buddy1–4 integration

**Buddy5 obstacle avoidance is NOT integrated. This is not a confirmed final build.**

Start with [firmware/integration/buddy1_4](firmware/integration/buddy1_4/README.md), [setup](docs/setup.md) and [integration status](firmware/integration/buddy1_4/STATUS.md). Download the UF2 and complete source/dashboard/setup bundle from its [artifacts folder](firmware/integration/buddy1_4/artifacts/). Feature folders each explain their logic and evidence.

[Recorded Buddy2/Buddy4 evidence](testing/evidence/README.md) is archived with pinned source commits and hashes. It documents earlier builds; current validation is recorded separately.

The original micro T-Kernel project remains at app_program/, kernel/, lib/, config/, device/ and build_make/. It is a separate build with [its original setup guide](docs/rtos_setup.md). Those existing team files are preserved.
