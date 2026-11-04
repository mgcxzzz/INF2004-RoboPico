# INF2004 RoboPico

## Experimental micro T-Kernel migration branch

`codex/micro-tkernel-migration` starts from the working main baseline and follows
the [staged RTOS migration plan](firmware/migration/micro_tkernel/README.md).
The current checkpoint is [Step4 connected barcode](firmware/migration/micro_tkernel/step_04_barcode/README.md):
Continuous V2 profile1, kernel-owned Wi-Fi/MQTT, both reference barcode decoders
and live mapped junction actions. **USB supervision is required; new network
and Step4 hardware acceptance is pending.** The bundle includes a barcode-free
Step3B network comparison image. User reports Step3B v0.1.1 motion/safety tests
passed; Step2 extended-mat L/R/bends remain pending. IMU/ramp is Step5 and Buddy5
is deferred. This branch is not a final Buddy1–4/5 release.
The SDK integration below remains the behaviour reference; it does not fulfil
the project's micro T-Kernel requirement by itself.

## SDK behaviour reference: PROVISIONAL Buddy1–4 integration

**Buddy5 obstacle avoidance is NOT integrated. This is not a confirmed final build.**

Start with [firmware/integration/buddy1_4](firmware/integration/buddy1_4/README.md), [setup](docs/setup.md) and [integration status](firmware/integration/buddy1_4/STATUS.md). Download the UF2 and complete source/dashboard/setup bundle from its [artifacts folder](firmware/integration/buddy1_4/artifacts/). Feature folders each explain their logic and evidence.

[Recorded Buddy2/Buddy4 evidence](testing/evidence/README.md) is archived with pinned source commits and hashes. It documents earlier builds; current validation is recorded separately.

The original micro T-Kernel project remains at app_program/, kernel/, lib/, config/, device/ and build_make/. It is a separate build with [its original setup guide](docs/rtos_setup.md). Those existing team files are preserved.
