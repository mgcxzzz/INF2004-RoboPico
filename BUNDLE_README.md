# RoboPico Buddy 1–5 RTOS bundle

Open `START_HERE.md` in the extracted v0.2.13 bundle. It gives
the flash, Wi-Fi, dashboard and first obstacle test steps. In a repository
checkout, the same guide is [docs/buddy1_5_v0_2_13.md](docs/buddy1_5_v0_2_13.md).

Flash `firmware/robopico_buddy1_5_rtos_v0_2_13.uf2` and launch the bundled
dashboard. Use **Start dashboard.cmd** on Windows or
`python3 tools/dashboard/run_dashboard.py` on macOS. The image uses the car's
saved Wi-Fi settings. See the [setup guide](docs/setup.md) for a changed network.

The bundle includes the combined firmware, dashboard, source, offline tests
and build manifest. The manifest records the source commit, toolchain, firmware
hash and RAM checks. End-to-end avoidance still needs a run on the robot.
