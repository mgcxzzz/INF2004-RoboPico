# Buddy 5 integration handoff — V1.3 test, same V1 API

**V1.3 FAILED its physical follow-up:** the29.484s run averaged11.30mm/s with repeated stop/surge/steering oscillation. Do not treat it as a smoothness improvement. See [V1.3 physical regression](V1_3_PHYSICAL_REGRESSION.md). V1.1 remains the comparison baseline; V1.3 source/artifact and original test rationale below are retained for diagnosis.


V1.3 is a new forward-effort test candidate, pending physical validation. Use V1.1 source commit `72e11ce` / its retained UF2 for the comparison baseline. V1.2 failed physically and is not recommended. Current source restores V1.1 optical/speed policy and changes ordinary forward actuation only; API/ownership signatures are unchanged. See [V1.3 scope](FORWARD_EFFORT_V1_3.md).

**Mandatory startup fix:** use V1.1 or later firmware/source, not original V1. If you already added Buddy5 code, bring in the updated `nav_coordinator.c` and rebuild your integrated UF2; the API and record layouts are unchanged. See [confirmed root cause and regression](STARTUP_FIX_V1_1.md).

Use this folder as the integration baseline. It retains Buddy 2/3 V12.2 optical/barcode/turn logic with V1.3 ordinary forward-effort changes, and its standalone line/barcode/junction circuit test. Build Buddy 5's ultrasonic sensing and obstacle planner into the same executable. The supplied UF2 has the weak hooks only, so it follows the circuit normally and has no obstacle detection yet.

## Start and test the baseline

Flash `firmware/buddy23_interface_v1_3.uf2`. Open the Pico USB serial port in PuTTY (115200 is a convenient setting; USB CDC has no physical UART baud dependency). Press `h`; check the Interface v1.3 banner and valid front calibration. Press `1`, then `k`. Calibration `w/l/r/k` is needed only if the front references are invalid or mounting/material changed; the offset barcode sensor remains A0 on GP27 with automatic decoding.

USB: `g` gives a two-second hands-free start. `f` runs with GP21 held. Either button or `s` stops. Power bank: boot, wait five seconds, both buttons three seconds, release, wait one second, tap GP21, wait two seconds. Stop and wait five seconds before changing power. `j` gives barcode results, `p` motion, `a` raw barcode ADC, and `z` the independent coordinator audit. Saved logs restore after reboot. Use profile 1 initially.

## Integration contract

Include `nav_api.h`. Call APIs from a short non-blocking poll/task, never an ISR. Do not write motor pins or create another motor-control loop. Do not block in `sleep_ms`, an ultrasonic echo wait, logging, or a loop waiting for a command; use timestamped sensor capture and a state machine. The existing control tick is 10 ms; late control still stops the robot.

Implement strong definitions of `buddy_extension_init()` and `buddy_extension_poll(uint64_t now_us)` from `buddy_hooks.h`. They run on core 0 inside the existing standalone main. Initialize your sensor in init; poll advances sensing/planning without blocking. The default weak functions do nothing. No Wi-Fi/RTOS/second-core infrastructure is enabled by this handoff.

Recommended sequence:

1. When your validated obstacle decision triggers, call `nav_request_takeover(MOTION_BUDDY5, &lease)`. `OK` cuts drive immediately and freezes line/barcode maneuver progression. It is not a guarantee the robot is already stationary.
2. Wait for `nav_get_status().mode == MOTION_PAUSED` and `.stopped`. Both measured wheel speeds must stay below 5 mm/s for 120 ms. Failure to settle within two seconds enters a fault.
3. Submit one finite movement. Store its returned command ID; wait for `nav_get_command(id, &result)` to report `MOTION_CMD_DONE` before advancing the avoidance plan. `CANCELLED` or `FAILED` is not completion. Results retain the latest 16 finished commands.
4. Continue with reverse/turn/forward steps supplied by your obstacle planner. Keep monitoring your sensor; if a new hazard appears, call `nav_request_stop(MOTION_BUDDY5)`. It cancels movement and retains the lease. A new takeover request can recover the same lease while stopping/paused.
5. Return the robot to the original route with a front sensing spot over the stripe and the obstacle clear. Request `nav_resume_line(MOTION_BUDDY5, lease, false)`. Resume requires a measured stop plus either front normalized level >=400 continuously for 150 ms. `NEED_LINE` means hold and reposition using your planner; it does not autonomously search.
6. If takeover interrupted a planned junction/U-turn, resume returns `NEED_RECOVERY`. Your planner must explicitly decide what to do; passing `true` abandons the stale maneuver rather than finishing it from the wrong position. A fully decoded command still pending before the takeover is preserved. A route already consumed by an interrupted turn is not magically restored.
7. Resume invalidates the old lease and gives control back to Buddy 3. Reset your planner state. A manual/button/USB safety stop ends the entire session; APIs cannot restart it without the operator's normal start sequence.

While an external owner controls the robot, the built-in barcode decoder and optical maneuver state are frozen. Partial barcode frames are discarded on handback; do not plan an avoidance path that depends on decoding a barcode halfway through takeover. New `nav_submit_barcode` requests return `BUSY` during avoidance.

## Movement parameters

```c
uint32_t lease, id;
MotionResult r = nav_request_takeover(MOTION_BUDDY5, &lease);
/* On later polls, after PAUSED/stopped: */
r = nav_drive_distance(MOTION_BUDDY5, lease, -60.0f, 40, &id); // reverse 60 mm
/* Wait for this id's DONE, then submit the next planner step: */
r = nav_turn_relative(MOTION_BUDDY5, lease, -45.0f, 45, MOTION_ROLLING, &id); // RIGHT
/* After the planner returns to the stripe and verifies clearance/stopping: */
r = nav_resume_line(MOTION_BUDDY5, lease, false);
```

These are separate state-machine steps, not a sequence to issue immediately in one function. Acceptance `OK` is not movement completion.

| Command | Range / meaning |
|---|---|
| Drive distance | +forward / -reverse, magnitude 1..1000 mm, 0.1 mm request resolution |
| Drive speed | 20..60 mm/s; 0 selects 45. Heading trim can adjust individual wheel targets by up to 8 mm/s |
| Relative angle | +LEFT / -RIGHT, magnitude 1..180 degrees, 0.1 degree request resolution |
| Turn wheel speed | 20..90 mm/s; 0 selects 45; tapers near target |
| `MOTION_ROLLING` | Inner wheel off, outer wheel forward; the axle translates around the inner wheel |
| `MOTION_IN_PLACE` | Counter-rotating wheels; new/untested on the real robot and may stall with floor drag |
| Completion | Encoder-relative goal within 1 mm or 1 degree, then <5 mm/s for 120 ms |
| Bounds | Deadline based on requested travel/speed, capped at 60 seconds; heading/direction/settling checks remain |

Distances/angles are encoder estimates, not externally measured position. Wheel slip, sensor scraping, turning radius and track width affect real motion. The executor uses the existing 110 mm wheel separation. Generic drive uses the existing normal controller/cap (30%); generic turns use V12 maneuver power (60% sustained, <=75% bounded 180 ms start). These caps apply only when an external primitive is active; normal V12.2 navigation keeps its existing power policy. No automatic ramp boost is added.

## Example source and build

`examples/buddy5_extension.c` is an opt-in dispatcher example. Its weak sensor/planner inputs default to false, so it will not trigger obstacle motion on its own. Supply real implementations of `buddy5_obstacle_detected`, `buddy5_obstacle_clear`, `buddy5_next_step`, and optionally `buddy5_step_completed`. Steps can be DRIVE, TURN, or RESUME. Replace this dispatcher entirely if your existing Buddy 5 state machine already performs that role.

With the Pico SDK, ARM GCC, CMake and Ninja installed, from this folder:

```powershell
cmake -S . -B build-buddy5 -G Ninja -DPICO_SDK_PATH="C:/path/to/pico-sdk" -DCMAKE_BUILD_TYPE=Release -DBUDDY_EXTENSION_SOURCE="C:/absolute/path/to/buddy5_extension.c"
cmake --build build-buddy5 --parallel 4
```

The target is `buddy23_interface_v1`; the resulting UF2 is in your build folder. Add extra sensor/planner sources and SDK libraries with `target_sources` / `target_link_libraries` in this folder's CMakeLists as needed. For the supplied example, pass the absolute path to `examples/buddy5_extension.c` and add your strong sensor/planner implementation source. Keep the standalone `main.c`; do not link a second `main`.

First test baseline following, then an obstacle-triggered stop only, then one reverse, then one rolling turn, then the complete path and resume. Host simulation checks command execution and the example workflow; real avoidance clearance/turn accuracy still needs physical testing. Keep `z` together with a short `p` capture to correlate accepted requests with measured wheels/PWM.

## Wiring ownership and future buddies

Existing pins: motors GP8..11, encoders GP0/1 and GP4/5, front A0 GP26/28, barcode A0 GP27, buttons GP20/21. GP19 is unused by this barcode version. Check your ultrasonic/servo pins against the group's full wiring map; this handoff does not allocate new pins.

Buddy 4 can call `nav_report_terrain(MOTION_BUDDY4, MOTION_ASCENDING)` or FLAT/DESCENDING, but V1 records telemetry only. `nav_request_speed_assist` returns `UNSUPPORTED`; no IMU decision silently changes the frozen driving behavior. Report fault bits on changes, not every poll. Agree one responsible source per bit before integration; clearing a shared bit is a global clear, not a per-reporter reference count. Fault reporting alone does not stop motors: make an explicit stop request if your policy requires it.

The coordinator state/motor commit uses a lock, but this standalone flash-saving harness is validated for cooperative core 0 only. Adding a second core or RTOS needs flash-safe lockout, an appropriate lock/port, and scheduling/timing validation. Calls are local C APIs inside one UF2; MQTT and other transport adapters are future integration work.
