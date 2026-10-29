# v2.3 + Buddy4 add-on v0.1 + ramp-assist v0.1 (test firmware, 2026-10-30)

**Base.** The work starts from the exact working SDK v2.3 at commit
`8d7f3aa4935565636bba013c74e563f5e8135bcb` (worktree `INF2004-RoboPico-v23b4`,
detached), plus the existing Buddy4 add-on v0.1 (`BUDDY4_ADDON_V0_1.md`).

**Scope.** It is a test build only.

- No change to:
  - targets or speed profiles;
  - controller gains or limits;
  - steering, line search, junctions, barcode or the coordinator;
  - stop paths;
  - MQTT transport or commands;
  - calibration commands.
- No RTOS code and no drive-bench code.
- The main repo checkout (buddy4, unfinished merge) is untouched.

**Two variants.** Both are built from the same source:

| UF2 | `RAMP_ASSIST_ACTIVE` | Motors |
|---|---|---|
| `buddy123_interface_wifi_v2_3_b4_v0_1_ra_v0_1_shadow.uf2` | 0 | Original v2.3 output. The logic, timing and telemetry run, and telemetry reports the PWM that *would* be added. |
| `buddy123_interface_wifi_v2_3_b4_v0_1_ra_v0_1_active.uf2` | 1 | Bounded feedforward assist applied. |

Both UF2s contain local Wi-Fi credentials. They are stored only in
`Embedded/bench_tools/buddy4/` and must never go into Git.

## What it does and how it interacts with the existing speed controller

Profile 1 ordinary line following uses the existing continuous drive
controller (`continuous_drive.h`), per wheel, every 10 ms:

```
output = slew_1200/s( clamp_0..420( ff(ref) + (ref - speed) + I ) ),   |I| <= 100, dI = 3*(ref - speed)*dt
ff(ref) = ref * 150/45 (left), ref * 180/45 (right)
```

The assist adds one term: `ff(ref) * boost_frac`, capped at 40 PWM. The change
to the controller is that term and three struct fields that start at zero.

- **Applied to both wheels with the same fraction.** The steering ratio from
  line following is preserved, and the wheel the controller is steering to
  near zero gets near-zero help.
- **It enters the same `base` as the existing feedforward**, so:
  - the existing clamp (0..420), the 1200/s slew, the I limit (±100) and the
    anti-windup rules apply to the total exactly as before;
  - the existing start-assist (up to 300) and the 420 limit stay in force.
- **Effect on the integral.** The assist supplies part of the extra effort
  that the integral would otherwise have to build up. The integral therefore
  grows *less* during a climb, which the host simulation confirms. When the
  assist ends there is no stored assist anywhere: the term is simply gone.
- **Smooth entry and exit.** The fraction ramps 0 → 0.20 → 0 at 1.0/s, which
  is at most 180 PWM/s, far below the 1200/s output slew. The release
  therefore never trips the slew/saturation tracking that would push the
  assist into the integral.
- **Leaving continuous mode.** When the car leaves continuous mode (junction,
  search, manoeuvre), the legacy controller is seeded from the output *minus*
  that step's added PWM (`ramp_assist_unboosted`). The assist is never carried
  into another controller.
- **Normal target speed is unchanged.** Profile 1 is 45 mm/s.

## Activation logic (`ramp_assist.h`, pure C)

**Input.** Buddy4's per-20 ms side estimate (right-minus-left height) and
Buddy4's own validity (MAD, gravity, timing, acceleration, turn, impact
guards; Tracking state; successful FIFO read). These come from
`b4v23_tilt()`, which is read-only. The assist does not require a completed or
accepted ramp event.

**Start.**

- |height| ≥ 4 mm with the same sign on 4 consecutive valid samples (80 ms).
- And: run active, coordinator in Auto (not paused), continuous drive selected,
  not in line recovery, episode not latched, and episode time below the
  timeout.

**End conditions** (with the reason logged):

| Reason | Condition | After it ends |
|---|---|---|
| LEVEL | ≤ 2 mm for 2 samples | ramp-out ≤ 200 ms |
| DESCENT | 3 mm below the activation peak for 2 samples (crest passed, so no downhill help) | latched until level |
| INVALID | Buddy4 tilt invalid for > 40 ms | ramp-out; may re-activate after 80 ms of valid tilt again |
| STALE | no Buddy4 period for > 60 ms | ramp-out |
| TIMEOUT | 2.5 s cumulative assist in one episode | latched until level |
| NOT_FOLLOWING | line search/recovery, junction/turn manoeuvre, external motion, profile 2 | **immediate** zero |
| PAUSED | coordinator not in Auto | **immediate** zero |
| STOPPED | run guard inactive: Stop, ESTOP, faults (encoder, direction, LS timeout…), run end | **immediate** zero (and the controller is reset by v2.3 anyway) |

- **Episodes.** An episode ends after 200 ms level, which clears the latch.
- **Stop behaviour.** All existing stop behaviour is unchanged. The assist
  never prevents or delays a stop: it only reads the guard.

## Constants and the evidence behind them (provisional)

| Constant | Value | Basis |
|---|---|---|
| Assist fraction | 0.20 (+30 L / +36 R PWM at 45 mm/s) | About 1/3 of the existing integral authority (±100). Below the controller's own proportional response at stall (+45). Well inside PWM 420: the observed climb peak was 224 + 36. See the evidence below. |
| Per-wheel cap | 40 PWM | A hard bound. With profile 1 the fraction gives at most 36. |
| Enter / persistence | ≥ 4 mm for 80 ms | Flat positive maximum in both captures was ≤ 2 mm, one valid −5 mm sample on the flat; climb snapshots were 4–10 mm. |
| Exit / descent | ≤ 2 mm; 3 mm drop from peak | Flat band ±2 mm; the upper-bracket descent fell 8 → 3 mm. |
| Timeout | 2.5 s | The rise from about 4 mm to the peak took about 1.5 s (upper bracket) and about 1.7 s (lower bracket, slowed). |
| Invalid / stale | 40 ms / 60 ms | 2 and 3 Buddy4 periods. |
| Ramp rate | 1.0/s (≤ 180 PWM/s) | Below the 1200/s slew, so release cannot wind the integral. |

**Evidence used.** These are Buddy4 source-2 snapshots at 500 ms and SDK
source 9 at 20 Hz.

- **2026-10-30, upper bracket with secured divider, completed 10 mm ramp.**
  - During the climb (heights 4, 5, 5, 8 mm), the right-wheel PWM median was
    173, identical to the flat section's 173. The climb peak was 224 versus a
    flat peak of 222.
  - There was no speed deficit; weaving dominated.
  - Conclusion: that climb needed no extra effort.
- **2026-10-29 run 3, lower bracket ("contact on descent caused stall").**
  - During the climb (heights 5, 6, 7, 10), **both** wheels slowed to
    8–25 mm/s at target 45.
  - The left PWM rose 182 → 272. That equals ff 150 + P ≈ 22 + I at its
    **100 limit**: the controller authority limit, not PWM 420.
  - The car then stalled at 8–9 mm with outputs fixed at 252/115, which equals
    ff(35) + 35 + 100 and ff(3) + 3 + 100.
  - The user attributed this to IR contact.
  - Assistance at 0.20 would raise that ceiling by about 23–30 PWM. That is
    deliberately not enough to drive through hard contact.

**Evidence is not sufficient to finalise the fraction or the tilt thresholds.**
These measurements are missing; the shadow build collects the first two
without changing motion:

1. **Per-sample (20 ms) tilt behaviour with the lower bracket**, on flat line
   following and on the correctly positioned climb:
   - valid/invalid fraction (weaving trips Buddy4's 20°/s turn and 200 mm/s²
     acceleration guards);
   - longest run ≥ 4 mm on the flat (bumps, tile seams);
   - the climb's run length above 4 mm.

   Source-15 periodic frames report valid/invalid counts, min/max and the
   longest run ≥ 4 mm every 500 ms. Snapshots at 500 ms could not show this.
2. **Whether a correctly positioned climb reaches the controller authority
   limit.** Source 15 reports max |I| per wheel and the count of 10 ms ticks
   with |I| ≥ 99. Source 9 reports speed and PWM.
   - If the shadow climb shows no I-limit ticks and no sustained speed
     deficit, the assist is not needed; do not run the active build.
3. **Ramp geometry**: run-up length for the 10 mm rise, so the expected climb
   time can be checked against the 2.5 s timeout.
4. **Lower-bracket clearance**, confirmed unpowered at entry, crest and descent
   with the ramp correctly positioned. This decides whether a stall is torque
   or contact. The assist must not be used to push through contact.

## Telemetry (minimal): source 15, 31-byte frame on `is28/tlm`

- **Sent when:** on activation, on termination (with the reason) and every
  500 ms during a run; one final frame after the run stops.
- **Gated by:** v2.3's telemetry switch.
- **Payload** (little endian, `ramp_assist.h`):
  - uptime;
  - event, build/assisting/latched/valid flags, last end reason, activations;
  - alpha (milli) and added PWM left/right (shadow: would-be);
  - current height, activation peak height;
  - interval max/min height, valid/invalid sample counts, longest run ≥ 4 mm;
  - max |I| left/right, I-limit ticks;
  - active duration, peak added PWM.
- **Decoding:** `python3 ramp_assist_decode.py <native capture> [from [to]]`
  prints CSV.
- **Dashboards:** existing dashboards show source 15 as an undecoded source;
  the native recorder keeps the hex.

## Files changed

**Versus the v0.1 add-on.** The full diff is in `RAMP_ASSIST_V0_1.patch`; the
pre-change copies are in `archive/b4_v0_1/`.

- **`continuous_drive.h`** (byte-identical in v0.1): 3 struct fields and 2 lines
  (`boost_pwm`, added to `base`). The original is in
  `archive/continuous_drive_8d7f3aa.h.txt` (sha256 9eadf64b…, matches
  SOURCE_SHA256.json).
- **`main.c`** (circuit_with_barcode): +3 lines (include,
  `ramp_assist_halt` on inactive, `ramp_assist_control` before the drive
  steps) and 1 changed call (`continuous_leave` seeded with
  `ramp_assist_unboosted`).
- **`wifi_harness.h`**: the v0.1 Buddy4 Reset hook now compiles only with
  `BUDDY4_ADDON=1`. This fixes a **v0.1 host-test regression**: v0.1 broke
  `test_wifi_harness.c` in `run_tests.py`; the firmware was unaffected. The
  firmware hook behaviour is unchanged.
- **`CMakeLists.txt`**: `RAMP_ASSIST_ACTIVE` option, `BUDDY4_ADDON=1`,
  versioned output names.
- **`buddy4/buddy4_service.c`, `buddy4/buddy4_v23.h`**: read-only
  `b4v23_tilt()` view. Buddy4 processing and validated sources are unchanged.
- **New:**
  - `ramp_assist.h`, `ramp_assist_harness.h`
  - `ramp_assist_decode.py`
  - `run_ramp_assist_tests.sh`
  - `ramp_assist_tests/` (test, plus the renamed 8d7f3aa reference controller)

**Versus 8d7f3aa.** 4 of the 185 manifest files differ: CMakeLists.txt,
continuous_drive.h, circuit_with_barcode/main.c and wifi_harness.h. None are
missing.

## Calibration (unchanged commands, but check before driving)

- **Commands.** `w`/`l`/`r`/`k` and `c` (restore preset, while stopped) are
  unchanged. The preset is white L163/R160, black L3457/R3700.
- **What boots.** Flashing keeps the saved v2.3 calibration record (v28),
  which is restored at boot. The car will therefore boot with the **last saved
  calibration (upper bracket + divider: white 191/154, black 1940/2306)**, not
  the preset.
- **Sensor change.** The **left front sensor was replaced** and the bracket is
  now lower. Neither the saved values nor the preset is known to fit.
- Do a stationary white/black check before any driving (staged steps below).
  Earlier findings to keep in mind:
  - optical coupling between the two front modules;
  - a divider-dependent left reading.

## Tests run (host; no hardware, no flashing)

- **`sh run_ramp_assist_tests.sh`** (clang, gcc, gcc ASan/UBSan):
  - 360,841 checks;
  - boost-0 output and state are bit-identical to the 8d7f3aa controller over
    120,000 random steps;
  - logic scenarios;
  - controller-plus-plant limits (420, slew, ±100);
  - after release: no integral excess and no residual boost;
  - climb speed deficit 7.65 → 4.18 mm/s in the simplified plant;
  - telemetry decoder round trip.
- **`run_tests.py`** (existing v2.3 suite, CC=clang): all 35 C executables, JS
  schema tests and Python helpers PASS. Before the Reset-hook fix, the v0.1
  tree failed `test_wifi_harness.c`.
- **`buddy4/host/run_host_test.sh`**: PASS, now also checking the tilt view
  (valid while tracking a 10 mm side tilt; invalid in Idle and Fault; one
  sample per period).
- **ARM builds** of both variants: see the build manifest.

The plant model is a first-order approximation for checking limits and
residuals only. It is not a prediction of the car's climb.
