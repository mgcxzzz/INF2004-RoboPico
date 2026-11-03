# Test Step 2 v0.1.1 — ADC correction first

Date: 2026-11-03. **Experimental; corrected build not yet tested on the car.**
Use **robopico_rtos_line_following_v0_1_1.uf2**, not the old v0.1 image and not
the similarly numbered Step 1 motor-bringup image.

## What changed

The old reader could declare a false timeout if RTOS preemption happened
between a not-ready check and its 100 us deadline check. A host reproduction
returned failure with READY=1 and ERR=0. The corrected reader checks READY again
at the original deadline. It still rejects a genuinely unfinished conversion
and a hardware ERR; interrupts remain enabled during conversion waits.

The ADC now waits for readiness after reset/enable. Failed reads retain the
first and latest reason, channel, discard/sample stage, sample index, CS,
elapsed microseconds, hardware clock/divider and reset state. All three read
counters are reported so freshness does not depend on seeing noisy raw values.

A failure during motion latches a stop, with supervisor reason **11 (ADC)**.
While stopped, the owner can reset/retry the ADC at most every 100 ms. Readiness
requires **three consecutive complete frames** freshly reading all three inputs.
A recovered runtime fault still requires **x**, then a new **g**; it never resumes
motion automatically. Failed front pairs are not half-published. Calibration
capture/save is blocked while acquisition is unavailable.

Steering, speed targets, optical thresholds, decoder/junction algorithms,
navigation APIs/coordinator, saved calibration priority and flash format are
unchanged. No calibration or Wi-Fi settings are written by this update/test.

## Exact first test — stopped, USB only

1. Close miniterm/PuTTY. Keep the motor battery disconnected for this idle test.
2. Enter BOOTSEL. Copy **robopico_rtos_line_following_v0_1_1.uf2** from the new
   bundle's top-level `firmware` folder onto RPI-RP2. Let it reboot.
3. Open the same USB console at **115200**, 8N1, flow control **None**.
   Confirm the banner says **RTOS LINE FOLLOWING v0.1.1**.
4. Wait about 5 seconds. Press **p**. Expect:
   - `CLOCK ... status=PASS`
   - `CONTROL ... flags=0xc active=0 usb=3`
   - `LINE ... flags=0x3`
   - `ADC state=2 good=3 ...`
   - `timeouts=0 errors=0 init_errors=0 bad_arguments=0`
   - `ADC_FIRST reason=none` and `ADC_LAST reason=none`
   - `CAL_FRONT ... valid=1` with the same saved references. On the reported
     test car these were **white=122,138 black=246,267 origin=1**.
5. Do **not** press g, w, l, r, k or c during this first test.
6. Close miniterm with **Ctrl+]**, or close PuTTY. Keep USB connected.
7. Extract the complete bundle. In a terminal whose current directory is the
   extracted bundle, run the read-only helper below. It sends **no commands**.

Mac (the tested friend's port):

```bash
python3 tools/rtos_migration/capture_adc_readiness.py --port /dev/cu.usbmodemMTK3PICO1
```

If the device name changed, run `ls /dev/cu.usbmodem*` and use the listed port.
To choose from the helper's list, omit `--port ...`. Use the Python environment
that already runs `serial.tools.miniterm`; it already has pyserial.

Windows: double-click **Check ADC readiness.cmd** and enter the current COM
port, or run:

```powershell
py -3 tools/rtos_migration/capture_adc_readiness.py --port COM4
```

8. The capture lasts about **72 seconds**, including a 2-second buffer drain.
   During seconds **20–40**, move white and black surfaces beneath the **left
   front**, **right front** and **side/barcode** sensors, one at a time.
   Keep the wheels stationary. This checks changing readings, not calibration.
9. When it finishes, send both files from **rtos_adc_session**:
   **`_usb.txt`** and **`_summary.json`**. Send them even if the result says REVIEW.

PASS means the helper observed at least 60 seconds of ready acquisition,
advancing reads on every channel, valid calibration, stationary motors and
passing clock/readiness checks with no recorded ADC failures. It does not
certify line following or sensor contrast. A nonzero **avoided_timeout** count
is allowed: the reader resumed late but found a completed, error-free conversion.
Any genuine timeout/ERR/reset failure remains REVIEW even if recovery succeeds.

If Python says pyserial is missing, use the existing miniterm environment or
install in a local environment on Mac:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install pyserial
python tools/rtos_migration/capture_adc_readiness.py --port /dev/cu.usbmodemMTK3PICO1
```

Manual alternative: keep console session logging enabled for at least 70 seconds,
move the surfaces during the middle of the capture, and press **p** at the end.
Send the entire console file. A screenshot alone may omit the first failure.

## Only after the idle test passes

1. Close the helper, connect the normal motor battery and reopen the USB console.
2. Press **p**; wait again for clock PASS, LINE flags0x3 and released buttons.
   If a previous emergency latch exists, release buttons and press **x**.
3. On a short straight with a loose USB cable, centre the car and press **g**.
   Expect a two-second countdown, then following. Press **s** after 3–5 seconds.
4. Stop immediately if it loses the line. Keep USB power connected.
5. Close the console and run the stopped log exporter:

```bash
python3 tools/rtos_migration/export_line_log.py --port /dev/cu.usbmodemMTK3PICO1
```

Windows can use **Export line log.cmd**. Send `_line.csv`, `_summary.json`,
`_usb.txt` from `rtos_line_session` plus the video before starting another run.
Then progress to bends and bare-cross junction tests in START_HERE.md.
Wi-Fi/dashboard, physical barcode decoding and IMU/ramp are still later stages.

## Diagnostic decoding

| Field | Meaning |
|---|---|
| ADC state 0 / 1 / 2 | Offline / qualifying / ready. |
| good | Complete consecutive qualification frames, required value3. |
| ch0 / ch1 / ch2 | Successful averaged reads: left GP26 / side GP27 / right GP28. |
| LINE raw | Left, right, side order; different from numerical ADC channel order. |
| reason=ready_timeout | READY still0 after deadline and final recheck. |
| reason=conversion_err | Hardware ERR on the completed conversion. |
| reason=reset_timeout / init_ready_timeout | Reset-DONE or post-enable readiness did not arrive. |
| stage 0 / 1 / 2 / 3 | Initialization / discarded first conversion / averaged sample / invalid argument. |
| channel=7 | No input selected: initialization or invalid-input diagnostic. |
| sample | 0 for discarded/init stage; 1–32 for averaged sample. |
| ADC_FIRST / ADC_LAST | First and most recent failure; retained across recovery until reboot. |

## Software checks

12 strict production-C programs and 34 Python checks pass, including the actual
portable ADC reader, completed conversion after modeled preemption, true timeout,
ERR both with/without preemption, failed-output preservation, timer wrap and
stopped-only three-frame recovery policy. Existing steering/coordinator/search/
U-turn tests continue passing. ARM firmware builds with strict warnings.
This is software evidence; the physical Pico's original failure branch still
requires these new diagnostics if it recurs. See `logs/v0_1_1` and the manifest.
