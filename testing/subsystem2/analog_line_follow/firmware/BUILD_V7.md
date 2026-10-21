# Analog line-follow V7 - 21 October 2026

Compiled and software-verified. Experimental profile4 physical test pending.

## User-requested experiment

Left black (including normalized1000) means equal45/45mm/s. Enter straight at
left>=850, exit at<=650; intermediate levels retain the previous state. When
left fades/rightwhite request30/60; right>=650 requests60/30; both>=750 gets
35/35 then120ms stop; both<150 gets left recovery bounded650ms/40mm. Reuses the
existing optical guard logic. Assumes strip is LEFT during white recovery;
does not infer physical centre or resolve all two-sensor ambiguities.

Select4 then k. Start left>=850/right<=450, calibration valid and released/rearmed.
Profile4 hard bound3s/150mm. All button, USB, encoder, direction, no-motion and
control-late stops retained. Existing PI,25% PWM cap and startup kick unchanged.
Profiles1/2 retain V6 edge rule; profile3 retains its diagnostic stages.

## Verification

- Host C11 strict warnings: plateau straight,850/650 hysteresis, right assist,
  all-white timeout/distance, both-dark stop, invalid calibration, start gates,
  full-range sampled demands in both hysteresis states; prior edge/response/PI/
  kick/rearm/mode/stop tests passed.
- Pico W SDK2.2.0 Arm GCC14.2 Release, application -Wall -Wextra -Werror.
- ELF text59328/data0/BSS12068; USBstdio on, UART off.
- UF2 110592 bytes; valid RP2040 family/headers/sequence; payload below reserved
  flash sector0x101ff000.
- Trace/SavedLog structure/version unchanged; profile4 extends selected-profile
  range to3/recorded-reserved byte4. Old records load. Older firmware rejects
  selected profile4: choose1 then k before downgrade if preserving calibration.
- Source comparison confirms old analog_demand/response_demand/breakaway/PI
  unchanged; new profile only dispatched when selected4. Both USB preflight
  and actual start enforce profile4 alignment. Short deadline/distance verified.
- No hardware validation or automatic centre-finding claim.

SHA256 buddy23_analog_line_follow_v7.uf2:

```
9e138d23f795c1e588bc22b4b2a30557db4a0b47c76c266491e09733dc8095ed
```

First test: short STRAIGHT, left fullyblack/rightwhite. h confirms V7/valid1;
4 then k; computerUSB OFF before bank ON; wait5s, BOTH3s, release BOTH, wait1s,
GP21tap/release,2s countdown. Either button stops. Wait3s after stop, bank OFF,
computerUSB ON; PuTTY1152008N1 flowNone, p. Send full log and movement observation.
