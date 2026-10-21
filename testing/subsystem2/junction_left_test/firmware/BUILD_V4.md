# Junction left V4 — 22 October 2026

Artifact buddy23_junction_left_v4.uf2:117248bytes,229blocks.

SHA256:

```
cb342c49e033679b1b07c7e219b8cf819b4bbdae1574d7be01ddddeec9252b04
```

User requested stronger effort after V3 left wheel stalled at30% while right
kept moving. User confirmed sensor housings scrape the mat; support ball free.
V4 is an explicit bounded PWM trial, not a measured torque command or assurance
that increased output overcomes drag. No hardware actuation at build host.

Turn only: one400permille pulse per wheel at first nonzero phase4 demand;
nominal150ms, released early at along-speed>=25mm/s or wrong-direction speed.
No repeat pulse if the turn stalls again. Sustained PI max350, loaded feedforward
310. Zero clears PI, kick and peak state. Both directions stop/settle before and
after turn. Forward controller/output remain250permille and unchanged from V11.

Absolute peak deadline is checked at motor commit and independently by the1ms
timer ISR; expired output is clamped to350 on the next timer tick (nominal150ms
plus timer resolution). Integer-only ISR path. Main cannot restore an expired
400 command. Late-control>=100ms/button/overall timeout still cut all outputs0.
The existing500ms no-motion-under-demand, wrong direction, invalid encoder,
6s/110degree turn and25s/900mm whole-run limits remain active.

Checks passed:

- Strict host C11 -Wall -Wextra -Werror -O2: prior junction sequence/branch/guard
  suite, plus both signed peak400/end-at150ms/early release/no retrigger/reset.
- Expired/stale400 command clamped350 for both signs; sustained stationary
  demand reaches350 after peak; overspeed reduces PI; bridge never drives both
  channels, forward<=250, turn<=400, no hidden shared-helper250 clipping.
- Forward PI/boost/bridge outputs exactly match V11 across speed/stop/start changes.
- V1-layout sector offsets/Trace36bytes preserved, SavedLog<=4096. V4 accepts
  prior version fields, fresh runs tag4. Peak events add trace samples within budget.
- Pico W Release SDK2.2.0, Arm GCC14.2.1, app -Wall -Wextra -Werror.
- ELF text62568/data0/BSS12232; firmware end0x1000e46c below log sector0x101ff000.
- UF2 headers/sequence,RP2040 family0xe48bff56,256byte payloads and all flash
  payload addresses verified below reserved sector. USBstdio present, UART absent.

Reflash,h verify v4 and valid1/side_valid1,2,k,h verify mode2. Keep same sensor
height for the comparison so prior calibration applies. Same bank/mat sequence,
one attempt then wait3s after stop and retrieve p after power-source switch.
Earlier restored traces retain V1/V2/V3 labels. Physical V4 turn still pending.
