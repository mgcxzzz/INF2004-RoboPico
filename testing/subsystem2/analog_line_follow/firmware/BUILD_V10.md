# Analog line-follow V10 - 21 October 2026

User requested10seconds for profile6 instead of5. Compiled/software-verified;
physical settling over the longer run remains unverified.

Changed only symmetric run limits5s/200mm ->10s/500mm and trace75ms->125ms.
The compile-time budget reserves full run,1s settling and3extra rows within100.
Controller mapping, recovery650ms/25mm, memory500ms, start/rearm, PI,25% PWM cap,
kick and all guards unchanged. All other profiles unchanged.

Checks passed: strict host controller suite and independent deadline boundary
with healthy control timestamps; Pico W Release SDK2.2.0/Arm GCC14.2 with strict
application warnings; saved Trace/SavedLog structure unchanged; symmetric logic
body and historical controller headers unchanged; valid UF2 family/sequence/
headers/payload below0x101ff000. No hardware test at build host.
ELF text63768/data0/BSS12092. UF2 119808 bytes. Existing calibration loads.

SHA256 buddy23_analog_line_follow_v10.uf2:

```
53cd16cfa30434168dd9b71246aa0b3e96ceb3d3b36300e58612d641cea91272
```

FlashV10, h verify valid1,6 then k. Same layout/calibration/load/line as before;
allow continuous line and stopping space for roughly500mm travel. Normal bank
start sequence, one power source; either button stops. Ends at10s/500mm or an
earlier guard. Wait3s after stop, power-source switch separately, p. Send full
log and whether centring improves after5s or continues swinging.
