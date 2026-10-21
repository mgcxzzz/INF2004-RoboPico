# User-reported analog evidence,21 October2026

Motors-off probe, supply confirmed3.3V, leftA0 GP26/rightA0 GP28. Earlier tests
had A0 incorrectly connected to digital-only GP6/7 and are NOT calibration data.

At the stated normal working height, after correcting those connections:
- Both-white averages: L130..131, R130..132 (subsequent sweeps usually131..134).
- Left-black/right-white: L321..324, R132..134.
- Right-black/left-white: R432..435, L131..134.
- Left black->white slow sweep:369,364,354,343,327,305,292,268,223,184,161,142,132;
  R stayed roughly132..135. Starting black differed from static reading.
- Right sweep: typically441..457 initially, briefly474/503, then443,306,245,202,
  177,162,146,140,133; L stayed129..133.
- White and black within-burst min/max did not overlap in supplied static tests.
  Averaged endpoints are stable enough to try calibrated control, but black
  plateau changes show why calibration must use the final height/lighting.

Geometry reported: black strip20mm wide, sensing points about35mm apart. User
intends to adjust spacing by trial. Calibration/edge alignment must be repeated
after moving sensor brackets or changing height. No actual analog motor run or
curve-following success is claimed yet. Digital integration steering and button
stops were previously checked on raised wheels; its offline start remains open.

## V1 analog motion evidence subsequently received

- Raised USB f: both wheels forward, release stopped at1310ms. Invalid0/0.
  User explained ADC values above1000 occurred when lifting the robot: increased
  height changes reflection. That run is not evidence for fixed-height calibration.
- Calibration at normal height: white127/131, black388/557, mask7/profile1.
  Same values restored=1 after switching power sources: persistence passed.
- Offline mat attempt: arming succeeded and CRC log restored36rows/dropped0.
  L raw296..301, R126..131, targets about74/56mm/s. PWM pulses180permille;
  wheels reached2.22/2.21mm, stalled and no-movement guard stopped at798ms.
  No invalid encoder transitions. User checked for physical obstruction: none.
  This suggests inadequate breakaway under floor load, NOT proven bank failure.
- Earlier V6 300mm floor runs used about180..200permille steady PWM at roughly
  90..100mm/s; they are useful load evidence, not a fixed PWM command for steering.
- User later reported centred left-black raw491..494, right-white134..137.
  Old black388 clips normalized left to1000; ADC itself is not saturated.
  Left black calibration needs recapturing at final height. Do not hardcode493.
