# PROVISIONAL integration status

Release: robopico_buddy1_4_integration_v0_1. This is not the final five-buddy system.

| Feature | Included | Evidence status |
|---|---|---|
|Buddy1 Wi-Fi/MQTT/dashboard|Yes|Working baseline retained; new image needs hardware heartbeat-under-motion check|
|Buddy2 motion/encoders|Yes|Original zero-assist parity and host checks; new image needs driving check|
|Buddy3 line/barcode/junction|Yes|Captured Z/Y replays and old combined hardware records; new full-course check pending|
|Buddy4 IMU/ramp assistance|Yes|Old combinedv0.5 ramp successes retained; new coordinator route needs physical acceptance|
|Buddy5 obstacle avoidance|No|Empty extension hooks and example only|

New image acceptance checklist: stationary startup/motor-off and saved-calibration readback; flat/bend; both-direction mapped barcode then junction; ramp; Stop/ESTOP and Pause/Resume during an attempted scan; sustained heartbeat while driving. Existing software tests and the ARM build are recorded under testing/evidence/current and artifacts/BUILD_MANIFEST.json.

Known limits: ramp tuning is from one car and a10mm one-wheel ramp; speed hold limits steering on a ramp; peak decoding assumes the printed label geometry and blurred response; raw history contains only first2048 samples. Saved car calibration is preserved; the tilted-front calibration is not silently substituted for the built-in preset.
