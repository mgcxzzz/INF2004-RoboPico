# Offline message decoders

`source15_decode.py capture.txt` dispatches the validated source15 ramp, peak and provisional requested/effective-target layouts. It accepts native timestamp/topic/hex capture lines or bare hex. Unknown layouts are reported rather than guessed. The separate ramp/peak scripts remain useful for legacy captures. These tools do not connect to MQTT.

For complete driving diagnosis export motion, detailed drive, navigation, raw sensors and messages/events, with build/calibration identity. See firmware/integration/buddy1_4/src/telemetry/README.md and docs/setup.md.
