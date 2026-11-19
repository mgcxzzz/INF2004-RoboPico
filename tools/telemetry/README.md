# Offline message decoders

`source15_decode.py capture.txt` dispatches the validated source15 ramp, peak and provisional requested/effective-target layouts. It accepts native timestamp/topic/hex capture lines or bare hex. Unknown layouts are reported rather than guessed. The separate ramp/peak scripts remain useful for legacy captures. These tools do not connect to MQTT.

After Stop, **Download run ZIP** collects dashboard frames, events and stopped
datasets with the build and calibration identity. The combined profile adds
`dashboard/obstacle.json` to the ZIP. Use `b5_rtos_decode.py` to replay the
Buddy 5 width and direction packets offline. See the
[dashboard guide](../../dashboard/README.md) and [setup guide](../../docs/setup.md).
