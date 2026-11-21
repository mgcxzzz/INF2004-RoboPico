# Team circuit test, run 4

Recorded by the IS28 team on 21 November 2026. Firmware **v0.4.2**, Demo
Console **v0.2.14**, Baseline **45 mm/s**. Mapping was N straight, Q U-turn,
A left and Z right. The demo evidence ends with the completed left turn.

Run 4 serves as the final circuit verification for the behaviours listed below.
The submission firmware, v0.4.3, adds a small adjustment to the stopped sonar
range check and its warning. The change was verified by firmware replay and
dashboard decoding. See the [release verification](../../../integration/logs/2026-11-21_v0.4.3/README.md).

[Selected dashboard events](events.csv) · [MQTT receipt summary](mqtt.txt) · [Source archive identity](capture.json) ·
[System test cases](../../TEST_LOG.md)

## Observed results

| Cases | Result in this run |
| --- | --- |
| SYS-10, SYS-12 | Start ID 54 was accepted and the car followed the straight and bends. The video shows the early tracking section. |
| SYS-13, SYS-14 | The coordinator accepted Z, Q and A and recorded the associated right, U-turn and left as completed. The footage shows each outgoing path. |
| SYS-16 | Buddy 4 accepted 12 mm and later 16 mm, with zero recorded rejections in run 4. Both crossings are visible. A reference height measurement is needed for an accuracy result. |
| SYS-17, SYS-18 | Buddy 5 detected the box at 284 mm and reported a 138.7 mm width estimate. The width trace lacked one record. Ground-truth width was not recorded. |
| SYS-19 | The footage shows the robot go around the box and resume travel on its original line. The coordinator logged `HANDBACK` to line following. |
| SYS-20, SYS-22 | The dashboard received live MQTT and the run ZIP contains all four stored datasets. The retained live history spans the demo. The ZIP includes earlier dashboard activity and reports overwritten older live rows. |

These are functional observations from one circuit run. The formal repeatability,
accuracy and timing targets have their own cases. Straight-through barcode
execution and pickup and inversion are outside this run's scope.

## Recording and log sync

The camera track starts at approximately 17:39:12 Singapore time and the dashboard recording starts one second earlier. At dashboard time 0:01 it displays Pico uptime 239364 ms, received at 17:39:12.170. The first nonzero encoder speed was received at 17:39:12.346. Barcode, hump and junction events provide additional points of comparison.

| Clip and evidence | Robot camera source | Dashboard source | Matching log time (Singapore) |
| --- | --- | --- | --- |
| 01 · Successive bends and S-shaped section | 3:32–4:06 | 3:33–4:07 | Selected motion feedback during continuous tracking |
| 02 · Z decode and right queued | 0:05–0:17 | 0:06–0:18 | Z decoded 17:39:21.891 |
| 02 · Q decode and U-turn queued | 3:20–3:35 | 3:21–3:36 | Q decoded 17:42:41.645 |
| 02 · A decode and left queued | 5:48–6:02 | 5:49–6:03 | A decoded 17:45:07.376 |
| 03 · First hump crossing | 1:07–1:20 | 1:08–1:21 | 12 mm accepted 17:40:27.325 |
| 03 · Second hump crossing | 4:17–4:33 | 4:18–4:34 | 16 mm accepted 17:43:40.435 |
| 04 · Z right turn | 3:09–3:25 | 3:10–3:26 | Right completed 17:42:32.143 |
| 05 · Q U-turn | 4:30–4:47 | 4:31–4:48 | U-turn completed 17:43:54.522 |
| 06 · A left turn | 5:56–6:11 | 5:57–6:12 | Left completed 17:45:22.221 |
| 07 · Obstacle trigger, probe and width | 1:31–1:41, 2:09–2:25 | 1:32–1:42, 2:10–2:26 | Trigger 17:40:44.884, width 17:41:27.931 |
| 07 · Bypass and line rejoin | 2:50–3:13 | 2:51–3:14 | Handback 17:42:22.937 |
| 08 · Live dashboard overview | | 0:16–0:33, 3:11–3:23, 4:25–4:41 | Motion, MQTT heartbeat, right-turn completion and accepted height |

See the [video index](../../../../videos/README.md) for filenames and unlisted links.
