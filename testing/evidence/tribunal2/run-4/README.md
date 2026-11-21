# Team circuit test, run 4

Recorded by the IS28 team on 21 November 2026. Firmware **v0.4.2**, Demo
Console **v0.2.14**, Baseline **45 mm/s**. Mapping was N straight, Q U-turn,
A left and Z right. The demo evidence ends with the completed left turn.

[Selected dashboard events](events.csv) · [Source archive identity](capture.json) ·
[Tribunal 2 test cases](../../../TRIBUNAL_2.md)

## Observed results

| Cases | Result in this run |
| --- | --- |
| T2-01, T2-02 | Start ID 54 was accepted and the car followed the straight and bends. The video shows the early tracking section. |
| T2-03, T2-04 | The coordinator accepted Z, Q and A and recorded the associated right, U-turn and left as completed. The footage shows each outgoing path. |
| T2-05 | Buddy 4 accepted 12 mm and later 16 mm, with zero recorded rejections in run 4. The first crossing is visible. A reference height measurement is needed for an accuracy result. |
| T2-06 | Buddy 5 detected the box at 284 mm and reported a 138.7 mm width estimate. The width trace lacked one record. Ground-truth width was not recorded. |
| T2-07 | The footage shows the robot go around the box and resume travel on its original line. The coordinator logged `HANDBACK` to line following. |
| T2-09, T2-10 | The dashboard received live MQTT and the run ZIP contains all four stored datasets. The retained live history spans the demo. The ZIP includes earlier dashboard activity and reports overwritten older live rows. |

These are functional observations from one circuit run. The formal repeatability,
accuracy and timing targets have their own cases. Straight-through barcode
execution, pickup and inversion, and the latest firmware smoke test need
separate hardware evidence.

## Recording and log sync

The dashboard recorder starts during the Ready countdown. At video 0:01.00
it displays Pico uptime 239364 ms, received at 17:39:12.170 Singapore time.
The first nonzero encoder speed was received at 17:39:12.346, corresponding
to dashboard video time about 0:01.1. The 12 mm hump, obstacle handback and
three junction turns provide additional sync points. The in/out points
below are source timestamps with lead-in and tail around each action.

| Evidence | Robot camera source | Dashboard source | Key log time (Singapore) |
| --- | --- | --- | --- |
| Line following and Z detection | 0:00–0:39 | 0:00–0:34 | Z accepted 17:39:21.891, bend confirmed 17:39:30.247 |
| First hump | 1:04–1:20 | 1:12–1:21 | 12 mm accepted 17:40:27.325 |
| Obstacle detection and width | 1:28–1:37, 2:15–2:33 | 1:32–1:38, 2:15–2:21 | Trigger 17:40:44.884, width 17:41:27.931 |
| Obstacle bypass and rejoin | 2:50–3:13 | 3:08–3:14 | Handback 17:42:22.937 |
| Z right | 3:10–3:23 | 3:14–3:23 | Right completed 17:42:32.143 |
| Q U-turn | 4:31–4:45 | 4:34–4:46 | U-turn completed 17:43:54.522 |
| A left | 5:52–6:11 | 5:55–6:12 | Left completed 17:45:22.221 |

The local evidence clips use the robot camera. The obstacle and turn clips
join the time ranges above and label each cut. See the [video index](../../../../videos/README.md)
for the clip names and sharing guidance.
