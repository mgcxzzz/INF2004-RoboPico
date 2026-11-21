# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `SYS-14`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

The [Run 4 record](../testing/system/logs/2026-11-21_run_4/README.md#recording-and-log-sync) lists source ranges and matching events. The robot footage and screen recording are synchronized to about one second. The first seven clips show the relevant live cards beside the robot. Clip 08 shows the wider dashboard.

| Clip | What it shows | Test cases | Unlisted link |
| --- | --- | --- | --- |
| `01-line-following-and-bends.mp4` · 0:34 | Continuous travel through successive bends, line state and live wheel speed | SYS-12 | [Watch clip 01](https://youtu.be/DSKYzSl4pN8) |
| `02-barcode-scans-z-q-a.mp4` · 0:41 | All three recorded decodes with their queued actions | SYS-13 | [Watch clip 02](https://youtu.be/gGe0tv3TXEg) |
| `03-two-hump-crossings.mp4` · 0:29 | Both physical crossings, with accepted peaks of 12 mm and 16 mm | SYS-16 | [Watch clip 03](https://youtu.be/LSbH6W17-fk) |
| `04-right-turn.mp4` · 0:16 | Z, right turn and outgoing tape | SYS-14 | [Watch clip 04](https://youtu.be/u8M4bE7olWs) |
| `05-u-turn.mp4` · 0:17 | Q, U-turn and outgoing tape | SYS-14 | [Watch clip 05](https://youtu.be/aMheU_oI-9U) |
| `06-left-turn.mp4` · 0:15 | A, left turn and outgoing tape | SYS-14 | [Watch clip 06](https://youtu.be/lvsAOKSMIwc) |
| `07-obstacle-detection-bypass-and-rejoin.mp4` · 0:49 | Stationary scan, width estimate, bypass and handback | SYS-17, SYS-18, SYS-19 | [Watch clip 07](https://youtu.be/0P49ioC_46U) |
| `08-dashboard-live-telemetry.mp4` · 0:45 | Live MQTT and motion feedback, obstacle handback, junction actions and accepted height | SYS-20 | [Watch clip 08](https://youtu.be/v_6Jqraz99s) |

Firmware **v0.4.2**, Demo Console **v0.2.14**. The mapping included **N → straight**. The captured evidence contains Z, Q and A, and no N decode. See the [v0.4.3 verification record](../testing/integration/logs/2026-11-21_v0.4.3/README.md) for the release replay.
