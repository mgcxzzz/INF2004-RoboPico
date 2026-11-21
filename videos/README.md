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

The [21 November team circuit run](../testing/system/logs/2026-11-21_run_4/README.md)
contains the source timestamps and matching log events. The clips below use
v0.4.2 and are ready for the team to share as unlisted videos.

| Clip | Test cases | Unlisted link |
| --- | --- | --- |
| `01-line-follow-and-barcode.mp4` | SYS-12, SYS-13 | Pending |
| `02-hump-crossing.mp4` | SYS-16 | Pending |
| `03-obstacle-bypass-and-rejoin.mp4` | SYS-17, SYS-18, SYS-19 | Pending |
| `04-barcode-junction-actions.mp4` | SYS-14 | Pending |
