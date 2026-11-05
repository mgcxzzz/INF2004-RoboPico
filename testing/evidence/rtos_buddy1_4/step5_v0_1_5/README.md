# Step5 v0.1.5 promotion evidence

Original user-supplied archive: STEP5_v0_1_5_FLAT_AND_RAMP_2026-11-05.zip.
Firmware source/artifact baseline:75325f29ad604c75a9d9ab8111658aa1536543d5.
This directory preserves the supplied review, timing summaries, USB diagnostics
and ramp controller export used to justify main promotion. MQTT captures and
repeated binary exports remain in the original archive; this is a curated subset.

Flat:0 accepted humps. One nominal10mm one-sided ramp:1 accepted hump,13mm max;
no physical push/lift; normal drive and Buddy4 assistance active. The measured
motion window maintained about50Hz IMU and100Hz motor control, with no new
read/late/FIFO/timeout/budget/log-drop faults. Startup lifetime anomalies are
not erased by that motion-window result. One ramp is not repeatability proof.
Full supported-mat left/right rejoin and sharp bends remain pending. New main
powerbank supervision and wireless exports have software validation only until
the first physical release check. See the integration STATUS.md.
