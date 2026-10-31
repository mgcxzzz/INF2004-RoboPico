# Source provenance

Working source baseline: branch integration/v23-buddy4-review, commit ac161d3bd9e0a7361419747755e6834c4fc98744, folder testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5. Original Continuous V2/V2.3 source reference:8d7f3aa4935565636bba013c74e563f5e8135bcb. Buddy2 evidence source99d1f503d7989f1282c9e0162b9d6183ab3459d9; standalone Buddy4 evidence sourcee6b19b04ec75c38a032608eb51021750e11f7555.

SOURCE_ORIGINS.json maps old file paths to descriptive paths and records pre-change hashes. Only active feature code, tests and required support files are imported; historical firmware copies/patch trees remain on their original branches. The single app main combines the prior diagnostic prefix with the real application body. Include paths are updated; include-main.c wrappers are removed.

Intentional implementation changes: bounded canonical Buddy4 assist API/coordinator policy/expiry/audit; common coordinator approval for physical A–Z results; reset peak partial state on Resume; new source15 C1 requested/effective ramp targets; provisional build labels; setup-path fixes. Steering/optical/junction/decoder and ramp control numeric constants, motor caps/slew, record28 calibration behavior and working network architecture are retained. Test expectations change only where the formerly unsupported assist API is now validated.

New ramp request encoding uses6 bytes with explicit8/13/10/8-bit fields plus7-bit kind/owner header. Value quantization is1milli for fraction/hold and0.01PWM for stall push. No native struct is transmitted. Importing/reformatting these modules is not a claim of a full new minimum-bit audit of every historical subsystem.
