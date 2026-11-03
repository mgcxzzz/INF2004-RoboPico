# User-reported Step2 v0.1 ADC readiness failure

Reported 2026-11-03; artifact commit ae0de5569c609d2f89c25163609fca67462a127c.
UF2 SHA256: 1a7920a9726ccf3d3eeda839922d7dfe8b1042450d3c5149477b99f67c322b7e.

Mac, USB-only, 115200; no successful driving run. Clean boot acquired valid
samples with LINE flags0x3 through the approximately4s report, then flags0x2
by approximately5s. Raw readings froze. CLOCK remained PASS, control flags0xc,
USB3 and released buttons. Imported SDK calibration remained valid:
white122,138 black246,267 origin1. Earlier session rejected g twice.

The old image did not log the ADC failure channel or branch, so neither an
actual READY timeout nor ERR can be established from those hardware logs.
Code review and an offline reproduction exposed a false-timeout path after
modeled task preemption. v0.1.1 fixes that path and adds first/last snapshots.
It does not claim the original physical failure branch has been identified.

Original user console files and teammate review are retained locally, not
copied into the public repository. Hardware acceptance of v0.1.1 is pending.
