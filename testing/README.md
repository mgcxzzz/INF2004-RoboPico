# RoboPico testing

Start with the [Tribunal 2 test cases](TRIBUNAL_2.md). They map the 12 functional
and nine non-functional requirements to procedures, pass criteria and evidence.

| What to review | Where |
| --- | --- |
| Final circuit evidence for v0.4.3 | [Run 4 results and synchronized clips](evidence/tribunal2/run-4/README.md) |
| v0.4.3 range tweak and build checks | [Release verification](evidence/tribunal2/v0.4.3/README.md) |
| Integration and system cases | [Integration checks](integration/TEST_LOG.md) · [Physical system results](system/TEST_LOG.md) |
| Subsystem cases | [Buddy 1](subsystem1/TEST_LOG.md) · [Buddy 2](subsystem2/TEST_LOG.md) · [Buddy 3](subsystem3/TEST_LOG.md) · [Buddy 4](subsystem4/TEST_LOG.md) · [Buddy 5](subsystem5/TEST_LOG.md) |
| Earlier tests and raw evidence | [Evidence archive and provenance](evidence/README.md) |
| Video evidence | [Clip index](../videos/README.md) |

## Repeat and record a test

Run the [production C, dashboard and coding checks](TRIBUNAL_2.md#build-and-repeat-the-automated-checks)
from the tested firmware commit. For a physical test, use its setup and success
criterion in the Tribunal 2 table. Start a fresh dashboard session and export the
run ZIP after the car is stopped, before a reset or another Start.

Record the test ID, date, team member, firmware version and commit or UF2 hash,
dashboard version, setup, expected and observed result, and the evidence location.
Use a short video where the log cannot establish the physical result. Keep the run
manifest with the original capture. A new result adds a row so earlier runs remain
traceable.
