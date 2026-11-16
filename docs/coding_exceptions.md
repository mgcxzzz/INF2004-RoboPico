# RoboPico C Standard 1.0: exception register

Accepted basis: project owner's 17 November 2026 approval of an adapted Barr-C
standard. These exceptions are deliberately limited; they do not waive all
requirements for the files listed. New exceptions need an explanation and review
in the same change. No full unmodified Barr-C certification is asserted.

| ID | Barr rules | Scope and reason | Required safeguards |
| --- | --- | --- | --- |
| CEX-01 | 1.1.a, 1.1.c | GNU C11 and necessary GCC atomics/attributes instead of strict ISO C99. `_Static_assert` protects layouts; atomic builtins preserve RTOS communication. | Pin compiler/options; retain size/range assertions. Keep hardware assembly in platform adapters and review ordering. |
| CEX-02 | Naming and source-style rules; 6.5.a | Upstream kernel/BSP, SDK, CYW43, TinyUSB, lwIP and their required public ABI names/types. Existing `usermain`, kernel integer typedefs and kernel high-level interrupt registration are retained. | Do not change upstream calling conventions or add an incompatible ISR attribute. Application interrupt callbacks still use `_isr`. Record local port modifications and dependency provenance. |
| CEX-03 | 5.4.b.i | Use the toolchain's `float` and `double` names rather than nonstandard `float32_t`/`float64_t` aliases. | Preserve existing controller precision and finite/range checks; record actual target precision. No reduction of precision for style alone. |
| CEX-04 | 5.6.a | Existing native one-bit semantic flags may use unsigned fields rather than ordinary `bool` objects, preserving Tribunal compact-domain requirements and shared record footprints. Wire flags remain explicit unsigned bits. | Use only 0/1 Boolean semantics, protect packed read-modify-write state, never transmit native layout. Signed bit-fields are not exempt. |
| CEX-05 | 4.2.a, 4.3.b–c | Feature-level umbrella headers and pure header-only algorithms remain permitted. This supports host-testable logic without duplicating definitions. | Each source includes its owning feature contract. Header helpers are static/inline; no externally linked storage definitions in public headers. Public headers must not expose private mutable state. |
| CEX-06 | 4.1.b | Module identity is the repository-relative path, not uniqueness in the first eight basename characters. Adapter and shared `imu_service` modules have separate paths. | Clear include guards and qualified includes; no ambiguous include search order. New names must remain descriptive. |
| CEX-07 | 6.3, 1.6.a | Register-address/lvalue macros and explicit discard casts `(void)` are allowed. Void-discard has no numeric range conversion; MMIO access cannot be replaced by a value-returning helper. | Parenthesize macro operands, avoid repeated evaluation/side-effect arguments, document MMIO addresses and ownership in the adapter. Numeric narrowing casts still require range reasoning. |

## Exclusions and recommendations

Recorded evidence, archived checkpoints and already published firmware bundles
remain historical records. They are not rewritten to appear compliant.

Barr's one-exit preference, avoidance of `continue` and printed-page placement
are recommendations. Early returns are acceptable for error handling with correct
cleanup. Source formatting is checked independently from semantic safety.

The 80-column policy allows unavoidable include paths/URLs and indivisible
tokens. It does not excuse ordinary long statements or unwrapped diagnostic
strings. Do not insert newlines into a transmitted string merely to satisfy the
source line limit.

## Numeric and synchronization review

Unsigned wrapping counters, checksums and modulo timestamps intentionally use
unsigned arithmetic. Record wrapping/saturation behavior at their contracts.
Fixed-width narrow integers are subject to C integer promotions: choosing an
unsigned typedef alone does not prove every intermediate is unsigned. Check
shift widths and promoted arithmetic when touching packet expressions.

Existing cross-core lock/atomic contracts take precedence over adding `volatile`
to ordinary synchronized objects. Keep volatile on MMIO and interrupt-visible
state where required. Never remove synchronization to obtain a cosmetic rule
pass.
