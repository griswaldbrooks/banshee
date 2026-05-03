# Testing

## BDD style

All tests in this repo — unit, integration, and HIL — are written **BDD-style**: explicit `Given` (preconditions / setup), `When` (action under test), `Then` (expected result). HIL scenario files (#34) make the structure literal with `[given]` / `[when]` / `[then]` TOML blocks. Rust unit tests use comments or block layout to mirror the same structure.

Why: makes intent obvious from the test alone, no decoder ring needed. Failure messages can read back the BDD sections so the diagnosis is "the *Then* clause failed", not just "assertion failed at line 42".

## Tolerances are configurable, never hard-coded

Hardware behaviour drifts. A tolerance that's right today (e.g. PWM ±5 / 255) will be too tight or too loose tomorrow. Bake configurability in from the first commit:

1. **Runner default** — built-in starting points.
2. **Scenario default** — `[given.tolerances]` block overrides the whole scenario.
3. **Per-expectation override** — inline `tolerance = …` on a single `[[then]]`.

Tightest scope wins. Never burn a tolerance into the assertion engine.

## HIL: two-layer measurement

Every assertion is evaluated against **both** layers:

- **Firmware contract** — motor PWM duty + direction the firmware commanded.
- **Mechanical effect** — wheel encoder direction + rate observed on the rig.

Both must pass. The diff between them is the diagnostic gold: PWM right + encoder wrong = downstream hardware fault; PWM wrong + encoder wrong = firmware bug; PWM right + encoder right = green.

Current sensing is deliberately deferred — add it later if a class of failures slips through both layers.

## What HIL is for, and what it isn't

- HIL is for catching firmware regressions on real hardware before a match. It's the safety net under code review + bench feel-test.
- HIL is **not** a replacement for hardware bench testing on the actual bot. The rig holds the bot in a defined harness; field conditions are different.
- Combat firmware changes that affect motor commands or signal handling get a hardware bench check **and** the HIL pass before merging. See `git-workflow.md`.

## Naming

- Scenario files: kebab-case, describe the behaviour (`neutral-stops-motors.toml`, `signal-loss-stops-within-250ms.toml`), not the implementation.
- Rust tests: `should_<behaviour>` or `<subject>_<behaviour>`.
