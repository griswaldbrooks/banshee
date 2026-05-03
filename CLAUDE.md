# Banshee

Project hub for **Banshee**, a combat robot fielded annually at [Weaponized Plastic](https://weaponizedplastic.com/). This repo holds firmware, electronics notes, mechanical/CAD references, season logs, and the docs site.

## Repo layout

| Path | Purpose |
|---|---|
| `controller-firmware/` | Arduino sketch (single .ino). PWM-input RC → differential drive. |
| `website/` | Docusaurus site, deployed to `griswaldbrooks.com/banshee/`. |
| `.github/workflows/` | `ci.yml` builds the sketch; `deploy-docs.yml` publishes the site. |
| `.claude/` | Skills, rules, agents, and Bash allowlist for Claude sessions. |

## Firmware quick reference

Arduino Uno or Leonardo, AVR core. Single sketch.

| Function | Pin | Notes |
|---|---|---|
| Angular rate (RC ch) | D0 | PWM 1000–2000 µs, neutral 1500 |
| Linear rate (RC ch) | D1 | PWM 1000–2000 µs, neutral 1500 |
| Motor 1 PWM / direction | D6 / D7 | |
| Motor 2 PWM / direction | D5 / D4 | |
| Status LED | D13 | On after `setup()` |

Constants: `MAX_DUTY=255`, `DEADBAND_DELTA=100`, `NEUTRAL_CMD=1500`.

Drive math: ISRs measure RC pulse width → `rc2vel()` maps to [-1, 1] with quadratic shaping → `sendCommand()` does `v_left = vel - ang`, `v_right = vel + ang` → PWM + direction pin.

## Commands

```bash
# Build
arduino-cli compile --fqbn arduino:avr:uno      controller-firmware
arduino-cli compile --fqbn arduino:avr:leonardo controller-firmware

# Flash
arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM0 controller-firmware

# Docs
cd website && npm install && npm run start    # local preview at :3000
cd website && npm run build                   # production build
```

## Workflow notes

- Default branch is `main`. (Renamed from `master`; remote `master` may still exist as a courtesy redirect.)
- Issues live at https://github.com/griswaldbrooks/banshee/issues. Labels: `ci`, `docs`, `firmware`, `electronics`, `mechanical`, `infra`, `claude`, `season-log` plus the GitHub defaults.
- This is a personal repo, low cadence — no PR template, no CONTRIBUTING.md.

## Conventions

- **Tooling lives behind pixi.** New toolchains (Rust, embassy, probe-rs, anything else) get pinned in `pyproject.toml` and exposed as `pixi run <task>` rather than ad-hoc installs. CI uses the same pixi tasks the dev box does. (#27, #28)
- **Tooling is Rust by default.** HIL runner, tester firmware, and any future on-host helpers are Rust. Python is acceptable for one-off scripts but not load-bearing tooling.
- **Tests are BDD-style.** `Given` / `When` / `Then`, with HIL scenario TOMLs making the structure explicit. See `.claude/rules/testing.md`.
- **Tolerances are configurable.** Never hard-code a tolerance threshold inside an assertion engine; expose runner default → scenario override → per-expectation override. (#35)
- **Single robot per repo.** Banshee lives here; sibling robots get their own repos following the same hub-modernization pattern.

## Pointers

- `.claude/skills/` — invocable workflow shortcuts (`build`, `flash`, `lint`, `render-diagram`, `wrapup`).
- `.claude/rules/` — git-workflow, firmware-style, and testing conventions.
- `.claude/agents/` — `code-reviewer`.
- Sister repo with the same patterns: `~/personal/cat-detector`.
