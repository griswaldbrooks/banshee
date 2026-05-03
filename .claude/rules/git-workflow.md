# Git workflow

## Branches

- Default branch is `main`.
- Feature branches: `<type>/<short-slug>`, e.g. `firmware/failsafe-watchdog`, `docs/season-2024`.
- Don't commit straight to `main` for non-trivial changes — open a PR.

## Commits

- Imperative subject ≤ 72 chars; "what changed and why" in the body if non-obvious.
- One concern per commit when practical.
- Don't `--amend` published commits; create a new commit instead.

## Pull requests

- One issue per PR when possible; cross-link the issue in the description.
- PR title mirrors the issue title; PR body is "Summary" + "Test plan".
- CI must be green before merging.

## Releases

- Tag with `vYYYY.MAJOR.MINOR` after each event season (e.g. `v2026.0.0`).
- GH Release notes summarize season changes; link the season log page.
