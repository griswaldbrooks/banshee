# Firmware style

Conventions for `controller-firmware/` until `.clang-format` lands (#18).

## Naming

- Macros / `#define` constants: `SCREAMING_SNAKE_CASE` (e.g. `NEUTRAL_CMD`, `MAX_DUTY`).
- Functions / methods: `camelCase` (e.g. `rc2Twist`, `sendCommand`). ISRs: `ALL_CAPS` to make them visually distinct (`ANG_RATE_ISR`).
- Class members: trailing underscore (e.g. `left_`, `is_ang_rate_ready_`).
- Local variables: prefix with type hint where it aids readability (`uiARCmd`, `ulARStartTime`); not required for new code.

## Structure

- Single sketch, no headers — keep classes inline in the `.ino`. If the file outgrows ~500 lines, split.
- Pin assignments via `#define` at the top of the file. Match `website/docs/electronics.md` exactly.

## Hardware patterns

- Mark all variables shared with ISRs `volatile`. Read into a local copy in `loop()` before doing anything with them.
- Clamp before driving hardware: `min(max_pw, max(min_pw, val))`.
- Avoid `new`/`delete`. Use globals or stack objects.
- No floating-point or `Serial` calls inside ISRs.
- All driving paths must respect a failsafe timeout (#21) once it lands.

## Comments

- Doxygen-style `@brief` / `@param` / `@return` blocks on public methods if behavior isn't obvious.
- No "what" comments restating the code. Only "why" comments where the reason isn't visible (workarounds, hardware quirks, RC convention references).
