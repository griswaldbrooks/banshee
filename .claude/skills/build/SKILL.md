---
name: build
description: Compile the Banshee Arduino sketch for both supported boards.
user_invocable: true
---

# Build

Compile `controller-firmware/` for both target boards. Mirrors what CI does.

```bash
arduino-cli core install arduino:avr           # one-time
arduino-cli compile --fqbn arduino:avr:uno      controller-firmware
arduino-cli compile --fqbn arduino:avr:leonardo controller-firmware
```

Use `--warnings all` to match CI strictness.
