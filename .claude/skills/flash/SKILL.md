---
name: flash
description: Upload the compiled sketch to a connected Arduino board.
user_invocable: true
---

# Flash

```bash
arduino-cli board list                                                       # find the port
arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM0 controller-firmware
```

Substitute `arduino:avr:leonardo` for Leonardo. Port is typically `/dev/ttyACM0` or `/dev/ttyUSB0` on Linux.
