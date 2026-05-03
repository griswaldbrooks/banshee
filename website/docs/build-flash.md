# Build & Flash

The firmware is an Arduino sketch in `controller-firmware/`. CI builds it for both `arduino:avr:uno` and `arduino:avr:leonardo` on every push.

## Prerequisites

- [arduino-cli](https://arduino.github.io/arduino-cli/)
- AVR core: `arduino-cli core install arduino:avr`

## Compile

```bash
arduino-cli compile --fqbn arduino:avr:uno controller-firmware
# or
arduino-cli compile --fqbn arduino:avr:leonardo controller-firmware
```

## Flash

Plug the board in, find its port, then:

```bash
arduino-cli board list
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno controller-firmware
```

## CI

GitHub Actions builds the sketch for both supported boards. See [`.github/workflows/ci.yml`](https://github.com/griswaldbrooks/banshee/blob/main/.github/workflows/ci.yml).
