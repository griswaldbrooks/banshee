# Electronics

:::info Stub
Tracked in [#11](https://github.com/griswaldbrooks/banshee/issues/11). Will be filled with: full pinout, RC receiver model, ESC/H-bridge, battery + power, graphviz block diagram.
:::

## Current pinout (from firmware)

| Function | Pin | Notes |
|---|---|---|
| Angular rate (RC ch) | D0 | Pulse width 1000–2000 µs, neutral 1500 |
| Linear rate (RC ch) | D1 | Pulse width 1000–2000 µs, neutral 1500 |
| Motor 1 PWM | D6 | |
| Motor 1 direction | D7 | |
| Motor 2 PWM | D5 | |
| Motor 2 direction | D4 | |
| Status LED | D13 | On = setup complete |
