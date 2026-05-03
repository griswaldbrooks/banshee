[![CI](https://github.com/griswaldbrooks/banshee/actions/workflows/ci.yml/badge.svg)](https://github.com/griswaldbrooks/banshee/actions/workflows/ci.yml)
[![Deploy Docs](https://github.com/griswaldbrooks/banshee/actions/workflows/deploy-docs.yml/badge.svg)](https://github.com/griswaldbrooks/banshee/actions/workflows/deploy-docs.yml)

# Banshee

Project hub for **Banshee**, a combat robot fielded annually at [Weaponized Plastic](https://weaponizedplastic.com/). This repo holds the firmware, electronics notes, mechanical/CAD references, season logs, and links.

- **Docs site:** https://griswaldbrooks.com/banshee/
- **CAD:** see [`website/docs/onshape.md`](website/docs/onshape.md)
- **Issues / roadmap:** [open issues](https://github.com/griswaldbrooks/banshee/issues)

## Firmware

Arduino sketch in [`controller-firmware/`](controller-firmware/). Builds for `arduino:avr:uno` and `arduino:avr:leonardo`.

```bash
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno controller-firmware
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 controller-firmware
```

CI builds both boards on every push — see [`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## Docs site

Docusaurus site in [`website/`](website/). Local preview:

```bash
cd website
npm install
npm run start
```

Deployed to GitHub Pages on push to `main` via [`.github/workflows/deploy-docs.yml`](.github/workflows/deploy-docs.yml).
