---
name: lint
description: Check sketch formatting with clang-format (dry run).
user_invocable: true
---

# Lint

```bash
clang-format --dry-run -Werror controller-firmware/controller-firmware.ino
```

To apply fixes:

```bash
clang-format -i controller-firmware/controller-firmware.ino
```

Style is defined by `.clang-format` at the repo root and enforced by the `lint` job in CI.
