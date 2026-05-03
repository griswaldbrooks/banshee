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

A repo-level `.clang-format` is tracked in [#18](https://github.com/griswaldbrooks/banshee/issues/18).
