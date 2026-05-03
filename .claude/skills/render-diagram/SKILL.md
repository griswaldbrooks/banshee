---
name: render-diagram
description: Render a graphviz DOT file to PNG in website/static/img/.
user_invocable: true
---

# Render Diagram

```bash
dot -Tpng -o website/static/img/<output_name>.png <input_file>.dot
```

DOT sources live alongside the rendered PNGs in `website/static/img/`. Use catppuccin-mocha colors for theme consistency.
