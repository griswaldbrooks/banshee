---
name: docs
description: Build, serve, or hot-reload the Docusaurus site under website/.
user_invocable: true
---

# Docs

```bash
cd website
npm install         # one-time / after dependency changes
npm run start       # localhost:3000 with hot reload
npm run build       # production build (catches broken links / bad MDX)
npm run serve       # serve the production build locally
```

CI builds and deploys on push to `main`; see `.github/workflows/deploy-docs.yml`.
