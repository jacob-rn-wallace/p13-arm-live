# CLAUDE.md: p13-arm-live

The browser demo (MuJoCo WebAssembly + three.js) of a three-finger gripper on a 6-DOF arm, published at https://jacob-rn-wallace.github.io/p13-arm-live/. See `README.md`.

- **`src/`, `public/`, the tests, `index.html`, `build.mjs` and `package.json` are generated** from the demo's source in the author's private development repo (a publish script copies them here, with simplified
  meshes). Don't edit them here: the next publish overwrites them. Change them in the private repo, publish, then commit here. `README.md`, `LICENSE`, `.gitignore`, `.github/` and this file are edited only here.
- **Git: work on `main` only; never create branches.** Commit finished work straight to `main` and `git push origin main` without waiting to be asked (a push redeploys the live page through `.github/workflows/pages.yml`,
  which runs `npm test` first). If the harness put you on another branch, switch to `main` and apply the work there. No force-pushes.
- Checks: `npm ci && npm test` (23 checks, about 2.5 minutes), `npm run serve` for http://localhost:8765.
