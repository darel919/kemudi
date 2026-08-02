# CI/CD and releases

Kemudi.js is built from the repository root with Bun. The web application is deployed from `app/kemudi.js`; Rust/WASM output used by the browser is stored in `app/kemudi.js/public/pkg/`.

## Local verification

```bash
bun install --frozen-lockfile
bun run test
bun run typecheck
bun run lint
bun run build
```

`bun run build` regenerates the Rust/WASM package and then creates the Nuxt production build. The production output can be checked locally with `bun run preview`.

## Vercel project

Configure the Vercel project with these values:

| Setting | Value |
| --- | --- |
| Root Directory | `app/kemudi.js` |
| Framework | Nuxt |
| Build Command | `bun run build` |
| Install Command | `bun install --frozen-lockfile` |

The repository contains `app/kemudi.js/vercel.json`. Its `ignoreCommand` runs from the Vercel Root Directory and compares the previous and current commits in `.`. A commit is skipped when it contains no changes under `app/kemudi.js`.

As a result, changes only to root documentation, `crates/kemudi-engine`, `spec/`, `fixtures/`, `packages/`, or `ports/kemudi-blox/` do not start a Vercel deployment. When a Rust change is intended for the web application, regenerate the WASM package and include the changed files under `app/kemudi.js/public/pkg/`.

## Releases

Release artifacts should come from a clean, verified build. Do not publish credentials, environment files, or unverified generated output. Keep the deployment identifier and build checks associated with each release so a previous verified deployment can be restored if necessary.
