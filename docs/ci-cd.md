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

The Vercel project uses the repository root so it can read the workspace lockfile and root configuration:

| Setting | Value |
| --- | --- |
| Root Directory | repository root (`.`) |
| Framework | Nuxt |
| Build Command | `bun run generate` |
| Install Command | `bun install --frozen-lockfile` |

The repository contains `vercel.json`. Its `ignoreCommand` compares the previous and current commits and passes only `app/kemudi.js` to `git diff`. A commit is skipped when it contains no changes under that directory. The app-level `app/kemudi.js/vercel.json` provides the equivalent settings if a separate Vercel project is created with `app/kemudi.js` as its Root Directory.

Vercel runs `bun run generate` because the Nuxt application has `ssr: false`. The generated static output is `app/kemudi.js/.output/public`. The build uses the checked-in WASM package under `app/kemudi.js/public/pkg/` and does not run `wasm-pack`; Rust/WASM compilation remains part of the local `bun run build` command and any CI job that regenerates the checked-in package.

As a result, changes only to root documentation, `crates/kemudi-engine`, `spec/`, `fixtures/`, `packages/`, or `ports/kemudi-blox/` do not start a Vercel deployment. When a Rust change is intended for the web application, regenerate the WASM package and include the changed files under `app/kemudi.js/public/pkg/`.

## Releases

Release artifacts should come from a clean, verified build. Do not publish credentials, environment files, or unverified generated output. Keep the deployment identifier and build checks associated with each release so a previous verified deployment can be restored if necessary.
