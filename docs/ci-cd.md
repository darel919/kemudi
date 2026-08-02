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


## Releases

Release artifacts should come from a clean, verified build. Do not publish credentials, environment files, or unverified generated output. Keep the deployment identifier and build checks associated with each release so a previous verified deployment can be restored if necessary.
