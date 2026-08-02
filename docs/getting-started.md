# Getting started

Kemudi.js is currently in early development. This guide covers the repository workflow and the production-oriented expectations for adding a feature.

The repository is a monorepo. The web application lives in `app/kemudi.js`, the Rust engine lives in `crates/kemudi-engine`, and the planned Roblox port lives in `ports/kemudi-blox`.

## Prerequisites

Install:

- Bun for JavaScript dependencies and scripts.
- Node.js 20+ for Nuxt and server tooling.
- Rust with the `wasm32-unknown-unknown` target.
- `wasm-pack` for the physics package.
- A WebGL 2 and WebAssembly-capable browser.

Verify the toolchain before making changes:

```bash
bun --version
node --version
rustc --version
rustup target list --installed
wasm-pack --version
```

## Install and run

```bash
bun install
bun run dev
```

Use the URL printed by Nuxt. The browser console should remain free of unexpected errors. Development diagnostics use scoped `console.debug` events; production diagnostics are disabled by default.

## Build like production

```bash
bun run build
bun run preview
```

The root scripts coordinate the web and Rust boundaries. `bun run build` builds the Rust/WASM artifact first and then builds the Nuxt application.

Always test important browser behavior against the built bundle as well as the development server. Differences in bundling, asset paths, feature detection, and runtime configuration can hide production-only failures.

## Development workflow

1. Read the relevant architecture guide.
2. Locate the existing feature and neighboring tests.
3. Define the acceptance criteria and failure behavior.
4. Write or update tests before implementation where practical.
5. Keep physics, rendering, networking, and content loading behind their existing boundaries.
6. Run focused checks, then the applicable full checks.
7. Verify important browser behavior using the built artifact.
8. Update public documentation when a contract, setting, schema, or command changes.

## Common checks

The root workspace exposes the currently available checks through `package.json`:

```bash
bun run lint
bun run typecheck
bun run test
bun run test:rust
bun run build
```

Integration and E2E commands are not currently exposed by the root workspace scripts.
